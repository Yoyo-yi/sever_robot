#include <iostream>
#include <string>
#include <time.h>
#include <chrono>
#include <csignal>
#include <unistd.h>

#include <thread>
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>
#include <sstream>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "control.hpp"
#include "update.hpp"

using json = nlohmann::json;

#define BOT_ADDR_IP "http://192.168.110.53:1448"    //BOT的IP地址

#define GET_POWER_INFO      BOT_ADDR_IP "/api/core/system/v1/power/status"           //电量获取
#define GET_MYLOCATION      BOT_ADDR_IP "/api/core/slam/v1/localization/pose"        //当前位置
#define GET_ACTION_STATE    BOT_ADDR_IP "/api/core/motion/v1/actions/:current"       //当前运动状态

#define POST_MOVE           BOT_ADDR_IP "/api/core/motion/v1/actions"           //移动
#define POST_RETURN_POLE    BOT_ADDR_IP "/api/core/motion/v1/actions"           //回桩

//路径点存储绝对路径
// #define POINT_FILE_PATH    "//home//orangepi//swy//patrol//config//config.json"
#define POINT_FILE_PATH    "//home//orangepi//python//gui//saved_data.json"



#define HOUR2MIN(HOUR,MIN) HOUR*60+MIN

#define PATROL_STAR_HOUR   13
#define PATROL_STAR_MIN    25
#define PATROL_END_HOUR    22
#define PATROL_END_MIN     10
#define PATROL_HZ          5 

int server_PORT = 9999;
std::string server_addr = "127.0.0.1";
//std::string server_addr = "192.168.110.149";
std::string client_addr;
char qt_socket_data[1000];
std::tm now_tm;

std::mutex mtx;
bool control_flag = 0;

struct power_status {
    int batteryPercentage;  //电池百分比
    int dockingStatus;      //对桩状态
    bool isCharging;        //是否正在充电
    bool isDCConnected;     //电源状态
};
power_status bot_power_status;

struct location_status {
    float x;
    float y;
    float z;
    float yaw;      //头朝向   
};
location_status bot_location_status;

// std::vector<location_status> patrol_points;
//预设路径
// std::vector<location_status> patrol_points = {
// {0.7, 3.0, 0, 0},
// {3.0, -1.3, 0, 0},
// {1.4, -2.0, 0, 0}
// };

struct user_setting { 
    int control;
    int automode;
    int low_battery;
    int start_time[2];
    int end_time[2];
    int time_flag;
    int patrol_frequency;
    std::vector<location_status> patrol_points;
};

user_setting UserSet_time;

// struct set_time {
//     int start_date;
//     int start_time;
//     int start_minute;
//     int end_date;
//     int end_time;
//     int end_minute;
//     int patrol_frequency;
// };
// set_time UserSet_time;


std::atomic<bool> isControl{true};
std::atomic<bool> isAutoMode_Time{false};
std::atomic<bool> isAutoMode_Power{false};


enum class HTTP_OD{
    BOT_POST = 0,
    BOT_GET,
    BOT_PUT,
    BOT_DELET
};

bool is_within_time_range(int in_hour, int in_min, int start_hour, int start_min, int end_hour, int end_min) {
    if (HOUR2MIN(start_hour, start_min) <= HOUR2MIN(end_hour, end_min)) {
        return (HOUR2MIN(in_hour,in_min) >= HOUR2MIN(start_hour, start_min)) && (HOUR2MIN(in_hour, in_min) < HOUR2MIN(end_hour, end_min));
    }
    else{
        return (HOUR2MIN(in_hour, in_min) >= HOUR2MIN(start_hour, start_min)) || (HOUR2MIN(in_hour, in_min) < HOUR2MIN(end_hour, end_min));
    }
}

void sleep_until(const std::tm& future_time) {
    auto future = std::chrono::system_clock::from_time_t(std::mktime(const_cast<std::tm*>(&future_time)));
    std::this_thread::sleep_until(future);
}

std::tm get_next_run_time(std::tm current_time, int interval_minutes) {
    current_time.tm_min += interval_minutes;
    current_time.tm_sec = 0;
    std::mktime(&current_time); // Normalize the time structure
    return current_time;
}



//读取坐标点
std::vector<location_status> get_point_from_file(const std::string& filename) {
    std::vector<location_status> locations;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return locations;
    }

    std::string one_line;
    while (std::getline(file, one_line)) {
        //消除一行中的"( )"，得到一个只有坐标系信息的字符串
        one_line.erase(std::remove(one_line.begin(), one_line.end(), '('), one_line.end());
        one_line.erase(std::remove(one_line.begin(), one_line.end(), ')'), one_line.end());

        //将坐标的字符串转换为流，方便取值>>操作
        std::stringstream point_stream(one_line);
        location_status loc;
        char comma;
        if (point_stream >> loc.x >> comma >> loc.y >> comma >> loc.z >> comma >> loc.yaw) {
            locations.push_back(loc);
        }
        else {
            std::cerr << "Error parsing line: " << one_line << std::endl;
        }
    }

    file.close();
    for (const auto& loc : locations) {
        std::cout << "x: " << loc.x << ", y: " << loc.y << ", z: " << loc.z << ", yaw: " << loc.yaw << std::endl;
    }

    return locations;
}


void updateUserSettingFromJson(const std::string& filename, user_setting& settings) {
    // 读取 JSON 文件
    std::ifstream json_file(filename);
    if (!json_file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    nlohmann::json json_data;
    json_file >> json_data;

    // 更新数据到 user_setting
    settings.control = json_data["control"];
    settings.automode = json_data["automode"];
    settings.low_battery = json_data["low_battery"];
    settings.start_time[0] = json_data["start_time_h"];
    settings.start_time[1] = json_data["start_time_m"];
    settings.end_time[0] = json_data["end_time_h"];
    settings.end_time[1] = json_data["end_time_m"];
    settings.time_flag = json_data["time_flag"];
    settings.patrol_frequency = json_data["patrol_frequency"];

    // 解析 "point" 字段为多个 location_status
    std::string points_str = json_data["point"];
    std::istringstream points_stream(points_str);
    std::string point;
    while (std::getline(points_stream, point, ' ')) {
        if (point.empty()) continue;

        // 解析单个点 (x, y, z)
        location_status loc;
        sscanf(point.c_str(), "(%f,%f,%f)", &loc.x, &loc.y, &loc.z);
        loc.yaw = 0.0f; // 默认 yaw 为 0
        settings.patrol_points.push_back(loc);
    }

    std::cout << "Settings updated from JSON file successfully!" << std::endl;
}




//static size_t PostCallback(void* contents, size_t size, size_t nmemb, std::string* response) 
//{
//    response->append((char*)contents, size * nmemb);
//    return size * nmemb;
//}

static size_t PostCallback(void* buffer, size_t size, size_t count, void* response)
{
    std::string* str = (std::string*)response;
    (*str).append((char*)buffer, size * count);

    return size * count;
}

int sendHttpPostRequest(const std::string& url, const std::string& messageData) 
{
    int POST_END = 1;
    std::string response;
    CURL* curl = curl_easy_init();
    if (curl) {
        CURLcode res;
        struct curl_slist* headers = NULL;

        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, PostCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)&response);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, false);//设定为不验证证书和HOST 
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, false);

        // 设置要发送的HTTP请求的URL
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

        // 设置HTTP请求头  (Content-Type类型请求头，用于指定发送的实体数据类型)
        headers = curl_slist_append(headers, "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:76.0)");
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // 设置要发送的数据
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, messageData.c_str());

        // 执行HTTP POST请求
        res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            //test
            // ROS_ERROR("curl_easy_perform() failed: %s", curl_easy_strerror(res));
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
            return 0;
        }
        // 清理资源
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        return 1;
    }
    else {
        return 0;
    }
}

//static size_t GetCallback(void* contents, size_t size, size_t nmemb, void* userp) 
//{
//    std::string* response = static_cast<std::string*>(userp);
//    response->append(static_cast<char*>(contents), size * nmemb);
//    return size * nmemb;
//}

static size_t GetCallback(void* buffer, size_t size, size_t count, void* response)
{
    std::string* str = (std::string*)response;
    (*str).append((char*)buffer, size * count);

    return size * count;
}

std::string sendHttpGetRequest(const std::string& url)
{
    std::string get_buffer;
    long retcode = 0;
    // 初始化请求库
    CURL* easy_handle = curl_easy_init();
    if (NULL != easy_handle)
    {
        CURLcode res;
        // 初始化填充请求头
        struct curl_slist* headers = NULL;
        //设备信息请求头 （在服务端严格的话，可以选择加入请求头链表）
        //headers = curl_slist_append(headers, "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:76.0)");
        //可以在请求头中加入请求来源
        //headers = curl_slist_append(headers, "Referer: https://www.lyshark.com");
        headers = curl_slist_append(headers, "Content-Type: application/json");
        // CURLOPT_HTTPHEADER 自定义设置请求头
        curl_easy_setopt(easy_handle, CURLOPT_HTTPHEADER, headers);

        // CURLOPT_URL 自定义请求的网站
        curl_easy_setopt(easy_handle, CURLOPT_URL, url.c_str());

        // CURLOPT_WRITEFUNCTION 设置回调函数,屏蔽输出
        curl_easy_setopt(easy_handle, CURLOPT_WRITEFUNCTION, GetCallback);
        curl_easy_setopt(easy_handle, CURLOPT_WRITEDATA, (void*)&get_buffer);

        // 执行CURL访问网站
        res = curl_easy_perform(easy_handle);
        if (res != CURLE_OK){
            curl_slist_free_all(headers);
            curl_easy_cleanup(easy_handle);
            return "err";
        }

        //char* ipAddress = { 0 };
        //// CURLINFO_PRIMARY_IP 获取目标IP信息
        //res = curl_easy_getinfo(easy_handle, CURLINFO_PRIMARY_IP, &ipAddress);
        //if ((CURLE_OK == res) && ipAddress)
        //{
        //    std::cout << "目标IP: " << ipAddress << std::endl;
        //}

        
        // CURLINFO_RESPONSE_CODE 获取目标返回状态
        res = curl_easy_getinfo(easy_handle, CURLINFO_RESPONSE_CODE, &retcode);
        if ((CURLE_OK == res) && retcode)
        {
            //std::cout << "返回状态码: " << retcode << std::endl;
        }

        curl_slist_free_all(headers);
        curl_easy_cleanup(easy_handle);
    }
    if (retcode == 200)
        return get_buffer;
    else
        return std::to_string(retcode);
    
}

static size_t DeleteCallback(void* buffer, size_t size, size_t count, void* response)
{
    std::string* str = (std::string*)response;
    (*str).append((char*)buffer, size * count);

    return size * count;
}

std::string sendHttpDeleteRequest(const std::string& url){

    std::string get_buffer;
    long retcode = 0;
    // 初始化请求库
    CURL* easy_handle = curl_easy_init();
    if (NULL != easy_handle)
    {
        CURLcode res;

        // CURLOPT_URL 自定义请求的网站
        curl_easy_setopt(easy_handle, CURLOPT_URL, url.c_str());

        curl_easy_setopt(easy_handle, CURLOPT_CUSTOMREQUEST, "DELETE");

        // CURLOPT_WRITEFUNCTION 设置回调函数,屏蔽输出
        curl_easy_setopt(easy_handle, CURLOPT_WRITEFUNCTION, DeleteCallback);
        curl_easy_setopt(easy_handle, CURLOPT_WRITEDATA, (void*)&get_buffer);

        // 执行CURL访问网站
        res = curl_easy_perform(easy_handle);
        if (res != CURLE_OK){
            curl_easy_cleanup(easy_handle);
            return "err";
        }

        
        // CURLINFO_RESPONSE_CODE 获取目标返回状态
        res = curl_easy_getinfo(easy_handle, CURLINFO_RESPONSE_CODE, &retcode);
        if ((CURLE_OK == res) && retcode)
        {
            //std::cout << "返回状态码: " << retcode << std::endl;
        }


        curl_easy_cleanup(easy_handle);
    }
    if (retcode == 200)
        return get_buffer;
    else
    return std::to_string(retcode);
}


//CURL全局初始化模块
void InitGlobalCurl(int current_OS)
{
    if (CURLE_OK != curl_global_init(current_OS))
    {
        std::cout << "CURL_HTTP init fail！！！！！！" << std::endl;
    }
}


int socketServer(int server_port, char* socket_data_ptr, int SEND_OR_REV) {

    int sockfd;
    struct sockaddr_in socket_ServerAddr;
    struct sockaddr_in socket_Client;

    std::cout << "<--------into socket--------->" << std::endl;
    /****创建socket连接(TCP:SOCK_STREAM  UDP:SOCK_DGRAM )*****/
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        std::cerr << "Socket creation failed" << std::endl;
        return -1;
    }

    int opt = 1;
    if(setsockopt(sockfd,SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1){
        std::cerr << " Socket set reuseaddr err " << std::endl;
        close(sockfd);
        return -1;
    }

    memset(&socket_ServerAddr, 0, sizeof(socket_ServerAddr));
    socket_ServerAddr.sin_family = AF_INET;
    socket_ServerAddr.sin_port = htons(server_port);

    if (inet_pton(AF_INET, server_addr.c_str(), &socket_ServerAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        close(sockfd);
        return -1;
    }
    /***bind本机地址***/
    if (bind(sockfd, (struct sockaddr*)&socket_ServerAddr, sizeof(socket_ServerAddr)) < 0) {
        std::cerr << "bind failed:" << errno << std::endl;
        close(sockfd);
        return -1;
    }

    /************TCP************/
    if (listen(sockfd, 1) < 0) {
        std::cerr << "listen failed" << std::endl;
        close(sockfd);
        return -1;
    }
    std::cout << "<--------socket create successfully!!!!!!--------->" << std::endl;

    //客户端申请连接
    //if(connect(sockfd,(struct sockaddr *)&socket_ServerAddr,sizeof(socket_ServerAddr)) == -1){
    //    std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
    //}


    //服务端接收申请连接的客户端的sock
    socklen_t clientAddrLen = sizeof(sockaddr);
    int clientSocket = accept(sockfd, (struct sockaddr*)&socket_Client, &clientAddrLen);
    if (clientSocket == -1) {
        std::cout << "Accept failed:" << std::endl;
        
        close(sockfd);
        return -1;
    }

    std::cout << "<--------Accept successfully--------->" << std::endl;
    //std::cout << "Accept client IP: " << inet_ntoa(socket_Client.sin_addr) << std::endl;

    //while (1)
    //{
    //    recv(sockfd, qt_UserSet_time, sizeof(qt_UserSet_time), 0);
    //}

    if (SEND_OR_REV == 1) {
        int recvResult = recv(clientSocket, socket_data_ptr, 500, 0);
        //test
        //int recvResult = recv(clientSocket, (char*)socket_data.c_str(), 1000, 0);
        if (recvResult > 0)
        {
            std::cout << "<--------recv successfully!--------->" << std::endl;
            //std::cout << "time:" << UserSet_time.start_time[0] <<","<< UserSet_time.start_time[1] << std::endl;
            //std::cout << "time:" << UserSet_time.end_time[0] <<","<< UserSet_time.end_time[1] << std::endl;
        }
        else if (recvResult == 0) {
            std::cerr << "Connection closed" << std::endl;
            close(clientSocket);
            close(sockfd);
            return -1;
        }
        else {
            std::cerr << "recv failed: " << strerror(errno) << std::endl;
            close(clientSocket);
            close(sockfd);
            return -1;

        }
    }
    else if (SEND_OR_REV == 0) {
        int sendResult = send(clientSocket, socket_data_ptr, 1000, 0);
        if (sendResult > 0)
        {
            std::cout << "<--------send successfully!--------->" << std::endl;

        }
        else if (sendResult == 0) {
            std::cerr << "Connection closed" << std::endl;
            close(clientSocket);
            close(sockfd);
            return -1;
        }
        else {
            std::cerr << "send failed: " << strerror(errno) << std::endl;
            close(clientSocket);
            close(sockfd);
            return -1;
        }
    }
    else {
        std::cerr << "socket mode err"<< std::endl;
        close(clientSocket);
        close(sockfd);
        return -1;
    }
    close(clientSocket);
    close(sockfd);
    return 1;

}


int GetUserInitData(user_setting& setting_data) {
    std::cout << "等待初始化配置" << std::endl;
    if (socketServer(9999,qt_socket_data, 1) == 1) {
        std::cout << "参数初始化成功" << std::endl;
    }
    else {
        std::cout << "参数初始化失败" << std::endl;
        return -1;
    }

    //test
    std::cout << "客户端数据:" << qt_socket_data << std::endl;
    
    json qt_socket_key = nlohmann::json::parse(qt_socket_data);
    setting_data.automode = qt_socket_key.at("mode").get<int>();
    setting_data.low_battery = qt_socket_key.at("low_battery").get<int>();
    setting_data.start_time[0] = qt_socket_key.at("start_hour").get<int>();
    setting_data.start_time[1] = qt_socket_key.at("start_minute").get<int>();
    setting_data.time_flag = qt_socket_key.at("time_flag").get<int>();
    setting_data.end_time[0] = qt_socket_key.at("end_hour").get<int>();
    setting_data.end_time[1] = qt_socket_key.at("end_minute").get<int>();
    setting_data.patrol_frequency = qt_socket_key.at("patrol_frequency").get<int>();

    //test
    std::cout << "巡逻模式:" << qt_socket_key.at("mode").get<int>() << std::endl;
    std::cout << "巡逻最低电量:" << qt_socket_key.at("low_battery").get<int>() << std::endl;
    std::cout << "开始时间小时:" << qt_socket_key.at("start_hour").get<int>() << std::endl;
    std::cout << "开始时间分钟:" << qt_socket_key.at("start_minute").get<int>() << std::endl;
    std::cout << "隔天标志位:" << qt_socket_key.at("time_flag").get<int>() << std::endl;
    std::cout << "结束时间小时:" << qt_socket_key.at("end_hour").get<int>() << std::endl;
    std::cout << "结束时间分钟:" << qt_socket_key.at("end_minute").get<int>() << std::endl;
    std::cout << "巡逻频率:" << qt_socket_key.at("patrol_frequency").get<int>() << std::endl;


    // 
    //std::cout << "开始时间小时:" << UserSet_time.start_hour << std::endl;
    //std::cout << "开始时间分钟:" << UserSet_time.start_minute << std::endl;
    //std::cout << "结束时间小时:" << UserSet_time.end_hour << std::endl;
    //std::cout << "结束时间分钟:" << UserSet_time.end_minute << std::endl;
    //std::cout << "巡逻频率:" << UserSet_time.patrol_frequency << std::endl;

    return 1;
}


std::string CreateActionPoint(double x, double y, double yaw)
{
    json temp_data;

    temp_data["action_name"] = "slamtec.agent.actions.MoveToAction";
    temp_data["options"]["target"]["x"] = x;
    temp_data["options"]["target"]["y"] = y;
    temp_data["options"]["target"]["z"] = 0;
    temp_data["options"]["move_options"]["mode"] = 0;
    temp_data["options"]["move_options"]["flags"] = "";
    temp_data["options"]["move_options"]["yaw"] = yaw;
    temp_data["options"]["move_options"]["acceptable_precision"] = 0;
    temp_data["options"]["move_options"]["fail_retry_count"] = 0;
    temp_data["options"]["move_options"]["speed_ratio"] = 0;
    std::string jsonData = temp_data.dump();

    return jsonData;
}

//(0.7, 3.0, 0)  (3.0, -1.3)  (1.4, -2.0, 0)
std::string CreateSeriesActionPoint(const std::vector<location_status>& points)
{
    json temp_data;

    temp_data["action_name"] = "slamtec.agent.actions.SeriesMoveToAction";

    for (const location_status& point : points) {
        temp_data["options"]["targets"].push_back({ {"x",point.x},{"y",point.y}, {"z",point.z} });
    }

    //temp_data["options"]["targets"].push_back({ {"x",0.7},{"y",3.0}, {"z",0} });
    //temp_data["options"]["targets"].push_back({ {"x",3.0},{"y",-1.3},{"z",0} });
    //temp_data["options"]["targets"].push_back({ {"x",1.4},{"y",-2.0},{"z",0} });

    temp_data["options"]["move_options"]["mode"] = 0;
    temp_data["options"]["move_options"]["flags"] = "";
    temp_data["options"]["move_options"]["yaw"] = 0;
    temp_data["options"]["move_options"]["acceptable_precision"] = 0;
    temp_data["options"]["move_options"]["fail_retry_count"] = 0;
    temp_data["options"]["move_options"]["speed_ratio"] = 0;
    std::string jsonData = temp_data.dump();

    return jsonData.c_str();
}

std::string GoHomeAction()
{
    json temp_data;

    temp_data["action_name"] = "slamtec.agent.actions.GoHomeAction";
    temp_data["options"]["gohome_options"]["flags"] = "dock";
    temp_data["options"]["gohome_options"]["back_to_landing"] = "";
    temp_data["options"]["gohome_options"]["charging_retry_count"] = 5;
    temp_data["options"]["gohome_options"]["move_options"]["mode"] = 0;
    std::string jsonData = temp_data.dump();

    return jsonData.c_str();
}

std::string MoveByControl(int direction, int time)
{
	std::unique_lock<std::mutex> lock(mtx);
	// 0：前进  1：后退  2：右转  3：左转
    json temp_data;

    temp_data["action_name"] = "slamtec.agent.actions.MoveByAction";

	if(direction<0 || direction>3) {
		std::cout << "direction err !!" << std::endl;
		return "err";
	}
    temp_data["options"]["direction"] = direction;
    temp_data["options"]["duration"] = time;


    std::string jsonData = temp_data.dump();

    return jsonData.c_str();
}


std::string BotOrder(const std::string& url, HTTP_OD CommandType, const std::string& http_data)
{
    if (CommandType == HTTP_OD::BOT_POST){
        if (http_data.empty()){
            std::cout << "POST data empty" << std::endl;
            return "err";
        }
        if (sendHttpPostRequest(url, http_data) == 0) {
            std::cout << "curl init err" << std::endl;
            return "err";
        }
        return "POST_OK";
    }
    else {
        std::cout << "order err" << std::endl;
        return "err";
    }
}

std::string BotOrder(const std::string& url, HTTP_OD CommandType)
{
    std::string http_data;
    if (!(http_data.empty())) {
        http_data.clear();
    }
    if (CommandType == HTTP_OD::BOT_GET) {
        http_data = sendHttpGetRequest(url);
        if (http_data == "err") {
            std::cout << "GET fail" << std::endl;
            return "err";
        }
        else {
            return http_data;
        }
        return "GET_OK";
    }
    else {
        std::cout << "order err" << std::endl;
        return "err";
    }
}

void BotControl(std::atomic<bool> &runFlag){

    Controller bot_control;
    unsigned char PS2_KEY;
    unsigned char X1,Y1,X2,Y2; 
    std::string direction_ord = "";
    std::cout << "into user control" << std::endl;
    while(runFlag)
    {	   
      PS2_KEY = bot_control.getKey();	 //手柄按键捕获处理
	//   printf("now key :  %s " , PS2_KEY);
	  switch(PS2_KEY)
	  {
	    //select键
	    case PSB_SELECT:
            // puts("PSB_SELECT");  	
            break;
	    //L3键
	    case PSB_L3:
            // puts("PSB_L3");  		
            break; 
	    //R3键		
	    case PSB_R3:
            // puts("PSB_R3");  		
            break; 
	    //start键		
	    case PSB_START:
            // puts("PSB_START");   	
            break;

	    //UP键
	    case PSB_PAD_UP:
            if(control_flag == 1){
                puts("PSB_PAD_UP"); 	
                direction_ord = MoveByControl(0,500);
                BotOrder(POST_MOVE, HTTP_OD::BOT_POST, direction_ord);
            }
			break;	     
	    //RIGHT键		
	    case PSB_PAD_RIGHT:
            if(control_flag == 1){
                puts("PSB_PAD_RIGHT");
                direction_ord = MoveByControl(2,500);
                BotOrder(POST_MOVE, HTTP_OD::BOT_POST, direction_ord);
            }		
			break;
	    //DOWN键按	
	    case PSB_PAD_DOWN:
            if(control_flag == 1){
                puts("PSB_PAD_DOWN");  	
			    direction_ord = MoveByControl(1,500);
			    BotOrder(POST_MOVE, HTTP_OD::BOT_POST, direction_ord);
            }
			break;
	    //LEFT键	
	    case PSB_PAD_LEFT:
            if(control_flag == 1){
                puts("PSB_PAD_LEFT");  	
			    direction_ord = MoveByControl(3,500);
			    BotOrder(POST_MOVE, HTTP_OD::BOT_POST, direction_ord);
            }

			break; 

	    //L2按键
	    case PSB_L2: 
            // puts("PSB_L2");  		
            break; 
	    //R2按键
	    case PSB_R2:
            // puts("PSB_R2");  		
            break; 
	    //L1按键
	    case PSB_L1:
            // puts("PSB_L1");  		
            break; 
	    //R1按键
	    case PSB_R1:
            // puts("PSB_R1");  		
            break; 
	    //三角形按键		
	    case PSB_TRIANGLE:
            if(control_flag == 1){
			    puts("PSB_TRIANGLE");  	
			    direction_ord = GoHomeAction();
			    BotOrder(POST_RETURN_POLE, HTTP_OD::BOT_POST, direction_ord);
            }
			break; 
	    //圆形键
	    case PSB_CIRCLE:
            puts("PSB_CIRCLE");  	
            control_flag = 0;
            break; 	    
	    //方形键
	    case PSB_SQUARE:
            puts("PSB_SQUARE");  	
            break;
	    //X按键
	    case PSB_CROSS:
            puts("PSB_X");		
            control_flag = 1;
            break;
							
	   }
      
	  //当L1或者R1按下时，读取摇杆数据的模拟值
	  if(PS2_KEY == PSB_L1 || PS2_KEY == PSB_R1)
	  {
		X1 = bot_control.readAnalogData(PSS_LX);
		printf("x1的模拟值：%d  ",X1);
		Y1 = bot_control.readAnalogData(PSS_LY);
		printf("y1的模拟值：%d  ",Y1);
		X2 = bot_control.readAnalogData(PSS_RX);
		printf("x2的模拟值：%d  ",X2);
		Y2 = bot_control.readAnalogData(PSS_RY);
		printf("y2的模拟值：%d  ",Y2);
	  }
 
     //下面的延时是必须要的,主要是为了避免过于频繁的发送手柄指令造成的不断重启
     delay(50);	
   }  
}



void ActionStatus(int polling_time,  std::atomic<bool> &Flag)
{
    usleep(500000);
    while (1) {
        std::string action_rx_data = BotOrder(GET_ACTION_STATE, HTTP_OD::BOT_GET);
        if (action_rx_data.compare("404") == 0) {
            std::cout << "Action end !!" << std::endl;
            break;
        }
        else {
            json action_status = nlohmann::json::parse(action_rx_data);
            if (action_status.at("state").at("status").get<int>() == 1)
            {
                usleep(polling_time);
            }
        }
        if(Flag == false){
            sendHttpDeleteRequest(GET_ACTION_STATE);
            break;
        } 
    }

}


int PowerStatusUpdata(power_status& status_data) {

    std::string  power_status_data = "";
    power_status_data = BotOrder(GET_POWER_INFO, HTTP_OD::BOT_GET);

    if (power_status_data.compare("err") == 0) return -1;     //若字符串相等返回0

    json power_status_key = nlohmann::json::parse(power_status_data);

    //电池数据更新
    {
        std::unique_lock<std::mutex> lock(mtx);
        status_data.batteryPercentage = power_status_key.at("batteryPercentage").get<int>();

        std::string is_dock = power_status_key.at("dockingStatus").get<std::string>();
        if (is_dock.compare("on_dock") == 0) {
            status_data.dockingStatus = 1;
        }
        else {
            status_data.dockingStatus = 0;
        }

        status_data.isCharging = power_status_key.at("isCharging").get<bool>();
        status_data.isDCConnected = power_status_key.at("isDCConnected").get<bool>();
    }

    return 1;
}

int LocationUpdata(location_status& location_status_data) {

    std::string  location_data = "";
    location_data = BotOrder(GET_MYLOCATION, HTTP_OD::BOT_GET);

    if (location_data.compare("err") == 0) return -1;     //若字符串相等返回0

    //test
    //std::cout << "当前位置: " << location_data << std::endl;

    json location_key = nlohmann::json::parse(location_data);

    {
        std::unique_lock<std::mutex> lock(mtx);
        location_status_data.x = location_key.at("x").get<float>();
        location_status_data.y = location_key.at("y").get<float>();
        location_status_data.z = location_key.at("z").get<float>();
        location_status_data.yaw = location_key.at("yaw").get<float>();
    }

    //test
    std::cout << "x:" << location_key.at("x").get<float>() << std::endl;
    std::cout << "y:" << location_key.at("y").get<float>() << std::endl;
    std::cout << "yaw:" << location_key.at("yaw").get<float>() << std::endl;


    return 1;
}

int BotUpdata() {

    json qt_user_need_data = {
        {"batteryPercentage", 0},
        {"dockingStatus", 0 },
        {"isCharging", false},
        {"x", 0},
        {"y", 0},
        {"yaw", 0}
    };
    std::string json_str;

    while (1)
    {
        PowerStatusUpdata(bot_power_status);
        //std::cout << "当前电量： " << bot_power_status.batteryPercentage << "%" << std::endl;
        LocationUpdata(bot_location_status);
        //std::cout << "当前位置： " << "( " << bot_location_status.x << " , " << bot_location_status.y << " , " << bot_location_status.yaw << " )" << std::endl;

        std::cout << "当前位置: " << bot_location_status.x << "  " << bot_location_status.y << std::endl;

        qt_user_need_data["batteryPercentage"] = bot_power_status.batteryPercentage;
        qt_user_need_data["dockingStatus"] = bot_power_status.dockingStatus;
        qt_user_need_data["isCharging"] = bot_power_status.isCharging;
        qt_user_need_data["x"] = bot_location_status.x;
        qt_user_need_data["y"] = bot_location_status.y;

        json_str = qt_user_need_data.dump();

        std::cout << json_str.c_str() << std::endl;

        socketServer(9998,(char*)json_str.c_str(), 0);

        usleep(1000000);
    }
    
}


int TimeMode(const user_setting& setting_data, std::atomic<bool> &runFlag) {

    std::string HTTP_RETURN;
    std::cout << "into Time mode" << std::endl;
    while (runFlag) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm;
        localtime_r(&now_time,&now_tm);

        //std::cout << "系统时间：  "<< now_tm.tm_hour << ":"<< now_tm.tm_min << ":" << now_tm.tm_sec << std::endl;
        //Sleep(1000);

        if (is_within_time_range(now_tm.tm_hour, now_tm.tm_min, setting_data.start_time[0], setting_data.start_time[1], setting_data.end_time[0], setting_data.end_time[1])) {
            std::cout << "系统时间：  " << now_tm.tm_hour << ":" << now_tm.tm_min << ":" << now_tm.tm_sec << std::endl;
            std::cout << "< Star Series Point >" << std::endl;
            HTTP_RETURN = BotOrder(POST_MOVE, HTTP_OD::BOT_POST, CreateSeriesActionPoint(UserSet_time.patrol_points));
            ActionStatus(2000,runFlag);
            //Sleep(50000);
            std::cout << "< Go Home >" << std::endl;
            HTTP_RETURN = BotOrder(POST_RETURN_POLE, HTTP_OD::BOT_POST, GoHomeAction().c_str());
            ActionStatus(2000,runFlag);
            //Sleep(35000);

            std::tm next_run_time = get_next_run_time(now_tm, setting_data.patrol_frequency);
            sleep_until(next_run_time);
        }
        else {
            ;
        }

        usleep(5000000);
    }
    std::cout << "out Time mode" << std::endl;
    return 0;
}


void TimeUpdata(std::tm& now_tm) {

    //std::unique_lock<std::mutex> lock(mtx);
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    localtime_r(&now_time,&now_tm);

}

 
int PowerMode(const user_setting& setting_data, std::atomic<bool> &runFlag) {

    std::string HTTP_RETURN;
    int working_flag(1);

    std::cout << "into Power mode" << std::endl;
    while (runFlag) {
        TimeUpdata(now_tm);



        //std::cout << "系统时间：  "<< now_tm.tm_hour << ":"<< now_tm.tm_min << ":" << now_tm.tm_sec << std::endl;
        //Sleep(1000);
        
        if (working_flag == 1 && runFlag == true) {
            if (bot_power_status.batteryPercentage <= setting_data.low_battery) {
                working_flag = 0;
            }
        }
        else if (working_flag == 2 && runFlag == true) {
            if (bot_power_status.batteryPercentage == 100) {
                working_flag = 1;
                std::cout << "< Charge Complete >" << std::endl;
                if (is_within_time_range(now_tm.tm_hour, now_tm.tm_min, setting_data.start_time[0], setting_data.start_time[1], setting_data.end_time[0], setting_data.end_time[1]) == 0) {
                    std::cout << " Wait until set working time " << std::endl;
                }
            }
        }

        if (working_flag == 1 && runFlag == true) {
            std::cout << "Current charge:" << bot_power_status.batteryPercentage << "%" << std::endl;
            if (is_within_time_range(now_tm.tm_hour, now_tm.tm_min, setting_data.start_time[0], setting_data.start_time[1], setting_data.end_time[0], setting_data.end_time[1])) {
                //std::cout << "系统时间：  " << now_tm.tm_hour << ":" << now_tm.tm_min << ":" << now_tm.tm_sec << std::endl;
                //std::cout << "< Star Series Point >" << std::endl;
                HTTP_RETURN = BotOrder(POST_MOVE, HTTP_OD::BOT_POST, CreateSeriesActionPoint(UserSet_time.patrol_points));
                ActionStatus(1000,runFlag);
                //Sleep(50000);
                

                TimeUpdata(now_tm);
                if (is_within_time_range(now_tm.tm_hour, now_tm.tm_min, setting_data.start_time[0], setting_data.start_time[1], setting_data.end_time[0], setting_data.end_time[1]) == 0) {
                    std::cout << "< Go Home on time>" << std::endl;
                    HTTP_RETURN = BotOrder(POST_RETURN_POLE, HTTP_OD::BOT_POST, GoHomeAction().c_str());
                    ActionStatus(1000,runFlag);
                }

            }
            else {
                ;
            }
        }
        else if (working_flag == 0 && runFlag == true) {
            if (is_within_time_range(now_tm.tm_hour, now_tm.tm_min, setting_data.start_time[0], setting_data.start_time[1], setting_data.end_time[0], setting_data.end_time[1])) {
                std::cout << "< Go Home low power>" << std::endl;
                HTTP_RETURN = BotOrder(POST_RETURN_POLE, HTTP_OD::BOT_POST, GoHomeAction().c_str());
                ActionStatus(1000,runFlag);
                working_flag = 2;
            }
            else {
                ;
            }
        }
        else if (working_flag == 2  && runFlag == true) {
            std::cout << " Bot is charging......." << std::endl;
            std::cout << "Current charge:" << bot_power_status.batteryPercentage << "%" << std::endl;
            usleep(10000000);
        }

        //usleep(1000000);
    }
    std::cout << "out power mode" << std::endl;
    return 0;
}


void signal_handler(int signal) 
{
    curl_global_cleanup();
    std::exit(signal);
}

int main(void)
{
    /************测试代码（1）************/
    std::string power_rx_data;
    json bettery_data;

    std::string action_state_rx_data;
    json action_stat;

    time_t timep;
    char buff[40] = { 0 };

    int key_num = 0;

    /************测试代码（1）************/

    
    memset(&UserSet_time, 0, sizeof(UserSet_time));

    // 基于linux平台优化curl
    InitGlobalCurl(CURL_GLOBAL_ALL);

    std::thread bot_updata(BotUpdata);
    bot_updata.detach();

    //用户数据解析
    updateUserSettingFromJson(POINT_FILE_PATH, UserSet_time);
    // GetUserInitData(UserSet_time);
    // UserSet_time.patrol_points = get_point_from_file(POINT_FILE_PATH);

    //注册终止条件，以释放curl产生的全局资源
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string lastHash = calculateFileHash(POINT_FILE_PATH);

    std::unique_ptr<std::thread> control_th;
    std::unique_ptr<std::thread> AutoTime_th;
    std::unique_ptr<std::thread> AutoPower_th;

    control_th = std::make_unique<std::thread>(BotControl, std::ref(isControl));
    control_th->detach();




    
    // std::thread control_th(BotControl,std::ref(isControl));
    // std::thread AutoTime_th(TimeMode,UserSet_time,std::ref(isAutoMode_Time));
    // std::thread AutoPower_th(PowerMode,UserSet_time,std::ref(isAutoMode_Power));
    // control_th.detach();

    std::cout << "<< Star patrol >>" << std::endl;
    while (1) {

        /************************测试代码（1）****************************/
        //std::cout << "是否发送（发送：1  不发送：0）：";
        //std::cin >> key_num;
        //if (key_num == 1) {

        //    //系统时间获取
        //    time(&timep);
        //    ctime_s(buff, sizeof(buff), &timep);
        //    std::cout << "当前时间: " << buff << std::endl;

        //    //power_rx_data = sendHttpGetRequest("http://192.168.110.53:1448/api/core/system/v1/power/status");
        //    //test
        //    //std::cout << "rx_date: " << rx_data << std::endl;

        //    power_rx_data = BotOrder(GET_POWER_INFO, HTTP_OD::BOT_GET);
        //    bettery_data = nlohmann::json::parse(power_rx_data);

        //    action_state_rx_data = BotOrder(GET_ACTION_STATE, HTTP_OD::BOT_GET);
        //    action_stat = nlohmann::json::parse(action_state_rx_data);

        //    //BotOrder(POST_MOVE_TO_POINT,HTTP_OD::BOT_POST,)

        //    try {
        //        
        //        std::cout << "电量：" << bettery_data.at("batteryPercentage").get<int>()
        //                  << "  充电桩状态：" << bettery_data.at("powerStage").get<std::string>()
        //                  << "  睡眠模式：" << bettery_data.at("sleepMode").get<std::string>() << std::endl;

        //        if (action_stat != 404){
        //            std::cout << "当前运动状态：" << action_stat.at("state").at("status").get<int>()
        //                      << "当前运动结果：" << action_stat.at("state").at("result").get<int>() << std::endl;
        //        }
        //        else {
        //            std::cout << "当前运动状态的api不存在，请确保运动api正在运行" << std::endl;
        //        }
        //        //std::cout << "当前时间: " << action_stat << std::endl;
        //    }
        //    catch (nlohmann::json::exception& e) {
        //        // 捕获并打印任何抛出的异常
        //        std::cout << "Exception: " << e.what() << std::endl;
        //    }
        //}
        /************************测试代码（1）****************************/

        /************************测试代码（2）****************************/
        //(0.7, 3.0, 0)  (3.0, -1.3)  (1.4, -2.0, 0)
        //std::cout << "Star 1"<< std::endl;
        //power_rx_data = BotOrder(POST_MOVE_TO_POINT, HTTP_OD::BOT_POST, CreateActionPoint(0.7, 3.0, 0).c_str());
        //Sleep(15000);
        //std::cout << "Star 2" << std::endl;
        //power_rx_data = BotOrder(POST_MOVE_TO_POINT, HTTP_OD::BOT_POST, CreateActionPoint(3.0, -1.3, 0).c_str());
        //Sleep(20000);
        //std::cout << "Star 3" << std::endl;
        //power_rx_data = BotOrder(POST_MOVE_TO_POINT, HTTP_OD::BOT_POST, CreateActionPoint(1.4, -2.0, 0).c_str());
        //Sleep(15000);
        //std::cout << "Go Home" << std::endl;
        //power_rx_data = BotOrder(POST_RETURN_POLE, HTTP_OD::BOT_POST, GoHomeAction().c_str());
        //Sleep(35000);
        /************************测试代码（2）****************************/


        /************************测试代码（3）****************************/

        // std::thread bot_updata(BotUpdata);
        // bot_updata.detach();



        // usleep(1000000);

        // switch (UserSet_time.mode)
        // {
        // case 0:
        //     TimeMode(UserSet_time);
        //     break;
        // case 1:
        //     PowerMode(UserSet_time);
        //     break;
        // default:
        //     break;
        // }

        // while(1){
        //     usleep(5000000);
        // }
        /************************测试代码（3）****************************/



        /************************测试代码（4）****************************/
        std::string currentHash = calculateFileHash(POINT_FILE_PATH);
        if (currentHash != lastHash) {
            std::cout << "Configuration file changed. Reloading..." << std::endl;
            std::ifstream file(POINT_FILE_PATH);
            if (!file) {
                std::cerr << "Failed to open config file: " << POINT_FILE_PATH << std::endl;
                // return;
            }
            json newConfig;
            file >> newConfig;
            // UserSet_time.control = newConfig.at("control").get<int>();
            updateUserSettingFromJson(POINT_FILE_PATH, UserSet_time);
            if(UserSet_time.control == 0){
                if(isControl == false){

                    isControl = true;
                    isAutoMode_Time = false;
                    isAutoMode_Power = false;
                    // control_th.detach();

                    usleep(1000000);
                    control_th = std::make_unique<std::thread>(BotControl, std::ref(isControl));
                    control_th->detach();

                }
            }else{
                isControl = false;
                switch (UserSet_time.automode)
                {
                case 0:
                    isControl = false;
                    isAutoMode_Time = true;
                    isAutoMode_Power = false;
                    // AutoTime_th.detach();

                    usleep(1000000);
                    AutoTime_th = std::make_unique<std::thread>(TimeMode, UserSet_time, std::ref(isAutoMode_Time));
                    AutoTime_th->detach();

                    break;

                case 1:
                    isControl = false;
                    isAutoMode_Time = false;
                    isAutoMode_Power = true;
                    // AutoPower_th.detach();

                    usleep(1000000);
                    AutoPower_th = std::make_unique<std::thread>(PowerMode, UserSet_time, std::ref(isAutoMode_Power));
                    AutoPower_th->detach();

                    break;

                default:
                    break;
                }
            }


            lastHash = currentHash;
        }
        /************************测试代码（4）****************************/
        usleep(1000000);
    }
    return 0;
}