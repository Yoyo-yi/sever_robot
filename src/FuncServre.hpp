#ifndef FUNCSERVER_HPP
#define FUNCSERVER_HPP

#include <iostream>
#include <string>
#include <mutex>

#include "ProcessData.hpp"
#include "control.hpp"
#include "ComServer.hpp"
#include "Toollib.hpp"
#include "types.h"

extern std::mutex mtx;
extern power_status bot_power_status;
extern location_status bot_location_status;
extern bool BotUpdataFlag;

void BotControl(std::atomic<bool> &runFlag){

    Controller bot_control;
    bool control_flag = 0;
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

// 遍历 points 并依次调用 CreateActionPoint
std::string ProcessPoints(const std::vector<location_status>& points, std::atomic<bool> &runFlag) {
    std::string HTTP_RETURN;

    for (const auto& point : points) {
        HTTP_RETURN = BotOrder(POST_MOVE, HTTP_OD::BOT_POST, CreateActionPoint(point.x, point.y, point.yaw));
        ActionStatus(2000,runFlag);
        if(!runFlag){
            std::cout << "interrupted by stop signal.\n";
            return "stop";
        }
        
    }

    return HTTP_RETURN;
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
    // std::cout << "x:" << location_key.at("x").get<float>() << std::endl;
    // std::cout << "y:" << location_key.at("y").get<float>() << std::endl;
    // std::cout << "yaw:" << location_key.at("yaw").get<float>() << std::endl;


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
        int PowerUpdataFlag = PowerStatusUpdata(bot_power_status);
        //std::cout << "当前电量： " << bot_power_status.batteryPercentage << "%" << std::endl;
        int LocationUpdataFlag = LocationUpdata(bot_location_status);
        //std::cout << "当前位置： " << "( " << bot_location_status.x << " , " << bot_location_status.y << " , " << bot_location_status.yaw << " )" << std::endl;


        // 数据更新服务是否成功
        if(PowerUpdataFlag != 1 || LocationUpdataFlag != 1){
            BotUpdataFlag = 0;
            continue;
        }else{
            BotUpdataFlag = 1;
        }

        std::cout << "当前位置: " << bot_location_status.x << "  " << bot_location_status.y << std::endl;

        qt_user_need_data["batteryPercentage"] = bot_power_status.batteryPercentage;
        qt_user_need_data["dockingStatus"] = bot_power_status.dockingStatus;
        qt_user_need_data["isCharging"] = bot_power_status.isCharging;
        qt_user_need_data["x"] = bot_location_status.x;
        qt_user_need_data["y"] = bot_location_status.y;

        json_str = qt_user_need_data.dump();

        // std::cout << json_str.c_str() << std::endl;

        // socketServer(SERVER_ADDR,9998,(char*)json_str.c_str(), 0);

        usleep(1000000);
    }
    
}



void HttpBotMove(const httplib::Request& req, httplib::Response& res) {
    std::cout << "Received POST request for /MovePoint" << std::endl;

    try {
        // 解析 JSON 数据
        auto jsonData = json::parse(req.body);
        json response_data;

        // 检查字段是否存在并提取
        if (jsonData.contains("x") && jsonData.contains("y") && jsonData.contains("yaw")) {
            double x = jsonData["x"];
            double y = jsonData["y"];
            double yaw = jsonData["yaw"];
            std::cout << x << " " << y << " " << yaw << std::endl;

            // 创建动作点并执行命令
            std::string move_point_flag = "";
            std::string point = CreateActionPoint(x, y, yaw);
            move_point_flag = BotOrder(POST_MOVE, HTTP_OD::BOT_POST, point);

            if (move_point_flag.compare("err") == 0){
                response_data = {
                    {"code", 500},
                    {"msg", NULL},
                    {"data","失败"}
                };
            }else{
                response_data = {
                    {"code", 200},
                    {"msg", NULL},
                    {"data","成功"}
                };
            }

            // 设置 HTTP 响应状态和内容
            res.status = 200;  // 设置 HTTP 状态为 200 OK
            res.set_content(response_data.dump(), "application/json");
            // res.set_content("Move command accepted", "text/plain");
        } else {
            // 字段缺失，返回 400 Bad Request
            response_data = {
                {"code", 400},
                {"msg", NULL},
                {"data","请求字段缺失"}
            };
            res.status = 400;
            res.set_content(response_data.dump(), "application/json");
            // res.set_content("Missing required fields: x, y, or yaw", "text/plain");
        }
    } catch (const json::exception& e) {
        // JSON 解析错误
        res.status = 400;  // 设置 HTTP 状态为 400 Bad Request
        res.set_content("Invalid JSON: " + std::string(e.what()), "text/plain");
    } catch (const std::exception& e) {
        // 其他错误
        res.status = 500;  // 设置 HTTP 状态为 500 Internal Server Error
        res.set_content("Internal Server Error: " + std::string(e.what()), "text/plain");
    }

}

void HttpBotData(const httplib::Request& req, httplib::Response& res) {
    std::cout << "Received GET request for /CurrentData" << std::endl;
    try {
            // 构造 JSON 响应
            json response_data;
            if(BotUpdataFlag = 1){
                response_data = {
                    {"code", 200},
                    {"msg", "null"},
                    {"data",{
                        {"x", bot_location_status.x},
                        {"y", bot_location_status.y},
                        {"yaw", bot_location_status.yaw},
                        {"dockingStatus", bot_power_status.dockingStatus},
                        {"isCharging", bot_power_status.isCharging},
                        {"batteryPercentage", bot_power_status.batteryPercentage}
                    }}

                };
            }else{
                response_data = {
                    {"code", 500},
                    {"msg", "null"},
                    {"data",{
                        {"x", bot_location_status.x},
                        {"y", bot_location_status.y},
                        {"yaw", bot_location_status.yaw},
                        {"dockingStatus", bot_power_status.dockingStatus},
                        {"isCharging", bot_power_status.isCharging},
                        {"batteryPercentage", bot_power_status.batteryPercentage}
                    }}

                };
            }
            

            // 设置成功的 HTTP 状态码和内容
            res.status = 200; // 200 OK
            res.set_content(response_data.dump(), "application/json");
    } catch (const std::exception& e) {
        // 捕获异常并返回错误状态
        res.status = 500; // 500 Internal Server Error
        res.set_content("Internal Server Error: " + std::string(e.what()), "text/plain");
    }
}

void BotLocalMap(const httplib::Request& req, httplib::Response& res) {
    // 本地 PNG 文件路径
    std::string file_path = MAP_FILE_PATH;

    // 打开文件
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        // 如果文件不存在或无法打开，返回 404
        res.status = 404;
        res.set_content("File not found or busy", "text/plain");
        return;
    }

    // 读取文件内容到字符串流
    std::ostringstream buffer;
    buffer << file.rdbuf();
    std::string file_content = buffer.str();


    // 设置 HTTP 响应头和内容
    res.status = 200;
    res.set_content(file_content, "image/png");


    // // 构造 JSON 数据
    // std::string json_content = R"({
    //     "code": 200,
    //     "msg": "map.png"
    // })";

    // // 构造多部分响应
    // std::ostringstream response;
    // response << "--boundary123\r\n"
    //          << "Content-Type: application/json\r\n\r\n"
    //          << json_content << "\r\n"
    //          << "--boundary123\r\n"
    //          << "Content-Type: image/png\r\n\r\n"
    //          << file_content << "\r\n"
    //          << "--boundary123--\r\n";

    // res.set_content(response.str(), "multipart/mixed; boundary=boundary123");
    // res.status = 200;




}

void registerRobotHandlers(httplib::Server& svr) {
    //http定点运动接口
    svr.Post("/MovetoPoint", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            HttpBotMove(req, res);
        } catch (const std::exception& e) {
            res.status = 500;  // 设置 HTTP 状态为 500 Internal Server Error
            res.set_content("Internal Server Error: " + std::string(e.what()), "text/plain");
        }
    });

    //机器人状态接口
    svr.Get("/CurrentData", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            HttpBotData(req, res);
        } catch (const std::exception& e) {
            res.status = 500; // 捕获外层异常并返回 500 错误状态
            res.set_content("Internal Server Error: " + std::string(e.what()), "text/plain");
        }
    });

        //机器人状态接口
    svr.Get("/map", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            BotLocalMap(req, res);
        } catch (const std::exception& e) {
            res.status = 500; // 捕获外层异常并返回 500 错误状态
            res.set_content("Internal Server Error: " + std::string(e.what()), "text/plain");
        }
    });
}

int HttpServer(){
    httplib::Server svr;
    registerRobotHandlers(svr);
    svr.listen("0.0.0.0", 5001);
    return 0;
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
            // ProcessPoints(UserSet_time.patrol_points,runFlag);
            ActionStatus(2000,runFlag);
            //Sleep(50000);
            std::cout << "< Go Home >" << std::endl;
            HTTP_RETURN = BotOrder(POST_RETURN_POLE, HTTP_OD::BOT_POST, GoHomeAction().c_str());
            ActionStatus(2000,runFlag);
            //Sleep(35000);

            std::tm next_run_time = get_next_run_time(now_tm, setting_data.patrol_frequency);
            sleep_until(next_run_time,runFlag);
        }
        else {
            ;
        }

        usleep(500000);
    }
    std::cout << "out Time mode" << std::endl;
    return 0;
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

void MapInit() {
    std::cout << " Bot is Init Map " << std::endl;
    int Map_init_Flag = sendHttpPutFile(PUT_MAP,MAP_STCM_FILE_PATH);
    if(Map_init_Flag){
        std::cout << " Init Map succeed! " << std::endl;
    }else{
        std::cout << " Init Map failed! " << std::endl;
    }

}





#endif
