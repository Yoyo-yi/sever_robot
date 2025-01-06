#ifndef HTTPSERVER_HPP
#define HTTPSERVER_HPP

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include "httplib.h"
#include "types.h"

using json = nlohmann::json;




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

static size_t PutCallback(void* buffer, size_t size, size_t count, void* response) 
{
    std::string* str = (std::string*)response;
    (*str).append((char*)buffer, size * count);

    return size * count;
}

int sendHttpPut(const std::string& url, const std::string& data, bool isFile) 
{
    int PUT_END = 1;
    std::string response;
    CURL* curl = curl_easy_init();
    if (curl) {
        CURLcode res;
        struct curl_slist* headers = NULL;

        if(isFile){
            FILE* file = fopen(data.c_str(), "rb");
            if (!file) {
                std::cerr << "Failed to open file: " << data << std::endl;
                return 0;
            }

            // 获取文件大小
            fseek(file, 0, SEEK_END);
            long fileSize = ftell(file);
            fseek(file, 0, SEEK_SET);

            // 设置 HTTP 基础选项
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, PutCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)&response);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, false); // 不验证证书
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, false);

            // 设置 HTTP 请求 URL
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

            // 设置 HTTP 请求头
            headers = curl_slist_append(headers, "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:76.0)");
            headers = curl_slist_append(headers, "Content-Type: application/octet-stream");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            // 设置 PUT 方法
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");

            // 设置文件流和读取回调
            curl_easy_setopt(curl, CURLOPT_READDATA, file);
            curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
            curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, static_cast<curl_off_t>(fileSize));

            // 执行 HTTP PUT 请求
            res = curl_easy_perform(curl);
            fclose(file);

            if (res != CURLE_OK) {
                std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                curl_slist_free_all(headers);
                curl_easy_cleanup(curl);
                return 0;
            }

        }else{
            // 处理字符串上传
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, PutCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)&response);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, false);  // 不验证证书
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, false);

            // 设置 HTTP 请求 URL
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

            // 设置 HTTP 请求头
            headers = curl_slist_append(headers, "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:76.0)");
            headers = curl_slist_append(headers, "Content-Type: text/plain");  // 设置为纯文本
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            // 设置 PUT 方法
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");

            // 设置要上传的字符串数据
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, data.length());

            // 执行 HTTP PUT 请求
            res = curl_easy_perform(curl);

            if (res != CURLE_OK) {
                std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                curl_slist_free_all(headers);
                curl_easy_cleanup(curl);
                return 0;
            }
        }
        
        // 清理资源
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        return 1;
    } else {
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




std::string BotOrder(const std::string& url, HTTP_OD CommandType, const std::string& http_data)
{
    std::cout << http_data << std::endl;
    if (CommandType == HTTP_OD::BOT_POST){
        if (http_data.empty()){
            std::cout << "POST data empty" << std::endl;
            return "err";
        }
        if (sendHttpPostRequest(url, http_data) == 0) {
            std::cout << "curl init err" << std::endl;
            return "err";
        }
        return "ok";
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
        return "ok";
    }
    else {
        std::cout << "order err" << std::endl;
        return "err";
    }
}



int socketServer(std::string server_addr, int server_port, char* socket_data_ptr, int SEND_OR_REV) {

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




#endif