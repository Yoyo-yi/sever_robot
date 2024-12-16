#include <iostream>
#include <string>
#include <time.h>
#include <chrono>
#include <csignal>
#include <unistd.h>

#include <thread>
#include <memory>
#include <mutex>
#include <fstream>
#include <sstream>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


#include "ProcessData.hpp"
#include "ComServer.hpp"
#include "FuncServre.hpp" 
#include "Toollib.hpp"
#include "types.h"


using json = nlohmann::json;





void signal_handler(int signal) 
{
    curl_global_cleanup();
    std::exit(signal);
}

int main(void)
{
    
    memset(&UserSet_time, 0, sizeof(UserSet_time));

    // 基于linux平台优化curl
    InitGlobalCurl(CURL_GLOBAL_ALL);

    // 初始地图上传到固件
    MapInit();

    // 创建数据服务线程
    // UI数据交互
    std::thread bot_updata(BotUpdata);
    bot_updata.detach();
    // http数据服务器
    std::thread http_server(HttpServer);
    http_server.detach();

    //用户数据解析（解析公共的json配置文件
    updateUserSettingFromJson(POINT_FILE_PATH, UserSet_time);
    // GetUserInitData(UserSet_time);
    // UserSet_time.patrol_points = get_point_from_file(POINT_FILE_PATH);

    //注册终止条件，以释放curl产生的全局资源
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string lastHash = calculateFileHash(POINT_FILE_PATH);

    // 线程指针（方便调用，当然如果用线程池会更方便
    std::unique_ptr<std::thread> control_th;
    std::unique_ptr<std::thread> AutoTime_th;
    std::unique_ptr<std::thread> AutoPower_th;

    control_th = std::make_unique<std::thread>(BotControl, std::ref(isControl));
    control_th->detach();

    
    // std::thread control_th(BotControl,std::ref(isControl));
    // std::thread AutoTime_th(TimeMode,UserSet_time,std::ref(isAutoMode_Time));
    // std::thread AutoPower_th(PowerMode,UserSet_time,std::ref(isAutoMode_Power));
    // control_th.detach();

    std::cout << "<< Star >>" << std::endl;
    while (1) {

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
            isControl = false;
            isAutoMode_Time = false;
            isAutoMode_Power = false;
            usleep(2000000);
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

        usleep(1000000);
    }
    return 0;
}