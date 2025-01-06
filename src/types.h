#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <atomic>


#define BOT_ADDR_IP "http://192.168.110.53:1448"    //BOT的IP地址

#define GET_POWER_INFO      BOT_ADDR_IP "/api/core/system/v1/power/status"           //电量获取
#define GET_MYLOCATION      BOT_ADDR_IP "/api/core/slam/v1/localization/pose"        //当前位置
#define GET_ACTION_STATE    BOT_ADDR_IP "/api/core/motion/v1/actions/:current"       //当前运动状态

#define PUT_MAP             BOT_ADDR_IP "/api/core/slam/v1/maps/stcm"                //初始化地图地址
#define PUT_SPEED           BOT_ADDR_IP "/api/core/system/v1/parameter"
                                        

#define POST_MOVE           BOT_ADDR_IP "/api/core/motion/v1/actions"           //移动
#define POST_RETURN_POLE    BOT_ADDR_IP "/api/core/motion/v1/actions"           //回桩


//路径点存储绝对路径
#define POINT_FILE_PATH    "//home//orangepi//swy//patrol//config//config.json"
// #define POINT_FILE_PATH    "//home//orangepi//python//gui//saved_data.json"

// 本地地图的路径
#define MAP_FILE_PATH           "//home//orangepi//swy//qt_ws//map.png"
#define MAP_STCM_FILE_PATH      "//home//orangepi//swy//qt_ws//map.stcm"


#define SERVER_ADDR  "127.0.0.1"
// #define SERVER_ADDR  "192.168.110.149"

int server_PORT = 9999;





// 线程控制
std::atomic<bool> isControl{true};
std::atomic<bool> isAutoMode_Time{false};
std::atomic<bool> isAutoMode_Power{false};
// 数据更新标志位
bool BotUpdataFlag = 1;

enum class HTTP_OD{
    BOT_POST = 0,
    BOT_GET,
    BOT_PUT,
    BOT_DELET
};

struct power_status {
    int batteryPercentage;  //电池百分比
    int dockingStatus;      //对桩状态
    bool isCharging;        //是否正在充电
    bool isDCConnected;     //电源状态
};

struct location_status {
    float x;
    float y;
    float z;
    float yaw;      //头朝向   
};

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



// 时间变量
std::tm now_tm;
// 全局数据锁
std::mutex mtx;


// 全局服务信息
power_status bot_power_status;

location_status bot_location_status;

user_setting UserSet_time;


#endif