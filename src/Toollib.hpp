#ifndef TOOLLIB_HPP
#define TOOLLIB_HPP

#include <iostream>
#include <atomic>

#include <time.h>
#include <chrono>
#include <thread>
#include <vector>

#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>


#include "types.h"

#define HOUR2MIN(HOUR,MIN) HOUR*60+MIN

bool is_within_time_range(int in_hour, int in_min, int start_hour, int start_min, int end_hour, int end_min) {
    if (HOUR2MIN(start_hour, start_min) <= HOUR2MIN(end_hour, end_min)) {
        return (HOUR2MIN(in_hour,in_min) >= HOUR2MIN(start_hour, start_min)) && (HOUR2MIN(in_hour, in_min) < HOUR2MIN(end_hour, end_min));
    }
    else{
        return (HOUR2MIN(in_hour, in_min) >= HOUR2MIN(start_hour, start_min)) || (HOUR2MIN(in_hour, in_min) < HOUR2MIN(end_hour, end_min));
    }
}

void sleep_until(const std::tm& future_time, std::atomic<bool>& running) {
    auto future = std::chrono::system_clock::from_time_t(std::mktime(const_cast<std::tm*>(&future_time)));
    auto now = std::chrono::system_clock::now();

    constexpr std::chrono::milliseconds check_interval(200);

    while (running && now < future) {
        auto next_check_time = now + check_interval;
        if (next_check_time >= future) {
            // 如果下一次检查已经超过目标时间，直接等待到目标时间
            std::this_thread::sleep_until(future);
            break;
        }else {
            // 等待到下一个检查时间点
            std::this_thread::sleep_until(next_check_time);
        }

        // 更新当前时间
        now = std::chrono::system_clock::now();

        // 检查运行状态
        if (!running) {
            std::cout << "Sleep interrupted by stop signal.\n";
            return;
        }


    }
    if (running) {
        std::cout << "Reached the target time.\n";
    }

    // std::this_thread::sleep_until(future);
}


void TimeUpdata(std::tm& now_tm) {

    //std::unique_lock<std::mutex> lock(mtx);
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    localtime_r(&now_time,&now_tm);

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

    settings.patrol_points.clear();
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




#endif