#ifndef PROCESSDATA_HPP
#define PROCESSDATA_HPP


#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <nlohmann/json.hpp>
#include <openssl/sha.h> // 用于计算文件哈希值
#include <chrono>

#include "types.h"

using json = nlohmann::json;

// 全局配置变量及其锁
json globalConfig;
std::mutex configMutex;
extern std::mutex mtx;

// 计算文件的 SHA-256 哈希值
std::string calculateFileHash(const std::string &filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return "";
    }

    SHA256_CTX ctx;
    SHA256_Init(&ctx);

    char buffer[4096];
    while (file.read(buffer, sizeof(buffer))) {
        SHA256_Update(&ctx, buffer, file.gcount());
    }
    if (file.gcount() > 0) {
        SHA256_Update(&ctx, buffer, file.gcount());
    }

    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256_Final(hash, &ctx);

    std::ostringstream oss;
    for (unsigned char c : hash) {
        oss << std::hex << std::setw(2) << std::setfill('0') << (int)c;
    }
    return oss.str();
}


// 加载 JSON 配置文件并更新全局变量
void loadConfig(const std::string &filePath) {
    std::ifstream file(filePath);
    if (!file) {
        std::cerr << "Failed to open config file: " << filePath << std::endl;
        return;
    }

    //相当于赋值整个文件的字节流，通过json格式解析
    try {
        json newConfig;
        file >> newConfig;

        // 使用锁保护全局配置变量
        std::lock_guard<std::mutex> lock(configMutex);
        globalConfig = newConfig;
        std::cout << "Configuration updated: " << globalConfig.dump(4) << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error parsing JSON file: " << e.what() << std::endl;
    }
}

// 监控配置文件的线程函数
void monitorConfigFile(const std::string &filePath, std::atomic<bool> &stopFlag, int checkIntervalMs = 2000) {
    std::string lastHash = calculateFileHash(filePath);

    while (!stopFlag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(checkIntervalMs));

        std::string currentHash = calculateFileHash(filePath);
        if (currentHash != lastHash) {
            std::cout << "Configuration file changed. Reloading..." << std::endl;
            loadConfig(filePath);
            lastHash = currentHash;
        }
    }
}



//底盘API数据预处理
std::string CreateActionPoint(double x, double y, double yaw)
{
    json temp_data;

    temp_data["action_name"] = "slamtec.agent.actions.MoveToAction";
    temp_data["options"]["target"]["x"] = x;
    temp_data["options"]["target"]["y"] = y;
    temp_data["options"]["target"]["z"] = 0;
    temp_data["options"]["move_options"]["mode"] = 0;
    temp_data["options"]["move_options"]["flags"] = "with_yaw";
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



#endif
