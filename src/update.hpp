#ifndef UPDATE_HPP
#define UPDATE_HPP


#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <nlohmann/json.hpp>
#include <openssl/sha.h> // 用于计算文件哈希值
#include <chrono>

using json = nlohmann::json;

// 全局配置变量及其锁
json globalConfig;
std::mutex configMutex;

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



#endif // UPDATE_HPP
