#include <iostream>
#include <chrono>
#include <ctime>

int main() {
    // 获取当前系统时间
    auto now = std::chrono::system_clock::now();
    
    // 转换为时间类型
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

    // 获取秒和分秒
    std::tm *localTime = std::localtime(&currentTime);
    int seconds = localTime->tm_sec;
    int minutes = localTime->tm_min;

    // 输出秒和分秒
    std::cout << "当前时间: " << minutes << " 分 " << seconds << " 秒\n";

    return 0;
}
