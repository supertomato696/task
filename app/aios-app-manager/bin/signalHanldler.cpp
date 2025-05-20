#include <iostream>
#include <csignal>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <condition_variable>

std::mutex mtx;
std::condition_variable cv;
bool signal_received = false;  // 用于标记信号是否接收

// 信号处理函数
void handle_signal(int signal) {
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (signal == SIGINT) {
            std::cout << "Caught SIGINT (Ctrl+C), preparing to exit..." << std::endl;
        } else if (signal == SIGTERM) {
            std::cout << "Caught SIGTERM, preparing to terminate..." << std::endl;
        }
        signal_received = true;  // 标记信号已接收
    }
    cv.notify_all();  // 通知其他等待的线程
}

// 处理信号的线程函数
void signal_handler_thread() {
    std::cout << "Signal handler thread running. Waiting for signals..." << std::endl;

    // 无限循环，等待条件变量的通知
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [] { return signal_received; });  // 等待信号通知

        // 信号被处理后的逻辑
        std::cout << "Signal received, performing cleanup..." << std::endl;
        // 模拟清理过程
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Cleanup done. Exiting program." << std::endl;
        break;
    }
}

int main() {
    // 设置信号处理函数
    struct sigaction sa;
    sa.sa_handler = handle_signal;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);

    // 捕获 SIGINT（Ctrl+C）
    if (sigaction(SIGINT, &sa, nullptr) == -1) {
        std::cerr << "Error setting up SIGINT handler" << std::endl;
        return 1;
    }

    // 捕获 SIGTERM（终止信号）
    if (sigaction(SIGTERM, &sa, nullptr) == -1) {
        std::cerr << "Error setting up SIGTERM handler" << std::endl;
        return 1;
    }

    // 启动处理信号的线程
    std::thread handler_thread(signal_handler_thread);

    std::cout << "Program running. Press Ctrl+C to stop or send SIGTERM to terminate." << std::endl;

    // 模拟程序运行
    while (true) {
      std::cout << "Program is running..." << std::endl;
        sleep(1);  // 模拟长时间运行的程序
    }

    // 等待信号处理线程完成
    handler_thread.join();

    return 0;
}
