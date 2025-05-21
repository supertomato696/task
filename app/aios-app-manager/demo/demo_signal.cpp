#include "SignalHandler.hpp"
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>

int main() {
//    SignalHandler::instance().register_default_signals();
//
//    SignalHandler::instance().register_handler(SIGINT, [](int signo) {
//        std::cout << "SIGINT (Ctrl+C) caught, exiting...\n";
//        exit(0);
//    });
//    SignalHandler::instance().register_handler(SIGTERM, [](int signo) {
//        std::cout << "SIGTERM caught\n";
//    });
//
//    std::cout << "PID: " << getpid() << ". Try kill -SIGINT <pid> or Ctrl+C\n";
//    while (true) {
//        // 业务主循环
//        std::this_thread::sleep_for(std::chrono::milliseconds(200));
//        SignalHandler::instance().poll();
//    }

    SignalHandler::instance().register_default_signals();
    SignalHandler::instance().start_worker();

    SignalHandler::instance().register_handler(SIGINT, [](int signo) {
        std::cout << "SIGINT caught, worker mode exit\n";
        exit(0);
    });

    std::cout << "Worker running. PID: " << getpid() << std::endl;
    while (true) {
        // 你主线程可以忙别的，不用管信号
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }

}
