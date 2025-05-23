// demo_sleep.cpp
#include <asio/io_context.hpp>
#include <iostream>
#include "LinuxAppProcessManager.hpp"
#include "SignalHandler.hpp"

int main() {
    asio::io_context io;

    SignalHandler::instance().register_default_signals();
    SignalHandler::instance().start_worker();

    SignalHandler::instance().register_handler(SIGINT, [&io](int signo) {
        std::cout << "SIGINT caught, worker mode exit\n";
        io.stop();
//        exit(0);
    });


    LinuxAppProcessManager pm{io};

    pm.registerExitCallback([](const ChildProcessInfo& p) {
        std::cout << "[CALLBACK] " << p.instanceId
                  << " exited with code " << p.exitCode << '\n';
    });

    LinuxAppInfo app;
    app.instanceId = "double_fork";
    app.execPath   = "/usr/bin/code";
//    app.entrance   = {"2"};
//    enum class ProcessState { Starting, Running, Stopping, Exited, Unknown, Failed };
    auto pi = pm.start(app);
    switch(pi.state) {
        case ProcessState::Starting:
            std::cout << "starting\n";
            break;
        case ProcessState::Running:
            std::cout << "running\n";
            break;
        case ProcessState::Stopping:
            std::cout << "stopping\n";
            break;
        case ProcessState::Exited:
            std::cout << "exited\n";
            break;
        case ProcessState::Unknown:
            std::cout << "unknown\n";
            break;
    }
    std::cout << "started pid=" << pi.pid << '\n';

    io.run();                              // 约 2 s 后回调触发
}
