// demo_sleep.cpp
#include <asio/io_context.hpp>
#include <iostream>
#include "LinuxAppProcessManager.hpp"

int main() {
    asio::io_context io;
    LinuxAppProcessManager pm{io};

    pm.registerExitCallback([](const ChildProcessInfo& p) {
        std::cout << "[CALLBACK] " << p.instanceId
                  << " exited with code " << p.exitCode << '\n';
    });

    LinuxAppInfo app;
    app.instanceId = "sleep2";
    app.execPath   = "/bin/sleep";
    app.entrance   = {"2"};

    auto pi = pm.start(app);
    std::cout << "started pid=" << pi.pid << '\n';

    io.run();                              // 约 2 s 后回调触发
}
