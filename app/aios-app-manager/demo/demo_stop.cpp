// demo_stop.cpp
#include <asio/io_context.hpp>
#include <chrono>
#include <iostream>
#include "LinuxAppProcessManager.hpp"

int main() {
    asio::io_context io;
    LinuxAppProcessManager pm{io};

    pm.registerExitCallback([](const ProcessInfo& p) {
        std::cout << "[STOP-CB] " << p.instanceId
                  << " exitCode=" << p.exitCode << '\n';
    });

    LinuxAppInfo app;
    app.instanceId = "sleep100";
    app.execPath   = "/bin/sleep";
    app.entrance   = {"100"};

    pm.start(app);

    // 1 秒后发送 stop -> SIGTERM；3 秒后自动 SIGKILL
    asio::steady_timer t(io, std::chrono::seconds(1));
    t.async_wait([&](auto){ pm.stop("sleep100"); });

    io.run();
}
