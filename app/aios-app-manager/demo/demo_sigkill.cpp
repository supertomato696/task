// demo_stop.cpp
#include <asio/io_context.hpp>
#include <chrono>
#include <iostream>
#include "LinuxAppProcessManager.hpp"

int main() {
    asio::io_context io;
    LinuxAppProcessManager pm{io};

    pm.registerExitCallback([](const ChildProcessInfo& p) {
        std::cout << "[STOP-CB] " << p.instanceId
                  << " exitCode=" << p.exitCode << '\n';
    });

    LinuxAppInfo app;
    app.instanceId = "signalHanlder";
    app.execPath   = "/home/tanbowen/Desktop/task/app/aios-app-manager/bin/signalHanldler";
    app.entrance   = {"100", "200", "autostart=true"};
    app.envInline = {"PATH=/usr/local/bin:/usr/bin:/bin:/usr/local/games:/usr/games,LD_LIBRARY_PATH=/test/lib:/test/lib64"};
    app.envFile = "/home/tanbowen/Desktop/task/app/aios-app-manager/environment-file/environment-config";

    pm.start(app);

    // 1 秒后发送 stop -> SIGTERM；3 秒后自动 SIGKILL
    asio::steady_timer t(io, std::chrono::seconds(1));
    t.async_wait([&](auto){ pm.stop("signalHanlder"); });

    io.run();
}
