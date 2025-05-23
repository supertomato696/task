#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
//#include "Logger.hpp"
#include "LinuxAppProcessManager.hpp"
#include "SignalHandler.hpp"

using namespace std::chrono;

// pretty time diff
static std::string toSec(steady_clock::duration d) {
    std::ostringstream os;
    os << duration_cast<seconds>(d).count() << "s";
    return os.str();
}

int main() {
    asio::io_context io;
    LinuxAppProcessManager pm{io};

    // ---------- 回调 ----------
    pm.registerExitCallback([](const ChildProcessInfo& p) {
        auto dur = p.endTime - p.startTime;
        std::cout << "[EXIT] " << std::setw(12) << p.instanceId
                  << "  pid=" << p.pid
                  << "  code=" << p.exitCode
                  << "  uptime=" << toSec(dur) << '\n';
    });


    SignalHandler::instance().register_default_signals();
    SignalHandler::instance().register_handler(SIGINT, [&io](int signo) {
        io.stop();
        std::cout << "SIGINT (Ctrl+C) caught, exiting...\n";
        // exit(0);
    });
    SignalHandler::instance().register_handler(SIGTERM, [](int signo) {
        std::cout << "SIGTERM caught\n";
    });

    SignalHandler::instance().start_worker();

    std::cout << "PID: " << getpid() << ". Try kill -SIGINT <pid> or Ctrl+C\n";

    // ---------- 启动 3 个进程 ----------
    const LinuxAppInfo info{
        .instanceId = "sleep_long",
        .execPath   = "/bin/sleep",
        .entrance   = {"10"},
    };

 for (int i{0};  i < 3; ++i) {
        pm.start(info);
    }

    // ---------- 定时任务 ----------
    // 5s: 重启 sleep_long
    asio::steady_timer t_restart(io, seconds(5));
    t_restart.async_wait([&](auto) {
        LinuxAppInfo info;
        info.instanceId = "sleep_long";
        info.execPath   = "/bin/sleep";
        info.entrance   = {"30"};
        std::cout << "\n>>> Restarting sleep_long …\n";
        auto pi = pm.restart(info);
        std::cout << "[RESTART] sleep_long new pid=" << pi.pid << '\n';
    });


    asio::steady_timer t_stop(io, seconds(8));
    t_stop.async_wait([&](auto) {
        std::cout << "\n>>> Stopping short tasks …\n";
        pm.stop("sleep_long");

    });





    io.run();
    std::cout << "All done.\n";
    return 0;
}