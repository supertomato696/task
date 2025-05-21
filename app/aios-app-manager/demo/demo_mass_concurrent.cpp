#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include "LinuxAppProcessManager.hpp"

using namespace std::chrono;
using Clock = steady_clock;

// ---------- 帮助函数 ----------
static std::string diff(const Clock::time_point& a, const Clock::time_point& b) {
    return std::to_string(duration_cast<seconds>(b - a).count()) + "s";
}

int main() {
    asio::io_context io;
    LinuxAppProcessManager pm{io};

    // 随机器
    std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> distSleep(5, 20);  // 每进程睡眠 5~20s

    // ---------- 回调 ----------
    pm.registerExitCallback([](const ChildProcessInfo& p) {
        std::cout << "[EXIT] " << p.instanceId
                  << " pid=" << p.pid
                  << " code=" << p.exitCode
                  << " uptime=" << diff(p.startTime, p.endTime) << '\n';
    });

    // ---------- 1. 批量启动 10 个子进程 ----------
    struct Entry { std::string id; int secs; };
    std::vector<Entry> entries;
    for (int i = 0; i < 10; ++i)
        entries.push_back({"app_" + std::to_string(i), distSleep(rng)});

    for (auto& e : entries) {
        LinuxAppInfo info;
        info.instanceId = e.id;
        info.execPath   = "/bin/sleep";
        info.entrance   = {std::to_string(e.secs)};
        auto pi = pm.start(info);
        std::cout << "[START] " << e.id << " pid=" << pi.pid
                  << " duration=" << e.secs << "s\n";
    }

    // ---------- 2. 定时批量操作 ----------
    // 5s 后重启 3 个随机进程
    asio::steady_timer t_restart(io, seconds(5));
    t_restart.async_wait([&](auto) {
        std::shuffle(entries.begin(), entries.end(), rng);
        for (int i = 0; i < 3; ++i) {
            LinuxAppInfo info;
            info.instanceId = entries[i].id;
            info.execPath   = "/bin/sleep";
            info.entrance   = {std::to_string(distSleep(rng))};
            std::cout << "\n>>> Restarting " << info.instanceId << '\n';
            pm.restart(info);
        }
    });

    // 8s 后停止另外 3 个随机进程
    asio::steady_timer t_stop(io, seconds(8));
    t_stop.async_wait([&](auto) {
        std::shuffle(entries.begin(), entries.end(), rng);
        for (int i = 0; i < 3; ++i) {
            std::cout << "\n>>> Stopping " << entries[i].id << '\n';
            pm.stop(entries[i].id);
        }
    });

    // ---------- 3. 运行事件循环 ----------
    io.run();
    std::cout << "All managed processes exited.\n";
    return 0;
}