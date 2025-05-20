#pragma once
#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include "Laucher.hpp"
#include "ProcessMonitor.hpp"
#include "ProcessTypes.hpp"
#include "LinuxAppInfor.hpp"

class LinuxAppProcessManager {
public:
    using ExitCallback = std::function<void(const ProcessInfo&)>;

    explicit LinuxAppProcessManager(asio::io_context& ctx);

    // ------------ lifecycle ------------
    [[nodiscard]] ProcessInfo start  (const LinuxAppInfo& app);
    void                       stop  (std::string_view instanceId);
    void                       stop  (const LinuxAppInfo& app)          { stop(app.instanceId); }
    [[nodiscard]] ProcessInfo  restart(const LinuxAppInfo& app);

    // ------------ query ----------------
    [[nodiscard]] ProcessInfo               query (std::string_view instanceId) const;
    [[nodiscard]] ProcessInfo               query (const LinuxAppInfo& app) const { return query(app.instanceId); }
    [[nodiscard]] std::vector<ProcessInfo>  list() const;

    // ------------ callback -------------
    void registerExitCallback(ExitCallback cb);

private:
    // 子进程退出集中处理
    void onChildExit(pid_t pid, int rawStatus);

    // 发起二阶段停止：SIGTERM → timer → SIGKILL
    void asyncGracefulKill(pid_t pid, const std::string& id);

    // -----------------------------------
    struct TimedEntry {
        ProcessInfo                          info;
        std::shared_ptr<asio::steady_timer>  termTimer;   // nullptr == 未在停止流程
    };

    asio::io_context&                                   io_;
    ProcessMonitor                                      monitor_;
    mutable std::shared_mutex                           mu_;          // 保护下表
    std::unordered_map<std::string, TimedEntry>         table_;       // id  -> entry
    std::unordered_map<pid_t,  std::string>             pid2id_;      // pid -> id
    std::vector<ExitCallback>                           callbacks_;

    static constexpr std::chrono::milliseconds kGracefulStopTimeout{1000};
};
