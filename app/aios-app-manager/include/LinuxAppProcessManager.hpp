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
private:
    inline static  const std::string TAG { "LinuxAppProcessManager"};
public:
    using ExitCallback = std::function<void(const ChildProcessInfo&)>;
    using CallbackId   = std::uint64_t;

    explicit LinuxAppProcessManager(asio::io_context& ctx);

    // ------------ lifecycle ------------
    [[nodiscard]] ChildProcessInfo start  (const LinuxAppInfo& app);
    void                       stop  (std::string_view instanceId);
    void                       stop  (const LinuxAppInfo& app)          { stop(app.instanceId); }
    void                       stop   (pid_t pid);
    [[nodiscard]] ChildProcessInfo  restart(const LinuxAppInfo& app);

    // ------------ query ----------------
    [[nodiscard]] ChildProcessInfo               query (std::string_view instanceId) const;
    [[nodiscard]] ChildProcessInfo               query (const LinuxAppInfo& app) const { return query(app.instanceId); }
    [[nodiscard]] std::vector<ChildProcessInfo>  list() const;

    // ------------ callback -------------
    [[nodiscard]] CallbackId  registerExitCallback(ExitCallback cb);
    [[nodiscard]] CallbackId registerExitCallback(std::string_view instancdId, ExitCallback cb);

    void removeExitCallback(CallbackId id);

private:
    // 子进程退出集中处理
    void onChildExit(pid_t pid, int rawStatus);

    // 发起二阶段停止：SIGTERM → timer → SIGKILL
    void asyncGracefulKill(pid_t pid, const std::string& id);

    // -----------------------------------
    struct TimedEntry {
        ChildProcessInfo                          info;
        std::shared_ptr<asio::steady_timer>  termTimer;   // nullptr == 未在停止流程
    };

    asio::io_context&                                   io_;
    ProcessMonitor                                      monitor_;
    mutable std::shared_mutex                           mu_;          // 保护下表
    std::unordered_map<std::string, TimedEntry>         table_;       // id  -> entry
    std::unordered_map<pid_t,  std::string>             pid2id_;      // pid -> id
    // std::vector<ExitCallback>                           callbacks_;
    struct CbEntry { ExitCallback cb; std::optional<std::string> filter;};
    std::unordered_map<CallbackId, CbEntry> callbacks_;
    CallbackId nextCbId_{1};

    static constexpr std::chrono::milliseconds kGracefulStopTimeout{1000};
};
