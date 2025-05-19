#pragma once

#include <asio.hpp>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

#include "ProcessTypes.hpp"
#include "LinuxAppInfo.hpp"
#include "ProcessMonitor.hpp"
namespace lapm {

class LinuxAppProcessManager {
public:
    using ExitCallback = std::function<void(const ProcessInfo&)>;

    explicit LinuxAppProcessManager(asio::io_context& ctx);
    ~LinuxAppProcessManager();

    // ---------- lifecycle ----------
    [[nodiscard]] ProcessInfo start(const LinuxAppInfo& app);
    void stop(std::string_view instanceId);
    void stop(const LinuxAppInfo& app) { stop(app.instanceId); }
    ProcessInfo restart(const LinuxAppInfo& app);

    // ---------- query --------------
    [[nodiscard]] ProcessInfo query(std::string_view instanceId) const;
    [[nodiscard]] ProcessInfo query(const LinuxAppInfo& app) const { return query(app.instanceId); }
    [[nodiscard]] std::vector<ProcessInfo> list() const;

    // ---------- callbacks ----------
    void registerExitCallback(ExitCallback cb);

private:
    struct ProcEntry {
        ProcessInfo info;
        std::shared_ptr<asio::steady_timer> killTimer; // for graceful stop
    };

    // ------------ impl internals ------------
    void onChildExit(pid_t pid, int status);

    asio::io_context& ctx_;
    ProcessMonitor monitor_;

    mutable std::shared_mutex mtx_;
    std::unordered_map<std::string, ProcEntry> byId_;
    std::unordered_map<pid_t, std::string> idByPid_;
    std::vector<ExitCallback> callbacks_;
};

} // namespace lapm
