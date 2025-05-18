// =============================================================
//  LinuxAppInfo.hpp  (C++20 skeleton)
// =============================================================
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

/**
 * Lightweight description of an application instance to be managed
 * by LinuxAppProcessManager.
 */
struct LinuxAppInfo {
    std::string                       instanceId;   // unique name per running instance
    std::filesystem::path             execPath;     // executable path
    std::vector<std::string>          entrance;     // argv-style launch args (argv[0] excluded)
    std::optional<int>                priority;     // nice value, optional
    std::optional<std::string>        envInline;    // "K=V,K=V" style env string
    std::optional<std::filesystem::path> envFile;   // file with KEY=VALUE lines
    bool                              autostart{false};
};

// =============================================================
//  ProcessInfo.hpp  (C++20 skeleton)
// =============================================================
#pragma once

#include <chrono>
#include <filesystem>
#include <string>
#include <sys/types.h>  // pid_t

enum class ProcessState {
    Starting,
    Running,
    Stopping,
    Exited,
    Unknown
};

struct ProcessInfo {
    pid_t                                        pid{0};
    ProcessState                                state{ProcessState::Unknown};
    std::string                                 instanceId;
    std::filesystem::path                       execPath;
    std::chrono::steady_clock::time_point       startTime{};
    int                                         exitCode{0};   // valid when state==Exited
};

// =============================================================
//  LinuxAppProcessManager.hpp  (C++20 skeleton)
// =============================================================
#pragma once

#include "LinuxAppInfo.hpp"
#include "ProcessInfo.hpp"
#include <functional>
#include <memory>
#include <string_view>
#include <vector>

namespace asio { class io_context; }

class LinuxAppProcessManager {
public:
    using ExitCallback = std::function<void(const ProcessInfo&)>;

    explicit LinuxAppProcessManager(asio::io_context& ctx);
    ~LinuxAppProcessManager();

    // ------------------- lifecycle -------------------
    [[nodiscard]] ProcessInfo start(const LinuxAppInfo& app);

    void                     stop(std::string_view instanceId);
    void                     stop(const LinuxAppInfo& app);

    [[nodiscard]] ProcessInfo restart(const LinuxAppInfo& app);

    // ------------------- queries ---------------------
    [[nodiscard]] ProcessInfo              query(std::string_view instanceId) const;
    [[nodiscard]] ProcessInfo              query(const LinuxAppInfo& app) const;
    [[nodiscard]] std::vector<ProcessInfo> list() const;

    // ------------------- events ----------------------
    void registerExitCallback(ExitCallback cb);

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

// =============================================================
//  LinuxAppProcessManager.cpp  (C++20 skeleton)
// =============================================================
#include "LinuxAppProcessManager.hpp"

#include <asio/io_context.hpp>
#include <shared_mutex>
#include <unordered_map>
#include <utility>

// Forward‑declared helper structs/classes can be defined in an anonymous
// namespace to keep symbols local to the translation unit.
namespace {
    struct ProcessKey {
        std::string instanceId;
        bool operator==(const ProcessKey&) const = default;
    };

    struct ProcessKeyHash {
        std::size_t operator()(const ProcessKey& k) const noexcept {
            return std::hash<std::string>{}(k.instanceId);
        }
    };
} // namespace

struct LinuxAppProcessManager::Impl {
    explicit Impl(asio::io_context& io)
        : ioContext{io}, signalSet{io, SIGCHLD} {
        // TODO: set up SIGCHLD async wait here
    }

    asio::io_context&                                              ioContext;
    asio::signal_set                                               signalSet;
    std::unordered_map<ProcessKey, ProcessInfo, ProcessKeyHash>    processes;
    mutable std::shared_mutex                                      mutex;
    std::vector<ExitCallback>                                      callbacks;
};

// ------------------- public API -------------------
LinuxAppProcessManager::LinuxAppProcessManager(asio::io_context& ctx)
    : p_(std::make_unique<Impl>(ctx)) {}

LinuxAppProcessManager::~LinuxAppProcessManager() = default;

ProcessInfo LinuxAppProcessManager::start(const LinuxAppInfo& app) {
    // TODO: fork + exec using app; populate ProcessInfo.
    // insert into map under unique lock.
    return {};
}

void LinuxAppProcessManager::stop(std::string_view instanceId) {
    // TODO: send SIGTERM/SIGKILL to pid.
}

void LinuxAppProcessManager::stop(const LinuxAppInfo& app) {
    stop(app.instanceId);
}

ProcessInfo LinuxAppProcessManager::restart(const LinuxAppInfo& app) {
    stop(app);
    return start(app);
}

ProcessInfo LinuxAppProcessManager::query(std::string_view instanceId) const {
    std::shared_lock lock{p_->mutex};
    if (auto it = p_->processes.find({std::string(instanceId)}); it != p_->processes.end()) {
        return it->second;
    }
    return {};
}

ProcessInfo LinuxAppProcessManager::query(const LinuxAppInfo& app) const {
    return query(app.instanceId);
}

std::vector<ProcessInfo> LinuxAppProcessManager::list() const {
    std::shared_lock lock{p_->mutex};
    std::vector<ProcessInfo> out;
    out.reserve(p_->processes.size());
    for (auto& [_, info] : p_->processes) {
        out.push_back(info);
    }
    return out;
}

void LinuxAppProcessManager::registerExitCallback(ExitCallback cb) {
    std::unique_lock lock{p_->mutex};
    p_->callbacks.push_back(std::move(cb));
}

// =============================================================
//  TODOs
//  * Implement async SIGCHLD handling and state transitions
//  * Finish fork/exec launcher with env + priority + entrance args
//  * Ensure locks are released before invoking callbacks
//  * Add exhaustive error handling & logging
// =============================================================












// =============================================================
//  linux_app_process_manager – Modular C++20 Skeleton (v2)
// =============================================================
//  This revision introduces:
//   1. Graceful‑then‑force stop sequence (SIGTERM → timeout → SIGKILL)
//   2. Dedicated EnvManager for inline/file env & LD_LIBRARY_PATH patch
//   3. Split Launcher / Monitor helpers
//   4. Clear timeout constants & modular header layout
//
//  Directory suggestion
//  ├── env/EnvManager.hpp
//  ├── process/Launcher.hpp
//  ├── process/Monitor.hpp
//  ├── include/ProcessTypes.hpp
//  ├── include/LinuxAppInfo.hpp
//  ├── include/LinuxAppProcessManager.hpp
//  └── src/LinuxAppProcessManager.cpp
// =============================================================

// =============================================================
//  include/ProcessTypes.hpp
// =============================================================
#pragma once

#include <chrono>
#include <filesystem>
#include <string>

#ifdef __linux__
#include <sys/types.h>
#endif

enum class ProcessState {
    Starting,
    Running,
    Stopping,
    Exited,
    Unknown
};

struct ProcessInfo {
    pid_t                                   pid {0};
    ProcessState                            state {ProcessState::Unknown};
    std::string                             instanceId;
    std::filesystem::path                   execPath;
    std::chrono::steady_clock::time_point   startTime;
    int                                     exitCode {0};
};

// =============================================================
//  include/LinuxAppInfo.hpp
// =============================================================
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

struct LinuxAppInfo {
    std::string                          instanceId;
    std::filesystem::path                execPath;
    std::vector<std::string>             entrance;      // argv extras
    std::optional<int>                   priority;      // nice
    std::optional<std::string>           envInline;     // KEY=VAL,KEY=VAL
    std::optional<std::filesystem::path> envFile;       // file with KEY=VAL per line
    bool                                 autostart {false};
};

// =============================================================
//  env/EnvManager.hpp
// =============================================================
#pragma once

#include "LinuxAppInfo.hpp"
#include <filesystem>
#include <string>

namespace env {

/** Apply inline and file based environment variables, and patch
 *  LD_LIBRARY_PATH with exec dir + sibling lib/lib64.
 */
struct EnvManager {
    static void apply(const LinuxAppInfo& app);

private:
    static void applyInline(const std::string& kvs);
    static void applyFile(const std::filesystem::path& file);
    static void patchLdLibrary(const std::filesystem::path& execPath);
};

} // namespace env

// =============================================================
//  process/Launcher.hpp
// =============================================================
#pragma once

#include "LinuxAppInfo.hpp"
#include "ProcessTypes.hpp"
#include "env/EnvManager.hpp"

namespace process {

struct Launcher {
    /** Fork‑and‑exec, returns ProcessInfo in parent, never returns in child. */
    static ProcessInfo launch(const LinuxAppInfo& app);
};

} // namespace process

// =============================================================
//  process/Monitor.hpp
// =============================================================
#pragma once

#include "ProcessTypes.hpp"
#include <asio/io_context.hpp>
#include <asio/signal_set.hpp>
#include <functional>
#include <unordered_map>
#include <vector>

namespace process {

struct Monitor {
    using ExitCallback = std::function<void(const ProcessInfo&)>;

    explicit Monitor(asio::io_context& io);

    void addProcess(const ProcessInfo& info);
    void removeProcess(pid_t pid);

    void registerExitCallback(ExitCallback cb);

    [[nodiscard]] ProcessInfo queryById(std::string_view id) const;
    [[nodiscard]] std::vector<ProcessInfo> snapshot() const;

private:
    void watchSigchld();
    void handleExits();

    mutable std::shared_mutex                          mtx_;
    std::unordered_map<pid_t, ProcessInfo>             procs_;
    std::vector<ExitCallback>                          cbs_;

    asio::signal_set                                   sigs_;
};

} // namespace process

// =============================================================
//  include/LinuxAppProcessManager.hpp
// =============================================================
#pragma once

#include "ProcessTypes.hpp"
#include "process/Monitor.hpp"
#include "process/Launcher.hpp"

#include <asio/io_context.hpp>
#include <chrono>
#include <string_view>

class LinuxAppProcessManager {
public:
    using ExitCallback = process::Monitor::ExitCallback;

    explicit LinuxAppProcessManager(asio::io_context& ctx,
                                    std::chrono::milliseconds stopTimeout = std::chrono::seconds(3));

    // lifecycle
    [[nodiscard]] ProcessInfo start  (const LinuxAppInfo& app);
    void                       stop  (std::string_view instanceId);
    void                       stop  (const LinuxAppInfo& app) { stop(app.instanceId); }
    [[nodiscard]] ProcessInfo  restart(const LinuxAppInfo& app);

    // query
    [[nodiscard]] ProcessInfo              query(std::string_view id) const;
    [[nodiscard]] std::vector<ProcessInfo> list() const;

    // events
    void registerExitCallback(ExitCallback cb);

private:
    void gracefulKill(pid_t pid) const;
    void forceKill   (pid_t pid) const;

    process::Monitor                monitor_;
    std::chrono::milliseconds       stopTimeout_;
};

// =============================================================
//  src/EnvManager.cpp  (implementation sketch)
// =============================================================
#include "env/EnvManager.hpp"
#include <cstdlib>
#include <fstream>
#include <sstream>

using namespace env;

void EnvManager::apply(const LinuxAppInfo& app) {
    if (app.envInline) applyInline(*app.envInline);
    if (app.envFile)   applyFile(*app.envFile);
    patchLdLibrary(app.execPath);
}

void EnvManager::applyInline(const std::string& kvs) {
    std::istringstream iss(kvs);
    std::string kv;
    while (std::getline(iss, kv, ',')) {
        auto pos = kv.find('=');
        if (pos == std::string::npos) continue;
        ::setenv(kv.substr(0, pos).c_str(), kv.substr(pos + 1).c_str(), 1);
    }
}

void EnvManager::applyFile(const std::filesystem::path& file) {
    std::ifstream fin(file);
    std::string line;
    while (std::getline(fin, line)) {
        auto pos = line.find('=');
        if (pos == std::string::npos) continue;
        ::setenv(line.substr(0, pos).c_str(), line.substr(pos + 1).c_str(), 1);
    }
}

void EnvManager::patchLdLibrary(const std::filesystem::path& exe) {
    auto dir = exe.parent_path();
    std::vector<std::filesystem::path> extras = {dir, dir / "lib", dir / "lib64"};

    const char* old = ::getenv("LD_LIBRARY_PATH");
    std::string  merged = old ? std::string(old) : "";
    for (auto& p : extras) {
        if (!std::filesystem::exists(p)) continue;
        if (!merged.empty()) merged += ':';
        merged += p.string();
    }
    ::setenv("LD_LIBRARY_PATH", merged.c_str(), 1);
}

// =============================================================
//  src/Launcher.cpp (implementation sketch)
// =============================================================
#include "process/Launcher.hpp"
#include <sys/wait.h>
#include <unistd.h>

using namespace process;

ProcessInfo Launcher::launch(const LinuxAppInfo& app) {
    std::vector<std::string> argStore;
    argStore.reserve(app.entrance.size() + 1);
    argStore.push_back(app.execPath.filename());
    argStore.insert(argStore.end(), app.entrance.begin(), app.entrance.end());

    std::vector<char*> argv;
    for (auto& s : argStore) argv.push_back(const_cast<char*>(s.c_str()));
    argv.push_back(nullptr);

    pid_t pid = ::fork();
    if (pid == -1) throw std::runtime_error("fork failed");

    if (pid == 0) {
        if (app.priority) ::nice(*app.priority);
        env::EnvManager::apply(app);
        ::execv(app.execPath.c_str(), argv.data());
        _exit(EXIT_FAILURE);
    }

    ProcessInfo info;
    info.pid        = pid;
    info.state      = ProcessState::Running;
    info.instanceId = app.instanceId;
    info.execPath   = app.execPath;
    info.startTime  = std::chrono::steady_clock::now();
    return info;
}

// =============================================================
//  src/Monitor.cpp (implementation sketch)
// =============================================================
#include "process/Monitor.hpp"
#include <sys/wait.h>

using namespace process;

Monitor::Monitor(asio::io_context& io) : sigs_(io, SIGCHLD) { watchSigchld(); }

void Monitor::addProcess(const ProcessInfo& info) {
    std::unique_lock lk(mtx_);
    procs_.emplace(info.pid, info);
}
void Monitor::removeProcess(pid_t pid) {
    std::unique_lock lk(mtx_);
    procs_.erase(pid);
}

void Monitor::registerExitCallback(ExitCallback cb) {
    std::unique_lock lk(mtx_);
    cbs_.push_back(std::move(cb));
}

void Monitor::watchSigchld() {
    sigs_.async_wait([this](auto, int){ handleExits(); watchSigchld(); });
}

void Monitor::handleExits() {
    int status = 0; pid_t pid;
    while ((pid = ::waitpid(-1, &status, WNOHANG)) > 0) {
        ExitCallbackList local;
        ProcessInfo      info;
        {
            std::unique_lock lk(mtx_);
            auto it = procs_.find(pid);
            if (it == procs_.end()) continue;
            info            = it->second;
            info.state      = ProcessState::Exited;
            info.exitCode   = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            procs_.erase(it);
            local = cbs_;
        }
        for (auto& cb : local) cb(info);
    }
}

ProcessInfo Monitor::queryById(std::string_view id) const {
    std::shared_lock lk(mtx_);
    for (auto& [_, info] : procs_) if (info.instanceId == id) return info;
    return {};
}
std::vector<ProcessInfo> Monitor::snapshot() const {
    std::shared_lock lk(mtx_);
    std::vector<ProcessInfo> out; out.reserve(procs_.size());
    for (auto& [_, v] : procs_) out.push_back(v);
    return out;
}

// =============================================================
//  src/LinuxAppProcessManager.cpp (public façade)
// =============================================================
#include "LinuxAppProcessManager.hpp"
#include <thread>
#include <unistd.h>

LinuxAppProcessManager::LinuxAppProcessManager(asio::io_context& ctx,
                                               std::chrono::milliseconds stopTimeout)
    : monitor_(ctx), stopTimeout_(stopTimeout) {}

ProcessInfo LinuxAppProcessManager::start(const LinuxAppInfo& app) {
    auto info = process::Launcher::launch(app);
    monitor_.addProcess(info);
    return info;
}

void LinuxAppProcessManager::gracefulKill(pid_t pid) const {
    ::kill(pid, SIGTERM);
    auto deadline = std::chrono::steady_clock::now() + stopTimeout_;
    while (std::chrono::steady_clock::now() < deadline) {
        if (::kill(pid, 0) == -1 && errno == ESRCH) return; // 已退出
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
void LinuxAppProcessManager::forceKill(pid_t pid) const { ::kill(pid, SIGKILL); }

void LinuxAppProcessManager::stop(std::string_view id) {
    auto info = monitor_.queryById(id);
    if (info.pid == 0) return;
    gracefulKill(info.pid);
    if (::kill(info.pid, 0) == 0) forceKill(info.pid);
}

ProcessInfo LinuxAppProcessManager::restart(const LinuxAppInfo& app) {
    stop(app.instanceId);
    return start(app);
}

ProcessInfo LinuxAppProcessManager::query(std::string_view id) const { return monitor_.queryById(id); }
std::vector<ProcessInfo> LinuxAppProcessManager::list() const { return monitor_.snapshot(); }
void LinuxAppProcessManager::registerExitCallback(ExitCallback cb) { monitor_.registerExitCallback(std::move(cb)); }
