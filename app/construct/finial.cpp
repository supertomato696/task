// =============================================================
//  Modular Linux App Process Manager – Full Skeleton (C++20)
//  textdoc id: 6829d0dfd82c8191a878c14ded7e7350   (v4)
// =============================================================
//  This single canvas packs minimal yet complete header/impl files
//  you can split into real *.hpp / *.cpp later.  Focus is correctness
//  of interfaces & threading strategy – not exhaustive error handling.
//
//  Required third‑party deps:
//      • standalone Asio 1.30+ (or <boost/asio.hpp> if preferred)
//      • fmtlib (for logging – replace with your logger of choice)
//
// =============================================================
//  1. Constants.hpp
// =============================================================
#pragma once

#include <chrono>

namespace lapm {
inline constexpr std::chrono::milliseconds kGracefulStopTimeout{3000};
} // namespace lapm

// =============================================================
//  2. ProcessTypes.hpp
// =============================================================
#pragma once

#include <chrono>
#include <filesystem>
#include <string>
#ifdef __linux__
#include <sys/types.h>
#endif

namespace lapm {

enum class ProcessState { Starting, Running, Stopping, Exited, Unknown };

struct ProcessInfo {
#ifdef __linux__
    pid_t                                   pid{};
#else
    int                                     pid{};
#endif
    ProcessState                            state{ProcessState::Unknown};
    std::string                             instanceId;
    std::filesystem::path                   execPath;
    std::chrono::steady_clock::time_point   startTime;
    int                                     exitCode{0};
};

} // namespace lapm

// =============================================================
//  3. LinuxAppInfo.hpp
// =============================================================
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lapm {

struct LinuxAppInfo {
    std::string                           instanceId;      // unique key
    std::filesystem::path                 execPath;        // absolute/relative path
    std::vector<std::string>              entrance;        // argv[1..]
    std::optional<int>                    priority;        // nice level
    std::optional<std::string>            envInline;       // "K=V,K2=V2"
    std::optional<std::filesystem::path>  envFile;         // path to env file
    bool                                  autostart{false};
};

} // namespace lapm

// =============================================================
//  4. EnvManager.hpp  – header‑only utility
// =============================================================
#pragma once

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace lapm {

class EnvManager {
public:
    using EnvMap = std::unordered_map<std::string, std::string>;

    static std::vector<std::string> buildEnvironment(const LinuxAppInfo& info) {
        EnvMap env = collectParentEnv();
        if (info.envInline) {
            parseInline(*info.envInline, env);
        }
        if (info.envFile) {
            parseFile(*info.envFile, env);
        }
        patchLdLibraryPath(info.execPath, env);
        // convert map -> vector<string buffer>
        std::vector<std::string> out;
        out.reserve(env.size());
        for (auto& [k, v] : env) {
            out.emplace_back(k + "=" + v);
        }
        return out;
    }

private:
    static EnvMap collectParentEnv() {
        EnvMap m;
        extern char** environ;
        for (char** p = environ; p && *p; ++p) {
            std::string_view sv(*p);
            auto pos = sv.find('=');
            if (pos == std::string_view::npos) continue;
            m.emplace(std::string(sv.substr(0, pos)), std::string(sv.substr(pos + 1)));
        }
        return m;
    }

    static void parseInline(std::string_view inlineStr, EnvMap& env) {
        size_t start = 0;
        while (start < inlineStr.size()) {
            size_t end = inlineStr.find(',', start);
            if (end == std::string_view::npos) end = inlineStr.size();
            std::string kv(trim(inlineStr.substr(start, end - start)));
            auto pos = kv.find('=');
            if (pos != std::string::npos) {
                env[kv.substr(0, pos)] = kv.substr(pos + 1);
            }
            start = end + 1;
        }
    }

    static void parseFile(const std::filesystem::path& file, EnvMap& env) {
        std::ifstream ifs(file);
        std::string line;
        while (std::getline(ifs, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            auto pos = line.find('=');
            if (pos != std::string::npos) {
                env[line.substr(0, pos)] = line.substr(pos + 1);
            }
        }
    }

    static void patchLdLibraryPath(const std::filesystem::path& exe,
                                   EnvMap& env) {
        auto dir = exe.is_absolute() ? exe.parent_path()
                                     : std::filesystem::current_path() / exe.parent_path();
        std::set<std::string> paths;
        auto split = [&](std::string_view sv) {
            size_t start = 0;
            while (start < sv.size()) {
                size_t pos = sv.find(':', start);
                if (pos == std::string_view::npos) pos = sv.size();
                paths.emplace(sv.substr(start, pos - start));
                start = pos + 1;
            }
        };
        if (auto it = env.find("LD_LIBRARY_PATH"); it != env.end()) split(it->second);
        auto push = [&](const std::filesystem::path& p) {
            paths.emplace(std::filesystem::weakly_canonical(p).string());
        };
        push(dir);
        push(dir / ".." / "lib");
        push(dir / ".." / "lib64");
        push(dir / "lib");
        push(dir / "lib64");
        std::string joined;
        for (auto& p : paths) {
            if (!joined.empty()) joined += ':';
            joined += p;
        }
        env["LD_LIBRARY_PATH"] = std::move(joined);
    }

    static std::string trim(std::string_view sv) {
        size_t b = 0, e = sv.size();
        while (b < e && std::isspace(static_cast<unsigned char>(sv[b]))) ++b;
        while (e > b && std::isspace(static_cast<unsigned char>(sv[e - 1]))) --e;
        return std::string(sv.substr(b, e - b));
    }
};

} // namespace lapm

// =============================================================
//  5. Launcher.hpp  – header‑only minimal impl
// =============================================================
#pragma once

#include <cerrno>
#include <cstring>
#include <fmt/core.h>
#include <stdexcept>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace lapm {

class Launcher {
public:
    static pid_t start(const LinuxAppInfo& app) {
        // argv
        std::vector<std::string> argvStrings;
        argvStrings.push_back(app.execPath.filename().string());
        argvStrings.insert(argvStrings.end(), app.entrance.begin(), app.entrance.end());
        std::vector<char*> argv;
        for (auto& s : argvStrings) argv.push_back(s.data());
        argv.push_back(nullptr);
        // envp
        auto envStrings = EnvManager::buildEnvironment(app);
        std::vector<char*> envp;
        for (auto& s : envStrings) envp.push_back(s.data());
        envp.push_back(nullptr);

        pid_t child = ::fork();
        if (child < 0) {
            throw std::system_error(errno, std::generic_category(), "fork failed");
        }
        if (child == 0) {
            // child process
            if (app.priority) {
                ::setpriority(PRIO_PROCESS, 0, *app.priority);
            }
            ::execve(app.execPath.c_str(), argv.data(), envp.data());
            // if execve returns -> error
            fmt::print(stderr, "execve failed: {}\n", std::strerror(errno));
            ::_exit(127);
        }
        return child; // parent gets pid
    }
};

} // namespace lapm

// =============================================================
//  6. ProcessMonitor.hpp   – header‑only async SIGCHLD handler
// =============================================================
#pragma once

#include <asio.hpp>
#include <sys/wait.h>
#include <functional>

namespace lapm {

class ProcessMonitor {
public:
    using ExitHandler = std::function<void(pid_t, int)>; // pid, exitStatus

    ProcessMonitor(asio::io_context& ctx, ExitHandler handler)
        : ctx_(ctx), signals_(ctx, SIGCHLD), handler_(std::move(handler)) {
        wait();
    }

private:
    void wait() {
        signals_.async_wait([this](const std::error_code& ec, int /*signal*/) {
            if (!ec) {
                reapChildren();
            }
            wait(); // continue waiting
        });
    }

    void reapChildren() {
        int status = 0;
        pid_t pid;
        while ((pid = ::waitpid(-1, &status, WNOHANG)) > 0) {
            handler_(pid, status);
        }
    }

    asio::io_context& ctx_;
    asio::signal_set signals_;
    ExitHandler handler_;
};

} // namespace lapm

// =============================================================
//  7. LinuxAppProcessManager.hpp (inline cpp impl at bottom)
// =============================================================
#pragma once

#include <asio.hpp>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

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

// =============================================================
//  7b. LinuxAppProcessManager.cpp   (inlined impl)
// =============================================================

#include <fmt/core.h>

namespace lapm {

LinuxAppProcessManager::LinuxAppProcessManager(asio::io_context& ctx)
    : ctx_(ctx),
      monitor_(ctx_, [this](pid_t pid, int status) { onChildExit(pid, status); }) {}

LinuxAppProcessManager::~LinuxAppProcessManager() = default;

ProcessInfo LinuxAppProcessManager::start(const LinuxAppInfo& app) {
    auto pid = Launcher::start(app);
    ProcessInfo pi;
    pi.pid = pid;
    pi.state = ProcessState::Running;
    pi.instanceId = app.instanceId;
    pi.execPath = app.execPath;
    pi.startTime = std::chrono::steady_clock::now();
    {
        std::unique_lock lk(mtx_);
        byId_[app.instanceId] = {pi, {}};
        idByPid_[pid] = app.instanceId;
    }
    return pi;
}

void LinuxAppProcessManager::stop(std::string_view id) {
    std::shared_lock rlk(mtx_);
    auto it = byId_.find(std::string(id));
    if (it == byId_.end()) return; // not running
    pid_t pid = it->second.info.pid;
    auto timer = std::make_shared<asio::steady_timer>(ctx_);
    it->second.killTimer = timer;
    rlk.unlock();

    // first gentle
    ::kill(pid, SIGTERM);
    timer->expires_after(kGracefulStopTimeout);
    timer->async_wait([this, pid](const std::error_code& ec) {
        if (ec) return; // cancelled
        // check still alive
        if (::kill(pid, 0) == 0) {
            ::kill(pid, SIGKILL);
        }
    });
}

ProcessInfo LinuxAppProcessManager::restart(const LinuxAppInfo& app) {
    stop(app.instanceId);
    // let monitor pick up exit then we can start again after small delay
    // For simplicity just post start
    asio::steady_timer t(ctx_, lapm::kGracefulStopTimeout);
    ProcessInfo ret;
    t.async_wait([this, &app, &ret](const std::error_code&) { ret = start(app); });
    ctx_.run_for(lapm::kGracefulStopTimeout + std::chrono::milliseconds(100));
    return ret;
}

ProcessInfo LinuxAppProcessManager::query(std::string_view id) const {
    std::shared_lock lk(mtx_);
    if (auto it = byId_.find(std::string(id)); it != byId_.end()) {
        return it->second.info;
    }
    return {};
}

std::vector<ProcessInfo> LinuxAppProcessManager::list() const {
    std::shared_lock lk(mtx_);
    std::vector<ProcessInfo> v;
    v.reserve(byId_.size());
    for (auto& [_, entry] : byId_) v.push_back(entry.info);
    return v;
}

void LinuxAppProcessManager::registerExitCallback(ExitCallback cb) {
    std::unique_lock lk(mtx_);
    callbacks_.push_back(std::move(cb));
}

void LinuxAppProcessManager::onChildExit(pid_t pid, int status) {
    std::unique_lock lk(mtx_);
    auto idIt = idByPid_.find(pid);
    if (idIt == idByPid_.end()) return;
    auto entryIt = byId_.find(idIt->second);
    if (entryIt == byId_.end()) return;
    auto& entry = entryIt->second;
    entry.info.state = ProcessState::Exited;
    if (WIFEXITED(status)) entry.info.exitCode = WEXITSTATUS(status);
    if (entry.killTimer) entry.killTimer->cancel();
    // release lock before callbacks
    auto infoCopy = entry.info;
    lk.unlock();
    for (auto& cb : callbacks_) cb(infoCopy);
    // clean maps
    std::unique_lock lk2(mtx_);
    idByPid_.erase(pid);
}

} // namespace lapm