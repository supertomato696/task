// =============================================================
//  Modular Linux App Process Manager – code skeleton  (C++20)
// =============================================================
//  Files in this single‑canvas demo (header‑only stubs except where noted):
//    • ProcessTypes.hpp      – ProcessState / ProcessInfo
//    • LinuxAppInfo.hpp      – user‑supplied process description
//    • EnvManager.hpp        – ***IMPLEMENTED IN THIS REVISION***
//    • Launcher.hpp/.cpp     – fork/exec wrapper  (TODO)
//    • ProcessMonitor.hpp    – SIGCHLD watcher   (TODO)
//    • LinuxAppProcessManager.hpp/.cpp – facade (TODO)
// -------------------------------------------------------------
//  NOTE: For brevity, only EnvManager.hpp gains full code here; other
//  headers keep skeleton placeholders awaiting further iterations.
// =============================================================

#ifndef LINUX_APP_PROCESS_MANAGER_MODULES_HPP
#define LINUX_APP_PROCESS_MANAGER_MODULES_HPP

#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

//--------------------------------------------------------------
//  ProcessTypes.hpp
//--------------------------------------------------------------

enum class ProcessState {
    Starting,
    Running,
    Stopping,
    Exited,
    Unknown
};

struct ProcessInfo {
    pid_t                                    pid{};
    ProcessState                             state{ProcessState::Starting};
    std::string                              instanceId;
    std::filesystem::path                    execPath;
    std::chrono::steady_clock::time_point    startTime;
    int                                      exitCode{0};
};

//--------------------------------------------------------------
//  LinuxAppInfo.hpp
//--------------------------------------------------------------

struct LinuxAppInfo {
    std::string                       instanceId;   // unique name per running instance
    std::filesystem::path             execPath;     // absolute / relative path
    std::vector<std::string>          entrance;     // argv[1..N] (argv[0] implied)
    std::optional<int>                priority;     // nice value
    std::optional<std::string>        envInline;    // KEY=VAL,KEY=VAL...
    std::optional<std::filesystem::path> envFile;   // file containing KEY=VAL per line
    bool                              autostart{false};
};

//--------------------------------------------------------------
//  EnvManager.hpp  (header‑only implementation)   ★ FULL CODE ★
//--------------------------------------------------------------

/**
 * EnvManager — constructs the `envp` array for exec‑style launching.
 *   • Merges current process environment with per‑app overrides
 *   • Supports inline KEY=VAL list and env‑file (one per line, # comment)
 *   • Auto‑patches LD_LIBRARY_PATH with executable dir, its parent/lib{,64}
 *   • Inline list has highest priority > file > existing env
 */
class EnvManager {
public:
    /**
     * Build a null‑terminated vector of char* suitable for execve.
     * Lifetime: strings stored internally; keep the vector in scope until exec.
     */
    static std::vector<std::string> buildEnvStrings(const LinuxAppInfo& app) {
        using Map = std::unordered_map<std::string, std::string>;

        Map env = captureCurrentEnv();

        // Merge envFile first (lowest override among user sources)
        if (app.envFile)
            merge(env, parseEnvFile(*app.envFile));

        // Merge inline string (highest override)
        if (app.envInline)
            merge(env, parseInline(*app.envInline));

        // Patch LD_LIBRARY_PATH
        patchLdLibraryPath(env, app.execPath);

        // Emit vector<string> "KEY=VAL"
        std::vector<std::string> out;
        out.reserve(env.size());
        for (auto& [k, v] : env)
            out.emplace_back(k + '=' + v);
        return out;
    }

private:
    //------------ helpers -------------
    static std::unordered_map<std::string, std::string> captureCurrentEnv() {
        std::unordered_map<std::string, std::string> out;
#ifdef __linux__
        extern char** environ;
        for (char** p = environ; p && *p; ++p) {
            std::string_view entry(*p);
            auto pos = entry.find('=');
            if (pos == std::string_view::npos) continue;
            out.emplace(entry.substr(0, pos), entry.substr(pos + 1));
        }
#endif
        return out;
    }

    static std::unordered_map<std::string, std::string> parseInline(std::string_view csv) {
        std::unordered_map<std::string, std::string> out;
        size_t start = 0;
        while (start < csv.size()) {
            auto end = csv.find(',', start);
            if (end == std::string_view::npos) end = csv.size();
            auto token = csv.substr(start, end - start);
            auto pos = token.find('=');
            if (pos != std::string_view::npos) {
                out.emplace(std::string(token.substr(0, pos)), std::string(token.substr(pos + 1)));
            }
            start = end + 1;
        }
        return out;
    }

    static std::unordered_map<std::string, std::string> parseEnvFile(const std::filesystem::path& file) {
        std::unordered_map<std::string, std::string> out;
        std::ifstream fin(file);
        std::string line;
        while (std::getline(fin, line)) {
            if (line.empty() || line[0] == '#') continue;
            auto pos = line.find('=');
            if (pos == std::string::npos) continue;
            out.emplace(line.substr(0, pos), line.substr(pos + 1));
        }
        return out;
    }

    static void merge(std::unordered_map<std::string, std::string>& base,
                      const std::unordered_map<std::string, std::string>& other) {
        for (auto& [k, v] : other) base[k] = v; // override or insert
    }

    static void patchLdLibraryPath(std::unordered_map<std::string, std::string>& env,
                                   const std::filesystem::path& execPath) {
        std::vector<std::filesystem::path> extra;
        auto binDir = execPath.parent_path();
        if (!binDir.empty()) {
            extra.push_back(binDir);
            auto parent = binDir.parent_path();
            if (!parent.empty()) {
                extra.push_back(parent / "lib");
                extra.push_back(parent / "lib64");
            }
        }
        std::string merged;
        if (auto it = env.find("LD_LIBRARY_PATH"); it != env.end() && !it->second.empty()) {
            merged = it->second;
        }
        for (auto& p : extra) {
            if (!std::filesystem::exists(p)) continue;
            std::string s = p.string();
            if (merged.find(s) == std::string::npos) {
                if (!merged.empty()) merged.append(1, ':');
                merged += s;
            }
        }
        if (!merged.empty()) env["LD_LIBRARY_PATH"] = merged;
    }
};

//--------------------------------------------------------------
//  Launcher.hpp  (stub, will call EnvManager::buildEnvStrings) 
//--------------------------------------------------------------
struct Launcher {
    // TODO: implement fork+exec using EnvManager
};

//--------------------------------------------------------------
//  ProcessMonitor.hpp  (stub)
//--------------------------------------------------------------
struct ProcessMonitor {
    // TODO: implement SIGCHLD route & table maintenance
};

//--------------------------------------------------------------
//  LinuxAppProcessManager (facade)  (stub)
//--------------------------------------------------------------
class LinuxAppProcessManager {
public:
    explicit LinuxAppProcessManager() = default; // ctx omitted for stub
private:
    // TODO: hold Launcher + ProcessMonitor
};

#endif // LINUX_APP_PROCESS_MANAGER_MODULES_HPP










// =============================================================
//  Linux App Process Manager – Modular Skeleton (C++20)
//  textdoc id: 6829d0dfd82c8191a878c14ded7e7350   (v3)
// =============================================================
//  This single canvas bundles header‑only stubs & minimal impl:
//    • Constants.hpp           – tuning knobs
//    • ProcessTypes.hpp        – enum & struct for runtime info
//    • LinuxAppInfo.hpp        – user‑supplied app description
//    • EnvManager.hpp          – env string/file + LD_LIBRARY_PATH
//    • Launcher.hpp            – ***NEW full implementation***
//    • LinuxAppProcessManager.hpp/.cpp – façade + async stop
// -------------------------------------------------------------
//  Note: For clarity all code is header‑only or inline‑impl; split
//  into .cpp files in real repo.
// =============================================================

/* =====================  Constants.hpp  ===================== */
#pragma once
#include <chrono>
namespace lapm {
constexpr std::chrono::milliseconds kGracefulStopTimeout{3000}; // 3 s
} // namespace lapm

/* ===================  ProcessTypes.hpp  ==================== */
#pragma once
#include <chrono>
#include <filesystem>
#include <string>
#ifdef __linux__
#include <sys/types.h>
#endif
namespace lapm {

enum class ProcessState {
    Starting,
    Running,
    Stopping,
    Exited,
    Unknown
};

struct ProcessInfo {
    pid_t                                 pid{-1};
    ProcessState                          state{ProcessState::Unknown};
    std::string                           instanceId;
    std::filesystem::path                 execPath;
    std::chrono::steady_clock::time_point startTime{};
    int                                   exitCode{0}; // valid if Exited
};

} // namespace lapm

/* ===================  LinuxAppInfo.hpp  ==================== */
#pragma once
#include <filesystem>
#include <optional>
#include <string>
#include <vector>
namespace lapm {
struct LinuxAppInfo {
    std::string                       instanceId;   // formerly pkgName
    std::filesystem::path             execPath;
    std::vector<std::string>          entrance;     // argv[1..]
    std::optional<int>                priority;     // nice value
    std::optional<std::string>        envInline;    // KEY=V,KEY2=V2
    std::optional<std::filesystem::path> envFile;   // file with KEY=V
    bool                              autostart{false};
};
} // namespace lapm

/* ===================  EnvManager.hpp  ====================== */
#pragma once
#include "LinuxAppInfo.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unordered_set>
namespace lapm {
class EnvManager {
public:
    static std::vector<std::string> buildEnvironment(const LinuxAppInfo& app) {
        // 1. collect parent env
        std::vector<std::string> envOut;
        for (char** p = environ; *p; ++p) envOut.emplace_back(*p);

        // 2. apply inline
        if (app.envInline && !app.envInline->empty()) {
            auto inlines = parseInline(*app.envInline);
            merge(envOut, inlines);
        }
        // 3. apply file
        if (app.envFile) {
            auto fileEnv = parseFile(*app.envFile);
            merge(envOut, fileEnv);
        }
        // 4. patch LD_LIBRARY_PATH
        patchLdLibraryPath(envOut, app.execPath);
        return envOut;
    }

private:
    static std::vector<std::string> parseInline(const std::string& str) {
        std::vector<std::string> out;
        std::stringstream ss(str);
        std::string item;
        while (std::getline(ss, item, ',')) {
            trim(item);
            if (!item.empty()) out.push_back(item);
        }
        return out;
    }
    static std::vector<std::string> parseFile(const std::filesystem::path& path) {
        std::vector<std::string> out;
        std::ifstream fin(path);
        std::string line;
        while (std::getline(fin, line)) {
            trim(line);
            if (line.empty() || line[0] == '#') continue;
            out.push_back(line);
        }
        return out;
    }
    static void merge(std::vector<std::string>& base, const std::vector<std::string>& override) {
        std::unordered_map<std::string, size_t> keyToIndex;
        for (size_t i = 0; i < base.size(); ++i) {
            auto k = base[i].substr(0, base[i].find('='));
            keyToIndex[k] = i;
        }
        for (const auto& kv : override) {
            auto k = kv.substr(0, kv.find('='));
            auto it = keyToIndex.find(k);
            if (it != keyToIndex.end()) base[it->second] = kv;
            else base.push_back(kv);
        }
    }
    static void patchLdLibraryPath(std::vector<std::string>& env, const std::filesystem::path& exec) {
        std::string key = "LD_LIBRARY_PATH";
        std::string current;
        bool found = false;
        for (auto& kv : env) {
            if (kv.rfind(key + "=", 0) == 0) { current = kv.substr(key.size()+1); found = true; break; }
        }
        std::unordered_set<std::string> dirs;
        auto addDir = [&](const std::filesystem::path& p){ if(!p.empty()) dirs.insert(p.string()); };
        addDir(exec.parent_path());
        addDir(exec.parent_path()/"lib");
        addDir(exec.parent_path()/"lib64");
        addDir(exec.parent_path().parent_path()/"lib");
        addDir(exec.parent_path().parent_path()/"lib64");
        // merge existing
        std::stringstream ss(current);
        std::string tok;
        while (std::getline(ss, tok, ':')) if (!tok.empty()) dirs.insert(tok);
        // rebuild
        std::string merged;
        for (auto it=dirs.begin(); it!=dirs.end(); ++it) {
            if (it!=dirs.begin()) merged += ':';
            merged += *it;
        }
        std::string newEntry = key + '=' + merged;
        if (found) {
            for (auto& kv : env) if (kv.rfind(key + "=", 0) == 0) { kv = newEntry; break; }
        } else {
            env.emplace_back(std::move(newEntry));
        }
    }
    static void trim(std::string& s) {
        auto notSpace = [](int ch){ return !std::isspace(ch); };
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
        s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    }
};
} // namespace lapm

/* =====================  Launcher.hpp  ====================== */
#pragma once
#include "LinuxAppInfo.hpp"
#include "EnvManager.hpp"
#include "ProcessTypes.hpp"
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstring>

namespace lapm {
class Launcher {
public:
    static pid_t start(const LinuxAppInfo& app) {
        // argv strings
        std::vector<std::string> argvStrs;
        argvStrs.reserve(app.entrance.size()+1);
        argvStrs.push_back(app.execPath.string());
        argvStrs.insert(argvStrs.end(), app.entrance.begin(), app.entrance.end());
        std::vector<char*> argv;
        for (auto& s : argvStrs) argv.push_back(s.data());
        argv.push_back(nullptr);

        // envp strings
        auto envStrs = EnvManager::buildEnvironment(app);
        std::vector<char*> envp;
        for (auto& s : envStrs) envp.push_back(s.data());
        envp.push_back(nullptr);

        pid_t pid = ::fork();
        if (pid == -1) throw std::runtime_error(std::string("fork failed: ") + std::strerror(errno));
        if (pid == 0) {
            // child
            if (app.priority) ::setpriority(PRIO_PROCESS, 0, *app.priority);
            ::execve(app.execPath.c_str(), argv.data(), envp.data());
            std::perror("execve");
            _exit(127);
        }
        return pid; // parent
    }
};
} // namespace lapm

/* ============  LinuxAppProcessManager.hpp/.cpp  ============ */
#pragma once
#include "Constants.hpp"
#include "Launcher.hpp"
#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>
#include <asio/signal_set.hpp>
#include <map>
#include <shared_mutex>
#include <unordered_map>

namespace lapm {
class LinuxAppProcessManager {
public:
    using ExitCallback = std::function<void(const ProcessInfo&)>;
    explicit LinuxAppProcessManager(asio::io_context& ctx) : ctx_(ctx), signals_(ctx_, SIGCHLD) {
        waitForChild();
    }

    // start ---------------------------------------------------
    [[nodiscard]] ProcessInfo start(const LinuxAppInfo& app) {
        auto pid = Launcher::start(app);
        ProcessInfo info;
        info.pid        = pid;
        info.state      = ProcessState::Running;
        info.instanceId = app.instanceId;
        info.execPath   = app.execPath;
        info.startTime  = std::chrono::steady_clock::now();
        {
            std::unique_lock lock(mtx_);
            processes_[info.instanceId] = info;
        }
        return info;
    }

    // stop (string_view) – async graceful‑force --------------
    void stop(std::string_view instanceId) { doStop(std::string(instanceId)); }
    void stop(const LinuxAppInfo& app)     { doStop(app.instanceId); }

    // query ---------------------------------------------------
    [[nodiscard]] ProcessInfo query(std::string_view id) const {
        std::shared_lock lock(mtx_);
        if (auto it = processes_.find(std::string(id)); it != processes_.end()) return it->second;
        return {};
    }
    [[nodiscard]] ProcessInfo query(const LinuxAppInfo& app) const { return query(app.instanceId); }
    [[nodiscard]] std::vector<ProcessInfo> list() const {
        std::vector<ProcessInfo> out;
        std::shared_lock lock(mtx_);
        for (auto& [_, p] : processes_) out.push_back(p);
        return out;
    }

    // callbacks ---------------------------------------------
    void registerExitCallback(ExitCallback cb) {
        std::unique_lock lock(cbMtx_);
        callbacks_.push_back(std::move(cb));
    }

private:
    // ------------------- impl data --------------------------
    asio::io_context&                   ctx_;
    asio::signal_set                    signals_;
    mutable std::shared_mutex           mtx_;
    std::map<std::string, ProcessInfo>  processes_;
    std::unordered_map<pid_t, std::unique_ptr<asio::steady_timer>> killTimers_;

    std::mutex                          cbMtx_;
    std::vector<ExitCallback>           callbacks_;

    // ----------------- child exit handling ------------------
    void waitForChild() {
        signals_.async_wait([this](auto ec, int){
            if (!ec) reapChildren();
            waitForChild();
        });
    }
    void reapChildren() {
        while (true) {
            int   status;
            pid_t pid = ::waitpid(-1, &status, WNOHANG);
            if (pid <= 0) break;
            ProcessInfo info;
            {
                std::unique_lock lock(mtx_);
                for (auto it = processes_.begin(); it != processes_.end(); ++it) {
                    if (it->second.pid == pid) { info = it->second; processes_.erase(it); break; }
                }
            }
            info.state = ProcessState::Exited;
            info.exitCode = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            // cancel timer if any
            killTimers_.erase(pid);
            // broadcast
            std::vector<ExitCallback> cbs;
            {
                std::scoped_lock lk(cbMtx_);
                cbs = callbacks_;
            }
            for (auto& cb : cbs) cb(info);
        }
    }

    // ---------------- graceful→force stop -------------------
    void doStop(const std::string& id) {
        ProcessInfo pi;
        {
            std::shared_lock lock(mtx_);
            auto it = processes_.find(id);
            if (it == processes_.end()) return;
            pi = it->second;
        }
        if (pi.state != ProcessState::Running) return;
        ::kill(pi.pid, SIGTERM);
        auto timer = std::make_unique<asio::steady_timer>(ctx_, kGracefulStopTimeout);
        pid_t pid = pi.pid;
        timer->async_wait([this, pid](auto ec){
            if (ec == asio::error::operation_aborted) return; // cancelled (process exited)
            // still active?
            if (::kill(pid, 0) == 0) {
                ::kill(pid, SIGKILL);
            }
        });
        killTimers_[pid] = std::move(timer);
    }
};
} // namespace lapm
