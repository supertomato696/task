// =============================================================
//  Linux App Process Manager – consolidated headers/impl (C++20)
//  textdoc id: 6829d0dfd82c8191a878c14ded7e7350  (v5)
// =============================================================
//  Modules
//  ├─ Constants.hpp           – timeout & tuning knobs
//  ├─ ProcessTypes.hpp        – ProcessState / ProcessInfo
//  ├─ LinuxAppInfo.hpp        – App description (all std::string)
//  ├─ EnvManager.hpp          – Env parsing & LD_LIBRARY_PATH patch
//  ├─ Launcher.hpp            – ***updated full impl uses execvp***
//  ├─ ProcessMonitor.hpp      – SIGCHLD monitor
//  └─ LinuxAppProcessManager.hpp / .cpp – orchestration (onChildExit now per‑id)
// =============================================================
//  NOTE  This file is a single‑canvas demo – split into real files in repo.
// =============================================================

#ifndef LAPM_ALL_IN_ONE_HPP
#define LAPM_ALL_IN_ONE_HPP

#include <asio.hpp>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

// =============================================================
//  Constants.hpp
// =============================================================
inline constexpr std::chrono::milliseconds kGracefulStopTimeout{3000};

// =============================================================
//  ProcessTypes.hpp
// =============================================================

enum class ProcessState { Starting, Running, Stopping, Exited, Unknown };

struct ProcessInfo {
    pid_t                                   pid{0};
    ProcessState                            state{ProcessState::Unknown};
    std::string                             instanceId;
    std::string                             execPath;
    std::chrono::steady_clock::time_point   startTime;
    std::chrono::steady_clock::time_point   endTime;
    int                                     exitCode{0};
};

// =============================================================
//  LinuxAppInfo.hpp
// =============================================================

struct LinuxAppInfo {
    std::string  instanceId;        // 必填 – 唯一标识
    std::string  execPath;          // 必填 – 可执行文件
    std::vector<std::string> entrance; // 启动参数 (argv[1..])
    std::optional<int> nice;        // 进程优先级 (nice value)
    std::optional<std::string> envInline;   // "A=B,C=D" 形式
    std::optional<std::string> envFile;     // 环境变量文件
    bool autostart{false};
};

// =============================================================
//  EnvManager.hpp  (header‑only)
// =============================================================

#include <fstream>
#include <sstream>
#include <unordered_set>

class EnvManager {
public:
    static std::vector<std::string> buildEnvironment(const LinuxAppInfo& app) {
        // 1) collect parent env
        std::vector<std::string> envStrings;
        for (char **env = environ; *env; ++env) envStrings.emplace_back(*env);

        // 2) inline
        if (app.envInline && !app.envInline->empty()) {
            auto pairs = parseInline(*app.envInline);
            merge(envStrings, pairs);
        }
        // 3) file
        if (app.envFile && !app.envFile->empty()) {
            auto pairs = parseFile(*app.envFile);
            merge(envStrings, pairs);
        }
        // 4) patch LD_LIBRARY_PATH
        patchLdLibraryPath(envStrings, app.execPath);
        return envStrings;
    }
private:
    static std::vector<std::pair<std::string,std::string>> parseInline(const std::string& s) {
        std::vector<std::pair<std::string,std::string>> v;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss,item,',')) {
            auto eq = item.find('=');
            if (eq==std::string::npos) continue;
            v.emplace_back(trim(item.substr(0,eq)), trim(item.substr(eq+1)));
        }
        return v;
    }
    static std::vector<std::pair<std::string,std::string>> parseFile(const std::string& path) {
        std::vector<std::pair<std::string,std::string>> v;
        std::ifstream ifs(path);
        if (!ifs) return v;
        std::string line;
        while (std::getline(ifs,line)) {
            line = trim(line);
            if (line.empty() || line[0]=='#') continue;
            auto eq = line.find('=');
            if (eq==std::string::npos) continue;
            v.emplace_back(trim(line.substr(0,eq)), trim(line.substr(eq+1)));
        }
        return v;
    }
    static void merge(std::vector<std::string>& envVec, const std::vector<std::pair<std::string,std::string>>& pairs) {
        std::unordered_map<std::string,size_t> index; // key -> idx in envVec
        for (size_t i=0;i<envVec.size();++i) {
            auto eq = envVec[i].find('=');
            if (eq!=std::string::npos) index[envVec[i].substr(0,eq)] = i;
        }
        for (auto& [k,v] : pairs) {
            std::string kv = k + "=" + v;
            if (auto it=index.find(k); it!=index.end()) envVec[it->second] = kv;
            else envVec.push_back(std::move(kv));
        }
    }
    static void patchLdLibraryPath(std::vector<std::string>& envVec, const std::string& execPath) {
        std::string dir = std::filesystem::path(execPath).parent_path();
        std::string parent = std::filesystem::path(dir).parent_path();
        std::vector<std::string> extra = {dir, parent + "/lib", parent + "/lib64"};
        std::string current;
        bool found=false;
        for (auto& s : envVec) {
            if (s.rfind("LD_LIBRARY_PATH=",0)==0) { current = s.substr(16); found=true; break; }
        }
        std::unordered_set<std::string> parts;
        std::stringstream ss(current);
        std::string token;
        while (std::getline(ss, token, ':')) parts.insert(token);
        for (auto& e: extra) if (!e.empty()) parts.insert(e);
        std::string joined;
        for (auto it=parts.begin(); it!=parts.end(); ++it) {
            if (it!=parts.begin()) joined += ':';
            joined += *it;
        }
        std::string newVar = "LD_LIBRARY_PATH=" + joined;
        if (found) {
            for (auto& s: envVec) if (s.rfind("LD_LIBRARY_PATH=",0)==0) { s=newVar; break; }
        } else envVec.push_back(std::move(newVar));
    }
    static std::string trim(const std::string& s) {
        const char* ws=" \t\n\r";
        size_t b=s.find_first_not_of(ws);
        size_t e=s.find_last_not_of(ws);
        return (b==std::string::npos)?"":s.substr(b,e-b+1);
    }
};

// =============================================================
//  Launcher.hpp  (UPDATED IMPLEMENTATION)
// =============================================================

class Launcher {
public:
    /**
     * Forks and execs the given application, returns child pid in parent.
     * Throws std::system_error on failure.
     */
    static pid_t start(const LinuxAppInfo& app) {
        pid_t pid = ::fork();
        if (pid < 0) {
            throw std::system_error(errno, std::generic_category(), "fork failed");
        }
        if (pid == 0) {
            // ==== child process ====
            ::setsid();
            if (app.nice && *app.nice!=0) {
                ::setpriority(PRIO_PROCESS, 0, *app.nice);
            }
            // Build argv
            std::vector<std::string> argv_str;
            argv_str.push_back(app.execPath);
            argv_str.insert(argv_str.end(), app.entrance.begin(), app.entrance.end());
            std::vector<char*> argv;
            for (auto& s: argv_str) argv.push_back(const_cast<char*>(s.c_str()));
            argv.push_back(nullptr);

            // Build envp
            auto envStrings = EnvManager::buildEnvironment(app);
            std::vector<char*> envp;
            envp.reserve(envStrings.size()+1);
            for (auto& es : envStrings) envp.push_back(const_cast<char*>(es.c_str()));
            envp.push_back(nullptr);

            ::execvp(app.execPath.c_str(), argv.data());
            // if returns, exec failed
            ::perror("execvp failed");
            ::_exit(EXIT_FAILURE);
        }
        // ==== parent process ====
        // Quickly check if child already exited (double‑fork scenario off)
        int status;
        pid_t res = ::waitpid(pid, &status, WNOHANG);
        if (res == pid && (WIFEXITED(status) || WIFSIGNALED(status))) {
            throw std::runtime_error("child exited immediately after launch");
        }
        return pid;
    }
};

// =============================================================
//  ProcessMonitor.hpp
// =============================================================

class ProcessMonitor {
public:
    using ExitHandler = std::function<void(pid_t,int)>;
    ProcessMonitor(asio::io_context& ctx, ExitHandler cb)
        : signals_(ctx, SIGCHLD), cb_(std::move(cb)) {
        wait();
    }
private:
    void wait() {
        signals_.async_wait([this](const asio::error_code& ec, int){
            if (!ec) handle();
            wait();
        });
    }
    void handle() {
        while (true) {
            int status;
            pid_t pid = ::waitpid(-1,&status,WNOHANG);
            if (pid<=0) break;
            cb_(pid,status);
        }
    }
    asio::signal_set signals_;
    ExitHandler cb_;
};

// =============================================================
//  LinuxAppProcessManager.hpp / .cpp (header‑only for brevity)
// =============================================================

class LinuxAppProcessManager {
public:
    using ExitCallback = std::function<void(const ProcessInfo&)>;

    explicit LinuxAppProcessManager(asio::io_context& ctx)
        : ctx_(ctx), monitor_(ctx, [this](pid_t p,int s){ onChildExit(p,s);} ) {}

    ProcessInfo start(const LinuxAppInfo& app) {
        auto pid = Launcher::start(app);
        ProcessInfo info;
        info.pid = pid;
        info.instanceId = app.instanceId;
        info.execPath = app.execPath;
        info.startTime = std::chrono::steady_clock::now();
        info.state = ProcessState::Running;
        {
            std::unique_lock lk(mtx_);
            table_[app.instanceId] = {info,nullptr};
            idByPid_[pid] = app.instanceId;
        }
        return info;
    }

    void stop(const std::string& id) { stopImpl(id); }
    void stop(const LinuxAppInfo& app) { stopImpl(app.instanceId); }

    ProcessInfo query(const std::string& id) const {
        std::shared_lock lk(mtx_);
        if (auto it=table_.find(id); it!=table_.end()) return it->second.info;
        return {};
    }

    std::vector<ProcessInfo> list() const {
        std::shared_lock lk(mtx_);
        std::vector<ProcessInfo> v;
        v.reserve(table_.size());
        for (auto& [k,e]:table_) v.push_back(e.info);
        return v;
    }

    void registerExitCallback(std::string_view id, ExitCallback cb) {
        std::unique_lock lk(mtx_);
        perIdCbs_[std::string(id)].push_back(std::move(cb));
    }

private:
    struct Entry {
        ProcessInfo info;
        std::shared_ptr<asio::steady_timer> killTimer;
    };

    void stopImpl(const std::string& id) {
        std::unique_lock lk(mtx_);
        auto it = table_.find(id);
        if (it==table_.end()) return;
        pid_t pid = it->second.info.pid;
        auto timer = std::make_shared<asio::steady_timer>(ctx_);
        it->second.killTimer = timer;
        lk.unlock();
        ::kill(pid,SIGTERM);
        timer->expires_after(kGracefulStopTimeout);
        timer->async_wait([pid](const std::error_code& ec){
            if (!ec && ::kill(pid,0)==0) ::kill(pid,SIGKILL);
        });
    }

    void onChildExit(pid_t pid,int status) {
        std::vector<ExitCallback> cbs;
        ProcessInfo infoCopy;
        {
            std::unique_lock lk(mtx_);
            auto idIt = idByPid_.find(pid);
            if (idIt==idByPid_.end()) return;
            auto eIt = table_.find(idIt->second);
            if (eIt==table_.end()) return;
            auto& entry = eIt->second;
            entry.info.state = ProcessState::Exited;
            entry.info.endTime = std::chrono::steady_clock::now();
            if (WIFEXITED(status)) entry.info.exitCode = WEXITSTATUS(status);
            if (entry.killTimer) entry.killTimer->cancel();
            infoCopy = entry.info;
            cbs = perIdCbs_[idIt->second]; // copy relevant callbacks only
            table_.erase(eIt);
            idByPid_.erase(idIt);
        }
        for (auto& cb: cbs) cb(infoCopy);
    }

    asio::io_context& ctx_;
    ProcessMonitor monitor_;

    mutable std::shared_mutex mtx_;
    std::map<std::string, Entry> table_;      // id -> entry
    std::unordered_map<pid_t,std::string> idByPid_;
    std::map<std::string,std::vector<ExitCallback>> perIdCbs_; // id‑scoped callbacks
};

#endif // LAPM_ALL_IN_ONE_HPP
