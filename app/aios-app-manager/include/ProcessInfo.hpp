#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <unistd.h>     // getpid, getppid, getcwd, readlink, getuid, getgid, gethostname
#include <pwd.h>        // getpwuid
#include <grp.h>        // getgrgid
#include <sys/stat.h>   // umask
#include <sys/resource.h> // getrusage
#include <climits>
#include <limits.h>
#include <sstream>
#include <fstream>
#include <cstring>
#include <chrono>
#include <ctime>

extern char **environ;

class ProcessInfo {
public:
    ProcessInfo(int argc, char* argv[]) {
        // 命令行参数
        args_.reserve(argc);
        for (int i = 0; i < argc; ++i)
            args_.emplace_back(argv[i]);

        // 环境变量
        for (char **env = environ; env && *env; ++env)
            envs_.emplace_back(*env);

        // PID & PPID
        pid_ = static_cast<int>(::getpid());
        ppid_ = static_cast<int>(::getppid());

        // 可执行文件路径
        char buf[PATH_MAX]{};
        ssize_t len = ::readlink("/proc/self/exe", buf, sizeof(buf)-1);
        execPath_ = (len > 0) ? std::string(buf, len) : (args_.empty() ? "" : args_[0]);

        // 当前工作目录
        char cwd_buf[PATH_MAX]{};
        cwd_ = ::getcwd(cwd_buf, sizeof(cwd_buf)) ? cwd_buf : "";

        // 主机名
        char hn_buf[HOST_NAME_MAX]{};
        hostname_ = (::gethostname(hn_buf, sizeof(hn_buf)) == 0) ? hn_buf : "";

        // 用户、组信息
        uid_ = ::getuid();
        gid_ = ::getgid();
        struct passwd* pwd = ::getpwuid(uid_);
        username_ = pwd ? pwd->pw_name : "";
        struct group* grp = ::getgrgid(gid_);
        groupname_ = grp ? grp->gr_name : "";

        // 父进程名
        parentName_ = getProcessName(ppid_);

        // umask
        mode_t old_mask = ::umask(0);
        ::umask(old_mask); // restore
        umask_ = old_mask;

        // 命令行完整字符串
        std::ifstream cmdline("/proc/self/cmdline");
        std::string raw;
        std::getline(cmdline, raw, '\0');
        std::ostringstream oss;
        for (const auto& arg : args_) {
            oss << arg << " ";
        }
        cmdlineStr_ = oss.str();

        // 进程状态
        std::ifstream statf("/proc/self/stat");
        if (statf) {
            statf >> statPid_ >> statComm_ >> statState_;
        }

        // 内存、CPU 占用
        struct rusage ru{};
        if (::getrusage(RUSAGE_SELF, &ru) == 0) {
            maxRssKb_ = ru.ru_maxrss; // KB
            userTime_ = static_cast<double>(ru.ru_utime.tv_sec) + ru.ru_utime.tv_usec / 1e6;
            sysTime_  = static_cast<double>(ru.ru_stime.tv_sec) + ru.ru_stime.tv_usec / 1e6;
        }

        // 启动时间（从/proc/self/stat获取starttime，然后和/proc/stat的btime换算）
        procStartTime_ = getProcStartTime();
    }

    // 各类打印
    void printProcessInfo(std::ostream& os = std::cout) const {
        os << "[Process Info]\n"
           << "  PID           : " << pid_ << "\n"
           << "  PPID          : " << ppid_ << " (" << parentName_ << ")\n"
           << "  Executable    : " << execPath_ << "\n"
           << "  CWD           : " << cwd_ << "\n"
           << "  Hostname      : " << hostname_ << "\n"
           << "  User          : " << username_ << " (uid=" << uid_ << ")\n"
           << "  Group         : " << groupname_ << " (gid=" << gid_ << ")\n"
           << "  Umask         : " << std::oct << umask_ << std::dec << "\n"
           << "  Cmdline       : " << cmdlineStr_ << "\n"
           << "  Status        : " << statState_ << "\n"
           << "  MaxRSS        : " << maxRssKb_ << " KB\n"
           << "  UserTime      : " << userTime_ << " s\n"
           << "  SysTime       : " << sysTime_ << " s\n";
        if (!procStartTime_.empty())
            os << "  StartTime     : " << procStartTime_ << "\n";
    }

    void printArgs(std::ostream& os = std::cout) const {
        os << "[Arguments] (" << args_.size() << ")\n";
        for (size_t i = 0; i < args_.size(); ++i)
            os << "  argv[" << i << "]: " << args_[i] << '\n';
    }

    void printEnv(std::ostream& os = std::cout) const {
        os << "[Environment Variables] (" << envs_.size() << ")\n";
        for (const auto& e : envs_)
            os << "  " << e << '\n';
    }

    void printAll(std::ostream& os = std::cout) const {
        printProcessInfo(os);
        printArgs(os);
        printEnv(os);
    }

    // 可扩展访问接口
    const std::vector<std::string>& args() const { return args_; }
    const std::vector<std::string>& envs() const { return envs_; }
    int pid()   const { return pid_; }
    int ppid()  const { return ppid_; }
    const std::string& execPath()   const { return execPath_; }
    const std::string& cwd()        const { return cwd_; }
    const std::string& hostname()   const { return hostname_; }
    uid_t uid()     const { return uid_; }
    gid_t gid()     const { return gid_; }
    const std::string& username()   const { return username_; }
    const std::string& groupname()  const { return groupname_; }
    mode_t umaskVal()const { return umask_; }
    const std::string& cmdlineStr() const { return cmdlineStr_; }
    char   state()   const { return statState_; }
    long   maxRssKb()const { return maxRssKb_; }
    double userTime()const { return userTime_; }
    double sysTime() const { return sysTime_; }
    const std::string& procStartTime() const { return procStartTime_; }
    const std::string& parentName() const { return parentName_; }

private:
    std::vector<std::string> args_;
    std::vector<std::string> envs_;
    int pid_{};
    int ppid_{};
    std::string execPath_;
    std::string cwd_;
    std::string hostname_;
    uid_t uid_{};
    gid_t gid_{};
    std::string username_;
    std::string groupname_;
    mode_t umask_{};
    std::string cmdlineStr_;
    int statPid_{};
    std::string statComm_;
    char statState_{};
    long maxRssKb_{};
    double userTime_{};
    double sysTime_{};
    std::string procStartTime_;
    std::string parentName_;

    // 通过 pid 获取进程名（仅限 Linux/proc）
    static std::string getProcessName(int pid) {
        std::ifstream statf("/proc/" + std::to_string(pid) + "/comm");
        std::string name;
        std::getline(statf, name);
        return name;
    }

    // 获取进程启动时间
    static std::string getProcStartTime() {
        // 读取 starttime (clock ticks since boot)
        std::ifstream stat("/proc/self/stat");
        std::string dummy;
        long long starttime = 0;
        for (int i = 0; i < 21 && stat; ++i)
            stat >> dummy;
        stat >> starttime;

        // 读取系统启动时间
        std::ifstream statfile("/proc/stat");
        std::string label;
        long long btime = 0;
        while (statfile >> label) {
            if (label == "btime") {
                statfile >> btime;
                break;
            }
        }

        long ticksPerSec = sysconf(_SC_CLK_TCK);
        long long absStart = btime + starttime / ticksPerSec;
        if (absStart == 0) return {};
        std::time_t tt = absStart;
        char buf[64]{};
        if (std::strftime(buf, sizeof(buf), "%F %T", std::localtime(&tt)))
            return buf;
        return {};
    }
};

