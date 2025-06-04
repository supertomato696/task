// =============================================================
//  Launcher.hpp  (header-only, C++20)
// =============================================================
#pragma once

#include <fcntl.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <iostream>
#include <cerrno>
#include <cstring>

#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "EnvManager.hpp"      // 依赖你的 header-only 环境变量工具
#include "LinuxAppInfor.hpp"    // execPath / entrance / nice ...

/**
 * 负责 fork+exec 启动进程，仅做“启动”这一件事。
 * 失败时始终抛异常；成功返回子进程 pid。
 *
 * *可扩展点*：
 *   • workingDir / stdio 重定向 / cgroup 设置
 *   • seccomp / prctl / capabilities
 */
class Launcher {
private:
    inline static  const std::string TAG { "Launcher" };
public:
    [[nodiscard]] static pid_t start(const LinuxAppInfo& app) {
        // --------------------------------------------------
        // 1) pipe 用于把 execve 的 errno 传回父进程
        // --------------------------------------------------
        int errPipe[2];
#if defined(O_CLOEXEC)
        if (::pipe2(errPipe, O_CLOEXEC) == -1) {
            throw std::system_error(errno, std::generic_category(), "pipe2 failed");
        }
#else
        if (::pipe(errPipe) == -1) {
            throw std::system_error(errno, std::generic_category(), "pipe failed");
        }
        ::fcntl(errPipe[0], F_SETFD, FD_CLOEXEC);
        ::fcntl(errPipe[1], F_SETFD, FD_CLOEXEC);
#endif

        pid_t pid = ::fork();
        if (pid < 0) {
            ::close(errPipe[0]);
            ::close(errPipe[1]);
            throw std::system_error(errno, std::generic_category(), "fork failed");
        }

        if (pid == 0) {
            // ========================== CHILD ==========================
            ::close(errPipe[0]);               // child 用不到读端

            // 会话 & nice
            ::setsid();
            if (app.priority && *app.priority != 0) {
                ::setpriority(PRIO_PROCESS, 0, *app.priority);
            }

            // -------- argv ----------
            std::vector<std::string> argvStore;
            argvStore.reserve(app.entrance.size() + 1);
            argvStore.push_back(app.execPath);             // argv[0]
            argvStore.insert(argvStore.end(),
                             app.entrance.begin(),
                             app.entrance.end());

            std::vector<char*> argv;
            argv.reserve(argvStore.size() + 1);
            for (auto& s : argvStore) argv.push_back(const_cast<char*>(s.c_str()));
            argv.push_back(nullptr);

            // -------- envp ----------
            auto envStore = EnvManager::buildEnvironment(app);
            std::vector<char*> envp;
            envp.reserve(envStore.size() + 1);
            for (auto& s : envStore) envp.push_back(const_cast<char*>(s.c_str()));
            envp.push_back(nullptr);

            // -------- exec ----------
            ::execve(app.execPath.c_str(), argv.data(), envp.data());
            std::cout << TAG << ": execve failed " << strerror(errno)<< std::endl;
            // 执行到此说明 exec 失败 → 将 errno 写到管道后退出
            int err = errno;
            ::write(errPipe[1], &err, sizeof(err));
            ::_exit(127);     // 不返回
        }

        // ========================== PARENT ==========================
        ::close(errPipe[1]);            // 仅读
        int childErr = 0;
        ssize_t n = ::read(errPipe[0], &childErr, sizeof(childErr));
        ::close(errPipe[0]);

        if (n > 0) {
            // 子进程 exec 失败；回收其退出码
            int status = 0;
            ::waitpid(pid, &status, 0);

            throw std::system_error(childErr,
                                    std::generic_category(),
                                    "execve failed in child");
        }

        // “秒退”检测：若 fork 后子进程很快退出，也通过 waitpid 捕获
        int status = 0;
        pid_t res  = ::waitpid(pid, &status, WNOHANG);
        if (res == pid && (WIFEXITED(status) || WIFSIGNALED(status))) {
            throw std::runtime_error("child terminated immediately after launch");
        }

        return pid;
    }

private:
    Launcher() = delete;   // 不可实例化
};
