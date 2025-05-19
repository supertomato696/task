// =============================================================
//  ProcessMonitor.hpp   (header-only, C++20)
//  依赖 header-only Asio   https://think-async.com
// =============================================================
#pragma once
#include <asio/io_context.hpp>
#include <asio/signal_set.hpp>

#include <sys/wait.h>
#include <unistd.h>

#include <functional>
#include <system_error>
#include <vector>

/**
 * ProcessMonitor
 * ---------------
 * 只负责捕获 SIGCHLD 并调用用户回调。
 *
 * 用法示例：
 *   asio::io_context io;
 *   ProcessMonitor mon{io, [&](pid_t pid, int status){
 *         bool exited = WIFEXITED(status);
 *         int  code   = exited ? WEXITSTATUS(status) : -1;
 *   }};
 */
class ProcessMonitor {
public:
    using ExitHandler = std::function<void(pid_t /*pid*/, int /*raw status*/)>;

    /** 构造后立即开始监听 SIGCHLD */
    ProcessMonitor(asio::io_context& ctx, ExitHandler cb)
        : ctx_(ctx), signals_(ctx, SIGCHLD), onExit_(std::move(cb)) {
        startWait();
    }

    /** 禁止拷贝，允许移动 */
    ProcessMonitor(const ProcessMonitor&)            = delete;
    ProcessMonitor& operator=(const ProcessMonitor&) = delete;
    ProcessMonitor(ProcessMonitor&&)                 = default;
    ProcessMonitor& operator=(ProcessMonitor&&)      = default;

private:
    void startWait() {
        signals_.async_wait(
            [this](const std::error_code& ec, int /*signal_number*/) {
                if (ec) return;           // 取消或 io_context 已停止
                reapChildren();
                startWait();              // 继续等待下一次 SIGCHLD
            });
    }

    void reapChildren() {
        while (true) {
            int   status = 0;
            pid_t pid    = ::waitpid(-1, &status, WNOHANG);
            if (pid > 0) {
                // 通知上层
                if (onExit_) onExit_(pid, status);
            } else if (pid == 0) {
                // 没有更多可收割
                break;
            } else {
                // pid < 0
                if (errno == ECHILD) break;   // 没子进程
                // 其他错误：短暂忽略，避免 busy loop
                break;
            }
        }
    }

    // ------------------------------------------------------------------
    asio::io_context& ctx_;
    asio::signal_set  signals_;
    ExitHandler       onExit_;
};
