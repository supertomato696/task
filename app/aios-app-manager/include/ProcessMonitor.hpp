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


/