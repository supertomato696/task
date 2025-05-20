#include "LinuxAppProcessManager.hpp"
#include "LinuxAppInfor.hpp"
#include "ProcessTypes.hpp"
#include "Laucher.hpp"

#include <asio/post.hpp>
#include <csignal>
#include <sys/wait.h>

#include <iostream>

// =============================================================
//  LinuxAppProcessManager.cpp
// =============================================================
#include "LinuxAppProcessManager.hpp"

#include <asio/post.hpp>
#include <csignal>
#include <utility>

using namespace std::chrono;

// -------------------------------------------------------------
//  ctor
// -------------------------------------------------------------
LinuxAppProcessManager::LinuxAppProcessManager(asio::io_context& ctx)
    : io_(ctx)
    , monitor_(ctx, [this](pid_t p, int st) { onChildExit(p, st); })
{}

// -------------------------------------------------------------
//  start
// -------------------------------------------------------------
ProcessInfo LinuxAppProcessManager::start(const LinuxAppInfo& app)
{
    ProcessInfo pi;
    pi.instanceId = app.instanceId;
    pi.execPath   = app.execPath;
    pi.state      = ProcessState::Starting;
    pi.startTime  = steady_clock::now();

    try {
        pid_t pid = Launcher::start(app);         // 可能抛异常
        pi.pid   = pid;
        pi.state = ProcessState::Running;

        std::unique_lock lk(mu_);
        table_[app.instanceId] = TimedEntry{pi, nullptr};
        pid2id_[pid]           = app.instanceId;
    }
    catch (const std::system_error& se) {
        pi.state    = ProcessState::Failed;
        pi.exitCode = -se.code().value();         // 负 errno
    }
    catch (const std::exception&) {
        pi.state    = ProcessState::Failed;
        pi.exitCode = -1;
    }
    return pi;
}

// -------------------------------------------------------------
//  stop (SIGTERM → timeout → SIGKILL)
// -------------------------------------------------------------
void LinuxAppProcessManager::stop(std::string_view id)
{
    std::unique_lock lk(mu_);
    auto it = table_.find(std::string(id));
    if (it == table_.end()) return;
    auto pid = it->second.info.pid;
    if (pid <= 0) return;

    // 已在停止流程就不重复
    if (it->second.termTimer) return;

    ::kill(pid, SIGTERM);

    auto timer = std::make_shared<asio::steady_timer>(io_, kGracefulStopTimeout);
    it->second.termTimer = timer;

    timer->async_wait([this, pid, timer](const std::error_code& ec) {
        if (ec == asio::error::operation_aborted) return;   // 已取消

        // if (::kill(pid, 0) == 0) ::kill(pid, SIGKILL);

        int status;
pid_t result = waitpid(pid, &status, WNOHANG);
if (result == 0) {
    std::cout << "asyncWaitForProcessToStop" << " Process " << pid << " is still running, sending SIGKILL" << std::endl;
    kill(-pid, SIGKILL);
    waitpid(pid, &status, 0);
}


std::cout << "asyncWaitForProcessToStop" << " Stopped process " << pid << std::endl;
    });
}

// -------------------------------------------------------------
//  restart = stop + start
// -------------------------------------------------------------
ProcessInfo LinuxAppProcessManager::restart(const LinuxAppInfo& app)
{
    stop(app.instanceId);
    /*** 小延迟可选：防止 pid 复用；此处省略，直接重启 ***/
    return start(app);
}

// -------------------------------------------------------------
//  query & list
// -------------------------------------------------------------
ProcessInfo LinuxAppProcessManager::query(std::string_view id) const
{
    std::shared_lock lk(mu_);
    if (auto it = table_.find(std::string(id)); it != table_.end())
        return it->second.info;
    return {};                                    // Unknown
}

std::vector<ProcessInfo> LinuxAppProcessManager::list() const
{
    std::vector<ProcessInfo> v;
    std::shared_lock lk(mu_);
    v.reserve(table_.size());
    for (auto& [_, ent] : table_) v.push_back(ent.info);
    return v;
}

// -------------------------------------------------------------
//  exit callback
// -------------------------------------------------------------
void LinuxAppProcessManager::registerExitCallback(ExitCallback cb)
{
    std::unique_lock lk(mu_);
    callbacks_.push_back(std::move(cb));
}

// -------------------------------------------------------------
//  SIGCHLD / waitpid handler
// -------------------------------------------------------------
void LinuxAppProcessManager::onChildExit(pid_t pid, int rawStatus)
{
    std::vector<ExitCallback> cbsCopy;
    ProcessInfo               infoCopy;

    {
        std::unique_lock lk(mu_);
        auto pidIt = pid2id_.find(pid);
        if (pidIt == pid2id_.end()) return;              // 非托管

        auto& ent = table_.at(pidIt->second);

        ent.info.state    = ProcessState::Exited;
        ent.info.endTime  = steady_clock::now();
        ent.info.exitCode = WIFEXITED(rawStatus)
                              ? WEXITSTATUS(rawStatus)
                              : -WTERMSIG(rawStatus);

        if (ent.termTimer) ent.termTimer->cancel();

        infoCopy = ent.info;
        cbsCopy  = callbacks_;

        pid2id_.erase(pidIt);
        // 若想删除表项，可 table_.erase(pidIt->second);
    }

    for (auto& cb : cbsCopy)
        asio::post(io_, [cb, infoCopy] { cb(infoCopy); });
}
