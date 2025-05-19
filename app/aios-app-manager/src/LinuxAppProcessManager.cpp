#include "LinuxAppProcessManager.hpp"

#include <asio/post.hpp>
#include <csignal>
#include <sys/wait.h>

using namespace std::chrono;

// ---------------- ctor ----------------
LinuxAppProcessManager::LinuxAppProcessManager(asio::io_context& ctx)
    : io_(ctx),
      monitor_(ctx, [this](pid_t p, int st) { onChildExit(p, st); })
{}

// ---------------- start ----------------
ProcessInfo LinuxAppProcessManager::start(const LinuxAppInfo& app) {
    ProcessInfo pi;
    pi.instanceId  = app.instanceId;
    pi.execPath    = app.execPath;
    pi.state       = ProcessState::Starting;
    pi.startTime   = steady_clock::now();

    std::error_code ec;
    pid_t pid = Launcher::start(app, ec);
    if (ec) {
        pi.state    = ProcessState::Failed;
        pi.exitCode = -ec.value();
        return pi;                          // 启动失败直接返回；调用方决定处理
    }

    pi.pid   = pid;
    pi.state = ProcessState::Running;

    {
        std::unique_lock lk(mu_);
        table_[app.instanceId] = TimedEntry{pi, nullptr};
        pid2id_[pid]           = app.instanceId;
    }
    return pi;
}

// ---------------- stop -----------------
void LinuxAppProcessManager::stop(std::string_view id) {
    std::unique_lock lk(mu_);
    auto it = table_.find(std::string(id));
    if (it == table_.end()) return;         // not running

    auto pid = it->second.info.pid;
    if (pid <= 0) return;

    // 已经在停止流程？
    if (it->second.termTimer) return;

    // 1) 发送 SIGTERM
    ::kill(pid, SIGTERM);

    // 2) 创建定时器等待 kGracefulStopTimeout
    auto timer = std::make_shared<asio::steady_timer>(io_, kGracefulStopTimeout);
    it->second.termTimer = timer;

    timer->async_wait([this, pid, idStr = std::string(id), timer](const std::error_code& ec) {
        if (ec == asio::error::operation_aborted) return;   // 子进程已自行退出

        // 如果仍然存活 → SIGKILL
        if (::kill(pid, 0) == 0) { ::kill(pid, SIGKILL); }
    });
}

// ---------------- restart --------------
ProcessInfo LinuxAppProcessManager::restart(const LinuxAppInfo& app) {
    stop(app.instanceId);
    return start(app);
}

// ---------------- query/list -----------
ProcessInfo LinuxAppProcessManager::query(std::string_view id) const {
    std::shared_lock lk(mu_);
    if (auto it = table_.find(std::string(id)); it != table_.end()) {
        return it->second.info;
    }
    return {};   // default-constructed ⇒ Unknown
}

std::vector<ProcessInfo> LinuxAppProcessManager::list() const {
    std::vector<ProcessInfo> v;
    std::shared_lock lk(mu_);
    for (auto& [id, ent] : table_) v.push_back(ent.info);
    return v;
}

// ---------------- callback -------------
void LinuxAppProcessManager::registerExitCallback(ExitCallback cb) {
    std::unique_lock lk(mu_);
    callbacks_.push_back(std::move(cb));
}

// ---------------- child exit -----------

void LinuxAppProcessManager::onChildExit(pid_t pid, int raw) {
    std::vector<ExitCallback> cbs;
    ProcessInfo               copy;

    {
        std::unique_lock lk(mu_);
        auto pidIt = pid2id_.find(pid);
        if (pidIt == pid2id_.end()) return;               // 非托管进程

        auto& ent = table_.at(pidIt->second);

        ent.info.state    = ProcessState::Exited;
        ent.info.endTime  = steady_clock::now();
        ent.info.exitCode = WIFEXITED(raw) ? WEXITSTATUS(raw) : -WTERMSIG(raw);

        // 取消可能存在的 timer
        if (ent.termTimer) ent.termTimer->cancel();

        copy = ent.info;                                  // 拷贝给回调
        cbs  = callbacks_;                                // 拷贝回调列表

        pid2id_.erase(pidIt);
        // 可选择保留或删除 table_ 条目；此处保留，供历史查询
    }

    // 异步广播，避免在信号处理路径内执行用户代码
    for (auto& cb : cbs) {
        asio::post(io_, [cb, copy] { cb(copy); });
    }
}
