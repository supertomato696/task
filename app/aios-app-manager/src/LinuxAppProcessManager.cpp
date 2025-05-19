
#include <LinunxAppProcessManager.hpp>
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



void linux_app_process_manager::handleSigchld(const asio::error_code& error, int signal_number) {
    if (error) {
        if (error == asio::error::operation_aborted) {
            std::cerr << "handleSigchld " << " Error handling SIGCHLD: Operation aborted" << std::endl;
            std::cerr << "handleSigchld " << " what: " << error.message() << std::endl;
        } else {
            std::cerr << "Error handling SIGCHLD: " << error.message() << std::endl;
        }
        return;
    }

    while (true) {
        int status;
        pid_t pid = waitpid(-1, &status, WNOHANG);
        if (pid <= 0) {
            if (pid == -1 && errno != ECHILD) {
                std::cerr << "Error calling waitpid: " << strerror(errno) << std::endl;
            }
            break;
        }

        if (WIFEXITED(status)) {
            std::cout << "Child process " << pid << " exited with status " << WEXITSTATUS(status) << std::endl;
        } else if (WIFSIGNALED(status)) {
            std::cout << "Child process " << pid << " was killed by signal " << WTERMSIG(status) << std::endl;
        }
        for (auto it = processes.begin(); it != processes.end(); ) {
            if (it->second == pid) {
                std::string pkgName(it->first);
                it = processes.erase(it);
                if (is_registed_observed_process_died) {
                    std::cout << "Child process " << pid << " died ,callback pkgName: " << pkgName<< std::endl;
                    m_process_died_callback(pkgName);
                }
            } else {
                ++it;
            }
        }
    }

    // 再次设置异步等待
    signals_.async_wait([this](const asio::error_code& error, int signal_number) {
        handleSigchld(error, signal_number);
    });
}

