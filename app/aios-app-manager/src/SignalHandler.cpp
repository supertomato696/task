#include "SignalHandler.hpp"
#include <iostream>
#include <cassert>

SignalHandler* SignalHandler::instance_ = nullptr;

SignalHandler& SignalHandler::instance() {
    static SignalHandler inst;
    return inst;
}

SignalHandler::SignalHandler() {
    instance_ = this;
}

SignalHandler::~SignalHandler() {
    stop_worker();
    // 恢复信号
    std::lock_guard<std::mutex> lock(cb_mtx_);
    for (const auto& entry : callbacks_) {
        ::signal(entry.first, SIG_DFL);
    }
}

int SignalHandler::register_handler(int signo, SignalCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mtx_);
    int handle = ++callback_id_;
    callbacks_[signo].emplace_back(handle, std::move(cb));
    enable_signal(signo);
    return handle;
}

void SignalHandler::unregister_handler(int signo, int handle) {
    std::lock_guard<std::mutex> lock(cb_mtx_);
    auto it = callbacks_.find(signo);
    if (it != callbacks_.end()) {
        auto& vec = it->second;
        vec.erase(std::remove_if(vec.begin(), vec.end(),
            [handle](const auto& p) { return p.first == handle; }), vec.end());
        if (vec.empty()) {
            disable_signal(signo);
            callbacks_.erase(it);
        }
    }
}

void SignalHandler::enable_signal(int signo) {
    struct sigaction sa {};
    sa.sa_handler = &SignalHandler::signal_dispatcher;
    ::sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; // 可调整
    if (::sigaction(signo, &sa, nullptr) < 0) {
        std::cerr << "Failed to set signal handler for signal " << signo << std::endl;
    }
}

void SignalHandler::disable_signal(int signo) {
    ::signal(signo, SIG_DFL);
}

void SignalHandler::signal_dispatcher(int signo) {
    // 投递到线程安全队列
    auto& inst = instance();
    {
        std::lock_guard<std::mutex> lock(inst.queue_mtx_);
        inst.signal_queue_.push(signo);
    }
    inst.queue_cv_.notify_one();
}

// ====== 队列消费与异步分发 ======

void SignalHandler::dispatch_all_pending() {
    std::queue<int> local_queue;
    {
        std::lock_guard<std::mutex> lock(queue_mtx_);
        std::swap(local_queue, signal_queue_);
    }
    while (!local_queue.empty()) {
        int signo = local_queue.front();
        local_queue.pop();
        // 查找并调用所有回调
        std::lock_guard<std::mutex> lock(cb_mtx_);
        auto it = callbacks_.find(signo);
        if (it != callbacks_.end()) {
            for (const auto& p : it->second) {
                try {
                    p.second(signo);
                } catch (const std::exception& e) {
                    std::cerr << "Signal handler exception: " << e.what() << std::endl;
                }
            }
        }
    }
}

// 可在主循环主动调用
void SignalHandler::poll() {
    dispatch_all_pending();
}

// 启动后台分发线程
void SignalHandler::start_worker() {
    if (running_) return;
    running_ = true;
    worker_thread_ = std::thread([this] { worker_func(); });
}

// 停止线程
void SignalHandler::stop_worker() {
    if (!running_) return;
    running_ = false;
    queue_cv_.notify_all();
    if (worker_thread_.joinable())
        worker_thread_.join();
}

void SignalHandler::worker_func() {
    while (running_) {
        std::unique_lock<std::mutex> lock(queue_mtx_);
        if (signal_queue_.empty()) {
            queue_cv_.wait(lock, [this] { return !signal_queue_.empty() || !running_; });
            if (!running_) break;
        }
        lock.unlock();
        dispatch_all_pending();
    }
}

void SignalHandler::register_default_signals() {
    enable_signal(SIGINT);
    enable_signal(SIGTERM);
    enable_signal(SIGCHLD);
    enable_signal(SIGHUP);
    // ...
}
