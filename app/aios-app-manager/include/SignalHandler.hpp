#pragma once

#include <functional>
#include <unordered_map>
#include <vector>
#include <queue>
#include <mutex>
#include <csignal>
#include <condition_variable>
#include <atomic>
#include <thread>

class SignalHandler {
public:
    using SignalCallback = std::function<void(int)>;

    static SignalHandler& instance();

    // 注册信号回调，返回句柄
    int register_handler(int signo, SignalCallback cb);
    void unregister_handler(int signo, int handle);

    void enable_signal(int signo);
    void disable_signal(int signo);

    void register_default_signals();

    // 异步事件队列相关
    // 启动后台分发线程
    void start_worker();
    // 停止后台线程
    void stop_worker();
    // 主动轮询信号队列（适合主循环集成）
    void poll();

    // 禁止拷贝
    SignalHandler(const SignalHandler&) = delete;
    SignalHandler& operator=(const SignalHandler&) = delete;

private:
    SignalHandler();
    ~SignalHandler();

    static void signal_dispatcher(int signo);

    // 信号队列（生产：信号线程，消费：主线程/worker）
    std::queue<int> signal_queue_;
    std::mutex queue_mtx_;
    std::condition_variable queue_cv_;

    // 回调存储
    std::atomic<int> callback_id_{0};
    std::unordered_map<int, std::vector<std::pair<int, SignalCallback>>> callbacks_;
    std::mutex cb_mtx_;

    // worker
    std::atomic<bool> running_{false};
    std::thread worker_thread_;

    static SignalHandler* instance_;
    void dispatch_all_pending();
    void worker_func();
};
