#pragma once
#include <asio.hpp>
#include <nlohmann/json.hpp>
#include <future>
#include <condition_variable>
#include <unordered_map>
#include <string>
#include <memory>
#include <thread>

namespace mcp::client2 {

/**
 * 线程安全、阻塞 / 异步两用的 Json‑RPC 2.0 客户端
 * 负责：
 *   • 维持 Asio 连接
 *   • 发送 JSON‑RPC 请求、等待响应
 *   • 为异步请求返回 std::future<nlohmann::json>
 */
class JsonRpcClient {
public:
    using NotificationCb = std::function<void(const nlohmann::json& params)>;
    JsonRpcClient(std::string host, uint16_t port);
    ~JsonRpcClient();

    /** 同步 RPC：结构体 → JSON → 等待 JSON → 结构体 */
    template<class Request, class Response>
    Response call(const Request& req)
    {
        nlohmann::json jreq = req;              // to_json(Request)
        auto jresp = callRaw(jreq);
        return jresp.get<Response>();           // from_json(Response)
    }

    /** 异步 RPC：返回 future<Response> */
    template<class Request, class Response>
    std::future<Response> callAsync(const Request& req)
    {
        nlohmann::json jreq = req;
        auto futJ = callRawAsync(jreq);
        // 把 future<json> 转 future<Response>
        auto bridge = std::make_shared<std::promise<Response>>();
        auto fut    = bridge->get_future();
        std::thread([f = std::move(futJ), b = bridge]() mutable {
            try {
                Response obj = f.get().template get<Response>();
                b->set_value(std::move(obj));
            } catch (...) {
                b->set_exception(std::current_exception());
            }
        }).detach();
        return fut;
    }
    
        /** 注册指定 method 的通知回调；线程安全 */
    void onNotification(const std::string& method, NotificationCb cb)
    {
        std::lock_guard lk(notifyMtx_);
        notifyMap_[method] = std::move(cb);
    }

private:
    /* ---------- low‑level ---------- */
    nlohmann::json                callRaw(const nlohmann::json&);
    std::future<nlohmann::json>   callRawAsync(const nlohmann::json&);

    void start();
    void connect();
    void readLoop();
    void sendRaw(const std::string&);

    /* ---------- members ---------- */
    asio::io_context                  io_;
    asio::ip::tcp::resolver           resolver_;
    std::shared_ptr<asio::ip::tcp::socket> sock_;
    asio::streambuf                   buf_;
    std::jthread                      ioThread_;

    std::mutex                        mtx_;
    std::condition_variable           cv_;
    uint64_t                          idGen_{1};

    std::unordered_map<std::string, nlohmann::json>              syncResp_;   // id -> result/error
    std::unordered_map<std::string, std::shared_ptr<std::promise<nlohmann::json>>> pendings_;

    std::string host_;
    uint16_t    port_;

        /* ---- 通知回调表 ---- */
    std::mutex notifyMtx_;
    std::unordered_map<std::string, NotificationCb> notifyMap_;
};

} // namespace mcp::client2