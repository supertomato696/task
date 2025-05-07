#pragma once
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <asio/io_context.hpp>
#include <nlohmann/json.hpp>

#include <functional>
#include <mutex>
#include <thread>

namespace mcp::transport {

/**
 * WebSocket 版传输层。
 *  - 每条 JSON‑RPC 消息放在一个 WS text frame 中
 *  - 目前只支持单客户端（第一个连上的连接）
 */
class AsioWsTransport {
public:
    using Json       = nlohmann::json;
    using MessageCb  = std::function<void(const Json&)>;

    explicit AsioWsTransport(uint16_t port);

    /** 启动监听 (非阻塞) */
    void start();
    /** 主动停止并回收线程 */
    void stop();

    /** 注册收到 JSON 消息时的回调 */
    void onMessage(MessageCb cb) { cb_ = std::move(cb); }

    /** 发送 JSON (自动 dump 为文本帧) */
    void send(const Json& msg);

private:
    using WsServer   = websocketpp::server<websocketpp::config::asio>;

    /* ---- WS 事件 ---- */
    void handleOpen(websocketpp::connection_hdl hdl);
    void handleClose(websocketpp::connection_hdl hdl);
    void handleMsg(websocketpp::connection_hdl hdl,
                   WsServer::message_ptr       msg);

    asio::io_context          io_;
    WsServer                  srv_;
    std::jthread              th_;

    std::mutex                conn_mtx_;
    websocketpp::connection_hdl client_;          // 只保存首个连接

    std::mutex                send_mtx_;
    MessageCb                 cb_;
};

} // namespace mcp::transport
