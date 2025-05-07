#pragma once
#include "ITransport.hpp"
#include <asio.hpp>
#include <thread>

namespace mcp::transport {

/** JSON‑RPC over TCP, 每条消息末尾 '\n' */
class AsioTcpTransport : public ITransport {
public:
    using MessageCb = std::function<void(const nlohmann::json&)>;
    AsioTcpTransport(uint16_t port = 9275);

    void start() override;
    void stop()  override;
    void onMessage(MessageCb cb);                 // 注册/替换回调（线程安全）
    void send(const nlohmann::json& msg) override;
    void sendRaw(const std::string& line);        // 已序列化 JSON + '\n'

private:
    /* async helpers */
    void do_accept();
    void do_read();
    void handle_line(std::string_view line);

    asio::io_context                               io_;
    asio::ip::tcp::acceptor                        acceptor_;
    std::shared_ptr<asio::ip::tcp::socket>         sock_;
    asio::streambuf                                buf_;
    std::jthread                                   io_thread_;
        /* state */
    std::mutex  cbMtx_;
    MessageCb   cb_;
    std::mutex                                     send_mtx_;
};

} // namespace mcp::transport