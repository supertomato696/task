#pragma once
#include "ITransport.hpp"
#include <asio.hpp>
#include <thread>

namespace mcp::transport {

/** JSON‑RPC over TCP, 每条消息末尾 '\n' */
class AsioTcpTransport : public ITransport {
public:
    AsioTcpTransport(uint16_t port = 9275);

    void start() override;
    void stop()  override;
    void send(const nlohmann::json& msg) override;

private:
    void do_accept();
    void do_read();
    void handle_line(std::string_view line);

    asio::io_context                               io_;
    asio::ip::tcp::acceptor                        acceptor_;
    std::shared_ptr<asio::ip::tcp::socket>         sock_;
    asio::streambuf                                buf_;
    std::jthread                                   thr_;
    std::mutex                                     send_mtx_;
};

} // namespace mcp::transport