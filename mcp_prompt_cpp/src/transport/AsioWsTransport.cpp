#include "transport/AsioWsTransport.hpp"
#include <iostream>

using websocketpp::connection_hdl;
using namespace mcp::transport;

AsioWsTransport::AsioWsTransport(uint16_t port)
{
    // 1) 基础配置
    srv_.clear_access_channels(websocketpp::log::alevel::all);
    srv_.init_asio(&io_);

    // 2) 绑定事件
    srv_.set_open_handler([this](connection_hdl h){ handleOpen(h); });
    srv_.set_close_handler([this](connection_hdl h){ handleClose(h); });
    srv_.set_message_handler([this](connection_hdl h, WsServer::message_ptr m){
        handleMsg(h, std::move(m));
    });

    // 3) 监听端口
    srv_.listen(port);
    srv_.start_accept();
}

/* ---------- lifecycle ---------- */
void AsioWsTransport::start()
{
    if(th_.joinable()) return;             // 已经在跑
    th_ = std::jthread([this]{ io_.run(); });
}

void AsioWsTransport::stop()
{
    srv_.stop_listening();
    io_.stop();
    if(th_.joinable()) th_.join();
}

/* ---------- send ---------- */
void AsioWsTransport::send(const Json& msg)
{
    std::scoped_lock lk(conn_mtx_);
    if(client_.expired()) return;          // 还没有客户端
    std::string text = msg.dump();

    std::scoped_lock slk(send_mtx_);
    websocketpp::lib::error_code ec;
    srv_.send(client_, text, websocketpp::frame::opcode::text, ec);
    if(ec) std::cerr << "Ws send error: " << ec.message() << "\n";
}

/* ---------- event: open / close / message ---------- */
void AsioWsTransport::handleOpen(connection_hdl hdl)
{
    std::scoped_lock lk(conn_mtx_);
    if(!client_.expired()){
        // 只允许一个客户端；后续连接直接关闭
        srv_.close(hdl, websocketpp::close::status::policy_violation,
                   "Multiple connections not supported");
        return;
    }
    client_ = hdl;
    std::cout << "[Ws] client connected\n";
}

void AsioWsTransport::handleClose(connection_hdl hdl)
{
    std::scoped_lock lk(conn_mtx_);
    if(hdl.lock() == client_.lock()){
        client_.reset();
        std::cout << "[Ws] client disconnected\n";
    }
}

void AsioWsTransport::handleMsg(connection_hdl /*hdl*/,
                                WsServer::message_ptr msg)
{
    if(!cb_) return;
    try{
        Json j = Json::parse(msg->get_payload());
        cb_(j);
    }catch(const std::exception& e){
        std::cerr << "Ws json parse error: " << e.what() << "\n";
    }
}
