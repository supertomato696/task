#include "transport/AsioTcpTransport.hpp"
#include <iostream>
#include <shared_mutex>
using namespace mcp::transport;
using asio::ip::tcp;

AsioTcpTransport::AsioTcpTransport(uint16_t port)
: acceptor_(io_, tcp::endpoint(tcp::v4(), port))
{}

void AsioTcpTransport::start(){
    do_accept();
    io_thread_ = std::jthread([this]{ io_.run(); });
}

void AsioTcpTransport::stop(){
    io_.stop();
    if(io_thread_.joinable()) io_thread_.join();
}

void AsioTcpTransport::onMessage(MessageCb cb)
{
    std::lock_guard lk(cbMtx_);
    cb_ = std::move(cb);
}


/* ---------- 发送 ---------- */
void AsioTcpTransport::send(const nlohmann::json& msg){
    sendRaw(msg.dump());
    // if(!sock_) return;
    // std::string data = msg.dump();
    // data.push_back('\n');
    // std::scoped_lock lk(send_mtx_);
    // asio::async_write(*sock_, asio::buffer(data),
    //     [](std::error_code ec,std::size_t){ if(ec) std::cerr<<"send error\n"; });
}


void AsioTcpTransport::sendRaw(const std::string& lineNoLf)
{
    if (!sock_ || !sock_->is_open()) return;
    std::string data = lineNoLf;
    if (data.empty() || data.back() != '\n') data.push_back('\n');

    std::scoped_lock lk(send_mtx_);
    asio::async_write(*sock_, asio::buffer(data),
        [](std::error_code ec, std::size_t){
            if (ec) std::cerr << "[tcp] send error: " << ec.message() << '\n';
        });
}

/* ========= accept / read ========= */
void AsioTcpTransport::do_accept(){
    // acceptor_.async_accept([this](std::error_code ec, tcp::socket s){
    //     if(!ec){
    //         sock_ = std::make_shared<tcp::socket>(std::move(s));
    //         do_read();
    //     }
    //     do_accept();                               // 继续监听后续连接
    // });

    acceptor_.async_accept(
        [this](std::error_code ec, tcp::socket s)
    {
        if (ec) {
            std::cerr << "[tcp] accept: " << ec.message() << '\n';
        } else {
            std::cout  << "[tcp] client connected: " << s.remote_endpoint() << '\n';
            sock_ = std::make_shared<tcp::socket>(std::move(s));
            do_read();
        }
        do_accept();     // 继续监听下一条连接
    });
}

/* ---------- 读取 ---------- */
void AsioTcpTransport::do_read(){
    // if(!sock_) return;
    // asio::async_read_until(*sock_, buf_, '\n',
    //     [this](std::error_code ec, std::size_t){
    //         if(ec){ std::cerr<<"read error: "<<ec.message()<<"\n"; return;}
    //         std::istream is(&buf_);
    //         std::string line; std::getline(is, line);      // 去除 '\n'
    //         handle_line(line);
    //         do_read();
    //     });

    if (!sock_) return;
    asio::async_read_until(*sock_, buf_, '\n',
        [this](std::error_code ec, std::size_t)
    {
        if (ec) {
            std::cerr << "[tcp] read: " << ec.message() << '\n';
            sock_.reset();
            return;
        }
        std::istream is(&buf_);
        std::string line; std::getline(is, line);     // 去掉 '\n'
        handle_line(line);
        do_read();   // 再次等待
    });
}

void AsioTcpTransport::handle_line(std::string_view line){
    // if(cb_){
    //     try{
    //         auto j = nlohmann::json::parse(line);
    //         cb_(j);
    //     }catch(const std::exception& e){
    //         std::cerr<<"json parse error: "<<e.what()<<"\n";
    //     }
    // }

    MessageCb cbCopy;
    {
        std::shared_lock lk(cbMtx_);
        cbCopy = cb_;
    }
    if (!cbCopy) return;

    try {
        auto j = nlohmann::json::parse(line);
        cbCopy(j);
    } catch (const std::exception& e) {
        std::cerr << "[tcp] json parse error: " << e.what() << '\n';
    }
}
