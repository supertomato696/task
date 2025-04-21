#include "transport/AsioTcpTransport.hpp"
#include <iostream>

using namespace mcp::transport;
using asio::ip::tcp;

AsioTcpTransport::AsioTcpTransport(uint16_t port)
: acceptor_(io_, tcp::endpoint(tcp::v4(), port))
{}

void AsioTcpTransport::start(){
    do_accept();
    thr_ = std::jthread([this]{ io_.run(); });
}

void AsioTcpTransport::stop(){
    io_.stop();
    if(thr_.joinable()) thr_.join();
}

void AsioTcpTransport::do_accept(){
    acceptor_.async_accept([this](std::error_code ec, tcp::socket s){
        if(!ec){
            sock_ = std::make_shared<tcp::socket>(std::move(s));
            do_read();
        }
        do_accept();                               // 继续监听后续连接
    });
}

/* ---------- 读取 ---------- */
void AsioTcpTransport::do_read(){
    if(!sock_) return;
    asio::async_read_until(*sock_, buf_, '\n',
        [this](std::error_code ec, std::size_t){
            if(ec){ std::cerr<<"read error: "<<ec.message()<<"\n"; return;}
            std::istream is(&buf_);
            std::string line; std::getline(is, line);      // 去除 '\n'
            handle_line(line);
            do_read();
        });
}

void AsioTcpTransport::handle_line(std::string_view line){
    if(cb_){
        try{
            auto j = nlohmann::json::parse(line);
            cb_(j);
        }catch(const std::exception& e){
            std::cerr<<"json parse error: "<<e.what()<<"\n";
        }
    }
}

/* ---------- 发送 ---------- */
void AsioTcpTransport::send(const nlohmann::json& msg){
    if(!sock_) return;
    std::string data = msg.dump();
    data.push_back('\n');
    std::scoped_lock lk(send_mtx_);
    asio::async_write(*sock_, asio::buffer(data),
        [](std::error_code ec,std::size_t){ if(ec) std::cerr<<"send error\n"; });
}