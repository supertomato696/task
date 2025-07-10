#include <simple_msg/rpc/rpc_server.hpp>
#include <thread>
#include <csignal>
#include <iostream>
#include <asio.hpp>
#include "example.pb.h"

constexpr uint32_t add_method_id = 1;
constexpr uint32_t push_method_id = 2;

void start_timer(asio::steady_timer& timer, std::function<void()> func) {
    timer.expires_after(std::chrono::milliseconds(1));
    timer.async_wait([&timer, func](const asio::error_code& ec) {
        func();
        start_timer(timer, func);
    });
}

int main(int argc, char** argv) {
    asio::io_context io_context;
    
    asio::signal_set signals(io_context, SIGINT, SIGTERM);
    signals.async_wait([&](auto, auto) {
        io_context.stop();
    });
    
    
    std::string server_name = "/tmp/simple_msg";
    simple::msg::rpc_server_t server{server_name};

//    server.subscribe_request<add_req, add_rsp>(add_method_id, [](add_req& req) -> simple::msg::rpc_result<add_rsp> {
//        add_rsp rsp;
//        rsp.set_result(req.left() + req.right());
//        return rsp;
//    });

    server.subscribe_request<add_req>(add_method_id, [&server, &io_context](void* conn, uint32_t method_id, uint32_t seq, add_req& req) {
        add_rsp rsp;
        rsp.set_result(req.left() + req.right());
        simple::msg::rpc_result<add_rsp> rsp_result{rsp};
        server.send_rsp<add_rsp>(conn, method_id, seq, rsp_result);
    });

//    asio::steady_timer timer(io_context);
//    auto func = [&server]() {
//        hello_push msg;
//        msg.set_hello("hello from server");
//        server.push(push_method_id, msg);
//    };
//    start_timer(timer, func);

    io_context.run();
    
    std::cout << "exit" << std::endl;
    return 0;
}
