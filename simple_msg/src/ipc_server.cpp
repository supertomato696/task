#include <simple_msg/ipc/ipc_server.hpp>
#include <thread>
#include <csignal>
#include <iostream>
#include <asio.hpp>
#include <memory>

using namespace std::chrono;
using namespace simple::msg;


void start_timer(asio::steady_timer& timer, std::function<void()> func) {
    timer.expires_after(std::chrono::seconds(1));
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
    uint8_t body[] = {'w', 'o', 'r', 'l', 'd', '\0'};
    simple::msg::ipc_server_t server{server_name};

    server.subscribe_req(1, [&body, &server](void* conn, std::unique_ptr<packet> req) {
        std::cout << "recv req = " << (char*)req->body().data() << std::endl;
        auto rsp = simple::msg::build_rsp_packet(req->cmd(), req->seq(), 0, (uint8_t*)body, sizeof(body));
        server.send_rsp(conn, std::move(rsp));
    });
    
    asio::steady_timer timer(io_context);
    auto func = [&body, &server]() {
        auto pack = simple::msg::build_push_packet(2, (uint8_t*)body, sizeof(body));
        server.send_push(pack);
    };
    start_timer(timer, func);

    io_context.run();
    
    return 0;
}
