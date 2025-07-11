#include <simple_msg/ipc/ipc_client.hpp>
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
    uint8_t body[] = {'h', 'e', 'l', 'l', 'o', '\0'};
    simple::msg::ipc_client_t client{server_name};

    client.subscribe_push(2, [](std::unique_ptr<packet> pack) {
        std::cout << "recv push = " << (char*)pack->body().data() << std::endl;
    });

    asio::steady_timer timer(io_context);
    timer.expires_after(std::chrono::seconds(3));

    auto func = [&body, &client]() {
        client.send_req(simple::msg::build_req_packet(1, body, sizeof(body)), 100);

        client.send_req(simple::msg::build_req_packet(1, body, sizeof(body)), [](std::unique_ptr<packet> rsp) {
            if (rsp) {
                std::cout << "recv rsp = " << (char*)rsp->body().data() << std::endl;
            } else {
                std::cout << "failed the recv rsp" << std::endl;
            }
        }, 3);
    };
    
    start_timer(timer, func);

    io_context.run();
    
    std::cout << "exit" << std::endl;
    return 0;
}
