#include <simple_msg/rpc/rpc_client.hpp>
#include <thread>
#include <csignal>
#include <iostream>
#include <asio.hpp>
#include "example.pb.h"


void start_timer(asio::steady_timer& timer, std::function<void()> func) {
    timer.expires_after(std::chrono::milliseconds (1));
    timer.async_wait([&timer, func](const asio::error_code& ec) {
        func();
        start_timer(timer, func);
    });
}

constexpr uint32_t add_method_id = 1;
constexpr uint32_t push_method_id = 2;

int main(int argc, char** argv) {
    asio::io_context io_context;

    volatile bool stop = false;
    asio::signal_set signals(io_context, SIGINT, SIGTERM);
    signals.async_wait([&](auto, auto) {
        stop = true;
        io_context.stop();
    });
    
    
    std::string server_name = "/tmp/simple_msg";
    simple::msg::rpc_client_t client{server_name};

    client.subscribe_push<hello_push>(push_method_id, [](hello_push& msg) {
        std::cout << "recv push = "  << msg.hello() << std::endl;
    });

    std::thread t([&client, &stop]() {
        for (int i = 0; !stop ; ++i) {
            add_req req;
            req.set_left(1);
            req.set_right(2);
            client.call<add_rsp>(add_method_id, req, [](simple::msg::rpc_result<add_rsp> result) {
                if (!result) {
                    std::cout << simple::msg::get_cur_time() << "    " << "add request failed" << std::endl;
                } else {
                    std::cout << simple::msg::get_cur_time() << "    "  << "add request result = " << result.value().result() << std::endl;
                }
            });
            std::this_thread::sleep_for(std::chrono::milliseconds (1));
        }
    });

    io_context.run();

    t.join();
    
    std::cout << "exit" << std::endl;
    return 0;
}
