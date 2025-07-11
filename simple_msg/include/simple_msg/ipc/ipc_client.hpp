#pragma once

#include <mutex>
#include <cstdint>
#include <unordered_map>
#include <functional>
#include <memory>
#include <utility>
#include <future>
#include <atomic>
#include <optional>

#include <simple_msg/detail/connection.hpp>

using namespace std::placeholders;
using asio::local::stream_protocol;

namespace simple::msg {

    class ipc_client_t {
        enum connection_state {
                state_connected = 1,
                state_disconnected = 2,
                state_connecting = 3,
        };
        constexpr static const uint64_t reconnect_timer_interval_ms = 1 * 1000;
        public:
            using recv_callback_t = std::function<void(std::unique_ptr<packet> pack)>;
        public:
            explicit ipc_client_t(std::string server_name)
            : m_work_guard(m_io_context.get_executor())
            , m_server_name(std::move(server_name))
            , m_connection_state(state_disconnected)
            , m_reconnect_timer(m_io_context) {
                std::cout << "ipc_client_t" << std::endl;
                m_io_thread = std::thread([this]() {
                    std::cout << "m_io_thread begin" << std::endl;
                    m_io_context.run();
                    std::cout << "m_io_thread end" << std::endl;
                });
                
                asio::post(m_io_context, [this]() {
                    do_connect();
                    start_reconnect_timer();
                });
            }
            
            ~ipc_client_t() {
                assert(std::this_thread::get_id() != m_io_thread.get_id());
                
                auto task = [this]() {
                    m_connection = nullptr;
                    asio::error_code ec;
                    m_reconnect_timer.cancel(ec);
                    m_push_processors.clear();
                    m_work_guard.reset();
                    m_io_context.stop();
                };
                
                asio::post(m_io_context, std::move(task));
                
                if (m_io_thread.joinable()) {
                    m_io_thread.join();
                }
            }

            std::unique_ptr<packet> send_req(packet pack, uint32_t timeout_mills) {
                std::promise<void> prom;
                auto fut = prom.get_future();
                std::unique_ptr<packet> rsp_pack;
                
                auto cb = [&prom, &rsp_pack](std::unique_ptr<packet> pack) {
                    rsp_pack = std::move(pack);
                    prom.set_value();
                };
                
                auto task = [this, pack = std::move(pack), cb, timeout_mills]() {
                    do_send_req(std::move(pack), cb, timeout_mills);
                };
                
                asio::post(m_io_context, std::move(task));
                
                fut.get();
                return rsp_pack;
            }
            
            void send_req(packet pack, recv_callback_t cb, uint32_t timeout_mills) {
                auto task = [this, pack = std::move(pack), cb, timeout_mills]() {
                    do_send_req(std::move(pack), cb, timeout_mills);
                };
                
                asio::post(m_io_context, std::move(task));
            }

            void cancel_sending(uint32_t cmd, uint32_t seq) {
                auto task = [this, cmd, seq]() {
                    if (m_connection) {
                        m_connection->cancel_sending(cmd, seq);
                    }
                };
                
                asio::post(m_io_context, std::move(task));
            }
            
            void subscribe_push(uint32_t cmd, recv_callback_t cb) {
                auto task = [this, cmd, cb]() {
                    m_push_processors[cmd] = std::move(cb);
                };
                
                asio::post(m_io_context, std::move(task));
            }

            void unsubscribe_push(uint32_t cmd) {
                auto task = [this, cmd]() {
                    m_push_processors.erase(cmd);
                };
                
                asio::post(m_io_context, std::move(task));
            }
        private:
            ipc_client_t(const ipc_client_t&) = delete;
            ipc_client_t& operator=(const ipc_client_t&) = delete;

            void do_send_req(packet pack, recv_callback_t cb, uint32_t timeout_mills) {
                if (!m_connection) {
                    do_connect();
                }

                if (m_connection) {
                    m_connection->send_req(std::move(pack), std::move(cb), timeout_mills);
                } else {
                    cb(nullptr);
                }
            }

            void do_connect() {
                if (m_connection_state != state_disconnected) {
                    return;
                }
                
                std::cout << get_cur_time() << "    " << "connect begin " << std::endl;

                m_connection = nullptr;
                
                stream_protocol::socket sock(m_io_context);
                asio::error_code ec;
                sock.connect(stream_protocol::endpoint(m_server_name), ec);
                
                std::cout << get_cur_time() << "    " << "connect ec = " << ec.message() << std::endl;
                
                if (!ec) {
                    std::cout << get_cur_time() << "    " << "connect success" << std::endl;

                    m_connection_state = state_connected;
                    m_connection = std::make_shared<connection_t>(m_io_context, false, std::move(sock)
                    , [this](connection_t* conn) {
                        on_connection_broken(conn);
                    }
                    , nullptr
                    , [this](connection_t* conn,  std::unique_ptr<packet> pack) {
                        on_receive_push(conn, std::move(pack));
                    });
                } else {
                    m_connection_state = state_disconnected;
                }
            }
            
            void start_reconnect_timer() {
                m_reconnect_timer.expires_after(std::chrono::milliseconds(reconnect_timer_interval_ms));
                m_reconnect_timer.async_wait([this](const asio::error_code& ec) {
                    on_reconnect_check();
                });
            }
            
            void on_reconnect_check() {
                if (m_connection_state == state_disconnected) {
                    do_connect();
                }
                
                start_reconnect_timer();
            }
            
            void on_connection_broken(connection_t* conn) {
                std::cout << get_cur_time() << "    " << "on_connection_broken" << std::endl;
                m_connection_state = state_disconnected;
                do_connect();
            }

            void on_receive_push(connection_t* conn,  std::unique_ptr<packet> pack) {
                auto it = m_push_processors.find(pack->cmd());
                if (it != m_push_processors.end()) {
                    it->second(std::move(pack));
                }
            }
        private:
            std::thread m_io_thread;
            asio::io_context m_io_context;
            asio::executor_work_guard<asio::io_context::executor_type> m_work_guard;

            std::string m_server_name;
            
            asio::steady_timer m_reconnect_timer;

            connection_state m_connection_state;
            std::shared_ptr<connection_t> m_connection;

            using map_cmd_2_callback_t = std::unordered_map<uint32_t, recv_callback_t >;
            map_cmd_2_callback_t m_push_processors;
        };

}
