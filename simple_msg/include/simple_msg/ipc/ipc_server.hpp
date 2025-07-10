#pragma once
#include <utility>
#include <future>
#include <cassert>

#include <simple_msg/detail/connection.hpp>

using namespace std::placeholders;

namespace simple::msg {
        class ipc_server_t {
            constexpr static uint32_t heartbeat_cmd = 0;
            using list_connection_t = std::list<std::shared_ptr<connection_t>>;
            struct file_deleter {
                file_deleter(std::string& file_name) {
                    unlink(file_name.c_str());
                }
            };

        public:
            using recv_req_callback_t = std::function<void(void*, std::unique_ptr<packet> pack)>;
        public:
            explicit ipc_server_t(std::string server_name)
            : m_work_guard(m_io_context.get_executor())
            , m_file_deleter(server_name)
            , m_acceptor(m_io_context, std::move(server_name), true) {
                if (chmod(server_name.c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH) == -1) {
                    std::cout << "Failed to chmod permission for " << server_name << std::endl;
                }
                m_io_thread = std::thread([this]() {
                    m_io_context.run();
                });
                
                asio::post(m_io_context, [this]() {
                    do_accept();
                });
            }

            ~ipc_server_t() {
                assert(std::this_thread::get_id() != m_io_thread.get_id());
                
                auto task = [this]() {
                    m_connections.clear();
                    m_req_processors.clear();
                    
                    asio::error_code ec;
                    m_acceptor.close(ec);
                    m_work_guard.reset();

                    m_io_context.stop();
                };
                
                asio::post(m_io_context, std::move(task));
                
                if (m_io_thread.joinable()) {
                    m_io_thread.join();
                }
            }
        public:
            void send_push(packet pack) {
                auto task = [this, pack = std::move(pack)]() {
                    for (auto& conncetion : m_connections) {
                        conncetion->send_push(pack);
                    }
                };
                
                asio::post(m_io_context, std::move(task));
            }
            
            void send_push(void* conn, packet pack) {
                auto task = [this, conn, pack = std::move(pack)]() {
                    connection_t* c = static_cast<connection_t*>(conn);
                    for (auto& conncetion : m_connections) {
                        if (conncetion.get() == c) {
                            conncetion->send_push(pack);
                            break;
                        }
                    }
                };
                
                asio::post(m_io_context, std::move(task));
            }

            void send_rsp(void* conn, packet pack) {
                auto task = [this, conn, pack = std::move(pack)]() {
                    connection_t* c = static_cast<connection_t*>(conn);
                    for (auto& conncetion : m_connections) {
                        if (conncetion.get() == c) {
                            conncetion->send_rsp(pack);
                            break;
                        }
                    }
                };
                
                asio::post(m_io_context, std::move(task));
            }

            void subscribe_req(uint32_t cmd, recv_req_callback_t cb) {
                auto task = [this, cmd, cb]() {
                    m_req_processors[cmd] = cb;
                };
                
                asio::post(m_io_context, std::move(task));
            }

            void unsubscribe_req(uint32_t cmd) {
                auto task = [this, cmd]() {
                    m_req_processors.erase(cmd);
                };
                
                asio::post(m_io_context, std::move(task));
            }
        private:
            ipc_server_t(const ipc_server_t&) = delete;
            ipc_server_t& operator=(const ipc_server_t&) = delete;

            void do_accept() {
                m_acceptor.async_accept(
                    [this](std::error_code ec, stream_protocol::socket sock) {
                        std::cout << get_cur_time() << "    " << "async_accept, ec = " << ec.value() << std::endl;
                        if (!ec) {
                          auto connection = std::make_shared<connection_t>(m_io_context, true, std::move(sock)
                          , [this](connection_t* conn) {
                              on_connection_broken(conn);
                          }
                          , [this](connection_t* conn,  std::unique_ptr<packet> pack) {
                              on_receive_req(conn, std::move(pack));
                          }
                         , nullptr
                        );
                          
                          m_connections.push_back(connection);
                      }

                      do_accept();
                    });
            }
            
            void on_connection_broken(connection_t* conn) {
                std::cout << get_cur_time() << "    " << "remove, broken connection" << std::endl;
                auto it = std::remove_if(m_connections.begin(), m_connections.end(), [conn](const auto& item) {
                    return item.get() == conn;
                });

                m_connections.erase(it, m_connections.end());
            }

            void on_receive_req(connection_t* conn, std::unique_ptr<packet> pack) {
                if (pack->cmd() == heartbeat_cmd) {
                    conn->send_rsp(build_rsp_packet(pack->cmd(), pack->seq(), 0, nullptr, 0));
                    return;
                }
                
                auto it = m_req_processors.find(pack->cmd());
                if (it != m_req_processors.end()) {
                    it->second(conn, std::move(pack));
                }
            }
        private:
            asio::io_context m_io_context;
            std::thread m_io_thread;
            asio::executor_work_guard<asio::io_context::executor_type> m_work_guard;
            
            file_deleter m_file_deleter;
            stream_protocol::acceptor m_acceptor;
            
            list_connection_t m_connections;

            using map_cmd_2_callback_t = std::unordered_map<uint32_t, recv_req_callback_t>;
            map_cmd_2_callback_t m_req_processors;
        };
}
