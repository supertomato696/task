#pragma once

#include <cstdint>
#include <unistd.h>
#include <string>
#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <list>
#include <utility>
#include <iostream>

#include <asio/write.hpp>
#include <asio/read.hpp>
#include <asio/buffer.hpp>
#include <asio/local/stream_protocol.hpp>
#include <asio/steady_timer.hpp>

#include <simple_msg/detail/io_buffer.hpp>
#include <simple_msg/ipc/packet.hpp>


using asio::local::stream_protocol;

namespace simple::msg {
    using recv_callback_t = std::function<void(std::unique_ptr<packet> pack)>;

    class connection_t : public std::enable_shared_from_this<connection_t> {
        constexpr static uint32_t max_read_buffer_size = 256*1024;
        constexpr static uint32_t max_write_buffer_size = 256*1024;
        constexpr static uint32_t active_connection_lifetime_ms = 5 * 1000;
        constexpr static uint64_t check_timer_interval_ms = 100;
        constexpr static uint64_t send_heartbeat_interval_ms = 2 * 1000;
        constexpr static uint64_t default_send_timeout_ms = 3 * 1000;

        struct request_t {
            packet pack;
            uint32_t timeout_mills{};
            recv_callback_t callback;
            std::chrono::steady_clock::time_point begin_time;
        };

        using map_packid_2_callback_t = std::unordered_map<uint64_t, request_t >;
        using packet_list_t = std::list<request_t>;
        
        public:
            using connection_broken_callback_t = std::function<void(connection_t* conn)>;
            using receive_packet_callback_t = std::function<void(connection_t* conn, std::unique_ptr<packet>)>;
        public:
            connection_t(asio::io_context& io_context, bool is_server, stream_protocol::socket sock, connection_broken_callback_t connection_broken_cb
                         , receive_packet_callback_t receive_req_packet_cb
                         , receive_packet_callback_t receive_push_packet_cb)
            : m_io_context(io_context)
            , m_is_server(is_server)
            , m_sock(std::move(sock))
            , m_read_buf(max_read_buffer_size)
            , m_write_buf(max_write_buffer_size)
            , async_write_pending(false)
            , m_connection_broken_callback(std::move(connection_broken_cb))
            , m_receive_req_packet_cb(std::move(receive_req_packet_cb))
            , m_receive_push_packet_cb(std::move(receive_push_packet_cb))
            , check_timer(m_io_context)
            , m_last_recv_time(std::chrono::steady_clock::now())
            , m_last_send_heartbeat_time(std::chrono::steady_clock::now()) {
                asio::post(m_io_context, [this]() {
                    do_read();
                    start_check_timer();
                });
            }

            ~connection_t() {
                std::cout << get_cur_time() << "    " << "~connection_t" << std::endl;

                for (auto& request : m_waiting_for_sending_packets) {
                    std::cout << get_cur_time() << "    " << "cancel request" << std::endl;
                    if (request.pack.is_req() && request.callback) {
                        if (request.callback != nullptr) {
                            request.callback(nullptr);
                        }
                    }
                }

                for (auto& request : m_waiting_for_response_requests) {
                    std::cout << get_cur_time() << "    " << "cancel request" << std::endl;
                    if (request.second.callback != nullptr) {
                        request.second.callback(nullptr);
                    }
                }
            }

            void send_push(packet pack) {
                send_packet(std::move(pack), nullptr, default_send_timeout_ms);
            }
            
            void send_rsp(packet pack) {
                send_packet(std::move(pack), nullptr, default_send_timeout_ms);
            }

            void send_req(packet pack, recv_callback_t cb, uint32_t timeout_mills) {
                send_packet(std::move(pack), cb, timeout_mills);
            }

            void cancel_sending(uint32_t cmd, uint32_t seq) {
                auto id = packet_id(cmd, seq);
                m_waiting_for_sending_packets.erase(std::remove_if(m_waiting_for_sending_packets.begin(), m_waiting_for_sending_packets.end(), [id](const auto& item) {
                    return packet_id(item.pack) == id;
                }), m_waiting_for_sending_packets.end());
                
                m_waiting_for_response_requests.erase(id);
            }
        private:
            connection_t(const connection_t&) = delete;
            connection_t& operator=(const connection_t&) = delete;

            void start_check_timer() {
                check_timer.expires_after(std::chrono::milliseconds(check_timer_interval_ms));
                check_timer.async_wait([weak_this = weak_from_this()](const asio::error_code& ec) {
                    auto shared_this = weak_this.lock();
                    if (!shared_this) {
                        return;
                    }
                    shared_this->on_check_timer();
                });
            }
        
            void send_packet(packet pack, recv_callback_t cb, uint32_t timeout_mills) {
                m_waiting_for_sending_packets.push_back(request_t{std::move(pack), timeout_mills, cb, std::chrono::steady_clock::now()});
                
                do_write();
            }
            
            void do_write() {
                if (async_write_pending) {
                    return;
                }

                if (m_waiting_for_sending_packets.empty()) {
                    return;
                }
                                
                auto request = std::move(m_waiting_for_sending_packets.front());
                m_waiting_for_sending_packets.pop_front();

                m_write_buf.clear();
                if (!encode_packet(request.pack, m_write_buf)) {
                    //invalid packet, too big
                    if (request.callback != nullptr) {
                        request.callback(nullptr);
                    }
                    return;
                }
                
                if (request.pack.is_req() && request.callback != nullptr) {
                    m_waiting_for_response_requests[packet_id(request.pack)] = std::move(request);
                }

                async_write_pending = true;
                asio::async_write(m_sock, asio::buffer(m_write_buf.read_head(), m_write_buf.size()), [weak_this = weak_from_this()](std::error_code ec, std::size_t len) {
                    auto shared_this = weak_this.lock();
                    if (!shared_this) {
                        return;
                    }

                    shared_this->async_write_pending = false;

                    if (ec) {
                        shared_this->m_connection_broken_callback(shared_this.get());
                        return;
                    }
                    
                    shared_this->do_write();
              });
            }
            
            void do_read() {
                auto size_to_read = (m_read_buf.free_size() > 0) ? m_read_buf.free_size() : m_read_buf.capacity();
                if (size_to_read <= 0) {
                    return;
                }
                
                auto buf = m_read_buf.prepare(size_to_read);

                m_sock.async_read_some(asio::buffer(buf.data, buf.size), [weak_this = weak_from_this()](std::error_code ec, std::size_t len) {
                    auto shared_this = weak_this.lock();
                    if (!shared_this) {
                        return;
                    }
                    
                    if (ec) {
                        shared_this->m_connection_broken_callback(shared_this.get());
                        return;
                    }

                    if (len > 0) {
                        shared_this->m_read_buf.commit(len);
                        shared_this->process_packet();
                    }
                    
                    shared_this->do_read();
                });
            }
            
            void process_packet() {
                for(;;) {
                    size_t consume_len = 0;
                    auto pack = decode_packet(m_read_buf.read_head(), m_read_buf.size(), consume_len);
                    m_read_buf.consume(consume_len);
                
                    if (!pack) {
                        break;
                    }

//                    std::cout << get_cur_time() << "    " << "recv pack, cmd = " << pack->cmd() << ", seq = " << pack->seq() << std::endl;

                    
                    m_last_recv_time = std::chrono::steady_clock::now();
                    
                    if (pack->is_rsp()) {
                        uint64_t pack_id = packet_id(pack);

                        auto it = m_waiting_for_response_requests.find(pack_id);
                        if (it != m_waiting_for_response_requests.end()) {
                            if (it->second.callback != nullptr) {
                                it->second.callback(std::move(pack));
                            }
                            m_waiting_for_response_requests.erase(it);
                        }
                    } else if (pack->is_req()) {
                        if (m_receive_req_packet_cb) {
                            auto begin = std::chrono::steady_clock::now();
                            m_receive_req_packet_cb(this, std::move(pack));
                            auto end = std::chrono::steady_clock::now();

                            auto callback_elapse = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
                            if (callback_elapse > 100) {
                                std::cout << "don't block the io thread!!!! cost time:  " << callback_elapse << std::endl;
//                                throw;
                            }
                        }
                    } else if (pack->is_push()) {
                        if (m_receive_push_packet_cb) {
                            auto begin = std::chrono::steady_clock::now();
                            m_receive_push_packet_cb(this, std::move(pack));
                            auto end = std::chrono::steady_clock::now();

                            auto callback_elapse = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
                            if (callback_elapse > 100) {
                                std::cout << "don't block the io thread!!!! cost time: " << callback_elapse << std::endl;
//                                throw;
                            }
                        }
                    }
                }
            }

            void on_check_timer() {
                auto now = std::chrono::steady_clock::now();

                for (auto it = m_waiting_for_response_requests.begin(); it != m_waiting_for_response_requests.end(); ) {
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second.begin_time).count() < it->second.timeout_mills) {
                        ++it;
                        continue;
                    }

                    if (it->second.callback != nullptr) {
                        it->second.callback(nullptr);
                    }
                    it = m_waiting_for_response_requests.erase(it);
                }

                for (auto it = m_waiting_for_sending_packets.begin(); it != m_waiting_for_sending_packets.end(); ) {
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->begin_time).count();
                    if (duration < it->timeout_mills) {
                        ++it;
                        break;
                    }

                    if (it->pack.is_req() && it->callback) {
                        it->callback(nullptr);
                    }

                    it = m_waiting_for_sending_packets.erase(it);
                }

                auto heartbeat_elapse = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_last_send_heartbeat_time).count();
                if (!m_is_server && (heartbeat_elapse > send_heartbeat_interval_ms)) {
                    send_req(build_req_packet(heartbeat_cmd), nullptr, 0);
                    m_last_send_heartbeat_time = now;
                }

                auto last_recv_elapse = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_last_recv_time).count();
                if (last_recv_elapse > active_connection_lifetime_ms) {
                    m_connection_broken_callback(this);
                }
                
                start_check_timer();
            }
        private:
            asio::io_context& m_io_context;
            bool m_is_server;
            stream_protocol::socket m_sock;

            io_buffer m_read_buf;
            io_buffer m_write_buf;
            volatile bool async_write_pending;

            connection_broken_callback_t m_connection_broken_callback;
            receive_packet_callback_t m_receive_req_packet_cb;
            receive_packet_callback_t m_receive_push_packet_cb;
            
            asio::steady_timer check_timer;
            std::chrono::steady_clock::time_point m_last_recv_time;
            std::chrono::steady_clock::time_point m_last_send_heartbeat_time;
            
            packet_list_t m_waiting_for_sending_packets;
            map_packid_2_callback_t m_waiting_for_response_requests;
    };
}
