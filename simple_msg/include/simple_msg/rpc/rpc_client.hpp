#pragma once
#include <optional>
#include <future>

#include <simple_msg/ipc/ipc_client.hpp>
#include <simple_msg/detail/rpc_protocol.hpp>
#include <google/protobuf/message.h>

namespace simple::msg {
        class rpc_client_t {
            constexpr inline static uint32_t default_timeout_mills = 3 * 1000;
        public:
            explicit rpc_client_t(std::string server_name, uint32_t timeout_mills = default_timeout_mills)
            : m_ipc_client(std::move(server_name))
            , m_timeout_mills(timeout_mills) {
            }

            template<typename T>
            rpc_result<T> call(uint32_t method_id, const google::protobuf::Message& request_message) {
                auto req = build_request_packet(method_id, request_message);

                std::promise<rpc_result<T>> p;
                auto f = p.get_future();
                m_ipc_client.send_req(std::move(req), [this, &p](std::unique_ptr<packet> rsp) {
                    p.set_value(parse_body<T>(rsp));
                }, m_timeout_mills);

                return f.get();
            }

            template<typename T>
            void call(uint32_t method_id, const google::protobuf::Message& request_message, std::function<void(rpc_result<T>)> callback) {
                auto req = build_request_packet(method_id, request_message);

                m_ipc_client.send_req(std::move(req), [this, callback](std::unique_ptr<packet> rsp) {
                    callback(parse_body<T>(rsp));
                }, m_timeout_mills);
            }
            
            template<typename T>
            void subscribe_push(uint32_t method_id, std::function<void(T&)> callback) {
                m_ipc_client.subscribe_push(method_id, [this, callback](std::unique_ptr<packet> rsp) {
                    auto result = parse_body<T>(rsp);
                    if (!result) {
                        return;
                    }

                    callback(result.value());
                });
            }

            void unregister_handler(uint32_t method_id) {
                m_ipc_client.unsubscribe_push(method_id);
            }
        private:
            ipc_client_t m_ipc_client;
            uint32_t m_timeout_mills;
        };
}
