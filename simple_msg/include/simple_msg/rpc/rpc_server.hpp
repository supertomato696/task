#pragma once
#include <google/protobuf/message.h>

#include <simple_msg/ipc/ipc_server.hpp>
#include <simple_msg/detail/rpc_protocol.hpp>

namespace simple::msg {
        class rpc_server_t {
        public:
            explicit rpc_server_t(std::string server_name)
            : m_ipc_server(std::move(server_name)) {
            }

            void push(uint32_t method_id, const google::protobuf::Message& request_message) {
                m_ipc_server.send_push(build_push_packet(method_id, request_message));
            }

            template<typename RSP>
            void send_rsp(void* conn, uint32_t method_id, uint32_t seq, rpc_result<RSP>& rsp_message) {
                m_ipc_server.send_rsp(conn, build_response_packet(method_id, seq, rsp_message));
            }

            template<typename REQ, typename RSP>
            void subscribe_request(uint32_t method_id, std::function<rpc_result<RSP>(REQ&)> handler) {
                m_ipc_server.subscribe_req(method_id, [this, handler](void* conn, std::unique_ptr<packet> req) {
                    auto req_message = parse_body<REQ>(req);
                    if (!req_message) {
                        return;
                    }

                    auto rsp_result = handler(req_message.value());
                    m_ipc_server.send_rsp(conn, build_response_packet(req, rsp_result));
                });
            }


            template<typename REQ>
            void subscribe_request(uint32_t method_id, std::function<void(void* conn, uint32_t method_id, uint32_t seq, REQ&)> handler) {
                m_ipc_server.subscribe_req(method_id, [this, handler](void* conn, std::unique_ptr<packet> req) {
                    auto req_message = parse_body<REQ>(req);
                    if (!req_message) {
                        return;
                    }

                    handler(conn, req->cmd(), req->seq(), req_message.value());
                });
            }

            void unsubscribe_request(uint32_t method_id) {
                m_ipc_server.unsubscribe_req(method_id);
            }
        private:
            ipc_server_t m_ipc_server;
        };

}
