#pragma once

#include <string_view>
#include <optional>

#include <simple_msg/ipc/packet.hpp>
#include <simple_msg/rpc/err_code.hpp>

#include <google/protobuf/message.h>


namespace simple::msg {
    inline packet build_push_packet(uint32_t method_id, const google::protobuf::Message& message) {
        uint32_t size = (uint32_t)message.ByteSizeLong();
        ibuffer body{size};
        message.SerializeToArray(body.data(), (int)size);

        return build_push_packet(method_id, body.data(), body.size());
    }

    inline packet build_request_packet(uint32_t method_id, const google::protobuf::Message& message) {
        uint32_t size = (uint32_t)message.ByteSizeLong();
        ibuffer body{size};
        message.SerializeToArray(body.data(), (int)size);

        return build_req_packet(method_id, body.data(), body.size());
    }

    template<typename RSP>
    packet build_response_packet(std::unique_ptr<packet>& req, rpc_result<RSP>& rsp_message) {
        if (!rsp_message) {
            return build_rsp_packet(req->cmd(), req->seq(), rsp_message.error(), nullptr, 0);
        }

        auto& proto_message = rsp_message.value();
        auto size = (uint32_t)proto_message.ByteSizeLong();
        ibuffer body{size};
        proto_message.SerializeToArray(body.data(), (int)size);

        return build_rsp_packet(req->cmd(), req->seq(), 0, body.data(), body.size());
    }

    template<typename RSP>
    packet build_response_packet(uint32_t method_id, uint32_t seq, rpc_result<RSP>& rsp_message) {
        if (!rsp_message) {
            return build_rsp_packet(method_id, seq, rsp_message.error(), nullptr, 0);
        }

        auto& proto_message = rsp_message.value();
        auto size = (uint32_t)proto_message.ByteSizeLong();
        ibuffer body{size};
        proto_message.SerializeToArray(body.data(), (int)size);

        return build_rsp_packet(method_id, seq, 0, body.data(), body.size());
    }

    template<typename T>
    rpc_result<T> parse_body(std::unique_ptr<packet>& pack) {
        if (pack == nullptr) {
            return rpc_unexpected_result{err_network};
        }
        auto ec = pack->ec();
        if (ec != 0) {
            return rpc_unexpected_result{ec};
        }

        T message;
        if (!message.ParseFromArray(pack->body().data(), pack->body().size())) {
            return rpc_unexpected_result{err_deserialize};
        }

        return rpc_result<T>{message};
    }

}

