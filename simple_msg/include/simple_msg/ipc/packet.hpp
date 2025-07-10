#pragma once
#include <memory>
#include <string>
#include <atomic>
#include <memory.h>

#include <simple_msg/detail/ibuffer.hpp>

namespace simple::msg {
#pragma pack(1)
struct packet_header {
    uint8_t         flag;
    uint32_t        cmd;
    uint32_t        seq;
    uint8_t         type;
    int32_t        ec;
    uint32_t        body_len;
};
#pragma pack()

constexpr inline const uint32_t heartbeat_cmd = 0;
constexpr inline uint8_t packet_begin_flag = 0x55;

constexpr inline uint8_t req_type = 1;
constexpr inline uint8_t rsp_type = 2;
constexpr inline uint8_t push_type = 3;

constexpr inline uint32_t header_length = sizeof(packet_header);
constexpr inline uint32_t max_body_length = 208*1024;

class packet {
    static inline std::atomic<uint32_t> seq_generator = {0};
public:
    packet() : m_cmd(0), m_seq(0), m_type(req_type), m_ec(0) {
    }
    
    packet(uint32_t cmd, uint8_t type, uint8_t* body_buf, uint32_t body_len, uint32_t seq, int32_t ec = 0)
    : m_cmd(cmd), m_seq((seq == 0)?next_seq():seq), m_type(type), m_ec(ec), m_body(body_len, body_buf) {
    }

    uint32_t cmd() const {
        return m_cmd;
    }
    
    uint32_t seq() const {
        return m_seq;
    }
    
    uint8_t type() {
        return m_type;
    }
    
    bool is_req() const {
        return m_type == req_type;
    }
    
    bool is_rsp() const {
        return m_type == rsp_type;
    }
    
    bool is_push() const {
        return m_type == push_type;
    }
    
    int32_t ec() const {
        return m_ec;
    }
 
    const ibuffer& body() const {
        return m_body;
    }
private:
    static uint32_t next_seq() {
        if (seq_generator == UINT_MAX) {
            seq_generator = 0;
        }
        
        return ++seq_generator;
    }
private:
    uint32_t        m_cmd;
    uint32_t        m_seq;
    uint8_t         m_type;
    int32_t        m_ec;
    ibuffer         m_body;
};

inline uint64_t packet_id(uint32_t cmd, uint32_t seq) {
    uint64_t id = cmd;
    id = id << 31 | seq;
    return id;
}

inline uint64_t packet_id(const std::unique_ptr<packet>& pack) {
    return packet_id(pack->cmd(), pack->seq());
}

inline uint64_t packet_id(const packet& pack) {
    return packet_id(pack.cmd(), pack.seq());
}

inline packet build_req_packet(uint32_t cmd, uint8_t* body_buf = nullptr, uint32_t body_len = 0) {
    return {cmd, req_type, body_buf, body_len, 0};
}

inline packet build_rsp_packet(uint32_t cmd, uint32_t seq, int32_t ec, uint8_t* body_buf, uint32_t body_len) {
    return {cmd, rsp_type, body_buf, body_len, seq, ec};
}

inline packet build_push_packet(uint32_t cmd, uint8_t* body_buf = nullptr, uint32_t body_len = 0) {
    return {cmd, push_type, body_buf, body_len, 0};
}

inline bool encode_packet(packet& pack, io_buffer& data_buf) {
    auto data_len = header_length + pack.body().size();
    if (data_len > data_buf.free_size()) {
        return false;
    }

    packet_header header{};
    header.flag = packet_begin_flag;
    header.cmd = pack.cmd();
    header.seq = pack.seq();
    header.ec = pack.ec();
    header.type = pack.type();
    header.body_len = pack.body().size();

    auto buf = data_buf.prepare(data_len);
    memcpy(buf.data, &header, header_length);
    if (!pack.body().empty()) {
        memcpy(buf.data + header_length, pack.body().data(), pack.body().size());
    }
    data_buf.commit(data_len);
    
    return true;
}

inline std::unique_ptr<packet> decode_packet(uint8_t* buf, size_t buf_len, size_t& consume_len) {
    consume_len = 0;
    do {
        for (; consume_len < buf_len; ++consume_len) {
            if (buf[consume_len] == packet_begin_flag) {
                break;
            }
        }
        
        uint8_t* buf_valid = buf + consume_len;
        size_t buf_valid_len = buf_len - consume_len;
        
        if (buf_valid_len < header_length) {
            return nullptr;
        }
        
        packet_header* header = (packet_header*)buf_valid;

        uint32_t body_len = header->body_len;        
        if (body_len > max_body_length) {
            consume_len += header_length;
            continue;
        }
        
        if (buf_valid_len < header_length + body_len) {
            return nullptr;
        }
        consume_len += header_length + body_len;

        uint32_t cmd = header->cmd;
        uint32_t seq = header->seq;
        int32_t ec = header->ec;
        uint8_t type = header->type;
        return std::make_unique<packet>(cmd, type, buf_valid + header_length,  body_len, seq, ec);
    } while (1);
}



    inline std::string get_cur_time() {
        return "";
    }
}
