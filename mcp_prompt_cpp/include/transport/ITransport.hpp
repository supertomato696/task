#pragma once
#include <functional>
#include <nlohmann/json.hpp>

namespace mcp::transport {

class ITransport {
public:
    using MessageHandler = std::function<void(const nlohmann::json&)>;

    virtual ~ITransport() = default;
    virtual void start() = 0;       // 进入事件循环（非阻塞）
    virtual void stop()  = 0;
    virtual void send(const nlohmann::json& msg) = 0;

    void onMessage(MessageHandler cb){ cb_ = std::move(cb); }
protected:
    MessageHandler cb_;
};

} // namespace mcp::transport