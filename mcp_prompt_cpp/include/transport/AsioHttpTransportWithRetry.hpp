#pragma once

#include <string>
#include <map>
#include <optional>
#include <memory>
#include <future>
#include <chrono>
#include <thread>
#include <asio.hpp>
#include <nlohmann/json.hpp>
#include "McpJsonRpc.hpp"
#include "mcp/requests/PromptRequests.hpp"
#include "mcp/requests/ResourceRequests.hpp"
#include "mcp/requests/ToolRequests.hpp"
#include "mcp/requests/CompletionRequests.hpp"
#include "mcp/requests/CoreRequests.hpp"

namespace mcp::client {

using json = nlohmann::json;

// ===== Transport 接口 =====
class Transport {
public:
    virtual ~Transport() = default;
    // 同步发送 JSON-RPC 请求
    virtual json send(const json& request) = 0;
    // 异步发送，可重写提供非阻塞 I/O
    virtual std::future<json> sendAsync(const json& request) {
        return std::async(std::launch::async, [this, request]() {
            return this->send(request);
        });
    }
};

// ===== HTTP Transport with Retry/Timeout =====
class AsioHttpTransportWithRetry : public Transport {
public:
    AsioHttpTransportWithRetry(asio::io_context& ioc,
                               std::string host,
                               std::string port = "80",
                               std::string target = "/",
                               int retry_count = 3,
                               std::chrono::milliseconds timeout = std::chrono::seconds(5))
        : io_context_(ioc), host_(std::move(host)), port_(std::move(port)), target_(std::move(target)),
          retry_count_(retry_count), timeout_(timeout) {}

    json send(const json& request) override {
        for (int attempt = 0; attempt <= retry_count_; ++attempt) {
            try { return doSendOnce(request); }
            catch (const std::exception& e) {
                if (attempt == retry_count_) throw;
                std::this_thread::sleep_for(std::chrono::milliseconds(100 * (1 << attempt)));
            }
        }
        throw std::runtime_error("All retry attempts failed");
    }

private:
    json doSendOnce(const json& request) {
        std::string body = request.dump();
        asio::ip::tcp::resolver resolver(io_context_);
        auto endpoints = resolver.resolve(host_, port_);
        asio::ip::tcp::socket socket(io_context_);

        asio::steady_timer timer(io_context_);
        bool timed_out = false;
        timer.expires_after(timeout_);
        timer.async_wait([&](const asio::error_code& ec) {
            if (!ec) { timed_out = true; socket.cancel(); }
        });

        asio::connect(socket, endpoints);
        if (timed_out) throw std::runtime_error("Connect timed out");

        std::ostringstream req;
        req << "POST " << target_ << " HTTP/1.1\r\n"
            << "Host: " << host_ << "\r\n"
            << "Content-Type: application/json\r\n"
            << "Content-Length: " << body.size() << "\r\n"
            << "Connection: close\r\n\r\n"
            << body;
        asio::write(socket, asio::buffer(req.str()));
        if (timed_out) throw std::runtime_error("Write timed out");

        asio::streambuf buf;
        asio::read_until(socket, buf, "\r\n\r\n");
        if (timed_out) throw std::runtime_error("Read header timed out");

        std::istream response(&buf);
        std::string status;
        std::getline(response, status);
        if (status.find("200") == std::string::npos) throw std::runtime_error("HTTP error: " + status);
        std::string header;
        while (std::getline(response, header) && header != "\r");

        std::ostringstream body_ss;
        if (buf.size() > 0) body_ss << &buf;
        std::error_code ec;
        while (asio::read(socket, buf.prepare(512), ec)) {
            buf.commit(512);
            body_ss << &buf;
        }
        timer.cancel();
        return json::parse(body_ss.str());
    }

    asio::io_context& io_context_;
    std::string host_, port_, target_;
    int retry_count_;
    std::chrono::milliseconds timeout_;
};

// ===== McpClient 客户端 =====
class McpClient {
public:
    using NotificationCb = std::function<void(const json& params)>;

    explicit McpClient(std::shared_ptr<Transport> transport)
        : transport_(std::move(transport)), nextId_(1) {}

    // 注册通知回调
    void onNotification(const std::string& method, NotificationCb cb) {
        notifCbs_[method] = std::move(cb);
    }

    // 派发通知
    void dispatchNotification(const json& msg) {
        auto m = msg.value("method", "");
        if (auto it = notifCbs_.find(m); it != notifCbs_.end()) {
            it->second(msg.value("params", json::object()));
        }
    }

    // ===== 同步接口 =====
    requests::ListPromptsResult   listPrompts(const std::optional<std::string>& cursor = {});
    requests::GetPromptResult     getPrompt(const std::string& name,
                                             const std::optional<std::map<std::string, std::string>>& args = {});
    requests::ListResourcesResult listResources(const std::optional<std::string>& cursor = {});
    requests::ReadResourceResult  readResource(const std::string& uri);
    requests::ListToolsResult     listTools(const std::optional<std::string>& cursor = {});
    requests::CallToolResult      callTool(const std::string& name, const json& arguments);
    requests::CompleteResult      complete(const requests::CompletionArgument& arg,
                                           const std::variant<requests::PromptReference, requests::ResourceReference>& ref);
    requests::CreateMessageResult createMessage(const requests::CreateMessageRequest::Params& params);

    // ===== 异步接口 =====
    std::future<requests::ListPromptsResult>   listPromptsAsync(const std::optional<std::string>& cursor = {});
    std::future<requests::GetPromptResult>     getPromptAsync(const std::string& name,
                                                               const std::optional<std::map<std::string, std::string>>& args = {});
    std::future<requests::ListResourcesResult> listResourcesAsync(const std::optional<std::string>& cursor = {});
    std::future<requests::ReadResourceResult>  readResourceAsync(const std::string& uri);
    std::future<requests::ListToolsResult>     listToolsAsync(const std::optional<std::string>& cursor = {});
    std::future<requests::CallToolResult>      callToolAsync(const std::string& name, const json& arguments);
    std::future<requests::CompleteResult>      completeAsync(const requests::CompletionArgument& arg,
                                                               const std::variant<requests::PromptReference, requests::ResourceReference>& ref);
    std::future<requests::CreateMessageResult> createMessageAsync(const requests::CreateMessageRequest::Params& params);

private:
    std::shared_ptr<Transport> transport_;
    int nextId_;
    std::map<std::string, NotificationCb> notifCbs_;

    // 同步 RPC
    json callRpc(const std::string& method, const json& params);
    // 异步 RPC
    std::future<json> callRpcAsync(const std::string& method, const json& params);
};

// --- 实现 ---
inline json McpClient::callRpc(const std::string& method, const json& params) {
    JSONRPCRequest req{.jsonrpc="2.0", .id=nextId_++, .method=method, .params=params};
    auto resp = transport_->send(json(req));
    if (resp.contains("error")) {
        auto err = resp.get<JSONRPCErrorResponse>();
        throw std::runtime_error("RPC error " + std::to_string(err.error.code) + ": " + err.error.message);
    }
    return resp.get<JSONRPCResponse>().result;
}

inline std::future<json> McpClient::callRpcAsync(const std::string& method, const json& params) {
    JSONRPCRequest req{.jsonrpc="2.0", .id=nextId_++, .method=method, .params=params};
    return transport_->sendAsync(json(req)).then([&](std::future<json> fut) {
        auto resp = fut.get();
        if (resp.contains("error")) {
            auto err = resp.get<JSONRPCErrorResponse>();
            throw std::runtime_error("RPC error " + std::to_string(err.error.code) + ": " + err.error.message);
        }
        return resp.get<JSONRPCResponse>().result;
    });
}

#define MCP_SYNC_METHOD(Method, RPC) \
inline auto McpClient::Method { return callRpc(RPC, params).get<decltype(std::declval<McpClient>().Method)>(); }

// 同步方法展开
inline requests::ListPromptsResult   McpClient::listPrompts(const std::optional<std::string>& cursor) {
    json p = json::object(); if(cursor) p["cursor"]=*cursor;
    return callRpc("prompts/list", p).get<requests::ListPromptsResult>();
}
inline requests::GetPromptResult     McpClient::getPrompt(const std::string& name,
                                                          const std::optional<std::map<std::string, std::string>>& args) {
    json p = {{"name", name}}; if(args) p["arguments"]=*args;
    return callRpc("prompts/get", p).get<requests::GetPromptResult>();
}
inline requests::ListResourcesResult McpClient::listResources(const std::optional<std::string>& cursor) {
    json p = json::object(); if(cursor) p["cursor"]=*cursor;
    return callRpc("resources/list", p).get<requests::ListResourcesResult>();
}
inline requests::ReadResourceResult  McpClient::readResource(const std::string& uri) {
    json p = {{"uri", uri}};
    return callRpc("resources/read", p).get<requests::ReadResourceResult>();
}
inline requests::ListToolsResult     McpClient::listTools(const std::optional<std::string>& cursor) {
    json p = json::object(); if(cursor) p["cursor"]=*cursor;
    return callRpc("tools/list", p).get<requests::ListToolsResult>();
}
inline requests::CallToolResult      McpClient::callTool(const std::string& name, const json& args) {
    json p = {{"name", name}, {"arguments", args}};
    return callRpc("tools/call", p).get<requests::CallToolResult>();
}
inline requests::CompleteResult      McpClient::complete(const requests::CompletionArgument& arg,
                                                          const std::variant<requests::PromptReference, requests::ResourceReference>& ref) {
    json p = {{"argument", arg}}; std::visit([&](auto&& r){p["ref"]=r;}, ref);
    return callRpc("completion/complete", p).get<requests::CompleteResult>();
}
inline requests::CreateMessageResult McpClient::createMessage(const requests::CreateMessageRequest::Params& params) {
    json p = params;
    return callRpc("sampling/createMessage", p).get<requests::CreateMessageResult>();
}

// 异步方法展开
inline std::future<requests::ListPromptsResult>   McpClient::listPromptsAsync(const std::optional<std::string>& cursor) {
    json p = json::object(); if(cursor) p["cursor"]=*cursor;
    return callRpcAsync("prompts/list", p).then([](std::future<json> f){return f.get().get<requests::ListPromptsResult>();});
}
inline std::future<requests::GetPromptResult>     McpClient::getPromptAsync(const std::string& name,
                                                                                  const std::optional<std::map<std::string, std::string>>& args) {
    json p = {{"name", name}}; if(args) p["arguments"]=*args;
    return callRpcAsync("prompts/get", p).then([](std::future<json> f){return f.get().get<requests::GetPromptResult>();});
}
inline std::future<requests::ListResourcesResult> McpClient::listResourcesAsync(const std::optional<std::string>& cursor) {
    json p = json::object(); if(cursor) p["cursor"]=*cursor;
    return callRpcAsync("resources/list", p).then([](std::future<json> f){return f.get().get<requests::ListResourcesResult>();});
}
inline std::future<requests::ReadResourceResult>  McpClient::readResourceAsync(const std::string& uri) {
    json p = {{"uri", uri}};
    return callRpcAsync("resources/read", p).then([](std::future<json> f){return f.get().get<requests::ReadResourceResult>();});
}
inline std::future<requests::ListToolsResult>     McpClient::listToolsAsync(const std::optional<std::string>& cursor) {
    json p = json::object(); if(cursor) p["cursor"]=*cursor;
    return callRpcAsync("tools/list", p).then([](std::future<json> f){return f.get().get<requests::ListToolsResult>();});
}
inline std::future<requests::CallToolResult>      McpClient::callToolAsync(const std::string& name, const json& args) {
    json p = {{"name", name}, {"arguments", args}};
    return callRpcAsync("tools/call", p).then([](std::future<json> f){return f.get().get<requests::CallToolResult>();});
}
inline std::future<requests::CompleteResult>      McpClient::completeAsync(const requests::CompletionArgument& arg,
                                                                                  const std::variant<requests::PromptReference, requests::ResourceReference>& ref) {
    json p = {{"argument", arg}}; std::visit([&](auto&& r){p["ref"]=r;}, ref);
    return callRpcAsync("completion/complete", p).then([](std::future<json> f){return f.get().get<requests::CompleteResult>();});
}
inline std::future<requests::CreateMessageResult> McpClient::createMessageAsync(const requests::CreateMessageRequest::Params& params) {
    json p = params;
    return callRpcAsync("sampling/createMessage", p).then([](std::future<json> f){return f.get().get<requests::CreateMessageResult>();});
}

} // namespace mcp::client
