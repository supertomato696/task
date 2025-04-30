#pragma once

#include <string>
#include <map>
#include <optional>
#include <memory>
#include <future>
#include <variant>
#include <nlohmann/json.hpp>
#include "protocol/McpJsonRpc.hpp"
#include "protocol/McpContent.hpp"
#include "protocol/McpCompletionRequests.hpp"
#include "protocol/McpToolRequests.hpp"
#include "protocol/McpResourceRequests.hpp"
#include "protocol/McpPromptRequests.hpp"
#include "protocol/McpRootsAndLoggingRequests.hpp"
#include "protocol/McpCapabilities.hpp"
#include "protocol/McpCoreRequests.hpp"
#include "protocol/McpSamplingBatch.hpp"
#include "protocol/McpMisc.hpp"
#include "protocol/McpSamplingBatch.hpp"

// #include "mcp/protocol/Promptprotocol.hpp"
// #include "mcp/protocol/Resourceprotocol.hpp"
// #include "mcp/protocol/Toolprotocol.hpp"
// #include "mcp/protocol/Completionprotocol.hpp"
// #include "mcp/protocol/Coreprotocol.hpp"
// #include "mcp/content/PromptContent.hpp"

namespace mcp::client {

using json = nlohmann::json;

// 抽象传输接口，用于发送 JSON-RPC 消息
class ITransport {
public:
    virtual ~ITransport() = default;
    virtual json send(const json& request) = 0;
};

// MCP 客户端，包含 Prompt/Resource/Tools/Completion/Sampling 等模块
class McpClient {
public:
    explicit McpClient(std::shared_ptr<ITransport> transport);

    // ----- Prompt -----
    protocol::ListPromptsResult listPrompts(const std::optional<std::string>& cursor = std::nullopt);
    protocol::GetPromptResult   getPrompt(const std::string& name,
                                          const std::optional<std::map<std::string, std::string>>& arguments = std::nullopt);
    std::future<protocol::ListPromptsResult> listPromptsAsync(const std::optional<std::string>& cursor = std::nullopt);
    std::future<protocol::GetPromptResult>   getPromptAsync(const std::string& name,
                                                             const std::optional<std::map<std::string, std::string>>& arguments = std::nullopt);

    // ----- Resource -----
    protocol::ListResourcesResult listResources(const std::optional<std::string>& cursor = std::nullopt);
    protocol::ReadResourceResult  readResource(const std::string& uri);
    std::future<protocol::ListResourcesResult> listResourcesAsync(const std::optional<std::string>& cursor = std::nullopt);
    std::future<protocol::ReadResourceResult>  readResourceAsync(const std::string& uri);

    // ----- Tools -----
    protocol::ListToolsResult listTools(const std::optional<std::string>& cursor = std::nullopt);
    protocol::CallToolResult callTool(const std::string& name, const json& arguments);
    std::future<protocol::ListToolsResult> listToolsAsync(const std::optional<std::string>& cursor = std::nullopt);
    std::future<protocol::CallToolResult>  callToolAsync(const std::string& name, const json& arguments);

    // ----- Completion -----
    protocol::CompleteResult complete(const protocol::CompletionArgument& argument,
                                      const std::variant<protocol::PromptReference, protocol::ResourceReference>& ref);
    std::future<protocol::CompleteResult> completeAsync(const protocol::CompletionArgument& argument,
                                                         const std::variant<protocol::PromptReference, protocol::ResourceReference>& ref);

    // ----- Sampling -----
    protocol::CreateMessageResult createMessage(const protocol::CreateMessageRequest::Params& params);
    std::future<protocol::CreateMessageResult> createMessageAsync(const protocol::CreateMessageRequest::Params& params);

private:
    std::shared_ptr<ITransport> transport_;
    int nextId_ = 1;

    // 通用 JSON-RPC 调用
    json callRpc(const std::string& method, const json& params);
};

// ---------------- Implementation ----------------

inline McpClient::McpClient(std::shared_ptr<ITransport> transport)
    : transport_(std::move(transport)), nextId_(1) {}

inline json McpClient::callRpc(const std::string& method, const json& params) {
    protocol::JSONRPCRequest req;
    req.jsonrpc = "2.0";
    req.id = nextId_++;
    req.method = method;
    req.params = params;

    json reqJson = req;
    json respJson = transport_->send(reqJson);

    if (respJson.contains("error")) {
        protocol::JSONRPCErrorResponse err = respJson.get<protocol::JSONRPCErrorResponse>();
        throw std::runtime_error("RPC error " + std::to_string(err.error.code) + ": " + err.error.message);
    }

    protocol::JSONRPCResponse resp = respJson.get<protocol::JSONRPCResponse>();
    return resp.result;
}

// ----- Prompt -----
inline protocol::ListPromptsResult McpClient::listPrompts(const std::optional<std::string>& cursor) {
    json p = json::object();
    if (cursor) p["cursor"] = *cursor;
    json res = callRpc("prompts/list", p);
    return res.get<protocol::ListPromptsResult>();
}

inline protocol::GetPromptResult McpClient::getPrompt(const std::string& name,
                                                      const std::optional<std::map<std::string, std::string>>& arguments) {
    json p = json{{"name", name}};
    if (arguments) p["arguments"] = *arguments;
    json res = callRpc("prompts/get", p);
    return res.get<protocol::GetPromptResult>();
}

inline std::future<protocol::ListPromptsResult> McpClient::listPromptsAsync(const std::optional<std::string>& cursor) {
    return std::async(std::launch::async, [this, cursor]() { return listPrompts(cursor); });
}

inline std::future<protocol::GetPromptResult> McpClient::getPromptAsync(const std::string& name,
                                                                       const std::optional<std::map<std::string, std::string>>& arguments) {
    return std::async(std::launch::async, [this, name, arguments]() { return getPrompt(name, arguments); });
}

// ----- Resource -----
inline protocol::ListResourcesResult McpClient::listResources(const std::optional<std::string>& cursor) {
    json p = json::object();
    if (cursor) p["cursor"] = *cursor;
    json res = callRpc("resources/list", p);
    return res.get<protocol::ListResourcesResult>();
}

inline protocol::ReadResourceResult McpClient::readResource(const std::string& uri) {
    json p = json{{"uri", uri}};
    json res = callRpc("resources/read", p);
    return res.get<protocol::ReadResourceResult>();
}

inline std::future<protocol::ListResourcesResult> McpClient::listResourcesAsync(const std::optional<std::string>& cursor) {
    return std::async(std::launch::async, [this, cursor]() { return listResources(cursor); });
}

inline std::future<protocol::ReadResourceResult> McpClient::readResourceAsync(const std::string& uri) {
    return std::async(std::launch::async, [this, uri]() { return readResource(uri); });
}

// ----- Tools -----
inline protocol::ListToolsResult McpClient::listTools(const std::optional<std::string>& cursor) {
    json p = json::object();
    if (cursor) p["cursor"] = *cursor;
    json res = callRpc("tools/list", p);
    return res.get<protocol::ListToolsResult>();
}

inline protocol::CallToolResult McpClient::callTool(const std::string& name, const json& arguments) {
    json p = json{{"name", name}, {"arguments", arguments}};
    json res = callRpc("tools/call", p);
    return res.get<protocol::CallToolResult>();
}

inline std::future<protocol::ListToolsResult> McpClient::listToolsAsync(const std::optional<std::string>& cursor) {
    return std::async(std::launch::async, [this, cursor]() { return listTools(cursor); });
}

inline std::future<protocol::CallToolResult> McpClient::callToolAsync(const std::string& name, const json& arguments) {
    return std::async(std::launch::async, [this, name, arguments]() { return callTool(name, arguments); });
}

// ----- Completion -----
inline protocol::CompleteResult McpClient::complete(const protocol::CompletionArgument& argument,
                                                    const std::variant<protocol::PromptReference, protocol::ResourceReference>& ref) {
    json p = json{{"argument", argument}};
    std::visit([&p](auto&& r) { p["ref"] = r; }, ref);
    json res = callRpc("completion/complete", p);
    return res.get<protocol::CompleteResult>();
}

inline std::future<protocol::CompleteResult> McpClient::completeAsync(const protocol::CompletionArgument& argument,
                                                                      const std::variant<protocol::PromptReference, protocol::ResourceReference>& ref) {
    return std::async(std::launch::async, [this, argument, ref]() { return complete(argument, ref); });
}

// ----- Sampling -----
inline protocol::CreateMessageResult McpClient::createMessage(const protocol::CreateMessageRequest::Params& params) {
    json p = params;
    json res = callRpc("sampling/createMessage", p);
    return res.get<protocol::CreateMessageResult>();
}

inline std::future<protocol::CreateMessageResult> McpClient::createMessageAsync(const protocol::CreateMessageRequest::Params& params) {
    return std::async(std::launch::async, [this, params]() { return createMessage(params); });
}

} // namespace mcp::client






// #pragma once
// #include "Types.hpp"
// #include <asio.hpp>
// #include <nlohmann/json.hpp>
// #include <mutex>
// #include <condition_variable>
// #include <thread>
//
// #include <atomic>
// #include <protocol/JsonRpc.hpp>
// #include <protocol/McpContent.hpp>
// #include <protocol/McpPromptprotocol.hpp>
// #include <protocol/McpCoreprotocol.hpp>
//
// namespace mcp::client {
//
// /** 简单阻塞式 Client：每次请求同步等待响应；连接断掉自动重连 */
// class McpClient {
// public:
//     using namespace  mcp::protocol;
//     /** host 可为 "127.0.0.1", port 9275 */
//     McpClient(std::string host, uint16_t port);
//     ~McpClient();
//
//     std::vector<Prompt>   listPrompts();
//     GetPromptResult           getPrompt(const std::string& name, const ArgMap& args = {});
// public:
//    std::future<std::vector<Prompt>> listPromptsAsync();
//    std::future<GetPromptResult>         getPromptAsync(const std::string& name, const ArgMap& args = {});
//
//
// public:
//   std::future<std::vector<nlohmann::json>> listResourcesAsync();
// //   std::future<nlohmann::json> readResourceAsync(const std::string& uri);
//
//     //    std::future<nlohmann::json> listResourcesAsync();
//        std::future<nlohmann::json> readResourceAsync(const std::string& uri);
//
// public:
// std::future<std::vector<nlohmann::json>> listToolsAsync(){
//     return asyncRpcCall("tools/list", {}, *this);
// }
// std::future<nlohmann::json> callToolAsync(const std::string& name,const json& args = {}){
//     return asyncRpcCall("tools/call", {{"name",name},{"arguments",args}}, *this);
// }
//
// private:
//     nlohmann::json rpcCall(const std::string& method, nlohmann::json params);
//     std::future<nlohmann::json> asyncRpcCall(const std::string& method, nlohmann::json params, McpClient& self);
//
//     /* ---- TCP internals ---- */
//     void start();
//     void do_connect();
//     void do_read();
//     void send_raw(const std::string& txt);
//
//     asio::io_context                  io_;
//     asio::ip::tcp::resolver           resolver_;
//     std::shared_ptr<asio::ip::tcp::socket> sock_;
//     asio::streambuf                   buf_;
//     const std::string                 host_;
//     uint16_t                          port_;
//     std::jthread                      th_;
//
//     std::mutex                        inflight_mtx_;
//     std::condition_variable           inflight_cv_;
//     std::unordered_map<std::string, nlohmann::json> responses_;
//         /* ------------ for async ------------- */
//     struct Pending {
//         std::promise<nlohmann::json> prom;
//     };
//     std::unordered_map<std::string, std::shared_ptr<Pending>> pendings_;
//
//     std::atomic_uint64_t              id_gen_{1};
// };
//
// } // namespace mcp::client


// 优先给出 Prompt 模块 的重构和实现步骤，包括数据结构、RPC 接口、模板引擎、客户端封装及单元测试示例。
// 请先基于该方案在项目中进行 Prompt 部分的改造，完成后我们再按计划逐步扩展 Resource、Tools、Completion、Core 等模块。
// 若有任何疑问或需要调整，随