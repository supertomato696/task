#pragma once

#include <string>
#include <map>
#include <optional>
#include <memory>
#include <future>
#include <variant>
#include <nlohmann/json.hpp>
#include "McpJsonRpc.hpp"
#include "mcp/requests/PromptRequests.hpp"
#include "mcp/requests/ResourceRequests.hpp"
#include "mcp/requests/ToolRequests.hpp"
#include "mcp/requests/CompletionRequests.hpp"
#include "mcp/requests/CoreRequests.hpp"
#include "mcp/content/PromptContent.hpp"

namespace mcp::client {

using json = nlohmann::json;

// 抽象传输接口，用于发送 JSON-RPC 消息
class Transport {
public:
    virtual ~Transport() = default;
    virtual json send(const json& request) = 0;
};

// MCP 客户端，包含 Prompt/Resource/Tools/Completion/Sampling 等模块
class McpClient {
public:
    explicit McpClient(std::shared_ptr<Transport> transport);

    // ----- Prompt -----
    requests::ListPromptsResult listPrompts(const std::optional<std::string>& cursor = std::nullopt);
    requests::GetPromptResult   getPrompt(const std::string& name,
                                          const std::optional<std::map<std::string, std::string>>& arguments = std::nullopt);
    std::future<requests::ListPromptsResult> listPromptsAsync(const std::optional<std::string>& cursor = std::nullopt);
    std::future<requests::GetPromptResult>   getPromptAsync(const std::string& name,
                                                             const std::optional<std::map<std::string, std::string>>& arguments = std::nullopt);

    // ----- Resource -----
    requests::ListResourcesResult listResources(const std::optional<std::string>& cursor = std::nullopt);
    requests::ReadResourceResult  readResource(const std::string& uri);
    std::future<requests::ListResourcesResult> listResourcesAsync(const std::optional<std::string>& cursor = std::nullopt);
    std::future<requests::ReadResourceResult>  readResourceAsync(const std::string& uri);

    // ----- Tools -----
    requests::ListToolsResult listTools(const std::optional<std::string>& cursor = std::nullopt);
    requests::CallToolResult callTool(const std::string& name, const json& arguments);
    std::future<requests::ListToolsResult> listToolsAsync(const std::optional<std::string>& cursor = std::nullopt);
    std::future<requests::CallToolResult>  callToolAsync(const std::string& name, const json& arguments);

    // ----- Completion -----
    requests::CompleteResult complete(const requests::CompletionArgument& argument,
                                      const std::variant<requests::PromptReference, requests::ResourceReference>& ref);
    std::future<requests::CompleteResult> completeAsync(const requests::CompletionArgument& argument,
                                                         const std::variant<requests::PromptReference, requests::ResourceReference>& ref);

    // ----- Sampling -----
    requests::CreateMessageResult createMessage(const requests::CreateMessageRequest::Params& params);
    std::future<requests::CreateMessageResult> createMessageAsync(const requests::CreateMessageRequest::Params& params);

private:
    std::shared_ptr<Transport> transport_;
    int nextId_ = 1;

    // 通用 JSON-RPC 调用
    json callRpc(const std::string& method, const json& params);
};

// ---------------- Implementation ----------------

inline McpClient::McpClient(std::shared_ptr<Transport> transport)
    : transport_(std::move(transport)), nextId_(1) {}

inline json McpClient::callRpc(const std::string& method, const json& params) {
    JSONRPCRequest req;
    req.jsonrpc = "2.0";
    req.id = nextId_++;
    req.method = method;
    req.params = params;

    json reqJson = req;
    json respJson = transport_->send(reqJson);

    if (respJson.contains("error")) {
        JSONRPCErrorResponse err = respJson.get<JSONRPCErrorResponse>();
        throw std::runtime_error("RPC error " + std::to_string(err.error.code) + ": " + err.error.message);
    }

    JSONRPCResponse resp = respJson.get<JSONRPCResponse>();
    return resp.result;
}

// ----- Prompt -----
inline requests::ListPromptsResult McpClient::listPrompts(const std::optional<std::string>& cursor) {
    json p = json::object();
    if (cursor) p["cursor"] = *cursor;
    json res = callRpc("prompts/list", p);
    return res.get<requests::ListPromptsResult>();
}

inline requests::GetPromptResult McpClient::getPrompt(const std::string& name,
                                                      const std::optional<std::map<std::string, std::string>>& arguments) {
    json p = json{{"name", name}};
    if (arguments) p["arguments"] = *arguments;
    json res = callRpc("prompts/get", p);
    return res.get<requests::GetPromptResult>();
}

inline std::future<requests::ListPromptsResult> McpClient::listPromptsAsync(const std::optional<std::string>& cursor) {
    return std::async(std::launch::async, [this, cursor]() { return listPrompts(cursor); });
}

inline std::future<requests::GetPromptResult> McpClient::getPromptAsync(const std::string& name,
                                                                       const std::optional<std::map<std::string, std::string>>& arguments) {
    return std::async(std::launch::async, [this, name, arguments]() { return getPrompt(name, arguments); });
}

// ----- Resource -----
inline requests::ListResourcesResult McpClient::listResources(const std::optional<std::string>& cursor) {
    json p = json::object();
    if (cursor) p["cursor"] = *cursor;
    json res = callRpc("resources/list", p);
    return res.get<requests::ListResourcesResult>();
}

inline requests::ReadResourceResult McpClient::readResource(const std::string& uri) {
    json p = json{{"uri", uri}};
    json res = callRpc("resources/read", p);
    return res.get<requests::ReadResourceResult>();
}

inline std::future<requests::ListResourcesResult> McpClient::listResourcesAsync(const std::optional<std::string>& cursor) {
    return std::async(std::launch::async, [this, cursor]() { return listResources(cursor); });
}

inline std::future<requests::ReadResourceResult> McpClient::readResourceAsync(const std::string& uri) {
    return std::async(std::launch::async, [this, uri]() { return readResource(uri); });
}

// ----- Tools -----
inline requests::ListToolsResult McpClient::listTools(const std::optional<std::string>& cursor) {
    json p = json::object();
    if (cursor) p["cursor"] = *cursor;
    json res = callRpc("tools/list", p);
    return res.get<requests::ListToolsResult>();
}

inline requests::CallToolResult McpClient::callTool(const std::string& name, const json& arguments) {
    json p = json{{"name", name}, {"arguments", arguments}};
    json res = callRpc("tools/call", p);
    return res.get<requests::CallToolResult>();
}

inline std::future<requests::ListToolsResult> McpClient::listToolsAsync(const std::optional<std::string>& cursor) {
    return std::async(std::launch::async, [this, cursor]() { return listTools(cursor); });
}

inline std::future<requests::CallToolResult> McpClient::callToolAsync(const std::string& name, const json& arguments) {
    return std::async(std::launch::async, [this, name, arguments]() { return callTool(name, arguments); });
}

// ----- Completion -----
inline requests::CompleteResult McpClient::complete(const requests::CompletionArgument& argument,
                                                    const std::variant<requests::PromptReference, requests::ResourceReference>& ref) {
    json p = json{{"argument", argument}};
    std::visit([&p](auto&& r) { p["ref"] = r; }, ref);
    json res = callRpc("completion/complete", p);
    return res.get<requests::CompleteResult>();
}

inline std::future<requests::CompleteResult> McpClient::completeAsync(const requests::CompletionArgument& argument,
                                                                      const std::variant<requests::PromptReference, requests::ResourceReference>& ref) {
    return std::async(std::launch::async, [this, argument, ref]() { return complete(argument, ref); });
}

// ----- Sampling -----
inline requests::CreateMessageResult McpClient::createMessage(const requests::CreateMessageRequest::Params& params) {
    json p = params;
    json res = callRpc("sampling/createMessage", p);
    return res.get<requests::CreateMessageResult>();
}

inline std::future<requests::CreateMessageResult> McpClient::createMessageAsync(const requests::CreateMessageRequest::Params& params) {
    return std::async(std::launch::async, [this, params]() { return createMessage(params); });
}

} // namespace mcp::client

// #pragma once

// #include <string>
// #include <map>
// #include <optional>
// #include <memory>
// #include <future>
// #include <nlohmann/json.hpp>
// #include "McpJsonRpc.hpp"
// #include "mcp/requests/PromptRequests.hpp"
// #include "mcp/requests/ResourceRequests.hpp"
// #include "mcp/requests/ToolRequests.hpp"
// #include "mcp/content/PromptContent.hpp"

// namespace mcp::client {

// using json = nlohmann::json;

// // 抽象 Transport 层，用于发送 JSON-RPC 消息
// class Transport {
// public:
//     virtual ~Transport() = default;
//     // 发送 JSON 请求，返回 JSON 响应
//     virtual json send(const json& request) = 0;
// };

// class McpClient {
// public:
//     explicit McpClient(std::shared_ptr<Transport> transport);

//     // --- Prompt 模块接口 ---
//     mcp::requests::ListPromptsResult listPrompts(
//         const std::optional<std::string>& cursor = std::nullopt
//     );

//     mcp::requests::GetPromptResult getPrompt(
//         const std::string& name,
//         const std::optional<std::map<std::string, std::string>>& arguments = std::nullopt
//     );

//     std::future<mcp::requests::ListPromptsResult> listPromptsAsync(
//         const std::optional<std::string>& cursor = std::nullopt
//     );

//     std::future<mcp::requests::GetPromptResult> getPromptAsync(
//         const std::string& name,
//         const std::optional<std::map<std::string, std::string>>& arguments = std::nullopt
//     );

//     // --- Resource 模块接口 ---
//     mcp::requests::ListResourcesResult listResources(
//         const std::optional<std::string>& cursor = std::nullopt
//     );

//     mcp::requests::ReadResourceResult readResource(
//         const std::string& uri
//     );

//     std::future<mcp::requests::ListResourcesResult> listResourcesAsync(
//         const std::optional<std::string>& cursor = std::nullopt
//     );

//     std::future<mcp::requests::ReadResourceResult> readResourceAsync(
//         const std::string& uri
//     );

//     // --- Tools 模块接口 ---
//     mcp::requests::ListToolsResult listTools(
//         const std::optional<std::string>& cursor = std::nullopt
//     );

//     mcp::requests::CallToolResult callTool(
//         const std::string& name,
//         const json& arguments
//     );

//     std::future<mcp::requests::ListToolsResult> listToolsAsync(
//         const std::optional<std::string>& cursor = std::nullopt
//     );

//     std::future<mcp::requests::CallToolResult> callToolAsync(
//         const std::string& name,
//         const json& arguments
//     );

// private:
//     std::shared_ptr<Transport> transport_;
//     int nextId_ = 1;

//     // 通用 JSON-RPC 调用封装
//     json callRpc(const std::string& method, const json& params);
// };

// // ---------------- Implementation ----------------

// inline McpClient::McpClient(std::shared_ptr<Transport> transport)
//     : transport_(std::move(transport)), nextId_(1) {}

// inline json McpClient::callRpc(const std::string& method, const json& params) {
//     JSONRPCRequest req;
//     req.jsonrpc = "2.0";
//     req.id = nextId_++;
//     req.method = method;
//     req.params = params;

//     // 序列化并发送请求
//     json reqJson = req;
//     json respJson = transport_->send(reqJson);

//     // 错误响应处理
//     if (respJson.contains("error")) {
//         JSONRPCErrorResponse errResp = respJson.get<JSONRPCErrorResponse>();
//         throw std::runtime_error("RPC error " + std::to_string(errResp.error.code)
//                                  + ": " + errResp.error.message);
//     }

//     // 成功响应
//     JSONRPCResponse resp = respJson.get<JSONRPCResponse>();
//     return resp.result;
// }

// // ----- Prompt 模块 -----
// inline mcp::requests::ListPromptsResult McpClient::listPrompts(const std::optional<std::string>& cursor) {
//     json params = json::object();
//     if (cursor) params["cursor"] = *cursor;
//     json resultJson = callRpc("prompts/list", params);
//     return resultJson.get<mcp::requests::ListPromptsResult>();
// }

// inline mcp::requests::GetPromptResult McpClient::getPrompt(
//     const std::string& name,
//     const std::optional<std::map<std::string, std::string>>& arguments
// ) {
//     json params = json{{"name", name}};
//     if (arguments) params["arguments"] = *arguments;
//     json resultJson = callRpc("prompts/get", params);
//     return resultJson.get<mcp::requests::GetPromptResult>();
// }

// inline std::future<mcp::requests::ListPromptsResult> McpClient::listPromptsAsync(
//     const std::optional<std::string>& cursor
// ) {
//     return std::async(std::launch::async, [this, cursor]() {
//         return listPrompts(cursor);
//     });
// }

// inline std::future<mcp::requests::GetPromptResult> McpClient::getPromptAsync(
//     const std::string& name,
//     const std::optional<std::map<std::string, std::string>>& arguments
// ) {
//     return std::async(std::launch::async, [this, name, arguments]() {
//         return getPrompt(name, arguments);
//     });
// }

// // ----- Resource 模块 -----
// inline mcp::requests::ListResourcesResult McpClient::listResources(const std::optional<std::string>& cursor) {
//     json params = json::object();
//     if (cursor) params["cursor"] = *cursor;
//     json resultJson = callRpc("resources/list", params);
//     return resultJson.get<mcp::requests::ListResourcesResult>();
// }

// inline mcp::requests::ReadResourceResult McpClient::readResource(const std::string& uri) {
//     json params = json{{"uri", uri}};
//     json resultJson = callRpc("resources/read", params);
//     return resultJson.get<mcp::requests::ReadResourceResult>();
// }

// inline std::future<mcp::requests::ListResourcesResult> McpClient::listResourcesAsync(
//     const std::optional<std::string>& cursor
// ) {
//     return std::async(std::launch::async, [this, cursor]() {
//         return listResources(cursor);
//     });
// }

// inline std::future<mcp::requests::ReadResourceResult> McpClient::readResourceAsync(
//     const std::string& uri
// ) {
//     return std::async(std::launch::async, [this, uri]() {
//         return readResource(uri);
//     });
// }

// // ----- Tools 模块 -----
// inline mcp::requests::ListToolsResult McpClient::listTools(const std::optional<std::string>& cursor) {
//     json params = json::object();
//     if (cursor) params["cursor"] = *cursor;
//     json resultJson = callRpc("tools/list", params);
//     return resultJson.get<mcp::requests::ListToolsResult>();
// }

// inline mcp::requests::CallToolResult McpClient::callTool(
//     const std::string& name,
//     const json& arguments
// ) {
//     json params = json{{"name", name}, {"arguments", arguments}};
//     json resultJson = callRpc("tools/call", params);
//     return resultJson.get<mcp::requests::CallToolResult>();
// }

// inline std::future<mcp::requests::ListToolsResult> McpClient::listToolsAsync(
//     const std::optional<std::string>& cursor
// ) {
//     return std::async(std::launch::async, [this, cursor]() {
//         return listTools(cursor);
//     });
// }

// inline std::future<mcp::requests::CallToolResult> McpClient::callToolAsync(
//     const std::string& name,
//     const json& arguments
// ) {
//     return std::async(std::launch::async, [this, name, arguments]() {
//         return callTool(name, arguments);
//     });
// }

// } // namespace mcp::client





#pragma once
#include "Types.hpp"
#include <asio.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <atomic>

namespace mcp::client {

/** 简单阻塞式 Client：每次请求同步等待响应；连接断掉自动重连 */
class McpClient {
public:

    /** host 可为 "127.0.0.1", port 9275 */
    McpClient(std::string host, uint16_t port);
    ~McpClient();

    std::vector<Prompt>   listPrompts();
    GetPromptResult           getPrompt(const std::string& name, const ArgMap& args = {});
public:
   std::future<std::vector<Prompt>> listPromptsAsync();
   std::future<GetPromptResult>         getPromptAsync(const std::string& name, const ArgMap& args = {});


public:
  std::future<std::vector<nlohmann::json>> listResourcesAsync();
//   std::future<nlohmann::json> readResourceAsync(const std::string& uri);

    //    std::future<nlohmann::json> listResourcesAsync();
       std::future<nlohmann::json> readResourceAsync(const std::string& uri);

public:
std::future<std::vector<nlohmann::json>> listToolsAsync(){
    return asyncRpcCall("tools/list", {}, *this);
}
std::future<nlohmann::json> callToolAsync(const std::string& name,const json& args = {}){
    return asyncRpcCall("tools/call", {{"name",name},{"arguments",args}}, *this);
}

private:
    nlohmann::json rpcCall(const std::string& method, nlohmann::json params);
    std::future<nlohmann::json> asyncRpcCall(const std::string& method, nlohmann::json params, McpClient& self);

    /* ---- TCP internals ---- */
    void start();
    void do_connect();
    void do_read();
    void send_raw(const std::string& txt);

    asio::io_context                  io_;
    asio::ip::tcp::resolver           resolver_;
    std::shared_ptr<asio::ip::tcp::socket> sock_;
    asio::streambuf                   buf_;
    const std::string                 host_;
    uint16_t                          port_;
    std::jthread                      th_;

    std::mutex                        inflight_mtx_;
    std::condition_variable           inflight_cv_;
    std::unordered_map<std::string, nlohmann::json> responses_;
        /* ------------ for async ------------- */
    struct Pending {
        std::promise<nlohmann::json> prom;
    };
    std::unordered_map<std::string, std::shared_ptr<Pending>> pendings_;

    std::atomic_uint64_t              id_gen_{1};
};

} // namespace mcp::client


优先给出 Prompt 模块 的重构和实现步骤，包括数据结构、RPC 接口、模板引擎、客户端封装及单元测试示例。
请先基于该方案在项目中进行 Prompt 部分的改造，完成后我们再按计划逐步扩展 Resource、Tools、Completion、Core 等模块。
若有任何疑问或需要调整，随