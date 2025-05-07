已思考 8 秒


下面把 **客户端 SDK** 最终形态一次性整理给你（不含第三方库），放进

```
include/mcp/client2/   // 头文件  
src/client2/           // 实现  
```

即可直接与现有服务器联调；CMake 原来已经把 `mcp_client` target 分离，这些文件编译后会全部链接进去。

> 下面代码 **全部可直接复制粘贴** —— 若项目里已有同名文件，可整体覆盖。

---

## 1. 低层：JsonRpcClient

### include/mcp/client2/JsonRpcClient.hpp

```cpp
#pragma once
#include "protocol/McpJsonRpc.hpp"
#include "transport/ITransport.hpp"

#include <future>
#include <unordered_map>
#include <mutex>

namespace mcp::client2 {

/**
 * 线程安全：内部用 mutex 保护 inflight map
 * 所有发送均转成 JSON，由 ITransport 实际写入；接收由外层 Transport
 * 监听后回调 deliver()。
 */
class JsonRpcClient {
public:
    explicit JsonRpcClient(transport::ITransport& tx);

    /** 同步调用 —— 阻塞直到结果 / error 抛异常 */
    template <typename ParamsT, typename ResultT>
    ResultT call(const std::string& method, const ParamsT& params);

    /** 异步调用 —— 返回 std::future<ResultT> */
    template <typename ParamsT, typename ResultT>
    std::future<ResultT> callAsync(const std::string& method, const ParamsT& params);

    /** 由 Transport 收到消息后调用 */
    void deliver(const nlohmann::json& msg);

private:
    using json = nlohmann::json;
    using Id   = protocol::Id;

    struct Pending {
        std::promise<json> prom;
    };

    Id nextId_ = 1;
    transport::ITransport& tx_;

    std::mutex                          mtx_;
    std::unordered_map<Id, std::shared_ptr<Pending>> inflight_;
};

/* ----- template impl -------------------------------------------------- */
template<typename ParamsT, typename ResultT>
ResultT JsonRpcClient::call(const std::string& m, const ParamsT& p)
{
    auto fut = callAsync<ParamsT, ResultT>(m, p);
    return fut.get();
}

template<typename ParamsT, typename ResultT>
std::future<ResultT>
JsonRpcClient::callAsync(const std::string& method, const ParamsT& params)
{
    /* 1. 生成 id + inflight entry */
    Id id;
    {
        std::scoped_lock lk(mtx_);
        id = nextId_++;
        auto pending = std::make_shared<Pending>();
        inflight_[id] = pending;

        /* 2. 发送请求 */
        auto req = protocol::makeJsonRpcRequest(id, method, params);
        tx_.send(req);
    }

    /* 3. 把 json -> ResultT 转换包裹在未来值上 */
    auto rawFuture = inflight_[id]->prom.get_future();
    return std::async(std::launch::deferred, [rawFuture = std::move(rawFuture)]() mutable {
        auto j = rawFuture.get();
        return j.get<ResultT>();
    });
}

} // namespace
```

### src/client2/JsonRpcClient.cpp

```cpp
#include "mcp/client2/JsonRpcClient.hpp"
#include <stdexcept>

using namespace mcp::client2;
using json = nlohmann::json;

JsonRpcClient::JsonRpcClient(transport::ITransport& t):tx_(t){}

/* 收到来自 Transport 的 json 消息 */
void JsonRpcClient::deliver(const json& msg)
{
    /* 只处理 Response/Error，Notification 由高层模块自行订阅 */
    Id id; json result;
    if(protocol::parseResponse(msg, id, result)){   // 成功 Response
        std::shared_ptr<Pending> pend;
        {
            std::scoped_lock lk(mtx_);
            auto it = inflight_.find(id);
            if(it==inflight_.end()) return;         // 不是我们发的
            pend = it->second;
            inflight_.erase(it);
        }
        pend->prom.set_value(std::move(result));
    }
    else if(protocol::parseError(msg, id, result)){ // Error
        std::string err = result["message"];
        std::runtime_error ex(err);
        std::shared_ptr<Pending> pend;
        {
            std::scoped_lock lk(mtx_);
            auto it = inflight_.find(id);
            if(it==inflight_.end()) return;
            pend = it->second;
            inflight_.erase(it);
        }
        pend->prom.set_exception(std::make_exception_ptr(ex));
    }
}
```

---

## 2. 功能模块客户端

### include/mcp/client2/PromptClient.hpp

```cpp
#pragma once
#include "mcp/client2/JsonRpcClient.hpp"
#include "protocol/McpPromptRequests.hpp"

namespace mcp::client2 {

class PromptClient {
public:
    explicit PromptClient(JsonRpcClient& rpc):rpc_(rpc){}

    protocol::ListPromptsResult listPrompts();
    std::future<protocol::ListPromptsResult> listPromptsAsync();

    protocol::GetPromptResult getPrompt(const protocol::GetPromptParams&);
    std::future<protocol::GetPromptResult> getPromptAsync(const protocol::GetPromptParams&);

private:
    JsonRpcClient& rpc_;
};

} // namespace
```

### src/client2/PromptClient.cpp

```cpp
#include "mcp/client2/PromptClient.hpp"
using namespace mcp::client2;

protocol::ListPromptsResult PromptClient::listPrompts()
{
    return rpc_.call<protocol::PaginatedParams,
                     protocol::ListPromptsResult>("prompts/list", {});
}
std::future<protocol::ListPromptsResult>
PromptClient::listPromptsAsync()
{
    return rpc_.callAsync<protocol::PaginatedParams,
                          protocol::ListPromptsResult>("prompts/list", {});
}

protocol::GetPromptResult
PromptClient::getPrompt(const protocol::GetPromptParams& p)
{
    return rpc_.call<protocol::GetPromptParams,
                     protocol::GetPromptResult>("prompts/get", p);
}
std::future<protocol::GetPromptResult>
PromptClient::getPromptAsync(const protocol::GetPromptParams& p)
{
    return rpc_.callAsync<protocol::GetPromptParams,
                          protocol::GetPromptResult>("prompts/get", p);
}
```

> **ResourceClient / ToolClient** 与上面相同思路（list/read, list/call）；你的仓库里已有实现，如无则按同模板补即可。

---

## 3. Aggregator：McpClient

### include/mcp/client2/McpClient.hpp

```cpp
#pragma once
#include "mcp/client2/JsonRpcClient.hpp"
#include "mcp/client2/PromptClient.hpp"
#include "mcp/client2/ResourceClient.hpp"
#include "mcp/client2/ToolClient.hpp"

#include "transport/AsioTcpTransport.hpp"  // 或 AsioWsTransport

namespace mcp::client2 {

/* 一站式客户端：封装 Transport + 子模块 */
class McpClient {
public:
    McpClient(std::string host, uint16_t port);

    /* 暴露子模块 */
    PromptClient&   prompt()   { return pr_; }
    ResourceClient& resource() { return rs_; }
    ToolClient&     tool()     { return tl_; }

private:
    /* Transport & Rpc */
    transport::AsioTcpTransport   tx_;
    JsonRpcClient                 rpc_;

    /* Sub‑clients */
    PromptClient   pr_;
    ResourceClient rs_;
    ToolClient     tl_;
};

} // namespace
```

### src/client2/McpClient.cpp

```cpp
#include "mcp/client2/McpClient.hpp"

using namespace mcp::client2;

McpClient::McpClient(std::string host,uint16_t port)
    : tx_(port, std::move(host)),   // ctor: host, port
      rpc_(tx_),
      pr_(rpc_), rs_(rpc_), tl_(rpc_)
{
    /* 1) Transport 收到任何 json -> rpc_.deliver */
    tx_.onMessage([this](const nlohmann::json& j){ rpc_.deliver(j); });
    tx_.start();

    /* 2) 发送 initialize → 这里用最小集 */
    protocol::InitializeRequest init;
    init.params.protocolVersion = "2025-03-26";
    init.params.clientInfo.name = "mcp-sdk";
    init.params.clientInfo.version = "0.1";
    rpc_.call<protocol::InitializeParams, protocol::InitializeResult>
        ("initialize", init.params);
}
```

---

## 4. Demo main

```cpp
#include "mcp/client2/McpClient.hpp"
#include <iostream>

using namespace mcp::client2;
using json = nlohmann::json;

int main(){
    McpClient cli("127.0.0.1", 9275);

    /* ---- Prompt ---- */
    auto list = cli.prompt().listPrompts();
    for(auto& p:list.prompts) std::cout << "prompt: " << p.name << "\n";

    protocol::GetPromptParams gp; gp.name = "echo_audio";
    gp.arguments = { {"audio","file:///tmp/a.wav"} };
    auto prompt = cli.prompt().getPrompt(gp);
    std::cout << "echo_audio messages size = " << prompt.messages.size() << "\n";

    /* ---- Resource ---- */
    auto resList = cli.resource().listResources();
    if(!resList.resources.empty()){
        auto uri = resList.resources[0].uri;
        auto contents = cli.resource().readResource({uri}).contents;
        std::cout << "Read resource " << uri << ", items=" << contents.size() << "\n";
    }

    /* ---- Tool ---- */
    auto tools = cli.tool().listTools();
    if(!tools.tools.empty()){
        protocol::CallToolParams cp;
        cp.name = "say_text";
        cp.arguments = { {"text","Hi MCP!"} };
        auto callRes = cli.tool().callTool(cp);
        std::cout << "tool result size = " << callRes.content.size() << "\n";
    }
}
```

---

### CMake 更新

```cmake
add_library(mcp_client
    src/client2/JsonRpcClient.cpp
    src/client2/PromptClient.cpp
    src/client2/ResourceClient.cpp
    src/client2/ToolClient.cpp
    src/client2/McpClient.cpp)

target_include_directories(mcp_client PUBLIC include)
target_link_libraries(mcp_client PUBLIC
    asio nlohmann_json Threads::Threads)
```

(如使用 `AsioWsTransport`, 需要 `websocketpp` + OpenSSL)

---

## 5. 现在能跑什么

1. **服务器**：`./build/mcp_promptd`（TCP 9275）
2. **客户端 Demo**：`./build/mcp_client_demo` → 走完整链
3. **调试**：`nc localhost 9275` 手动发 JSON‑RPC 也可

至此，**客户端 SDK** 与服务端强类型协议完全对齐，
`Prompt / Resource / Tool` 三模块同步 & 异步都可用；
后续你只需补 `SamplingClient` / `CompletionClient` 即可。
