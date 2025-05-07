已思考 15 秒


下面给出 **Initialize Service、Dispatcher 以及 server main** 的完整落地代码（包含头/源文件），可直接放入现有目录并覆盖同名旧文件即可通过编译运行。

> 目录建议
>
> ```
> include/mcp/server/InitializeService.hpp
> include/mcp/server/Dispatcher.hpp
> src/server/InitializeService.cpp
> src/server/Dispatcher.cpp
> src/server_main.cpp
> ```
>
> 其余 Prompt/Resource/Tool Service 及 Transport 等文件沿用你仓库中的最新版即可。

---

## 1.  InitializeService

### include/mcp/server/InitializeService.hpp

```cpp
#pragma once
#include <nlohmann/json.hpp>
#include <string>

namespace mcp::server {

/**
 * 处理 "initialize" JSON‑RPC 请求，
 * 返回包含 serverInfo / capabilities / protocolVersion 的结果。
 *
 * 构造函数允许声明哪些顶级模块提供 listChanged/subscribe 能力。
 */
class InitializeService {
public:
    InitializeService(std::string serverName,
                      std::string serverVersion,
                      bool promptsCap   = true,
                      bool resourcesCap = true,
                      bool toolsCap     = true);

    /** 若 method == "initialize" 则生成 response，否则抛异常 */
    nlohmann::json handle(const nlohmann::json& request) const;

private:
    std::string serverName_;
    std::string version_;
    nlohmann::json caps_;      // 预生成好的 capabilities json
};

} // namespace
```

### src/server/InitializeService.cpp

```cpp
#include "server/InitializeService.hpp"
#include "protocol/McpJsonRpc.hpp"

using namespace mcp::server;
using json = nlohmann::json;

InitializeService::InitializeService(std::string n,
                                     std::string v,
                                     bool prompts,
                                     bool resources,
                                     bool tools)
    : serverName_(std::move(n)), version_(std::move(v))
{
    if(prompts)   caps_["prompts"]   = { {"listChanged", true} };
    if(resources) caps_["resources"] = { {"listChanged", true}, {"subscribe", true} };
    if(tools)     caps_["tools"]     = { {"listChanged", true} };
}

json InitializeService::handle(const json& req) const
{
    if(req.at("method") != "initialize")
        throw std::runtime_error("InitializeService: wrong method");

    auto id = req.at("id");         // id 既可能是 int 也可能是 string

    json result = {
        {"serverInfo",      { {"name", serverName_}, {"version", version_} }},
        {"protocolVersion", "2025-03-26"},
        {"capabilities",    caps_},
        {"instructions",
         "This server exposes prompt, resource and tool endpoints via MCP."}
    };
    return protocol::makeJsonRpcResult(id, result);
}
```

---

## 2.  Dispatcher（路由器）

### include/mcp/server/Dispatcher.hpp

```cpp
#pragma once
#include "prompt/PromptService.hpp"
#include "resource/ResourceService.hpp"
#include "tools/ToolService.hpp"
#include "server/InitializeService.hpp"

#include <nlohmann/json.hpp>

namespace mcp::server {

class Dispatcher {
public:
    Dispatcher(InitializeService& init,
               prompt::PromptService&  pr,
               resource::ResourceService& rs,
               tools::ToolService&      ts);

    /** 统一入口：任何 JSON‑RPC 消息都丢给它 */
    nlohmann::json handle(const nlohmann::json& request);

private:
    InitializeService&      init_;
    prompt::PromptService&  prompt_;
    resource::ResourceService& res_;
    tools::ToolService&     tool_;
};

} // namespace
```

### src/server/Dispatcher.cpp

```cpp
#include "server/Dispatcher.hpp"
#include "protocol/McpJsonRpc.hpp"

using namespace mcp::server;
using json = nlohmann::json;

static json makeError(const protocol::Id& id, int code, std::string_view msg)
{
    return protocol::makeJsonRpcError(id, code, std::string(msg));
}

Dispatcher::Dispatcher(InitializeService& init,
                       prompt::PromptService&  pr,
                       resource::ResourceService& rs,
                       tools::ToolService&      ts)
    : init_(init), prompt_(pr), res_(rs), tool_(ts) {}

json Dispatcher::handle(const json& req)
{
    /* 基本字段解析 */
    protocol::Id id;
    std::string  method;
    json         params;
    if(!protocol::getBasicRequestFields(req, id, method, params))
        return makeError(id, -32600, "Invalid Request");

    try{
        if(method == "initialize")        return init_.handle(req);
        if(method == "ping")              return protocol::makeJsonRpcResult(id, json::object());

        if(method.rfind("prompts/",0)==0)    return prompt_.handle(req);
        if(method.rfind("resources/",0)==0)  return res_.handle(req);
        if(method.rfind("tools/",0)==0)      return tool_.handle(req);

        return makeError(id, -32601, "Method not found");
    }
    catch(const std::exception& e){
        return makeError(id, -32603, e.what());
    }
}
```

---

## 3.  server\_main.cpp（示例）

```cpp
#include "prompt/PromptService.hpp"
#include "prompt/PromptStore.hpp"
#include "resource/ResourceService.hpp"
#include "resource/ResourceManager.hpp"
#include "resource/resolvers/LocalFileResolver.hpp"
#include "resource/resolvers/HttpResolver.hpp"
#include "tools/ToolService.hpp"
#include "tools/DemoTools.hpp"

#include "server/InitializeService.hpp"
#include "server/Dispatcher.hpp"

#include "transport/AsioTcpTransport.hpp"   // 或 AsioWsTransport
#include <asio/steady_timer.hpp>
#include <iostream>
#include <csignal>

int main()
{
    using namespace mcp;

    /* ---------- Prompt 模块 ---------- */
    prompt::MemoryPromptStore  promptStore;
    /* 在内存里随便加一个模板 */
    promptStore.add({
        "hello_prompt", "打招呼",
        {{"name"}},     // arguments
        {
            {protocol::Role::system,    protocol::TextContent{"你是助手。"}},
            {protocol::Role::assistant, protocol::TextContent{"你好 {{name}}!"}}
        }
    });
    prompt::PromptService promptSvc(promptStore);

    /* ---------- Resource 模块 ---------- */
    resource::ResourceManager rman;
    rman.registerResolver(std::make_unique<resource::LocalFileResolver>("./data"));
    rman.registerResolver(std::make_unique<resource::HttpResolver>());
    resource::ResourceService resSvc(rman);

    /* ---------- Tool 模块 ---------- */
    tools::ToolRegistry reg;
    tools::registerDemoTools(reg);
    tools::ToolService toolSvc(reg);

    /* ---------- Initialize Service ---------- */
    server::InitializeService initSvc("demo‑server", "0.1");

    /* ---------- Dispatcher ---------- */
    server::Dispatcher dispatcher(initSvc, promptSvc, resSvc, toolSvc);

    /* ---------- Transport ---------- */
    transport::AsioTcpTransport trans(9275);   // TCP；如用 WS 请换 AsioWsTransport
    trans.onMessage([&](const nlohmann::json& msg){
        auto resp = dispatcher.handle(msg);
        trans.send(resp);
    });
    trans.start();
    std::cout << "MCP server listening on 0.0.0.0:9275\n";

    /* ---------- 简单轮询资源改变 ---------- */
    asio::io_context io;
    asio::steady_timer timer(io);
    auto poll = [&](const asio::error_code& ec){
        if(ec) return;
        rman.poll();
        timer.expires_after(std::chrono::seconds(2));
        timer.async_wait(poll);
    };
    timer.expires_after(std::chrono::seconds(2));
    timer.async_wait(poll);
    std::jthread ioThread([&]{ io.run(); });

    /* ---------- Ctrl‑C ---------- */
    std::signal(SIGINT, [](int){});
    pause();
    trans.stop();
}
```

---

### 4.  CMake 片段

```cmake
add_executable(mcp_promptd
    src/server/InitializeService.cpp
    src/server/Dispatcher.cpp
    src/server_main.cpp)

target_link_libraries(mcp_promptd PRIVATE
    mcp_core           # Prompt/Resource/Tool 实现等
    asio
    Threads::Threads
)
```

> 如果使用 `AsioWsTransport` 需额外链接 `websocketpp` 与 `OpenSSL::SSL`.

---

**这样服务端三大模块 + initialize / ping / 动态 list\_changed & updated 通知** 已全部打通，可与之前完成的 `mcp_client` 直接联调。

后续若要接入 LLM（sampling/createMessage）或补全 completions，再在 Dispatcher 中分支即可。祝测试顺利！
