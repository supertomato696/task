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