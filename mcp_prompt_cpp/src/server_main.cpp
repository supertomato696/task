#include "prompt/PromptService.hpp"
#include "prompt/PromptStore.hpp"          // MemoryPromptStore
#include "resource/ResourceService.hpp"
#include "resource/ResourceManager.hpp"
#include "resource/resolvers/LocalFileResolver.hpp"
#include "resource/resolvers/HttpResolver.hpp"
#include "tools/ToolService.hpp"
#include "tools/DemoTools.hpp"

#include "server/Dispatcher.hpp"
#include "protocol/McpCoreRequests.hpp"

#include "transport/AsioWsTransport.hpp"   // or AsioTcpTransport

#include <asio/steady_timer.hpp>
#include <csignal>
#include <iostream>

int main()
{
    using namespace mcp;

    /* ---------- Prompt ---------- */
    prompt::MemoryPromptStore promptStore;
    promptStore.add({
        /* key = */"echo_audio",
        /* description = */"返回用户上传音频并回应",
        /* argument schema (optional) */
        {/*名字*/"audio"}
        ,
        /* messages */
        {
            {protocol::Role::user,
                protocol::TextContent{"file:///tmp/a.wav"}},
            {protocol::Role::assistant,
                protocol::TextContent{"已收到音频，播放完毕。"}}
        }
    });

    prompt::PromptService promptSvc(promptStore);

    /* ---------- Resource ---------- */
    resource::ResourceManager rm;
    rm.registerResolver(std::make_unique<resource::LocalFileResolver>("./var/data"));
    rm.registerResolver(std::make_unique<resource::HttpResolver>());
    resource::ResourceService resSvc(rm);

    /* ---------- Tools ---------- */
    tools::ToolRegistry reg;
    tools::registerDemoTools(reg);
    tools::ToolService toolSvc(reg);

    /* ---------- Dispatcher ---------- */
    server::Dispatcher disp(promptSvc, resSvc, toolSvc);

    /* ---------- Transport ---------- */
    transport::AsioWsTransport trans(9275);      // 或 AsioTcpTransport
    trans.onMessage([&](const nlohmann::json& j){
        if(j.contains("method") && j["method"] == "initialize"){
            trans.send(server::handleInitialize(j));
            return;
        }
        trans.send(disp.handle(j));
    });
    trans.start();
    std::cout << "MCP server (WS) listening on :9275\n";

    /* ---------- 文件改动轮询推送 ---------- */
    asio::io_context io;
    asio::steady_timer timer(io);
    std::function<void(const asio::error_code&)> poll;
    poll = [&](const asio::error_code& ec){
        if(ec) return;
        rm.poll();                         // 内部触发 onChanged → resSvc broadcast
        timer.expires_after(std::chrono::seconds(2));
        timer.async_wait(poll);
    };
    timer.expires_after(std::chrono::seconds(2));
    timer.async_wait(poll);
    std::jthread ioTh([&]{ io.run(); });

    /* ---------- Ctrl‑C ---------- */
    std::signal(SIGINT, [](int){});
    pause();
    trans.stop();
}