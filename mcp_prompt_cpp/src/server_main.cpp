#include "prompt/PromptService.hpp"
#include "prompt/PromptStore.hpp"            // ← 仅这一行即可
#include "prompt/PromptTemplate.hpp"

#include "resource/ResourceService.hpp"
#include "resource/ResourceManager.hpp"
#include "resource/resolvers/LocalFileResolver.hpp"
#include "resource/resolvers/HttpResolver.hpp"

#include "tools/ToolService.hpp"
#include "tools/DemoTools.hpp"

#include "server/Dispatcher.hpp"
#include "protocol/McpCoreRequests.hpp"

#include "transport/AsioTcpTransport.hpp"    // ← 如已完成 WS，可换回 AsioWsTransport

#include <asio/steady_timer.hpp>
#include <csignal>
#include <iostream>

int main()
{
    using namespace mcp;

    /* ---------- Prompt ---------- */
    prompt::PromptStore promptStore;

    prompt::PromptTemplate echoAudio;
    echoAudio.meta.name        = "echo_audio";
    echoAudio.meta.description = "返回用户上传音频并回应";
    echoAudio.meta.arguments   = { {{"name","audio"}} };

    echoAudio.messages = {
        { protocol::Role::User,
          protocol::TextContent{ "file:///tmp/a.wav" } },
        { protocol::Role::Assistant,
          protocol::TextContent{ "已收到音频，播放完毕。" } }
    };

    promptStore.addTemplate(std::move(echoAudio));

    prompt::PromptService promptSvc(promptStore);

    /* ---------- Resources ---------- */
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
    transport::AsioTcpTransport trans(9275);  // 如果已完成 WS，可换回 AsioWsTransport
    trans.onMessage([&](const nlohmann::json& j){
        if(j.contains("method") && j["method"] == "initialize"){
            trans.send(server::handleInitialize(j));
            return;
        }
        trans.send(disp.handle(j));
    });
    trans.start();
    std::cout << "MCP server listening on :9275\n";

    /* ---------- 轮询文件变更 ---------- */
    asio::io_context io;
    asio::steady_timer timer(io);
    std::function<void(const asio::error_code&)> poll;
    poll = [&](const asio::error_code& ec){
        if(ec) return;
        rm.poll();                              // 触发 list_changed / updated
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
    return 0;
}
