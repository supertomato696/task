// src/server_main.cpp
#include "prompt/PromptService.hpp"
#incluce "prompt/SqlitePromptStore.hpp"
#include "transport/AsioTcpTransport.hpp"
#include "resource/ResouceManager.hpp"
#include <csignal>
#include <iostream>

using nlohmann::json;

int main() {
    using namespace mcp;
//    PromptStore store;
    SqlitePromptStore store("/var/lib/mcp/prompts.db");  // 新

    // 简易模板
    PromptTemplate daily{
        "daily_summary",
        "生成每日总结",
        json::parse(R"(
            [
              {"role":"system",   "content":"你是一名车载助理。"},
              {"role":"user",     "content":"今天概览：{{overview}}"},
              {"role":"assistant","content":"好的，以下是总结："}
            ])"),
        {"overview"}
    };
//    store.add(daily);

        store.add({
        "echo_audio",
        "返回用户上传的音频并回应",
        json::parse(R"(
            [
              {"role":"user", "content":"{{audio}}"},
              {"role":"assistant","content":"已收到音频，播放完毕。"}
            ])"),
        {"audio"}
    });

    PromptService svc(store);

//    json req = {
//        {"jsonrpc","2.0"},
//        {"id","1"},
//        {"method","prompts/get"},
//        {"params",{
//            {"name","daily_summary"},
//            {"arguments",{{"overview","<file://log/2025‑04‑20.txt>"}}}
//        }}
//    };
//    std::cout << svc.handle(req).dump(2) << std::endl;
     transport::AsioTcpTransport t{9275};

    /* ---------- 业务回调 ---------- */
    t.onMessage([&](const json& req){
        try{
            json resp = svc.handle(req);
            t.send(resp);
        }catch(const std::exception& e){
            json err = {
                {"jsonrpc","2.0"},
                {"id", req.value("id",0)},
                {"error", {{"code",-32603},{"message",e.what()}}}
            };
            t.send(err);
        }
    });

    t.start();
    std::cout<<"Prompt server listening on 0.0.0.0:9275\n";

    resource::ResourceManager rman;
rman.registerResolver(std::make_unique<resource::LocalFileResolver>("/var/data"));   // 根目录自定
rman.registerResolver(std::make_unique<resource::HttpResolver>());
ResourceService rsvc(rman);
PromptService   psvc(store);

transport.onMessage([&](const json& req){
    try{
        const std::string m = req.at("method");
        json resp;
        if(m.rfind("prompts/",0)==0)     resp = psvc.handle(req);
        else if(m.rfind("resources/",0)==0) resp = rsvc.handle(req);
        else if(m=="ping")                resp = json{{"jsonrpc","2.0"},{"id",req["id"]},{"result",json::object()}};
        else throw std::runtime_error("method not supported");
        transport.send(resp);
    }catch(const std::exception& e){
        transport.send({{"jsonrpc","2.0"},{"id",req.value("id",0)},{"error",{{"code",-32603},{"message",e.what()}}}});
    }
});

/* 周期轮询，示例每 2 秒 */
asio::steady_timer timer(io);
auto poll = [&](auto& self, const asio::error_code& ec){
    if(ec) return;
    rman.poll();
    timer.expires_after(std::chrono::seconds(2));
    timer.async_wait(std::bind(poll, std::ref(self), std::placeholders::_1));
};
timer.expires_after(std::chrono::seconds(2));
timer.async_wait(std::bind(poll, std::ref(poll), std::placeholders::_1));

    /* ---------- Ctrl‑C 退出 ---------- */
    std::signal(SIGINT, [](int){});
    pause();
    t.stop();
    return 0;
}