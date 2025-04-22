// src/server_main.cpp
// #include "prompt/PromptService.hpp"
// #include "prompt/SqlitePromptStore.hpp"
// #include "transport/AsioTcpTransport.hpp"
// #include "resource/ResourceService.hpp"
// #include "resource/ResourceManager.hpp"
// #include  "resource/resolvers/HttpResolver.hpp"
// #include "resource/resolvers/LocalFileResolver.hpp"
// #include <csignal>
// #include <iostream>
//
// using nlohmann::json;
//
// int main() {
//     using namespace mcp;
// //    PromptStore store;
//     SqlitePromptStore store("/var/lib/mcp/prompts.db");  // 新
//
//     // 简易模板
//     PromptTemplate daily{
//         "daily_summary",
//         "生成每日总结",
//         json::parse(R"(
//             [
//               {"role":"system",   "content":"你是一名车载助理。"},
//               {"role":"user",     "content":"今天概览：{{overview}}"},
//               {"role":"assistant","content":"好的，以下是总结："}
//             ])"),
//         {"overview"}
//     };
// //    store.add(daily);
//
//         store.add({
//         "echo_audio",
//         "返回用户上传的音频并回应",
//         json::parse(R"(
//             [
//               {"role":"user", "content":"{{audio}}"},
//               {"role":"assistant","content":"已收到音频，播放完毕。"}
//             ])"),
//         {"audio"}
//     });
//
//     PromptService svc(store);
//
// //    json req = {
// //        {"jsonrpc","2.0"},
// //        {"id","1"},
// //        {"method","prompts/get"},
// //        {"params",{
// //            {"name","daily_summary"},
// //            {"arguments",{{"overview","<file://log/2025‑04‑20.txt>"}}}
// //        }}
// //    };
// //    std::cout << svc.handle(req).dump(2) << std::endl;
//      transport::AsioTcpTransport t{9275};
//
//     /* ---------- 业务回调 ---------- */
//     t.onMessage([&](const json& req){
//         try{
//             json resp = svc.handle(req);
//             t.send(resp);
//         }catch(const std::exception& e){
//             json err = {
//                 {"jsonrpc","2.0"},
//                 {"id", req.value("id",0)},
//                 {"error", {{"code",-32603},{"message",e.what()}}}
//             };
//             t.send(err);
//         }
//     });
//
//     t.start();
//     std::cout<<"Prompt server listening on 0.0.0.0:9275\n";
//
//     resource::ResourceManager rman;
// rman.registerResolver(std::make_unique<resource::LocalFileResolver>("/var/data"));   // 根目录自定
// rman.registerResolver(std::make_unique<resource::HttpResolver>());
// ResourceService rsvc(rman);
// PromptService   psvc(store);
//
// transport.onMessage([&](const json& req){
//     try{
//         const std::string m = req.at("method");
//         json resp;
//         if(m.rfind("prompts/",0)==0)     resp = psvc.handle(req);
//         else if(m.rfind("resources/",0)==0) resp = rsvc.handle(req);
//         else if(m=="ping")                resp = json{{"jsonrpc","2.0"},{"id",req["id"]},{"result",json::object()}};
//         else throw std::runtime_error("method not supported");
//         transport.send(resp);
//     }catch(const std::exception& e){
//         transport.send({{"jsonrpc","2.0"},{"id",req.value("id",0)},{"error",{{"code",-32603},{"message",e.what()}}}});
//     }
// });
//
// /* 周期轮询，示例每 2 秒 */
// asio::steady_timer timer(io);
// auto poll = [&](auto& self, const asio::error_code& ec){
//     if(ec) return;
//     rman.poll();
//     timer.expires_after(std::chrono::seconds(2));
//     timer.async_wait(std::bind(poll, std::ref(self), std::placeholders::_1));
// };
// timer.expires_after(std::chrono::seconds(2));
// timer.async_wait(std::bind(poll, std::ref(poll), std::placeholders::_1));
//
//     /* ---------- Ctrl‑C 退出 ---------- */
//     std::signal(SIGINT, [](int){});
//     pause();
//     t.stop();
//     return 0;
// }

//
// // src/server_main.cpp
// #include "prompt/PromptService.hpp"
// #include "prompt/SqlitePromptStore.hpp"
// #include "transport/AsioTcpTransport.hpp"
// #include "resource/ResourceService.hpp"
// #include "resource/ResourceManager.hpp"
// #include "resource/resolvers/HttpResolver.hpp"
// #include "resource/resolvers/LocalFileResolver.hpp"
// #include <csignal>
// #include <iostream>
// #include <atomic>
// #include <asio/steady_timer.hpp>
// #include <chrono>
//
// using namespace mcp;
// using json = nlohmann::json;
//
// std::atomic_bool g_running{true};
//
// int main() {
//     // ================= 初始化服务 =================
//     // 初始化 Prompt 存储
//     SqlitePromptStore promptStore("/var/lib/mcp/prompts.db");
//
//     // 初始化资源管理器
//     resource::ResourceManager resourceManager;
//     resourceManager.registerResolver(std::make_unique<resource::LocalFileResolver>("/var/data"));
//     resourceManager.registerResolver(std::make_unique<resource::HttpResolver>());
//
//     // 创建业务服务
//     PromptService promptService(promptStore);
//     ResourceService resourceService(resourceManager);
//
//     // 初始化网络传输层
//     transport::AsioTcpTransport transport(9275);
//
//     // ================= 业务回调处理 =================
//     transport.onMessage([&](const json& req) {
//         try {
//             const std::string method = req.at("method");
//             json resp;
//
//             // 方法路由
//             if (method.rfind("prompts/", 0) == 0) {
//                 resp = promptService.handle(req);
//             }
//             else if (method.rfind("resources/", 0) == 0) {
//                 resp = resourceService.handle(req);
//             }
//             else if (method == "ping") {
//                 resp = {
//                     {"jsonrpc", "2.0"},
//                     {"id", req["id"]},
//                     {"result", {}}
//                 };
//             }
//             else {
//                 throw std::runtime_error("Method not supported");
//             }
//
//             transport.send(resp);
//         }
//         catch (const std::exception& e) {
//             json error = {
//                 {"jsonrpc", "2.0"},
//                 {"id", req.value("id", nullptr)},
//                 {"error", {
//                     {"code", -32603},
//                     {"message", e.what()}
//                 }}
//             };
//             transport.send(error);
//         }
//     });
//
//     // ================= 定时轮询任务 =================
//
//     asio::io_context io;
//     asio::steady_timer timer(io);
//
//     // 定义轮询函数（正确捕获方式）
//     auto poll_resources = [&](const asio::error_code& ec) {
//         if (ec || !g_running) return;
//
//         // 轮询资源变更
//         resourceManager.poll();
//
//         // 重新设置定时器
//         timer.expires_after(std::chrono::seconds(2));
//         timer.async_wait([&](const asio::error_code& e) { // ✅ 正确捕获所有需要的变量
//             poll_resources(e);
//         });
//     };
//
//     // 启动定时器
//     timer.expires_after(std::chrono::seconds(2));
//     timer.async_wait(poll_resources);
//
//     // ================= 主线程循环 =================
//     // while (g_running) {
//     //     std::this_thread::sleep_for(std::chrono::seconds(2)); // ✅ 正确的时间字面量
//     // }
//
//
//     // 启动IO上下文线程
//     std::jthread io_thread([&io]{
//         while (g_running) {
//             try {
//                 io.run();
//             }
//             catch (const std::exception& e) {
//                 std::cerr << "IO error: " << e.what() << std::endl;
//             }
//         }
//     });
//
//     // ================= 启动服务 =================
//     transport.start();
//     std::cout << "Server started on 0.0.0.0:9275\n";
//
//     // ================= 信号处理 =================
//     std::signal(SIGINT, [](int){
//         std::cout << "\nShutting down...\n";
//         g_running = false;
//     });
//
//     // 主线程等待
//     while (g_running) {
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
//
//     // ================= 清理资源 =================
//     transport.stop();
//     io.stop();
//
//     return 0;
// }

// src/server_main.cpp
#include <asio.hpp>
#include <iostream>

#include "protocol/JsonRpc.hpp"

#include "transport/AsioTcpTransport.hpp"

#include "prompt/PromptService.hpp"
#include "prompt/PromptStore.hpp"          // MemoryPromptStore; 如有 SqlitePromptStore 可换
// #include "prompt/SqlitePromptStore.hpp"

#include "resource/ResourceService.hpp"
#include "resource/ResourceManager.hpp"
#include "resource/resolvers/LocalFileResolver.hpp"
#include "resource/resolvers/HttpResolver.hpp"

#include "tools/ToolService.hpp"
#include "tools/ToolRegistry.hpp"
#include "tools/DemoTools.hpp"   //  registerDemoTools(registry)
#include "protocol/JsonRpc.hpp"

using json = nlohmann::json;
namespace proto = mcp::protocol;


int main(int /*argc*/, char** /*argv*/)
{
    using namespace mcp;

    /* ---------------- io_context & transport ---------------- */
    asio::io_context io;
    // transport::AsioTcpTransport transport(9275, io);   // 0.0.0.0:9275
    transport::AsioTcpTransport transport(9275);   // 0.0.0.0:9275

    /* ---------------- Prompt service ---------------- */
    prompt::MemoryPromptStore store;                   // 可换成 SqlitePromptStore
    store.add({
        "echo_audio",
        "返回用户上传的音频并回应",
        json::parse(R"([
            {"role":"user",      "content":"{{audio}}"},
            {"role":"assistant", "content":"已收到音频，播放完毕。"}
        ])"),
        {"audio"}
    });
    prompt::PromptService promptSvc(store);

    /* ---------------- Resource service ---------------- */
    resource::ResourceManager rman;
    rman.registerResolver(std::make_unique<resource::LocalFileResolver>("/var/data"));
    rman.registerResolver(std::make_unique<resource::HttpResolver>());
    resource::ResourceService resourceSvc(rman);

    /* ---------------- Tool service ---------------- */
    tools::ToolRegistry registry;
    tools::registerDemoTools(registry);     // Demo: say_text / car_info
    tools::ToolService toolSvc(registry);

    /* ---------------- Dispatcher ---------------- */
    transport.onMessage([&](const json& req){
        try{
            std::string method = req.at("method");
            json resp;

            if(method.rfind("prompts/",0)==0)        resp = promptSvc.handle(req);
            else if(method.rfind("resources/",0)==0) resp = resourceSvc.handle(req);
            else if(method.rfind("tools/",0)==0)     resp = toolSvc.handle(req);
            else if(method == "ping")                resp = protocol::makeResult(protocol::parseId(req));
            else                                     throw std::runtime_error("method not supported");

            transport.send(resp);
        }catch(const std::exception& e){
            transport.send(protocol::makeError(protocol::parseId(req), -32603, e.what()));
        }
    });

    transport.start();
    std::cout << "MCP server listening on 0.0.0.0:9275\n";

    /* ---------------- Periodic file‑system polling ---------------- */
    asio::steady_timer timer(io);
    std::function<void(const asio::error_code&)> poll =
        [&](const asio::error_code& ec){
            if(ec) return;
            rman.poll();                                  // emit updated callbacks
            timer.expires_after(std::chrono::seconds(2)); // every 2 s
            timer.async_wait(poll);
        };
    timer.expires_after(std::chrono::seconds(2));
    timer.async_wait(poll);

    /* ---------------- Ctrl‑C graceful shutdown ---------------- */
#if defined(SIGINT) || defined(SIGTERM)
    asio::signal_set signals(io, SIGINT, SIGTERM);
    signals.async_wait([&](auto, int){
        std::cout << "\nSignal caught, shutting down...\n";
        transport.stop();
        io.stop();
    });
#endif

    io.run();
    return 0;
}
