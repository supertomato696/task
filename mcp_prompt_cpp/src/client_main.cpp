// #include "mcp/client/McpClient.hpp"
// #include <iostream>

// int main(){
//     mcp::client::McpClient cli("127.0.0.1", 9275);

//     auto list = cli.listPrompts();
//     for(auto& p:list) std::cout << p.name << "\n";

//     auto res = cli.getPrompt("echo_audio", {{"audio","file:///tmp/a.wav"}});
//     for(auto& m: res.messages){
//         std::cout << m.role << ": " << m.content.dump() << "\n";
//     }
// }


// src/client_main.cpp
#include "mcp/client/McpClient.hpp"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <variant>

int main()
{
    using namespace mcp::client;
    using protocol::Resource;
    using protocol::ResourceContents;

    /* --------------------------------------------------------
     *  1. 连接 MCP 服务器
     * ------------------------------------------------------ */
    McpClient cli{"127.0.0.1", 9275};
    std::cout << "Connected to MCP server.\n";

    /* --------------------------------------------------------
     *  2. 同步查询可用 Prompt 列表
     * ------------------------------------------------------ */
    std::cout << "\n== prompts/list ==\n";
    for (const auto& p : cli.listPrompts())
        std::cout << "  • " << p.name << "  (" << p.description << ")\n";

    /* --------------------------------------------------------
     *  3. 取一个示例 prompt 模板
     * ------------------------------------------------------ */
    try {
        auto pr = cli.getPrompt("echo_audio",
                                {{"audio","file:///tmp/sample.wav"}});
        std::cout << "\n== prompts/get echo_audio ==\n";
        for (const auto& msg : pr.messages)
            std::cout << "  [" << msg["role"] << "] "
                      << msg["content"].dump() << '\n';
    } catch (const std::exception& e){
        std::cerr << "prompts/get failed: " << e.what() << '\n';
    }

    /* --------------------------------------------------------
     *  4. 异步获取资源列表
     * ------------------------------------------------------ */
    std::cout << "\nRequesting resources/list (async)…\n";

    auto futList = cli.listResourcesAsync();

    /* 等待并展示结果 —— 这里用 get() 阻塞主线程，实际项目可放 UI 线程 */
    try {
        auto resources = futList.get();
        std::cout << "resources/list got " << resources.size() << " items\n";

        // 打印前 3 个示例
        for (size_t i = 0; i < std::min<size_t>(resources.size(), 3); ++i) {
            const Resource& r = resources[i];
            std::cout << "  [" << i << "] "
                      << std::quoted(r.name) << "  →  " << r.uri << '\n';
        }

        if (!resources.empty()) {
            /* ------------------------------------------------
             *  5. 读首个资源内容（仍然异步）
             * ---------------------------------------------- */
            auto futContent = cli.readResourceAsync(resources[0].uri);
            std::cout << "\nReading first resource: " << resources[0].uri << '\n';

            auto contents = futContent.get();
            std::cout << "  contents count = " << contents.size() << '\n';

            // 仅演示第一块内容
            std::visit([&](auto&& part){
                using T = std::decay_t<decltype(part)>;
                if constexpr(std::is_same_v<T, protocol::TextResourceContents>)
                    std::cout << "  (text, " << part.text.size() << " bytes)\n";
                else
                    std::cout << "  (blob, " << part.blob.size() << "‑base64‑bytes)\n";
            }, contents.front());
        }

    } catch (const std::exception& e){
        std::cerr << "resources async failed: " << e.what() << '\n';
    }

    /* --------------------------------------------------------
     *  6. Ctrl‑C 捕获 & 退出
     * ------------------------------------------------------ */
    std::cout << "\nPress Ctrl‑C to quit…\n";
    std::signal(SIGINT, [](int){ std::exit(0); });
    pause();
}
