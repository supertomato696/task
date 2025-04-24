// examples/demo.cpp
#include <iostream>
#include <thread>
#include <asio.hpp>
#include <nlohmann/json.hpp>

#include "WebSocketTransport.hpp"
#include "McpClientSubscriptions.hpp"
#include "ResourceServiceModules.hpp"
#include "TemplateEngine.hpp"
#include "MultimodalAssembler.hpp"

using namespace mcp::client;
using namespace mcp::resource;
using namespace mcp::utils;
using json = nlohmann::json;

int main() {
    try {
        // 1. Asio I/O context
        asio::io_context ioc;

        // 2. ResourceService: register file and HTTP resolvers
        auto resourceService = std::make_shared<ResourceService>();
        resourceService->addResolver(std::make_shared<FileResolver>());
        resourceService->addResolver(std::make_shared<HttpResolver>(ioc));

        // 3. WebSocketTransport with notification handler (placeholder)
        auto wsTransport = std::make_shared<WebSocketTransport>(
            "ws://localhost:8080/mcp",
            [&](const json& msg) {
                // will be set to dispatch into client below
            }
        );

        // 4. MCP client with subscriptions
        McpClientSubscribable client(wsTransport);
        // hook transport notifications into client
        wsTransport->setMessageHandler([&](const json& msg){
            client.dispatchNotification(msg);
        });

        // 5. Subscribe to notifications
        client.subscribePromptsListChanged();
        client.subscribeToolsListChanged();
        client.subscribeResource("file://path/to/config.json");

        // 6. List and get prompts
        auto listRes = client.listPrompts();
        std::cout << "Available prompts:\n";
        for (auto& p : listRes.prompts) {
            std::cout << " - " << p.name << "\n";
        }

        // 7. Get a prompt with arguments
        std::map<std::string, std::string> args = {{"user_name","Alice"}};
        auto getRes = client.getPrompt(listRes.prompts[0].name, args);

        // 8. Assemble messages (render + load resources)
        MultimodalAssembler assembler(resourceService);
        auto assembled = assembler.assemble(getRes.messages, args);

        // 9. Send to LLM via sampling
        requests::CreateMessageRequest::Params sampParams;
        sampParams.messages    = assembled;
        sampParams.maxTokens   = 256;
        sampParams.temperature = 0.7;

        auto sampRes = client.createMessage(sampParams);
        std::cout << "LLM response: " << sampRes.content.text << "\n";

        // 10. Keep the program alive to receive async notifications
        std::cout << "Listening for updates... Press Ctrl+C to exit.\n";
        ioc.run();

    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}
