#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <map>
#include <functional>
#include <nlohmann/json.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <asio/asio.hpp>

namespace mcp::client {

using json = nlohmann::json;
using ws_client = websocketpp::client<websocketpp::config::asio_client>;
using connection_hdl = websocketpp::connection_hdl;

/// WebSocketTransport: 双向 JSON-RPC over WebSocket
class WebSocketTransport : public Transport {
public:
    /// 回调用于分发通知和RPC响应
    using MessageHandler = std::function<void(const json&)>;

    WebSocketTransport(const std::string& uri, MessageHandler handler)
        : ws_client_(), handler_(std::move(handler)), uri_(uri) {
        ws_client_.init_asio();
        ws_client_.set_message_handler(
            [this](connection_hdl hdl, ws_client::message_ptr msg) {
                onMessage(msg->get_payload());
            }
        );
        websocketpp::lib::error_code ec;
        auto con = ws_client_.get_connection(uri_, ec);
        if (ec) throw std::runtime_error("WS connect init failed: " + ec.message());
        hdl_ = con->get_handle();
        ws_client_.connect(con);
        // 在后台线程运行
        thread_ = std::thread([this]{ ws_client_.run(); });
    }

    ~WebSocketTransport() {
        ws_client_.stop();
        if (thread_.joinable()) thread_.join();
    }

    // 同步send通过异步实现
    nlohmann::json send(const nlohmann::json& request) override {
        return sendAsync(request).get();
    }

    // 异步发送：将promise存储并发送文本帧
    std::future<json> sendAsync(const json& request) override {
        int id = request.value("id", -1);
        std::promise<json> prom;
        auto fut = prom.get_future();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            promises_[id] = std::move(prom);
        }
        ws_client_.send(hdl_, request.dump(), websocketpp::frame::opcode::text);
        return fut;
    }

private:
    void onMessage(const std::string& payload) {
        auto j = json::parse(payload);
        if (j.contains("id")) {
            int id = j["id"].get<int>();
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = promises_.find(id);
            if (it != promises_.end()) {
                it->second.set_value(j);
                promises_.erase(it);
            }
        } else if (j.contains("method")) {
            // 通知消息
            handler_(j);
        }
    }

    ws_client ws_client_;
    connection_hdl hdl_;
    std::thread thread_;
    std::mutex mutex_;
    std::map<int, std::promise<json>> promises_;
    MessageHandler handler_;
    std::string uri_;
};

} // namespace mcp::client


int main () {
    asio::io_context ioc;
auto transport = std::make_shared<WebSocketTransport>(
    "ws://localhost:8080/mcp",
    [&](const nlohmann::json& msg){
        client.dispatchNotification(msg);
    }
);
McpClient client(transport);

// 注册具体通知处理
client.onNotification("notifications/resources/updated", 
    [](auto const& params){
        std::cout << \"Resource updated: \\" << params["uri"] << std::endl;
    }
);

// 调用订阅RPC
client.callRpc("resources/subscribe", json{{"uri","file://path/to/resource"}});
return 0;
}




#pragma once

#include <string>
#include <future>
#include <nlohmann/json.hpp>
#include "McpJsonRpc.hpp"
#include "mcp/client/WebSocketTransport.hpp"

namespace mcp::client {

using json = nlohmann::json;

// 扩展 McpClient: 订阅 Prompt/Resource/Tool 更新通知
class McpClientSubscribable : public McpClient {
public:
    using McpClient::McpClient; // 继承构造

    /// 订阅资源更新
    std::future<void> subscribeResource(const std::string& uri) {
        // 先注册回调再订阅
        // 使用 notifications/resources/updated 回调
        onNotification("notifications/resources/updated",
            [uri](const json& params){
                // 资源更新回调示例
                std::cout << "Resource updated: " << params["uri"].get<std::string>() << std::endl;
            }
        );
        // 调用 RPC 订阅
        return callRpcAsync("resources/subscribe", json{{"uri", uri}})
            .then([](std::future<json> f) {
                // 完成订阅，无需处理返回值
            });
    }

    /// 订阅 Prompt 列表变更
    void subscribePromptsListChanged() {
        // 注册 callback
        onNotification("notifications/prompts/list_changed",
            [](const json&){
                std::cout << "Prompts list changed" << std::endl;
            }
        );
        // MCP 协议无需显式RPC订阅; server 自动推送
    }

    /// 订阅工具列表变更
    void subscribeToolsListChanged() {
        onNotification("notifications/tools/list_changed",
            [](const json&){
                std::cout << "Tools list changed" << std::endl;
            }
        );
        // 无需显式RPC
    }
};

} // namespace mcp::client
