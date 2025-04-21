#pragma once
#include "Types.hpp"
#include <asio.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>
//#include <jthread>
#include <atomic>

namespace mcp::client {

/** 简单阻塞式 Client：每次请求同步等待响应；连接断掉自动重连 */
class McpClient {
public:
    /** host 可为 "127.0.0.1", port 9275 */
    McpClient(std::string host, uint16_t port);
    ~McpClient();

    std::vector<PromptInfo>   listPrompts();
    GetPromptResult           getPrompt(const std::string& name, const ArgMap& args = {});
public:
   std::future<std::vector<PromptInfo>> listPromptsAsync();
   std::future<GetPromptResult>         getPromptAsync(const std::string& name, const ArgMap& args = {});


public:
//   std::future<std::vector<nlohmann::json>> listResourcesAsync();
//   std::future<nlohmann::json> readResourceAsync(const std::string& uri);

       std::future<nlohmann::json> listResourcesAsync();
       std::future<nlohmann::json> readResourceAsync(const std::string& uri);
private:
    nlohmann::json rpcCall(const std::string& method, nlohmann::json params);
    std::future<nlohmann::json> asyncRpcCall(const std::string& method, nlohmann::json params, McpClient& self);

    /* ---- TCP internals ---- */
    void start();
    void do_connect();
    void do_read();
    void send_raw(const std::string& txt);

    asio::io_context                  io_;
    asio::ip::tcp::resolver           resolver_;
    std::shared_ptr<asio::ip::tcp::socket> sock_;
    asio::streambuf                   buf_;
    const std::string                 host_;
    uint16_t                          port_;
    std::jthread                      th_;

    std::mutex                        inflight_mtx_;
    std::condition_variable           inflight_cv_;
    std::unordered_map<std::string, nlohmann::json> responses_;
        /* ------------ for async ------------- */
    struct Pending {
        std::promise<nlohmann::json> prom;
    };
    std::unordered_map<std::string, std::shared_ptr<Pending>> pendings_;

    std::atomic_uint64_t              id_gen_{1};
};

} // namespace mcp::client