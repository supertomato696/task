// #include "mcp/client2/JsonRpcClient.hpp"
// #include <iostream>
//
// using namespace mcp::client2;
// using asio::ip::tcp;
// using json = nlohmann::json;
//
// /* ======== ctor / dtor ======== */
// JsonRpcClient::JsonRpcClient(std::string h, uint16_t p)
//     : resolver_(io_), host_(std::move(h)), port_(p)
// {
//     start();
// }
// JsonRpcClient::~JsonRpcClient()
// {
//     io_.stop();
//     if (ioThread_.joinable()) ioThread_.join();
// }
//
// /* ======== public low‑level ======== */
// json JsonRpcClient::callRaw(const json& reqIn)
// {
//     // 拷贝以便注入 id
//     json req = reqIn;
//     const std::string id = std::to_string(idGen_++);
//     req["jsonrpc"] = "2.0";
//     req["id"]      = id;
//
//     sendRaw(req.dump() + '\n');
//
//     std::unique_lock lk(mtx_);
//     cv_.wait(lk, [&]{ return syncResp_.count(id) > 0; });
//
//     json resp = std::move(syncResp_[id]);
//     syncResp_.erase(id);
//     lk.unlock();
//
//     if (resp.contains("error"))
//         throw std::runtime_error("RPC‑error: "
//             + resp["error"]["message"].get<std::string>());
//     return resp["result"];
// }
//
// std::future<json> JsonRpcClient::callRawAsync(const json& reqIn)
// {
//     json req = reqIn;
//     const std::string id = std::to_string(idGen_++);
//     req["jsonrpc"] = "2.0";
//     req["id"]      = id;
//
//     auto prom = std::make_shared<std::promise<json>>();
//     auto fut  = prom->get_future();
//
//     {
//         std::lock_guard lk(mtx_);
//         pendings_[id] = prom;
//     }
//     sendRaw(req.dump() + '\n');
//     return fut;
// }
//
// /* ======== transport ======== */
// void JsonRpcClient::start()
// {
//     connect();
//     ioThread_ = std::jthread([this]{ io_.run(); });
// }
//
// void JsonRpcClient::connect()
// {
//     resolver_.async_resolve(host_, std::to_string(port_),
//         [this](std::error_code ec, tcp::resolver::results_type eps)
//     {
//         if (ec) { std::cerr << "resolve: " << ec.message() << '\n'; return; }
//         sock_ = std::make_shared<tcp::socket>(io_);
//         asio::async_connect(*sock_, eps,
//             [this](std::error_code ec, const tcp::endpoint&)
//         {
//             if (ec) { std::cerr << "connect: " << ec.message() << '\n'; return; }
//             readLoop();
//         });
//     });
// }
//
// void JsonRpcClient::readLoop()
// {
//     if (!sock_) return;
//     asio::async_read_until(*sock_, buf_, '\n',
//         [this](std::error_code ec, std::size_t)
//     {
//         if (ec) { std::cerr << "read: " << ec.message() << '\n'; connect(); return; }
//
//         std::istream is(&buf_);
//         std::string  line; std::getline(is, line);
//
//         try {
//             json msg = json::parse(line);
//                 /* ----------- 1. 服务器通知 (无 id) ----------- */
//     if (!msg.contains("id") && msg.contains("method")) {
//         std::string m = msg["method"];
//         json        p = msg.value("params", json::object());
//
//         std::lock_guard lk(notifyMtx_);
//         if (auto it = notifyMap_.find(m); it != notifyMap_.end())
//             it->second(p);                  // 直接在 IO 线程调用
//         // 未注册则静默忽略
//         return readLoop();
//     }
//
//     /* ----------- 2. request / response 处理(与之前相同) ----------- */
//             std::string id = msg.value("id", "");        // server notifications may omit id
//
//             std::lock_guard lk(mtx_);
//             if (auto it = pendings_.find(id); it != pendings_.end()) {
//                 it->second->set_value(msg.contains("result") ? msg["result"] : msg);
//                 pendings_.erase(it);
//             } else {
//                 syncResp_[id] = msg;
//                 cv_.notify_all();
//             }
//         } catch (const std::exception& e) {
//             std::cerr << "json‑parse error: " << e.what() << '\n';
//         }
//         readLoop();
//     });
// }
//
// void JsonRpcClient::sendRaw(const std::string& s)
// {
//     if (!sock_ || !sock_->is_open())
//         throw std::runtime_error("socket not ready");
//     asio::post(io_, [s, sock = sock_]
//     {
//         asio::async_write(*sock, asio::buffer(s),
//             [](std::error_code ec, std::size_t)
//         {
//             if (ec) std::cerr << "send: " << ec.message() << '\n';
//         });
//     });
// }



#include "mcp/client2/JsonRpcClient.hpp"
#include <stdexcept>

using namespace mcp::client2;
using json = nlohmann::json;

JsonRpcClient::JsonRpcClient(transport::ITransport& t):tx_(t){}

/* 收到来自 Transport 的 json 消息 */
void JsonRpcClient::deliver(const json& msg)
{
    /* 只处理 Response/Error，Notification 由高层模块自行订阅 */
    Id id; json result;
    if(protocol::parseResponse(msg, id, result)){   // 成功 Response
        std::shared_ptr<Pending> pend;
        {
            std::scoped_lock lk(mtx_);
            auto it = inflight_.find(id);
            if(it==inflight_.end()) return;         // 不是我们发的
            pend = it->second;
            inflight_.erase(it);
        }
        pend->prom.set_value(std::move(result));
    }
    else if(protocol::parseError(msg, id, result)){ // Error
        std::string err = result["message"];
        std::runtime_error ex(err);
        std::shared_ptr<Pending> pend;
        {
            std::scoped_lock lk(mtx_);
            auto it = inflight_.find(id);
            if(it==inflight_.end()) return;
            pend = it->second;
            inflight_.erase(it);
        }
        pend->prom.set_exception(std::make_exception_ptr(ex));
    }
}