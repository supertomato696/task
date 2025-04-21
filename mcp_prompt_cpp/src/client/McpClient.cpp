#include "mcp/client/McpClient.hpp"
#include <iostream>

using namespace mcp::client;
using asio::ip::tcp;
using json = nlohmann::json;

/* ---------- ctor/dtor ---------- */
McpClient::McpClient(std::string h,uint16_t p)
: resolver_(io_), host_(std::move(h)), port_(p)
{
    start();
}

McpClient::~McpClient(){
    io_.stop();
    if(th_.joinable()) th_.join();
}

/* ---------- public RPC wrappers ---------- */
std::vector<PromptInfo> McpClient::listPrompts(){
    json res = rpcCall("prompts/list", {} )["prompts"];
    std::vector<PromptInfo> out;
    for(auto& p: res){
        PromptInfo pi;
        pi.name        = p["name"];
        pi.description = p.value("description", "");
        pi.arguments   = p.value("arguments", std::vector<std::string>{});
        out.push_back(std::move(pi));
    }
    return out;
}

GetPromptResult McpClient::getPrompt(const std::string& n,const ArgMap& args){
    json params = { {"name",n}, {"arguments", args} };
    json res    = rpcCall("prompts/get", params);
    GetPromptResult r;
    r.description = res.value("description","");
    for(auto& m : res["messages"])
        r.messages.push_back({ m["content"], m["role"] });
    return r;
}

//std::future<std::vector<nlohmann::json>> McpClient::listResourcesAsync(){
//    return asyncRpcCall("resources/list", {}, *this);
//}

// std::future<nlohmann::json> McpClient::listResourcesAsync(){
//     return asyncRpcCall("resources/list", {}, *this)
//            .then([](auto f){ return f.get()["resources"]
//                                    .get<std::vector<json>>(); });
// }

std::future<std::vector<json>> McpClient::listResourcesAsync(){
    auto prom = std::make_shared<std::promise<std::vector<json>>>();
    auto fut  = prom->get_future();

    asyncRpcCall("resources/list", {}, *this)
        .then([prom](std::future<json> f){
            try{
                auto vec = f.get()["resources"]
                             .get<std::vector<json>>();
                prom->set_value(std::move(vec));
            }catch(...){
                prom->set_exception(std::current_exception());
            }
        });
    return fut;
}


std::future<nlohmann::json> McpClient::readResourceAsync(const std::string& uri){
    return asyncRpcCall("resources/read", {{"uri",uri}}, *this);
}

/* ---------- 内部：同步 RPC ---------- */
json McpClient::rpcCall(const std::string& method, json params){
    std::string id = std::to_string(id_gen_++);
    json req = { {"jsonrpc","2.0"},{"id",id},{"method",method},{"params",std::move(params)} };
    send_raw(req.dump()+"\n");

    std::unique_lock lk(inflight_mtx_);
    inflight_cv_.wait(lk, [&]{ return responses_.count(id)>0; });
    json resp = std::move(responses_[id]);
    responses_.erase(id);
    lk.unlock();

    if(resp.contains("error"))
        throw std::runtime_error("RPC error: "+resp["error"]["message"].get<std::string>());
    return resp["result"];
}

/* ----------- helper: asyncRpcCall ------------- */
std::future<json> McpClient::asyncRpcCall(const std::string& method,
                               json params,
                               McpClient& self)
{
    std::string id = std::to_string(self.id_gen_++);
    auto pend   = std::make_shared<McpClient::Pending>();
    std::future<json> fut = pend->prom.get_future();

    {
        std::lock_guard lk(self.inflight_mtx_);
        self.pendings_[id] = pend;
    }

    json req = { {"jsonrpc","2.0"},{"id",id},
                 {"method",method},{"params",std::move(params)} };
    self.send_raw(req.dump()+"\n");
    return fut;
}

/* ---------- 网络 ---------- */
void McpClient::start(){
    do_connect();
    th_ = std::jthread([this]{ io_.run(); });
}

void McpClient::do_connect(){
    resolver_.async_resolve(tcp::resolver::query(host_, std::to_string(port_)),
        [this](auto ec, tcp::resolver::results_type ep){
            if(ec){ std::cerr<<"resolve error: "<<ec.message()<<"\n"; return;}
            sock_ = std::make_shared<tcp::socket>(io_);
            asio::async_connect(*sock_, ep,
                [this](auto ec, auto){ 
                    if(ec){ std::cerr<<"connect error\n"; return;}
                        /* ---------- MCP initialize ---------- */
                        json initReq = {
                            {"jsonrpc","2.0"},
                            {"id","init-1"},
                            {"method","initialize"},
                            {"params",{
                                {"protocolVersion","2025-03-26"},
                                {"clientInfo", {{"name","seat-sdk"},{"version","0.1"}}},
                                {"capabilities",{
                                    {"prompts",  {}} ,
                                    {"resources",{}}
                                }}
                            }}
                        };
                        send_raw(initReq.dump()+"\n");
                        do_read();
                });
        });
}

void McpClient::do_read(){
    if(!sock_) return;
    asio::async_read_until(*sock_, buf_, '\n',
        [this](auto ec,std::size_t){
            if(ec){ std::cerr<<"read: "<<ec.message()<<"\n"; do_connect(); return;}
            std::istream is(&buf_);
            std::string line; std::getline(is, line);
            try{
                json j = json::parse(line);
                std::string id = j.value("id","0");
                {
                    std::lock_guard g(inflight_mtx_);
//                    responses_[id]=std::move(j);
                        // for sync
                    responses_[id] = j;
                    // for async
                    auto it = pendings_.find(id);
                    if(it != pendings_.end()) {
                        it->second->prom.set_value(j.contains("result")? j["result"] : j);
                        pendings_.erase(it);
                    }
                    
                    
                }
                inflight_cv_.notify_all();
            }catch(const std::exception& e){
                std::cerr<<"parse: "<<e.what()<<"\n";
            }
            do_read();
        });
}

void McpClient::send_raw(const std::string& txt){
    if(!sock_ || !sock_->is_open()) throw std::runtime_error("socket closed");
    asio::post(io_, [s=sock_,txt]{
        asio::async_write(*s, asio::buffer(txt),
            [](auto ec,std::size_t){ if(ec) std::cerr<<"send: "<<ec.message()<<"\n"; });
    });
}