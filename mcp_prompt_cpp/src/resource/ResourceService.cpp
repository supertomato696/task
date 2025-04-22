// src/resource/ResourceService.cpp
#include "resource/ResourceService.hpp"

using namespace mcp::resource;
using json = nlohmann::json;

json ResourceService::handle(const json& req){
    // const std::string id     = req.value("id","0");
    json id = req["id"];          // 保留类型
    const std::string method = req.at("method");
    const json params = req.value("params", json::object());

    if(method=="resources/list") return list(id);
    if(method=="resources/read") return read(id, params);
    throw std::runtime_error("unknown resource method");
}

json ResourceService::list(const std::string& id){
    json res = { {"resources", rm_.listResources()} };
    return { {"jsonrpc","2.0"},{"id",id},{"result",res} };
}

json ResourceService::read(const std::string& id,const json& p){
    auto uri = p.at("uri").get<std::string>();
    json res = { {"contents", rm_.readResource(uri)} };
    return { {"jsonrpc","2.0"},{"id",id},{"result",res} };
}