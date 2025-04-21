#include "protocol/InitializeService.hpp"

using namespace mcp;
using json = nlohmann::json;

InitializeService::InitializeService(std::string n,std::string v,
                                     bool prompts,bool resources)
: serverName_(std::move(n)), version_(std::move(v))
{
    caps_ = json::object();
    if(prompts)
        caps_["prompts"] = { {"listChanged", true} };
    if(resources)
        caps_["resources"] = { {"listChanged", true}, {"subscribe", true} };
}

json InitializeService::handle(const json& req){
    if(req.at("method") != "initialize")
        throw std::runtime_error("InitializeService: wrong method");

    const std::string id = req.at("id").get<std::string>();

    json result = {
        {"serverInfo",   { {"name",serverName_}, {"version",version_} }},
        {"protocolVersion","2025-03-26"},
        {"capabilities", caps_},
        {"instructions",
         "This server offers prompt templates via prompts/* and "
         "resources via resources/* ."}
    };
    return { {"jsonrpc","2.0"}, {"id",id}, {"result",result} };
}