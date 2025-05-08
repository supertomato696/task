#include "mcp/server/InitializeService.hpp"
#include "protocol/McpJsonRpc.hpp"

using namespace mcp::server;
using json = nlohmann::json;

InitializeService::InitializeService(std::string n,
                                     std::string v,
                                     bool prompts,
                                     bool resources,
                                     bool tools)
    : serverName_(std::move(n)), version_(std::move(v))
{
    if(prompts)   caps_["prompts"]   = { {"listChanged", true} };
    if(resources) caps_["resources"] = { {"listChanged", true}, {"subscribe", true} };
    if(tools)     caps_["tools"]     = { {"listChanged", true} };
}

json InitializeService::handle(const json& req) const
{
    if(req.at("method") != "initialize")
        throw std::runtime_error("InitializeService: wrong method");

    auto id = req.at("id");         // id 既可能是 int 也可能是 string

    json result = {
        {"serverInfo",      { {"name", serverName_}, {"version", version_} }},
        {"protocolVersion", "2025-03-26"},
        {"capabilities",    caps_},
        {"instructions",
         "This server exposes prompt, resource and tool endpoints via MCP."}
    };
    return protocol::makeJsonRpcResult(id, result);
}

//inline json handleInitialize(const json& req)
//{
//    protocol::InitializeRequest r = req.get<protocol::InitializeRequest>();
//
//    protocol::InitializeResult res;
//    res.capabilities.prompts    = {};
//    res.capabilities.resources  = {};
//    res.capabilities.tools      = {};
//    res.protocolVersion         = r.params.protocolVersion;   // mirror back
//    res.serverInfo.name         = "mcp-demo-server";
//    res.serverInfo.version      = "0.1";
//
//    return protocol::makeJsonRpcResult(r.id, res);
//}