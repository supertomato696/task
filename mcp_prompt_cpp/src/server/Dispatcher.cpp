#include "mcp/server/Dispatcher.hpp"
#include "protocol/McpJsonRpc.hpp"

using namespace mcp::server;
using json = nlohmann::json;



static json makeError(const mcp::protocol::Id& id, int code, std::string_view msg)
{
    return mcp::protocol::makeJsonRpcError(id, code, std::string(msg));
}

Dispatcher::Dispatcher(InitializeService& init,
                       prompt::PromptService&  pr,
                       resource::ResourceService& rs,
                       tools::ToolService&      ts)
    : init_(init), prompt_(pr), res_(rs), tool_(ts) {}

json Dispatcher::handle(const json& req)
{
    /* 基本字段解析 */
    protocol::RequestId id;
    std::string  method;
    json         params;
//    if(!protocol::getBasicRequestFields(req, id, method, params))
//        return makeError(id, -32600, "Invalid Request");

    try{
        if(method == "initialize")        return init_.handle(req);
        if(method == "ping")              return protocol::makeJsonRpcResult(id, json::object());

        if(method.rfind("prompts/",0)==0)    return prompt_.handleRpc(req);
        if(method.rfind("resources/",0)==0)  return res_.handle(req);
        if(method.rfind("tools/",0)==0)      return tool_.handle(req);

        return makeError(id, -32601, "Method not found");
    }
    catch(const std::exception& e){
        return makeError(id, -32603, e.what());
    }
}