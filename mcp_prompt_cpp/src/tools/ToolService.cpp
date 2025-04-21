#include "tools/ToolService.hpp"

using namespace mcp::tools;
using json = nlohmann::json;

json ToolService::handle(const json& req){
    // const std::string id = req.value("id","0");
    json id = req["id"];
    const std::string method = req.at("method");
    const json params = req.value("params", json::object());

    if(method=="tools/list")  return list(id);
    if(method=="tools/call")  return call(id, params);
    throw std::runtime_error("unknown tool method");
}

json ToolService::list(const std::string& id){
    return { {"jsonrpc","2.0"},{"id",id},
             {"result", {{"tools", reg_.listTools()}} } };
}

json ToolService::call(const std::string& id,const json& p){
    std::string name = p.at("name");
    json args = p.value("arguments", json::object());
    // 
    json content;
   bool isErr = false;
   try{
        content = reg_.invoke(name, args);
    }catch(const std::exception& e){
        isErr = true;
        content = json::array({{{"type","text"},{"text",std::string("error:")+e.what()}}});
   }

    // json result = {
    //     {"content",content},
    //     {"isError",false}
    // };

       json result = { {"content",content} };
   if(isErr) result["isError"] = true;
    return { {"jsonrpc","2.0"},{"id",id},{"result",result} };
}