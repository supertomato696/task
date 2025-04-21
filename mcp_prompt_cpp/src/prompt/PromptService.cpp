// src/prompt/PromptService.cpp
#include "prompt/PromptService.hpp"
#include <inja/inja.hpp>

using namespace mcp;
using nlohmann::json;

static json make_error(const std::string& id,int code,const std::string& msg){
    return { {"jsonrpc","2.0"}, {"id", id}, {"error", {{"code",code},{"message",msg}}}};
}

json PromptService::handle(const json& req){
    const std::string id = req.value("id","0");
    const std::string method = req.at("method");
    const json params = req.value("params", json::object());

    if(method=="prompts/list")
        return listPrompts(id, params);
    if(method=="prompts/get")
        return getPrompt(id, params);
    return make_error(id,-32601,"Method not found");
}

/* ---------- prompts/list ---------- */
json PromptService::listPrompts(const std::string& id,const json&){
    json arr = json::array();
    for(auto& p: store_.all()){
        arr.push_back({ {"name",p.name}, {"description",p.description}, {"arguments",p.arguments} });
    }
    return { {"jsonrpc","2.0"},{"id",id},{"result", { {"prompts", arr } } } };
}

/* ---------- prompts/get ---------- */
json PromptService::getPrompt(const std::string& id,const json& params){
    const std::string name = params.at("name");
    const auto* tpl = store_.find(name);
    if(!tpl) return make_error(id, -32000, "prompt not found");

    // 1. 模板渲染
    inja::Environment env;
    json context = params.value("arguments", json::object());
    json rendered = env.render_json(tpl->messages.dump(), context);

    // 2. 消息装配 (处理多模态占位符 → Content)
    json msgs = json::array();
    for(const auto& m : rendered){
        std::string role = m.at("role");
        std::string val  = m.at("content").get<std::string>();
        auto pm = mma_.assemble(role, val);
        msgs.push_back(pm);
    }

    json result = { {"messages", msgs}, {"description", tpl->description} };
    return { {"jsonrpc","2.0"},{"id",id},{"result", result} };
}