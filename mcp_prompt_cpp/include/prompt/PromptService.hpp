// include/prompt/PromptService.hpp
#pragma once
#include "protocol/JsonRpc.hpp"
#include "prompt/PromptStore.hpp"
#include "prompt/TemplateEngine.hpp"
#include "prompt/MultiModalAssembler.hpp"
#include "protocol/McpContent.hpp"
#include <nlohmann/json.hpp>
#include <unordered_map>

namespace mcp {

struct PromptTemplate {
    std::string name;
    std::string description;
    nlohmann::json messages;   // 原始 JSON 模板 (messages 数组)
    std::vector<std::string>  arguments;
};

class PromptStore {
public:
    // 内存版本
    void add(const PromptTemplate& t){ store_[t.name]=t; }
    const PromptTemplate* find(const std::string& n) const {
        auto it = store_.find(n); return it==store_.end()?nullptr:&it->second;
    }
    std::vector<PromptTemplate> all() const {
        std::vector<PromptTemplate> v; for(auto&[_,p]:store_) v.push_back(p); return v;
    }
private: std::unordered_map<std::string,PromptTemplate> store_;
};



/* -------- JSON‑RPC Handler -------- */
class PromptService {
public:
    explicit PromptService(PromptStore& st):store_(st){}
//    explicit PromptService(PromptStore& store);

    /** 输入 JSON‑RPC request (2.0) → 返回 response/result */
    nlohmann::json handle(const nlohmann::json& req);

//    /** Handle JSON-RPC request, throw on protocol error */
//    protocol::JsonRpcResponse handle(const protocol::JsonRpcRequest& req);

private:
//    protocol::JsonRpcResponse handleList(const protocol::JsonRpcRequest&);
//    protocol::JsonRpcResponse handleGet (const protocol::JsonRpcRequest&);


private:
     lohmann::json listPrompts(const std::string& id,const nlohmann::json& params);
    nlohmann::json getPrompt (const std::string& id,const nlohmann::json& params);

    PromptStore&           store_;
    TemplateEngine         tpl_;
    MultiModalAssembler    mma_;
};

} // namespace mcp