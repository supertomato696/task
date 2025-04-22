#include "tools/ToolRegistry.hpp"
#include <stdexcept>

using namespace mcp::tools;
using json = nlohmann::json;

void ToolRegistry::registerTool(std::string n, ToolDef d){
    map_.emplace(std::move(n), std::move(d));
}

std::vector<json> ToolRegistry::listTools() const {
    std::vector<json> v;
    for(auto& [name,def]: map_){
        v.push_back({
            {"name",name},
            {"description",def.description},
            {"inputSchema",def.inputSchema}
        });
    }
    return v;
}

ToolReply ToolRegistry::invoke(const std::string& n,const ToolArgs& a) const{
    auto it = map_.find(n);
    if(it==map_.end()) throw std::runtime_error("tool not found");
    return it->second.callback(a);
}


// （为简洁，未做 JSON‑Schema 校验；如需可接入 [vocab/json‑schema‑validator](https://github.com/pboettch/json-schema-validator) ）