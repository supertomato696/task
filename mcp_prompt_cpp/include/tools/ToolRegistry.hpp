#pragma once
#include <functional>
#include <nlohmann/json.hpp>
#include <unordered_map>

namespace mcp::tools {

using ToolArgs  = nlohmann::json;              // 传入参数 (object)
using ToolReply = nlohmann::json;              // 返回 content[] JSON array

struct ToolDef {
    nlohmann::json  inputSchema;               // JSON‑Schema fragment
    std::string     description;
    ToolReply       (*callback)(const ToolArgs&);
};

class ToolRegistry {
public:
    void registerTool(std::string name, ToolDef def);
    std::vector<nlohmann::json> listTools() const;         // → Tool[]
    ToolReply invoke(const std::string& name, const ToolArgs& args) const;

private:
    std::unordered_map<std::string, ToolDef> map_;
};

} // namespace