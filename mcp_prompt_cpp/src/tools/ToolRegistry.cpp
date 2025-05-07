#include "tools/ToolRegistry.hpp"
#include <stdexcept>

using namespace mcp::tools;

void ToolRegistry::registerTool(const ToolDef& def)
{
    map_.emplace(def.meta.name, def);
}

std::vector<protocol::Tool> ToolRegistry::listTools() const
{
    std::vector<protocol::Tool> v;
    v.reserve(map_.size());
    for (auto& [_, d] : map_) v.push_back(d.meta);
    return v;
}

ToolReply ToolRegistry::invoke(const std::string& name,
                               const ToolArgs&    args) const
{
    auto it = map_.find(name);
    if(it == map_.end())
        throw std::runtime_error("tool not found: " + name);

    return it->second.callback(args);
}

void ToolRegistry::registerTool(const ToolDef& def)
{
    map_.emplace(def.meta.name, def);
    if(onChanged_) onChanged_();
}

void ToolRegistry::removeTool(const std::string& name)
{
    map_.erase(name);
    if(onChanged_) onChanged_();
}
