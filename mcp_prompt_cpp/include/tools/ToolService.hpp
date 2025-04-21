#pragma once
#include "tools/ToolRegistry.hpp"
#include <nlohmann/json.hpp>

namespace mcp::tools {

class ToolService {
public:
    explicit ToolService(ToolRegistry& r):reg_(r){}
    nlohmann::json handle(const nlohmann::json& req);

private:
    nlohmann::json list(const std::string& id);
    nlohmann::json call(const std::string& id,const nlohmann::json& p);

    ToolRegistry& reg_;
};

}