#pragma once
#include <nlohmann/json.hpp>
#include <string>

namespace mcp {

/** 处理 "initialize" 请求，返回 InitializeResult */
class InitializeService {
public:
    InitializeService(std::string serverName, std::string version,
                      bool hasPrompts, bool hasResources);

    /** 若 method=="initialize" 则处理，否则抛异常 */
    nlohmann::json handle(const nlohmann::json& req);

    /** capabilities for reuse by other services */
    nlohmann::json serverCapabilities() const { return caps_; }

private:
    std::string serverName_, version_;
    nlohmann::json caps_;
};

} // namespace