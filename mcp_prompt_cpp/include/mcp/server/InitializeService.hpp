#pragma once
#include <nlohmann/json.hpp>
#include <string>

namespace mcp::server {

/**
 * 处理 "initialize" JSON‑RPC 请求，
 * 返回包含 serverInfo / capabilities / protocolVersion 的结果。
 *
 * 构造函数允许声明哪些顶级模块提供 listChanged/subscribe 能力。
 */
class InitializeService {
public:
    InitializeService(std::string serverName,
                      std::string serverVersion,
                      bool promptsCap   = true,
                      bool resourcesCap = true,
                      bool toolsCap     = true);

    /** 若 method == "initialize" 则生成 response，否则抛异常 */
    nlohmann::json handle(const nlohmann::json& request) const;

private:
    std::string serverName_;
    std::string version_;
    nlohmann::json caps_;      // 预生成好的 capabilities json
};

} // namespace