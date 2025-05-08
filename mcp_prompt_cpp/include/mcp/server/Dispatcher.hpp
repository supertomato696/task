#pragma once
#include "prompt/PromptService.hpp"
#include "resource/ResourceService.hpp"
#include "tools/ToolService.hpp"
#include "mcp/server/InitializeService.hpp"

#include <nlohmann/json.hpp>

namespace mcp::server {

class Dispatcher {
public:
    Dispatcher(InitializeService& init,
               prompt::PromptService&  pr,
               resource::ResourceService& rs,
               tools::ToolService&      ts);

    /** 统一入口：任何 JSON‑RPC 消息都丢给它 */
    nlohmann::json handle(const nlohmann::json& request);

private:
    InitializeService&      init_;
    prompt::PromptService&  prompt_;
    resource::ResourceService& res_;
    tools::ToolService&     tool_;
};

} // namespace