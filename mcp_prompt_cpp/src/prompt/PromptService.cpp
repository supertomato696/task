#include "prompt/PromptService.hpp"
#include <stdexcept>

using namespace mcp::prompt;
namespace proto = mcp::protocol;

/* ---------- ctor ---------- */
PromptService::PromptService(PromptStore& s, Notify n)
    : store_(s), notify_(std::move(n)) {}

/* ===================================================================== */
/*                           JSON‑RPC dispatcher                         */
/* ===================================================================== */
PromptService::json PromptService::handleRpc(const json& req)
{
    const std::string id     = req.value("id", "0");
    const std::string method = req.at("method");

    try {
        /* ------------ prompts/list ------------ */
        if (method == "prompts/list") {
            auto result = handleList();
            return proto::makeJsonRpcResult(id, result);
        }

        /* ------------ prompts/get ------------- */
        if (method == "prompts/get") {
            proto::GetPromptRequest gpReq = req.at("params");
            auto result = handleGet(gpReq);
            return proto::makeJsonRpcResult(id, result);
        }

        return proto::makeJsonRpcError(id, -32601, "Method not found");
    }
    catch (const std::exception& e) {
        return proto::makeJsonRpcError(id, -32603, e.what());
    }
}

/* ===================================================================== */
/*                              prompts/list                             */
/* ===================================================================== */
proto::ListPromptsResult PromptService::handleList() const
{
    proto::ListPromptsResult res;
    res.prompts = store_.listTemplates();   // meta 列表
    return res;
}

/* ===================================================================== */
/*                               prompts/get                             */
/* ===================================================================== */
proto::GetPromptResult
PromptService::handleGet(const proto::GetPromptRequest& req)
{
    /* MCP 允许没有 arguments 字段；将其视为空 map */
    std::map<std::string,std::string> args = req.params.arguments.value_or(std::map<std::string,std::string>{});

    // (
    //     req.params.arguments.begin(), req.arguments.end());
    std::unordered_map<std::string, std::string> args_map(args.begin(), args.end());
    auto filled = store_.renderPrompt(req.params.name, args_map);
    return filled.data;          // 即 {description, messages}
}

/* ===================================================================== */
/*                   notifications/prompts/list_changed                   */
/* ===================================================================== */
void PromptService::fireListChanged() const
{
    if (!notify_) return;

    json note = {
        {"jsonrpc","2.0"},
        {"method","notifications/prompts/list_changed"},
        {"params", json::object() }
    };
    notify_(note);
}
