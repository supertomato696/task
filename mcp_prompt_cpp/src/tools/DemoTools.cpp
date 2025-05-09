#include "tools/DemoTools.hpp"
#include <fstream>

using namespace mcp::tools;
using json = nlohmann::json;

namespace  protocol = mcp::protocol;

/* ---------- say_text {"text":string} --------------------------- */
static ToolReply say_text(const ToolArgs& a)
{
    std::string txt = a.value("text", "");
    protocol::PromptMessage out;
    out.role = protocol::Role::Assistant;
    out.content = protocol::TextContent{ .text = "🗣️ " + txt };
    return { std::move(out) };
}

/* ---------- car_info {}  → EmbeddedResource -------------------- */
static ToolReply car_info(const ToolArgs&)
{
    std::ifstream f("/var/lib/mcp/car_status.json");
    std::string body{ std::istreambuf_iterator<char>(f), {} };

    protocol::EmbeddedResource res;
    res.resource = protocol::TextResourceContents{
        .uri      = "file:///var/lib/mcp/car_status.json",
        .mimeType = "application/json",
        .text     = body
    };

    protocol::PromptMessage msg;
    msg.role    = protocol::Role::Assistant;
    msg.content = std::move(res);
    return { std::move(msg) };
}

/* ---------- 注册入口 ------------------------------------------- */
void registerDemoTools(ToolRegistry& reg)
{
    /* say_text */
    protocol::Tool meta1;
    meta1.name        = "say_text";
    meta1.description = "朗读指定文本";
    meta1.inputSchema = R"({"type":"object",
                             "properties":{"text":{"type":"string"}},
                             "required":["text"]})"_json;

    reg.registerTool({meta1, &say_text});

    /* car_info */
    protocol::Tool meta2;
    meta2.name        = "car_info";
    meta2.description = "返回车辆状态 JSON";
    meta2.inputSchema = R"({"type":"object","properties":{}})"_json;

    reg.registerTool({meta2, &car_info});
}
