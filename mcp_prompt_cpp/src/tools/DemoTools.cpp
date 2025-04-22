#include "tools/ToolRegistry.hpp"
#include "tools/DemoTools.hpp"
#include <fstream>
using json = nlohmann::json;
namespace mcp::tools {


/* ---- say_text: {"text": string} → assistant TextContent ---- */
static ToolReply say_text(const ToolArgs& a){
    std::string t = a.value("text","");
    return json::array({{
        {"type","text"},{"text",std::string("🗣️ ")+t}
    }});
}

/* ---- car_info: 无参 → EmbeddedResource (假读文件) ---- */
static ToolReply car_info(const ToolArgs&){
    std::ifstream f("/var/lib/mcp/car_status.json");
    std::string body{std::istreambuf_iterator<char>(f),{}};
    return json::array({{
        {"type","resource"},
        {"resource",{
             {"uri","file:///var/lib/mcp/car_status.json"},
             {"mimeType","application/json"},
             {"text",body}
        }}
    }});
}

/* ---- 注册 ---- */
void registerDemoTools(ToolRegistry& reg){
    reg.registerTool("say_text", {
        /*inputSchema*/{
            {"type","object"},
            {"properties",{{"text", {{"type","string"}}}}},
            {"required",{"text"}}
        },
        /*desc*/"朗读指定文本",
        /*cb*/ &say_text
    });
    reg.registerTool("car_info", {
        {{"type","object"}, {"properties", nlohmann::json::object()}},
        "返回车辆状态 JSON",
        &car_info
    });
}

}