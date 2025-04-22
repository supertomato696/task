
#pragma once
#include <inja/inja.hpp>
#include <nlohmann/json.hpp>

namespace mcp::prompt {

/* 一个轻量封装，便于统一替换 / Mock */
class TemplateEngine {
public:
    TemplateEngine() = default;

    /** 渲染任意 JSON (字符串里可含 {{var}}) */
    nlohmann::json renderJson(const nlohmann::json& tpl,
                              const nlohmann::json& context = nlohmann::json::object()) const
    {
        //return env_.render_json(tpl, context);
        #if defined(INJA_VERSION_MAJOR) && (INJA_VERSION_MAJOR > 3 || (INJA_VERSION_MAJOR==3 && INJA_VERSION_MINOR>=4))
        return env_.render_json(tpl, context);          // 新接口 (>=3.4)
        #else
        return nlohmann::json::parse(env_.render(tpl.dump(), context)); // 兼容旧版
        #endif
    }

    /** 渲染文本模板 */
    std::string renderText(const std::string& tpl,
                           const nlohmann::json& context) const
    {   return env_.render(tpl, context); }

private:
    mutable inja::Environment env_;
};

} // namespace mcp::prompt