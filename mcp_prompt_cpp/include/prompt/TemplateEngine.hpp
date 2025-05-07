#pragma once
#include <unordered_map>
#include <string>
#include <regex>

namespace mcp::prompt {

/**
 *  将 {{name}} 占位符替换为实参。
 *  - 未找到对应实参时占位符原样保留
 *  - 不支持复杂表达式，仅简单 key
 */
 
class TemplateEngine {
public:
    static std::string render(const std::string& tpl,
                              const std::unordered_map<std::string,std::string>& args)
    {
        static const std::regex re(R"(\{\{\s*([a-zA-Z0-9_]+)\s*\}\})");

        std::string out;
        size_t last = 0;
        for (auto it = std::sregex_iterator(tpl.begin(), tpl.end(), re);
             it != std::sregex_iterator(); ++it)
        {
            out.append(tpl.substr(last, it->position() - last));

            std::string key = (*it)[1];
            auto f = args.find(key);
            out.append(f == args.end() ? it->str() : f->second);

            last = it->position() + it->length();
        }
        out.append(tpl.substr(last));
        return out;
    }

//     std::string render(const std::string& tpl, const std::unordered_map<std::string, std::string>& args) {
//     // 使用 fmt 库来处理字符串替换
//     std::string result = tpl;
//     for (const auto& [key, value] : args) {
//         result = fmt::format(result, fmt::arg(key, value));
//     }
//     return result;
// }

};

} // namespace