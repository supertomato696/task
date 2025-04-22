
#pragma once
#include <nlohmann/json.hpp>
#include <string>
#include <variant>

namespace mcp::protocol {

/* -------  帮助类型  ------- */
using json = nlohmann::json;
/* ---------- ID 类型 (允空 / int / string) ---------- */
using Id = std::variant<std::monostate, int64_t, std::string>;

/* tiny helper 把 Id → json */
inline json idToJson(const Id& id) {
    return std::visit([](auto&& v) -> json {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            return nullptr; // 或 json::object() 根据协议要求
        } else {
            return json(v); // 正常处理 int/string
        }
    }, id);
}


/* -------  构造器  ------- */
inline json makeRequest(const Id& id,
                        const std::string& method,
                        json params = json::object())
{
    json j = { {"jsonrpc","2.0"}, {"method",method}, {"params",std::move(params)} };
     json idJson = idToJson(id);
    if (!idJson.is_null()) { // ✅ 更安全的空值检查
        j["id"] = std::move(idJson);
    }
    return j;
 //   if(!std::holds_alternative<std::monostate>(id))
  //      j["id"] = idToJson(id);
  //  return j;
}

inline json makeResult(const Id& id, json result = json::object()){
        return { {"jsonrpc","2.0"},
             {"id",   idToJson(id)},
             {"result", std::move(result)} };
}

inline json makeError (const Id& id, int code, const std::string& msg){
    return { {"jsonrpc","2.0"},
             {"id",   idToJson(id)},
             {"error", {{"code",code},{"message",msg}} } };
}

/* -------  id 解析工具  ------- */
inline Id parseId(const json& j){
    if(j.contains("id")){
        if(j["id"].is_number_integer()) return j["id"].get<int64_t>();
        if(j["id"].is_string())         return j["id"].get<std::string>();
    }
    return {}; // empty
}

} // namespace mcp::protocol
