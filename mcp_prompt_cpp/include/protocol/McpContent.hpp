#pragma once
#include <nlohmann/json.hpp>
#include <variant>
#include <string>
#include <vector>
#include <optional>
#include <variant>

namespace mcp::protocol {

using json = nlohmann::json;


/* -----------------------------------------------------------
 *  Annotations  (可选)
 * -----------------------------------------------------------*/
struct Annotations {
    std::vector<std::string> audience;          // 可为空
    std::optional<double>    priority;          // 0‑1
};
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Annotations, audience, priority);

    // MCP 协议要求的序列化
inline void to_json(nlohmann::json& j, const Annotations& a) {
    j = nlohmann::json{};  // 清空初始对象
    j["audience"] = a.audience; // 始终序列化 audience，哪怕是空的
    if (a.priority) {
        j["priority"] = *a.priority;
    }
}

// MCP 协议要求的反序列化
inline void from_json(const nlohmann::json& j, Annotations& a) {
    // audience 必须存在且为数组，即使为空
    if (j.contains("audience") && j["audience"].is_array()) {
        j.at("audience").get_to(a.audience);
    } else {
        a.audience.clear(); // 如果没有就默认为空数组
    }

    // priority 是可选的
    if (j.contains("priority") && j["priority"].is_number()) {
        double val = j["priority"].get<double>();
        if (val >= 0.0 && val <= 1.0) {
            a.priority = val;
        } else {
            // 超出范围不设置，符合 MCP 规范的容错处理
            a.priority.reset();
        }
    } else {
        a.priority.reset();
    }
}

/* -----------------------------------------------------------
 *  Resource  (resources/list 里的元素)
 * -----------------------------------------------------------*/
struct Resource {
    std::string              name;
    std::string              uri;               // 必填
    std::optional<std::string> description;
    std::optional<std::string> mimeType;
    std::optional<uint64_t>    size;
    std::optional<Annotations> annotations;
};

/* --- JSON ⇄ struct --- */
inline void to_json(json& j, const Resource& r)
{
    j = {{"name", r.name}, {"uri", r.uri}};
    if (r.description)  j["description"] = *r.description;
    if (r.mimeType)     j["mimeType"]    = *r.mimeType;
    if (r.size)         j["size"]        = *r.size;
    if (r.annotations)  j["annotations"] = *r.annotations;
}
inline void from_json(const json& j, Resource& r)
{
    j.at("name").get_to(r.name);
    j.at("uri" ).get_to(r.uri);
    if (j.contains("description")) j.at("description").get_to(r.description.emplace());
    if (j.contains("mimeType"))    j.at("mimeType").get_to(r.mimeType.emplace());
    if (j.contains("size"))        j.at("size").get_to(r.size.emplace());
    if (j.contains("annotations")) j.at("annotations").get_to(r.annotations.emplace());
}

/* -----------------------------------------------------------
 *  ResourceContents  (resources/read → contents[ ] 里的元素)
 * -----------------------------------------------------------*/
struct TextResourceContents {
    std::string uri;
    std::string text;
    std::optional<std::string> mimeType;
};
struct BlobResourceContents {
    std::string uri;
    std::string blob;          // base64
    std::optional<std::string> mimeType;
};

using ResourceContents = std::variant<TextResourceContents, BlobResourceContents>;

/* --- JSON ⇄ struct --- */
inline void to_json(json& j, const TextResourceContents& t)
{
    j = {{"uri", t.uri}, {"text", t.text}};
    if (t.mimeType) j["mimeType"] = *t.mimeType;
}
inline void from_json(const json& j, TextResourceContents& t)
{
    j.at("uri").get_to(t.uri);
    j.at("text").get_to(t.text);
    if (j.contains("mimeType")) j.at("mimeType").get_to(t.mimeType.emplace());
}

inline void to_json(json& j, const BlobResourceContents& b)
{
    j = {{"uri", b.uri}, {"blob", b.blob}};
    if (b.mimeType) j["mimeType"] = *b.mimeType;
}
inline void from_json(const json& j, BlobResourceContents& b)
{
    j.at("uri").get_to(b.uri);
    j.at("blob").get_to(b.blob);
    if (j.contains("mimeType")) j.at("mimeType").get_to(b.mimeType.emplace());
}

/* variant 转换器 */
inline void to_json(json& j, const ResourceContents& rc)
{
    std::visit([&](auto&& v){ j = v; }, rc);
}
inline void from_json(const json& j, ResourceContents& rc)
{
    if (j.contains("text"))
        rc = j.get<TextResourceContents>();
    else
        rc = j.get<BlobResourceContents>();
}



// /* ---------- 基础 Content 结构 ---------- */
// struct TextContent   { std::string text; json annotations; };
// struct ImageContent  { std::string mimeType; std::string data; json annotations; };
// struct AudioContent  { std::string mimeType; std::string data; json annotations; };
//
// struct EmbeddedResource {
//     json               resource;     // TextResourceContents 或 BlobResourceContents
//     json               annotations;
// };
//
// enum class ContentTag { Text, Image, Audio, Resource };

    
/* ---------- 基础内容结构 ---------- */
struct TextContent {
    std::string text;
    nlohmann::json annotations = nlohmann::json::object();
};
struct ImageContent {
    std::string mimeType;
    std::string data;           // base64
    nlohmann::json annotations = nlohmann::json::object();
};
struct AudioContent {
    std::string mimeType;
    std::string data;           // base64
    nlohmann::json annotations = nlohmann::json::object();
};
struct EmbeddedResource {
    nlohmann::json resource;    // {"uri":...,"mimeType":...,"text"/"blob":...}
    nlohmann::json annotations = nlohmann::json::object();
};


/* ---------- JSON 序列化 (显式加 type) ---------- */
inline void to_json(json& j, const TextContent& v){
    j = { {"type","text"}, {"text",v.text} };
    if(!v.annotations.empty()) j["annotations"] = v.annotations;
}
inline void from_json(const json& j, TextContent& v){
    v.text = j.at("text");
    v.annotations = j.value("annotations", json::object());
}

inline void to_json(json& j, const ImageContent& v){
    j = { {"type","image"}, {"mimeType",v.mimeType}, {"data",v.data} };
    if(!v.annotations.empty()) j["annotations"] = v.annotations;
}
inline void from_json(const json& j, ImageContent& v){
    v.mimeType    = j.at("mimeType");
    v.data        = j.at("data");
    v.annotations = j.value("annotations", json::object());
}

inline void to_json(json& j, const AudioContent& v){
    j = { {"type","audio"}, {"mimeType",v.mimeType}, {"data",v.data} };
    if(!v.annotations.empty()) j["annotations"] = v.annotations;
}
inline void from_json(const json& j, AudioContent& v){
    v.mimeType    = j.at("mimeType");
    v.data        = j.at("data");
    v.annotations = j.value("annotations", json::object());
}

inline void to_json(json& j, const EmbeddedResource& v){
    j = { {"type","resource"}, {"resource",v.resource} };
    if(!v.annotations.empty()) j["annotations"] = v.annotations;
}
inline void from_json(const json& j, EmbeddedResource& v){
    v.resource    = j.at("resource");
    v.annotations = j.value("annotations", json::object());
}


// /* ---------- JSON 转换 ---------- */
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TextContent,  text, annotations)
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImageContent, mimeType, data, annotations)
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AudioContent, mimeType, data, annotations)
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EmbeddedResource, resource, annotations)

/* ---------- 枚举 Tag ---------- */
enum class ContentTag { Text, Image, Audio, Resource };



// /* ---------- 序列化 ---------- */
// inline void to_json(json& j, const TextContent&   v){ j = {{ "type","text"  },{ "text",v.text }};  if(!v.annotations.empty())  j["annotations"]=v.annotations; }
// inline void to_json(json& j, const ImageContent&  v){ j = {{ "type","image" },{ "mimeType",v.mimeType },{ "data",v.data }}; if(!v.annotations.empty()) j["annotations"]=v.annotations; }
// inline void to_json(json& j, const AudioContent&  v){ j = {{ "type","audio" },{ "mimeType",v.mimeType },{ "data",v.data }}; if(!v.annotations.empty()) j["annotations"]=v.annotations; }
// inline void to_json(json& j, const EmbeddedResource& v){
//     j = { { "type","resource" }, { "resource",v.resource } };
//     if(!v.annotations.empty()) j["annotations"]=v.annotations;
// }

/* ---------- PromptMessage ---------- */
struct PromptMessage {
    std::string role;   // "user" / "assistant"
    // ContentTag  tag     = ContentTag::Text;
    std::variant<TextContent, ImageContent, AudioContent, EmbeddedResource> content;
};


/* JSON ↔ PromptMessage */
inline void to_json(nlohmann::json& j, const PromptMessage& pm){
    j["role"] = pm.role;
    std::visit([&](auto&& obj){ j["content"] = obj; }, pm.content);
}
inline void from_json(const nlohmann::json& j, PromptMessage& pm){
    pm.role = j.at("role");
    const auto& c = j.at("content");
    std::string type = c.value("type","text");
    if(type=="text")      pm.content = c.get<TextContent>();
    else if(type=="image")pm.content = c.get<ImageContent>();
    else if(type=="audio")pm.content = c.get<AudioContent>();
    else                   pm.content = c.get<EmbeddedResource>();
}



    // --------------------------------------
    struct Prompt {
    std::string name;
    std::string description;
    std::vector<std::string> arguments;
};

    // struct PromptMessage {
    //     nlohmann::json content;   // 直接保留 JSON, 方便传给 LLM
    //     std::string    role;
    // };



    struct GetPromptResult {
        std::string                   description;
        std::vector<PromptMessage>    messages;
    };


} // namespace mcp::protocol