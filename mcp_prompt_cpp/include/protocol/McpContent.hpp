#pragma once
#include <nlohmann/json.hpp>
#include <variant>
#include <string>
#include <vector>

namespace mcp::protocol {

using json = nlohmann::json;

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
    ContentTag  tag     = ContentTag::Text;
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



} // namespace mcp::protocol