#pragma once
#include <nlohmann/json.hpp>

namespace mcp::protocol {

using json = nlohmann::json;

/* ---------- 基础 Content 结构 ---------- */
struct TextContent   { std::string text; json annotations; };
struct ImageContent  { std::string mimeType; std::string data; json annotations; };
struct AudioContent  { std::string mimeType; std::string data; json annotations; };

struct EmbeddedResource {
    json               resource;     // TextResourceContents 或 BlobResourceContents
    json               annotations;
};

enum class ContentTag { Text, Image, Audio, Resource };

/* ---------- 序列化 ---------- */
inline void to_json(json& j, const TextContent&   v){ j = {{ "type","text"  },{ "text",v.text }};  if(!v.annotations.empty())  j["annotations"]=v.annotations; }
inline void to_json(json& j, const ImageContent&  v){ j = {{ "type","image" },{ "mimeType",v.mimeType },{ "data",v.data }}; if(!v.annotations.empty()) j["annotations"]=v.annotations; }
inline void to_json(json& j, const AudioContent&  v){ j = {{ "type","audio" },{ "mimeType",v.mimeType },{ "data",v.data }}; if(!v.annotations.empty()) j["annotations"]=v.annotations; }
inline void to_json(json& j, const EmbeddedResource& v){
    j = { { "type","resource" }, { "resource",v.resource } };
    if(!v.annotations.empty()) j["annotations"]=v.annotations;
}

struct PromptMessage {
    std::string role;           // "user" / "assistant"
    ContentTag  tag;
    std::variant<TextContent, ImageContent, AudioContent, EmbeddedResource> content;
};

inline void to_json(json& j, const PromptMessage& pm){
    j["role"] = pm.role;
    std::visit([&](auto&& obj){ j["content"] = obj; }, pm.content);
}

} // namespace mcp::protocol