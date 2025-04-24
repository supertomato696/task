#pragma once
#include <nlohmann/json.hpp>
#include <variant>
#include <string>
#include <vector>
#include <optional>
#include <variant>

namespace mcp::protocol {

using json = nlohmann::json;


enum class Role {
    User,       // 对应协议 "user"
    Assistant   // 对应协议 "assistant"
};

// 枚举 ⇄ JSON 转换
inline void to_json(nlohmann::json& j, Role role) {
    switch (role) {
        case Role::User:       j = "user"; break;
        case Role::Assistant:  j = "assistant"; break;
        default: throw std::invalid_argument("Invalid Role value");
    }
}

inline void from_json(const nlohmann::json& j, Role& role) {
    const std::string s = j.get<std::string>();
    if (s == "user") {
        role = Role::User;
    } else if (s == "assistant") {
        role = Role::Assistant;
    } else {
        // throw nlohmann::json::parse_error::create(501, "Invalid Role value: " + s, &j
        // );
    }
}

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



   // 新增 PromptArgument 结构体
   struct PromptArgument {
    std::string name;                   // [必填] 参数名称
    std::optional<std::string> description; // [可选] 参数描述
    bool required = false;              // [必填] 是否必须提供
};

// 重命名/修改 PromptInfo 为 Prompt
struct Prompt {
    std::string name;                   // [必填] 提示名称
    std::optional<std::string> description; // [可选] 描述
    std::vector<PromptArgument> arguments; // [可选] 参数列表
};

inline void to_json(nlohmann::json& j, const PromptArgument& arg) {
    j = {{"name", arg.name}, {"required", arg.required}};
    if (arg.description) j["description"] = *arg.description;
}

inline void from_json(const nlohmann::json& j, PromptArgument& arg) {
    j.at("name").get_to(arg.name);
    j.at("required").get_to(arg.required);
    if (j.contains("description")) j.at("description").get_to(arg.description.emplace());
}


inline void to_json(nlohmann::json& j, const Prompt& p) {
    j = {{"name", p.name}};
    if (p.description) j["description"] = *p.description;
    if (!p.arguments.empty()) j["arguments"] = p.arguments;
}

inline void from_json(const nlohmann::json& j, Prompt& p) {
    j.at("name").get_to(p.name);
    if (j.contains("description")) j.at("description").get_to(p.description.emplace());
    if (j.contains("arguments")) j.at("arguments").get_to(p.arguments);
}

    // —— AudioContent ——
// Audio 提供给或来自 LLM 的音频
struct AudioContent {
    // 必填字段
    std::string type{"audio"};
    std::string data;        // base64 编码的音频数据
    std::string mimeType;    // 音频的 MIME 类型
    // 可选注释
    std::optional<Annotations> annotations;
};

inline void to_json(json& j, const AudioContent& a) {
    j = json{
        {"type",     a.type},
        {"data",     a.data},
        {"mimeType", a.mimeType}
    };
    if (a.annotations) {
        j["annotations"] = *a.annotations;
    }
}

inline void from_json(const json& j, AudioContent& a) {
    // “type” 应始终为 "audio"，可选校验
    j.at("data").get_to(a.data);
    j.at("mimeType").get_to(a.mimeType);
    if (j.contains("annotations")) {
        j.at("annotations").get_to(a.annotations.emplace());
    }
}

/* -----------------------------------------------------------
 *  ImageContent  (resources/read → contents[ ] 里的元素)
 * -----------------------------------------------------------*/
struct TextContent {
    std::string text;
    std::string type{"text"};// [必填]
    std::optional<Annotations> annotations; // [可选]
};

struct ImageContent {
    std::string data;        // [必填] base64
    std::string mimeType;    // [必填]
    std::optional<Annotations> annotations;
};


inline void to_json(nlohmann::json& j, const TextContent& t) {
    j = {
        {"type", "text"},
        {"text", t.text}
    };
    if (t.annotations) {
        j["annotations"] = *t.annotations;
    }
}

inline void from_json(const nlohmann::json& j, TextContent& t) {
    // j.at("type").get_to<std::string>(); // 验证 type 字段
    j.at("text").get_to(t.text);
    if (j.contains("annotations")) {
        j.at("annotations").get_to(t.annotations.emplace());
    }
}


inline void to_json(nlohmann::json& j, const ImageContent& img) {
    j = {
        {"type", "image"},
        {"data", img.data},
        {"mimeType", img.mimeType}
    };
    if (img.annotations) {
        j["annotations"] = *img.annotations;
    }
}

inline void from_json(const nlohmann::json& j, ImageContent& img) {
    if (j.at("type") != "image") throw std::invalid_argument("Invalid type");
    j.at("data").get_to(img.data);
    j.at("mimeType").get_to(img.mimeType);
    if (j.contains("annotations")) {
        j.at("annotations").get_to(img.annotations.emplace());
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

 
// ResourceContents：读取单个资源时返回的内容描述（可以是子资源）
struct ResourceContents {
    std::string uri;                       // 必须
    std::optional<std::string> mimeType;
};

inline void to_json(json& j, const ResourceContents& rc) {
    j = json{{"uri", rc.uri}};
    if (rc.mimeType) j["mimeType"] = *rc.mimeType;
}

inline void from_json(const json& j, ResourceContents& rc) {
    j.at("uri").get_to(rc.uri);
    if (j.contains("mimeType")) j.at("mimeType").get_to(rc.mimeType);
}

// ResourceTemplate：资源模板，用于构造新 URI
struct ResourceTemplate {
    std::string name;                      // 必须
    std::string uriTemplate;               // 必须
    std::optional<std::string> description;
    std::optional<std::string> mimeType;
    std::optional<Annotations> annotations;
};

inline void to_json(json& j, const ResourceTemplate& rt) {
    j = json{
        {"name",        rt.name},
        {"uriTemplate", rt.uriTemplate}
    };
    if (rt.description)  j["description"]  = *rt.description;
    if (rt.mimeType)     j["mimeType"]     = *rt.mimeType;
    if (rt.annotations)  j["annotations"]  = *rt.annotations;
}

inline void from_json(const json& j, ResourceTemplate& rt) {
    j.at("name").get_to(rt.name);
    j.at("uriTemplate").get_to(rt.uriTemplate);
    if (j.contains("description"))  j.at("description").get_to(rt.description);
    if (j.contains("mimeType"))     j.at("mimeType").get_to(rt.mimeType);
    if (j.contains("annotations"))  j.at("annotations").get_to(rt.annotations.emplace());
}


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

/* -----------------------------------------------------------
 *  EmbeddedResource  (resources/read → contents[ ] 里的元素)
 * -----------------------------------------------------------*/


struct EmbeddedResource {
    std::string type{"resource"};
    // 资源内容：文本 or 二进制
    std::variant<TextResourceContents, BlobResourceContents> resource;
    std::optional<Annotations> annotations;
};

inline void to_json(json& j, const EmbeddedResource& e) {
    j["type"] = e.type;
    std::visit([&j](auto&& arg){
        j["resource"] = arg;
    }, e.resource);
    if (e.annotations) {
        j["annotations"] = *e.annotations;
    }
}

inline void from_json(const json& j, EmbeddedResource& e) {
    const auto& r = j.at("resource");
    // 依据字段判断是哪种类型
    if (r.contains("text")) {
        e.resource = r.get<TextResourceContents>();
    } else {
        e.resource = r.get<BlobResourceContents>();
    }
    if (j.contains("annotations")) {
        j.at("annotations").get_to(e.annotations.emplace());
    }
}


    
// /* ---------- 基础内容结构 ---------- */
// struct TextContent {
//     std::string text;
//     nlohmann::json annotations = nlohmann::json::object();
// };
// struct ImageContent {
//     std::string mimeType;
//     std::string data;           // base64
//     nlohmann::json annotations = nlohmann::json::object();
// };
// struct AudioContent {
//     std::string mimeType;
//     std::string data;           // base64
//     nlohmann::json annotations = nlohmann::json::object();
// };
// struct EmbeddedResource {
//     nlohmann::json resource;    // {"uri":...,"mimeType":...,"text"/"blob":...}
//     nlohmann::json annotations = nlohmann::json::object();
// };


// /* ---------- JSON 序列化 (显式加 type) ---------- */
// inline void to_json(json& j, const TextContent& v){
//     j = { {"type","text"}, {"text",v.text} };
//     if(!v.annotations.empty()) j["annotations"] = v.annotations;
// }
// inline void from_json(const json& j, TextContent& v){
//     v.text = j.at("text");
//     v.annotations = j.value("annotations", json::object());
// }

// inline void to_json(json& j, const ImageContent& v){
//     j = { {"type","image"}, {"mimeType",v.mimeType}, {"data",v.data} };
//     if(!v.annotations.empty()) j["annotations"] = v.annotations;
// }
// inline void from_json(const json& j, ImageContent& v){
//     v.mimeType    = j.at("mimeType");
//     v.data        = j.at("data");
//     v.annotations = j.value("annotations", json::object());
// }

// inline void to_json(json& j, const AudioContent& v){
//     j = { {"type","audio"}, {"mimeType",v.mimeType}, {"data",v.data} };
//     if(!v.annotations.empty()) j["annotations"] = v.annotations;
// }
// inline void from_json(const json& j, AudioContent& v){
//     v.mimeType    = j.at("mimeType");
//     v.data        = j.at("data");
//     v.annotations = j.value("annotations", json::object());
// }

// inline void to_json(json& j, const EmbeddedResource& v){
//     j = { {"type","resource"}, {"resource",v.resource} };
//     if(!v.annotations.empty()) j["annotations"] = v.annotations;
// }
// inline void from_json(const json& j, EmbeddedResource& v){
//     v.resource    = j.at("resource");
//     v.annotations = j.value("annotations", json::object());
// }


// /* ---------- JSON 转换 ---------- */
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TextContent,  text, annotations)
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImageContent, mimeType, data, annotations)
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AudioContent, mimeType, data, annotations)
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EmbeddedResource, resource, annotations)



/* ---------- PromptMessage ---------- */
struct PromptMessage {
    std::string role;   // "user" / "assistant"

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