#pragma once
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <future>
#include <variant>

namespace mcp::client {

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

    struct Annotations {
        std::vector<Role> audience;   // [必填] 使用枚举类型
        std::optional<double> priority; // [可选] 0-1
    };

    // JSON 转换实现
    inline void to_json(nlohmann::json& j, const Annotations& a) {
        j = nlohmann::json::object();
        j["audience"] = a.audience; // 自动使用枚举转换器

        if (a.priority) {
            const double val = *a.priority;
            if (val >= 0.0 && val <= 1.0) {
                j["priority"] = val;
            }
            // 超出范围不序列化
        }
    }

    inline void from_json(const nlohmann::json& j, Annotations& a) {
        // audience 必须为合法枚举数组
        if (j.contains("audience") && j["audience"].is_array()) {
            j.at("audience").get_to(a.audience);
        } else {
            a.audience.clear();
        }

        // priority 范围校验
        if (j.contains("priority") && j["priority"].is_number()) {
            const double val = j["priority"].get<double>();
            if (val >= 0.0 && val <= 1.0) {
                a.priority = val;
            } else {
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

    // 新增分页结果结构体
    struct ListPromptsResult {
        std::optional<nlohmann::json> _meta;     // [可选] 元数据
        std::optional<std::string> nextCursor;   // [可选] 分页游标
        std::vector<Prompt> prompts;             // [必填] 提示列表
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


    inline void to_json(nlohmann::json& j, const ListPromptsResult& res) {
        j = {{"prompts", res.prompts}}; // 必须包含 prompts
        if (res._meta) j["_meta"] = *res._meta;
        if (res.nextCursor) j["nextCursor"] = *res.nextCursor;
    }

    inline void from_json(const nlohmann::json& j, ListPromptsResult& res) {
        j.at("prompts").get_to(res.prompts);
        if (j.contains("_meta")) j.at("_meta").get_to(res._meta.emplace());
        if (j.contains("nextCursor")) j.at("nextCursor").get_to(res.nextCursor.emplace());
    }



    // 修改后的 GetPromptResult 定义
    struct GetPromptResult {
        std::optional<nlohmann::json> _meta;       // [可选] 元数据
        std::optional<std::string> description;    // [可选] 改为可选
        std::vector<PromptMessage> messages;       // [必填] 必须存在（可为空数组）
    };

    // 对应的 JSON 转换
    inline void to_json(nlohmann::json& j, const GetPromptResult& res) {
        j = nlohmann::json::object(); // 显式创建空对象

        // 必须包含 messages（即使为空数组）
        j["messages"] = res.messages;

        // 可选字段处理
        if (res._meta) {
            j["_meta"] = *res._meta;
        }
        if (res.description) {
            j["description"] = *res.description;
        }
    }

    inline void from_json(const nlohmann::json& j, GetPromptResult& res) {
        // 强制要求 messages 字段存在
        j.at("messages").get_to(res.messages);

        // 处理可选字段
        if (j.contains("_meta")) {
            j.at("_meta").get_to(res._meta.emplace());
        }
        if (j.contains("description")) {
            j.at("description").get_to(res.description.emplace());
        }
    }

    // Types.hpp 新增内容结构体
    struct TextContent {
        std::string text;                   // [必填]
        std::optional<Annotations> annotations; // [可选]
    };

    struct ImageContent {
        std::string data;        // [必填] base64
        std::string mimeType;    // [必填]
        std::optional<Annotations> annotations;
    };

    struct AudioContent {
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
        j.at("type").get_to<std::string>(); // 验证 type 字段
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


    inline void to_json(nlohmann::json& j, const AudioContent& a) {
        j = {
            {"type", "audio"},
            {"data", a.data},
            {"mimeType", a.mimeType}
        };
        if (a.annotations) {
            j["annotations"] = *a.annotations;
        }
    }

    inline void from_json(const nlohmann::json& j, AudioContent& a) {
        if (j.at("type") != "audio") throw std::invalid_argument("Invalid type");
        j.at("data").get_to(a.data);
        j.at("mimeType").get_to(a.mimeType);
        if (j.contains("annotations")) {
            j.at("annotations").get_to(a.annotations.emplace());
        }
    }


    // 修改为类型安全的内容容器
    using Content = std::variant<TextContent, ImageContent, AudioContent>;

    struct PromptMessage {
        std::string role;   // "user" 或 "assistant"
        Content content;    // 强类型内容
    };


    // Content 的变体序列化
    inline void to_json(nlohmann::json& j, const Content& c) {
        std::visit([&](auto&& arg) { j = arg; }, c);
    }

    // 更新 Content 的反序列化逻辑
    inline void from_json(const nlohmann::json& j, Content& c) {
        const std::string type = j.at("type");
        if (type == "text") {
            c = j.get<TextContent>();
        } else if (type == "image") {
            c = j.get<ImageContent>();
        } else if (type == "audio") {
            c = j.get<AudioContent>();
        } else if (type == "resource") { // 新增资源类型处理
            c = j.get<EmbeddedResource>();
        } else {
            // throw nlohmann::json::parse_error::create(
            //     501, "Unknown content type: " + type, &j
            // );
        }
    }



    // Types.hpp 中新增/修改内容
    struct EmbeddedResource {
        ResourceContents resource;          // [必填] Text/Blob 资源变体
        std::optional<Annotations> annotations; // [可选]
    };

    // 更新内容变体类型
    using Content = std::variant<
        TextContent,
        ImageContent,
        AudioContent,
        EmbeddedResource // 新增嵌入式资源类型
    >;


    inline void to_json(nlohmann::json& j, const EmbeddedResource& er) {
        j = {
            {"type", "resource"},
            {"resource", er.resource}
        };
        if (er.annotations) {
            j["annotations"] = *er.annotations;
        }
    }

    inline void from_json(const nlohmann::json& j, EmbeddedResource& er) {
        if (j.at("type") != "resource") {
            // throw nlohmann::json::parse_error::create(
            //     501, "Invalid type for EmbeddedResource", &j
            // );
        }
        j.at("resource").get_to(er.resource);
        if (j.contains("annotations")) {
            j.at("annotations").get_to(er.annotations.emplace());
        }
    }


    bool isTextResource(const EmbeddedResource& res) {
        return std::holds_alternative<TextResourceContents>(res.resource);
    }


    // 添加资源访问助手函数
    template<typename T>
    const T* getResource(const EmbeddedResource& res) {
        return std::get_if<T>(&res.resource);
    }

    // 使用示例
    if (auto* txt = getResource<TextResourceContents>(er)) {
        std::cout << "Text resource: " << txt->text;
    }




    // Types.hpp 中修改/新增内容

    // 基础资源内容结构体
    struct ResourceContents {
        std::string uri;                   // [必填]
        std::optional<std::string> mimeType; // [可选]
    };

    // 文本资源内容
    struct TextResourceContents : ResourceContents {
        std::string text; // [必填]
    };

    // 二进制资源内容
    struct BlobResourceContents : ResourceContents {
        std::string blob; // [必填] base64
    };

    // 资源类型变体
    using ResourceContentsVariant = std::variant<
        TextResourceContents,
        BlobResourceContents
    >;

    // 完整资源元数据
    struct Resource {
        std::string name;                   // [必填]
        std::string uri;                    // [必填]
        std::optional<std::string> description; // [可选]
        std::optional<std::string> mimeType;    // [可选]
        std::optional<uint64_t> size;           // [可选]
        std::optional<Annotations> annotations; // [可选]
    };


    // 基类通用字段处理
    inline void to_json(nlohmann::json& j, const ResourceContents& rc) {
        j = {{"uri", rc.uri}};
        if (rc.mimeType) j["mimeType"] = *rc.mimeType;
    }

    inline void from_json(const nlohmann::json& j, ResourceContents& rc) {
        j.at("uri").get_to(rc.uri);
        if (j.contains("mimeType")) j.at("mimeType").get_to(rc.mimeType.emplace());
    }


    inline void to_json(nlohmann::json& j, const TextResourceContents& t) {
        to_json(j, static_cast<const ResourceContents&>(t)); // 序列化基类
        j["text"] = t.text;
    }

    inline void from_json(const nlohmann::json& j, TextResourceContents& t) {
        from_json(j, static_cast<ResourceContents&>(t)); // 反序列化基类
        j.at("text").get_to(t.text);
    }


    inline void to_json(nlohmann::json& j, const BlobResourceContents& b) {
        to_json(j, static_cast<const ResourceContents&>(b)); // 序列化基类
        j["blob"] = b.blob;
    }

    inline void from_json(const nlohmann::json& j, BlobResourceContents& b) {
        from_json(j, static_cast<ResourceContents&>(b)); // 反序列化基类
        j.at("blob").get_to(b.blob);
    }


    inline void to_json(nlohmann::json& j, const Resource& r) {
        j = {
            {"name", r.name},
            {"uri", r.uri}
        };
        if (r.description) j["description"] = *r.description;
        if (r.mimeType) j["mimeType"] = *r.mimeType;
        if (r.size) j["size"] = *r.size;
        if (r.annotations) j["annotations"] = *r.annotations;
    }

    inline void from_json(const nlohmann::json& j, Resource& r) {
        j.at("name").get_to(r.name);
        j.at("uri").get_to(r.uri);
        if (j.contains("description")) j.at("description").get_to(r.description.emplace());
        if (j.contains("mimeType")) j.at("mimeType").get_to(r.mimeType.emplace());
        if (j.contains("size")) j.at("size").get_to(r.size.emplace());
        if (j.contains("annotations")) j.at("annotations").get_to(r.annotations.emplace());
    }


    // ResourceContents 变体序列化
    inline void to_json(nlohmann::json& j, const ResourceContentsVariant& rc) {
        std::visit([&](auto&& arg) { j = arg; }, rc);
    }

    inline void from_json(const nlohmann::json& j, ResourceContentsVariant& rc) {
        if (j.contains("text")) {
            rc = j.get<TextResourceContents>();
        } else if (j.contains("blob")) {
            rc = j.get<BlobResourceContents>();
        } else {
            // throw nlohmann::json::parse_error::create(
            //     501, "Unknown resource type", &j
            // );
        }
    }






// struct Prompt {
//     std::string name;
//     std::string description;
//     std::vector<std::string> arguments;
// };
//
// struct PromptMessage {
//     nlohmann::json content;   // 直接保留 JSON, 方便传给 LLM
//     std::string    role;
// };
//
//
//
// struct GetPromptResult {
//     std::string                   description;
//     std::vector<PromptMessage>    messages;
// };
//
// using ArgMap = std::unordered_map<std::string,std::string>;

} // namespace mcp::client