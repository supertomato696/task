#pragma once
#include "Types.hpp"
#include <asio.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <atomic>

namespace mcp::client {

/** 简单阻塞式 Client：每次请求同步等待响应；连接断掉自动重连 */
class McpClient {
public:

    /** host 可为 "127.0.0.1", port 9275 */
    McpClient(std::string host, uint16_t port);
    ~McpClient();

    std::vector<Prompt>   listPrompts();
    GetPromptResult           getPrompt(const std::string& name, const ArgMap& args = {});
public:
   std::future<std::vector<Prompt>> listPromptsAsync();
   std::future<GetPromptResult>         getPromptAsync(const std::string& name, const ArgMap& args = {});


public:
  std::future<std::vector<nlohmann::json>> listResourcesAsync();
//   std::future<nlohmann::json> readResourceAsync(const std::string& uri);

    //    std::future<nlohmann::json> listResourcesAsync();
       std::future<nlohmann::json> readResourceAsync(const std::string& uri);

public:
std::future<std::vector<nlohmann::json>> listToolsAsync(){
    return asyncRpcCall("tools/list", {}, *this);
}
std::future<nlohmann::json> callToolAsync(const std::string& name,const json& args = {}){
    return asyncRpcCall("tools/call", {{"name",name},{"arguments",args}}, *this);
}

private:
    nlohmann::json rpcCall(const std::string& method, nlohmann::json params);
    std::future<nlohmann::json> asyncRpcCall(const std::string& method, nlohmann::json params, McpClient& self);

    /* ---- TCP internals ---- */
    void start();
    void do_connect();
    void do_read();
    void send_raw(const std::string& txt);

    asio::io_context                  io_;
    asio::ip::tcp::resolver           resolver_;
    std::shared_ptr<asio::ip::tcp::socket> sock_;
    asio::streambuf                   buf_;
    const std::string                 host_;
    uint16_t                          port_;
    std::jthread                      th_;

    std::mutex                        inflight_mtx_;
    std::condition_variable           inflight_cv_;
    std::unordered_map<std::string, nlohmann::json> responses_;
        /* ------------ for async ------------- */
    struct Pending {
        std::promise<nlohmann::json> prom;
    };
    std::unordered_map<std::string, std::shared_ptr<Pending>> pendings_;

    std::atomic_uint64_t              id_gen_{1};
};

} // namespace mcp::client


#pragma once
#include <string>
#include <vector>
#include <optional>
#include <variant>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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
        j.at("annotations").get_to(a.annotations);
    }
}

// —— 嵌入式资源 ——
// 嵌入到消息中的资源，可能是文本或二进制
struct TextResourceContents {
    std::string text;    // 文本内容
    std::string uri;     // 资源 URI
    std::optional<std::string> mimeType;
};

inline void to_json(json& j, const TextResourceContents& t) {
    j = json{{"text", t.text}, {"uri", t.uri}};
    if (t.mimeType) j["mimeType"] = *t.mimeType;
}

inline void from_json(const json& j, TextResourceContents& t) {
    j.at("text").get_to(t.text);
    j.at("uri").get_to(t.uri);
    if (j.contains("mimeType")) {
        j.at("mimeType").get_to(t.mimeType);
    }
}

struct BlobResourceContents {
    std::string blob;    // base64 编码的二进制数据
    std::string uri;     // 资源 URI
    std::optional<std::string> mimeType;
};

inline void to_json(json& j, const BlobResourceContents& b) {
    j = json{{"blob", b.blob}, {"uri", b.uri}};
    if (b.mimeType) j["mimeType"] = *b.mimeType;
}

inline void from_json(const json& j, BlobResourceContents& b) {
    j.at("blob").get_to(b.blob);
    j.at("uri").get_to(b.uri);
    if (j.contains("mimeType")) {
        j.at("mimeType").get_to(b.mimeType);
    }
}

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
        j.at("annotations").get_to(e.annotations);
    }
}

// —— Resource ——
// 服务端已知、可读取的资源
struct Resource {
    std::string name;                      // 资源名称
    std::string uri;                       // 资源 URI
    std::optional<std::string> description;
    std::optional<std::string> mimeType;
    std::optional<int64_t> size;          // 原始内容大小（字节）
    std::optional<Annotations> annotations;
};

inline void to_json(json& j, const Resource& r) {
    j = json{
        {"name", r.name},
        {"uri",  r.uri}
    };
    if (r.description)  j["description"]  = *r.description;
    if (r.mimeType)     j["mimeType"]     = *r.mimeType;
    if (r.size)         j["size"]         = *r.size;
    if (r.annotations)  j["annotations"]  = *r.annotations;
}

inline void from_json(const json& j, Resource& r) {
    j.at("name").get_to(r.name);
    j.at("uri").get_to(r.uri);
    if (j.contains("description"))  j.at("description").get_to(r.description);
    if (j.contains("mimeType"))     j.at("mimeType").get_to(r.mimeType);
    if (j.contains("size"))         j.at("size").get_to(r.size);
    if (j.contains("annotations"))  j.at("annotations").get_to(r.annotations);
}


#pragma once
#include <string>
#include <vector>
#include <optional>
#include <variant>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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
        j.at("annotations").get_to(a.annotations);
    }
}

// —— 嵌入式资源 ——
// 嵌入到消息中的资源，可能是文本或二进制
struct TextResourceContents {
    std::string text;    // 文本内容
    std::string uri;     // 资源 URI
    std::optional<std::string> mimeType;
};

inline void to_json(json& j, const TextResourceContents& t) {
    j = json{{"text", t.text}, {"uri", t.uri}};
    if (t.mimeType) j["mimeType"] = *t.mimeType;
}

inline void from_json(const json& j, TextResourceContents& t) {
    j.at("text").get_to(t.text);
    j.at("uri").get_to(t.uri);
    if (j.contains("mimeType")) {
        j.at("mimeType").get_to(t.mimeType);
    }
}

struct BlobResourceContents {
    std::string blob;    // base64 编码的二进制数据
    std::string uri;     // 资源 URI
    std::optional<std::string> mimeType;
};

inline void to_json(json& j, const BlobResourceContents& b) {
    j = json{{"blob", b.blob}, {"uri", b.uri}};
    if (b.mimeType) j["mimeType"] = *b.mimeType;
}

inline void from_json(const json& j, BlobResourceContents& b) {
    j.at("blob").get_to(b.blob);
    j.at("uri").get_to(b.uri);
    if (j.contains("mimeType")) {
        j.at("mimeType").get_to(b.mimeType);
    }
}

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
        j.at("annotations").get_to(e.annotations);
    }
}

// —— Resource ——
// 服务端已知、可读取的资源
struct Resource {
    std::string name;                      // 资源名称
    std::string uri;                       // 资源 URI
    std::optional<std::string> description;
    std::optional<std::string> mimeType;
    std::optional<int64_t> size;          // 原始内容大小（字节）
    std::optional<Annotations> annotations;
};

inline void to_json(json& j, const Resource& r) {
    j = json{
        {"name", r.name},
        {"uri",  r.uri}
    };
    if (r.description)  j["description"]  = *r.description;
    if (r.mimeType)     j["mimeType"]     = *r.mimeType;
    if (r.size)         j["size"]         = *r.size;
    if (r.annotations)  j["annotations"]  = *r.annotations;
}

inline void from_json(const json& j, Resource& r) {
    j.at("name").get_to(r.name);
    j.at("uri").get_to(r.uri);
    if (j.contains("description"))  j.at("description").get_to(r.description);
    if (j.contains("mimeType"))     j.at("mimeType").get_to(r.mimeType);
    if (j.contains("size"))         j.at("size").get_to(r.size);
    if (j.contains("annotations"))  j.at("annotations").get_to(r.annotations);
}

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
    if (j.contains("annotations"))  j.at("annotations").get_to(rt.annotations);
}

// ——— 工具调用结果中的内容块 ———

// CallToolResult 中单条 content 可以和 PromptMessage 类似，因此可复用 TextContent、ImageContent、AudioContent、EmbeddedResource

// CallToolResult 结构体示例
struct CallToolResult {
    std::vector<std::variant<TextContent, ImageContent, AudioContent, EmbeddedResource>> content;
    std::optional<bool> isError;
    std::optional<json> _meta;
};

inline void to_json(json& j, const CallToolResult& ctr) {
    if (ctr._meta)  j["_meta"] = *ctr._meta;
    j["content"] = json::array();
    for (auto& item : ctr.content) {
        json elem;
        std::visit([&elem](auto&& arg){ elem = arg; }, item);
        j["content"].push_back(elem);
    }
    if (ctr.isError) j["isError"] = *ctr.isError;
}

inline void from_json(const json& j, CallToolResult& ctr) {
    if (j.contains("_meta"))   j.at("_meta").get_to(ctr._meta);
    for (auto& elem : j.at("content")) {
        auto type = elem.at("type").get<std::string>();
        if (type == "text")     ctr.content.emplace_back(elem.get<TextContent>());
        else if (type == "image")   ctr.content.emplace_back(elem.get<ImageContent>());
        else if (type == "audio")   ctr.content.emplace_back(elem.get<AudioContent>());
        else if (type == "resource")ctr.content.emplace_back(elem.get<EmbeddedResource>());
    }
    if (j.contains("isError")) j.at("isError").get_to(ctr.isError);
}

#pragma once
#include <string>
#include <vector>
#include <optional>
#include <variant>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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
        j.at("annotations").get_to(a.annotations);
    }
}

// —— 嵌入式资源 ——
// 嵌入到消息中的资源，可能是文本或二进制
struct TextResourceContents {
    std::string text;    // 文本内容
    std::string uri;     // 资源 URI
    std::optional<std::string> mimeType;
};

inline void to_json(json& j, const TextResourceContents& t) {
    j = json{{"text", t.text}, {"uri", t.uri}};
    if (t.mimeType) j["mimeType"] = *t.mimeType;
}

inline void from_json(const json& j, TextResourceContents& t) {
    j.at("text").get_to(t.text);
    j.at("uri").get_to(t.uri);
    if (j.contains("mimeType")) {
        j.at("mimeType").get_to(t.mimeType);
    }
}

struct BlobResourceContents {
    std::string blob;    // base64 编码的二进制数据
    std::string uri;     // 资源 URI
    std::optional<std::string> mimeType;
};

inline void to_json(json& j, const BlobResourceContents& b) {
    j = json{{"blob", b.blob}, {"uri", b.uri}};
    if (b.mimeType) j["mimeType"] = *b.mimeType;
}

inline void from_json(const json& j, BlobResourceContents& b) {
    j.at("blob").get_to(b.blob);
    j.at("uri").get_to(b.uri);
    if (j.contains("mimeType")) {
        j.at("mimeType").get_to(b.mimeType);
    }
}

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
        j.at("annotations").get_to(e.annotations);
    }
}

// —— Resource ——
// 服务端已知、可读取的资源
struct Resource {
    std::string name;                      // 资源名称
    std::string uri;                       // 资源 URI
    std::optional<std::string> description;
    std::optional<std::string> mimeType;
    std::optional<int64_t> size;          // 原始内容大小（字节）
    std::optional<Annotations> annotations;
};

inline void to_json(json& j, const Resource& r) {
    j = json{
        {"name", r.name},
        {"uri",  r.uri}
    };
    if (r.description)  j["description"]  = *r.description;
    if (r.mimeType)     j["mimeType"]     = *r.mimeType;
    if (r.size)         j["size"]         = *r.size;
    if (r.annotations)  j["annotations"]  = *r.annotations;
}

inline void from_json(const json& j, Resource& r) {
    j.at("name").get_to(r.name);
    j.at("uri").get_to(r.uri);
    if (j.contains("description"))  j.at("description").get_to(r.description);
    if (j.contains("mimeType"))     j.at("mimeType").get_to(r.mimeType);
    if (j.contains("size"))         j.at("size").get_to(r.size);
    if (j.contains("annotations"))  j.at("annotations").get_to(r.annotations);
}

