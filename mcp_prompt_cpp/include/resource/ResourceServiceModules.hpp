#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <future>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <asio.hpp>
#include "McpContent.hpp"       // PromptMessage, TextResourceContents, BlobResourceContents, ResourceReference, EmbeddedResource
#include "Base64.hpp"          // Base64 encoder

namespace mcp::resource {

using json = nlohmann::json;
using content::TextResourceContents;
using content::BlobResourceContents;
using content::ResourceReference;
using content::EmbeddedResource;

// ===== ResourceResolver 接口 =====
class ResourceResolver {
public:
    virtual ~ResourceResolver() = default;
    virtual bool supports(const std::string& uri) const = 0;
    virtual std::vector<uint8_t> fetch(const std::string& uri) = 0;
    virtual std::future<std::vector<uint8_t>> fetchAsync(const std::string& uri) {
        return std::async(std::launch::async, [this, uri](){ return this->fetch(uri); });
    }
};

// ===== FileResolver: 处理 file:// URI =====
class FileResolver : public ResourceResolver {
public:
    bool supports(const std::string& uri) const override {
        return uri.rfind("file://", 0) == 0;
    }
    std::vector<uint8_t> fetch(const std::string& uri) override {
        std::string path = uri.substr(7);
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) throw std::runtime_error("File open failed: " + path);
        return std::vector<uint8_t>(std::istreambuf_iterator<char>(ifs), {});
    }
};

// ===== HttpResolver: 处理 http:// URI =====
class HttpResolver : public ResourceResolver {
public:
    HttpResolver(asio::io_context& ioc,
                 int retry_count = 2,
                 std::chrono::milliseconds timeout = std::chrono::seconds(5))
        : ioc_(ioc), retry_count_(retry_count), timeout_(timeout) {}

    bool supports(const std::string& uri) const override {
        return uri.rfind("http://", 0) == 0;
    }

    std::vector<uint8_t> fetch(const std::string& uri) override {
        // 简化实现：仅支持 http://host[:port]/path
        auto pos = uri.find('/', 7);
        std::string hostport = uri.substr(7, pos - 7);
        std::string target = uri.substr(pos);
        std::string host;
        std::string port = "80";
        auto colon = hostport.find(':');
        if (colon != std::string::npos) {
            host = hostport.substr(0, colon);
            port = hostport.substr(colon + 1);
        } else {
            host = hostport;
        }
        for (int attempt = 0; attempt <= retry_count_; ++attempt) {
            try {
                return doFetch(host, port, target);
            } catch (...) {
                if (attempt == retry_count_) throw;
                std::this_thread::sleep_for(std::chrono::milliseconds(100 * (1 << attempt)));
            }
        }
        throw std::runtime_error("HttpResolver: all retries failed");
    }

private:
    std::vector<uint8_t> doFetch(const std::string& host,
                                 const std::string& port,
                                 const std::string& target) {
        asio::ip::tcp::resolver resolver(ioc_);
        auto endpoints = resolver.resolve(host, port);
        asio::ip::tcp::socket socket(ioc_);
        asio::steady_timer timer(ioc_);
        bool timed_out = false;
        timer.expires_after(timeout_);
        timer.async_wait([&](auto ec){ if(!ec){ timed_out=true; socket.cancel(); }});

        asio::connect(socket, endpoints);
        if (timed_out) throw std::runtime_error("Connect timed out");

        // 构造 HTTP GET 请求
        std::ostringstream req;
        req << "GET " << target << " HTTP/1.1\r\n"
            << "Host: " << host << "\r\n"
            << "Connection: close\r\n\r\n";
        asio::write(socket, asio::buffer(req.str()));
        if (timed_out) throw std::runtime_error("Write timed out");

        // 读取状态行 + headers
        asio::streambuf buf;
        asio::read_until(socket, buf, "\r\n\r\n");
        if (timed_out) throw std::runtime_error("Read header timed out");

        std::istream resp(&buf);
        std::string status;
        std::getline(resp, status);
        if (status.find("200") == std::string::npos) throw std::runtime_error(status);
        std::string header;
        while(std::getline(resp, header) && header!="\r");

        // 读取 body
        std::vector<uint8_t> data;
        if (buf.size()>0) {
            auto buf_data = asio::buffer_cast<const char*>(buf.data());
            data.insert(data.end(), buf_data, buf_data + buf.size());
        }
        std::error_code ec;
        while (asio::read(socket, buf, ec)) {
            auto buf_data = asio::buffer_cast<const char*>(buf.data());
            data.insert(data.end(), buf_data, buf_data + buf.size());
            buf.consume(buf.size());
        }
        timer.cancel();
        return data;
    }

    asio::io_context& ioc_;
    int retry_count_;
    std::chrono::milliseconds timeout_;
};

// ===== ResourceService: 调用 Resolver 并封装 MCP 类型 =====
class ResourceService {
public:
    void addResolver(std::shared_ptr<ResourceResolver> resolver) {
        resolvers_.push_back(resolver);
    }

    // 同步获取，并封装为 Text or Blob
    std::variant<TextResourceContents, BlobResourceContents> get(const std::string& uri) {
        auto bytes = fetchBytes(uri);
        if (isText(uri)) {
            TextResourceContents tr;
            tr.uri = uri;
            tr.text = std::string(bytes.begin(), bytes.end());
            tr.mimeType = guessMime(uri);
            return tr;
        } else {
            BlobResourceContents br;
            br.uri = uri;
            br.blob = Base64::encode(bytes.data(), bytes.size());
            br.mimeType = guessMime(uri);
            return br;
        }
    }

    // 异步获取
    std::future<std::variant<TextResourceContents, BlobResourceContents>> getAsync(const std::string& uri) {
        return std::async(std::launch::async, [this, uri]() { return get(uri); });
    }

private:
    std::vector<std::shared_ptr<ResourceResolver>> resolvers_;

    std::vector<uint8_t> fetchBytes(const std::string& uri) {
        for (auto& r: resolvers_) {
            if (r->supports(uri)) {
                return r->fetch(uri);
            }
        }
        throw std::runtime_error("No resolver for URI: " + uri);
    }

    bool isText(const std::string& uri) const {
        static const std::vector<std::string> exts = {".txt",".json",".md",".xml"};
        for (auto& e: exts) if (uri.rfind(e) == uri.size()-e.size()) return true;
        return false;
    }

    std::string guessMime(const std::string& uri) const {
        auto pos = uri.rfind('.');
        if (pos==std::string::npos) return "application/octet-stream";
        auto ext = uri.substr(pos+1);
        if (ext=="txt") return "text/plain";
        if (ext=="json") return "application/json";
        if (ext=="png") return "image/png";
        if (ext=="jpg"||ext=="jpeg") return "image/jpeg";
        if (ext=="mp3") return "audio/mpeg";
        return "application/octet-stream";
    }
};

} // namespace mcp::resource


// ===== MultimodalAssembler: 整合渲染和资源加载 =====
namespace mcp::utils {

using resource::ResourceService;
using content::PromptMessage;
using content::EmbeddedResource;
using content::ResourceReference;
using content::TextContent;
using content::ImageContent;
using content::AudioContent;

class MultimodalAssembler {
public:
    MultimodalAssembler(std::shared_ptr<ResourceService> svc)
        : service_(std::move(svc)) {}

    std::vector<PromptMessage> assemble(
        const std::vector<PromptMessage>& templates,
        const std::map<std::string, std::string>& args
    ) {
        // 第一步: 文本模板和引用嵌入
        auto rendered = TemplateEngine::render(templates, args);
        std::vector<PromptMessage> out;
        // 第二步: 资源加载
        for (auto& msg: rendered) {
            if (auto* ref = std::get_if<ResourceReference>(&msg.content)) {
                auto resVar = service_->get(ref->uri);
                EmbeddedResource er;
                er.type = "resource";
                er.annotations = {};
                // 填充 resource 内容
                if (std::holds_alternative<TextResourceContents>(resVar)) {
                    er.resource = std::get<TextResourceContents>(resVar);
                } else {
                    er.resource = std::get<BlobResourceContents>(resVar);
                }
                msg.content = er;
            }
            out.push_back(msg);
        }
        return out;
    }

private:
    std::shared_ptr<ResourceService> service_;
};

} // namespace mcp::utils
