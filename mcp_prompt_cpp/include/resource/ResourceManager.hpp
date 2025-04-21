#pragma once
#include "resource/resolvers/IResolver.hpp"
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace mcp::resource {

class ResourceManager {
public:
    void registerResolver(std::unique_ptr<IResolver> r);

    /** 列出所有 resolver 可识别的资源 (一次性扫描) */
    std::vector<nlohmann::json> listResources();

    /** 读取单个 URI，返回 contents JSON array (符合 schema) */
    nlohmann::json readResource(const std::string& uri);

    /* ---------- subscribe/updated ---------- */
    using UpdatedCb = std::function<void(const std::string& uri)>;
    void subscribe (const std::string& uri, UpdatedCb cb);
    void unsubscribe(const std::string& uri);

    /** 周期轮询文件变更；由外部 timer 调用 */
    void poll();

private:
    std::vector<std::unique_ptr<IResolver>> resolvers_;
    std::mutex                              sub_mtx_;
    std::unordered_map<std::string, UpdatedCb> subs_;
    std::unordered_map<std::string, std::filesystem::file_time_type> mtime_;
};

}