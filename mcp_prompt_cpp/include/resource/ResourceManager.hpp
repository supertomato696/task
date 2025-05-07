#pragma once
/*****************************************************************************
 *  ResourceManager  ·  central registry for IResolver instances
 *
 *  · 提供 listResources / listTemplates / readResource 三个同步 API
 *  · 支持 subscribe(uri, cb) / unsubscribe(uri)
 *  · 本地文件型 resolver 可调用 poll() 主动检查 mtime 并触发 cb
 *
 *  Author:  architectural pass – 2025‑05‑07
 *****************************************************************************/
#include "resource/IResolver.hpp"

#include <unordered_map>
#include <vector>
#include <memory>
#include <mutex>
#include <filesystem>
#include <functional>

namespace mcp::resource {

class ResourceManager {
public:
    using UpdatedCb = std::function<void(const std::string& uri)>;

    /* ------------ resolver registration ------------ */
    void registerResolver(std::unique_ptr<IResolver> r);

    /* ------------ synchronous APIs for Service layer ------------ */
    std::vector<protocol::Resource>
    listResources();                          // O(#items)

    std::vector<protocol::ResourceTemplate>
    listTemplates();                          // May be empty

    std::vector<protocol::ResourceContents>
    readResource(const std::string& uri);     // throw if no resolver

    /* ------------ subscription for change notification ------------ */
    void subscribe  (const std::string& uri, UpdatedCb cb);
    void unsubscribe(const std::string& uri);

    /* ------------ polling (for local‑fs resolvers) ------------ */
    void poll();      // Call periodically (e.g. 2 s) from server

private:
    IResolver* findResolver(const std::string& uri) const;

    /* state */
    std::vector<std::unique_ptr<IResolver>> resolvers_;

    /* subscribe table */
    mutable std::mutex subMtx_;
    std::unordered_map<std::string, UpdatedCb> subs_;
    std::unordered_map<std::string, std::filesystem::file_time_type> mtime_;
};

} // namespace mcp::resource
