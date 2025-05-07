#pragma once
#include "resource/IResolver.hpp"
#include <filesystem>

namespace mcp::resource {

/**
 *  读取 file://… 资源
 *  · root 目录限定，防止越界访问
 *  · list() 递归扫描 root，生成 protocol::Resource
 */
class LocalFileResolver : public IResolver {
public:
    explicit LocalFileResolver(std::filesystem::path root);

    bool accepts(const std::string& uri) const override;
    std::vector<protocol::ResourceContents> read(const std::string& uri) override;
    std::vector<protocol::Resource>         list() override;

private:
    std::string toMime(const std::filesystem::path& p) const;
    bool        isText(const std::string& mime) const;

    std::filesystem::path root_;
};

} // namespace
