#pragma once
#include "resource/resolvers/IResolver.hpp"
#include <string>
#include <vector>
#include "protocol/McpContent.hpp"
namespace mcp::resource {

    // using protocol      = mcp::protocol;
/**
 *  下载 http(s)://… 资源（GET）
 *  · list() 返回空，因无法枚举远程目录
 *  · 支持 follow‑redirect / 简易 MIME 推断
 */
class HttpResolver : public IResolver {
public:
    bool accepts(const std::string& uri) const override;
    std::vector<protocol::ResourceContents> read(const std::string& uri) override;
    std::vector<protocol::Resource>         list() override { return {}; }

private:
    std::string sniffMime(const std::string& url,
                          const std::string& headerMime,
                          const std::string& fallbackExt);

    std::string b64(const std::string& bin);
};

} // namespace
