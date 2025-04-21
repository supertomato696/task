#pragma once
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace mcp::resource {

/** 解析特定 URI scheme 的抽象接口 */
class IResolver {
public:
    virtual ~IResolver() = default;

    /** 返回 true 表示此 resolver 能处理该 URI */
    virtual bool accepts(const std::string& uri) const = 0;

    /** 将外部资源读取为 TextResourceContents / BlobResourceContents JSON 数组 */
    virtual nlohmann::json read(const std::string& uri) = 0;

    /** 枚举根目录下可列举资源；非文件型 resolver 返回空 */
    virtual std::vector<nlohmann::json> list() { return {}; }
    virtual ~IResolver() = default;
};

} // namespace mcp::resource