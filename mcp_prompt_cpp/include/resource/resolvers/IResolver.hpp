// include/resource/IResolver.hpp
#pragma once
#include "protocol/McpResourceRequests.hpp"

namespace mcp::resource {

using protocol::Resource;
using protocol::ResourceTemplate;
using protocol::ResourceContents;

class IResolver {
public:
    virtual ~IResolver() = default;

    /// 该 resolver 是否能处理某个 uri
    virtual bool accepts(const std::string& uri) const = 0;

    /// 读取具体资源；返回一组 Contents（一个文件可拆块）
    virtual std::vector<ResourceContents> read(const std::string& uri) = 0;

    /// 列出所有已知资源（file scan / remote list）
    virtual std::vector<Resource> list() = 0;

    /// 可选：列出模板（如 file://{path}/{name}.log）
    virtual std::vector<ResourceTemplate> listTemplates() { return {}; }
};

} // namespace
