// include/prompt/MultiModalAssembler.hpp
#pragma once
#include "protocol/McpContent.hpp"
#include <vector>
#include <string>


namespace mcp {

class MultiModalAssembler {
public:
//    /** Convert "{{uri}}" or "{{text}}" placeholder *value* into proper Content object */
//    protocol::PromptMessage assemble(const std::string& role,
//                                     const std::string& raw);

    /** 给定占位符原始字符串 (来自 prompt 模板渲染后) 解析成 PromptMessage */
    protocol::PromptMessage assemble(const std::string& role,
                                     const std::string& rawVal);

  private:
    // heuristic: detect scheme or file suffix
    static bool isUri(const std::string& v);
    static bool isAudioExt(const std::string& p);        // ★ 新增
    static bool isImageDataUri(const std::string& v);
    static std::string toMime(const std::string& ext);
    static std::string b64(const std::string& bin);

    static std::string readFileB64(const std::string& path);    // 统一 base64

  private:
    protocol::PromptMessage makeText (const std::string&);
    protocol::PromptMessage makeImage(const std::string& uri);
    protocol::PromptMessage makeEmbeddedResource(const std::string& uri);
};

}