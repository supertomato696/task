include/prompt/MultiModalAssembler.hpp

#pragma once
#include "protocol/McpContent.hpp"          // PromptMessage / TextContent / …
#include <string>

namespace mcp::prompt {

class MultiModalAssembler {
public:
    /** 把 raw 字符串转成 PromptMessage（含强类型 content） */
    protocol::PromptMessage assemble(const std::string& role,
                                     const std::string& raw) const;

private:
    /* helpers */
    static bool isUri       (const std::string& s);
    static bool isDataImage (const std::string& s);
    static bool isAudioFile (const std::string& path);
    static std::string mimeFromExt (const std::string& ext);

    /* file → base64 */
    static std::string readFileB64(const std::string& path);
};

} // namespace