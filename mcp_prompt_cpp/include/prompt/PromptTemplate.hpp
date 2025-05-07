#pragma once
# include "protocol/McpContent.hpp"

#include <vector>

namespace mcp::prompt {

    /** 一条“模板化”的 Prompt（服务器侧存储用） */
    struct PromptTemplate {
        protocol::Prompt                         meta;      ///< name / description / arguments[]
        std::vector<protocol::PromptMessage>     messages;  ///< 可能含 {{placeholder}}
    };

    /* --- JSON(*) 序列化，用于持久化到文件 / SQLite --- */
    inline void to_json(nlohmann::json& j, const PromptTemplate& p)
    {
        j = nlohmann::json{
            {"prompt",  p.meta},
            {"messages",p.messages}
        };
    }
    inline void from_json(const nlohmann::json& j, PromptTemplate& p)
    {
        j.at("prompt").get_to(p.meta);
        j.at("messages").get_to(p.messages);
    }

} // namespace