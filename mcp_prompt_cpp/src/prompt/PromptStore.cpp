#include "prompt/PromptStore.hpp"
#include <openssl/md5.h>             // 用于生成实例 key 的哈希
#include <sstream>
#include <iomanip>

using namespace mcp::prompt;
namespace proto = mcp::protocol;

/* ===================================================================== */
/*                               模板 CRUD                               */
/* ===================================================================== */
bool PromptStore::addTemplate(PromptTemplate t)
{
    std::scoped_lock lk(mtx_);
    bool isNew = templates_.find(t.meta.name) == templates_.end();
    templates_[t.meta.name] = std::move(t);
    filledCache_.clear();               // 模板变了，失效旧缓存
    return isNew;
}

bool PromptStore::removeTemplate(const std::string& name)
{
    std::scoped_lock lk(mtx_);
    bool erased = templates_.erase(name) != 0;
    if (erased) filledCache_.clear();
    return erased;
}

std::optional<PromptTemplate>
PromptStore::getTemplate(const std::string& name) const
{
    std::scoped_lock lk(mtx_);
    auto it = templates_.find(name);
    if (it == templates_.end()) return std::nullopt;
    return it->second;
}

std::vector<proto::Prompt> PromptStore::listTemplates() const
{
    std::scoped_lock lk(mtx_);
    std::vector<proto::Prompt> v;
    v.reserve(templates_.size());
    for (auto& [_, t] : templates_) v.push_back(t.meta);
    return v;
}

/* ===================================================================== */
/*                              渲染 & 缓存                              */
/* ===================================================================== */
FilledPrompt
PromptStore::renderPrompt(const std::string& name,
                          const std::unordered_map<std::string,std::string>& args)
{
    std::string key = makeKey(name, args);

    {   /* -- 1) 若缓存命中直接返回 -- */
        std::scoped_lock lk(mtx_);
        auto fnd = filledCache_.find(key);
        if (fnd != filledCache_.end()) return fnd->second;
    }

    /* -- 2) 取模板并渲染 -- */
    auto optTpl = getTemplate(name);
    if (!optTpl) throw std::runtime_error("prompt template not found: " + name);

    const PromptTemplate& tpl = *optTpl;

    std::vector<proto::PromptMessage> rendered;
    rendered.reserve(tpl.messages.size());

    MultiModalAssembler mma;

    for (const auto& tmplMsg : tpl.messages)
    {
        proto::PromptMessage msg = tmplMsg;

        /* 仅当 content 是 TextContent 时尝试替换 */
        if (std::holds_alternative<proto::TextContent>(tmplMsg.content))
        {
            auto tc = std::get<proto::TextContent>(tmplMsg.content);
            tc.text = TemplateEngine::render(tc.text, args);
            msg.content = std::move(tc);
        }

        /* 富媒体占位符（如 file://something.wav）在导入时就应转换，
           若仍是 URI，可在此处再次 assemble */
        if (std::holds_alternative<proto::TextContent>(msg.content))
        {
            auto tc = std::get<proto::TextContent>(msg.content);
            msg = mma.assemble(msg.role, tc.text);
        }

        rendered.push_back(std::move(msg));
    }

    /* -- 3) 构造 FilledPrompt & 缓存 -- */
    FilledPrompt fp;
    fp.key  = key;
    fp.data.description = tpl.meta.description;
    fp.data.messages    = std::move(rendered);

    {
        std::scoped_lock lk(mtx_);
        filledCache_.emplace(key, fp);
    }
    return fp;
}

std::optional<FilledPrompt>
PromptStore::getFilled(const std::string& key) const
{
    std::scoped_lock lk(mtx_);
    auto it = filledCache_.find(key);
    if (it == filledCache_.end()) return std::nullopt;
    return it->second;
}

void PromptStore::clearFilledCache()
{
    std::scoped_lock lk(mtx_);
    filledCache_.clear();
}

/* ===================================================================== */
/*                          辅助：生成缓存 key                            */
/* ===================================================================== */
std::string PromptStore::makeKey(const std::string& name,
                                 const std::unordered_map<std::string,std::string>& args)
{
    /* 将 args 排序后拼接，保证顺序不影响哈希 */
    std::vector<std::pair<std::string,std::string>> kv(args.begin(), args.end());
    std::sort(kv.begin(), kv.end(),
              [](auto& a, auto& b){ return a.first < b.first; });

    std::string concat = name;
    for (auto& [k,v] : kv) { concat += "|" + k + "=" + v; }

    unsigned char md5[16];
    MD5(reinterpret_cast<const unsigned char*>(concat.data()),
        concat.size(), md5);

    std::ostringstream oss;
    for (unsigned char c : md5) oss << std::hex << std::setw(2)
                                    << std::setfill('0') << int(c);
    return name + "|" + oss.str();
}


//3 使用示例
//
//PromptStore store;
//
///* 1) 导入模板一次即可 */
//PromptTemplate daily;
//daily.meta.name = "daily_summary";
//daily.meta.description = "生成每日总结";
//daily.meta.arguments = { {"overview","概览文本",true} };
//
//proto::PromptMessage m1,m2,m3;
//m1.role="system";    m1.content = proto::TextContent{"你是一名车载助理。"};
//m2.role="user";      m2.content = proto::TextContent{"今天概览：{{overview}}"};
//m3.role="assistant"; m3.content = proto::TextContent{"好的，总结如下："};
//daily.messages = {m1,m2,m3};
//store.addTemplate(std::move(daily));
//
///* 2) 运行时调用 renderPrompt */
//auto filled = store.renderPrompt("daily_summary", {
//    {"overview","行驶 120km，油耗 8.1L"}
//});
//
///* filled.data 即 GetPromptResult，可直接序列化返回 */
//nlohmann::json resp = filled.data;