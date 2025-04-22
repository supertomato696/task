
#pragma once
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <vector>
#include <string>

namespace mcp::prompt {

using json = nlohmann::json;

/* ---------- PromptTemplate 与序列化 ---------- */
struct PromptTemplate {
    std::string name;
    std::string description;
    json        messages;       // JSON array of PromptMessage
    std::vector<std::string> arguments;
};

inline void to_json(json& j, const PromptTemplate& p){
    j = { {"name",p.name},
          {"description",p.description},
          {"messages",p.messages},
          {"arguments",p.arguments} };
}
inline void from_json(const json& j, PromptTemplate& p){
    p.name        = j.at("name");
    p.description = j.value("description","");
    p.messages    = j.at("messages");
    p.arguments   = j.value("arguments", std::vector<std::string>{});
}

/* ---------- 基类接口 ---------- */
class IPromptStore {
public:
    virtual ~IPromptStore() = default;
    virtual void add(const PromptTemplate& t)=0;
    virtual const PromptTemplate* find(const std::string& n) const =0;
    virtual std::vector<PromptTemplate> all() const =0;
};

/* ---------- 简单内存实现 ---------- */
class MemoryPromptStore : public IPromptStore {
public:
    void add(const PromptTemplate& t) override { map_[t.name]=t; }

    const PromptTemplate* find(const std::string& n) const override {
        auto it=map_.find(n); return it==map_.end()? nullptr : &it->second;
    }

    std::vector<PromptTemplate> all() const override {
        std::vector<PromptTemplate> v;
        for(auto& [_,p] : map_) v.push_back(p);
        return v;
    }
private:
    std::unordered_map<std::string,PromptTemplate> map_;
};

} // namespace mcp::prompt