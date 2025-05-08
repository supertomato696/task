#include "prompt/MemoryBackend.hpp"

using namespace mcp::prompt;

bool MemoryBackend::upsert(const StoredPrompt& sp)
{
    std::scoped_lock lk(mtx_);
    map_[sp.meta.name] = sp;
    return true;
}
bool MemoryBackend::erase(const std::string& n)
{
    std::scoped_lock lk(mtx_);
    return map_.erase(n)!=0;
}
std::optional<StoredPrompt>
MemoryBackend::get(const std::string& n) const
{
    std::scoped_lock lk(mtx_);
    auto it = map_.find(n);
    if(it==map_.end()) return std::nullopt;
    return it->second;
}
std::vector<StoredPrompt> MemoryBackend::list() const
{
    std::scoped_lock lk(mtx_);
    std::vector<StoredPrompt> v;
    for(auto& [_,sp]:map_) v.push_back(sp);
    return v;
}