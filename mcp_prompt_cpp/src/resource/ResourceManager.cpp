#include "resource/ResourceManager.hpp"
#include <stdexcept>

using namespace mcp::resource;
namespace fs = std::filesystem;

/* ------------------------------------------------------------------ */
void ResourceManager::registerResolver(std::unique_ptr<IResolver> r)
{
    resolvers_.push_back(std::move(r));
}

/* ------------------------------------------------------------------ */
std::vector<protocol::Resource> ResourceManager::listResources()
{
    std::vector<protocol::Resource> out;
    for (auto &r : resolvers_) {
        auto part = r->list();
        out.insert(out.end(),
                   std::make_move_iterator(part.begin()),
                   std::make_move_iterator(part.end()));
    }
    return out;
}

std::vector<protocol::ResourceTemplate> ResourceManager::listTemplates()
{
    std::vector<protocol::ResourceTemplate> out;
    for (auto &r : resolvers_) {
        auto part = r->listTemplates();
        out.insert(out.end(),
                   std::make_move_iterator(part.begin()),
                   std::make_move_iterator(part.end()));
    }
    return out;
}

std::vector<protocol::ResourceContents>
ResourceManager::readResource(const std::string& uri)
{
    if (auto *res = findResolver(uri))
        return res->read(uri);
    throw std::runtime_error("no resolver for "+uri);
}

/* ------------------------------------------------------------------ */
void ResourceManager::subscribe(const std::string& uri, UpdatedCb cb)
{
    std::scoped_lock lk(subMtx_);
    subs_[uri] = std::move(cb);

    /* init mtime cache for local file */
    if (uri.rfind("file://",0)==0) {
        fs::path p(uri.substr(7));
        if (fs::exists(p))
            mtime_[uri] = fs::last_write_time(p);
    }
}

void ResourceManager::unsubscribe(const std::string& uri)
{
    std::scoped_lock lk(subMtx_);
    subs_.erase(uri);
    mtime_.erase(uri);
}

/* ------------------------------------------------------------------ */
void ResourceManager::poll()
{
    std::scoped_lock lk(subMtx_);
    for (auto &[uri, cb] : subs_)
    {
        if (uri.rfind("file://",0)!=0) continue;        // 仅轮询本地文件
        fs::path p(uri.substr(7));
        if (!fs::exists(p)) continue;

        auto ts = fs::last_write_time(p);
        auto &prev = mtime_[uri];
        if (prev.time_since_epoch().count() == 0) {     // 首次
            prev = ts;
            continue;
        }
        if (ts != prev) {
            prev = ts;
            if (cb) cb(uri);
        }
    }
}

/* ------------------------------------------------------------------ */
IResolver* ResourceManager::findResolver(const std::string& uri) const
{
    for (auto &r : resolvers_)
        if (r->accepts(uri)) return r.get();
    return nullptr;
}
