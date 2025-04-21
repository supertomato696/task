#include "resource/ResourceManager.hpp"
#include <filesystem>

using namespace mcp::resource;
namespace fs = std::filesystem;

void ResourceManager::registerResolver(std::unique_ptr<IResolver> r){
    resolvers_.push_back(std::move(r));
}

std::vector<nlohmann::json> ResourceManager::listResources(){
    std::vector<nlohmann::json> out;
    for(auto& r:resolvers_){
        auto part = r->list();
        out.insert(out.end(), part.begin(), part.end());
    }
    return out;
}

nlohmann::json ResourceManager::readResource(const std::string& uri){
    for(auto& r:resolvers_) if(r->accepts(uri)) return r->read(uri);
    throw std::runtime_error("no resolver for "+uri);
}

/* ---------------- subscribe / polling ---------------- */
void ResourceManager::subscribe(const std::string& uri, UpdatedCb cb){
    std::scoped_lock lk(sub_mtx_);
    // if(uri.rfind("file://",0)==0){
    //     fs::path p(uri.substr(7));
    //     if(fs::exists(p)) mtime_[uri]=fs::last_write_time(p);
    // }
    
    subs_[uri] = std::move(cb);
    if(std::filesystem::exists(uri.substr(7)))
        mtime_[uri] = fs::last_write_time(uri.substr(7));
}

void ResourceManager::unsubscribe(const std::string& uri){
    std::scoped_lock lk(sub_mtx_);
    subs_.erase(uri);
    mtime_.erase(uri);
}

void ResourceManager::poll(){
    std::scoped_lock lk(sub_mtx_);
    for(auto& [uri,cb] : subs_){
        fs::path p(uri.substr(7));
        if(!fs::exists(p)) continue;
        auto ts = fs::last_write_time(p);
        if(ts != mtime_[uri]){
            mtime_[uri] = ts;
            if(cb) cb(uri);
        }
    }
}