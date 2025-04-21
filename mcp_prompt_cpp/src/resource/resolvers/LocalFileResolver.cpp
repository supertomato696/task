#include "resource/resolvers/LocalFileResolver.hpp"
#include <fstream>
#include <iterator>
#include <openssl/evp.h>

using namespace mcp::resource;
namespace fs = std::filesystem;
using json = nlohmann::json;

static std::string base64(const std::string& bin){
    // std::string out; out.resize(4*((bin.size()+2)/3));
    size_t len = 4*((bin.size()+2)/3);
    std::string out(len,'\0');
    EVP_EncodeBlock(reinterpret_cast<unsigned char*>(&out[0]),
                    reinterpret_cast<const unsigned char*>(bin.data()), bin.size());
    return out;
}

LocalFileResolver::LocalFileResolver(std::string root):root_(std::move(root)){}

bool LocalFileResolver::accepts(const std::string& uri) const{
    return uri.rfind("file://",0)==0;
}

json LocalFileResolver::read(const std::string& uri){
    std::string path = uri.substr(7);
    fs::path p(path);
    // 大小写敏感，若文件是 .TXT 会走 blob 路径；建议 tolower。
    bool isText = p.extension()==".txt" || p.extension()==".json" || p.extension()==".md";

    std::ifstream f(path, std::ios::binary);
    std::string buf{std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};

    if(isText){
        return json::array({{
            {"uri",uri},{"mimeType",toMime(p)},{"text",buf}
        }});
    }
    return json::array({{
        {"uri",uri},{"mimeType",toMime(p)},{"blob",base64(buf)}
    }});
}

std::vector<json> LocalFileResolver::list(){
    std::vector<json> v;
    for(auto& p: fs::recursive_directory_iterator(root_)){
        if(p.is_regular_file()){
            std::string uri = "file://"+p.path().string();
            v.push_back({
                {"name",p.path().filename().string()},
                {"uri",uri},
                {"mimeType", toMime(p.path())},
                {"size", p.file_size()}
            });
        }
    }
    return v;
}

std::string LocalFileResolver::toMime(const fs::path& p){
    auto ext = p.extension().string();
    std::transform(ext.begin(),ext.end(),ext.begin(),::tolower);
    if(ext==".txt") return "text/plain";
    if(ext==".json")return "application/json";
    if(ext==".png") return "image/png";
    if(ext==".wav") return "audio/wav";
    return "application/octet-stream";
}