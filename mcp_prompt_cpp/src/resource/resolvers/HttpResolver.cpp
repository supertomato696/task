// *(HttpResolver 仅演示 accepts+curl download，省略正文)*

#include "resource/resolvers/HttpResolver.hpp"
#include <httplib.h>
#include <filesystem>
#include <openssl/evp.h>

using namespace mcp::resource;
namespace fs = std::filesystem;
using json = nlohmann::json;

/* ---------- helpers ---------- */
static std::string b64(const std::string& bin){
    // std::string out; out.resize(4*((bin.size()+2)/3));
    size_t len = 4*((bin.size()+2)/3);
    std::string out(len, '\0');
    EVP_EncodeBlock(reinterpret_cast<unsigned char*>(&out[0]),
                    reinterpret_cast<const unsigned char*>(bin.data()), bin.size());
    return out;
}

bool HttpResolver::accepts(const std::string& uri) const{
    return uri.rfind("http://",0)==0 || uri.rfind("https://",0)==0;
}

std::string HttpResolver::sniffMime(const std::string& url,
                                    const std::string& headerMime,
                                    const std::string& fallbackExt)
{
    if(!headerMime.empty()) return headerMime;
    std::string ext = fs::path(fallbackExt).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if(ext==".png")  return "image/png";
    if(ext==".jpg"||ext==".jpeg") return "image/jpeg";
    if(ext==".wav")  return "audio/wav";
    if(ext==".mp3")  return "audio/mpeg";
    if(ext==".json") return "application/json";
    if(ext==".txt")  return "text/plain";
    return "application/octet-stream";
}

/* ---------- read ---------- */
json HttpResolver::read(const std::string& uri){
    // httplib::Client cli(httplib::detail::host_port_pair(uri).first.c_str(),
    //                     httplib::detail::host_port_pair(uri).second);
    httplib::Client cli(uri.c_str());
    cli.set_follow_location(true);

    auto res = cli.Get(uri.c_str());
    if(!res) throw std::runtime_error("HTTP error: "+std::string(cli.get_last_error_message()));

    std::string mime = sniffMime(uri, res->get_header_value("Content-Type"),
                                 fs::path(uri).filename().string());

    // 判断文本 vs 二进制
    bool isText = mime.starts_with("text/") || mime=="application/json";

    if(isText){
        return json::array({{
            {"uri",uri},{"mimeType",mime},{"text", res->body }
        }});
    }
    return json::array({{
        {"uri",uri},{"mimeType",mime},{"blob", b64(res->body)}
    }});
}