// *(HttpResolver 仅演示 accepts+curl download，省略正文)*


#include "resource/resolvers/HttpResolver.hpp"
#include <httplib.h>
#include <filesystem>
#include <openssl/evp.h>

using namespace mcp::resource;
namespace fs   = std::filesystem;
using json     = nlohmann::json;

/* ---------- helpers ---------- */
static std::string base64(const std::string& bin){
    std::string out(4 * ((bin.size() + 2) / 3), '\0');
    EVP_EncodeBlock(reinterpret_cast<unsigned char*>(out.data()),
                    reinterpret_cast<const unsigned char*>(bin.data()),
                    static_cast<int>(bin.size()));
    return out;
}

/* ---------- IResolver impl ---------- */
bool HttpResolver::accepts(const std::string& uri) const {
    return uri.rfind("http://",0)==0 || uri.rfind("https://",0)==0;
}

/* ---------- MIME sniff ---------- */
std::string HttpResolver::sniffMime(const std::string& /*url*/,
                                    const std::string& headerMime,
                                    const std::string& fname)
{
    if(!headerMime.empty()){
        auto semi = headerMime.find(';');
        return semi == std::string::npos ? headerMime
                                         : headerMime.substr(0, semi);
    }
    std::string ext = fs::path(fname).extension().string();
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
    // // 解析 URI 结构
    // size_t scheme_end = uri.find("://");
    // size_t host_start = (scheme_end != std::string::npos) ? scheme_end + 3 : 0;
    // size_t path_start = uri.find('/', host_start);
    //
    // std::string host = uri.substr(host_start, path_start - host_start);
    // std::string path = (path_start != std::string::npos)
    //                  ? uri.substr(path_start)
    //                  : "/";
    //
    // // 初始化客户端
    // httplib::Client cli(host.c_str());
    // cli.set_follow_location(true);
    // cli.set_connection_timeout(3);  // 3秒连接超时
    // cli.set_read_timeout(10);       // 10秒读取超时

    /*  cpp‑httplib 头‑only；直接构造 URL client
        (内部区分 http / https，需定义 CPPHTTPLIB_OPENSSL_SUPPORT) */
    httplib::Client cli(uri.c_str());
    cli.set_follow_location(true);

    auto res = cli.Get(uri.c_str());
    if(!res){
        // auto err = cli.get_last_error();   // enum httplib::Error
        auto err = res.error();
        throw std::runtime_error( "HTTP request failed: " + httplib::to_string(err) +
            " [URI: " + uri + "]");
    }

    std::string mime = sniffMime(uri,
                                 res->get_header_value("Content-Type"),
                                 fs::path(uri).filename().string());

    bool isText = mime.starts_with("text/") || mime == "application/json";

    if(isText){
        return json::array({{
            {"uri", uri},
            {"mimeType", mime},
            {"text", res->body}
        }});
    }
    return json::array({{
        {"uri", uri},
        {"mimeType", mime},
        {"blob", base64(res->body)}
    }});
}