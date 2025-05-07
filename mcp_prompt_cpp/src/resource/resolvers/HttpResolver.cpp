#include "resource/resolvers/HttpResolver.hpp"
#include <httplib.h>
#include <filesystem>
#include <openssl/evp.h>

using namespace mcp::resource;
namespace fs = std::filesystem;
using protocol::ResourceContents;

/* ---------- helpers ---------- */
std::string HttpResolver::b64(const std::string& bin)
{
    std::string out;
    out.resize(4 * ((bin.size() + 2) / 3));
    int len = EVP_EncodeBlock(reinterpret_cast<unsigned char*>(out.data()),
                              reinterpret_cast<const unsigned char*>(bin.data()),
                              static_cast<int>(bin.size()));
    out.resize(len);
    return out;
}

bool HttpResolver::accepts(const std::string& uri) const
{
    return uri.rfind("http://",0)==0 || uri.rfind("https://",0)==0;
}

/* mime fallback 根据扩展名 */
std::string HttpResolver::sniffMime(const std::string& /*url*/,
                                    const std::string& headerMime,
                                    const std::string& fallbackExt)
{
    if(!headerMime.empty()) return headerMime;

    auto ext = fs::path(fallbackExt).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if(ext == ".png" ) return "image/png";
    if(ext == ".jpg" || ext==".jpeg") return "image/jpeg";
    if(ext == ".json") return "application/json";
    if(ext == ".txt" ) return "text/plain";
    if(ext == ".wav" ) return "audio/wav";
    if(ext == ".mp3" ) return "audio/mpeg";
    return "application/octet-stream";
}

/* ---------- read ---------- */
std::vector<ResourceContents> HttpResolver::read(const std::string& uri)
{
    if(!accepts(uri))
        throw std::invalid_argument("HttpResolver uri scheme mismatch");

    /* --- 解析 host/path --- */
    const auto pos = uri.find("://");
    const auto host_start = pos == std::string::npos ? 0 : pos + 3;
    const auto path_start = uri.find('/', host_start);

    std::string host = uri.substr(host_start,
                       path_start == std::string::npos ? std::string::npos
                                                       : path_start - host_start);
    std::string path = path_start == std::string::npos ? "/" : uri.substr(path_start);

    /* --- 发起 GET --- */
    httplib::Client cli(host.c_str());
    cli.set_follow_location(true);
    cli.set_connection_timeout(3);
    cli.set_read_timeout(5);

    auto res = cli.Get(path.c_str());
    if(!res)
        throw std::runtime_error("HTTP request failed ("+
            httplib::to_string(res.error())+")");

    if(res->status >= 400)
        throw std::runtime_error("HTTP "+std::to_string(res->status));

    std::string mime = sniffMime(uri,
                                 res->get_header_value("Content-Type"),
                                 fs::path(uri).filename().string());

    bool isText = mime.starts_with("text/") || mime=="application/json";

    ResourceContents rc;
    rc.uri      = uri;
    rc.mimeType = mime;
    rc.data     = isText ? std::variant<std::string,std::string>{res->body}
                         : std::variant<std::string,std::string>{ b64(res->body) };

    return { std::move(rc) };
}
