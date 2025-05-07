#include "resource/resolvers/LocalFileResolver.hpp"
#include <fstream>
#include <iterator>
#include <openssl/evp.h>

using namespace mcp::resource;
namespace fs = std::filesystem;
using protocol::Resource;
using protocol::ResourceContents;

/* -------- base64 helper -------- */
static std::string b64(const std::string& bin)
{
    std::string out;
    out.resize(4 * ((bin.size() + 2) / 3));
    int len = EVP_EncodeBlock(reinterpret_cast<unsigned char*>(out.data()),
                              reinterpret_cast<const unsigned char*>(bin.data()),
                              static_cast<int>(bin.size()));
    out.resize(len);
    return out;
}

/* ------------------------------------------------------------------ */
LocalFileResolver::LocalFileResolver(fs::path root)
: root_(std::move(root)) {}

/* ------------------------------------------------------------------ */
bool LocalFileResolver::accepts(const std::string& uri) const
{
    return uri.rfind("file://", 0) == 0;
}

/* ------------------------------------------------------------------ */
std::vector<ResourceContents>
LocalFileResolver::read(const std::string& uri)
{
    if(!accepts(uri))
        throw std::invalid_argument("LocalFileResolver uri scheme mismatch");

    fs::path p = uri.substr(7);
    if(!fs::exists(p) || !fs::is_regular_file(p))
        throw std::runtime_error("file not exist");

    std::ifstream f(p, std::ios::binary);
    std::string buf{std::istreambuf_iterator<char>(f),{}};

    ResourceContents rc;
    rc.uri      = uri;
    rc.mimeType = toMime(p);

    if (isText(rc.mimeType))
        rc.data = buf;                 // text
    else
        rc.data = b64(buf);            // blob (b64)

    return { std::move(rc) };
}

/* ------------------------------------------------------------------ */
std::vector<Resource> LocalFileResolver::list()
{
    std::vector<Resource> vec;
    for(auto &entry : fs::recursive_directory_iterator(root_))
    {
        if(!entry.is_regular_file()) continue;

        Resource r;
        r.uri      = "file://" + entry.path().string();
        r.name     = entry.path().filename().string();
        r.mimeType = toMime(entry.path());
        r.size     = entry.file_size();
        vec.push_back(std::move(r));
    }
    return vec;
}

/* ------------------------------------------------------------------ */
std::string LocalFileResolver::toMime(const fs::path& p) const
{
    auto ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if(ext == ".txt" ) return "text/plain";
    if(ext == ".md"  ) return "text/markdown";
    if(ext == ".json") return "application/json";
    if(ext == ".png" ) return "image/png";
    if(ext == ".jpg" || ext == ".jpeg") return "image/jpeg";
    if(ext == ".wav" ) return "audio/wav";
    if(ext == ".mp3" ) return "audio/mpeg";
    return "application/octet-stream";
}

bool LocalFileResolver::isText(const std::string& mime) const
{
    return mime.starts_with("text/") || mime == "application/json";
}
