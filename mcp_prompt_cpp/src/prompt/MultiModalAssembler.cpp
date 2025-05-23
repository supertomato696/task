// src/prompt/MultiModalAssembler.cpp
#include "prompt/MultiModalAssembler.hpp"
#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <nlohmann/json.hpp>



#include <openssl/evp.h>

using namespace mcp::prompt;
namespace fs = std::filesystem;
namespace proto = mcp::protocol;

/* ===== helpers ===== */
bool MultiModalAssembler::isUri(const std::string& s)
{ return s.find("://") != std::string::npos; }

bool MultiModalAssembler::isDataImage(const std::string& s)
{
    return std::regex_match(s, std::regex(R"(data:image\/[a-zA-Z0-9.+-]+;base64,.*)"));
}

bool MultiModalAssembler::isAudioFile(const std::string& p)
{
    std::string ext = fs::path(p).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext==".wav"||ext==".mp3"||ext==".flac"||ext==".m4a"||ext==".ogg";
}

std::string MultiModalAssembler::mimeFromExt(const std::string& extRaw)
{
    std::string ext = extRaw;
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext=="png")      return "image/png";
    if (ext=="jpg"||ext=="jpeg") return "image/jpeg";
    if (ext=="wav")      return "audio/wav";
    if (ext=="mp3")      return "audio/mpeg";
    if (ext=="flac")     return "audio/flac";
    if (ext=="m4a")      return "audio/mp4";
    if (ext=="ogg")      return "audio/ogg";
    if (ext=="json")     return "application/json";
    return "application/octet-stream";
}

std::string MultiModalAssembler::readFileB64(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    std::string bin{std::istreambuf_iterator<char>(f), {}};

    const size_t encLen = 4*((bin.size()+2)/3);
    std::string out(encLen, '\0');
    EVP_EncodeBlock(reinterpret_cast<unsigned char*>(&out[0]),
                    reinterpret_cast<const unsigned char*>(bin.data()),
                    bin.size());
    return out;
}

/* ===== assemble main ===== */
proto::PromptMessage MultiModalAssembler::assemble(const std::string& role,
                                                   const std::string& raw) const
{
    proto::PromptMessage msg;
    msg.role = (role == "user") ? proto::Role::User : proto::Role::Assistant;

    /* 1. 本地文件 URI -> audio/image/resource */
    if (isUri(raw) && raw.rfind("file://",0)==0)
    {
        std::string path = raw.substr(7);
        std::string ext  = fs::path(path).extension().string().substr(1);

        if (isAudioFile(path))                   // ---- AudioContent
        {
            proto::AudioContent ac;
            ac.mimeType = mimeFromExt(ext);
            ac.data     = readFileB64(path);
            msg.content = std::move(ac);
            return msg;
        }
        else if (ext==".png"||ext==".jpg"||ext==".jpeg")  // ---- ImageContent
        {
            proto::ImageContent ic;
            ic.mimeType = mimeFromExt(ext);
            ic.data     = readFileB64(path);
            msg.content = std::move(ic);
            return msg;
        }
        else                                      // ---- EmbeddedResource (blob or text)
        {
            proto::EmbeddedResource er;
            auto mime = mimeFromExt(ext);

            if (mime.starts_with("text/") || mime=="application/json") {
                proto::TextResourceContents trc;
                trc.uri      = raw;
                trc.mimeType = mime;
                std::ifstream f(path);
                trc.text.assign(std::istreambuf_iterator<char>(f), {});
                er.resource = trc;
            } else {
                proto::BlobResourceContents brc;
                brc.uri      = raw;
                brc.mimeType = mime;
                brc.blob     = readFileB64(path);
                er.resource  = brc;
            }
            msg.content = std::move(er);
            return msg;
        }
    }

    /* 2. data:image/... */
    if (isDataImage(raw))
    {
        size_t mimeEnd = raw.find(';');
        proto::ImageContent ic;
        ic.mimeType = raw.substr(5, mimeEnd - 5);          // image/png
        ic.data     = raw.substr(raw.find(',') + 1);
        msg.content = std::move(ic);
        return msg;
    }

    /* 3. 纯文本 */
    proto::TextContent tc;
    tc.text = raw;
    msg.content = std::move(tc);
    return msg;
}
