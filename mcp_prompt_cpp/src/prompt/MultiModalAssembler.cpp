// src/prompt/MultiModalAssembler.cpp
#include "prompt/MultiModalAssembler.hpp"
#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <nlohmann/json.hpp>
#include <inja/inja.hpp>
#include <openssl/evp.h>      // base64 (任何实现均可)

using namespace mcp;
using nlohmann::json;
namespace pr = mcp::protocol;
namespace fs = std::filesystem;

///* ---------- 简易文件读取 + base64 ---------- */
//static std::string readFile(const std::string& path){
//    std::ifstream f(path, std::ios::binary);
//    return { std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>() };
//}

std::string MultiModalAssembler::readFileB64(const std::string& path){
    std::ifstream f(path, std::ios::binary);
    std::string bin{std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
    return b64(bin);
}

static std::string b64(const std::string& bin){
    std::string out(EVP_EncodeBlock(nullptr, nullptr, 0), '\0'); // size hint
    unsigned len = 4*((bin.size()+2)/3);
    out.resize(len);
    EVP_EncodeBlock(reinterpret_cast<unsigned char*>(&out[0]),
                    reinterpret_cast<const unsigned char*>(bin.data()),
                    bin.size());
    return out;
}

/* ---------- 工具 --------- */
bool MultiModalAssembler::isUri(const std::string& v){ return v.rfind("://",0) != std::string::npos; }
bool MultiModalAssembler::isImageDataUri(const std::string& v){
    return std::regex_match(v, std::regex(R"(data:image/[^;]+;base64,.*)"));
}
bool MultiModalAssembler::isAudioExt(const std::string& p){
    std::string ext = fs::path(p).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext==".wav"||ext==".mp3"||ext==".flac"||ext==".m4a"||ext==".ogg";
}
std::string MultiModalAssembler::toMime(const std::string& ext){
    std::string e = ext; std::transform(e.begin(),e.end(),e.begin(),::tolower);
    if(e=="png") return "image/png";
    if(e=="jpg"||e=="jpeg") return "image/jpeg";
    if(e=="wav") return "audio/wav";
    if(e=="mp3") return "audio/mpeg";
    if(e=="flac")return "audio/flac";
    if(e=="m4a") return "audio/mp4";
    if(e=="ogg") return "audio/ogg";
    return "application/octet-stream";
}

/* ---------- assemble 主逻辑 --------- */
PromptMessage MultiModalAssembler::assemble(const std::string& role,const std::string& v)
{
    PromptMessage msg; msg.role = role;

    /* --- 1. 音频 file:// 直接转 AudioContent --- */
    if(isUri(v) && v.rfind("file://",0)==0 && isAudioExt(v)){
        std::string path = v.substr(7);                 // 去掉 file://
        std::string ext  = fs::path(path).extension().string().substr(1); // 不带点
        AudioContent ac;
        ac.mimeType = toMime(ext);
        ac.data     = readFileB64(path);
        msg.tag     = ContentTag::Audio;
        msg.content = ac;
        return msg;
    }

    /* --- 2. data:image/… --- */
    if(isImageDataUri(v)){
        size_t semi = v.find(';');
        ImageContent ic;
        ic.mimeType = v.substr(5, semi-5);              // image/xxx
        ic.data     = v.substr(v.find(',')+1);
        msg.tag     = ContentTag::Image;
        msg.content = ic;
        return msg;
    }

    /* --- 3. 其他 URI → EmbeddedResource (含图片文件等) --- */
    if(isUri(v) && v.rfind("file://",0)==0){
        std::string path = v.substr(7);
        std::string ext  = fs::path(path).extension().string().substr(1);
        EmbeddedResource er;
        er.resource = {
            {"uri", v},
            {"mimeType", toMime(ext)},
            {"blob", readFileB64(path)}
        };
        msg.tag     = ContentTag::Resource;
        msg.content = er;
        return msg;
    }

    /* --- 4. 默认文本 --- */
    TextContent tc{ v, {} };
    msg.tag     = ContentTag::Text;
    msg.content = tc;
    return msg;
}