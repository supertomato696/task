#pragma once
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <LinuxAppInfor.hpp>

extern char **environ;   // POSIX global

struct LinuxAppInfo;     // 前向声明，实际定义见 LinuxAppInfor.hpp

class EnvManager {
public:
    /** 返回最终 env 列表，格式 KEY=VAL */
    static std::vector<std::string> buildEnvironment(const LinuxAppInfo& app)
    {
        // 1) 父环境
        std::vector<std::string> envVecString;
        for (char **p = environ; *p; ++p) envVecString.emplace_back(*p);

        // 2) file -> 3) inline   （inline 优先级最高）
        if (app.envFile && !app.envFile->empty())
            merge(envVecString, parseFile(*app.envFile));

        if (app.envInline && !app.envInline->empty())
            merge(envVecString, parseInline(*app.envInline));

        // 4) 补丁 LD_LIBRARY_PATH
        patchLdLibraryPath(envVecString, app.execPath);
        return envVecString;
    }

private:
    // ---------- 解析 ----------
    // static std::vector<std::pair<std::string,std::string>>
    // parseInline(const std::string& s)
    // {
    //     std::vector<std::pair<std::string,std::string>> v;
    //     std::stringstream ss(s);
    //     std::string item;
    //     while (std::getline(ss,item,',')) {
    //         auto eq = item.find('=');
    //         if (eq == std::string::npos) continue;
    //         v.emplace_back(trim(item.substr(0,eq)), trim(item.substr(eq+1)));
    //     }
    //     return v;
    // }

        static std::vector<std::pair<std::string, std::string>>
    parseInline(const std::string &csv) {
        std::vector<std::pair<std::string, std::string>> out;
        std::stringstream ss(csv);
        std::string kv;
        while (std::getline(ss, kv, ',')) {
            auto eq = kv.find('=');
            if (eq == std::string::npos) continue;
            std::string k = trim(kv.substr(0, eq));
            std::string v = trim(kv.substr(eq + 1));
            if (!k.empty()) out.emplace_back(std::move(k), std::move(v));
        }
        return out;
    }

    static std::vector<std::pair<std::string, std::string>>
    parseFile(const std::string &path) {
        std::vector<std::pair<std::string, std::string>> out;
        std::ifstream ifs(path);
        if (!ifs) return out;
        std::string line;
        while (std::getline(ifs, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            auto eq = line.find('=');
            if (eq == std::string::npos) continue;
            std::string k = trim(line.substr(0, eq));
            std::string v = trim(line.substr(eq + 1));
            if (!k.empty()) out.emplace_back(std::move(k), std::move(v));
        }
        return out;
    }

    // ---------- 合并 ----------
    static void merge(std::vector<std::string>& env,
                      const std::vector<std::pair<std::string,std::string>>& pairs)
    {
        // 先索引到 env 中现有键
        std::unordered_map<std::string,size_t> idx;
        for (size_t i=0;i<env.size();++i) {
            auto pos = env[i].find('=');
            if (pos!=std::string::npos) idx.emplace(env[i].substr(0,pos), i);
        }

        for (auto& [k,v] : pairs) {
            if (k=="LD_LIBRARY_PATH") {
                // ----------- 特殊合并 -----------
                std::string old;
                if (auto it=idx.find(k); it!=idx.end())
                    old = env[it->second].substr(16);      // strip "LD_LIBRARY_PATH="
                std::string merged = joinPath(old, v);     // 去重 + 拼接
                std::string kv = "LD_LIBRARY_PATH=" + merged;
                if (auto it=idx.find(k); it!=idx.end()) env[it->second] = std::move(kv);
                else { idx[k]=env.size(); env.push_back(std::move(kv)); }
            } else {
                // ----------- 常规覆盖 -----------
                std::string kv = k + "=" + v;
                if (auto it=idx.find(k); it!=idx.end()) env[it->second] = std::move(kv);
                else                                   env.push_back(std::move(kv));
            }
        }
    }

    // ---------- 补丁 LD_LIBRARY_PATH ----------
    static void patchLdLibraryPath(std::vector<std::string>& env,
                                   const std::string& execPath)
    {
        std::string dir     = std::filesystem::path(execPath).parent_path().parent_path().string();
        std::string parentDir  = std::filesystem::path(dir).parent_path();
        std::vector<std::string> extras;
        if (!dir.empty()) extras.push_back(dir);
        if (!parentDir.empty()) {
            extras.push_back(parentDir + "/lib");
            extras.push_back(parentDir + "/lib64");
        } 

        // 找现有变量
        auto it = std::find_if(env.begin(), env.end(), [](const std::string& s){
            return s.rfind("LD_LIBRARY_PATH=",0)==0;
        });

        std::string value;
        if (it!=env.end()) value = it->substr(16);     // strip
        value = joinPath(value, joinVector(extras, ":"));

        std::string newVar = "LD_LIBRARY_PATH=" + value;
        if (it!=env.end()) *it = std::move(newVar);
        else               env.push_back(std::move(newVar));
    }

    // ---------- util ----------
    static std::string joinPath(const std::string& a, const std::string& b)
    {
        std::unordered_set<std::string> set;
        auto collect=[&](const std::string& s){
            std::stringstream ss(s);
            std::string tok;
            while (std::getline(ss,tok,':')) if(!tok.empty()) set.insert(tok);
        };
        collect(a); collect(b);
        std::string joined;
        for (auto it=set.begin(); it!=set.end(); ++it) {
            if (it!=set.begin()) joined.push_back(':');
            joined += *it;
        }
        return joined;

        // std::vector<std::string> vec;
        // auto collect = [&](const std::string& s) {
        //     std::stringstream ss(s);
        //     std::string tok;
        //     while (std::getline(ss, tok, ':')) {
        //         if (!tok.empty() && std::find(vec.begin(), vec.end(), tok) == vec.end()) {
        //             vec.push_back(tok);
        //         }
        //     }
        // };

    }

    static std::string joinVector(const std::vector<std::string>& v, const char* sep)
    {
        std::string out;
        for (auto& s : v) {
            if (s.empty()) continue;
            if (!out.empty()) out += sep;
            out += s;
        }
        return out;
    }

    static std::string trim(const std::string& s)
    {
        const char* ws=" \t\r\n";
        size_t b = s.find_first_not_of(ws);
        size_t e = s.find_last_not_of(ws);
        return (b==std::string::npos) ? "" : s.substr(b, e-b+1);
    }
};



