// =============================================================
//  Modular Linux App Process Manager (C++20) – Consolidated Code
//  textdoc id: 6829d0dfd82c8191a878c14ded7e7350   (v6  – EnvManager overhaul)
// =============================================================
//  NOTE: This single file aggregates the essential headers/impls so
//  you can copy‑split into real *.hpp / *.cpp.  Unchanged components
//  are elided with "..." to keep focus on the new EnvManager.hpp.
//  Compile‑ready versions should restore the full code.
//
// -------------------------------------------------------------
//  Constants.hpp, ProcessTypes.hpp, LinuxAppInfo.hpp, Launcher.hpp,
//  ProcessMonitor.hpp, LinuxAppProcessManager.{hpp,cpp}
//  (UNCHANGED from v5 – omitted here for brevity)
// -------------------------------------------------------------

/*
 *  ================================
 *  EnvManager.hpp   (header‑only)
 *  ================================
 *  Responsibilities:
 *    • Collect parent process environment (via `environ`).
 *    • Merge inline KEY=VAL pairs (comma‑separated) and env‑file lines.
 *    • Guarantee later sources override earlier ones (inline > file > parent).
 *    • Auto‑patch LD_LIBRARY_PATH to include:
 *         ‑  executable directory
 *         ‑  parentDir/lib  and parentDir/lib64
 *         (while preserving any existing entries, deduplicated & POSIX order).
 *    • Return vector<string> ready for conversion to `char* const*`.
 *
 *  Design choices:
 *    1. Use `unordered_map<string,size_t>` index to enable O(1) overwrite.
 *    2. Preserve original insertion order except for overwritten keys.
 *    3. Deduplicate LD_LIBRARY_PATH segments with `unordered_set`.
 */

#pragma once

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

extern char **environ; // POSIX global

// Forward decl of LinuxAppInfo (from other header)
struct LinuxAppInfo;

class EnvManager {
public:
    /// Build final environment vector for execve
    static std::vector<std::string> buildEnvironment(const LinuxAppInfo &app) {
        std::vector<std::string> envStrings;
        envStrings.reserve(128);

        // 1) parent env
        for (char **env = environ; *env; ++env) {
            envStrings.emplace_back(*env);
        }

        // 2) build index for quick overwrite
        std::unordered_map<std::string, std::size_t> idx;
        for (std::size_t i = 0; i < envStrings.size(); ++i) {
            auto eq = envStrings[i].find('=');
            if (eq != std::string::npos)
                idx.emplace(envStrings[i].substr(0, eq), i);
        }

        // 3) inline KV (highest precedence)
        if (app.envInline && !app.envInline->empty()) {
            applyPairs(parseInline(*app.envInline), envStrings, idx);
        }
        // 4) env file
        if (app.envFile && !app.envFile->empty()) {
            applyPairs(parseFile(*app.envFile), envStrings, idx);
        }

        // 5) LD_LIBRARY_PATH patch (lowest overwrite precedence)
        patchLdLibraryPath(envStrings, idx, app.execPath);

        return envStrings; // RVO
    }

private:
    // ---------- helpers --------------------------------------------------

    static std::string trim(std::string_view sv) {
        const char *ws = " \t\n\r";
        std::size_t b = sv.find_first_not_of(ws);
        if (b == std::string::npos) return {};
        std::size_t e = sv.find_last_not_of(ws);
        return std::string{sv.substr(b, e - b + 1)};
    }

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

    static void applyPairs(const std::vector<std::pair<std::string, std::string>> &pairs,
                           std::vector<std::string> &envVec,
                           std::unordered_map<std::string, std::size_t> &idx) {
        for (auto &[k, v] : pairs) {
            std::string kv = k + "=" + v;
            auto it = idx.find(k);
            if (it != idx.end()) {
                envVec[it->second] = kv; // overwrite in‑place preserves order
            } else {
                idx[k] = envVec.size();
                envVec.push_back(std::move(kv));
            }
        }
    }

    static void patchLdLibraryPath(std::vector<std::string> &envVec,
                                   std::unordered_map<std::string, std::size_t> &idx,
                                   const std::string &execPath) {
        // Derive candidate directories
        std::filesystem::path exe(execPath);
        std::string dir = exe.parent_path().string();
        std::string parentDir = exe.parent_path().parent_path().string();
        std::vector<std::string> extras;
        if (!dir.empty()) extras.push_back(dir);
        if (!parentDir.empty()) {
            extras.push_back(parentDir + "/lib");
            extras.push_back(parentDir + "/lib64");
        }

        // Grab existing LD_LIBRARY_PATH if present
        std::string current;
        auto it = idx.find("LD_LIBRARY_PATH");
        if (it != idx.end()) {
            const std::string &full = envVec[it->second];
            current = full.substr(strlen("LD_LIBRARY_PATH="));
        }

        // Build set for deduplication, preserve insertion order
        std::unordered_set<std::string> seen;
        std::vector<std::string> ordered;

        auto push_unique = [&](const std::string &p) {
            if (p.empty()) return;
            if (seen.insert(p).second) ordered.push_back(p);
        };

        // existing first (maintain user order)
        if (!current.empty()) {
            std::stringstream ss(current);
            std::string tok;
            while (std::getline(ss, tok, ':')) push_unique(tok);
        }
        // then extras
        for (auto &e : extras) push_unique(e);

        // Join
        std::string joined;
        for (std::size_t i = 0; i < ordered.size(); ++i) {
            if (i) joined += ':';
            joined += ordered[i];
        }
        std::string newVar = "LD_LIBRARY_PATH=" + joined;

        if (it != idx.end()) {
            envVec[it->second] = std::move(newVar);
        } else {
            idx["LD_LIBRARY_PATH"] = envVec.size();
            envVec.push_back(std::move(newVar));
        }
    }
};
