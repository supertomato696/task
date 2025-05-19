#pragma once
#include <unordered_map>
#include <vector>
#include <iostream>

inline std::unordered_map<std::string,std::string>
snapshotEnv(const std::vector<std::string>& vec)
{
    std::unordered_map<std::string,std::string> m;
    for (auto& s : vec) {
        auto pos = s.find('=');
        if (pos == std::string::npos) continue;
        m.emplace(s.substr(0,pos), s.substr(pos+1));
    }
    return m;
}

inline void printDiff(const std::unordered_map<std::string,std::string>& before,
                      const std::unordered_map<std::string,std::string>& after)
{
    for (auto& [k,v] : after) {
        auto it = before.find(k);
        if (it == before.end())
            std::cout << "+ " << k << '=' << v << '\n';
        else if (it->second != v)
            std::cout << "~ " << k << ": " << it->second << " -> " << v << '\n';
    }
    for (auto& [k,_] : before) {
        if (!after.contains(k))
            std::cout << "- " << k << '\n';
    }
}
