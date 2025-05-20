#pragma once
#include <string>
#include <vector>
#include <optional>
  struct LinuxAppInfo {
    std::string                           instanceId;      // unique key
    std::string                execPath;        // absolute/relative path
    std::vector<std::string>              entrance;        // argv[1..]
    std::optional<int>                    priority;        // nice level
    std::optional<std::string>            envInline;       // "K=V,K2=V2"
    std::optional<std::string>  envFile;         // path to env file
    bool                                  autostart{false};
};

