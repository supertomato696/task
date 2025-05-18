#include <string>
#include <vector>
    struct LinuxAppInfo {
        std::string pkgName;
        std::string exec;    // bin
        std::string name;
        int32_t  nice;
        std::string iconPath;
        std::vector<char> appIcon;
        int32_t  pType;
        std::string appType; //for calculate oom_adj
        std::string description;
        std::string version;
        std::string categories;   // environment file path
        std::string mimeType;
        bool startUpNotify;
        std::string actions;
        bool terminal;
        std::string genericName;
        bool autostart;
        std::string entrance;   // parameters
    };