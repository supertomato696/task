#include <gtest/gtest.h>
#include "EnvManager.hpp"
#include "LinuxAppInfo.hpp"

TEST(EnvManager, InlineOverridesFileOverridesParent) {
    // Parent env
    putenv(const_cast<char*>("FOO=parent"));
    putenv(const_cast<char*>("BAR=parent"));

    // Temp file
    std::string path = "/tmp/test_env_file.env";
    std::ofstream(path) << "FOO=file\nBAR=file\n";
    LinuxAppInfo app;
    app.execPath = "/bin/true";
    app.envFile  = path;
    app.envInline = "FOO=inline";

    auto vec = EnvManager::buildEnvironment(app);
    std::unordered_map<std::string,std::string> m;
    for (auto& s: vec) {
        auto p = s.find('=');
        if (p!=std::string::npos) m[s.substr(0,p)] = s.substr(p+1);
    }
    EXPECT_EQ(m["FOO"], "inline");      // inline wins
    EXPECT_EQ(m["BAR"], "file");        // file overrides parent

    std::filesystem::remove(path);
}
