#include <asio/io_context.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cstdlib>

#include "EnvManager.hpp"
#include "LinuxAppInfor.hpp"      // 你的 LinuxAppInfo 声明
#include "utils_env.hpp"          // snapshotEnv / printDiff

extern char **environ;            // <- 必须声明

// 把父进程环境变量采集为 vector<string>
static std::vector<std::string> collectParentEnv()
{
    std::vector<std::string> vec;
    for (char **p = environ; *p; ++p)
        vec.emplace_back(*p);     // *p 是 "KEY=VAL"
    return vec;
}

int main()
{
    namespace fs = std::filesystem;

    // ---------- 1. 设置父环境 ----------
    putenv(const_cast<char*>("FOO=parent"));
    putenv(const_cast<char*>("BAR=parent"));
    putenv(const_cast<char*>("BAZ=parent"));

    // ---------- 2. 创建临时 env 文件 ----------
    fs::path tmp = fs::temp_directory_path() / "demo_env.env";
    std::ofstream(tmp) << "BAR=file\nNEW_FROM_FILE=1\n";

    // ---------- 3. 构造 LinuxAppInfo ----------
    LinuxAppInfo app;
    app.instanceId = "dummy";
    app.execPath   = "/bin/true";
    app.entrance   = {};                               // 无额外参数
    app.envInline  = "FOO=inline,BAR=inline";          // inline 优先级最高
    app.envFile    = tmp.string();

    // ---------- 4. 对比前 / 后 ----------
    auto beforeVec = collectParentEnv();               // 修正点
    std::cout << "Environment before:\n";
    for (const auto& e : beforeVec)
        std::cout << e << '\n';
    auto before    = snapshotEnv(beforeVec);

    auto afterVec  = EnvManager::buildEnvironment(app);
    auto after     = snapshotEnv(afterVec);

    std::cout << "Environment diff:\n";
    printDiff(before, after);

    // ---------- 5. 验证 LD_LIBRARY_PATH 补丁 ----------
    const char* beforeLd = ::getenv("LD_LIBRARY_PATH");
    std::cout << "\nLD_LIBRARY_PATH (parent) : "
              << (beforeLd ? beforeLd : "(unset)") << '\n';

    std::cout << "LD_LIBRARY_PATH (patched): "
              << after["LD_LIBRARY_PATH"] << '\n';

    fs::remove(tmp);
    return 0;
}