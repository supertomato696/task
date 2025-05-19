#include <asio/io_context.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cstdlib>

#include "EnvManager.hpp"
#include "LinuxAppInfo.hpp"
#include "utils_env.hpp"

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
    app.entrance   = {};
    app.envInline  = "FOO=inline,BAR=inline";    // 优先级最高
    app.envFile    = tmp.string();

    // ---------- 4. 对比前后 ----------
    auto before = snapshotEnv(std::vector<std::string>{environ, nullptr});
    auto afterVec = EnvManager::buildEnvironment(app);
    auto after  = snapshotEnv(afterVec);

    std::cout << "Environment diff:\n";
    printDiff(before, after);

    fs::remove(tmp);

    // … demo_env.cpp 最后部分
auto beforeLd = getenv("LD_LIBRARY_PATH");
std::cout << "LD_LIBRARY_PATH  (parent) : "
          << (beforeLd ? beforeLd : "(unset)") << '\n';

auto afterMap = snapshotEnv(afterVec);
std::cout << "LD_LIBRARY_PATH  (patched): "
          << afterMap["LD_LIBRARY_PATH"] << '\n';

    return 0;
}
