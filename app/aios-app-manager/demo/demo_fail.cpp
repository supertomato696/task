// demo_fail.cpp
#include <asio/io_context.hpp>
#include <iostream>
#include "LinuxAppProcessManager.hpp"

int main() {
    asio::io_context io;
    LinuxAppProcessManager pm{io};

    LinuxAppInfo bad;
    bad.instanceId = "bad";
    bad.execPath   = "/bin/does_not_exist";

    auto pi = pm.start(bad);
    if (pi.state == ProcessState::Failed) {
        std::cerr << "start failed, ec=" << pi.exitCode << '\n';
    }
}
