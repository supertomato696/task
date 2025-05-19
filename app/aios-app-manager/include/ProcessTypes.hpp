#pragma once

#include <chrono>
#include <filesystem>
#include <string>
#ifdef __linux__
#include <sys/types.h>
#endif

namespace lapm {

enum class ProcessState { Starting, Running, Stopping, Exited, Unknown };

struct ProcessInfo {
#ifdef __linux__
    pid_t                                   pid{};
#else
    int                                     pid{};
#endif
    ProcessState                            state{ProcessState::Unknown};
    std::string                             instanceId;
    std::string                   execPath;
    std::chrono::steady_clock::time_point   startTime;
    int                                     exitCode{0};
};

} // namespace lapm