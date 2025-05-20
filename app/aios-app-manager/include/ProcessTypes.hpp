#pragma once

#include <chrono>
#include <filesystem>
#include <string>
#ifdef __linux__
#include <sys/types.h>
#endif



enum class ProcessState { Starting, Running, Stopping, Exited, Unknown, Failed };

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
    std::chrono::steady_clock::time_point   endTime;
    int                                     exitCode{0};
};

