#pragma once

#include <chrono>
#include <filesystem>
#include <string>
#ifdef __linux__
#include <sys/types.h>
#endif



enum class ProcessState { Starting, Running, Stopping, Exited, Unknown, Failed };

struct ChildProcessInfo {

    pid_t                                   pid{};

    ProcessState                            state{ProcessState::Unknown};
    std::string                             instanceId;
    std::string                   execPath;
    std::chrono::steady_clock::time_point   startTime;
    std::chrono::steady_clock::time_point   endTime;
    int                                     exitCode{0};
};

