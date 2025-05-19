#pragma once

#include <cerrno>
#include <cstring>
#include <fmt/core.h>
#include <stdexcept>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include "LinuxAppInfo.hpp"
#include "ProcessMonitor.hpp"
#include "EnvManager.hpp"



  int linux_app_process_manager::start_linux_app(const LinuxAppInfo& linuxAppInfo) {
        std::string command = linuxAppInfo.exec;

        auto it = processes.find(linuxAppInfo.pkgName);
        if (it != processes.end()) {
            std::cout << " Process "<< linuxAppInfo.pkgName << " is already started , pid: " <<  it->second << std::endl;
            return it->second;
        }
        int exist_pid = get_pid_by_exec(command);
        if (exist_pid > 0) {
            std::cout << " Process "<< linuxAppInfo.pkgName << " is already started , existed pid: " <<  exist_pid << std::endl;
            return exist_pid;
        }

        pid_t pid = fork();
        if (pid == 0) {
            // Child process
//            setenv("XDG_RUNTIME_DIR","/run/user/20009",1);

            setsid();
            set_ld_library_path(command);

            parse_and_set_environment(linuxAppInfo.categories);

//            if (!linuxAppInfo.actions.empty()) {
//                std::ifstream  file(linuxAppInfo.actions);
//                if (file.is_open()) {
//                    std::stringstream buffer;
//                    buffer << file.rdbuf();
//                    std::string content = buffer.str();
//                    parse_and_set_environment(content);
//                }
//            }


            if (!linuxAppInfo.actions.empty()) {
                set_environment_variable_from_file(linuxAppInfo.actions);
            }

            current_process_env();

            std::vector<char*> execArgs;
            execArgs.push_back(const_cast<char*>(command.c_str()));

            if (!linuxAppInfo.entrance.empty()) {
                execArgs.push_back(const_cast<char *>(linuxAppInfo.entrance.c_str()));
            }

            execArgs.push_back(nullptr);

            execvp(command.c_str(), execArgs.data());
            std::cerr << "start_linux_app" << " execvp: Error executing " << command << " " << strerror(errno) << std::endl;
            exit(EXIT_FAILURE);
        } else if (pid > 0) {
            // Parent process
            int status;
            pid_t result = waitpid(pid, &status, WNOHANG);
            if (result == 0) {
                processes[linuxAppInfo.pkgName] = pid;
                if (linuxAppInfo.nice != 0 && setpriority(PRIO_PROCESS, pid, linuxAppInfo.nice) == 0) {
                    std::cout << "setpriority   pid: " <<  linuxAppInfo.nice << std::endl;
                }
                std::cout << "start_linux_app Started process pid: " << pid << " nice: " << linuxAppInfo.nice << std::endl;
                return pid;
            } else if (result == pid) {
                if (WIFEXITED(status) || WIFSIGNALED(status)) {
                    std::cerr << "start_linux_app" << " Process pid: " << pid << " failed to start or a double fork occurs" << std::endl;
                    return -1;
                }
            } else {
                // result == -1; waitpid failed
                std::cerr << "Failed to start process " << pid << std::endl;
                return -1;
            }
        }

        // pid == -1, fork failed
        std::cerr << "start_linux_app" << " Failed to fork: " << strerror(errno) << std::endl;

        return -1;
    }

