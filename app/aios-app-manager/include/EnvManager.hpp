#pragma once
#include <iostream>
#include <sys/types.h>
#include <iostream>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <condition_variable>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <cstring>
#include <vector>
//#include "impl/storage/db_def.h"
#include "bos_aios_app.h"
#include  <task_util.h>
#include <cstdlib>
#include <asio.hpp>

#include <LinuxAppInfor.hpp>



#include <limits.h>
#include <string>
#include <libgen.h>
#include <fstream>
#include <sstream>



    std::string get_executable_path(const std::string& appPath) {
        if (appPath.empty()) {
            std::cout << "empty path" << std::endl;
            return "";
        }

        char realPath[PATH_MAX];
        if (realpath(appPath.c_str(), realPath) == nullptr) {
            std::cerr << "Failed to resolve the absolute path" << std::endl;
            return "";
        }
        return std::string(realPath);
    }

    std::vector<std::string> get_library_paths(const std::string& execPath) {
        if (execPath.empty()) {
            std::cout << "invalid augment " << execPath << std::endl;
            return {};
        }

        char exe_path_cstr[PATH_MAX];
        std::strcpy(exe_path_cstr, execPath.c_str());
        std::string execDir = dirname(exe_path_cstr);

        std::string exe_dir_str(execDir);
        char exe_dir_cstr[PATH_MAX];
        std::strcpy(exe_dir_cstr, exe_dir_str.c_str());
        std::string parentDir = dirname(exe_dir_cstr);

        std::strcpy(exe_path_cstr, parentDir.c_str());
        std::string grandParentDir = dirname(exe_path_cstr);
        std::vector<std::string> libPaths;
        libPaths.push_back(execDir + "/lib");
        libPaths.push_back(execDir + "/lib64");


        libPaths.push_back(parentDir + "/lib");
        libPaths.push_back(parentDir + "/lib64");

        libPaths.push_back(grandParentDir + "/lib");
        libPaths.push_back(grandParentDir + "/lib64");

        return libPaths;
    }


    void add_path_to_env(const char* envVar, const std::vector<std::string>& paths) {
        const char* currentValue = std::getenv(envVar);
        std::string updateValue;

        if (currentValue != nullptr) {
            std::cout << envVar<< " before set  " << currentValue << std::endl;
            updateValue = std::string(currentValue);
        } else {
            std::cout << envVar << " doesn't exist!" << std::endl;
        }

        for (const auto& path : paths ) {
            if (!updateValue.empty()) {
                updateValue += ":";
            }
            updateValue += path;
        }

        if (setenv(envVar, updateValue.c_str(), 1) != 0) {
            std::cerr << "Failed to set " << envVar << " value!" << std::endl;
            return ;
        }

        std::cout << "set " << envVar << ": "<< updateValue << " successful!" << std::endl;

    }

    void set_ld_library_path(const std::string& exec_path) {
        std::string ab_path = get_executable_path(exec_path);
        if (ab_path.empty()) {
            return ;
        }

        std::vector<std::string> lib_paths = get_library_paths(ab_path);

        add_path_to_env("LD_LIBRARY_PATH", lib_paths);
    }


    std::vector<std::pair<std::string, std::string>> parse_environment_string(const std::string& env_str) {
        if (env_str.empty()) {
            return {};
        }

        std::vector<std::pair<std::string, std::string>> env_vars;
        std::istringstream env_stream(env_str);
        std::string token;

        while (std::getline(env_stream, token, ',')) {
            size_t pos = token.find('=');
            if (pos != std::string::npos) {
                std::string key = token.substr(0, pos);
                std::string value = token.substr(pos + 1);
                env_vars.emplace_back(key, value);
            } else {
                std::cerr << "Invalid format for environment variable: " << token << std::endl;
            }
        }

        return env_vars;
    }

    void set_environment_variable(const std::string& key, const std::string&  value) {
        if (key.empty()) {
            return ;
        }

        if (key == "UID") {
            if (!value.empty()) {
                uid_t uid = static_cast<uid_t>(std::stoi(value));
                if (setuid(uid) != 0) {
                    std::cerr << "Failed to set UID to " << value << std::endl;
                } else {
                    std::cout << "Set UID to " << value << std::endl;
                }
            }  else {
                std::cerr << "UID value is empty, cannot set UID." << std::endl;
            }

            return ;
        }

        if (value.empty()) {
            if (unsetenv(key.c_str()) != 0) {
                std::cerr << "Failed to unset environment variable: " << key << std::endl;
                return ;
            } else {
                std::cout << "Unset " << key << std::endl;
            }
        }  else {
            if (setenv(key.c_str(), value.c_str(), 1) != 0) {
                std::cerr << "Failed to set environment variable: " << key << std::endl;
            } else {
                std::cout << "Set " << key << "=" << value << std::endl;
            }
        }

        return ;
    }

    void set_environment_variable(const std::vector<std::pair<std::string, std::string>>& env_vars) {
        if (env_vars.empty()) {
            return ;
        }

        for (const auto& env_var : env_vars) {
            set_environment_variable(env_var.first, env_var.second);
        }
    }



    void parse_and_set_environment(const std::string& env_str) {
        if (env_str.empty()) {
            return ;
        }
        auto env_vars = parse_environment_string(env_str);
        set_environment_variable(env_vars);
    }

    std::string trim(const std::string &str) {
        size_t first = str.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return {};
        }
        size_t last = str.find_last_not_of(" \t\r\n");
        return str.substr(first, last - first + 1);
    }

    void set_environment_variable_from_file(const std::string& filePath) {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: "  << filePath << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {

            size_t commentPos = line.find('#');
            if (commentPos != std::string::npos) {
                line = line.substr(0, commentPos);
            }

            line = trim(line);

            if (line.empty()) {
                continue;
            }

            size_t delimiterPos = line.find('=');
            if (delimiterPos == std::string::npos) {
                std::cerr << "Invalid format in file: " << filePath << ", line: " << line << std::endl;
                continue;
            }

            std::string name = trim(line.substr(0, delimiterPos));
            std::string value = trim(line.substr(delimiterPos + 1));


            if (value.front() == '"' && value.back() == '"') {
                value = value.substr(1, value.size() - 2);
            }

            if (setenv(name.c_str(), value.c_str(), 1) == 0) {
                std::cout << "Set environment variable: " << name << " = " << value << std::endl;
            } else {
                std::cerr << "Failed to set environment variable: " << name << std::endl;
            }
        }

        file.close();
    }

    void current_process_env() {
            std::string envFile = "/proc/self/environ";
            std::ifstream file(envFile);
            if (!file.is_open()) {
                std::cerr << "failed to open: " << envFile << std::endl;
                return;
            }

            std::stringstream buffer;
            buffer << file.rdbuf();
            std::string content = buffer.str();

            size_t start = 0;
            size_t end = content.find('\0');
            while (end != std::string::npos) {
                std::string envVar = content.substr(start, end - start);
                std::cout << envVar << std::endl;
                start = end + 1;
                end = content.find('\0', start);
            }

            file.close();

    }

