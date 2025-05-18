#include <sys/resource.h>
#include "linux_app_process_manager.h"


namespace bos::framework::em {

    linux_app_process_manager::linux_app_process_manager()
            : signals_(get_application().get_io_context(), SIGCHLD) {
        signals_.async_wait([this](const asio::error_code& error, int signal_number) {
            handleSigchld(error, signal_number);
        });
    }

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

// 停止Linux应用程序
    void linux_app_process_manager::stop_linux_app(const LinuxAppInfo &linuxAppInfo) {
        auto it = processes.find(linuxAppInfo.pkgName);
        if (it != processes.end()) {
            kill(-(it->second), SIGTERM);
            pid_t pid = it->second;
            remove_pid(pid);
            auto timer = std::make_shared<asio::steady_timer>(get_application().get_io_context());
            asyncWaitForProcessToStop(pid, timer);
        }
        else {
            std::cout << "stop_linux_app(const LinuxAppInfo &linuxAppInfo) " << "not found pid " << linuxAppInfo.pkgName << std::endl;
        }
    }
    // 停止Linux应用程序
    void linux_app_process_manager::stop_linux_app(const std::string& pkgName) {
        auto it = processes.find(pkgName);
        if (it != processes.end()) {
            kill(-(it->second), SIGTERM);
            pid_t pid = it->second;
            remove_pid(pid);
            auto timer = std::make_shared<asio::steady_timer>(get_application().get_io_context());
            asyncWaitForProcessToStop(pid, timer);
        } else {
            std::cout << "stop linux app(const std::string& pkgName)" << "not found pid" << pkgName << std::endl;
        }
    }
    // 停止Linux应用程序
    void linux_app_process_manager::stop_linux_app(pid_t pid) {
        for (auto it = processes.begin(); it != processes.end(); ) {
            if (it->second == pid) {
                kill(-pid, SIGTERM);
                auto timer = std::make_shared<asio::steady_timer>(get_application().get_io_context());
                asyncWaitForProcessToStop(it->second, timer);
                break;
            }
        }
    }

    int linux_app_process_manager::get_pid_by_exec(std::string& exec) {
        std::string command = "ps -ef | grep " + exec + " | grep -v grep | awk '{print $2}'";
        int pid;
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            std::cerr << "popen() failed!" << std::endl;
            return -1;
        }
        char buffer[128];
        if (fgets(buffer, 128, pipe) != NULL) {
            if (std::sscanf(buffer, "%d", &pid) == 1) {
                std::cout << "PID: " << pid << std::endl;
                pclose(pipe);
                return pid;
            } else {
                std::cerr << "Failed to read PID from the command output." << std::endl;
            }
        }
        pclose(pipe);
        return -1;
    }

    void linux_app_process_manager::remove_pid(pid_t pid) {
        for (auto it = processes.begin(); it != processes.end(); ) {
            if (it->second == pid) {
                 it = processes.erase(it);
                std::cout << "remove_pid: " << pid << std::endl;
            } else {
                ++it;
            }
        }
    }

// 重新启动Linux应用程序
    int linux_app_process_manager::restart_linux_app(const LinuxAppInfo &linuxAppInfo)  {
        stop_linux_app(linuxAppInfo);
        pid_t newPid = start_linux_app(linuxAppInfo);
        if (newPid == -1) {
            std::cerr << "restart_linux_app" << " Failed to restart process " << linuxAppInfo.pkgName << std::endl;
        }
        return newPid;
    }

// 异步等待进程停止
    void linux_app_process_manager::asyncWaitForProcessToStop(pid_t pid, std::shared_ptr<asio::steady_timer> timer) {
        timer->expires_after(std::chrono::seconds(1));
        timer->async_wait([this, pid, timer](const asio::error_code& error) {

                int status;
                pid_t result = waitpid(pid, &status, WNOHANG);
                if (result == 0) {
                    std::cout << "asyncWaitForProcessToStop" << " Process " << pid << " is still running, sending SIGKILL" << std::endl;
                    kill(-pid, SIGKILL);
                    waitpid(pid, &status, 0);
                }
               // for (auto it = processes.begin(); it != processes.end(); ) {
               //      if (it->second == pid) {
               //         it = processes.erase(it);
               //      } else {
               //         ++it;
               //      }
               // }

                std::cout << "asyncWaitForProcessToStop" << " Stopped process " << pid << std::endl;
        });
    }

    void linux_app_process_manager::checkExitStatus(pid_t pid) {
        int status;
        pid_t result;
        result = waitpid(pid, &status, WNOHANG);

        if (result == 0) {
            std::cout << "checkExitStatus" << " Process " << pid << " is still running" << std::endl;
            return;
        } else if (result == -1) {
            std::cerr << "checkExitStatus" << " Failed to wait for process " << pid << std::endl;
            return;
        }

        if (WIFEXITED(status)) {
            std::cout << "checkExitStatus" << " Process " << pid << " exited with status " << WEXITSTATUS(status) << std::endl;
        } else if (WIFSIGNALED(status)) {

            std::cout << "checkExitStatus" << " Process " << pid << " was killed by signal " << WTERMSIG(status) << std::endl;
        }

        for (auto it = processes.begin(); it != processes.end(); ) {
            if (it->second == pid) {
                it = processes.erase(it);
            } else {
                ++it;
            }
        }

    }
    void linux_app_process_manager::register_process_died(std::function<void(std::string& pkgname)> callback) {
        m_process_died_callback = callback;
        is_registed_observed_process_died = true;
    }

    void linux_app_process_manager::handleSigchld(const asio::error_code& error, int signal_number) {
        if (error) {
            if (error == asio::error::operation_aborted) {
                std::cerr << "handleSigchld " << " Error handling SIGCHLD: Operation aborted" << std::endl;
                std::cerr << "handleSigchld " << " what: " << error.message() << std::endl;
            } else {
                std::cerr << "Error handling SIGCHLD: " << error.message() << std::endl;
            }
            return;
        }

        while (true) {
            int status;
            pid_t pid = waitpid(-1, &status, WNOHANG);
            if (pid <= 0) {
                if (pid == -1 && errno != ECHILD) {
                    std::cerr << "Error calling waitpid: " << strerror(errno) << std::endl;
                }
                break;
            }

            if (WIFEXITED(status)) {
                std::cout << "Child process " << pid << " exited with status " << WEXITSTATUS(status) << std::endl;
            } else if (WIFSIGNALED(status)) {
                std::cout << "Child process " << pid << " was killed by signal " << WTERMSIG(status) << std::endl;
            }
            for (auto it = processes.begin(); it != processes.end(); ) {
                if (it->second == pid) {
                    std::string pkgName(it->first);
                    it = processes.erase(it);
                    if (is_registed_observed_process_died) {
                        std::cout << "Child process " << pid << " died ,callback pkgName: " << pkgName<< std::endl;
                        m_process_died_callback(pkgName);
                    }
                } else {
                    ++it;
                }
            }
        }

        // 再次设置异步等待
        signals_.async_wait([this](const asio::error_code& error, int signal_number) {
            handleSigchld(error, signal_number);
        });
    }

    std::string linux_app_process_manager::get_executable_path(const std::string& appPath) {
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

    std::vector<std::string> linux_app_process_manager::get_library_paths(const std::string& execPath) {
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


    void linux_app_process_manager::add_path_to_env(const char* envVar, const std::vector<std::string>& paths) {
        const char* currentValue = std::getenv(envVar);
        std::string updateValue;

        if (currentValue != NULL) {
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

    void linux_app_process_manager::set_ld_library_path(const std::string& exec_path) {
        std::string ab_path = get_executable_path(exec_path);
        if (ab_path.empty()) {
            return ;
        }

        std::vector<std::string> lib_paths = get_library_paths(ab_path);

        add_path_to_env("LD_LIBRARY_PATH", lib_paths);
    }


    std::vector<std::pair<std::string, std::string>> linux_app_process_manager::parse_environment_string(const std::string& env_str) {
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

    void linux_app_process_manager::set_environment_variable(const std::string& key, const std::string&  value) {
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

    void linux_app_process_manager::set_environment_variable(const std::vector<std::pair<std::string, std::string>>& env_vars) {
        if (env_vars.empty()) {
            return ;
        }

        for (const auto& env_var : env_vars) {
            set_environment_variable(env_var.first, env_var.second);
        }
    }



    void linux_app_process_manager::parse_and_set_environment(const std::string& env_str) {
        if (env_str.empty()) {
            return ;
        }
        auto env_vars = parse_environment_string(env_str);
        set_environment_variable(env_vars);
    }

    std::string linux_app_process_manager::trim(const std::string &str) {
        size_t first = str.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return {};
        }
        size_t last = str.find_last_not_of(" \t\r\n");
        return str.substr(first, last - first + 1);
    }

    void linux_app_process_manager::set_environment_variable_from_file(const std::string& filePath) {
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

    void linux_app_process_manager::current_process_env() {
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

    std::string linux_app_process_manager::find_package_name_by_pid(const pid_t pid) {
        for (const auto& pair : processes) {
            if (pair.second == pid) {
                return pair.first;
            }
        }
        return {};
    }

}

