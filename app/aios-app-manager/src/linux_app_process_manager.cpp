#include <sys/resource.h>
#include "linux_app_process_manager.h"

#include "EnvManager.hpp"
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
        if (fgets(buffer, 128, pipe) != nullptr) {
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




    std::string linux_app_process_manager::find_package_name_by_pid(const pid_t pid) {
        for (const auto& pair : processes) {
            if (pair.second == pid) {
                return pair.first;
            }
        }
        return {};
    }

}

