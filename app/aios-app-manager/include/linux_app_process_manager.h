
#ifndef BOS_CORE_SERVICE_LINUX_APP_PROCESS_MANAGER_H
#define BOS_CORE_SERVICE_LINUX_APP_PROCESS_MANAGER_H

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




namespace bos::framework::em {

    class linux_app_process_manager {
    public:
        linux_app_process_manager();

        int start_linux_app(const LinuxAppInfo& linuxAppInfo);
        int restart_linux_app(const LinuxAppInfo& linuxAppInfo);
        void stop_linux_app(const LinuxAppInfo &linuxAppInfo);
        void stop_linux_app(const std::string& pkgName);
        void stop_linux_app(pid_t pid);
        int get_pid_by_exec(std::string& exec);

    public:
        std::string find_package_name_by_pid (const pid_t pid);
        void register_process_died(std::function<void(std::string& pkgname)> callback);
    private:
        void checkExitStatus(pid_t pid);
        void handleSigchld(const asio::error_code& error, int signal_number);
        void asyncWaitForProcessToStop(pid_t pid, std::shared_ptr<asio::steady_timer> timer);
        void remove_pid(pid_t pid);

    private:
        std::string get_executable_path(const std::string& appPath);
        std::vector<std::string> get_library_paths(const std::string& execPath);
        void add_path_to_env(const char* envVar, const std::vector<std::string>& paths);
        void set_ld_library_path(const std::string& exec_path);

    private:
        std::vector<std::pair<std::string, std::string>> parse_environment_string(const std::string& env_str);
        void set_environment_variable(const std::string& key, const std::string&  value);

        void set_environment_variable(const std::vector<std::pair<std::string, std::string>>& env_vars);

        void parse_and_set_environment(const std::string& env_str);

        void set_environment_variable_from_file(const std::string& filePath);
        std::string trim(const std::string& str);
        void current_process_env();
    private:
        std::map<std::string, pid_t> processes;
        asio::signal_set signals_;
        std::function<void(std::string& pkgname)> m_process_died_callback;
        bool is_registed_observed_process_died{false};

    };

}
#endif //BOS_CORE_SERVICE_LINUX_APP_PROCESS_MANAGER_H
