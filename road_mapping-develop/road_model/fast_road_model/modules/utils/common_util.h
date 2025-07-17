

#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <gflags/gflags.h>
#include <boost/filesystem.hpp>
#include <boost/threadpool.hpp>
#include "utils/macro_util.h"
#include "utils/log_util.h"

// DEFINE_bool(skip_process_bar_waiting, false, "skip_process_bar_waiting");

namespace fsdmap {
namespace utils {

template<typename res_fun>
class ResPtr {
public:
    ResPtr(res_fun release_ptr) : _rel_ptr(release_ptr) {
    }
    
    ResPtr(res_fun open_ptr, res_fun release_ptr) : _rel_ptr(release_ptr) {
        open_ptr();
    }

    virtual ~ResPtr() {
        _rel_ptr();
    }

private:
    res_fun _rel_ptr;
};

class ProcessBar {
public:
    ProcessBar() {
        reset();
    }
    virtual ~ProcessBar() {}
    volatile std::atomic<int64_t> num_total;
    std::atomic<int64_t> num_finish;
    std::atomic<int64_t> num_biz;
    std::atomic<int64_t> num_error;

    template<typename FormatString, typename... Args>
    void wait(int64_t sleep_sec, const FormatString &desc, const Args &... args) {
        if (num_total == 0) {
            LOG_INFO("{} empty", desc);
            return;
        }
        while (1) {
            int rate = num_finish * 100 / num_total;
            LOG_INFO("{}[{}%, {}, {}]", 
                    desc, rate, num_finish, num_biz);
            if (num_finish == num_total) {
                break;
            }
            // if (FLAGS_skip_process_bar_waiting || sleep_sec == 0) {
            if (sleep_sec == 0) {
                break;
            }
            sleep(sleep_sec);
        }
    }

    void reset() {
        num_total = 0;
        num_finish = 0;
        num_biz = 0;
        num_error = 0;
    }

    void set_error() {
        ++num_error;
    }

public:
    std::mutex mutex;
};

class ThreadPoolProxy {
public:
    ThreadPoolProxy(int thread_num) {
        _thread_pool = std::make_shared<boost::threadpool::pool>(thread_num);
    }
    virtual ~ThreadPoolProxy() {}

    void schedule(std::function<void(ProcessBar *bar)> run_fun);
    void schedule(std::function<void()> run_fun);

    template<typename FormatString, typename... Args>
    void wait(int64_t bar_freq, const FormatString &bar_log, const Args &... args) {
        _bar.wait(bar_freq, bar_log, args...);
        _thread_pool->wait();
        last_err_num = _bar.num_error;
        _bar.reset();
    }

    void wait() {
        _thread_pool->wait();
        last_err_num = _bar.num_error;
        _bar.reset();
    }

    bool had_error() {
       return last_err_num > 0;
    }

private:
    ProcessBar _bar;
    std::shared_ptr<boost::threadpool::pool> _thread_pool;
    int last_err_num;
};


extern int get_memory_by_pid(pid_t pid, std::string name);

extern int get_machine_memory();

extern std::string get_curr_meminfo(std::string &str);

extern int glob_dir_all(const char * dir, std::vector<std::string> &ret, bool need_dir=false);

extern int glob_dir(const char * dir, std::vector<std::string> &ret);

extern int utm_2_wgs(int zone, Eigen::Vector3d &utm, Eigen::Vector3d &wgs);

extern int wgs_2_utm(int zone, Eigen::Vector3d &wgs, Eigen::Vector3d &utm);

extern int creat_dir(const char *path);

}
}
