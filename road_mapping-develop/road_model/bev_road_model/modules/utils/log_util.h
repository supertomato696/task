

#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include "utils/time_checker.h"
#include "utils/string_util.h"

namespace fsdmap {
namespace utils {

using LogLevel = spdlog::level::level_enum;
class Logger;

extern Logger* global_log_ptr; 

class Logger {
// public:
    Logger() {};
    ~Logger() {
        // print();
        spdlog::drop_all();
    };

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

public:
    static Logger* instance() {
        if (global_log_ptr == NULL) {
            global_log_ptr = new Logger();
        }
        return global_log_ptr;
        // static Logger instance;
        // return &instance;
    }

    inline void setLogLevel(const LogLevel &term_log_level, const LogLevel &file_log_level) {
        _file_log_level = file_log_level;
        _console_log_level = term_log_level;
    }

    inline bool setup(const std::string& filename, const std::string &name="log") {
        auto logger = spdlog::get(name);

        if (!logger) {
            auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console->set_level(_console_log_level);

            auto file = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename, true);
            file->set_level(_file_log_level);

            std::vector<spdlog::sink_ptr> sinks;
            sinks.push_back(console);
            sinks.push_back(file);

            logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
            logger->set_level(spdlog::level::trace);
            logger->flush_on(spdlog::level::level_enum::warn);

            spdlog::register_logger(logger);
            //spdlog::flush_every(std::chrono::seconds(5));
        }
        _logger = logger.get();

        if (!_logger) {
            return false;
        }

        return true;
    };

    template<typename Level, typename FormatString, typename... Args>
    void log_d(const Level& level, const char * file, int line, const FormatString &fmt, const Args &... args) {
        if (_logger) {
            if (file != "") {
                std::string log_str = utils::fmt(fmt, args...);
                _logger->log(level, "[{}:{}]{}", file, line, log_str);
            } else {
                _logger->log(level, fmt, args...);
            }
        }
    }

    template<typename Level, typename FormatString, typename... Args>
    void log(const Level& level, const FormatString &fmt, const Args &... args) {
        log_d(level, "", 0, fmt, args...);
    }

    template<typename Level>
    void clog_d(const Level& level, const char * file, int line, const char * fmt, ...) {
        if (_logger) {
            va_list args;
            va_start(args, fmt);
            std::string log_str = utils::vcfmt(fmt, args);
            va_end(args);
            if (file != "") {
                //_logger->log(level, "[{}:{}]{}", __FILENAME__, __LINE__, log_str);
                _logger->log(level, "[{}:{}]{}", file, line, log_str);
            } else {
                _logger->log(level, log_str);
            }
        }
    }

    template<typename Level>
    void clog(const Level& level, const char * fmt, ...) {
        if (_logger) {
            va_list args;
            va_start(args, fmt);
            std::string log_str = utils::vcfmt(fmt, args);
            va_end(args);
            _logger->log(level, log_str);
        }
    }

    template<typename FormatString, typename... Args>
    Logger& push_info(const FormatString &fmt, const Args &... args) {
        if (fmt == NULL) {
            return *this;
        }
        _log_list.push_back(utils::fmt(fmt, args...));
        return *this;
    }

    void print() {
        _timer.check();
        std::string final_log = utils::fmt("[id={}][COST={}]",
                _logid_str, _timer.get_total());
        for (auto &log_str : _log_list) {
            std::string log_item = utils::fmt("[{}]", log_str);
            final_log.append(log_item);
        }
        log(spdlog::level::info, "{}", final_log);
        _log_list.clear();
    }

    fsdmap::TimeChecker& get_timer() {
        return _timer;
    }

    void set_logid(const std::string &id) {
        _logid_str = id;
    }

    // inline void loggerDrop() {
    //     spdlog::drop_all();
    // }

private:
    // std::shared_ptr<spdlog::logger> _logger;
    spdlog::logger* _logger;
    LogLevel _file_log_level    = LogLevel::trace;
    LogLevel _console_log_level = LogLevel::info;
    std::vector<std::string> _log_list;
    fsdmap::TimeChecker _timer;
    std::string _logid_str;
};

}
}

#define __TO_STR(f1,f2) "__"#f1"_"#f2"__"

#ifndef LOG_TRACE
#define LOG_TRACE(format, ...) (fsdmap::utils::Logger::instance()->log_d(spdlog::level::trace, __FILENAME__, __LINE__, format, ##__VA_ARGS__));
#endif // !LOG_TRACE

#ifndef LOG_DEBUG
#define LOG_DEBUG(format, ...) (fsdmap::utils::Logger::instance()->log(spdlog::level::debug, format, ##__VA_ARGS__));
#endif // !LOG_DEBUG

#ifndef LOG_INFO
#define LOG_INFO(format, ...) (fsdmap::utils::Logger::instance()->log(spdlog::level::info, format, ##__VA_ARGS__));
#endif // !LOG_INFO

#ifndef LOG_WARN
#define LOG_WARN(format, ...) (fsdmap::utils::Logger::instance()->log_d(spdlog::level::warn, __FILENAME__, __LINE__, format, ##__VA_ARGS__));
#endif // !LOG_WARN

#ifndef LOG_ERROR
#define LOG_ERROR(format, ...) (fsdmap::utils::Logger::instance()->log_d(spdlog::level::err, __FILENAME__, __LINE__, format, ##__VA_ARGS__));
#endif // !LOG_ERROR

#ifndef CLOG_TRACE
#define CLOG_TRACE(format, ...) (fsdmap::utils::Logger::instance()->clog_d(spdlog::level::trace, __FILENAME__,__LINE__, format, ##__VA_ARGS__));
#endif // !LOG_TRACE

#ifndef CLOG_DEBUG
#define CLOG_DEBUG(format, ...) (fsdmap::utils::Logger::instance()->clog(spdlog::level::debug, format, ##__VA_ARGS__));
#endif // !LOG_DEBUG

#ifndef CLOG_INFO
#define CLOG_INFO(format, ...) (fsdmap::utils::Logger::instance()->clog(spdlog::level::info, format, ##__VA_ARGS__));
#endif // !LOG_INFO

#ifndef CLOG_WARN
#define CLOG_WARN(format, ...) (fsdmap::utils::Logger::instance()->clog_d(spdlog::level::warn, __FILENAME__, __LINE__, format, ##__VA_ARGS__));
#endif // !LOG_WARN

#ifndef CLOG_ERROR
#define CLOG_ERROR(format, ...) (fsdmap::utils::Logger::instance()->clog_d(spdlog::level::err, __FILENAME__, __LINE__, format, ##__VA_ARGS__));
#endif // !LOG_ERROR
