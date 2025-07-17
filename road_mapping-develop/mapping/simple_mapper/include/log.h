//
//
//
#pragma once

#include <string>
#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

using LogLevel = spdlog::level::level_enum;

class Logger
{
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

public:
    static Logger *instance()
    {
        static Logger instance;
        return &instance;
    }

    inline void setLogLevel(const LogLevel &term_log_level, const LogLevel &file_log_level)
    {
        _file_log_level = file_log_level;
        _console_log_level = term_log_level;
    }

    inline bool setup(const std::string &filename, const std::string &name = "simplemapper")
    {
        _logger = spdlog::get(name);

        if (!_logger)
        {
            auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console->set_level(_console_log_level);

            auto file = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename, true);
            file->set_level(spdlog::level::level_enum::trace);

            std::vector<spdlog::sink_ptr> sinks;
            sinks.push_back(console);
            sinks.push_back(file);

            _logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
            _logger->set_level(spdlog::level::trace);
            _logger->flush_on(spdlog::level::level_enum::warn);

            spdlog::register_logger(_logger);
            // spdlog::flush_every(std::chrono::seconds(5));
        }

        if (!_logger)
        {
            return false;
        }

        return true;
    };

    template <typename Level, typename FormatString, typename... Args>
    void log(const Level &level, const FormatString &fmt, const Args &...args)
    {
        if (_logger)
        {
            _logger->log(level, fmt, args...);
        }
    }

    inline void loggerDrop()
    {
        spdlog::drop_all();
    }

private:
    Logger() {};
    ~Logger() {
    };
    std::shared_ptr<spdlog::logger> _logger;
    LogLevel _file_log_level = LogLevel::trace;
    LogLevel _console_log_level = LogLevel::info;
};

#ifndef LOG_TRACE
#define LOG_TRACE(format, ...) (Logger::instance()->log(spdlog::level::trace, format, ##__VA_ARGS__));
#endif // !LOG_TRACE

#ifndef LOG_DEBUG
#define LOG_DEBUG(format, ...) (Logger::instance()->log(spdlog::level::debug, format, ##__VA_ARGS__));
#endif // !LOG_DEBUG

#ifndef LOG_INFO
#define LOG_INFO(format, ...) (Logger::instance()->log(spdlog::level::info, format, ##__VA_ARGS__));
#endif // !LOG_INFO

#ifndef LOG_WARN
#define LOG_WARN(format, ...) (Logger::instance()->log(spdlog::level::warn, format, ##__VA_ARGS__));
#endif // !LOG_WARN

#ifndef LOG_ERROR
#define LOG_ERROR(format, ...) (Logger::instance()->log(spdlog::level::err, format, ##__VA_ARGS__));
#endif // !LOG_ERROR
