//
//
//
#include "Log.h"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

using namespace hdmap_build;

Log::Log()
{
}

bool Log::setup(const std::string &filename)
{
    const std::string name = "hdmap_build";
    _logger = spdlog::get(name);

    if (!_logger)
    {
        auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console->set_level(spdlog::level::level_enum::info);

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
}
