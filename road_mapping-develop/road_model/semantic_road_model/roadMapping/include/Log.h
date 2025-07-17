//
//
//

#ifndef HDMAP_BUILD_LOG_H
#define HDMAP_BUILD_LOG_H
#include <string>
#include <memory>
#include <spdlog/spdlog.h>

using namespace std;
namespace hdmap_build
{
    static bool calcPercent(std::size_t index, std::size_t count, double &percent)
    {
        if (index == count - 1)
        {
            percent = 100.0;
            return true;
        }
        for (std::size_t i = 0; i <= 100; i += 5)
        {
            if (static_cast<std::size_t>(static_cast<double>(i) * 0.01 * static_cast<double>(count)) == index)
            {
                percent = i;
                return true;
            }
        }
        return false;
    }

    class Log
    {
        Log();
        ~Log() = default;
        Log(const Log &) = delete;
        Log &operator=(const Log &) = delete;

    public:
        static Log *instance()
        {
            static Log instance;
            return &instance;
        }

        bool setup(const std::string &filename);

        template <typename Level, typename FormatString, typename... Args>
        void log(const Level &level, const FormatString &fmt, const Args &...args)
        {
            if (_logger)
            {
                _logger->log(level, fmt, args...);
            }
        }

    private:
        std::shared_ptr<spdlog::logger> _logger;
    };
}
#ifndef VULCAN_LOG_TRACE
#define VULCAN_LOG_TRACE(format, ...) (hdmap_build::Log::instance()->log(spdlog::level::trace, format, ##__VA_ARGS__));
#endif // !VULCAN_LOG_TRACE

#ifndef VULCAN_LOG_DEBUG
#define VULCAN_LOG_DEBUG(format, ...) (hdmap_build::Log::instance()->log(spdlog::level::debug, format, ##__VA_ARGS__));
#endif // !VULCAN_LOG_DEBUG

#ifndef VULCAN_LOG_INFO
#define VULCAN_LOG_INFO(format, ...) (hdmap_build::Log::instance()->log(spdlog::level::info, format, ##__VA_ARGS__));
#endif // !VULCAN_LOG_INFO

#ifndef VULCAN_LOG_WARN
#define VULCAN_LOG_WARN(format, ...) (hdmap_build::Log::instance()->log(spdlog::level::warn, format, ##__VA_ARGS__));
#endif // !VULCAN_LOG_WARN

#ifndef VULCAN_LOG_ERROR
#define VULCAN_LOG_ERROR(format, ...) (hdmap_build::Log::instance()->log(spdlog::level::err, format, ##__VA_ARGS__));
#endif // !VULCAN_LOG_ERROR

#endif // HDMAP_BUILD_LOG_H
