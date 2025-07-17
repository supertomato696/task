

#pragma once

#include <iostream>
#include <fstream>
#include <stdarg.h>
#include <vector>
#include "utils/macro_util.h"
#include <spdlog/fmt/fmt.h>
#include <nlohmann/json.hpp>
namespace fsdmap
{
    namespace utils
    {

        template <typename FormatString, typename... Args>
        std::string fmt(const FormatString &fmt, const Args &...args)
        {
            return fmt::format(fmt, args...);
        }

        extern void split(std::string s, std::string delimiter, std::vector<std::string> &vec);

        extern std::string cfmt(const char *fmt, ...);

        extern std::string vcfmt(const char *fmt, va_list ap);

        template <typename T>
        T safe_get(const nlohmann::json& j, const std::string& key, const T defaultValue) {
            // 检查字段是否存在
            if (j.contains(key)) {
                // 字段存在，尝试读取
                return j[key].get<T>();
            } else {
                // 字段不存在，返回默认值
                // std::cerr << "Field '" << key << "' not found. Using default value." << std::endl;
                // LOG_WARN("Field [{}] not found !",key);
                return defaultValue;
            }
        }

    }
}
