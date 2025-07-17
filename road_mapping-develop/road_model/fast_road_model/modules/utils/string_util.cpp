

#include "utils/string_util.h"

namespace fsdmap {
namespace utils {

void split(std::string s, std::string delimiter, std::vector<std::string> &vec) {
    size_t pos_start = 0;
    size_t pos_end = 0;
    size_t delim_len = delimiter.length();
    std::string token;
    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        vec.push_back(token);
    }
    if (pos_start != s.length()) {
        token = s.substr(pos_start, s.length());
        vec.push_back(token);
    }
}

std::string cfmt(const char * fmt, ...) {
    char buff[1024] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    va_end(args);
    return buff;
}

std::string vcfmt(const char * fmt, va_list ap) {
    char buff[1024] = {0};
    vsnprintf(buff, sizeof(buff), fmt, ap);
    return buff;
}

}
}
