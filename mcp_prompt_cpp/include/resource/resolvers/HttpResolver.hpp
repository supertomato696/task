#pragma once
#include "IResolver.hpp"

#include <regex>

namespace mcp::resource {

class HttpResolver : public IResolver {
public:
    bool accepts(const std::string& uri) const override;
    nlohmann::json read(const std::string& uri) override;

private:
    std::string sniffMime(const std::string& url,
                          const std::string& headerMime,
                          const std::string& fallbackExt);
};

} // namespace