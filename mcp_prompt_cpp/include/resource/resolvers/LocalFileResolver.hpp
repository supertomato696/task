#pragma once
#include "IResolver.hpp"
#include <filesystem>

namespace mcp::resource {

class LocalFileResolver : public IResolver {
public:
    explicit LocalFileResolver(std::string root = "/");

    bool accepts(const std::string& uri) const override;
    nlohmann::json read(const std::string& uri) override;
    std::vector<nlohmann::json> list() override;

private:
    std::string root_;
    std::string toMime(const std::filesystem::path& p);
};

} // namespace