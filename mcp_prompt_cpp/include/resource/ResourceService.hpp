#pragma once
#include "resource/ResourceManager.hpp"
#include <nlohmann/json.hpp>

namespace mcp::resource {

class ResourceService {
public:
    explicit ResourceService(resource::ResourceManager& rm):rm_(rm){}

    nlohmann::json handle(const nlohmann::json& req);

private:
    nlohmann::json list(const std::string& id);
    nlohmann::json read(const std::string& id,const nlohmann::json& p);
    resource::ResourceManager& rm_;
};

}