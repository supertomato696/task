#include "resource/ResourceService.hpp"

using namespace mcp;
using namespace mcp::resource;
using json = nlohmann::json;

namespace  protocol = mcp::protocol;

/* ------------------------------------------------------------------ */
ResourceService::ResourceService(ResourceManager& rm, Notify n)
: rm_(rm), notify_(std::move(n))
{
    /* 资源列表改变（例如添加新 Resolver）→ 广播 list_changed */
    rm_.setListChangedCb([this]{ fireListChanged(); });
}

/* ------------------------------------------------------------------ */
json ResourceService::handle(const json& rpc)
{
    protocol::RequestId  id;
    std::string   method;
    json          params;

    if(!protocol::parseRequest(rpc, id, method, params))
        return protocol::makeJsonRpcError(id, -32600, "invalid request");

    try {
        if(method == "resources/list")
            return onList(id, params.get<protocol::ListResourcesParams>());

        if(method == "resources/templates/list")
            return onTemplatesList(id, params.get<protocol::PaginatedParams>());

        if(method == "resources/read")
            return onRead(id, params.get<protocol::ReadResourceParams>());

        if(method == "resources/subscribe")
            return onSubscribe(id, params.get<protocol::SubscribeParams>());

        if(method == "resources/unsubscribe")
            return onUnsubscribe(id, params.get<protocol::UnsubscribeParams>());

        /* 未知方法 */
        return protocol::makeJsonRpcError(id, -32601, "method not found");
    }
    catch(const std::exception& e) {
        return protocol::makeJsonRpcError(id, -32000, e.what());
    }
}

/* ------------------------------------------------------------------ */
/* resources/list */
json ResourceService::onList(const protocol::RequestId& id,
                             const protocol::ListResourcesParams&)
{
    protocol::ListResourcesResult res;
    res.resources = rm_.listResources();
    return protocol::makeJsonRpcResult(id, res);
}

/* resources/templates/list —— 支持分页 */
json ResourceService::onTemplatesList(const protocol::RequestId& id,
                                      const protocol::PaginatedParams& p)
{
    constexpr std::size_t PAGE = 100;

    auto all = rm_.listTemplates();
    std::size_t start = 0;
    if(p.cursor) start = static_cast<std::size_t>(std::stoull(*p.cursor));

    protocol::ListResourceTemplatesResult res;
    auto first = all.begin() + std::min(start, all.size());
    auto last  = all.begin() + std::min(start+PAGE, all.size());
    res.resourceTemplates.assign(first, last);

    if(start + PAGE < all.size())
        res.nextCursor = std::to_string(start + PAGE);

    return protocol::makeJsonRpcResult(id, res);
}

/* resources/read */
json ResourceService::onRead(const protocol::RequestId& id,
                             const protocol::ReadResourceParams& p)
{
    protocol::ReadResourceResult res;
    res.contents = rm_.readResource(p.uri);
    return protocol::makeJsonRpcResult(id, res);
}

/* subscribe / unsubscribe */
json ResourceService::onSubscribe(const protocol::RequestId& id,
                                  const protocol::SubscribeParams& p)
{
    rm_.subscribe(p.uri, [this](const std::string& u){ fireUpdated(u); });
    return protocol::makeJsonRpcResult(id, protocol::Result{});
}

json ResourceService::onUnsubscribe(const protocol::RequestId& id,
                                    const protocol::UnsubscribeParams& p)
{
    rm_.unsubscribe(p.uri);
    return protocol::makeJsonRpcResult(id, protocol::Result{});
}

/* ------------------------------------------------------------------ */
/* notifications */
void ResourceService::fireListChanged()
{
    if(!notify_) return;
    protocol::ResourceListChangedNotification n;
    notify_(json(n));
}

void ResourceService::fireUpdated(const std::string& uri)
{
    if(!notify_) return;
    protocol::ResourceUpdatedNotification n;
    n.params.uri = uri;
    notify_(json(n));
}
