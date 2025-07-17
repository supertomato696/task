//
//
//

#ifndef HDMAP_BUILD_DATAMANAGER_H
#define HDMAP_BUILD_DATAMANAGER_H
#include <unordered_map>
#include <memory>
#include <atomic>
#include <mutex>
#include <map>
#include <set>
#include <vector>
#include <Vec.h>
#include "../data-access-engine/dao/config_address.h"
#include "../data-access-engine/proxy/tile_proxy.h"
#include "../data-access-engine/proxy/feature_proxy_base.h"
#include "../data-access-engine/proxy/link_proxy.h"
#include "../data-access-engine/proxy/lane_proxy.h"

namespace data_access_engine
{
    class DAEInterface;
    class ConfigAddress;

    class RoadGeometryManager;

    template <int T>
    class ProjectionHelper;

    class TileInfoProxy;

    class LaneProxy;
    class LaneBoundaryProxy;
    class LaneGroupProxy;
    class LinkProxy;
    class NodeProxy;
    class RoadBoundaryProxy;
    class JunctionProxy;
    class TrafficInfoProxy;
    class PositionObjectProxy;
    class DataQualityProxy;
    class StaticOddProxy;
}

namespace hdmap_build
{
    class RoadDataManager
    {
    public:
        RoadDataManager();
        virtual ~RoadDataManager();

        class LinkExtProxyCreator : public data_access_engine::FeatureProxyCreator<data_access_engine::LinkProxy>
        {
        public:
            virtual data_access_engine::LinkProxy *create()
            {
                return new data_access_engine::LinkExtProxy();
            }
            virtual data_access_engine::LinkProxy *create(data_access_engine::FeatureProxyBase *p)
            {
                return new data_access_engine::LinkExtProxy(p);
            }
        };

        class LaneExtProxyCreator : public data_access_engine::FeatureProxyCreator<data_access_engine::LaneProxy>
        {
        public:
            virtual data_access_engine::LaneProxy *create()
            {
                return new data_access_engine::LaneExtProxy();
            }
            virtual data_access_engine::LaneProxy *create(data_access_engine::FeatureProxyBase *p)
            {
                return new data_access_engine::LaneExtProxy(p);
            }
        };

    public:
        static RoadDataManager *getInstance();

        static data_access_engine::DAEInterface *getDAEInterface();

        static data_access_engine::ConfigAddress *getConfigAddress();

        data_access_engine::RoadGeometryManager *getRoadGeometryManager();

        data_access_engine::ProjectionHelper<15> *getProjectionHelper();

        Vec3 getBasePoint();

        void setBasePoint(const Vec3 &pt);

        // 根据tile号下载数据,下载后的数据存储在ID2TileMap中需要重新进行解析
        bool download_all_tile_data(const std::vector<int> &tile_ids, const std::string &strRoadAddress, const std::string &strRoadBranch, data_access_engine::ID2TileMap &tiles_map);

    private:
        static std::atomic<RoadDataManager *> s_RoadDataManager;
        //        static RoadDataManager* s_RoadDataManager;
        static std::mutex s_RoadDataManger_mutex;
        Vec3 _basePoint;
    };
}

#endif // HDMAP_BUILD_DATAMANAGER_H
