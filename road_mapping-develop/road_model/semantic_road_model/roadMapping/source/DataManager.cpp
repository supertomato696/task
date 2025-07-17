//
//
//
#include "DataManager.h"
#include "../data-access-engine/data_access_engine.h"
#include "../data-access-engine/proxy/lane_proxy.h"
#include "../data-access-engine/proxy/tile_proxy.h"
#include "../data-access-engine/manager/road_geometry_mgr.h"
#include "../data-access-engine/dao/road_tile_dao.h"

using namespace hdmap_build;

std::atomic<RoadDataManager *> RoadDataManager::s_RoadDataManager(nullptr);
// RoadDataManager* RoadDataManager::s_RoadDataManager = nullptr;
std::mutex RoadDataManager::s_RoadDataManger_mutex;

RoadDataManager::RoadDataManager()
{
    _basePoint.x = 0.0;
    _basePoint.y = 0.0;
    _basePoint.z = 0.0;
}

RoadDataManager::~RoadDataManager()
{
}

RoadDataManager *RoadDataManager::getInstance()
{
    if (s_RoadDataManager == nullptr)
    {
        std::lock_guard<std::mutex> locker(s_RoadDataManger_mutex);
        if (s_RoadDataManager == nullptr)
        {
            s_RoadDataManager = new RoadDataManager;
        }
    }

    return s_RoadDataManager;
}

data_access_engine::DAEInterface *RoadDataManager::getDAEInterface()
{
    return data_access_engine::DAEInterface::get_instance();
}

data_access_engine::RoadGeometryManager *RoadDataManager::getRoadGeometryManager()
{
    return getDAEInterface()->get_road_geometry_manager();
}

data_access_engine::ProjectionHelper<15> *RoadDataManager::getProjectionHelper()
{
    return getDAEInterface()->get_projection_helper();
}

data_access_engine::ConfigAddress *RoadDataManager::getConfigAddress()
{
    return data_access_engine::ConfigAddress::get_instance();
}

Vec3 RoadDataManager::getBasePoint()
{
    return _basePoint;
}

void RoadDataManager::setBasePoint(const Vec3 &pt)
{
    std::lock_guard<std::mutex> locker(s_RoadDataManger_mutex);
    _basePoint = pt;
}

bool RoadDataManager::download_all_tile_data(const std::vector<int> &tile_ids, const std::string &strRoadAddress, const std::string &strRoadBranch, data_access_engine::ID2TileMap &tiles_map)
{
    std::cout << "------download_all_tile_data-------";

    auto conf = data_access_engine::ConfigAddress::get_instance();
    conf->set_road_server_address(strRoadAddress);
    conf->set_road_server_download_branch(strRoadBranch);

    auto mgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    std::cout << " start down_load ====================" << std::endl;

    // 开始下载tile内所有数据
    if (!mgr->init_tiles_by_id(tile_ids))
    {
        std::cout << " init tile id failed !" << std::endl;
        return false;
    }

    //    std::cout<<" init tile id success !"<<std::endl;
    //    auto proj = data_access_engine::DAEInterface::get_instance()->get_projection_helper();
    //    proj->set_destination_Proj4_string("+proj=geocent +datum=WGS84");

    // int link_num = 0;
    if (!mgr->load_tiles_by_id(tile_ids, nullptr, 4))
    {
        std::cout << " load tiles data failed !" << std::endl;
        return false;
    }

    if (!mgr->get_road_tiles(tiles_map))
    {
        std::cout << " get road tiles data failed !" << std::endl;
        return false;
    }

    return true;
}
