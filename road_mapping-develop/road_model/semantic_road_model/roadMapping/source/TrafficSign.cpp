//
//
//
#include "TrafficSign.h"
#include "RoadLayer.h"
#include "DataManager.h"
#include "Log.h"
#include "Geometries/BaseAlgorithm.h"

#include "../data-access-engine/proxy/position_proxy.h"

using namespace std;
using namespace hdmap_build;

TrafficSign::TrafficSign()
{
}

TrafficSign::~TrafficSign()
{
}

std::shared_ptr<data_access_engine::PositionObjectProxy> TrafficSign::createObject(const vector<Vec3> &points, data_access_engine::TileInfoPtr &tile)
{
    if (points.size() < 5)
        return nullptr;

    auto basePt = RoadDataManager::getInstance()->getBasePoint();

    auto obj = std::make_shared<data_access_engine::PositionObjectProxy>();
    data_access_engine::Vector3D pos(0.0, 0.0, 0.0);
    for (int i = 0; i < points.size(); ++i)
    {
        auto pt = obj->mutable_border()->mutable_pts()->add();
        pt->set_x(points[i].x);
        pt->set_y(points[i].y);
        pt->set_z(points[i].z);
        auto pp = points[i];
        pp += basePt;

        pos.X() += pp.x;
        pos.Y() += pp.y;
        pos.Z() += pp.z;
    }
    pos.X() /= points.size();
    pos.Y() /= points.size();
    pos.Z() /= points.size();

    auto pMgr = RoadDataManager::getInstance()->getRoadGeometryManager();
    pMgr->make_new_id(pos, obj, tile, true);
    tile->mutable_position_objects()->push_back(obj);
    auto version = obj->id()->version();
    obj->mutable_id()->set_version(version + 1);
    return obj;
}