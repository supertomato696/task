//
//
//

#ifndef HDMAP_BUILD_TRAFFICSIGN_H
#define HDMAP_BUILD_TRAFFICSIGN_H
#include "Vec.h"
#include "Base/Types.h"
#include "Base/Array.h"
#include "Geometries/Coordinate.h"

#include "../data-access-engine/manager/road_geometry_mgr.h"

using namespace std;
using namespace Engine;
using namespace Engine::Base;
using namespace Engine::Geometries;

namespace hdmap_build
{
    struct TrafficSignObj
    {
        Array<Coordinate> plygonPnts; // 矩形框点集
        Coordinate positionPt;        // 定位点
        int entity_id;                // 实体id
        Coordinate centerPt;          // 中心点
        // TLineTwoPoints axisTwoPts; //轴线
        int nearstLineIndex;
    };

    class TrafficSign
    {
    public:
        TrafficSign();
        ~TrafficSign();

        // 初始化参数
        void InitParameter(string strTilePath, string road_branch, int tile_id);

        std::shared_ptr<data_access_engine::PositionObjectProxy> createObject(const vector<Vec3> &arrPolygonPts, data_access_engine::TileInfoPtr &tile);
    };
}

#endif // HDMAP_BUILD_TRAFFICSIGN_H
