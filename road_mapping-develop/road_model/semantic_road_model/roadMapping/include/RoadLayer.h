//
//
//

#ifndef HDMAP_BUILD_ROADLAYER_H
#define HDMAP_BUILD_ROADLAYER_H

#include "../data-access-engine/proxy/tile_proxy.h"
#include "Vec.h"
#include "HDLane.h"
#include <vector>
#include <map>

namespace data_access_engine
{
    class LaneGroupProxy;
    class LaneSectionProxy;
    class LaneBoundaryProxy;
    class LaneBoundaryRangeProxy;
    class LaneProxy;
}
namespace hdmap_build
{
    struct LaneSection;
    struct RoadSection
    {
        std::shared_ptr<data_access_engine::LaneGroupProxy> road;
        std::map<int, std::vector<std::shared_ptr<LaneSection>>> lanes;
    };
    struct LaneSection
    {
        std::shared_ptr<data_access_engine::LaneSectionProxy> lanesection;
        std::vector<Vec3> left_points;
        std::vector<Vec3> right_points;
        std::vector<Vec3> middle_points;

        std::weak_ptr<RoadSection> road_section;
        std::weak_ptr<data_access_engine::LaneProxy> lane;
    };
    struct LanePoint
    {
        Vec3 pos;
        Vec3 heading;
        std::weak_ptr<LaneSection> lanesection;
        std::weak_ptr<data_access_engine::LaneBoundaryProxy> lane_boundary;
        int lane_id;
    };

    class RoadLayer
    {
    public:
        RoadLayer();
        virtual ~RoadLayer();

        void setOriTileID(int tile_id);

        void setTilePath(string tile_path);

        //*****************************************构造proxy*****************************************//
        void createBoundaryRange(data_access_engine::LaneBoundaryRangeProxy *&boundaryRange,
                                 const std::shared_ptr<const data_access_engine::LaneBoundaryProxy> &b);

        std::shared_ptr<data_access_engine::LaneBoundaryProxy>
        createBoundary(const std::vector<Vec3> &points, float dis = 0.0);

        // 创建LaneGroupProxy
        std::shared_ptr<data_access_engine::LaneGroupProxy> createRoad(std::vector<HDLane::GroupLane> &vecGroupLanes, data_access_engine::TileInfoPtr &tile);

        // 创建RoadBoundaryProxy
        std::shared_ptr<data_access_engine::RoadBoundaryProxy> CreateRoadBoundary(const std::vector<Vec3> &points);

        //*****************************************测试输出*****************************************//
        void WriteLaneToObj(std::shared_ptr<data_access_engine::LaneProxy> &lane, string filePath, string name);

        void WriteTileObj(data_access_engine::TileInfoList tiles, string filePath, string name);

        void WriteBoundaryToObj(std::shared_ptr<data_access_engine::LaneBoundaryProxy> &lb, string filePath, string name);

        void WriteBoundarysToObj(std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> &tilelbs, string filePath, string name);

        void WriteTileMapsObj(data_access_engine::ID2TileMap tiles_maps, string filePath, string name);

        //*****************************************获取要素*****************************************//
        // 从tiles内获取所有的lanegroup
        std::vector<std::shared_ptr<data_access_engine::LaneGroupProxy>> getLaneGroup(const data_access_engine::ID2TileMap &tiles_map);

        std::vector<std::shared_ptr<data_access_engine::LaneProxy>> getLane(const data_access_engine::ID2TileMap &tiles_map);

        // 从ID2TileMap中解析所有的只包含车道线的LaneBoundury
        std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> getLanesLaneBoundary(const data_access_engine::ID2TileMap &tiles_map);

        // 从ID2TileMap中解析所有的LaneBoundury（此laneboundary中信息较多，包含所有的车道线，中心线，道路边界）
        // 通过type筛选得到需要的需要的类型
        // enum LaneBoundaryType {//车道边界类型
        //         UNKNOWN_BOUNDARY = 0;
        //         LANELINE = 1;//车道标线
        //         GUARDRAIL = 2;//防护栏
        //         TRAFFIC_CONE = 10;//锥桶
        //         PUNCHEON = 11;//离散型障碍物（包括短柱:可表达石墩\短柱等无法穿越的障碍物）
        //         repeated int32 types = 4; //LaneBoundaryType
        std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> getLaneBoundary(const data_access_engine::ID2TileMap &tiles_map, int type = 4);

        // 获取所有的道路边界
        std::vector<std::shared_ptr<data_access_engine::RoadBoundaryProxy>> getRoadBoundary(const data_access_engine::ID2TileMap &tiles_map);

        // 获取所有的定位标识
        std::vector<std::shared_ptr<data_access_engine::PositionObjectProxy>> getPosition(const data_access_engine::ID2TileMap &tiles_map);

        void getLaneBounderyPoints(std::shared_ptr<data_access_engine::LaneBoundaryProxy> &lb_proxy, std::vector<Vec3> &pointList);

        void GetLeftLineStringsInTiles(data_access_engine::TileInfoList &corrtiles, std::vector<std::shared_ptr<data_access_engine::LaneSectionProxy>> &LaneSections, Array<LineString *> &laneLines);

        //*****************************************具体处理函数*****************************************//
        bool BreakUpRoad(std::shared_ptr<data_access_engine::LaneGroupProxy> &_laneGroup);

        // 将创建的lanegroup中数据与现有数据进行去重
        void DealDuplicateLaneGroup(std::vector<std::vector<LineObj>> &lineobjs, std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> &laneBoundarys);

        void DealDuplicateRoadBoundary(std::vector<std::vector<LineObj>> &lineobjs, std::vector<std::shared_ptr<data_access_engine::RoadBoundaryProxy>> &roadBoundarys);

        // 根据提取的lineobjs获取数据经过的tile
        std::vector<int> GetTileIDsByData(std::vector<std::vector<LineObj>> &lineobjs, bool PROJ);

    private:
        LineString *LaneBoundaryToLineString(std::shared_ptr<data_access_engine::LaneBoundaryProxy> &laneboundary, bool utm = false);
        LineString *RoadBoundaryToLineString(std::shared_ptr<data_access_engine::RoadBoundaryProxy> &roadboundary, bool utm = false);

    protected:
        std::string _strLayerName;
        std::string m_strtileFile;
        int m_oriTileID;
    };
}
#endif // HDMAP_BUILD_ROADLAYER_H
