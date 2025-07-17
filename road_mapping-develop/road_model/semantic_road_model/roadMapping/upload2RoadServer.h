//
//
// 将数据上传至roadserver，数据坐标系UTM
//

#ifndef ROADMAPPING_UPLOAD2ROADSERVER_H
#define ROADMAPPING_UPLOAD2ROADSERVER_H

#include "afterProcessCommon.h"

#include "../hdmap_server/data-access-engine/data_access_engine.h"
#include "../hdmap_server/data-access-engine/manager/road_geometry_mgr.h"
#include "../hdmap_server/data-access-engine/proxy/lane_proxy.h"
#include "../hdmap_server/data-access-engine/proxy/tile_proxy.h"
#include "../hdmap_server/data-access-engine/proxy/lane_proxy.h"
#include "../hdmap_server/data-access-engine/proxy/position_proxy.h"
#include "../hdmap_server/data-access-engine/proxy/traffic_proxy.h"
#include "../hdmap_server/data-access-engine/proxy/common_proxy.h"

/* 上传过程中需要用到的结构体 */

// 单条线结构
struct SingleLine
{
    std::vector<data_access_engine::Vector3D> linePtsVec;
};

// 车道组结构（内部线需按照从左到右顺序排列好）
struct LaneGroup
{
    std::vector<SingleLine> lbVec;
};

// 定位要素结构
struct Position
{
    // 定位要素点集，当为闭合多边形时，首尾点仅传入一个！！！
    std::vector<data_access_engine::Vector3D> psPtsVec;

    // 对应position.proto中enum ObjectType
    int ObjectType;
    // 对应箭头属性
    int ArrowType = 0;
};

class upload2RoadServer
{
public:
    /*******************************************一个任务内多要素proxy生成**************************************************/
    // 任务内所有结果包装proxy
    static void CreatLaneGroupProxyAll(const std::vector<LaneGroup> &vecLGs);

    static void CreatRoadBoundaryProxyAll(const std::vector<SingleLine> &vevRBs);

    static void CreatPositionProxyAll(const std::vector<Position> vecRMs);

    static void CreatTrafficInfoProxyAll(const std::vector<Position> vecRMs);

    static void CreatJunctionProxyAll(Engine::Base::Array<RoadMapping::Junction> vecJunctions);

    /*******************************************************************************************************************/

    /*******************************************单个要素proxy生成**************************************************/
    // 包装RoadBoundaryProxy
    static std::shared_ptr<data_access_engine::RoadBoundaryProxy> CreatRoadBoundaryProxy(const SingleLine &rb, int type);

    // 包装PositionObjectProxy
    static std::shared_ptr<data_access_engine::PositionObjectProxy> CreatPositionProxy(const Position &ps);

    // 包装TrafficInfoProxy 如果不需要属性信息请使用CreatPositionProxy
    static std::shared_ptr<data_access_engine::TrafficInfoProxy> CreateTrafficInfoProxy(const Position &ps);

    // 包装JunctionProxy vevLines为一个路口内所有道路边界
    static std::shared_ptr<data_access_engine::JunctionProxy> CreatJunctionProxy(RoadMapping::Junction junction_input);

    // 包装JunctionProxy内的ImpassableAreaProxy
    static std::shared_ptr<data_access_engine::ImpassableAreaProxy> CreatImpassableAreaProxy(RoadMapping::ImpassableArea impassable_input);

    /******LaneGroupProxy*****/
    // 包装LaneBoundaryProxy
    static std::shared_ptr<data_access_engine::LaneBoundaryProxy> CreateLaneBoundaryProxy(const SingleLine &lb);

    // 包装LaneBoundaryRangeProxy
    static void CreateBoundaryRange(data_access_engine::LaneBoundaryRangeProxy *&boundaryRange,
                                    const std::shared_ptr<const data_access_engine::LaneBoundaryProxy> &b);

    // 包装LaneGroupProxy
    static std::shared_ptr<data_access_engine::LaneGroupProxy> CreateLaneGroupProxy(const LaneGroup &vecGroupLanes);
    /*********************************************************************************************/

    /*******************************************数据上传**************************************************/
    /*
     *address：上传地址 常用"172.21.207.124:8081"
     *road_branch：上传的路网分支
     *editor：上传作者
     *outputDir：数据上传前存储上传的laneboundary和地面标识
     */
    static void upLoadTiles(std::string address, std::string road_branch, std::string editor, std::string outputDir);

    /*******************************************tile内数据输出obj*****************************************/
    static void WriteBoundarysToObj(const data_access_engine::TileInfoList &tiles, std::string filePath, std::string name);

    static void WriteRoadMarkToObj(const data_access_engine::TileInfoList &tiles, std::string path, std::string name);

    static void WriteToObj(std::vector<data_access_engine::Vector3D> vecPts, std::string path, std::string name);

    static void WriteToObj(std::vector<std::vector<data_access_engine::Vector3D>> vecVecPts, std::string path, std::string name);
};
#endif // ROADMAPPING_UPLOAD2ROADSERVER_H
