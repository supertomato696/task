

#ifndef ROADMAPPING_UPLOAD2ROADSERVER_H
#define ROADMAPPING_UPLOAD2ROADSERVER_H

#include "../thirdparty/hdmap_server/data-access-engine/data_access_engine.h"
#include "../thirdparty/hdmap_server/data-access-engine/manager/road_geometry_mgr.h"
#include "../thirdparty/hdmap_server/data-access-engine/proxy/lane_proxy.h"
#include "../thirdparty/hdmap_server/data-access-engine/proxy/tile_proxy.h"
#include "../thirdparty/hdmap_server/data-access-engine/proxy/lane_proxy.h"
#include "../thirdparty/hdmap_server/data-access-engine/proxy/position_proxy.h"
#include "../thirdparty/hdmap_server/data-access-engine/proxy/traffic_proxy.h"
#include "../thirdparty/hdmap_server/data-access-engine/proxy/common_proxy.h"
#include "dao/data_processer.h"


#include "road_model/road_model_proc_trans_interface.h"
#include "road_model/road_model_meta.h"
/* 上传过程中需要用到的结构体 */

//单条线结构
struct SingleLine{
    std::vector<data_access_engine::Vector3D> linePtsVec;
};

//车道组结构（内部线需按照从左到右顺序排列好）
struct LaneGroup{
    std::vector<SingleLine> lbVec;
};

//定位要素结构
struct Position
{
    //定位要素点集，当为闭合多边形时，首尾点仅传入一个！！！
    std::vector<data_access_engine::Vector3D> psPtsVec;

    //对应position.proto中enum ObjectType
    int ObjectType;
};

class upload2RoadServer{
public:
    /*******************************************一个任务内多要素proxy生成**************************************************/
    //任务内所有结果包装proxy
    static void CreatLaneGroupProxyAll(const std::vector<LaneGroup>& vecLGs);
    static void testCreatLaneGroupProxyAll(std::vector<fsdmap::RoadLaneGroupInfo*>& vecLGs, fsdmap::dao::DataProcessorBase* dp);

    static void CreatRoadBoundaryProxyAll(const std::vector<SingleLine>& vevRBs);
    static void testCreatRoadBoundaryProxyAll(const std::vector<fsdmap::RoadBoundarySegmentInfo*> vevRBs, fsdmap::dao::DataProcessorBase* dp);
    
    static void CreatPositionProxyAll(const std::vector<Position> WvecRMs);

    static void CreatTrafficInfoProxyAll(const std::vector<Position> vecRMs);

    static void CreatJunctionProxyAll(const std::vector<std::vector<SingleLine>>& vecVecLines);

    /*******************************************************************************************************************/

    /*******************************************单个要素proxy生成**************************************************/
     //包装LaneProxy
    static std::shared_ptr<data_access_engine::LaneExtProxy> testCreateLaneProxy(fsdmap::RoadLaneInfo& vecLanes, fsdmap::dao::DataProcessorBase* dp, int& seq_no, std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> &leftboundarys);

    //包装RoadBoundaryProxy
    static std::shared_ptr<data_access_engine::RoadBoundaryProxy> CreatRoadBoundaryProxy(const SingleLine& rb, int type);
    static std::shared_ptr<data_access_engine::RoadBoundaryProxy> testCreatRoadBoundaryProxy(const fsdmap::RoadBoundarySegmentInfo& rb, fsdmap::dao::DataProcessorBase* dp);

    //包装PositionObjectProxy
    static std::shared_ptr<data_access_engine::PositionObjectProxy> CreatPositionProxy(const Position& ps);
    static std::shared_ptr<data_access_engine::PositionObjectProxy> testCreatPositionProxy(const fsdmap::RoadObjectInfo& rb, fsdmap::dao::DataProcessorBase* dp); //bev_road_model上传定位要素代码

    //包装TrafficInfoProxy 如果不需要属性信息请使用CreatPositionProxy
    static std::shared_ptr<data_access_engine::TrafficInfoProxy> CreateTrafficInfoProxy(const Position& ps);
    static std::shared_ptr<data_access_engine::TrafficInfoProxy> testCreateTrafficInfoProxy(fsdmap::RoadObjectInfo& rb, fsdmap::dao::DataProcessorBase* dp); //bev_road_model上传定位要素代码

    //包装JunctionProxy vevLines为一个路口内所有道路边界
    static std::shared_ptr<data_access_engine::JunctionProxy> CreatJunctionProxy(const std::vector<SingleLine>& vevLines);
    static std::shared_ptr<data_access_engine::JunctionProxy> testCreatJunctionProxy(const fsdmap::JunctionInfo& junction, fsdmap::dao::DataProcessorBase* dp);

    /******LaneGroupProxy*****/
    //包装LaneBoundaryProxy
    static std::shared_ptr<data_access_engine::LaneBoundaryProxy> CreateLaneBoundaryProxy(const SingleLine& lb);
    static std::shared_ptr<data_access_engine::LaneBoundaryProxy> testCreateLaneBoundaryProxy(const fsdmap::RoadLaneBoundaryInfo& lb, fsdmap::dao::DataProcessorBase* dp);

    //包装LaneBoundaryRangeProxy
    static void CreateBoundaryRange(data_access_engine::LaneBoundaryRangeProxy*& boundaryRange,
                                    const std::shared_ptr<const data_access_engine::LaneBoundaryProxy>& b);
    //包装LaneGroupProxy
    static std::shared_ptr<data_access_engine::LaneGroupProxy> newCreateLaneGroupProxy(fsdmap::RoadLaneGroupInfo& vecGroupLanes , fsdmap::dao::DataProcessorBase* dp);
    static std::shared_ptr<data_access_engine::LaneGroupProxy> CreateLaneGroupProxy(const LaneGroup& vecGroupLanes);
    static std::shared_ptr<data_access_engine::LaneGroupProxy> testCreateLaneGroupProxy(fsdmap::RoadLaneGroupInfo& vecGroupLanes , fsdmap::dao::DataProcessorBase* dp);
    
    //包装不可通行区域
    //包装JunctionProxy内的ImpassableAreaProxy
    static std::shared_ptr<data_access_engine::ImpassableAreaProxy> testCreatImpassableAreaProxy(const fsdmap::ImpassableAreaInfo& impassable_input, fsdmap::dao::DataProcessorBase* dp);

    //包装trafficlight
    static std::shared_ptr<data_access_engine::TrafficInfoProxy> testCreatTrafficLightProxy(const fsdmap::RoadObjectInfo& traffic_light, fsdmap::dao::DataProcessorBase* dp);
    /*********************************************************************************************/

    /*******************************************数据上传**************************************************/
    /*
     *address：上传地址 常用"172.21.207.124:8081"
     *road_branch：上传的路网分支
     *editor：上传作者
     *outputDir：数据上传前存储上传的laneboundary和地面标识
     */
    static void upLoadTiles(std::string address, std::string road_branch, std::string editor, std::string outputDir);
    static void testupLoadTiles(std::string address, std::string road_branch, std::string editor, std::string outputDir);

    /*******************************************tile内数据输出obj*****************************************/
    static void WriteBoundarysToObj(const data_access_engine::TileInfoList& tiles, std::string filePath, std::string name);

    static void WriteRoadMarkToObj(const data_access_engine::TileInfoList& tiles, std::string path, std::string name);

    static void WriteToObj(std::vector<data_access_engine::Vector3D> vecPts, std::string path, std::string name);

    static void WriteToObj(std::vector<std::vector<data_access_engine::Vector3D>> vecVecPts, std::string path, std::string name);
};
#endif //ROADMAPPING_UPLOAD2ROADSERVER_H
