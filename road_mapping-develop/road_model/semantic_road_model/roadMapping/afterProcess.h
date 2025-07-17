#pragma once

#include "pclPtType.h"
#include "afterProcessCommon.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/LineString.h"
#include "../hdmap_server/data-access-engine/manager/road_geometry_mgr.h"
#include "../hdmap_server/data-access-engine/proxy/lane_proxy.h"
#include "../hdmap_server/data-access-engine/proxy/position_proxy.h"
#include "../hdmap_server/data-access-engine/proxy/traffic_proxy.h"
#include "../hdmap_server/data-access-engine/proxy/common_proxy.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

namespace RoadMapping{

class afterProcess{
public:
    //处理lanegroup
    static void afterProcessLB(std::string dataDir, const Array<Coordinate>& polygonPts, Array<LG>& arrLGs); //对初始生成的线后处理总流程
    static void ClipLinesByLink(Array<LB>& arrLines, Array<Coordinate>& refLine); //将超出link范围外的线进行裁剪
    static bool IsConnect(const Array<Coordinate>& firstLine, const Array<Coordinate>& compareLine);//判断两条线段是否能连接
    static void ConnectLanes(std::string dataDir,Array<Coordinate>& refLine, Array<LB> &arrLines);//首先将本能合并的线进行合并
    static void GetLaneGroupByLinkPts(Array<LB>& arrLines, Array<Coordinate>& refLine, Array<LG>& arrLGs); //根据link的形状点切割lanegroup
    static void CalLaneGroupSeq(Array<Coordinate>& refLine, Array<LG>& arrLGs); //根据参考线重新组织lanegroup内顺序，从做左到右排序
    static void DeleteShortLine(LG& lg); //删除一个lanegroup内长度短于最少线比例1/3的线
    static void CombineLGs(Array<LG>& arrLGs); //将前后车道数一致，并且首尾点差距不大相同的车道组进行合并
    static void AdjustLGs(std::string dataDir,Array<LG>& arrLGs); //将车道组进行节点对其操作
    static bool AdjustCutOneLG(LG& lg, Array<int> noCutArr,LG& newlg); //根据拉伸后效果判断是否需要将一个车道组重新打断
    static void GetOutstandLane(Array<LB>& arrLBs, int& resIndex);//判定最凸出线
    static void GetHideLane(Array<LB>& arrLBs, int& resIndex);//判定最不凸出，隐藏比较深的线
    static void Flexibility(LB& dealCHDLane, const LB& compareCHDLane);//对线进行拉伸操作

    static void RfreshZ(std::string dataDir,Array<LG>& arrLGs);//对线进行重新刷高程操作
    static void GenerateOffsetLine(Array<Coordinate>& lineCroods, double distance);
    static void LGsDelete(std::string dataDir,Array<LG>& arrLGs);//利用黄标线进行数据删除 目前为双线
    static bool IsYellowLine(LB lg,const pcl::KdTreeFLANN<MyColorPointType>::Ptr kdtree_cloud);//根据感知结果判断是否为黄标线

    //处理roadMark
    static void afterProcessStopLine(std::string dataDir, const Array<Coordinate>& polygonPts, Array<RM> &arrResultRMs);
    static void afterProcessRM(std::string dataDir, std::vector<std::string> links, const Array<Coordinate>& polygonPts, Array<RM>& arrRMs); //对初始生成的线后处理总流程
    static void filterRMbyRoi(std::string dataDir, std::vector<std::string> links, Array<RM>& arrRMs); //过滤作业区以内的地面标识

    //处理roadBoundary
    static void afterProcessRB(std::string dataDir, std::vector<std::string> links, const Array<Coordinate>& polygonPts, Array<LB>& arrRBs); //对初始生成的线后处理总流程
    static void ConnectRBs(std::string dataDir, std::string midDir,Array<Engine::Geometries::Coordinate>& refLine, Array<LB>& arrRBs, Array<LB>& arrLRBs, Array<LB>& arrRRBs); //对初始生成的线进行连接，区分左右
    static void CalLeftOrRight(Array<Coordinate>& refLine, Array<LB>& arrRBs, Array<LB>& arrLeftLbs, Array<LB>& arrRightLbs); //针对道路边界分左右处理，并进行排序
    static void SingleSideRBDeal(Array<Coordinate>& refLine, Array<LB>& arrRBs);//单侧去重和连接
    static void Flexibility(Coordinate sPt, Coordinate ePt, Array<Coordinate>& refLine, Array<Coordinate>& resPts);//对线进行拉伸操作
    static void RfreshZ(std::string dataDir,Array<LB>& arrLBs);//对线进行重新刷高程操作
    static void DuplicateRemovald(Array<Array<Engine::Geometries::Coordinate>> arrArrRefLine, Array<Array<LB>> arrArrLeftLbs, Array<Array<LB>> arrArrRightLbs, Array<LB>& resArrLBs, double tolerance);//针对道路边界的去重

    //处理路口面
    static void afterProcessLukou(std::string dataDir, const Array<Coordinate>& polygonPts, Engine::Base::Array<Junction>& arrJunction); //对初始生成的线后处理总流程
    void afterProcessCrossWalk(std::string dataDir, std::string global_pcd, const Array<Coordinate>& polygonPts, const Engine::Base::Array<RM> &arrStopLineObjs, const Engine::Base::Array<Engine::Geometries::Coordinate> lukou_center_pts, Engine::Base::Array<RM> &arrCrossWalkObjs);//生成人行横道
    static void afterProcessImpassableArea(std::string dataDir, Engine::Base::Array<ImpassableArea>& areas);
    static void readImpassableArea(std::string impassablearea_obj, Engine::Base::Array<ImpassableArea>& areas, int subtype);
    
    //偏转坐标，并转换为wgs84
    static void ConvertUtmToWgsAdd(std::vector<double> t_utm_world, int utm_num, Engine::Base::Array<LG> &arrLGs, Engine::Base::Array<RM> &arrRMs,
                                     Engine::Base::Array<LB> &arrRBs, Engine::Base::Array<Junction> &arrJunctions);

    static void UtmToWgsAdd(std::vector<double> t_utm_world, int utm_num, Array<Coordinate>& pnts);

    static void readJunctionRM(std::string dataDir, std::vector<std::string> links, const Array<Coordinate>& polygonPts, Array<RM> &arrRMs);

    static void objsToPointElementPCD(const Array<Array<Engine::Geometries::Coordinate>>& arrLines, std::string pcdpath, uint16_t ele_type=0, uint16_t type1=0, uint16_t type2=0, uint16_t type3=0,float heading=0.0, float score=1.0);
};

} //namespace RoadMapping