#include "json.hpp"
#include <iostream>
#include <fstream>
#include "../hdmap_server/data-access-engine/proxy/lane_proxy.h"
#include "../hdmap_server/data-access-engine/proxy/tile_proxy.h"
#include "./include/ElementDeal.h"
#include "./include/DataManager.h"
#include "./include/CommonUtil.h"

#include "pclPtType.h"
#include "upload2RoadServer.h"
#include "afterProcess.h"
#include "afterProcessCommon.h"
#include "AfterProcessStopLine.h"
#include "AfterProcessJunctionBoundary.h"
#include "bindrelation.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

void transStructLine(Engine::Base::Array<RoadMapping::LB> arrRBs, std::vector<SingleLine> &vecLine)
{
    for (int i = 0; i < arrRBs.GetCount(); ++i)
    {
        if (arrRBs[i].linePts.GetCount() < 2)
            continue;
        SingleLine newLine;
        for (int j = 0; j < arrRBs[i].linePts.GetCount(); ++j)
            newLine.linePtsVec.push_back(data_access_engine::Vector3D(arrRBs[i].linePts[j].x, arrRBs[i].linePts[j].y, arrRBs[i].linePts[j].z));
        vecLine.push_back(newLine);
    }
}

void transStructLG(Engine::Base::Array<RoadMapping::LG> arrLGs, std::vector<LaneGroup> &vecLG)
{
    for (int i = 0; i < arrLGs.GetCount(); ++i)
    {
        LaneGroup newLG;
        transStructLine(arrLGs[i].laneGroupLBs, newLG.lbVec);
        if (newLG.lbVec.size() > 1)
            vecLG.push_back(newLG);
    }
}

void transStructRM(Engine::Base::Array<RoadMapping::RM> arrRMs, std::vector<Position> &vecPosition, std::vector<Position> &vecTrafficInfo)
{
    for (int i = 0; i < arrRMs.GetCount(); ++i)
    {
        int n = arrRMs[i].plygonPnts.GetCount();
        Position newPosition;
        newPosition.ObjectType = arrRMs[i].ObjectType;
        newPosition.ArrowType = arrRMs[i].ArrowType;

        if (arrRMs[i].ObjectType == 7)
        {
            if (n != 4)
            {
                std::cout << "此箭头的外包框个数不等于4，请检查数据！！！" << std::endl;
                continue;
            }

            for (int j = 0; j < n; ++j)
            {
                newPosition.psPtsVec.push_back(data_access_engine::Vector3D(arrRMs[i].plygonPnts[j].x, arrRMs[i].plygonPnts[j].y, arrRMs[i].plygonPnts[j].z));
            }
            vecTrafficInfo.push_back(newPosition);
        }
        else
        {
            if (n < 2)
                continue;
            for (int j = 0; j < n; ++j)
            {
                newPosition.psPtsVec.push_back(data_access_engine::Vector3D(arrRMs[i].plygonPnts[j].x, arrRMs[i].plygonPnts[j].y, arrRMs[i].plygonPnts[j].z));
            }
            vecPosition.push_back(newPosition);
        }
    }
}

void transStructJuntion(Engine::Base::Array<Engine::Base::Array<RoadMapping::LB>> arrArrRBs, std::vector<std::vector<SingleLine>> &vecVecLines)
{
    for (int i = 0; i < arrArrRBs.GetCount(); ++i)
    {
        std::vector<SingleLine> newLines;
        transStructLine(arrArrRBs[i], newLines);
        vecVecLines.push_back(newLines);
    }
}
void split(const std::string &str, const std::string &delim, vector<std::string> &vecData)
{

    if ("" == str)
    {
        return;
    }
    char *strs = new char[str.length() + 1];
    strncpy(strs, str.c_str(), str.length());
    char *ptr = NULL;
    char *p = strtok_r(strs, delim.data(), &ptr);
    while (p)
    {
        string s = p;
        vecData.push_back(s);
        p = strtok_r(NULL, delim.data(), &ptr);
    }
    delete[] strs;
    strs = NULL;
}
using namespace data_access_engine;
void getRightStopLine(const string &taskId, Array<RoadMapping::LB> &stopLineArray)
{
    map<string, string> mapTaskTiles;
    mapTaskTiles["29723"] = "322711688,322711689,322711691,322711688,322711689,322711691,322711689,322711683,322711684,322711686,322711689,322711683,322711689,322711683,322711688,322711689,322711683,322711691";
    mapTaskTiles["29721"] = "322711693,322711695,322711692,322711693,322711692,322711693,322711695,322711692,322711693,322711695";
    mapTaskTiles["29719"] = "322711697,322711611,322711699,322711697,322711611,322711696,322711697,322711611,322711697,322711611";
    mapTaskTiles["29698"] = "322711716,322711717,322711719,322711730,322711731,322711717,322711719,322711717,322711718,322711719,322711717,322711718,322711719,322711724,322711725,322711695,322711717,322711719,322711717,322711719";
    mapTaskTiles["29678"] = "322711752,322711746,322711703,322711746,322711703,322711746";
    mapTaskTiles["29668"] = "322708969,322708962,322708963,322708968,322708969,322708962,322708963,322708969,322708963,322708972,322708966,322708969,322708963,322708972,322708969,322708963,322708972,322708966,322708969,322708963,322708972,322708966,322708969,322708963,322708972,322708966,322708969,322708963,322708972,322708966,322708969,322708963,322708972,322708966,322708963";
    mapTaskTiles["29667"] = "322709316,322708974,322708970,322708971";
    mapTaskTiles["29665"] = "322708978,322708984,322708986,322708973,322708975";
    mapTaskTiles["29659"] = "322708983,322708988,322708989,322708990,322708991,322708988,322708989,322708990,322708983,322709332,322708988,322708989,322708990,322708991,322708988,322708989,322708990,322708991,322708988,322708989,322708990,322708991,322708988,322708989,322708990,322708988,322708989,322708990,322708988,322708989,322708990,322708991,322708988,322708990,322708991";
    mapTaskTiles["29656"] = "322708923,322709268,322708926,322708921,322708923,322709268,322708926,322708921,322708923,322708926,322708923,322709268,322708926,322708923,322708926,322708923,322708924,322708926,322708923,322708926,322708923,322708921,322708923,322708923,322708926,322708923,322709268,322708926,322709265,322708923,322708926,322708920,322708921,322708923,322708926,322709265,322708923,322708926";

    map<string, string> mapTaskBranch;
    mapTaskBranch["29723"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675767014_44176";
    mapTaskBranch["29721"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675767013_95515";
    mapTaskBranch["29719"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675767012_35764";
    mapTaskBranch["29698"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675766999_50504";
    mapTaskBranch["29678"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675766988_46762";
    mapTaskBranch["29668"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675760060_18305";
    mapTaskBranch["29667"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675760059_57003";
    mapTaskBranch["29665"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675760058_60927";
    mapTaskBranch["29659"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675760054_12414";
    mapTaskBranch["29656"] = "road_2023q1beijingshanghailisanrenwu_1674866256585499_1675760052_66734";

    string branch = mapTaskBranch[taskId];
    string strTileId = mapTaskTiles[taskId];

    auto _mgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    auto conf = data_access_engine::ConfigAddress::get_instance();
    conf->set_road_server_address("172.21.207.124:8081");
    conf->set_road_server_download_branch(branch);

    data_access_engine::RoadTileDownloadParam param;
    param.judge_editable = false;
    param.editor_name = "sd_converter";
    vector<string> vecTile;
    set<int> extTile;
    split(strTileId, ",", vecTile);

    for (string tile : vecTile)
    {
        int tileId = atol(tile.data());
        extTile.insert(tileId);
    }

    std::vector<int> tids(extTile.begin(), extTile.end());
    _mgr->load_tiles_by_id(tids, &param, 4);
    _mgr->correct_tiles();
    string tileId = "";
    vector<std::shared_ptr<data_access_engine::TileInfoProxy>> vecTileInfo;
    for (int tileId : tids)
    {
        std::shared_ptr<data_access_engine::TileInfoProxy> tileInfo2 = _mgr->get_road_tile(tileId);
        const SharedProxyVector<PositionObjectProxy> &vecPosition = tileInfo2->position_objects();
        for (shared_ptr<const PositionObjectProxy> position : vecPosition)
        {
            if (position->type().get_value_or(0) != 8)
            {
                continue;
            }
            RoadMapping::LB lb;
            const PolylineProxy *pole = position->pole();
            if (pole != NULL)
            {
                const SharedProxyVector<PointProxy> &vecPt = pole->pts();
                for (shared_ptr<const PointProxy> pt : vecPt)
                {
                    Coordinate coord(pt->x(), pt->y(), pt->z());
                    lb.linePts.Add(coord);
                }
            }
            const PolygonProxy *border = position->border();
            if (border != NULL)
            {
                const SharedProxyVector<PointProxy> &vecPt = border->pts();
                for (shared_ptr<const PointProxy> pt : vecPt)
                {
                    Coordinate coord(pt->x(), pt->y(), pt->z());
                    lb.linePts.Add(coord);
                }
            }
            stopLineArray.Add(lb);
        }
    }
    _mgr->clear_all();
}

//////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    std::cout << "uploadLane argc:" << argc << std::endl;
    std::string dataDir = argv[1];
    std::string parseJosnFile = argv[2];
    std::string global_pcd_path = argv[3];
    bool isUploadLineProxy = false;
    //    std::string road_branch = "";
    //    if(argc > 4)
    //    {
    //       road_branch= argv[4];
    //       isUploadLineProxy = true;
    //    }

    std::cout << "dataDir:" << dataDir << std::endl;
    std::cout << "parseJosnFile:" << parseJosnFile << std::endl;
    std::cout << "global_pcd_path:" << global_pcd_path << std::endl;
    std::cout << "isUploadLineProxy:" << isUploadLineProxy << std::endl;

    nlohmann::json parseJosn;
    std::ifstream ifs_json(parseJosnFile);
    if (!ifs_json.is_open())
    {
        ifs_json.close();
        std::cout << "参数文件打开失败！！！" << std::endl;
        return 0;
    }

    parseJosn << ifs_json;
    std::string road_branch = parseJosn["road_branch"];
    std::vector<double> t_utm_world = parseJosn["t_utm_world"];
    int utm_num = parseJosn["utm_num"];
    std::vector<std::string> links = parseJosn["links"];

    std::string address = "172.21.207.124:8081";
    std::string editor = "test";

    // 解析离散任务的polygon
    Engine::Base::Array<Engine::Geometries::Coordinate> polygon_pts;
    RoadMapping::afterProcessCommon::ReadPolygonPts(parseJosnFile, polygon_pts);
    if (!polygon_pts.IsEmpty())
    {
        Array<Array<Coordinate>> lines;
        lines.Add(polygon_pts);
        hdmap_build::CommonUtil::WriteToOBJ(lines, dataDir, "task_polygon");
    }

    // 解析离散任务的路口中心点
    Engine::Base::Array<Engine::Geometries::Coordinate> lukou_center_pts;
    RoadMapping::afterProcessCommon::ReadCenterPts(parseJosnFile, lukou_center_pts);
    if (!lukou_center_pts.IsEmpty())
    {
        hdmap_build::CommonUtil::WriteToTxt(lukou_center_pts, dataDir, "sub_crosses_center", true);
    }

    // TODO:qzc  提高效率，暂时关闭
#if 0
    // 根据车道线生成车道组
    Array<RoadMapping::LG> arrLGsAll;
    Array<RoadMapping::LB> linkArray;
    for (auto link : links)
    {
        RoadMapping::LB linkLB;
        linkLB.link = link;
        std::string linkDataDir = dataDir + "/" + link;

        std::string refLinePath = linkDataDir + "/laneBoundaryFromSeg/refLink.pcd";
        RoadMapping::afterProcessCommon::ReadRfLine(refLinePath, linkLB.linePts);
        linkArray.Add(linkLB);

        Array<RoadMapping::LG> arrLGs;
        std::cout << "......后处理车道组:" << link << std::endl;
        RoadMapping::afterProcess::afterProcessLB(linkDataDir, polygon_pts, arrLGs);

        if (!arrLGs.IsEmpty())
            arrLGsAll.Add(arrLGs);
    }

    // 读取道路边界
    std::cout << "......后处理道路边界:" << std::endl;
    Array<RoadMapping::LB> arrLBsAll;
    RoadMapping::afterProcess::afterProcessRB(dataDir, links, polygon_pts, arrLBsAll);
#endif 

    // 读取地面标识
    Array<RoadMapping::RM> arrRMs;
    std::cout << "......后处理地面标识......" << std::endl;
    RoadMapping::afterProcess::afterProcessRM(dataDir, links, polygon_pts, arrRMs);

    /////////////////////////////////WSS-后处理停止线//////////////////////////////////////
    Array<RoadMapping::RM> junctionRm;
    Array<RoadMapping::RM> stopLineReuslt;
    // RoadMapping::afterProcess::readJunctionRM(dataDir, links, polygon_pts, junctionRm);
    // AfterProcessStopLine* processStopLine = new AfterProcessStopLine();
    // processStopLine->setStopLineArray(junctionRm);
    // processStopLine->setJunctionPt(lukou_center_pts);
    // processStopLine->setLaneGroupArray(arrLGsAll);
    // processStopLine->setRefLink(linkArray);
    // processStopLine->process();
    // stopLineReuslt = processStopLine->getResultStopLine();
    // delete processStopLine;
    /////////////////////////////////WSS-后处理停止线//////////////////////////////////////

    /////////////////////////////////后处理停止线//////////////////////////////////////
    std::cout << "......后处理停止线......" << std::endl;
    RoadMapping::afterProcess::afterProcessStopLine(dataDir, polygon_pts, stopLineReuslt);

    // 将停止线包装为pcd提供给yy
    Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> stop_lines;
    for (int i = 0; i < stopLineReuslt.GetCount(); i++)
    {
        if (stopLineReuslt[i].plygonPnts.GetCount() == 2)
            stop_lines.Add(stopLineReuslt[i].plygonPnts);
    }

    std::string lukou_path = dataDir + "/LuKou";
    std::string stoplinepcd = lukou_path + "/object_stopline.pcd";
    // std::cout << "stoplinepcd:" << stoplinepcd << std::endl;
    if (!stop_lines.IsEmpty())
    {
        RoadMapping::afterProcess::objsToPointElementPCD(stop_lines, stoplinepcd, 6);
    }

    if (!stopLineReuslt.IsEmpty())
    {
        arrRMs.Add(stopLineReuslt);
    }

    std::cout << "......根据停止线后处理人行横道......" << std::endl;
    Array<RoadMapping::RM> crossWalkReuslt;
    RoadMapping::afterProcess afterProcess_crosswolk;
    afterProcess_crosswolk.afterProcessCrossWalk(dataDir, global_pcd_path, polygon_pts, stopLineReuslt, lukou_center_pts, crossWalkReuslt);   
    if (!crossWalkReuslt.IsEmpty())
    {
        arrRMs.Add(crossWalkReuslt);
    }

    // //根据停止线、人行横道生成路口边界
    // AfterProcessJunctionBoundary* processJuncBoundary = new AfterProcessJunctionBoundary();
    // processJuncBoundary->setCrossWalkArray(crossWalkReuslt);
    // processJuncBoundary->setJunctionPt(lukou_center_pts);
    // processJuncBoundary->setLaneGroupArray(arrLGsAll);
    // processJuncBoundary->process();
    // Array<RoadMapping::LB>& junctionBoundary = processJuncBoundary->getJunctionBoundaryArray();
    // Array<Array<RoadMapping::LB>> arrArrLUkouLBs;
    // if(junctionBoundary.GetCount() != 0){
    //     arrArrLUkouLBs.Add(junctionBoundary);
    // }
    // delete processJuncBoundary;

    // //将路口边界包装为pcd提供给yy
    // Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> lukoubd_lines;
    // for(int i = 0; i < junctionBoundary.GetCount(); i++)
    // {
    //     if (junctionBoundary[i].linePts.GetCount() > 2)
    //         lukoubd_lines.Add(junctionBoundary[i].linePts);
    // }

    // std::string lukou_path_1 = dataDir + "/LuKou";
    // std::string lukoubdpcd = lukou_path_1 + "/object_lukoubd.pcd";
    // std::cout<<"lukoubdpcd:"<<lukoubdpcd<<std::endl;
    // RoadMapping::afterProcess::objsToPointElementPCD(lukoubd_lines, lukoubdpcd, 7);
    //////////////////////////////////////////////////////////////////////////

    // 读取路口面
    std::cout << "......后处理路口面......" << std::endl;
    Engine::Base::Array<RoadMapping::Junction> arrJunction;
    RoadMapping::afterProcess::afterProcessLukou(dataDir, polygon_pts, arrJunction);

    // 将路口边界包装为pcd提供给yy
    Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> lukoubd_lines;
    for (int i = 0; i < arrJunction.GetCount(); i++)
    {
        if (arrJunction[i].lukou_polygon_pts.GetCount() > 3)
        {
            lukoubd_lines.Add(arrJunction[i].lukou_polygon_pts);
        }
    }

    std::string lukou_path_1 = dataDir + "/LuKou";
    std::string lukoubdpcd = lukou_path_1 + "/junction_lukoubd.pcd";
    std::cout << "lukoubdpcd:" << lukoubdpcd << std::endl;
    RoadMapping::afterProcess::objsToPointElementPCD(lukoubd_lines, lukoubdpcd, 7);

    // 后处理不可通行区域
    Engine::Base::Array<RoadMapping::ImpassableArea> areas;
    RoadMapping::afterProcess::afterProcessImpassableArea(dataDir, areas);

    // 不可通行区域输出为pcd提供给yy
    pcl::PointCloud<PointElement>::Ptr area_cloud_ptr(new pcl::PointCloud<PointElement>);
    for (int i = 0; i < areas.GetCount(); i++)
    {
        for (int j = 0; j < areas[i].plygonPnts.GetCount(); j++)
        {
            PointElement new_pt_ele;
            new_pt_ele.x = areas[i].plygonPnts[j].x;
            new_pt_ele.y = areas[i].plygonPnts[j].y;
            new_pt_ele.z = areas[i].plygonPnts[j].z;
            new_pt_ele.ele_type = 8;
            new_pt_ele.type1 = areas[i].subtype;
            new_pt_ele.index = j;
            new_pt_ele.id = i;
            area_cloud_ptr->points.push_back(new_pt_ele);
        }
    }

    if (!area_cloud_ptr->empty())
    {
        std::string impassarea_pcd = lukou_path_1 + "/junction_impassablearea.pcd";
        pcl::io::savePCDFileBinary(impassarea_pcd, *area_cloud_ptr);
    }

    // 不可通行区域与路口边界绑定
    RoadMapping::BindRelation::bind_impassablearea_lukoubd(areas, arrJunction);

    // TODO:qzc  提高效率，暂时关闭
#if 0
    if (isUploadLineProxy)
    {
        // 坐标偏转回原始坐标
        std::cout << ".......坐标转换......" << std::endl;
        RoadMapping::afterProcess::ConvertUtmToWgsAdd(t_utm_world, utm_num, arrLGsAll, arrRMs, arrLBsAll, arrJunction);

        // 包装proxy
        Coordinate base_pt; // 从要素中选择第一个元素为 锚点，用来进行 tile id 换算
        if (!arrLGsAll.IsEmpty() && !arrLGsAll[0].laneGroupLBs.IsEmpty() && !arrLGsAll[0].laneGroupLBs[0].linePts.IsEmpty())
            base_pt = arrLGsAll[0].laneGroupLBs[0].linePts[0];
        else if (!arrLBsAll.IsEmpty() && !arrLBsAll[0].linePts.IsEmpty())
            base_pt = arrLBsAll[0].linePts[0];
        else if (!arrRMs.IsEmpty() && !arrRMs[0].plygonPnts.IsEmpty())
            base_pt = arrRMs[0].plygonPnts[0];
        else if (!arrJunction.IsEmpty() && !arrJunction[0].lukou_polygon_pts.IsEmpty())
            base_pt = arrJunction[0].lukou_polygon_pts[0];
        else
        {
            std::cout << "don't have data to upload!!!" << endl;
            return 0;
        }

        std::cout << "......进行数据proxy创建......" << std::endl;
        int tile_id = hdmap_build::CommonUtil::WGS_to_tile_ID(base_pt.x, base_pt.y, base_pt.z);
        auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
        pMgr->init_tiles_by_id({tile_id});

        std::cout << "LaneGroupProxy创建------->" << std::endl;
        std::vector<LaneGroup> vecLG;
        transStructLG(arrLGsAll, vecLG);
        upload2RoadServer::CreatLaneGroupProxyAll(vecLG);

        std::cout << "RoadBoundaryProxy创建------->" << std::endl;
        std::vector<SingleLine> vecRBlines;
        transStructLine(arrLBsAll, vecRBlines);
        upload2RoadServer::CreatRoadBoundaryProxyAll(vecRBlines);

        // std::cout<<"TrafficInfoProxy创建------->"<<std::endl;
        // std::vector<Position> vecPosition, vecTrafficInfo;
        // transStructRM(arrRMs, vecPosition, vecTrafficInfo);
        // upload2RoadServer::CreatPositionProxyAll(vecPosition);
        // upload2RoadServer::CreatTrafficInfoProxyAll(vecTrafficInfo);

        std::cout << "JunctionProxy创建------->" << std::endl;
        upload2RoadServer::CreatJunctionProxyAll(arrJunction);

        // 上传动作
        std::cout << "......数据上传......" << std::endl;
        std::string dataOut = dataDir + "/roadMappingResult/";
        upload2RoadServer::upLoadTiles(address, road_branch, editor, dataOut);
        std::cout << "--------uploadLane finish-------" << std::endl;
    }
#endif

}
