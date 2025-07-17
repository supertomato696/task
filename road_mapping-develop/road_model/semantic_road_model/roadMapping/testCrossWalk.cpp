//
//
//
#include "json.hpp"
#include "processCrosswalk.h"
#include "afterProcessCommon.h"
#include "afterProcess.h"
#include "upload2RoadServer.h"
#include "./include/CommonUtil.h"

void transStructRM(Engine::Base::Array<RoadMapping::RM> arrRMs, std::vector<Position> &vecPosition, std::vector<Position> &vecTrafficInfo)
{
    for (int i = 0; i < arrRMs.GetCount(); ++i)
    {
        int n = arrRMs[i].plygonPnts.GetCount();
        Position newPosition;
        newPosition.ObjectType = arrRMs[i].ObjectType;

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

int main(int argc, char *argv[])
{
    std::string dataDir = argv[1];
    std::string road_branch = "yxx_test";
    if (argc > 2)
        road_branch = argv[2];
    std::string lukouDir = dataDir + "/LuKou";

    // 读入停止线、路口中心点
    std::string filepath_stopline = lukouDir + "/stopline_after.obj";
    Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> stop_lines;
    RoadMapping::afterProcessCommon::ReadObj(filepath_stopline, stop_lines);

    std::string lukoudian_file = dataDir + "/sub_crosses_center.txt";
    Engine::Base::Array<Engine::Geometries::Coordinate> lukou_center_pts;
    RoadMapping::afterProcessCommon::Readtxt(lukoudian_file, lukou_center_pts);
    std::string global_pcd_path = "";
    Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> edgePts;
    RoadMapping::processCrosswalk crosswalkpross;
    crosswalkpross.get_crosswalk_by_stopline(global_pcd_path, lukou_center_pts, stop_lines, lukouDir, edgePts, false);

    std::string task_polygon_file = dataDir + "/task_polygon.obj";
    Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> lines;
    RoadMapping::afterProcessCommon::ReadObj(task_polygon_file, lines);
    Engine::Base::Array<Engine::Geometries::Coordinate> polygonPts = lines[0];
    Engine::Base::Array<RoadMapping::RM> arrRMs;
    // 将数据根据polygon进行裁剪
    int n = edgePts.GetCount();
    for (int i = n - 1; i >= 0; i--)
    {
        Array<Coordinate> clipPts = edgePts[i];
        clipPts.Delete(0);
        double disOri = hdmap_build::CommonUtil::GetLength(clipPts);
        RoadMapping::afterProcessCommon::ClipByPolygon(clipPts, polygonPts);
        double dis = hdmap_build::CommonUtil::GetLength(clipPts);
        if (fabs(disOri - dis) > 0.1)
            edgePts.Delete(i);
    }
    if (!edgePts.IsEmpty())
    {
        hdmap_build::CommonUtil::WriteToOBJ(edgePts, lukouDir, "crosswalk", true);
        for (int i = 0; i < edgePts.GetCount(); ++i)
        {
            int nPts = edgePts[i].GetCount();
            if (nPts < 4) // 多边形，最少为4个点，首尾点相同 只存一个
                continue;

            edgePts[i].Delete(nPts - 1);

            RoadMapping::RM newRm;
            newRm.plygonPnts = edgePts[i];
            newRm.ObjectType = 10; // CROSS_WALK = 10;//斑马线
            arrRMs.Add(newRm);
        }
    }

    std::string parseJosnFile = dataDir + "/fsd_mapbuild_task.json";
    nlohmann::json parseJosn;
    std::ifstream ifs_json(parseJosnFile);
    if (!ifs_json.is_open())
    {
        ifs_json.close();
        std::cout << "参数文件打开失败！！！" << std::endl;
        return 0;
    }

    // parseJosn<<ifs_json;
    // std::vector<double> t_utm_world = parseJosn["t_utm_world"];
    // int utm_num = parseJosn["utm_num"];

    // std::cout<<".......坐标转换......"<<std::endl;
    // Array<RoadMapping::LG> arrLGsAll;
    // Array<RoadMapping::LB> arrLBsAll;
    // Array<Array<RoadMapping::LB>> arrArrLUkouLBs;
    // RoadMapping::afterProcess::ConvertUtmToWgsAdd(t_utm_world, utm_num,arrLGsAll,arrRMs,arrLBsAll, arrArrLUkouLBs);

    // Coordinate base_pt;
    // base_pt = arrRMs[0].plygonPnts[0];
    //  std::cout<<"TrafficInfoProxy创建------->"<<std::endl;
    // std::vector<Position> vecPosition, vecTrafficInfo;
    // transStructRM(arrRMs, vecPosition, vecTrafficInfo);
    // upload2RoadServer::CreatPositionProxyAll(vecPosition);
    // upload2RoadServer::CreatTrafficInfoProxyAll(vecTrafficInfo);

    //  //上传动作
    // std::string  address = "172.21.207.124:8081";
    // std::string editor = "test";
    // std::cout<<"......数据上传......"<<std::endl;
    // std::string  dataOut = dataDir + "/roadMappingResult/";
    // upload2RoadServer::upLoadTiles(address,road_branch,editor, dataOut);
    // std::cout<<"--------uploadLane finish-------"<<std::endl;
}