//
//
//
#include "pclPtType.h"
#include "earth.hpp"
#include "Utils.h"
#include "units.hpp"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"
#include <Eigen/Core>
#include <map>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/common.hpp>
#include "json.hpp"
#include <proj_api.h>
#include <iomanip>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>

using namespace Engine::Geometries;
using namespace Engine::Base;

// 将点云进行转换 参数1：输出文件夹 参数2：原始参数文件
int main(int argc, char *argv[])
{
    std::string dataDir = argv[1];
    std::string pcdFile = argv[2];
    std::string parse_josn = argv[3];
    std::string outputpcdpath = argv[4];

    //     std::string dataDir = "/home/test/data/102041/fsd_mapbuild_out/332491454/";
    //     std::string pcdFile = "/home/test/data/102041/monolink/332491454.pcd";
    //     std::string parse_josn = "/home/test/data/102041/fsd_mapbuild_out/332491454/parse_json.json";
    //     std::string outputpcdpath = "/home/test/data/102041/fsd_mapbuild_out/332491454/transCloud.pcd";

    auto start1 = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::io::loadPCDFile(pcdFile, *pc_ptr);
    std::cout << "......截取点云文件：" << pcdFile << std::endl;
    if (pc_ptr->empty())
        return 0;
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1-start1);
    std::cout << "readpcd time: " << duration1.count() / 1000.0 << std::endl;
    
    //    pcl::PointXYZ minpt, maxpt;
    //
    //    Eigen::Vector4f min_pt;
    //    Eigen::Vector4f max_pt;
    //    pcl::getMinMax3D(*pc_ptr, min_pt, max_pt);

    // 读入转换参数
    std::ifstream ifs_parseInfo(parse_josn);
    if (!ifs_parseInfo.is_open())
    {
        ifs_parseInfo.close();
        std::cout << "没有发现parse_josn：" << parse_josn << endl;
        return 0;
    }

    // 输入参考线
    // 读取参考link线， 并保存为 link.pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr refLinkCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr refPolygonCloud(new pcl::PointCloud<pcl::PointXYZ>);
    nlohmann::json parseInfo;
    parseInfo << ifs_parseInfo;
    std::string center_line = parseInfo["link_geom"];
    int utm_num = parseInfo["utm_num"];

    Utils::GisPolyline new_line;
    boost::geometry::read_wkt(center_line, new_line);

    std::vector<double> p_utm_world = parseInfo["t_utm_world"];
    Eigen::Vector3d t_utm_world(p_utm_world[0], p_utm_world[1], p_utm_world[2]);

    Array<Coordinate> linkPts;
    for (const auto &pt : new_line)
    {
        double x = pt.get<0>() * tools::gl_deg;
        double y = pt.get<1>() * tools::gl_deg;
        double z = pt.get<2>();

        Eigen::Vector3d llh(x, y, z);

        // 转换为UTM坐标
        Eigen::Vector3d xyz = tools::Earth::llh2UTM(llh, utm_num);
        Eigen::Vector3d xyz_offset = xyz - t_utm_world;
        refLinkCloud->push_back(pcl::PointXYZ(xyz_offset(0), xyz_offset(1), xyz_offset(2)));

        linkPts.Add(Coordinate(xyz_offset(0), xyz_offset(1), xyz_offset(2)));
    }
    pcl::io::savePCDFileBinary(dataDir + "/link.pcd", *refLinkCloud);

    //    //读取多边形
    //    Utils::GisPolygon new_py;
    //    std::string task_geom = parseInfo["task_geom"];
    //    boost::geometry::read_wkt(task_geom, new_py);
    //
    //    for (const auto &pt : new_py.outer()){
    //        double x = pt.get<0>() * tools::gl_deg;
    //        double y = pt.get<1>() * tools::gl_deg;
    //        double z = pt.get<2>();
    //        Eigen::Vector3d llh(x,y,z);
    //        //转换为UTM坐标
    //        Eigen::Vector3d xyz = tools::Earth::llh2UTM(llh, utm_num);
    //        Eigen::Vector3d xyz_offset = xyz - t_utm_world;
    //        refPolygonCloud->push_back(pcl::PointXYZ(xyz_offset(0), xyz_offset(1), xyz_offset(2)));
    //    }
    //    pcl::io::savePCDFileBinary(dataDir+"/task_plygon_geom.pcd", *refPolygonCloud);

    // 截取点云
    //    Engine::Geometries::Coordinate pntProject;
    //    Int32 nSegIndex = 0;
    //    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr_new(new pcl::PointCloud<MyColorPointType>);
    //    for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++) {
    //        MyColorPointType &pcl_p = (*iter);
    //        Engine::Geometries::Coordinate pntHitTest(pcl_p.x, pcl_p.y, pcl_p.z);
    //        BaseAlgorithm::GetNearestPntToLineset(pntHitTest, linkPts, pntProject, nSegIndex);
    //        double dis = pntHitTest.DistanceXY(pntProject);
    //        if(dis > 30)
    //            continue;
    //        pc_ptr_new->push_back(pcl_p);
    //    }
    // 截取车道线点云
    Array<Engine::Geometries::Coordinate *> *coordinates = new Array<Engine::Geometries::Coordinate *>;
    int nPts = linkPts.GetCount();
    for (int i = 0; i < nPts; ++i)
    {
        coordinates->Add(new Coordinate(linkPts[i]));
    }
    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr_new(new pcl::PointCloud<MyColorPointType>);
    for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++)
    {
        MyColorPointType &pcl_p = (*iter);
        Engine::Geometries::Coordinate pntHitTest(pcl_p.x, pcl_p.y, pcl_p.z);
        Engine::Geometries::Coordinate pntProject;
        Engine::Base::Int32 nSegIndex = -1;
        bool bFindInLine = true;
        double dis = BaseAlgorithm3D::GetDistanceXYPointToLinesegments(pntHitTest, coordinates, pntProject, nSegIndex, bFindInLine);
        if (dis > 30)
            continue;
        if (nSegIndex == 0 && pntProject.DistanceXY(linkPts[0]) < 1.0E-5)
            continue;
        if (nSegIndex == nPts - 2 && pntProject.DistanceXY(linkPts[nPts - 1]) < 1.0E-5)
            continue;
        pc_ptr_new->push_back(pcl_p);
    }

    std::cout << "[pclcloudprocess]: save cloud size: " << pc_ptr_new->size() << std::endl;

    if (pc_ptr_new->size() > 0) {
        pcl::io::savePCDFileBinary(outputpcdpath, *pc_ptr_new);
    }

    return 0;
}
