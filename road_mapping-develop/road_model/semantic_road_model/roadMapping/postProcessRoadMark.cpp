#include "postProcessRoadMark.h"
#include "cluster.h"
#include "pclPtType.h"
#include "pclFilter.h"
#include "Utils.h"
#include "json.hpp"
#include "earth.hpp"
#include "Geometries/GeometryAlgorithm.h"
#include "./include/CommonUtil.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <pcl-1.12/pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/filesystem.hpp>
#include <pcl/sample_consensus/method_types.h>//随机参数估计方法
#include <pcl/sample_consensus/model_types.h>//模型定义
#include <pcl/segmentation/sac_segmentation.h>//基于采样一致性分割的类的头文件
#include <pcl/filters/voxel_grid.h>//基于体素网格化的滤波
#include <pcl/filters/extract_indices.h>//索引提取
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/registration/icp.h>
#include <pcl/console/print.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <proj_api.h>
#include <queue>
#include <string.h>

#include <iostream>
#include <pcl/registration/icp.h>
#include <cmath>
#include <string.h>
#include <stdio.h>

namespace RoadMapping{
#define RAD_TO_DEG	57.29577951308232

void PostProcessRoadMark::run(std::string dataDir, std::string outputDir)
{
    std::string reconsMergeOutputDir = dataDir+"/reconstruction/output";
    std::string bevInfoJsonFile = reconsMergeOutputDir + "/bev_info.json";
    std::cout<<"bevInfoJsonFile: "<<bevInfoJsonFile<<std::endl;
    std::ifstream ifs_bevInfo(bevInfoJsonFile);
    if(!ifs_bevInfo.is_open()){
        ifs_bevInfo.close();
        return;
    }
    nlohmann::json bevInfo;
    bevInfo<<ifs_bevInfo;
    int utm_num = bevInfo["utm_num"];
    float pixelSize = bevInfo["pixel_size_m"];
    Eigen::Matrix3d T_utm_bev = Eigen::Matrix3d::Identity();
    T_utm_bev<<bevInfo["T_utm_bev"][0][0], bevInfo["T_utm_bev"][0][1], bevInfo["T_utm_bev"][0][2],
               bevInfo["T_utm_bev"][1][0], bevInfo["T_utm_bev"][1][1], bevInfo["T_utm_bev"][1][2],
               bevInfo["T_utm_bev"][2][0], bevInfo["T_utm_bev"][2][1], bevInfo["T_utm_bev"][2][2];
    std::cout<<"T_utm_bev:\n"<<T_utm_bev<<std::endl;
    Eigen::Vector2d UTM_base = T_utm_bev.block<2,1>(0,2);
    projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
    std::string utm_param = "+proj=utm";
    utm_param = utm_param + " +zone=" + std::to_string(utm_num) + "N +ellps=WGS84 +no_defs";
    projPJ g_utm = pj_init_plus(utm_param.data());

    auto UTM2llh = [&](Eigen::Vector3d utmPos){
        Eigen::Vector3d llh = utmPos;
        pj_transform(g_utm, g_pWGS84, 1, 1, &llh(0), &llh(1), &llh(2));
        std::swap(llh(0), llh(1));
        return llh;
    };

    Eigen::Vector3d init_llh = UTM2llh(Eigen::Vector3d(UTM_base(0), UTM_base(1), 0.f));
    std::cout<<"init_llh: "<<std::setprecision(16)<<init_llh.transpose()<<std::endl;

    Eigen::Matrix3d Cen = tools::Earth::Pos2Cne(init_llh).transpose();
    Eigen::Vector3d init_ecef = tools::Earth::LLH2ECEF(init_llh);
    auto ENU2UTM = [&](Eigen::Vector3d pos_enu){
        Eigen::Vector3d pos_ecef = init_ecef + Cen*pos_enu;
        Eigen::Vector3d pos_llh = tools::Earth::ECEF2LLH(pos_ecef);
        Eigen::Vector3d pos_utm(pos_llh(1), pos_llh(0), pos_llh(2));
        pj_transform(g_pWGS84, g_utm, 1, 1, &pos_utm(0), &pos_utm(1), &pos_utm(2));
        return pos_utm;
    };

    //get lane boundary
    std::string laneBoundaryFromSegDir = dataDir+"/laneBoundaryFromSeg";
    pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::string> files;
    Utils::getFiles(laneBoundaryFromSegDir, files, [](std::string str)->bool{
        return str.rfind("_trjCloud.pcd")==str.length()-strlen("_trjCloud.pcd");
    });
    for(auto file : files){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::string file_name = laneBoundaryFromSegDir+"/"+file;
        pcl::io::loadPCDFile (file_name, *cloud);
        if(cloud->size()>refLaneBoundaryCloud->size()){
            refLaneBoundaryCloud = cloud;
        }
    }
    std::cout<<"at PostProcessRoadMark, refLaneBoundaryCloud size: "<<refLaneBoundaryCloud->size()<<std::endl;
    {
        //transform refLaneBoundaryCloud coordinate
        nlohmann::json laneBoundaryJson;
        std::ifstream ifs_laneBoundary(laneBoundaryFromSegDir+"/laneBoundary.json");
        laneBoundaryJson<<ifs_laneBoundary;
        Eigen::Vector3d ref_llh;
        ref_llh<<laneBoundaryJson["base"]["lat"], laneBoundaryJson["base"]["lon"], laneBoundaryJson["base"]["alt"];
        std::cout<<"ref_llh: "<<std::setprecision(16)<<ref_llh.transpose()<<std::endl;
        Eigen::Vector3f disp = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, ref_llh).cast<float>();
        for(int i=0; i<refLaneBoundaryCloud->size(); i++){
            refLaneBoundaryCloud->at(i).getVector3fMap() += disp;
        }
        pcl::io::savePCDFileBinary(outputDir+"/refLaneBoundaryCloud.pcd", *refLaneBoundaryCloud);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneBoundary(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree_refLaneBoundary->setInputCloud(refLaneBoundaryCloud); // 设置要搜索的点云，建立KDTree


     //"line_id": 255, "arrow_id": 200, "road_id": 150, "boundary_id": 100
    std::string roadSegImgFile = reconsMergeOutputDir + "/bev_road_seg.png";
    cv::Mat img_bevRoadSeg = cv::imread(roadSegImgFile, cv::IMREAD_GRAYSCALE);
    std::cout << "img rows: " << img_bevRoadSeg.rows << " cols: " << img_bevRoadSeg.cols << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr roadArrowCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<img_bevRoadSeg.rows; i++){
        for(int j=0; j<img_bevRoadSeg.cols; j++){
            if(img_bevRoadSeg.at<uchar>(i,j)!=200) continue;
            Eigen::Vector3d utmPos(T_utm_bev(0,0)*j+T_utm_bev(0,1)*i+T_utm_bev(0,2),
                        T_utm_bev(1,0)*j+T_utm_bev(1,1)*i+T_utm_bev(1,2), 0.f);
            Eigen::Vector3d llh = UTM2llh(utmPos);
            Eigen::Vector3d pos = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
            roadArrowCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
        }
    }
    if(roadArrowCloud!=NULL && roadArrowCloud->size()>0){
        pcl::io::savePCDFileBinary(outputDir+"/roadArrowCloud.pcd", *roadArrowCloud);
    }
    float radius = 0.15f;
    int minPts = 5;
    std::vector<std::vector<int>> clusters;
    Inference::DBSCAN(roadArrowCloud, radius, minPts, clusters);
    std::cout<<"extract traffic arrows, clusters size: "<<clusters.size()<<std::endl;
    nlohmann::json obj;
    obj["base"]["lat"] = init_llh(0);
    obj["base"]["lon"] = init_llh(1);
    obj["base"]["alt"] = init_llh(2);

    for(int index=0; index<clusters.size(); index++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*roadArrowCloud, clusters[index], *clusterCloud);
        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(index)+"_roadArrowCluster.pcd", *clusterCloud);
        if(clusterCloud->size() < 500){
            continue;
        }
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            centroid += clusterCloud->at(i).getVector3fMap();
        }
        centroid = centroid/clusterCloud->size();
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            cov += posDiff*posDiff.transpose();
        }
        cov = cov/clusterCloud->size();
        cov = (cov + cov.transpose())/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.compute(cov);
        Eigen::Vector3f eigenvalues = es.eigenvalues();
        std::cout<<index<<" th cluster, eigenvalues: "<<eigenvalues.transpose()<<std::endl;
        if(eigenvalues(0) > 1e-3 || eigenvalues(1) < 100*eigenvalues(0)){
            continue;
        }
        Eigen::Matrix3f U = es.eigenvectors();
        Eigen::Vector3f nz = U.col(0);
        if(nz.z() < 0.f) nz = -nz;
        std::cout<<"nz: "<<nz.transpose()<<std::endl;
        if(nz.z() < 0.7f){
            continue;
        }
        //determine heading direction
        std::vector<int> indices;
        std::vector<float> squaredDistances;
        pcl::PointXYZ searchPt(centroid.x(), centroid.y(), centroid.z());
        kdtree_refLaneBoundary->radiusSearch(searchPt, 20.f, indices, squaredDistances);
        std::cout<<"searched indices size: "<<indices.size()<<std::endl;
        if(indices.size() < 2){
            continue;
        }
        std::sort(indices.begin(), indices.end());
        Eigen::Vector3f direction = refLaneBoundaryCloud->at(indices.back()).getVector3fMap() -
                                        refLaneBoundaryCloud->at(indices.front()).getVector3fMap();
        std::cout<<"direction: "<<direction.transpose()<<std::endl;
        direction.normalize();
        Eigen::Vector3f nx = direction - direction.dot(nz)*nz;
        nx.normalize();
        Eigen::Vector3f ny = nz.cross(nx);
        float x_min = 1e6, x_max = -1e6, y_min = 1e6, y_max = -1e6;
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            float x = posDiff.dot(nx);
            if(x < x_min) x_min = x;
            if(x > x_max) x_max = x;
            float y = posDiff.dot(ny);
            if(y < y_min) y_min = y;
            if(y > y_max) y_max = y;
        }
        std::cout<<"x length: "<<x_max - x_min<<" y length: "<<y_max - y_min<<std::endl;
        if(x_max - x_min > 12.f || y_max - y_min > 3.f || x_max - x_min < 5.f){
            continue;
        }
        Eigen::Vector3f LeftFront = centroid + x_max*nx + y_max*ny;
        Eigen::Vector3f RightFront = centroid + x_max*nx + y_min*ny;
        Eigen::Vector3f LeftRear = centroid + x_min*nx + y_max*ny;
        Eigen::Vector3f RightRear = centroid + x_min*nx + y_min*ny;
        pcl::PointCloud<pcl::PointXYZ>::Ptr boxCloud(new pcl::PointCloud<pcl::PointXYZ>);
        auto generateSegmentCloud = [&](Eigen::Vector3f p0, Eigen::Vector3f p1){
            Eigen::Vector3f dp = p1 - p0;
            float num = dp.norm()/0.1f;
            for(int i=0; i<num; i++){
                Eigen::Vector3f pos = p0 + (i/num)*dp;
                boxCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            }
        };
        generateSegmentCloud(LeftFront, RightFront);
        generateSegmentCloud(RightFront, RightRear);
        generateSegmentCloud(RightRear, LeftRear);
        generateSegmentCloud(LeftRear, LeftFront);
        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(index)+"_box.pcd", *boxCloud);

        nlohmann::json singleTrafficArrow;
        singleTrafficArrow["id"] = index;
        singleTrafficArrow["LeftFront"] = {LeftFront.x(), LeftFront.y(), LeftFront.z()};
        singleTrafficArrow["RightFront"] = {RightFront.x(), RightFront.y(), RightFront.z()};
        singleTrafficArrow["RightRear"] = {RightRear.x(), RightRear.y(), RightRear.z()};
        singleTrafficArrow["LeftRear"] = {LeftRear.x(), LeftRear.y(), LeftRear.z()};
        obj["trafficArrow"].push_back(singleTrafficArrow);
    }
    std::ofstream fid_trafficArrow(outputDir+"/trafficArrow.json");
    //fid_trafficArrow << std::setw(4) << obj << std::endl;
    fid_trafficArrow<< obj << std::endl;
}

void PostProcessRoadMark::run_yxx(std::string dataDir, std::string pcdPath,std::string refFile,std::string outputDir)
{
#if 0
    //假设utm_zone = 50
    int utm_num = 50;
    std::string parse_json = dataDir + "/parse_json.json";

    //读取参数中的度带号，偏转点的utm坐标
    Eigen::Vector3d offset_utm;
    Utils::readParse(parse_json, utm_num, offset_utm);

    std::string lidarFile = pcdPath;
    std::cout<<"lidarFile: "<<lidarFile<<std::endl;
    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::io::loadPCDFile(lidarFile, *pc_ptr);

    if(pc_ptr->empty())
        return;

    //输入第一个点作为偏转点
    MyColorPointType pcl_p = (*pc_ptr->begin());
    Eigen::Vector3d firstPt = Eigen::Vector3d(pcl_p.x, pcl_p.y, pcl_p.z);
    Eigen::Vector3d init_llh = tools::Earth::UTM2llh(Eigen::Vector3d(firstPt(0), firstPt(1), 0.f),utm_num);

    std::cout<<"init_llh: "<<init_llh.transpose()<<std::endl;

    Eigen::Matrix3d Cen = tools::Earth::Pos2Cne(init_llh).transpose();
    Eigen::Vector3d init_ecef = tools::Earth::LLH2ECEF(init_llh);

    //get lane boundary
    std::string laneBoundaryFromSegDir = dataDir+"/laneBoundaryFromSeg";
    pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::string> files;
    Utils::getFiles(laneBoundaryFromSegDir, files, [](std::string str)->bool{
        return str.rfind("_trjCloud.pcd")==str.length()-strlen("_trjCloud.pcd");
    });
    for(auto file : files){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::string file_name = laneBoundaryFromSegDir+"/"+file;
        pcl::io::loadPCDFile (file_name, *cloud);
        if(cloud->size()>refLaneBoundaryCloud->size()){
            refLaneBoundaryCloud = cloud;
        }
    }
    std::cout<<"at PostProcessRoadMark, refLaneBoundaryCloud size: "<<refLaneBoundaryCloud->size()<<std::endl;
    {
        //transform refLaneBoundaryCloud coordinate
        nlohmann::json laneBoundaryJson;
        std::ifstream ifs_laneBoundary(laneBoundaryFromSegDir+"/laneBoundary.json");
        laneBoundaryJson<<ifs_laneBoundary;
        Eigen::Vector3d ref_llh;
        ref_llh<<laneBoundaryJson["base"]["lat"], laneBoundaryJson["base"]["lon"], laneBoundaryJson["base"]["alt"];
        std::cout<<"ref_llh: "<<std::setprecision(16)<<ref_llh.transpose()<<std::endl;
        Eigen::Vector3f disp = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, ref_llh).cast<float>();
        for(int i=0; i<refLaneBoundaryCloud->size(); i++){
            refLaneBoundaryCloud->at(i).getVector3fMap() += disp;
        }
        pcl::io::savePCDFileBinary(outputDir+"/refLBRes.pcd", *refLaneBoundaryCloud);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneBoundary(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree_refLaneBoundary->setInputCloud(refLaneBoundaryCloud); // 设置要搜索的点云，建立KDTree

    //label： "line": 80, "arrow_id": 100, "road_id": 20
    pcl::PointCloud<pcl::PointXYZ>::Ptr roadArrowCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++){
        MyColorPointType &pcl_p = (*iter);
        if(pcl_p.label == 100)
        {
            Eigen::Vector3d llh = tools::Earth::UTM2llh(Eigen::Vector3d(pcl_p.x, pcl_p.y, pcl_p.z),utm_num);
            Eigen::Vector3d pos = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
            roadArrowCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
        }
    }

    if(roadArrowCloud!=NULL && roadArrowCloud->size()>0){
        pcl::io::savePCDFileBinary(outputDir+"/roadArrowCloud.pcd", *roadArrowCloud);
    }
    float radius = 0.15f;
    int minPts = 5;
    std::vector<std::vector<int>> clusters;
    Inference::DBSCAN(roadArrowCloud, radius, minPts, clusters);
    std::cout<<"extract traffic arrows, clusters size: "<<clusters.size()<<std::endl;
    nlohmann::json obj;
    obj["base"]["lat"] = init_llh(0);
    obj["base"]["lon"] = init_llh(1);
    obj["base"]["alt"] = init_llh(2);

    for(int index=0; index<clusters.size(); index++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*roadArrowCloud, clusters[index], *clusterCloud);
        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(index)+"_roadArrowCluster.pcd", *clusterCloud);
        if(clusterCloud->size() < 500){
            continue;
        }
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            centroid += clusterCloud->at(i).getVector3fMap();
        }
        centroid = centroid/clusterCloud->size();
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            cov += posDiff*posDiff.transpose();
        }
        cov = cov/clusterCloud->size();
        cov = (cov + cov.transpose())/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.compute(cov);
        Eigen::Vector3f eigenvalues = es.eigenvalues();
        std::cout<<index<<" th cluster, eigenvalues: "<<eigenvalues.transpose()<<std::endl;
        if(eigenvalues(0) > 1e-3 || eigenvalues(1) < 100*eigenvalues(0)){
            continue;
        }
        Eigen::Matrix3f U = es.eigenvectors();
        Eigen::Vector3f nz = U.col(0);
        if(nz.z() < 0.f) nz = -nz;
        std::cout<<"nz: "<<nz.transpose()<<std::endl;
        if(nz.z() < 0.7f){
            continue;
        }
        //determine heading direction
        std::vector<int> indices;
        std::vector<float> squaredDistances;
        pcl::PointXYZ searchPt(centroid.x(), centroid.y(), centroid.z());
        kdtree_refLaneBoundary->radiusSearch(searchPt, 20.f, indices, squaredDistances);
        std::cout<<"searched indices size: "<<indices.size()<<std::endl;
        if(indices.size() < 2){
            continue;
        }
        std::sort(indices.begin(), indices.end());
        Eigen::Vector3f direction = refLaneBoundaryCloud->at(indices.back()).getVector3fMap() -
                                    refLaneBoundaryCloud->at(indices.front()).getVector3fMap();
        std::cout<<"direction: "<<direction.transpose()<<std::endl;
        direction.normalize();
        Eigen::Vector3f nx = direction - direction.dot(nz)*nz;
        nx.normalize();
        Eigen::Vector3f ny = nz.cross(nx);
        float x_min = 1e6, x_max = -1e6, y_min = 1e6, y_max = -1e6;
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            float x = posDiff.dot(nx);
            if(x < x_min) x_min = x;
            if(x > x_max) x_max = x;
            float y = posDiff.dot(ny);
            if(y < y_min) y_min = y;
            if(y > y_max) y_max = y;
        }
        std::cout<<"x length: "<<x_max - x_min<<" y length: "<<y_max - y_min<<std::endl;
        if(x_max - x_min > 12.f || y_max - y_min > 3.f || x_max - x_min < 5.f){
            continue;
        }
        Eigen::Vector3f LeftFront = centroid + x_max*nx + y_max*ny;
        Eigen::Vector3f RightFront = centroid + x_max*nx + y_min*ny;
        Eigen::Vector3f LeftRear = centroid + x_min*nx + y_max*ny;
        Eigen::Vector3f RightRear = centroid + x_min*nx + y_min*ny;
        pcl::PointCloud<pcl::PointXYZ>::Ptr boxCloud(new pcl::PointCloud<pcl::PointXYZ>);
        auto generateSegmentCloud = [&](Eigen::Vector3f p0, Eigen::Vector3f p1){
            Eigen::Vector3f dp = p1 - p0;
            float num = dp.norm()/0.1f;
            for(int i=0; i<num; i++){
                Eigen::Vector3f pos = p0 + (i/num)*dp;
                boxCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            }
        };
        generateSegmentCloud(LeftFront, RightFront);
        generateSegmentCloud(RightFront, RightRear);
        generateSegmentCloud(RightRear, LeftRear);
        generateSegmentCloud(LeftRear, LeftFront);
        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(index)+"_box.pcd", *boxCloud);

        nlohmann::json singleTrafficArrow;
        singleTrafficArrow["id"] = index;
        singleTrafficArrow["LeftFront"] = {LeftFront.x(), LeftFront.y(), LeftFront.z()};
        singleTrafficArrow["RightFront"] = {RightFront.x(), RightFront.y(), RightFront.z()};
        singleTrafficArrow["RightRear"] = {RightRear.x(), RightRear.y(), RightRear.z()};
        singleTrafficArrow["LeftRear"] = {LeftRear.x(), LeftRear.y(), LeftRear.z()};
        obj["trafficArrow"].push_back(singleTrafficArrow);
    }
    std::ofstream fid_trafficArrow(outputDir+"/trafficArrow.json");
    //fid_trafficArrow << std::setw(4) << obj << std::endl;
    fid_trafficArrow<< obj << std::endl;
#endif
}

void PostProcessRoadMark::run_yxx_2(std::string dataDir,std::string pcdPath,std::string refFile,std::string outputDir)
{
#if 0
    // 假设utm_zone = 50
    int utm_num = 50;
    std::string parse_json = dataDir + "/parse_json.json";

    // 读取参数中的度带号，偏转点的utm坐标
    Eigen::Vector3d offset_utm;
    Utils::readParse(parse_json, utm_num, offset_utm);

    std::string lidarFile = pcdPath;
    // std::cout<<"lidarFile: "<<lidarFile<<std::endl;
    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::io::loadPCDFile(pcdPath, *pc_ptr);

    if(pc_ptr->empty())
        return;

    //get lane boundary
    std::string laneBoundaryFromSegDir = dataDir+"/laneBoundaryFromSeg";
    pcl::PointCloud<pcl::PointXYZI>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<std::string> files;
    Utils::getFiles(laneBoundaryFromSegDir, files, [](std::string str)->bool{
        return str.rfind("_trjCloud.pcd")==str.length()-strlen("_trjCloud.pcd");
    });
    int n = 0;
    for(auto file : files){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::string file_name = laneBoundaryFromSegDir+"/"+file;
        pcl::io::loadPCDFile(file_name, *cloud);
        for (auto iter = cloud->begin(); iter != cloud->end(); iter++)
        {
            refLaneBoundaryCloud->push_back(pcl::PointXYZI(iter->x, iter->y, iter->z, n));
        }
        n++;
    }

    //label： "line": 80, "arrow_id": 100, "road_id": 20
    pcl::PointCloud<pcl::PointXYZ>::Ptr roadArrowCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr otherRMCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++){
        MyColorPointType &pcl_p = (*iter);
        if(pcl_p.label == 100)
        {
            roadArrowCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }
        else if(pcl_p.label == 110)
        {
            otherRMCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }
    }
    std::cout<<"-------提取箭头------"<<std::endl;
    road_mark_base(roadArrowCloud, refLaneBoundaryCloud, outputDir, 100, 500, 5.0, 3.0, 0.5); //提取箭头
    std::cout<<"-------提取其他地面要素------"<<std::endl;
    road_mark_base(otherRMCloud, refLaneBoundaryCloud, outputDir, 110, 200, 1.0, 3.0, 1.0); //提取sign_line, slow_down_triangle, speed_sign, diamond, bicycle_sign, parking_line
#endif
}

void PostProcessRoadMark::run_yxx_fin_in(std::string dataDir,std::string pcdPath,std::string refFile,std::string outputDir)
{
    //1、读取所有地面标识
    std::string lidarFile = pcdPath;
    std::cout << "lidarFile: " << lidarFile << std::endl;
    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::io::loadPCDFile(lidarFile, *pc_ptr);

    pcl::PointCloud<MyColorPointType>::Ptr cloudfilter100(new pcl::PointCloud<MyColorPointType>);
    // pclFilter::ConditionalRemoval(pc_ptr, cloudfilter100, "cloud_pano_seg", 17);
    pclFilter::ConditionalRemoval(pc_ptr, cloudfilter100, "cloud_bev_label_1", 4);

    pcl::PointCloud<MyColorPointType>::Ptr cloudfilter110(new pcl::PointCloud<MyColorPointType>);
    // pclFilter::ConditionalRemoval(pc_ptr, cloudfilter110, "cloud_pano_seg", 18);
    pclFilter::ConditionalRemoval(pc_ptr, cloudfilter110, "cloud_bev_label_1", 5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr roadArrowCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr otherRMCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto iter = cloudfilter100->begin(); iter != cloudfilter100->end(); iter++){
        MyColorPointType &pcl_p = (*iter);
        roadArrowCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
//        otherRMCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
    }

    for (auto iter = cloudfilter110->begin(); iter != cloudfilter110->end(); iter++){
        MyColorPointType &pcl_p = (*iter);
        otherRMCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
    }

    //2、将点进行聚类,提取外包盒
    std::ifstream ifs_parseInfo(refFile);
    if(!ifs_parseInfo.is_open()){
        ifs_parseInfo.close();
        return;
    }

    nlohmann::json parseInfo;
    parseInfo<<ifs_parseInfo;
    std::vector<std::string> links = parseInfo["links"];

    std::vector<std::string> files;
    pcl::PointCloud<pcl::PointXYZI>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < links.size(); ++i) {
        std::string laneBoundaryFromSeg = dataDir+"/" + links[i] + "/laneBoundaryFromSeg";
        std::vector<std::string> file_names;
        Utils::getFiles(laneBoundaryFromSeg, file_names, [](std::string str)->bool{
            return boost::algorithm::ends_with(str, "_trjCloud.pcd");
        });

        for (auto filename : file_names) {
            std::string file_path = laneBoundaryFromSeg + "/" + filename;
            files.push_back(file_path);
        }
    }

    int n = 0;
    for(auto file : files){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(file, *cloud);
        for (auto iter = cloud->begin(); iter != cloud->end(); iter++)
        {
            refLaneBoundaryCloud->push_back(pcl::PointXYZI(iter->x, iter->y, iter->z, n));
        }
        n++;
    }

    std::cout<<"-------提取箭头------"<<std::endl;
    road_mark_base(roadArrowCloud, refLaneBoundaryCloud, outputDir, 100, 50, 3.0, 4.0, 0.5); //提取箭头
    // // 模板箭头
    // wq_road_mark_base(roadArrowCloud, refLaneBoundaryCloud, outputDir, 100, 50, 3.0, 4.0, 0.5); //提取箭头
    // 大模型箭头
    // std::string global_cloud_path = dataDir+"/../model_pcd/global_cloud.pcd"; 
    // std::string bev_path = dataDir+"/bev_obj"; 
    // std::string data_path = dataDir+"/bev_obj"; 
    // run_bev_mark(data_path, global_cloud_path, bev_path, outputDir, refLaneBoundaryCloud);
    std::cout<<"-------提取其他地面要素------"<<std::endl;
    road_mark_base(otherRMCloud, refLaneBoundaryCloud, outputDir, 110, 100, 2.0, 3.75, 1.0); //提取sign_line, slow_down_triangle, speed_sign, diamond, bicycle_sign, parking_line

    //处理有重叠区域的数据
    mix_diff_roadmark(outputDir);
}

void PostProcessRoadMark::run_yxx_bev_road(std::string dataDir,std::string fsd_mapbuildout, std::string pcdPath,std::string refFile,std::string outputDir)
{
    //datadir = 任务路径
    std::string mapping_output = dataDir+"/mapping_output";
    std::string bevInfoJsonFile = mapping_output + "/bev_info.json";
    std::cout<<"bevInfoJsonFile: "<<bevInfoJsonFile<<std::endl;
    std::ifstream ifs_bevInfo(bevInfoJsonFile);
    if(!ifs_bevInfo.is_open()){
        ifs_bevInfo.close();
        return;
    }
    nlohmann::json bevInfo;
    bevInfo<<ifs_bevInfo;
    float pixelSize = bevInfo["pixel_size_m"];
    Eigen::Matrix3d T_utm_bev = Eigen::Matrix3d::Identity();
    T_utm_bev<<bevInfo["T_utm_bev"][0][0], bevInfo["T_utm_bev"][0][1], bevInfo["T_utm_bev"][0][2],
            bevInfo["T_utm_bev"][1][0], bevInfo["T_utm_bev"][1][1], bevInfo["T_utm_bev"][1][2],
            bevInfo["T_utm_bev"][2][0], bevInfo["T_utm_bev"][2][1], bevInfo["T_utm_bev"][2][2];
    std::cout<<"T_utm_bev:\n"<<T_utm_bev<<std::endl;

    //get lane boundary
    std::ifstream ifs_parseInfo(refFile);
    if(!ifs_parseInfo.is_open()){
        ifs_parseInfo.close();
        return;
    }

    nlohmann::json parseInfo;
    parseInfo<<ifs_parseInfo;
    std::vector<std::string> links = parseInfo["links"];

    std::vector<double> p_utm_world = parseInfo["t_utm_world"];
    Eigen::Vector3d t_utm_world(p_utm_world[0], p_utm_world[1], p_utm_world[2]);

    std::vector<std::string> files;
    pcl::PointCloud<pcl::PointXYZI>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < links.size(); ++i) {
        std::string laneBoundaryFromSeg = fsd_mapbuildout+"/" + links[i] + "/laneBoundaryFromSeg";
        std::vector<std::string> file_names;
        Utils::getFiles(laneBoundaryFromSeg, file_names, [](std::string str)->bool{
            return boost::algorithm::ends_with(str, "_trjCloud.pcd");
        });

        for (auto filename : file_names) {
            std::string file_path = laneBoundaryFromSeg + "/" + filename;
            files.push_back(file_path);
        }
    }

    int n = 0;
    for(auto file : files){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(file, *cloud);
        for (auto iter = cloud->begin(); iter != cloud->end(); iter++)
        {
            refLaneBoundaryCloud->push_back(pcl::PointXYZI(iter->x, iter->y, iter->z, n));
        }
        n++;
    }


    //"line_id": 255, "arrow_id": 200, "road_id": 150, "boundary_id": 100
    std::string roadSegImgFile = mapping_output + "/bev_road_seg.png";
    cv::Mat img_bevRoadSeg = cv::imread(roadSegImgFile, cv::IMREAD_GRAYSCALE);
    std::cout << "img rows: " << img_bevRoadSeg.rows << " cols: " << img_bevRoadSeg.cols << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr roadArrowCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<img_bevRoadSeg.rows; i++){
        for(int j=0; j<img_bevRoadSeg.cols; j++){
            if(img_bevRoadSeg.at<uchar>(i,j)!=200) continue;
            Eigen::Vector3d utmPos(T_utm_bev(0,0)*j+T_utm_bev(0,1)*i+T_utm_bev(0,2),
                                   T_utm_bev(1,0)*j+T_utm_bev(1,1)*i+T_utm_bev(1,2), 0.f);

            Eigen::Vector3d utmoff = utmPos - t_utm_world;
            roadArrowCloud->push_back(pcl::PointXYZ(utmoff(0), utmoff(1), utmoff(2)));
        }
    }
    road_mark_base(roadArrowCloud, refLaneBoundaryCloud, outputDir, 200, 50, 1.0, 4.0, 0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr roadOtherCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<img_bevRoadSeg.rows; i++){
        for(int j=0; j<img_bevRoadSeg.cols; j++){
            if(img_bevRoadSeg.at<uchar>(i,j)!=170) continue;
            Eigen::Vector3d utmPos(T_utm_bev(0,0)*j+T_utm_bev(0,1)*i+T_utm_bev(0,2),
                                   T_utm_bev(1,0)*j+T_utm_bev(1,1)*i+T_utm_bev(1,2), 0.f);

            Eigen::Vector3d utmoff = utmPos - t_utm_world;
            roadOtherCloud->push_back(pcl::PointXYZ(utmoff(0), utmoff(1), utmoff(2)));
        }
    }
    road_mark_base(roadOtherCloud, refLaneBoundaryCloud, outputDir, 170, 50, 1.0, 4.0, 1.0);
}

void PostProcessRoadMark::run_yxx_fin_add_type(std::string dataDir,std::string pcdPath,std::string refFile,std::string outputDir)
{
    //1、加载点云
    std::string lidarFile = pcdPath;
    std::cout << "lidarFile: " << lidarFile << std::endl;
    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::io::loadPCDFile(lidarFile, *pc_ptr);

    //2、将点进行聚类,提取外包盒
    std::ifstream ifs_parseInfo(refFile);
    if(!ifs_parseInfo.is_open()){
        ifs_parseInfo.close();
        return;
    }

    nlohmann::json parseInfo;
    parseInfo<<ifs_parseInfo;
    std::vector<std::string> links = parseInfo["links"];

    std::vector<std::string> files;
    pcl::PointCloud<pcl::PointXYZI>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < links.size(); ++i) {
        std::string laneBoundaryFromSeg = dataDir+"/" + links[i] + "/laneBoundaryFromSeg";
        std::vector<std::string> file_names;
        Utils::getFiles(laneBoundaryFromSeg, file_names, [](std::string str)->bool{
            return boost::algorithm::ends_with(str, "_trjCloud.pcd");
        });

        for (auto filename : file_names) {
            std::string file_path = laneBoundaryFromSeg + "/" + filename;
            files.push_back(file_path);
        }
    }

    int n = 0;
    for(auto file : files){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(file, *cloud);
        for (auto iter = cloud->begin(); iter != cloud->end(); iter++)
        {
            refLaneBoundaryCloud->push_back(pcl::PointXYZI(iter->x, iter->y, iter->z, n));
        }
        n++;
    }

    std::cout<<"-------提取箭头------"<<std::endl;
    road_mark_add_type(pc_ptr, refLaneBoundaryCloud, outputDir, 100, 50, 3.0, 4.0, 0.5); //提取箭头

    // std::cout<<"-------提取其他地面要素------"<<std::endl;
    // road_mark_add_type(pc_ptr, refLaneBoundaryCloud, outputDir, 110, 100, 2.0, 3.75, 1.0); //提取sign_line, slow_down_triangle, speed_sign, diamond, bicycle_sign, parking_line

    // //处理有重叠区域的数据
    // mix_diff_roadmark(outputDir);
}

void PostProcessRoadMark::road_mark_base(const pcl::PointCloud<pcl::PointXYZ>::Ptr &roadArrowCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud, std::string outputDir, int label, int minPts, double minX, double maxY, double minY)
{
    int ele_type;
    std::string filename;
    if (label == 100) {
        ele_type = 3;
        filename = "object_arrow.pcd";
    } else if (label == 110) {
        ele_type = 4;
        filename = "object_other_mark.pcd";
    } else {
        // do nothing;
    }

    std::string labelstr = std::to_string(label);
    if(roadArrowCloud!=NULL && roadArrowCloud->size()>0){
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_roadmark.pcd", *roadArrowCloud);
    }

    pcl::PointCloud<PointElement>::Ptr other_mark_output(new pcl::PointCloud<PointElement>);
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_refLaneBoundary(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    std::map<int, std::vector<Eigen::Vector3f>> CornerPointMap;

    kdtree_refLaneBoundary->setInputCloud(refLaneBoundaryCloud); // 设置要搜索的点云，建立KDTree

    float radius = 0.15f;
    int minPtsSerch = 5;
    std::vector<std::vector<int>> clusters;
    Inference::DBSCAN(roadArrowCloud, radius, minPtsSerch, clusters);
    std::cout<<"extract clusters size: "<<clusters.size()<<std::endl;

    //测试输出聚类后数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < clusters.size(); ++i) {
        for(int j = 0; j < clusters[i].size();j++)
        {
            pcl::PointXYZ curPXYZ = roadArrowCloud->at(clusters[i][j]);
            clustersCloudWithI->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
        }
    }
    if (!clustersCloudWithI->empty())
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_roadmark_clustersCloud.pcd", *clustersCloudWithI);

    nlohmann::json obj;
    int ClusterNum = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(int index=0; index<clusters.size(); index++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudOri(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*roadArrowCloud, clusters[index], *clusterCloudOri);
//        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(index)+"_roadArrowCluster.pcd", *clusterCloudOri);

        //针对每个聚类进行去噪
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::StatisticalOutlierRemoval(clusterCloudOri, clusterCloud, 10, 2.0);
        // std::cout<<index<<" :cluster has points :"<<clusterCloud->size()<<std::endl;
        if(clusterCloud->size() < minPts){
            continue;
        }
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            centroid += clusterCloud->at(i).getVector3fMap();
        }
        centroid = centroid/clusterCloud->size();
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            cov += posDiff*posDiff.transpose();
        }
        cov = cov/clusterCloud->size();
        cov = (cov + cov.transpose())/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.compute(cov);
        Eigen::Vector3f eigenvalues = es.eigenvalues();
        std::cout<<index<<" th cluster, eigenvalues: "<<eigenvalues.transpose()<<std::endl;
        if(eigenvalues(0) > 1e-3 || eigenvalues(1) < 100*eigenvalues(0)){
            continue;
        }
        Eigen::Matrix3f U = es.eigenvectors();
        Eigen::Vector3f nz = U.col(0);
        if(nz.z() < 0.f) nz = -nz;
        std::cout<<"nz: "<<nz.transpose()<<std::endl;
        if(nz.z() < 0.7f){
            continue;
        }
        // determine heading direction
        std::vector<int> indices;
        std::vector<float> squaredDistances;
        pcl::PointXYZI searchPt(centroid.x(), centroid.y(), centroid.z(), 0);
        kdtree_refLaneBoundary->nearestKSearch(searchPt, 1, indices, squaredDistances);
        // std::cout<<"searched indices size: "<<indices.size()<<std::endl;
        if(indices.empty()){
            continue;
        }

        int refCount = refLaneBoundaryCloud->size();
        int indexS  = indices[0];
        int indexE = indexS + 1;
        if(indexE >= refCount)
        {
            indexE = indexS;
            indexS = indexS - 1;
        }
        auto int1 = int(refLaneBoundaryCloud->at(indexS).intensity);
        auto int2 = int(refLaneBoundaryCloud->at(indexE).intensity);
        if(int1 != int2)
        {
            indexE = indexS;
            indexS = indexS - 1;
        }

        Eigen::Vector3f direction = refLaneBoundaryCloud->at(indexE).getVector3fMap() -
                                    refLaneBoundaryCloud->at(indexS).getVector3fMap();
        std::cout<<"direction: "<<direction.transpose()<<std::endl;
        direction.normalize();
        Eigen::Vector3f nx = direction - direction.dot(nz)*nz;
        nx.normalize();
        Eigen::Vector3f ny = nz.cross(nx);
        float x_min = 1e6, x_max = -1e6, y_min = 1e6, y_max = -1e6;
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            float x = posDiff.dot(nx);
            if(x < x_min) x_min = x;
            if(x > x_max) x_max = x;
            float y = posDiff.dot(ny);
            if(y < y_min) y_min = y;
            if(y > y_max) y_max = y;
        }
        std::cout<<"x length: "<<x_max - x_min<<" y length: "<<y_max - y_min<<std::endl;
        if(x_max - x_min > 8.f || y_max - y_min > maxY || x_max - x_min < minX || y_max - y_min < minY){
            continue;
        }
        Eigen::Vector3f LeftFront = centroid + x_max*nx + y_max*ny;
        Eigen::Vector3f RightFront = centroid + x_max*nx + y_min*ny;
        Eigen::Vector3f LeftRear = centroid + x_min*nx + y_max*ny;
        Eigen::Vector3f RightRear = centroid + x_min*nx + y_min*ny;

        auto generateSegmentCloud = [&](Eigen::Vector3f p0, Eigen::Vector3f p1){
            Eigen::Vector3f dp = p1 - p0;
            float num = dp.norm()/0.1f;
            for(int i=0; i<num; i++){
                Eigen::Vector3f pos = p0 + (i/num)*dp;
                boxCloud->push_back(pcl::PointXYZI(pos(0), pos(1), pos(2), index));
            }
        };
        generateSegmentCloud(LeftFront, RightFront);
        generateSegmentCloud(RightFront, RightRear);
        generateSegmentCloud(RightRear, LeftRear);
        generateSegmentCloud(LeftRear, LeftFront);

        nlohmann::json singleTrafficArrow;
        singleTrafficArrow["id"] = index;
        singleTrafficArrow["LeftFront"] = {LeftFront.x(), LeftFront.y(), LeftFront.z()};
        singleTrafficArrow["RightFront"] = {RightFront.x(), RightFront.y(), RightFront.z()};
        singleTrafficArrow["RightRear"] = {RightRear.x(), RightRear.y(), RightRear.z()};
        singleTrafficArrow["LeftRear"] = {LeftRear.x(), LeftRear.y(), LeftRear.z()};
        obj[labelstr].push_back(singleTrafficArrow);

        CornerPointMap[ClusterNum].push_back(RightFront);
        CornerPointMap[ClusterNum].push_back(LeftFront);
        CornerPointMap[ClusterNum].push_back(LeftRear);
        CornerPointMap[ClusterNum].push_back(RightRear);
        ClusterNum += 1;
    }

    for (int dir_index = 0; dir_index < ClusterNum; dir_index++)
    {
        generateOutputCloud(dir_index, 0, 0, other_mark_output, CornerPointMap[dir_index], 0, ele_type);
    }

    if(other_mark_output!=NULL && other_mark_output->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + filename, *other_mark_output);
    }

    std::ofstream fid_trafficArrow(outputDir+"/" + labelstr + "_roadmark.json");
    //fid_trafficArrow << std::setw(4) << obj << std::endl;
    fid_trafficArrow<< obj << std::endl;
    if(boxCloud!=NULL && boxCloud->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_box.pcd", *boxCloud);
    }
}

// 获得聚类中点云最可能的形状
int PostProcessRoadMark::getClusterBevShape(const std::vector<int>& cluster, const std::map<int, int>& indexMap, const pcl::PointCloud<ConRmPointType>::Ptr& cloudfilter) 
{
    if (cluster.empty()) {
        return -1; 
    }

    std::map<int, int> frequencyMap;
    int mostFrequentValue = -1;
    int maxCount = 0;

    for (const auto& roadmarkIndex : cluster) {
        auto cloudfilterIndexIt = indexMap.find(roadmarkIndex);
        
        if (cloudfilterIndexIt != indexMap.end()) {
            int cloudfilterIndex = cloudfilterIndexIt->second;
            int cloud_bev_shape = cloudfilter->at(cloudfilterIndex).cloud_bev_shape;
            frequencyMap[cloud_bev_shape]++;
        }
    }

    for (const auto& pair : frequencyMap) {
        if (pair.second > maxCount) {
            maxCount = pair.second;
            mostFrequentValue = pair.first;
        }
    }
    return mostFrequentValue;
}


void PostProcessRoadMark::road_mark_add_type(const pcl::PointCloud<MyColorPointType>::Ptr &pcPtr, const pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud, std::string outputDir, int label, int minPts, double minX, double maxY, double minY)
{
    pcl::PointCloud<ConRmPointType>::Ptr cloudfilter(new pcl::PointCloud<ConRmPointType>);
    int ele_type;
    std::string filename;

    if (label == 100) {
        ele_type = 3;
        filename = "object_arrow.pcd";
        pclFilter::ConditionalRemoval(pcPtr, cloudfilter, "cloud_bev_label_1", 4);
    } else if (label == 110) {
        ele_type = 4;
        filename = "object_other_mark.pcd";
        pclFilter::ConditionalRemoval(pcPtr, cloudfilter, "cloud_bev_label_1", 5);
    } else {
        // do nothing
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr roadmarkCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::map<int, int> indexMap; // 用于记录roadmarkCloud中点的索引和cloudfilter中点的索引

    int cloudfilterIndex = 0; // cloudfilter的当前索引
    for (auto iter = cloudfilter->begin(); iter != cloudfilter->end(); iter++, cloudfilterIndex++){
        ConRmPointType &pcl_p = (*iter);
        roadmarkCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, 0));
        indexMap[roadmarkCloud->size()-1] = cloudfilterIndex;
    }

    std::string labelstr = std::to_string(label);
    if(roadmarkCloud!=NULL && roadmarkCloud->size()>0){
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_roadmark.pcd", *roadmarkCloud);
    }

    std::map<int, std::vector<Eigen::Vector3f>> CornerPointMap;
    std::map<int, int> cluster_bev_shape_map; // 用于记录每个聚类中点云的形状

    std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> lane_clouds_cache;
    for (const auto& pt : *refLaneBoundaryCloud) {
        const int line_id = static_cast<int>(pt.intensity);
        auto& cloud = lane_clouds_cache[line_id];
        if (!cloud) cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud->push_back(pcl::PointXYZ(pt.x, pt.y, 0));
    }

    float radius = 0.15f;
    int minPtsSerch = 5;
    std::vector<std::vector<int>> clusters;
    Inference::DBSCAN(roadmarkCloud, radius, minPtsSerch, clusters);
    std::cout<<"extract clusters size: "<<clusters.size()<<std::endl;

    //测试输出聚类后数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < clusters.size(); ++i) {
        for(int j = 0; j < clusters[i].size();j++)
        {
            pcl::PointXYZ curPXYZ = roadmarkCloud->at(clusters[i][j]);
            clustersCloudWithI->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
        }
    }
    if (!clustersCloudWithI->empty())
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_roadmark_clustersCloud.pcd", *clustersCloudWithI);

    nlohmann::json obj;
    int ClusterNum = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(int index=0; index<clusters.size(); index++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudOri(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*roadmarkCloud, clusters[index], *clusterCloudOri);
        // pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(index)+"_roadArrowCluster.pcd", *clusterCloudOri);

        // 获得聚类中点云最可能的形状
        int cluster_bev_shape = getClusterBevShape(clusters[index], indexMap, cloudfilter);
        std::cout<<"cluster_bev_shape: "<<cluster_bev_shape<<std::endl;

        //针对每个聚类进行去噪
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::StatisticalOutlierRemoval(clusterCloudOri, clusterCloud, 10, 2.0);
        // std::cout<<index<<" :cluster has points :"<<clusterCloud->size()<<std::endl;
        if(clusterCloud->size() < minPts){
            continue;
        }
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            centroid += clusterCloud->at(i).getVector3fMap();
        }
        centroid = centroid/clusterCloud->size();
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            cov += posDiff*posDiff.transpose();
        }
        cov = cov/clusterCloud->size();
        cov = (cov + cov.transpose())/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.compute(cov);
        Eigen::Vector3f eigenvalues = es.eigenvalues();
        if(eigenvalues(0) > 1e-3 || eigenvalues(1) < 50*eigenvalues(0)){
            std::cout<<index<<" th cluster, eigenvalues: "<<eigenvalues.transpose()<<std::endl;
            continue;
        }
        Eigen::Matrix3f U = es.eigenvectors();
        Eigen::Vector3f nz = U.col(0);
        if(nz.z() < 0.f) nz = -nz;
        std::cout<<"nz: "<<nz.transpose()<<std::endl;
        if(nz.z() < 0.7f){
            continue;
        }

        // determine heading direction
        pcl::PointXYZ target_point(centroid.x(), centroid.y(), centroid.z());
        auto nearby_result = findLinesWithDynamicThreshold(target_point, 4.0, 10.0, lane_clouds_cache);
        auto dir_result = filterLinesByDirection(nearby_result, 30.0f);
        Eigen::Vector3f direction = dir_result.main_direction;

        // int refCount = refLaneBoundaryCloud->size();
        // int indexS  = indices[0];
        // int indexE = indexS + 1;
        // if(indexE >= refCount)
        // {
        //     indexE = indexS;
        //     indexS = indexS - 1;
        // }
        // auto int1 = int(refLaneBoundaryCloud->at(indexS).intensity);
        // auto int2 = int(refLaneBoundaryCloud->at(indexE).intensity);
        // if(int1 != int2)
        // {
        //     indexE = indexS;
        //     indexS = indexS - 1;
        // }

        // Eigen::Vector3f direction = refLaneBoundaryCloud->at(indexE).getVector3fMap() -
        //                             refLaneBoundaryCloud->at(indexS).getVector3fMap();
        std::cout<<"direction: "<<direction.transpose()<<std::endl;
        direction.normalize();
        Eigen::Vector3f nx = direction - direction.dot(nz)*nz;
        nx.normalize();
        Eigen::Vector3f ny = nz.cross(nx);
        float x_min = 1e6, x_max = -1e6, y_min = 1e6, y_max = -1e6;
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            float x = posDiff.dot(nx);
            if(x < x_min) x_min = x;
            if(x > x_max) x_max = x;
            float y = posDiff.dot(ny);
            if(y < y_min) y_min = y;
            if(y > y_max) y_max = y;
        }
        std::cout<<"x length: "<<x_max - x_min<<" y length: "<<y_max - y_min<<std::endl;
        if(x_max - x_min > 8.f || y_max - y_min > maxY || x_max - x_min < minX || y_max - y_min < minY){
            continue;
        }
        Eigen::Vector3f LeftFront = centroid + x_max*nx + y_max*ny;
        Eigen::Vector3f RightFront = centroid + x_max*nx + y_min*ny;
        Eigen::Vector3f LeftRear = centroid + x_min*nx + y_max*ny;
        Eigen::Vector3f RightRear = centroid + x_min*nx + y_min*ny;

        auto generateSegmentCloud = [&](Eigen::Vector3f p0, Eigen::Vector3f p1){
            Eigen::Vector3f dp = p1 - p0;
            float num = dp.norm()/0.1f;
            for(int i=0; i<num; i++){
                Eigen::Vector3f pos = p0 + (i/num)*dp;
                boxCloud->push_back(pcl::PointXYZI(pos(0), pos(1), pos(2), index));
            }
        };
        generateSegmentCloud(LeftFront, RightFront);
        generateSegmentCloud(RightFront, RightRear);
        generateSegmentCloud(RightRear, LeftRear);
        generateSegmentCloud(LeftRear, LeftFront);

        nlohmann::json singleTrafficArrow;
        singleTrafficArrow["id"] = index;
        singleTrafficArrow["LeftFront"] = {LeftFront.x(), LeftFront.y(), LeftFront.z()};
        singleTrafficArrow["RightFront"] = {RightFront.x(), RightFront.y(), RightFront.z()};
        singleTrafficArrow["RightRear"] = {RightRear.x(), RightRear.y(), RightRear.z()};
        singleTrafficArrow["LeftRear"] = {LeftRear.x(), LeftRear.y(), LeftRear.z()};
        obj[labelstr].push_back(singleTrafficArrow);

        CornerPointMap[ClusterNum].push_back(RightFront);
        CornerPointMap[ClusterNum].push_back(LeftFront);
        CornerPointMap[ClusterNum].push_back(LeftRear);
        CornerPointMap[ClusterNum].push_back(RightRear);
        cluster_bev_shape_map[ClusterNum] = cluster_bev_shape;
        ClusterNum += 1;
    }

    // 打印 CornerPointMap
    // for (const auto& pair : CornerPointMap) {
    //     int clusterNum = pair.first;
    //     const std::vector<Eigen::Vector3f>& points = pair.second;
    //     std::cout << "before process Cluster [" << clusterNum << "] has " << points.size() << " corner points:" << std::endl;
    //     for (const auto& point : points) {
    //         std::cout << "  Point: " << point.transpose() << std::endl;
    //     }
    // }

    std::map<int, std::vector<Eigen::Vector3f>> newCornerPointMap;
    afterProcess(CornerPointMap, refLaneBoundaryCloud, newCornerPointMap);

    // 打印 newCornerPointMap
    // for (const auto& pair : newCornerPointMap) {
    //     int clusterNum = pair.first;
    //     const std::vector<Eigen::Vector3f>& points = pair.second;
    //     std::cout << "after process Cluster [" << clusterNum << "] has " << points.size() << " corner points:" << std::endl;
    //     for (const auto& point : points) {
    //         std::cout << "  Point: " << point.transpose() << std::endl;
    //     }
    // }

    pcl::PointCloud<PointElement>::Ptr road_mark_output(new pcl::PointCloud<PointElement>);
    for (int dir_index = 0; dir_index < ClusterNum; dir_index++)
    {
        generateOutputCloud(dir_index, 0, cluster_bev_shape_map[dir_index], road_mark_output, newCornerPointMap[dir_index], 0, ele_type);
    } 

    if(road_mark_output!=NULL && road_mark_output->size()>0) {
        pcl::io::savePCDFileBinary(outputDir+"/" + filename, *road_mark_output);
    }      

    // std::ofstream fid_trafficArrow(outputDir+"/" + labelstr + "_roadmark.json");
    // //fid_trafficArrow << std::setw(4) << obj << std::endl;
    // fid_trafficArrow<< obj << std::endl;
    // if(boxCloud!=NULL && boxCloud->size()>0)
    // {
    //     pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_box.pcd", *boxCloud);
    // }

}

void PostProcessRoadMark::roadmark_json2RoadMarkGeo(std::string outputDir, std::string labelstr, std::vector<RoadMarkGeo>& resRoadMarkGeo)
{
    std::string filename = outputDir +"/" + labelstr + "_roadmark.json";
    std::ifstream ifs_RM(filename);
    if (!ifs_RM.is_open()) {
        ifs_RM.close();
        std::cout<<"没有地面标识结果json文件:"<<filename<<std::endl;
        return;
    }

    nlohmann::json RMJson;
    RMJson<<ifs_RM;
    for (const auto &obj : RMJson[labelstr]) {
        RoadMarkGeo newRM;
        newRM.index = obj["id"];

        auto lf = obj["LeftFront"];
        newRM.plypts.push_back(Eigen::Vector3f(lf[0], lf[1], lf[2]));
        auto rf = obj["RightFront"];
        newRM.plypts.push_back(Eigen::Vector3f(rf[0], rf[1], rf[2]));
        auto rr = obj["RightRear"];
        newRM.plypts.push_back(Eigen::Vector3f(rr[0], rr[1], rr[2]));
        auto lr = obj["LeftRear"];
        newRM.plypts.push_back(Eigen::Vector3f(lr[0], lr[1], lr[2]));
        newRM.enve = Engine::Geometries::Envelope(lf[0], lf[1],rr[0], rr[1]);
        newRM.flag = false;
        resRoadMarkGeo.push_back(newRM);
    }
}
void PostProcessRoadMark::roadmark_RoadMarkGeo2json(std::string outputDir, std::string labelstr, std::vector<RoadMarkGeo>& resRoadMarkGeo)
{
    nlohmann::json obj;
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < resRoadMarkGeo.size(); ++i) {
        //重写json
        nlohmann::json singleTrafficArrow;
        singleTrafficArrow["id"] = resRoadMarkGeo[i].index;
        singleTrafficArrow["LeftFront"] = {resRoadMarkGeo[i].plypts[0].x(), resRoadMarkGeo[i].plypts[0].y(), resRoadMarkGeo[i].plypts[0].z()};
        singleTrafficArrow["RightFront"] = {resRoadMarkGeo[i].plypts[1].x(), resRoadMarkGeo[i].plypts[1].y(), resRoadMarkGeo[i].plypts[1].z()};
        singleTrafficArrow["RightRear"] = {resRoadMarkGeo[i].plypts[2].x(), resRoadMarkGeo[i].plypts[2].y(), resRoadMarkGeo[i].plypts[2].z()};
        singleTrafficArrow["LeftRear"] = {resRoadMarkGeo[i].plypts[3].x(), resRoadMarkGeo[i].plypts[3].y(), resRoadMarkGeo[i].plypts[3].z()};
        obj[labelstr].push_back(singleTrafficArrow);

        auto generateSegmentCloud = [&](Eigen::Vector3f p0, Eigen::Vector3f p1){
            Eigen::Vector3f dp = p1 - p0;
            float num = dp.norm()/0.1f;
            for(int m=0; m<num; m++){
                Eigen::Vector3f pos = p0 + (m/num)*dp;
                boxCloud->push_back(pcl::PointXYZI(pos(0), pos(1), pos(2), resRoadMarkGeo[i].index));
            }
        };
        generateSegmentCloud(resRoadMarkGeo[i].plypts[0], resRoadMarkGeo[i].plypts[1]);
        generateSegmentCloud(resRoadMarkGeo[i].plypts[1], resRoadMarkGeo[i].plypts[2]);
        generateSegmentCloud(resRoadMarkGeo[i].plypts[2], resRoadMarkGeo[i].plypts[3]);
        generateSegmentCloud(resRoadMarkGeo[i].plypts[3], resRoadMarkGeo[i].plypts[0]);
    }
    std::ofstream fid_trafficArrow(outputDir+"/" + labelstr + "_roadmark.json");
    //fid_trafficArrow << std::setw(4) << obj << std::endl;
    fid_trafficArrow<< obj << std::endl;
    if(boxCloud!=NULL && boxCloud->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_box_new.pcd", *boxCloud);
    }
}
void PostProcessRoadMark::recalroadmark(const pcl::PointCloud<pcl::PointXYZI>::Ptr &Cloud100, const pcl::PointCloud<pcl::PointXYZI>::Ptr &Cloud110, int index_100, vector<int> mix110, std::vector<RoadMarkGeo>& resRoadMarkGeo_100, std::vector<RoadMarkGeo>& resRoadMarkGeo_110)
{
    //得到100中的点
    pcl::PointCloud<pcl::PointXYZI>::Ptr fitercloud_100(new pcl::PointCloud<pcl::PointXYZI>);
    pclFilter::ConditionalRemoval(Cloud100, fitercloud_100, "i", resRoadMarkGeo_100[index_100].index);

    //得到110中的点
    for (int i = 0; i < mix110.size(); ++i)
        pclFilter::ConditionalRemoval(Cloud110, fitercloud_100, "i", resRoadMarkGeo_110[mix110[i]].index);

    //未完整 有待商榷
}

void PostProcessRoadMark::mix_diff_roadmark(std::string outputDir)
{
    //读取不同label的box
    std::vector<RoadMarkGeo> resRoadMarkGeo_100;
    roadmark_json2RoadMarkGeo(outputDir, "100", resRoadMarkGeo_100);

    std::vector<RoadMarkGeo> resRoadMarkGeo_110;
    roadmark_json2RoadMarkGeo(outputDir, "110", resRoadMarkGeo_110);

    if (resRoadMarkGeo_100.empty() || resRoadMarkGeo_110.empty())
        return;

    double tolerMix = 0.7;
    //求交计算，均在二维空间中进行
    int n100 = resRoadMarkGeo_100.size();
    for (int i = n100 - 1; i >= 0; --i)
    {
        RoadMarkGeo cur100 = resRoadMarkGeo_100[i];
        if (cur100.index == 14)
            int lll = 0;
        double cur100Area = cur100.enve.GetArea();
        vector<int> delete110; //记录需要删除的110数据
        vector<int> delete100; //记录需要删除的110数据
        vector<int> mix110; //记录需要融合的110数据
        for (int j = 0; j < resRoadMarkGeo_110.size(); ++j)
        {
            if (resRoadMarkGeo_110[j].index == 263)
                int llll = 0;
            Engine::Geometries::Envelope intersectEnv;
            if (cur100.enve.Intersection(resRoadMarkGeo_110[j].enve, intersectEnv))//判定相交
            {
                double radio = intersectEnv.GetArea() / cur100Area;
                if (radio < tolerMix)
                    delete110.push_back(j);
                else
                    delete100.push_back(i);
            }
        }

        if (!delete110.empty())
        {
            int n = delete110.size();
            for (int j = n - 1; j >= 0; --j)
                resRoadMarkGeo_110.erase(resRoadMarkGeo_110.begin() + delete110[j]);
        }

        if(!delete100.empty())
            resRoadMarkGeo_100.erase(resRoadMarkGeo_100.begin() + i);
    }
    roadmark_RoadMarkGeo2json(outputDir, "100", resRoadMarkGeo_100);
    roadmark_RoadMarkGeo2json(outputDir, "110", resRoadMarkGeo_110);
}
/////////////////////////////////////////wq  test ///////////////////////////////////////////////////////////////////////

void PostProcessRoadMark::wq_WriteToOBJ(Engine::Base::Array<Engine::Geometries::Coordinate> &vecLanes, std::string strTilePath, std::string name)
{
    string dirPath = strTilePath;
    if (dirPath.back() != '/' || dirPath.back() != '\\')
    {
        dirPath += "/";
    }

    string objFullPath = dirPath + name + ".obj";

    ofstream ofs(objFullPath);
    if (!ofs.is_open())
    {
        cout << "obj 保存不成功" << endl;
        return;
    }

    vector<hdmap_build::Vec3> lineVec;
    for (int i = 0; i < vecLanes.GetCount(); i++)
    {

        Engine::Geometries::Coordinate curCrood = vecLanes[i];
        hdmap_build::Vec3 coor;
        coor.x = curCrood.x;
        coor.y = curCrood.y;
        coor.z = curCrood.z;
        ofs << std::fixed;
        ofs << "v " << coor.x << " " << coor.y << " " << coor.z << endl;
        lineVec.push_back(coor);

    }

    int K = 0;

    if (lineVec.size() <= 1)
    {
        return ;
    }
    ofs << "l ";
    for (auto j = 0 ; j < lineVec.size(); j++)
    {
        ofs << ++K << " ";
    }
    ofs << endl;
    ofs.close();
    return;
}


void PostProcessRoadMark::wq_adjust_center(Engine::Base::Array<Engine::Geometries::Coordinate> &vecLanes, Eigen::Vector3f centroid, Engine::Base::Array<Engine::Geometries::Coordinate> &vecLanes_out)
{
    for (auto i = 0 ; i < vecLanes.GetCount(); i++){

        Engine::Geometries::Coordinate per_point = vecLanes[i];
        Engine::Geometries::Coordinate cur_point (per_point.x+centroid[0], per_point.y+centroid[1], per_point.z+centroid[2]);
        vecLanes_out.Add(cur_point);
    }
}

void PostProcessRoadMark::wq_getConcaveHull(Engine::Base::Array<Engine::Geometries::Coordinate>& clusterpts, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull)
    {
        if (clusterpts.IsEmpty())
            return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < clusterpts.GetCount(); ++i) {
            cloud_1_ptr->push_back(pcl::PointXYZ(clusterpts[i].x, clusterpts[i].y, clusterpts[i].z));
        }

        //求聚类点云的凹包
        pcl::ConcaveHull<pcl::PointXYZ> chull;  //创建多边形提取对象
        chull.setInputCloud (cloud_1_ptr);  //设置输入点云为投影后点云cloud_projected
        chull.setAlpha (0.08);                   //设置alpha值为0.1
        chull.reconstruct (*output);       //重建提取创建凹多边形
        cloud_hull = output;
    }

void PostProcessRoadMark::generateOutputCloud(int index, float yaw, int arrow_type, pcl::PointCloud<PointElement>::Ptr &outputCloud, std::vector<Eigen::Vector3f> &FourPointMap, float score, int ele_type)
    {
        // 右前 - 左前 - 左后 - 右后，逆时针
        PointElement rfper_point;
        rfper_point.x = FourPointMap[0][0];
        rfper_point.y = FourPointMap[0][1];
        rfper_point.z = FourPointMap[0][2];
        rfper_point.ele_type = ele_type;
        rfper_point.type1 = arrow_type;
        rfper_point.id = index;
        rfper_point.index = 0;
        rfper_point.heading = yaw;
        rfper_point.score = score;
        outputCloud->push_back(rfper_point);

        PointElement lfper_point;
        lfper_point.x = FourPointMap[1][0];
        lfper_point.y = FourPointMap[1][1];
        lfper_point.z = FourPointMap[1][2];
        lfper_point.ele_type = ele_type;
        lfper_point.type1 = arrow_type;
        lfper_point.id = index;
        lfper_point.index = 1;
        lfper_point.heading = yaw;
        lfper_point.score = score;
        outputCloud->push_back(lfper_point);

        PointElement lrper_point;
        lrper_point.x = FourPointMap[2][0];
        lrper_point.y = FourPointMap[2][1];
        lrper_point.z = FourPointMap[2][2];
        lrper_point.ele_type = ele_type;
        lrper_point.type1 = arrow_type;
        lrper_point.id = index;
        lrper_point.index = 2;
        lrper_point.heading = yaw;
        lrper_point.score = score;
        outputCloud->push_back(lrper_point);

        PointElement rrper_point;
        rrper_point.x = FourPointMap[3][0];
        rrper_point.y = FourPointMap[3][1];
        rrper_point.z = FourPointMap[3][2];
        rrper_point.ele_type = ele_type;
        rrper_point.type1 = arrow_type;
        rrper_point.id = index;
        rrper_point.index = 3;
        rrper_point.heading = yaw;
        rrper_point.score = score;
        outputCloud->push_back(rrper_point);
    }

void PostProcessRoadMark::wq_fix_edge_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr &clusterCloud ,pcl::PointCloud<pcl::PointXYZ>::Ptr &sampleCloud)
    {
        if(clusterCloud==NULL)
            return;
        Engine::Base::Array<Engine::Geometries::Coordinate> interPts ;
        std::vector<int> ori_pts_index;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr sampleCloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f per_cloud =  clusterCloud->at(i).getVector3fMap();
            Engine::Geometries::Coordinate per_point (per_cloud[0], per_cloud[1], per_cloud[2]);
            interPts.Add(per_point);
        }
        

        wq_getConcaveHull(interPts, sampleCloud);
        if(sampleCloud!=NULL&& sampleCloud->size()>0)
        {
            std::cout<<"size is " <<sampleCloud->size()<<std::endl;
        }

        // box.Clean();
        // for(int i=0; i<sampleCloud->size(); i++){
        //     Eigen::Vector3f per_cloud =  clusterCloud->at(i).getVector3fMap();
        //     Engine::Geometries::Coordinate per_point (per_cloud[0], per_cloud[1], per_cloud[2]);
        //     box.Add(per_point);
        // }

        
    }

double PostProcessRoadMark::interior_ratio(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtr_src, 
                                           const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtr_ref,
                                           double max_range = 0.15) {
    std::vector<double> distance_list;

    int k_NN = 1;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_ref;
    std::vector<int> kIdxSearch(k_NN);
    std::vector<float> kSquaredDist(k_NN);
    kdtree_ref.setInputCloud(cloudPtr_ref);
    int nr = 0;
    for (size_t i = 0; i < cloudPtr_src->size(); ++i) {
        auto search_Pt = cloudPtr_src->points[i];
        if (kdtree_ref.nearestKSearch(search_Pt, k_NN, kIdxSearch, kSquaredDist) > 0 &&
            kSquaredDist.front() < max_range*max_range) {
            distance_list.push_back(kSquaredDist.front());
        }
    }
    int interior_num  = distance_list.size();
    double ratio = double(interior_num) / cloudPtr_src->size();
    return ratio;
}




bool PostProcessRoadMark::getInterpolationLine(Engine::Base::Array<Engine::Geometries::Coordinate> &coordinates, std::vector<int>& ori_pts_index, double interval, int save_ori_pts)
{
    if (coordinates.GetCount() < 2)
    {
        return false;
    }

    double totalLength = 0.0f;
    for (int i = 0; i < coordinates.GetCount() - 1; i++)
    {
        totalLength += coordinates[i].Distance(coordinates[i + 1]);
    }

    if (totalLength <= 0.0f)
    {
        return false;
    }

    double length;
    float thresholld = interval;
    Engine::Geometries::Coordinate startPoint, endPoint, point;
    Engine::Base::Array<Engine::Geometries::Coordinate> preLine = coordinates;
    Engine::Base::Array<Engine::Geometries::Coordinate> newLine;
    if(save_ori_pts)
    {
        newLine.Add(coordinates[0]);
        ori_pts_index.push_back(0);
        for (int i = 0; i < preLine.GetCount() - 1;i++)
        {
            startPoint = preLine[i];
            endPoint = preLine[i + 1];
            length = startPoint.Distance(endPoint);
            if (length < interval)
            {
                newLine.Add(endPoint);
                ori_pts_index.push_back(newLine.GetCount() - 1);
                continue;
            }
            int radio = int(length / interval);
            double dis = 0.0;
            for (int j = 1; j <= radio; ++j)
            {
                dis = j * interval;
                if (fabs(length - dis) < 0.25 * interval)
                    continue;
                point.x = startPoint.x + (endPoint.x - startPoint.x)*(dis) / length;
                point.y = startPoint.y + (endPoint.y - startPoint.y)*(dis) / length;
                point.z = startPoint.z + (endPoint.z - startPoint.z)*(dis) / length;
                newLine.Add(point);
            }
            newLine.Add(endPoint);
            ori_pts_index.push_back(newLine.GetCount() - 1);
        }
    }
    else
    {
        newLine.Add(coordinates[0]);
        for (int i = 0; i < preLine.GetCount() - 1; )
        {
            startPoint = preLine[i];
            endPoint = preLine[i + 1];
            length = startPoint.Distance(endPoint);
            if (length < thresholld)
            {
                thresholld -= length;
                preLine.Delete(i);
                continue;
            }

            point.x = startPoint.x + (endPoint.x - startPoint.x)*(thresholld) / length;
            point.y = startPoint.y + (endPoint.y - startPoint.y)*(thresholld) / length;
            point.z = startPoint.z + (endPoint.z - startPoint.z)*(thresholld) / length;
            newLine.Add(point);
            preLine[i] = point;
            thresholld = interval;
        }
        if (newLine[newLine.GetCount() - 1].Distance(coordinates[coordinates.GetCount() - 1]) > 1.0E-5)
        {
            newLine.Add(coordinates[coordinates.GetCount() - 1]);
        }
        else
        {
            newLine[newLine.GetCount() - 1] = coordinates[coordinates.GetCount() - 1];
        }
    }

    coordinates = newLine;
    return true;
}

void PostProcessRoadMark::rotate_yaw(Engine::Base::Array<Engine::Geometries::Coordinate> &input, Engine::Base::Array<Engine::Geometries::Coordinate> &output, double yaw) {
    // 正数为逆时针，负数为顺时针
    // double radian = yaw;
    // //  
    // Eigen::AngleAxisd angle_axis(radian, Eigen::Vector3d(0, 0, 1));
    // // Eigen::Vector3f src ,tar;
    // for (int i = 0; i < input.GetCount();i++) {
    //     Eigen::Vector3d src(input[i].x , input[i].y , input[i].z);
    //     Eigen::Vector3d tar = angle_axis.matrix() * src;
    //     output.Add(Engine::Geometries::Coordinate(tar[0], tar[1], tar[2]));
    // }
    // return ;
    Eigen::Matrix3d rotation;
    rotation << cos(yaw), -sin(yaw), 0,
                sin(yaw), cos(yaw), 0,
                0, 0, 1;
    for (int i = 0; i < input.GetCount();i++) {
        Eigen::Vector3d src(input[i].x , input[i].y , input[i].z);
        Eigen::Vector3d tar = rotation * src;
        output.Add(Engine::Geometries::Coordinate(tar[0], tar[1], tar[2]));
    }
    return ;
}

double PostProcessRoadMark::calu_yaw(Engine::Geometries::Coordinate& v1, Engine::Geometries::Coordinate& v2) 
{
    Engine::Geometries::Coordinate vec_1 (v1.x, v1.y, 0);
    Engine::Geometries::Coordinate vec_2 (v2.x, v2.y, 0);

    double dot = v1.x * v2.x + v1.y * v2.y;
    double det = v1.x * v2.y - v1.y * v2.x;
	double d  =( dot )/ std::sqrt((v1.x*v1.x + v1.y*v1.y)*(v2.x*v2.x + v2.y*v2.y));
    double theta = std::acos(d);
    if(det > 0 && dot > 0) return theta ; 
    if(det > 0 && dot < 0) return theta ; 
    if(det < 0 && dot < 0) return -theta ;
    if(det < 0 && dot > 0) return -theta ;
}

double PostProcessRoadMark::calu_yaw(Eigen::Vector3f& v1, Eigen::Vector3f& v2) 
{
    Eigen::Vector3f vec_1 (v1[0], v1[1], 0);
    Eigen::Vector3f vec_2 (v2[0], v2[1], 0);

    double dot = v1[0] * v2[0] + v1[1] * v2[1];
    double det = v1[0] * v2[1] - v1[1] * v2[0];
	double d  =( dot )/ std::sqrt((v1[0]*v1[0] + v1[1]*v1[1])*(v2[0]*v2[0] + v2[1]*v2[1]));
    double theta = std::acos(d);
    if(det > 0 && dot > 0) return theta ; 
    if(det > 0 && dot < 0) return theta ; 
    if(det < 0 && dot < 0) return -theta ;
    if(det < 0 && dot > 0) return -theta ;
}


void PostProcessRoadMark::wq_icp_type(  std::string outputDir,
                                        Eigen::Vector3f direction,
                                        Eigen::Vector3f centroid,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr &clusterCloud,
                                        Engine::Base::Array<Engine::Geometries::Coordinate> &arrow_edgePts,
                                        std::string arrow_type ,
                                        std::map<std::string, double> &arrow_icpPts,
                                        int flag)
{    
    Engine::Base::Array<Engine::Geometries::Coordinate> arrow_edgePts_out;
    Engine::Base::Array<Engine::Geometries::Coordinate> arrow_edgePts_adj;
    Engine::Base::Array<Engine::Geometries::Coordinate> clusterCloudEdgePts;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudEdge(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr BoxEdge(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr BoxEdge_icp(new pcl::PointCloud<pcl::PointXYZ>);
    // Engine::Base::Array<Engine::Geometries::Coordinate> arrow_edgePts;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (flag == 1){
        direction[0] = -direction[0];
        direction[1] = -direction[1];
        direction[2] = -direction[2];
    }

    arrow_type = arrow_type + "_" + std::to_string(flag);

    wq_WriteToOBJ(arrow_edgePts, outputDir, arrow_type);
    Engine::Geometries::Coordinate arrow_forward_dir (1, 0, 0);
    Engine::Geometries::Coordinate cloud_forward_dir (direction[0], direction[1], direction[2]);
    double yaw = calu_yaw(arrow_forward_dir ,cloud_forward_dir);
    // double yaw = arrow_forward_dir.AngleWith(cloud_forward_dir) ;
    std::cout <<"wq arrow  "<< arrow_forward_dir.x <<" " <<arrow_forward_dir.y <<" "<< arrow_forward_dir.z <<std::endl;
    std::cout <<"wq box_dir "<<direction <<std::endl;
    std::cout <<"wq yaw  "<<yaw <<std::endl;
    rotate_yaw(arrow_edgePts, arrow_edgePts_adj,yaw);
    wq_WriteToOBJ(arrow_edgePts_adj, outputDir, arrow_type+ "_adj");
    
    wq_adjust_center(arrow_edgePts_adj, centroid, arrow_edgePts_out);
    wq_WriteToOBJ(arrow_edgePts_out, outputDir, arrow_type+"_out");
    std::vector<int> ori_pts_index;
    getInterpolationLine(arrow_edgePts_out, ori_pts_index, 0.04,1);
    for(int i=0; i<arrow_edgePts_out.GetCount(); i++){
        Engine::Geometries::Coordinate per_point = arrow_edgePts_out[i];
        BoxEdge->push_back(pcl::PointXYZ(per_point.x, per_point.y, per_point.z));
    }

    //聚类边缘提取
    wq_fix_edge_icp(clusterCloud, clusterCloudEdge);
    if(clusterCloudEdge!=NULL&& clusterCloudEdge->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + arrow_type +"clustercloud.pcd", *clusterCloudEdge);
    }

    wq_WriteToOBJ(arrow_edgePts_out, outputDir, arrow_type+"_out_chazhi");

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputTarget(clusterCloudEdge); //参照点
    icp.setInputSource(BoxEdge); //待移动点
    icp.setMaxCorrespondenceDistance(0.1);
    // // 设置最大迭代次数(默认 1)
    icp.setMaximumIterations(300);
    // // 设置 两次变换矩阵之间的差值 (默认 2)
    icp.setTransformationEpsilon(1e-6);
    icp.setRANSACOutlierRejectionThreshold(0.5);
    // // 设置 均方误差(默认 3)
    icp.setEuclideanFitnessEpsilon(3);
    icp.setUseReciprocalCorrespondences(1);
    icp.align(*BoxEdge_icp);

    double ratio = interior_ratio( clusterCloudEdge, BoxEdge_icp);
    std::cout << "Estimated ratio is "<<ratio << std::endl;

    if(BoxEdge_icp->size()>0)
    {   
        std::cout << "icp success "<< std::endl;
        pcl::io::savePCDFileBinary(outputDir+"/" + arrow_type + "box_icp.pcd", *BoxEdge_icp);
        arrow_icpPts[arrow_type] = ratio;
    }
    else{
        std::cout << "icp failed "<< std::endl;
        arrow_icpPts[arrow_type] = 0;
    }


}



void PostProcessRoadMark::wq_road_mark_base(const pcl::PointCloud<pcl::PointXYZ>::Ptr &roadArrowCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud, std::string outputDir, int label, int minPts, double minX, double maxY, double minY)
{
    std::string labelstr = std::to_string(100);
    if(roadArrowCloud!=NULL && roadArrowCloud->size()>0){
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_roadmark.pcd", *roadArrowCloud);
    }

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_refLaneBoundary(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree_refLaneBoundary->setInputCloud(refLaneBoundaryCloud); // 设置要搜索的点云，建立KDTree

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_Center(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr CenterList(new pcl::PointCloud<pcl::PointXYZI>);
    //聚类后点云
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > ClusterList;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr clusterList(new pcl::PointCloud<pcl::PointXYZI>);
    //index dir   默认0  1左  2直行  3右  4直行左转   5直行右转
    std::map<int, int> CenterDirMap;
    std::map<int, Eigen::Vector3f> DirVecMap;
    std::map<int, Eigen::Vector3f> LeftVecMap;
    std::map<int, Eigen::Vector3f> CenterMap;
    std::map<int, std::vector<Eigen::Vector3f>> CornerPointMap;
    std::map<int, double> ArrowScoreMap;

    float radius = 0.15f;
    int minPtsSerch = 5;
    std::vector<std::vector<int>> clusters;
    Inference::DBSCAN(roadArrowCloud, radius, minPtsSerch, clusters);
    std::cout<<"extract clusters size: "<<clusters.size()<<std::endl;

    //测试输出聚类后数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < clusters.size(); ++i) {
        for(int j = 0; j < clusters[i].size();j++)
        {
            pcl::PointXYZ curPXYZ = roadArrowCloud->at(clusters[i][j]);
            clustersCloudWithI->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
        }
    }
    if (!clustersCloudWithI->empty())
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_roadmark_clustersCloud.pcd", *clustersCloudWithI);

    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudForward(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudLeft(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudRight(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudForwardLeft(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudForwardRight(new pcl::PointCloud<pcl::PointXYZ>);
    //储存所有的arrow信息
    std::vector<std::vector<std::pair<std::string, double>>> all_arrow_icp_point;

    nlohmann::json obj;
    int ClusterNum = 0 ;

    Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> forwardleftedgePts_list;

    pcl::PointCloud<pcl::PointXYZI>::Ptr boxCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointElement>::Ptr Arrow_output(new pcl::PointCloud<PointElement>);

    for(int index=0; index<clusters.size(); index++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudOri(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*roadArrowCloud, clusters[index], *clusterCloudOri);
        // pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(index)+"_roadArrowCluster.pcd", *clusterCloudOri);

        //针对每个聚类进行去噪
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::StatisticalOutlierRemoval(clusterCloudOri, clusterCloud, 10, 2.0);
        // std::cout<<index<<" :cluster has points :"<<clusterCloud->size()<<std::endl;
        if(clusterCloud->size() < minPts){
            continue;
        }
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            centroid += clusterCloud->at(i).getVector3fMap();
        }
        centroid = centroid/clusterCloud->size();
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            cov += posDiff*posDiff.transpose();
        }
        cov = cov/clusterCloud->size();
        cov = (cov + cov.transpose())/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.compute(cov);
        Eigen::Vector3f eigenvalues = es.eigenvalues();
        std::cout<<index<<" th cluster, eigenvalues: "<<eigenvalues.transpose()<<std::endl;
        if(eigenvalues(0) > 1e-3 || eigenvalues(1) < 100*eigenvalues(0)){
            continue;
        }
        //倾斜？？
        Eigen::Matrix3f U = es.eigenvectors();
        Eigen::Vector3f nz = U.col(0);
        if(nz.z() < 0.f) {
        nz = -nz;
        }
        std::cout<<"nz: "<<nz.transpose()<<std::endl;
        if(nz.z() < 0.7f){
            continue;
        }

        //determine heading direction
        std::vector<int> indices;
        std::vector<float> squaredDistances;
        pcl::PointXYZI searchPt(centroid.x(), centroid.y(), centroid.z(), 0);
        kdtree_refLaneBoundary->nearestKSearch(searchPt, 1, indices, squaredDistances);
        if(indices.empty()){
            continue;
        }

        int refCount = refLaneBoundaryCloud->size();
        int indexS  = indices[0];
        int indexE = indexS + 1;
        if(indexE >= refCount)
        {
            indexE = indexS;
            indexS = indexS - 1;
        }
        auto int1 = int(refLaneBoundaryCloud->at(indexS).intensity);
        auto int2 = int(refLaneBoundaryCloud->at(indexE).intensity);
        if(int1 != int2)
        {
            indexE = indexS;
            indexS = indexS - 1;
        }

        Eigen::Vector3f direction = refLaneBoundaryCloud->at(indexE).getVector3fMap() -
                                    refLaneBoundaryCloud->at(indexS).getVector3fMap();
        std::cout<<"direction: "<<direction.transpose()<<std::endl;
        direction.normalize();
        std::cout<<"norm direction: "<<direction.transpose()<<std::endl;
        // Eigen::Vector3f nx = direction - direction.dot(nz)*nz;
        // nx.normalize();
        Eigen::Vector3f nx = direction ;
        Eigen::Vector3f ny = nz.cross(nx);
        float x_min = 1e6, x_max = -1e6, y_min = 1e6, y_max = -1e6;
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            float x = posDiff.dot(nx);
            if(x < x_min) x_min = x;
            if(x > x_max) x_max = x;
            float y = posDiff.dot(ny);
            if(y < y_min) y_min = y;
            if(y > y_max) y_max = y;
        }
        std::cout<<"x length: "<<x_max - x_min<<" y length: "<<y_max - y_min<<std::endl;
        if(x_max - x_min > 10.f || y_max - y_min > maxY || x_max - x_min < minX || y_max - y_min < minY){
            continue;
        }
        Eigen::Vector3f LeftFront = centroid + x_max*nx + y_max*ny;
        Eigen::Vector3f RightFront = centroid + x_max*nx + y_min*ny;
        Eigen::Vector3f LeftRear = centroid + x_min*nx + y_max*ny;
        Eigen::Vector3f RightRear = centroid + x_min*nx + y_min*ny;

        //包络框 icp
        float forward_box [8][3] = {{-3, -0.15, 0},{0.6, -0.15, 0},{0.6, -0.45, 0},{3,0,0},{0.6,0.45,0},{0.6,0.15,0},{-3,0.15,0},{-3, -0.15, 0}};
        float leftbox [10][3] = {{-0.8, -0.4, 0},{-4, -0.4, 0},{-4, -0.7, 0},{-0.1,-0.7,0},{1,0.4,0},{2,0.4,0},{0.4,0.8,0},{-1.1, 0.4, 0},{-0.1,0.4,0},{-0.8,-0.4,0}};
        float rightbox [10][3] = {{-0.8, 0.4, 0},{-4, 0.4, 0},{-4, 0.7, 0},{-0.1,0.7,0},{1,-0.4,0},{2,-0.4,0},{0.4,-0.8,0},{-1.1, -0.4, 0},{-0.1,-0.4,0},{-0.8,0.4,0}};
        float forwardleftbox [15][3] = {{-2.6, -0.4, 0},{1, -0.4, 0},{1, -0.7, 0},{3.4,-0.25,0},{1,0.2,0},{1,-0.1,0},{-1,-0.1,0},{-0.2,0.7,0},{0.8,0.7,0},{-0.8,1.1,0},{-2.3,0.7,0},{-1.4,0.7,0},{-2.2,-0.1,0},{-2.6,-0.1,0},{-2.6, -0.4, 0}};
        float forwardrightbox [15][3] = {{-2.6, 0.4, 0},{1, 0.4, 0},{1, 0.7, 0},{3.4,0.25,0},{1,-0.2,0},{1,0.1,0},{-1,0.1,0},{-0.2,-0.7,0},{0.8,-0.7,0},{-0.8,-1.1,0},{-2.3,-0.7,0},{-1.4,-0.7,0},{-2.2,0.1,0},{-2.6,0.1,0},{-2.6, 0.4, 0}};

        Engine::Base::Array<Engine::Geometries::Coordinate> forwardedgePts;
        Engine::Base::Array<Engine::Geometries::Coordinate> leftedgePts;
        Engine::Base::Array<Engine::Geometries::Coordinate> rightedgePts;
        Engine::Base::Array<Engine::Geometries::Coordinate> forwardrightedgePts;
        Engine::Base::Array<Engine::Geometries::Coordinate> forwardleftedgePts;

        for(int i = 0; i <8; i++){
            Engine::Geometries::Coordinate newCrood(forward_box[i][0], forward_box[i][1], forward_box[i][2]);
            forwardedgePts.Add(newCrood);
        }
        for(int i = 0; i <10; i++){
            Engine::Geometries::Coordinate newCrood(leftbox[i][0], leftbox[i][1], leftbox[i][2]);
            leftedgePts.Add(newCrood);
        }
        for(int i = 0; i <10; i++){
            Engine::Geometries::Coordinate newCrood(rightbox[i][0], rightbox[i][1], rightbox[i][2]);
            rightedgePts.Add(newCrood);
        }
        for(int i = 0; i <15; i++){
            Engine::Geometries::Coordinate newCrood(forwardrightbox[i][0], forwardrightbox[i][1], forwardrightbox[i][2]);
            forwardrightedgePts.Add(newCrood);
        }
        for(int i = 0; i <15; i++){
            Engine::Geometries::Coordinate newCrood(forwardleftbox[i][0], forwardleftbox[i][1], forwardleftbox[i][2]);
            forwardleftedgePts.Add(newCrood);
        }

        // Engine::Base::Array<Engine::Geometries::Coordinate> &arrow_edgePts
        std::map<std::string, double> arrow_icpPts;
        wq_icp_type(outputDir,direction,centroid, clusterCloud, forwardedgePts ,std::to_string(index)+"_"+"forward", arrow_icpPts,0);
        wq_icp_type(outputDir,direction,centroid, clusterCloud, leftedgePts ,std::to_string(index)+"_""left", arrow_icpPts,0);
        wq_icp_type(outputDir,direction,centroid, clusterCloud, rightedgePts ,std::to_string(index)+"_""right", arrow_icpPts,0);
        wq_icp_type(outputDir,direction,centroid, clusterCloud, forwardrightedgePts ,std::to_string(index)+"_""forward_right", arrow_icpPts,0);
        wq_icp_type(outputDir,direction,centroid, clusterCloud, forwardleftedgePts ,std::to_string(index)+"_""forward_left", arrow_icpPts,0);
        wq_icp_type(outputDir,direction,centroid, clusterCloud, forwardedgePts ,std::to_string(index)+"_""forward", arrow_icpPts,1);
        wq_icp_type(outputDir,direction,centroid, clusterCloud, leftedgePts ,std::to_string(index)+"_""left", arrow_icpPts,1);
        wq_icp_type(outputDir,direction,centroid, clusterCloud, rightedgePts ,std::to_string(index)+"_""right", arrow_icpPts,1);
        wq_icp_type(outputDir,direction,centroid, clusterCloud, forwardrightedgePts ,std::to_string(index)+"_""forward_right", arrow_icpPts,1);
        wq_icp_type(outputDir,direction,centroid, clusterCloud, forwardleftedgePts ,std::to_string(index)+"_""forward_left", arrow_icpPts,1);

        for (auto it :arrow_icpPts){
            std::cout << "arrow icp pts" << it.first <<" "<<it.second <<std::endl;
        }

        std::vector<std::pair<std::string, double>> arrow_icp_point(arrow_icpPts.begin(), arrow_icpPts.end());
        std::sort(arrow_icp_point.begin(), arrow_icp_point.end(), [] (auto& a, auto& b)  {return a.second >= b.second;});
        std::cout << "arrow icp point is " <<arrow_icp_point[0].first <<  std::endl;
        all_arrow_icp_point.push_back(arrow_icp_point);
        
        //写入centerlist 
        CenterDirMap[ClusterNum] = 0;
        ClusterList[ClusterNum] = clusterCloud;
        CenterList->push_back(pcl::PointXYZI(centroid.x(), centroid.y(), centroid.z(),index));
        //方向判断：
        string type_name = arrow_icp_point[0].first;
        double arrow_score = arrow_icp_point[0].second;
        std::cout << "wq output arrow type " << type_name << std::endl;

        if(type_name == "forward_0" || type_name == "forward_1" ){
            CenterDirMap[ClusterNum] = 2;
            if(type_name == "forward_0")
            {
                Eigen::Vector3f arrow_dir(direction[0], direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
            else
            {   
                Eigen::Vector3f arrow_dir(-direction[0], -direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
        }
        if(type_name == "left_0" || type_name == "left_1" ){
            CenterDirMap[ClusterNum] = 1;
            if(type_name == "left_0")
            {
                Eigen::Vector3f arrow_dir(direction[0], direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
            else
            {   
                Eigen::Vector3f arrow_dir(-direction[0], -direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
        }
        if(type_name == "right_0" || type_name == "right_1" ){
            CenterDirMap[ClusterNum] = 3;
            if(type_name == "right_0")
            {
                Eigen::Vector3f arrow_dir(direction[0], direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
            else
            {   
                Eigen::Vector3f arrow_dir(-direction[0], -direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
        }
        if(type_name == "forward_left_0" || type_name == "forward_left_1" ){
            CenterDirMap[ClusterNum] = 4;
            if(type_name == "forward_left_0")
            {
                Eigen::Vector3f arrow_dir(direction[0], direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
            else
            {   
                Eigen::Vector3f arrow_dir(-direction[0], -direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
        }
        if(type_name == "forward_right_0" || type_name == "forward_right_1" ){
            CenterDirMap[ClusterNum] = 5;
            if(type_name == "forward_right_0")
            {
                Eigen::Vector3f arrow_dir(direction[0], direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
            else
            {   
                Eigen::Vector3f arrow_dir(-direction[0], -direction[1], 0);
                DirVecMap[ClusterNum] = arrow_dir;
            }
        }

        auto generateSegmentCloud = [&](Eigen::Vector3f p0, Eigen::Vector3f p1){
            Eigen::Vector3f dp = p1 - p0;
            float num = dp.norm()/0.1f;
            for(int i=0; i<num; i++){
                Eigen::Vector3f pos = p0 + (i/num)*dp;
                boxCloud->push_back(pcl::PointXYZI(pos(0), pos(1), pos(2), index));
            }
        };
        generateSegmentCloud(LeftFront, RightFront);
        generateSegmentCloud(RightFront, RightRear);
        generateSegmentCloud(RightRear, LeftRear);
        generateSegmentCloud(LeftRear, LeftFront);

        nlohmann::json singleTrafficArrow;
        singleTrafficArrow["id"] = index;
        singleTrafficArrow["ArrowType"] = CenterDirMap[ClusterNum];
        singleTrafficArrow["LeftFront"] = {LeftFront.x(), LeftFront.y(), LeftFront.z()};
        singleTrafficArrow["RightFront"] = {RightFront.x(), RightFront.y(), RightFront.z()};
        singleTrafficArrow["RightRear"] = {RightRear.x(), RightRear.y(), RightRear.z()};
        singleTrafficArrow["LeftRear"] = {LeftRear.x(), LeftRear.y(), LeftRear.z()};
        singleTrafficArrow["changed"] = 0;
        singleTrafficArrow["ArrowCenter"] = {centroid.x(), centroid.y(), centroid.z()};
        singleTrafficArrow["ArrowDir"] = {DirVecMap[ClusterNum][0], DirVecMap[ClusterNum][1], 0};
        obj[labelstr].push_back(singleTrafficArrow);
        CenterMap[ClusterNum] = centroid;
        CornerPointMap[ClusterNum].push_back(RightFront);
        CornerPointMap[ClusterNum].push_back(LeftFront);
        CornerPointMap[ClusterNum].push_back(LeftRear);
        CornerPointMap[ClusterNum].push_back(RightRear);
        ArrowScoreMap[ClusterNum] = arrow_score;
        ClusterNum += 1;
    }

    //pca 中心判断左右逻辑   用来修正错误箭头
    kdtree_Center->setInputCloud(CenterList);
    for (int dir_index = 0; dir_index < ClusterNum; dir_index++)
    {
        //找出非直行
        if (CenterDirMap[dir_index] != 2)
        {
            std::vector<int> indices;
            pcl::PointXYZI curPoint = CenterList->at(dir_index);
            std::vector<float> squaredDistances;
            kdtree_Center->radiusSearch(curPoint, 6 ,indices, squaredDistances);
            if(indices.empty()){
                continue;
            }
            for (int i = 0; i < indices.size(); i++) 
            {
                int cur_index = indices[i];
                if(CenterDirMap[cur_index] == 2)
                {   
                    //直行
                    Eigen::Vector3f pos = CenterList->at(cur_index).getVector3fMap();
                    Eigen::Vector3f cur = CenterList->at(dir_index).getVector3fMap();
                    Eigen::Vector3f pos_cur = cur - pos;
                    Eigen::Vector3f dir = DirVecMap[cur_index];
                    Eigen::Vector3f z_dir (0, 0 ,1);
                    float cur_dist = pos_cur.dot(z_dir.cross(dir));

                    if (cur_dist > 0 && CenterDirMap[dir_index] == 3) 
                    {
                        CenterDirMap[dir_index] = 4;
                        obj[labelstr][dir_index]["changed"] = 1;
                        DirVecMap[dir_index] = -DirVecMap[dir_index];
                    }
                    else if (cur_dist > 0 && CenterDirMap[dir_index] == 5) 
                    {
                        CenterDirMap[dir_index] = 1;
                        obj[labelstr][dir_index]["changed"] = 1;
                        DirVecMap[dir_index] = -DirVecMap[dir_index];
                    }
                    else if (cur_dist < 0 && CenterDirMap[dir_index] == 4) 
                    {
                        CenterDirMap[dir_index] = 3;
                        obj[labelstr][dir_index]["changed"] = 1;
                        DirVecMap[dir_index] = -DirVecMap[dir_index];
                    }
                    else if (cur_dist < 0 && CenterDirMap[dir_index] == 1)
                    {
                        CenterDirMap[dir_index] = 5;
                        obj[labelstr][dir_index]["changed"] = 1;
                        DirVecMap[dir_index] = -DirVecMap[dir_index];
                    }
                    break;
                }
            }
        }

    }

    for (int dir_index = 0; dir_index < ClusterNum; dir_index++)
    {
        obj[labelstr][dir_index]["ArrowType"] = CenterDirMap[dir_index];
        Eigen::Vector3f forwards_dir (1, 0 ,0);
        double yaw = calu_yaw(forwards_dir, DirVecMap[dir_index]);
        generateOutputCloud(dir_index, yaw, CenterDirMap[dir_index], Arrow_output, CornerPointMap[dir_index], ArrowScoreMap[dir_index], 3);

    }

    for (int dir_index = 0; dir_index < CenterDirMap.size(); dir_index++){
        if (CenterDirMap[dir_index] == 1){
            for(int i=0; i < ClusterList[dir_index]->size(); i++){
                pcl::PointXYZ curPXYZ = ClusterList[dir_index]->at(i);
                clusterCloudLeft->push_back(pcl::PointXYZ(curPXYZ.x, curPXYZ.y, curPXYZ.z));         
            }
        }
        else if (CenterDirMap[dir_index] == 2){
            for(int i=0; i < ClusterList[dir_index]->size(); i++){
                pcl::PointXYZ curPXYZ = ClusterList[dir_index]->at(i);
                clusterCloudForward->push_back(pcl::PointXYZ(curPXYZ.x, curPXYZ.y, curPXYZ.z));
            }
        }
        else if (CenterDirMap[dir_index] == 3){
            for(int i=0; i < ClusterList[dir_index]->size(); i++){
                pcl::PointXYZ curPXYZ = ClusterList[dir_index]->at(i);
                clusterCloudRight->push_back(pcl::PointXYZ(curPXYZ.x, curPXYZ.y, curPXYZ.z));
            }
        }
        else if (CenterDirMap[dir_index] == 4){
            for(int i=0; i < ClusterList[dir_index]->size(); i++){
                pcl::PointXYZ curPXYZ = ClusterList[dir_index]->at(i);
                clusterCloudForwardLeft->push_back(pcl::PointXYZ(curPXYZ.x, curPXYZ.y, curPXYZ.z));
            }
        }
        else if (CenterDirMap[dir_index] == 5){
            for(int i=0; i < ClusterList[dir_index]->size(); i++){
                pcl::PointXYZ curPXYZ = ClusterList[dir_index]->at(i);
                clusterCloudForwardRight->push_back(pcl::PointXYZ(curPXYZ.x, curPXYZ.y, curPXYZ.z));
            }
        }

    }


    std::ofstream fid_trafficArrow(outputDir+"/" + labelstr + "_roadmark.json");
    //fid_trafficArrow << std::setw(4) << obj << std::endl;
    fid_trafficArrow<< obj << std::endl;

    std::ofstream fid_trafficArrow_wq(outputDir+"/" + labelstr + "_roadmark_dir.json");
    //fid_trafficArrow << std::setw(4) << obj << std::endl;
    fid_trafficArrow_wq<< obj << std::endl;
    if(boxCloud!=NULL && boxCloud->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_box.pcd", *boxCloud);
    }

    if(clusterCloudForward!=NULL && clusterCloudForward->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_forward.pcd", *clusterCloudForward);
    }

    if(clusterCloudLeft!=NULL && clusterCloudLeft->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_left.pcd", *clusterCloudLeft);
    }

    if(clusterCloudRight!=NULL && clusterCloudRight->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_right.pcd", *clusterCloudRight);
    }

    if(clusterCloudForwardLeft!=NULL && clusterCloudForwardLeft->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_forward_left.pcd", *clusterCloudForwardLeft);
    }

    if(clusterCloudForwardRight!=NULL && clusterCloudForwardRight->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + labelstr + "_forward_right.pcd", *clusterCloudForwardRight);
    }

    if(Arrow_output!=NULL && Arrow_output->size()>0)
    {
        pcl::io::savePCDFileBinary(outputDir+"/" + "object_arrow.pcd", *Arrow_output);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PostProcessRoadMark::run_bev_mark(std::string dataDir,std::string global_pcd_path,std::string bev_obj_dir,std::string outputDir, pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud)
{
        _output = outputDir;
        _VoxelGrid_size = 0.2f;
        std::cout << "开始处理点云数据" << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(global_pcd_path, *pc_ptr);
        if (pc_ptr->points.empty())
        {
            std::cout<<"点云文件无数据：global_pcd_path:"<<global_pcd_path<<std::endl;
            return;
        }

        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter(new pcl::PointCloud<MyColorPointType>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ori_bevlukoucloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_bev_label_1", 4);
        for (auto iter = cloudfilter->begin(); iter != cloudfilter->end(); iter++) {
            MyColorPointType &pcl_p = (*iter);
            ori_bevlukoucloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }
        if (ori_bevlukoucloud->empty() || ori_bevlukoucloud->points.empty())
        {
            std::cout<<"无路口bev_label点云："<<global_pcd_path<<std::endl;
            return;
        }
        pcl::io::savePCDFileBinary(outputDir+"/ori_cloud_bev_label_arrow.pcd",*ori_bevlukoucloud);

        //对点云进行体素滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::VoxelGridSampling(ori_bevlukoucloud, cloudVoxelGrid, _VoxelGrid_size);
        pcl::io::savePCDFileBinary(outputDir+"/ori_cloud_bev_label_mark_Voxel.pcd",*cloudVoxelGrid);

        ///////////////////bev数据////////////////////////
        std::cout << "开始处理bev数据" << std::endl;
        if(!boost::filesystem::exists(bev_obj_dir))
        {
            return;
        }

        Engine::Base::Array<BevPolygon> ori_bev_lukou_polygons;
        bev_lukou_obj(bev_obj_dir, outputDir, ori_bev_lukou_polygons);
        if(ori_bev_lukou_polygons.IsEmpty())
        {
            std::cout << "bev数据为空" << std::endl;
            return;
        }

        //为每个多边形打分
        cal_bev_polygon_core(ori_bev_lukou_polygons, cloudVoxelGrid, refLaneBoundaryCloud, 0.50);

        //filter多余多边形
        filter_bev_polygons();

        //传出对应pcd框
        polygon_to_pcd(outputDir);
}
void PostProcessRoadMark::polygon_to_pcd(std::string outputDir)
    {   
        pcl::PointCloud<PointElement>::Ptr Arrow_output(new pcl::PointCloud<PointElement>);

        for (int j = 0; j < _bev_lukou_bds_polys.GetCount(); j++)
        {   
            auto it = _bev_lukou_bds_polys[j];
            Engine::Geometries::Coordinate middle_pt;
            GetAveragePt(it.polygon_pts, middle_pt);
            Eigen::Vector3f centroid (middle_pt.x, middle_pt.y, middle_pt.z);
            auto direction = it.polygon_pts[1] - it.polygon_pts[2];
            Eigen::Vector3f dir (direction.x, direction.y, direction.z);
            Eigen::Vector3f nz (0, 0, 1) ;
            dir.normalize();
            Eigen::Vector3f nx = dir ;
            Eigen::Vector3f ny = nz.cross(nx);
            float x_max = 3.25;
            float x_min = -3.25;
            float y_max = 0.8;
            float y_min = -0.8;

            int arrow_type = it.subtype;
            if (arrow_type == 1||arrow_type == 3||arrow_type ==8||arrow_type ==9||arrow_type==10||arrow_type==11){
                y_max = 0.8;
                y_min = -0.8;
            }
            else if (arrow_type == 2){
                y_max = 0.5;
                y_min = -0.5;
            }
            else if (arrow_type == 4||arrow_type ==5){
                y_max = 0.9;
                y_min = -0.9;
            }
            else if (arrow_type == 7){
                y_max = 1.15;
                y_min = -1.15;
            }
            else{
                y_max = 1.2;
                y_min = -1.2;
            }
            Eigen::Vector3f LeftFront = centroid + x_max*nx + y_max*ny;
            Eigen::Vector3f RightFront = centroid + x_max*nx + y_min*ny;
            Eigen::Vector3f LeftRear = centroid + x_min*nx + y_max*ny;
            Eigen::Vector3f RightRear = centroid + x_min*nx + y_min*ny;

            Engine::Base::Array<Engine::Geometries::Coordinate> new_polygon_pts; 
            Engine::Geometries::Coordinate p0(RightFront[0], RightFront[1], RightFront[2]);
            Engine::Geometries::Coordinate p1(LeftFront[0], LeftFront[1], LeftFront[2]);
            Engine::Geometries::Coordinate p2(LeftRear[0], LeftRear[1], LeftRear[2]);
            Engine::Geometries::Coordinate p3(RightRear[0], RightRear[1], RightRear[2]);

            new_polygon_pts.Add(p0);
            new_polygon_pts.Add(p1);
            new_polygon_pts.Add(p2);
            new_polygon_pts.Add(p3);
            
            for (int i = 0 ; i < new_polygon_pts.GetCount(); i++)
            {
                Engine::Geometries::Coordinate per_point = new_polygon_pts[i];
                PointElement pcd_point;
                pcd_point.x = per_point.x;
                pcd_point.y = per_point.y;
                pcd_point.z = per_point.z;
                pcd_point.ele_type = 3;
                pcd_point.type1 = it.subtype;
                pcd_point.id = j;
                pcd_point.index = i;
                pcd_point.heading = 0;
                pcd_point.score = 0;
                Arrow_output->push_back(pcd_point);
            }
        }
        if(Arrow_output!=NULL && Arrow_output->size()>0)
        {
            pcl::io::savePCDFileBinary(outputDir+"/" + "object_arrow.pcd", *Arrow_output);
        }
    }

    void PostProcessRoadMark::pts_geo_polygon(const Engine::Base::Array<Engine::Geometries::Coordinate>& plyCoords, Engine::Geometries::Polygon& resPolygon)
    {
        Engine::Base::Array<Engine::Geometries::Coordinate*>* newCoords = new Engine::Base::Array<Engine::Geometries::Coordinate*>();
        for (int i = 0; i < plyCoords.GetCount(); ++i)
        {
            Engine::Geometries::Coordinate* newPoint = new Engine::Geometries::Coordinate();
            newPoint->x = plyCoords[i].x;
            newPoint->y = plyCoords[i].y;
            newPoint->z = plyCoords[i].z;
            newCoords->Add(newPoint);
        }

        Engine::Geometries::LinearRing* shellLinearRing = new Engine::Geometries::LinearRing(newCoords);
        // if(!shellLinearRing->IsClosed())
        // {
        //     shellLinearRing->GetCoordinates()->Add(shellLinearRing->GetStartPoint());
        // }
        Engine::Geometries::Polygon polygon(shellLinearRing);
        resPolygon = polygon;
    }

    void PostProcessRoadMark::get_Envelope(const Engine::Base::Array<Engine::Geometries::Coordinate>& arrPoints, Engine::Geometries::Envelope& envelope)
    {
        if(arrPoints.IsEmpty())
        {
            return;
        }
        Engine::Base::Double dMinX, dMaxX, dMinY, dMaxY;
        dMinX = arrPoints[0].x;
        dMaxX = arrPoints[0].x;
        dMinY = arrPoints[0].y;
        dMaxY = arrPoints[0].y;

        for (int i = 1; i < arrPoints.GetCount(); i++)
        {
            if (arrPoints[i].x < dMinX)
            {
                dMinX = arrPoints[i].x;
            }
            else if (arrPoints[i].x > dMaxX)
            {
                dMaxX = arrPoints[i].x;
            }

            if (arrPoints[i].y < dMinY)
            {
                dMinY = arrPoints[i].y;
            }
            else if (arrPoints[i].y > dMaxY)
            {
                dMaxY = arrPoints[i].y;
            }
        }

        envelope.SetMinX(dMinX);
        envelope.SetMaxX(dMaxX);
        envelope.SetMinY(dMinY);
        envelope.SetMaxY(dMaxY);
    }

void WriteToOBJ(Array<Array<Coordinate>> &vecLanes, std::string strTilePath, std::string name, bool utm)
{
    string dirPath = strTilePath;
    if (dirPath.back() != '/' || dirPath.back() != '\\')
    {
        dirPath += "/";
    }

    string objFullPath = dirPath + name + ".obj";

    ofstream ofs(objFullPath);
    if (!ofs.is_open())
    {
        cout << "obj 保存不成功" << endl;
        return;
    }

    vector<vector<hdmap_build::Vec3>> lineVec;
    for (int i = 0; i < vecLanes.GetCount(); i++)
    {
        vector<hdmap_build::Vec3> curVec;
        for (auto j = 0; j < vecLanes[i].GetCount(); j++)
        {
            Coordinate curCrood = vecLanes[i][j];
            hdmap_build::Vec3 coor;
            coor.x = curCrood.x;
            coor.y = curCrood.y;
            coor.z = curCrood.z;

            if(!utm)
            {
                coor.x = coor.x * DEG_TO_RAD;
                coor.y = coor.y * DEG_TO_RAD;

                projPJ  g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
                projPJ  g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
                pj_transform(g_pWGS84, g_utm, 1, 1, &coor.x, &coor.y, &coor.z);
            }
            ofs << std::fixed;
            ofs << "v " << coor.x << " " << coor.y << " " << coor.z << endl;
            curVec.push_back(coor);
        }
        lineVec.push_back(curVec);
    }
    int K = 0;
    for (auto i = 0; i < lineVec.size(); i++)
    {
        if (lineVec[i].size() <= 1)
        {
            continue;
        }
        ofs << "l ";
        for (auto j = 0 ; j < lineVec[i].size(); j++)
        {
            ofs << ++K << " ";
        }
        ofs << endl;
    }

    ofs.close();
    return;
}
    void PostProcessRoadMark::cal_bev_polygon_core(Engine::Base::Array<BevPolygon>& bev_crosswalks_polygons, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_hull, pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud, double toler)
    {
        std::mutex mtx;
        std::cout << "------>对bev每个数据打分" << std::endl;

        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_refLaneBoundary(new pcl::KdTreeFLANN<pcl::PointXYZI>);
        kdtree_refLaneBoundary->setInputCloud(refLaneBoundaryCloud); // 设置要搜索的点云，建立KDTree
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_xy(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
        coefficients->values.resize (4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0;
        // 创建滤波器对象
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud_hull);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected_xy);
        
        // pcl::io::savePCDFileBinary(_output+"/hull_cloud_bev_label_crosswalk_xy.pcd",*cloud_projected_xy);
        //生成边缘点检测kdtree
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        kdtree_cloud->setInputCloud(cloud_projected_xy); // 设置要搜索的点云，建立KDTree
        double piexl_area = _VoxelGrid_size * _VoxelGrid_size;

#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
        for (int i = 0; i < bev_crosswalks_polygons.GetCount(); i++)
        {
             // std::cout<<i<<std::endl;
             if(i % 10000 == 0)
            {
                double ndeal = double(i) / double(bev_crosswalks_polygons.GetCount());

                std::cout << "处理进度：" << int(ndeal * 100) << "%, " << i << " / " << bev_crosswalks_polygons.GetCount() << std::endl;
            }

            // if(bev_crosswalks_polygons[i].polygon_pts.GetCount() < 4)
            // {
            //     continue;
            // }

            Engine::Geometries::Coordinate middle_pt;
            GetAveragePt(bev_crosswalks_polygons[i].polygon_pts, middle_pt);
            Eigen::Vector3f centroid (middle_pt.x, middle_pt.y, middle_pt.z);
            
            //输出方向
            std::vector<int> d_indices;
            std::vector<float> d_squaredDistances;
            pcl::PointXYZI searchPt(centroid[0], centroid[1], centroid[2], 0);
            kdtree_refLaneBoundary->nearestKSearch(searchPt, 1, d_indices, d_squaredDistances);
            if(d_indices.empty()){
                continue;
            }
            int refCount = refLaneBoundaryCloud->size();
            int indexS  = d_indices[0];
            int indexE = indexS + 1;
            if(indexE >= refCount)
            {
                indexE = indexS;
                indexS = indexS - 1;
            }
            auto int1 = int(refLaneBoundaryCloud->at(indexS).intensity);
            auto int2 = int(refLaneBoundaryCloud->at(indexE).intensity);
            if(int1 != int2)
            {
                indexE = indexS;
                indexS = indexS - 1;
            }

            Eigen::Vector3f direction = refLaneBoundaryCloud->at(indexE).getVector3fMap() -
                                        refLaneBoundaryCloud->at(indexS).getVector3fMap();
            Eigen::Vector3f nz (0, 0, 1) ;
            direction.normalize();
            Eigen::Vector3f nx = direction ;
            Eigen::Vector3f ny = nz.cross(nx);
            float x_max = 3.25;
            float x_min = -3.25;
            float y_max = 1.2;
            float y_min = -1.2;

            Eigen::Vector3f LeftFront = centroid + x_max*nx + y_max*ny;
            Eigen::Vector3f RightFront = centroid + x_max*nx + y_min*ny;
            Eigen::Vector3f LeftRear = centroid + x_min*nx + y_max*ny;
            Eigen::Vector3f RightRear = centroid + x_min*nx + y_min*ny;

            bev_crosswalks_polygons[i].polygon_pts.Clear();
            Engine::Geometries::Coordinate p0(RightFront[0], RightFront[1], RightFront[2]);
            Engine::Geometries::Coordinate p1(LeftFront[0], LeftFront[1], LeftFront[2]);
            Engine::Geometries::Coordinate p2(LeftRear[0], LeftRear[1], LeftRear[2]);
            Engine::Geometries::Coordinate p3(RightRear[0], RightRear[1], RightRear[2]);

            bev_crosswalks_polygons[i].polygon_pts.Add(p0);
            bev_crosswalks_polygons[i].polygon_pts.Add(p1);
            bev_crosswalks_polygons[i].polygon_pts.Add(p2);
            bev_crosswalks_polygons[i].polygon_pts.Add(p3);
            bev_crosswalks_polygons[i].polygon_pts.Add(p0);

            
            //面积最小为3 最大为25
            double area = hdmap_build::CommonUtil::ComputePolygonArea(bev_crosswalks_polygons[i].polygon_pts);
            // if(area < 3 || area > 18.0)
            // if(area < 3 || area > 40.0)
            // {
            //     continue;
            // }

            //在中心点30米范围内检索点,最小个数需满足 toler * area / 体素面积
            std::vector<int> k_indices;
            std::vector<float> k_distances;
            pcl::PointXYZ newPt(middle_pt.x, middle_pt.y, 0.0);
            kdtree_cloud->radiusSearch(newPt, 10.0, k_indices, k_distances);
            
            double all_num = area / piexl_area;
            double minPts = toler * all_num;

            if(k_indices.size() < minPts)
            {
                continue;
            }

            int matchnum = 0;
            Engine::Geometries::Envelope newEnvelope;
            get_Envelope(bev_crosswalks_polygons[i].polygon_pts, newEnvelope);
            Engine::Geometries::Polygon bev_obj_polygon;
            pts_geo_polygon(bev_crosswalks_polygons[i].polygon_pts, bev_obj_polygon);
            for (int j = 0; j < k_indices.size(); j++)
            {
                auto& pt = cloud_projected_xy->points[k_indices[j]];
                Engine::Geometries::Coordinate curPt(pt.x, pt.y, pt.z);
                if(newEnvelope.Intersects(curPt)) //粗匹配
                {
                    if(Engine::Geometries::GeometryAlgorithm::PtInPolygon(&bev_obj_polygon, &curPt))
                    {
                        matchnum++;
                    }
                }
            }
            
            if(matchnum > minPts)
            {
                double score = matchnum / all_num;
                // std::cout<<score<< " ";
                BevPolygon new_bev_crosswalk;
                new_bev_crosswalk = bev_crosswalks_polygons[i];
                new_bev_crosswalk.isSave = true;
                new_bev_crosswalk.poly_enve = newEnvelope;
                new_bev_crosswalk.area = area;
                new_bev_crosswalk.score = score;
                new_bev_crosswalk.num_crosswalk_pts_in = matchnum;
                new_bev_crosswalk.middlept = middle_pt;
                // std::lock_guard<std::mutex> lck(mtx);
                _bev_lukou_bds_polys.Add(new_bev_crosswalk);
            }
        }
    
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> bev_save_polygons;
        for (int i = 0; i < _bev_lukou_bds_polys.GetCount(); i++)
        {
            _bev_lukou_bds_polys[i].index = i + 1;
            // std::cout<< _bev_lukou_bds_polys[i].index << ":score-" << _bev_lukou_bds_polys[i].score << ",  ";
            bev_save_polygons.Add(_bev_lukou_bds_polys[i].polygon_pts);
        } 
        
        // WriteToOBJ(bev_save_polygons, _output, "score_filter_bev_arrow" + std::to_string(int(toler*100)), true); 
    }

    void PostProcessRoadMark::filter_bev_polygons()
    {
        std::cout << "------>根据分数去重" << std::endl;
        for (int i = 0; i < _bev_lukou_bds_polys.GetCount(); i++)
        {
            if(!_bev_lukou_bds_polys[i].isSave)
            {
                continue;
            }
            for (int j = 0; j < _bev_lukou_bds_polys.GetCount(); j++)
            {
                if(!_bev_lukou_bds_polys[j].isSave)
                {
                    continue;
                }
                if(_bev_lukou_bds_polys[i].index == _bev_lukou_bds_polys[j].index)
                {
                    continue;
                }
                Engine::Geometries::Envelope Intersec;
                bool isOverlap = _bev_lukou_bds_polys[i].poly_enve.Intersection(_bev_lukou_bds_polys[j].poly_enve, Intersec);
                if(!isOverlap)
                {
                    continue;
                }
                
                // if(_bev_lukou_bds_polys[i].index == 101 && _bev_lukou_bds_polys[j].index == 108)
                // {
                //     std::cout << "101 : poly_enve GetMaxX GetMaxY GetMinX GetMinY"  << _bev_lukou_bds_polys[i].poly_enve.GetMaxX() << _bev_lukou_bds_polys[i].poly_enve.GetMaxY() << _bev_lukou_bds_polys[i].poly_enve.GetMinX()<< _bev_lukou_bds_polys[i].poly_enve.GetMinY()<< std::endl;
                //     std::cout << "108 : poly_enve GetMaxX GetMaxY GetMinX GetMinY"  << _bev_lukou_bds_polys[j].poly_enve.GetMaxX() << _bev_lukou_bds_polys[j].poly_enve.GetMaxY() << _bev_lukou_bds_polys[j].poly_enve.GetMinX()<< _bev_lukou_bds_polys[j].poly_enve.GetMinY()<< std::endl;
                //     std::cout << "Intersec : poly_enve GetMaxX GetMaxY GetMinX GetMinY"  << Intersec.GetMaxX() << Intersec.GetMaxY() << Intersec.GetMinX()<< Intersec.GetMinY()<< std::endl;
                // }
                
                double overlap1 = Intersec.GetArea() / _bev_lukou_bds_polys[i].area;
                double overlap2 = Intersec.GetArea() / _bev_lukou_bds_polys[j].area;
                if(overlap1 > 0.1 || overlap2 > 0.1)
                {
                    int n1 = _bev_lukou_bds_polys[i].num_crosswalk_pts_in;
                    int n2 = _bev_lukou_bds_polys[j].num_crosswalk_pts_in;
                    
                    //判断包含点数在相差不大的时候 以score为主 否则以点数为主
                    int diff = fabs(n1 - n2);
                    double diffRadio1 = double(diff) / double(n1);
                    double diffRadio2 = double(diff) / double(n2);
                    if(fabs(diffRadio1 - diffRadio2) < 0.1)
                    {
                        //看哪个包含了更多的点
                        if(_bev_lukou_bds_polys[j].score > _bev_lukou_bds_polys[i].score)
                        {
                            // std::cout<< _bev_lukou_bds_polys[i].index << "败于" << _bev_lukou_bds_polys[j].index<<",  ";
                            _bev_lukou_bds_polys[i].isSave = false;
                            break;
                        }
                        else
                        {
                            // std::cout<< _bev_lukou_bds_polys[i].index << "胜于" << _bev_lukou_bds_polys[j].index<<",  ";
                            _bev_lukou_bds_polys[j].isSave = false;
                        }
                    }
                    else
                    {
                        //看哪个包含了更多的点
                        if(n2 > n1)
                        {
                            // std::cout<< _bev_lukou_bds_polys[i].index << "败于" << _bev_lukou_bds_polys[j].index<<",  ";
                            _bev_lukou_bds_polys[i].isSave = false;
                            break;
                        }
                        else
                        {
                            // std::cout<< _bev_lukou_bds_polys[i].index << "胜于" << _bev_lukou_bds_polys[j].index<<",  ";
                            _bev_lukou_bds_polys[j].isSave = false;
                        }
                    }

                    /////////////////////////////////////////只是分数方案//////////////////////////////////////////////
                    // if(_bev_lukou_bds_polys[j].score > _bev_lukou_bds_polys[i].score)
                    // {
                    //     // std::cout<< _bev_lukou_bds_polys[i].index << "败于" << _bev_lukou_bds_polys[j].index<<",  ";
                    //     _bev_lukou_bds_polys[i].isSave = false;
                    //     break;
                    // }
                    // else
                    // {
                    //     // std::cout<< _bev_lukou_bds_polys[i].index << "胜于" << _bev_lukou_bds_polys[j].index<<",  ";
                    //     _bev_lukou_bds_polys[j].isSave = false;
                    // }
                }
            }
        } 
        int n = _bev_lukou_bds_polys.GetCount();
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> bev_save_polygons;
        for (int i = n - 1; i >= 0 ; i--)
        {
            if(!_bev_lukou_bds_polys[i].isSave)
            {
                _bev_lukou_bds_polys.Delete(i);
            }
            else
            {
                bev_save_polygons.Add(_bev_lukou_bds_polys[i].polygon_pts);
                _bev_lukou_bds_polys[i].index = bev_save_polygons.GetCount();
            }
        } 
        // hdmap_build::CommonUtil::WriteToOBJ(bev_save_polygons, _output, "lukou", true);  
    }

    void PostProcessRoadMark::bev_lukou_obj(const std::string bev_obj_floder, const std::string out_obj_floder,Engine::Base::Array<BevPolygon>& bev_lukou_polygons)
    {
        if(!boost::filesystem::exists(bev_obj_floder))
        {
            return;
        }

        Engine::Base::Array<BevPolygon> ori_bev_lukou_polygons;

        boost::filesystem::path path(bev_obj_floder);
        for (const auto& iter : boost::filesystem::directory_iterator(path)) 
        {
            if (boost::filesystem::is_directory(iter.path()))
            {
                std::string bev_lukou_pcd = iter.path().string()+"/arrow_bev.pcd";
                if(boost::filesystem::exists(bev_lukou_pcd))
                {
                    std::cout<<"读取bev arrow:"<<bev_lukou_pcd<<std::endl;
                    ReadObjFromPcd(bev_lukou_pcd, ori_bev_lukou_polygons);
                }
            }
        }
        if(!ori_bev_lukou_polygons.IsEmpty())
        {
            for (int i = 0; i < ori_bev_lukou_polygons.GetCount(); i++)
            {
                int n = ori_bev_lukou_polygons[i].polygon_pts.GetCount();
                if(ori_bev_lukou_polygons[i].polygon_pts.GetCount() < 3)
                {
                    continue;
                }
                
                //将多边形进行闭合
                Engine::Geometries::Coordinate s = ori_bev_lukou_polygons[i].polygon_pts[0];
                Engine::Geometries::Coordinate e = ori_bev_lukou_polygons[i].polygon_pts[n - 1];
                if(s.DistanceXY(e) > 0.15)
                {
                    ori_bev_lukou_polygons[i].polygon_pts.Add(s);
                }
                else
                {
                    ori_bev_lukou_polygons[i].polygon_pts[n - 1] = ori_bev_lukou_polygons[i].polygon_pts[0];
                }

                bev_lukou_polygons.Add(ori_bev_lukou_polygons[i]);
            }
            
            // hdmap_build::CommonUtil::WriteToOBJ(bev_lukou_polygons, out_obj_floder, "bev_utm_lukou_polygon", true);
        }
    }

    void PostProcessRoadMark::ReadObjFromPcd(std::string bev_lukou_pcd, Engine::Base::Array<BevPolygon> &arrArrPts)
    {
        if(!boost::filesystem::exists(bev_lukou_pcd))
        {
            std::cout<<"文件不存在:"<<bev_lukou_pcd<<std::endl;
            return;//检查文件的存在与否
        }
            
        pcl::PointCloud<PointElement>::Ptr pc_ptr(new pcl::PointCloud<PointElement>);
        pcl::io::loadPCDFile(bev_lukou_pcd, *pc_ptr);

        std::map<int, pcl::PointCloud<PointElement>> line_map;
        for (auto& pt : pc_ptr->points)
        {
            line_map[pt.id].points.push_back(pt);
        }
        for(auto& line : line_map)
        {
            if(line.second.points.size() > 3)
            {
                BevPolygon newBevPolygon;

                for(auto& it : line.second.points){
                    newBevPolygon.polygon_pts.Add(Engine::Geometries::Coordinate(it.x,it.y,it.z));
                    newBevPolygon.subtype = it.type1;
                }
                arrArrPts.Add(newBevPolygon);
            }
        }
    }

    void PostProcessRoadMark::GetAveragePt(const Engine::Base::Array<Engine::Geometries::Coordinate>& arrCloudPts, Engine::Geometries::Coordinate& midpt)
    {
        if (arrCloudPts.IsEmpty())
        {
            return;
        }

        double sumX = 0, sumY = 0, sumZ = 0;
        int nCount = arrCloudPts.GetCount();

        for (int i = 0; i < nCount; i++)
        {
            sumX += arrCloudPts[i].x;
            sumY += arrCloudPts[i].y;
            sumZ += arrCloudPts[i].z;
        }

        midpt.x = sumX / nCount;
        midpt.y = sumY / nCount;
        midpt.z = sumZ / nCount;
    }

    void PostProcessRoadMark::afterProcess(
        const std::map<int, std::vector<Eigen::Vector3f>>& cornerPointMap,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& refLaneBoundaryCloud,
        std::map<int, std::vector<Eigen::Vector3f>>& newCornerPointMap)
    {
        if (cornerPointMap.empty()) {
            std::cerr << "Error: cornerPointMap is empty. Cannot proceed with arrow feature extraction." << std::endl;
            return;
        } 
        //------------------------ 阶段0：参数设置 ----------
        newCornerPointMap = cornerPointMap;
        const float CLUSTER_TOL = 4.0f;
        const float DIR_THRESH = std::cos(10.0f * M_PI / 180.0f);
        const float PROJ_CLUSTER_TOL = 3.5f;
        const int MIN_CLUSTER_SIZE = 2;

        //------------------------ 阶段1：箭头特征提取 ------------------------
        std::vector<ArrowFeature> arrow_features;
        arrow_features.reserve(cornerPointMap.size());

        pcl::PointCloud<pcl::PointXYZI> arrow_centers_cloud;
        arrow_centers_cloud.reserve(cornerPointMap.size());

        for (const auto& [id, points] : cornerPointMap) {
            if (points.size() != 4) continue;

            Eigen::Vector3f pointsXY[4];
            for (size_t i = 0; i < 4; ++i) {
                pointsXY[i] << points[i].x(), points[i].y(), 0.0f;
            }

            const Eigen::Vector3f center = 0.25f * (pointsXY[0] + pointsXY[1] + pointsXY[2] + pointsXY[3]);
            const Eigen::Vector3f edge_vec = (pointsXY[1] - pointsXY[0]).squaredNorm() >
                                            (pointsXY[3] - pointsXY[0]).squaredNorm() ?
                                            (pointsXY[1] - pointsXY[0]) : (pointsXY[3] - pointsXY[0]);

            arrow_features.emplace_back(center, edge_vec.normalized(), id, points);

            pcl::PointXYZI pt;
            pt.getVector3fMap() = center;
            pt.intensity = static_cast<float>(arrow_features.size() - 1);
            arrow_centers_cloud.push_back(std::move(pt));
        }

        auto arrow_centers_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(arrow_centers_cloud);
        //------------------------ 阶段2：车道数据预处理 ------------------------
        std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> lane_clouds_cache;

        lane_clouds_cache.reserve(refLaneBoundaryCloud->size());

        for (const auto& pt : *refLaneBoundaryCloud) {
            const int line_id = static_cast<int>(pt.intensity);

            auto& cloud = lane_clouds_cache.emplace(
                line_id, std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()
            ).first->second;

            cloud->emplace_back(pt.x, pt.y, 0.0f);
        }

        //------------------------ 阶段3：聚类处理 ------------------------
        if (!arrow_centers_cloud_ptr || arrow_centers_cloud_ptr->empty()) {
            std::cerr << "Error: arrow_centers_cloud_ptr is empty or invalid. Cannot create KDTree." << std::endl;
            return;
        }

        std::vector<pcl::PointIndices> raw_clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setInputCloud(arrow_centers_cloud_ptr);
        ec.setClusterTolerance(CLUSTER_TOL);
        ec.setMinClusterSize(MIN_CLUSTER_SIZE);
        auto kd_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
        kd_tree->setInputCloud(arrow_centers_cloud_ptr);
        ec.setSearchMethod(kd_tree);
        ec.extract(raw_clusters);

        // debug
        // std::cout << "第一次聚类结果：" << std::endl;
        // for (const auto& cluster : raw_clusters) {
        //     std::cout << "Cluster with " << cluster.indices.size() << " points: ";
        //     for (int idx : cluster.indices) {
        //         std::cout << idx << " " ;
        //     }
        //     std::cout << std::endl;
        // }

        std::vector<std::vector<int>> valid_clusters;
        valid_clusters.reserve(raw_clusters.size());

        for (int i = 0; i < static_cast<int>(raw_clusters.size()); ++i) {
            const auto& cluster = raw_clusters[i];
            Eigen::Vector3f sum_dir = Eigen::Vector3f::Zero();
            for (int idx : cluster.indices) {
                sum_dir += arrow_features[static_cast<int>(arrow_centers_cloud_ptr->at(idx).intensity)].direction;
            }
            const Eigen::Vector3f avg_dir = sum_dir.normalized();

            std::vector<int> valid;
            valid.reserve(cluster.indices.size());
            for (int idx : cluster.indices) {
                const auto& dir = arrow_features[static_cast<int>(arrow_centers_cloud_ptr->at(idx).intensity)].direction;
                if (abs(dir.dot(avg_dir)) > DIR_THRESH) valid.push_back(idx);
            }

            if (!valid.empty()) valid_clusters.emplace_back(std::move(valid));
        }

        // debug
        // std::cout << "对第一次聚类方向过滤后的结果：" << std::endl;
        // for (const auto& valid_cluster : valid_clusters) {
        //     std::cout << "valid_cluster with " << valid_cluster.size() << " points: ";
        //     for (int idx : valid_cluster) {
        //         std::cout << idx << " ";
        //     }
        //     std::cout << std::endl;
        // }

        //------------------------ 阶段4：车道线方向 ----------------------
        std::vector<Eigen::Vector3f> cluster_line_dirs(valid_clusters.size());

        for (int i = 0; i < static_cast<int>(valid_clusters.size()); ++i) {
            const auto& cluster = valid_clusters[i];
            Eigen::Vector3f cluster_center = Eigen::Vector3f::Zero();
            for (int idx : cluster) {
                const auto& point = arrow_centers_cloud_ptr->at(idx);
                cluster_center += arrow_features[static_cast<int>(point.intensity)].center;
            }
            cluster_center /= static_cast<float>(cluster.size());

            auto nearby_result = findLinesWithDynamicThreshold(
                {cluster_center.x(), cluster_center.y(), cluster_center.z()},
                4.0, 10.0, lane_clouds_cache
            );

            auto dir_result = filterLinesByDirection(std::move(nearby_result), 30.0f);
            cluster_line_dirs[i] = dir_result.main_direction;
        }
        // debug
        // std::cout << "每个聚类公共车道线方向：" << std::endl;
        // for (const auto& line_dir : cluster_line_dirs) {
        //     std::cout << "line_dir: " << line_dir.transpose() << std::endl;
        // }

        //------------------------ 阶段5：投影计算 ----------------------
        for (int i = 0; i < static_cast<int>(valid_clusters.size()); ++i) {
            std::vector<Eigen::Vector3f> projections;
            std::vector<int> proj_indices;
            projections.reserve(valid_clusters[i].size());
            proj_indices.reserve(valid_clusters[i].size());

            for (int idx : valid_clusters[i]) {
                const auto& arrow = arrow_features[static_cast<int>(arrow_centers_cloud_ptr->at(idx).intensity)];
                const Eigen::Vector3f& lane_dir = cluster_line_dirs[i];

                Eigen::Vector3f proj = arrow.center;
                if (lane_dir.squaredNorm() > 0.0f) {
                    float t = arrow.center.dot(lane_dir);
                    proj = t * lane_dir;
                }

                projections.emplace_back(proj);
                proj_indices.emplace_back(idx);
            }

           // debug
            // std::cout << "投影点：" << std::endl;
            // for (const auto& projection : projections) {
            //     std::cout << "projection: " << projection.transpose() << std::endl;
            // }

            // 投影点聚类
            auto proj_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            proj_cloud->reserve(projections.size());
            for (const auto& p : projections) {
                proj_cloud->emplace_back(p.x(), p.y(), p.z());
            }

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_proj;
            ec_proj.setClusterTolerance(PROJ_CLUSTER_TOL);
            ec_proj.setMinClusterSize(MIN_CLUSTER_SIZE);
            ec_proj.setInputCloud(proj_cloud);

            std::vector<pcl::PointIndices> proj_clusters;
            ec_proj.extract(proj_clusters);

            // debug
            // std::cout << "投影聚类结果：" << std::endl;
            // for (const auto& cluster : proj_clusters) {
            //     std::cout << "proj_clusters Cluster with " << cluster.indices.size() << " points: ";
            //     for (int idx : cluster.indices) {
            //         std::cout << proj_indices[idx] << " ";
            //     }
            //     std::cout << std::endl;
            // }

            // 坐标变换计算
            for (const auto& cluster : proj_clusters) {
                if (cluster.indices.size() < 2) continue;

                Eigen::Vector3f cluster_center = Eigen::Vector3f::Zero();
                float avg_length = 0.0f, avg_wide = 0.0f;

                for (int idx : cluster.indices) {
                    const auto& arrow = arrow_features[proj_indices[idx]];
                    cluster_center += arrow.center;
                    avg_wide += (arrow.points[1] - arrow.points[0]).norm();
                    avg_length += (arrow.points[3] - arrow.points[0]).norm();
                }

                const float inv_size = 1.0f / cluster.indices.size();
                cluster_center *= inv_size;
                avg_wide *= inv_size;
                avg_length *= inv_size;

                const Eigen::Vector3f lane_dir = cluster_line_dirs[i];
                const Eigen::Vector3f perp_dir(-lane_dir.y(), lane_dir.x(), 0.0f);
                const Eigen::Vector3f lane_component = lane_dir * lane_dir.dot(cluster_center);

                for (int idx : cluster.indices) {
                    const auto& arrow = arrow_features[proj_indices[idx]];

                    const Eigen::Vector3f new_center = arrow.center + (lane_component - lane_dir * lane_dir.dot(arrow.center));
                    const float scale_length = avg_length / (arrow.points[3] - arrow.points[0]).norm();
                    const float scale_width = avg_wide / (arrow.points[1] - arrow.points[0]).norm();

                    std::vector<Eigen::Vector3f> new_pts;
                    new_pts.reserve(4);
                    for (const auto& pt : arrow.points) {
                        const Eigen::Vector3f rel = pt - arrow.center;
                        const Eigen::Vector3f new_rel(
                            rel.dot(arrow.direction) * scale_length * lane_dir +
                            rel.dot(Eigen::Vector3f(-arrow.direction.y(), arrow.direction.x(), 0.0f)) * scale_width * perp_dir
                        );
                        new_pts.emplace_back(new_center + new_rel);
                    }

                    newCornerPointMap[arrow.original_id] = std::move(new_pts);
                }
            }
        }
    }


    PostProcessRoadMark::NearbyLinesResult PostProcessRoadMark::findNearbyLinesWithMinPoints(
        const pcl::PointXYZ& target_point,
        double threshold,
        const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache,
        int min_points)
    {
        NearbyLinesResult result;
        const double squared_threshold = threshold * threshold;
        const auto& tx = target_point.x;
        const auto& ty = target_point.y;
        const auto& tz = target_point.z;

        std::vector<std::pair<int, size_t>> line_sizes;

        for (const auto& [line_id, cloud] : lane_clouds_cache) {
            if (!cloud || cloud->empty()) continue;

            auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            for (const auto& pt : *cloud) {
                const double dx = pt.x - tx;
                const double dy = pt.y - ty;
                const double dz = pt.z - tz;
                if ((dx*dx + dy*dy + dz*dz) <= squared_threshold) {
                    filtered_cloud->push_back(pt);
                }
            }

            if (filtered_cloud->size() >= min_points) {
                result.line_ids.push_back(line_id);
                result.line_points[line_id] = filtered_cloud;
                line_sizes.emplace_back(line_id, filtered_cloud->size());
            }
        }

        // 如果找到的线条数量大于3，保留点数最多的前三条
        if (line_sizes.size() > 3) {
            std::sort(line_sizes.begin(), line_sizes.end(), [](const auto& a, const auto& b) {
                return a.second > b.second;
            });

            // 只保留前三条
            result.line_ids = {line_sizes[0].first, line_sizes[1].first, line_sizes[2].first};
            std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> new_line_points;
            for (const auto& [line_id, _] : line_sizes) {
                if (new_line_points.size() < 3) {
                    new_line_points[line_id] = result.line_points[line_id];
                }
            }
            result.line_points = std::move(new_line_points);
        }

        return result;
    }

    PostProcessRoadMark::NearbyLinesResult PostProcessRoadMark::findLinesWithDynamicThreshold(
        const pcl::PointXYZ& target_point,
        double initial_threshold,
        double max_threshold,
        const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache)
    {
        double current_threshold = initial_threshold;
        while (current_threshold <= max_threshold) {
            auto result = findNearbyLinesWithMinPoints(target_point, current_threshold, lane_clouds_cache, 2);
            if (result.line_ids.size() >= 3) return result;
            current_threshold *= 1.5;
        }
        return findNearbyLinesWithMinPoints(target_point, max_threshold, lane_clouds_cache, 2);
    }


    PostProcessRoadMark::DirectionFilterResult PostProcessRoadMark::filterLinesByDirection(
        const NearbyLinesResult& candidates,
        float angle_threshold_deg)
    {
        DirectionFilterResult result;
        if (candidates.line_ids.empty()) return result;


        std::vector<Eigen::Vector3f> directions;
        std::vector<int> valid_lines;

        for (const int line_id : candidates.line_ids) {
            const auto& cloud = candidates.line_points.at(line_id);
            if (cloud->size() < 2) continue;

            Eigen::Vector3f dir;
            if (cloud->size() == 2) {  // 两点直接计算
                const auto& p1 = cloud->at(0);
                const auto& p2 = cloud->at(1);
                dir = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
            } else {  // PCA主成分分析
                pcl::PCA<pcl::PointXYZ> pca;
                pca.setInputCloud(cloud);
                dir = pca.getEigenVectors().col(0);
            }

            if (dir.norm() < 1e-6) continue;
            directions.push_back(dir.normalized());
            valid_lines.push_back(line_id);
        }

        if (directions.empty()) return result;

        Eigen::Vector3f sum_dir = Eigen::Vector3f::Zero();
        for (const auto& dir : directions) {
            sum_dir += dir;
        }
        Eigen::Vector3f avg_dir = sum_dir / directions.size();
        avg_dir.normalize();

        const float cos_th = std::cos(angle_threshold_deg * M_PI / 180.0f);
        std::vector<Eigen::Vector3f> filtered_directions;

        for (size_t i = 0; i < directions.size(); ++i) {
            if (directions[i].dot(avg_dir) >= cos_th) {
                result.filtered_lines.push_back(valid_lines[i]);
                filtered_directions.push_back(directions[i]);
            }
        }

        if (!filtered_directions.empty()) {
            Eigen::Vector3f sum_dir = Eigen::Vector3f::Zero();
            for (const auto& dir : filtered_directions) {
                sum_dir += dir;
            }
            result.main_direction = sum_dir.normalized();
        }

        return result;
    }

} //namespace RoadMapping