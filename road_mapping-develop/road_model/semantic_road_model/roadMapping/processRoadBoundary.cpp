#include "processRoadBoundary.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>             //边界提取
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法
#include <pcl/sample_consensus/model_types.h>  //模型定义
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/filters/extract_indices.h>       //索引提取
#include <pcl/segmentation/region_growing.h>
#include <boost/thread/thread.hpp>

#include "cluster.h"
#include "Utils.h"
#include "json.hpp"
#include <proj_api.h>
#include "pclPtType.h"
#include "pclFilter.h"
#include "skeleton_cluster.hpp"

using namespace Engine::Base;
using namespace Engine::Geometries;

namespace RoadMapping
{
    void ProcessRoadBoundary::EigenSolverEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr &Cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor, double radius, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        kdtree_cloud->setInputCloud(Cloud); // 设置要搜索的点云，建立KDTree

        std::vector<int> deleteVec;
        for (int i = 0; i < Cloud->size(); ++i)
        {
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            kdtree_cloud->radiusSearch(Cloud->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
            if (pointIdxRadiusSearch.empty())
            {
                // 找最近点
                int K = 1;
                kdtree_cloud->nearestKSearch(Cloud->at(i), K, pointIdxRadiusSearch, pointRadiusSquaredDistance);
            }
            // 寻找参考线最近点
            int K = 1;
            std::vector<int> Idx;
            std::vector<float> SquaredDistance;
            kdtree_refLaneAnchor->nearestKSearch(Cloud->at(i), K, Idx, SquaredDistance);
            if (Idx[0] == 0)
                Idx[0] = 1;
            Eigen::Vector3f refDirection = refLaneBoundaryCloud->at(Idx[0]).getVector3fMap() - refLaneBoundaryCloud->at(Idx[0] - 1).getVector3fMap();

            // 特征值计算
            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
            int totalNum = 0;
            for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
            {
                centroid += Cloud->at(pointIdxRadiusSearch[j]).getVector3fMap();
                totalNum += 1;
            }

            centroid = centroid / totalNum;
            for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
            {
                A += (Cloud->at(pointIdxRadiusSearch[j]).getVector3fMap() - centroid) * (Cloud->at(pointIdxRadiusSearch[j]).getVector3fMap() - centroid).transpose();
            }

            A = A / totalNum;
            A = (A + A.transpose()) / 2;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
            es.compute(A);
            Eigen::Vector3f eigenvalues = es.eigenvalues();
            // std::cout<<"eigenvalues: "<<eigenvalues.transpose()<<std::endl;
            Eigen::Matrix3f eigenvectors = es.eigenvectors();
            Eigen::Vector3f direction = eigenvectors.col(2);
            if (direction.dot(refDirection) < 0.f)
            {
                direction = -direction;
            }

            cloud_normals->push_back(pcl::Normal(direction[0], direction[1], direction[2]));

            // 计算角度
            double theta = atan2(direction.y(), direction.x()) - atan2(refDirection.y(), refDirection.x()); // 弧度
            if (theta > M_PI)
            {
                theta -= 2 * M_PI;
            }
            if (theta < -M_PI)
            {
                theta += 2 * M_PI;
            }
            double value = fabs(theta * 180 / M_PI);
            //            std::cout<<"夹角："<<value<<std::endl;
            if (fabs(value - 90) < 20)
            { // 大概大于80度以后
                deleteVec.push_back(i);
                //                std::cout<<"index："<<i<<std::endl;
                //                std::cout<<"refindex："<<Idx[0]<<std::endl;
                //                std::cout<<"direction："<<direction<<std::endl;
                //                std::cout<<"refDirection："<<refDirection<<std::endl;
                //                std::cout<<"点："<<Cloud->at(i)<<std::endl;
            }
        }

        if (!deleteVec.empty())
        {
            // 删除特征向量垂直的数据
            int n = deleteVec.size() - 1;
            for (int i = n; i >= 0; i--)
            {
                cloud_normals->erase(cloud_normals->begin() + deleteVec[i]);
                Cloud->erase(Cloud->begin() + deleteVec[i]);
            }
        }
    }

    void ProcessRoadBoundary::EigenSolverEstimationNew(pcl::PointCloud<pcl::PointXYZ>::Ptr &Cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor, double radius, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, std::map<int, int>& indexMap)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        kdtree_cloud->setInputCloud(Cloud); // 设置要搜索的点云，建立KDTree

        std::vector<int> deleteVec;
        for (int i = 0; i < Cloud->size(); ++i)
        {
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            kdtree_cloud->radiusSearch(Cloud->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
            if (pointIdxRadiusSearch.empty())
            {
                // 找最近点
                int K = 1;
                kdtree_cloud->nearestKSearch(Cloud->at(i), K, pointIdxRadiusSearch, pointRadiusSquaredDistance);
            }
            // 寻找参考线最近点
            int K = 1;
            std::vector<int> Idx;
            std::vector<float> SquaredDistance;
            kdtree_refLaneAnchor->nearestKSearch(Cloud->at(i), K, Idx, SquaredDistance);
            if (Idx[0] == 0)
                Idx[0] = 1;
            Eigen::Vector3f refDirection = refLaneBoundaryCloud->at(Idx[0]).getVector3fMap() - refLaneBoundaryCloud->at(Idx[0] - 1).getVector3fMap();

            // 特征值计算
            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
            int totalNum = 0;
            for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
            {
                centroid += Cloud->at(pointIdxRadiusSearch[j]).getVector3fMap();
                totalNum += 1;
            }

            centroid = centroid / totalNum;
            for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
            {
                A += (Cloud->at(pointIdxRadiusSearch[j]).getVector3fMap() - centroid) * (Cloud->at(pointIdxRadiusSearch[j]).getVector3fMap() - centroid).transpose();
            }

            A = A / totalNum;
            A = (A + A.transpose()) / 2;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
            es.compute(A);
            Eigen::Vector3f eigenvalues = es.eigenvalues();
            // std::cout<<"eigenvalues: "<<eigenvalues.transpose()<<std::endl;
            Eigen::Matrix3f eigenvectors = es.eigenvectors();
            Eigen::Vector3f direction = eigenvectors.col(2);
            if (direction.dot(refDirection) < 0.f)
            {
                direction = -direction;
            }

            cloud_normals->push_back(pcl::Normal(direction[0], direction[1], direction[2]));

            // 计算角度
            double theta = atan2(direction.y(), direction.x()) - atan2(refDirection.y(), refDirection.x()); // 弧度
            if (theta > M_PI)
            {
                theta -= 2 * M_PI;
            }
            if (theta < -M_PI)
            {
                theta += 2 * M_PI;
            }
            double value = fabs(theta * 180 / M_PI);
            //            std::cout<<"夹角："<<value<<std::endl;
            if (fabs(value - 90) < 20)
            { // 大概大于80度以后
                deleteVec.push_back(i);
                //                std::cout<<"index："<<i<<std::endl;
                //                std::cout<<"refindex："<<Idx[0]<<std::endl;
                //                std::cout<<"direction："<<direction<<std::endl;
                //                std::cout<<"refDirection："<<refDirection<<std::endl;
                //                std::cout<<"点："<<Cloud->at(i)<<std::endl;
            }
        }

        if (!deleteVec.empty())
        {
            // 删除特征向量垂直的数据
            int n = deleteVec.size() - 1;
            for (int i = n; i >= 0; i--)
            {
                int deleteIndex = deleteVec[i];

                indexMap.erase(deleteIndex);
                
                cloud_normals->erase(cloud_normals->begin() + deleteVec[i]);
                Cloud->erase(Cloud->begin() + deleteVec[i]);
            }

            // 更新indexMap
            std::map<int, int> updatedIndexMap;
            int currentIndex = 0;
            for (const auto& pair : indexMap) {
                if (pair.first >= currentIndex) {
                    updatedIndexMap[currentIndex] = pair.second;
                    currentIndex++;
                }
            }
            indexMap.swap(updatedIndexMap);
        }
    }

    void ProcessRoadBoundary::road_cloud_edge_precess(std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir)
    {
        //------------------------筛选label点云---------------------------
        std::cout << "lidarFile: " << pcdPath << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(pcdPath, *pc_ptr);

        // 输入道路点和extend点
        // label： "line": 80, "arrow_id": 100, "road_id": 0
        pcl::PointCloud<pcl::PointXYZ>::Ptr roadcloud(new pcl::PointCloud<pcl::PointXYZ>);
        int fil_list[] = {0, 80, 90, 100, 110, 120, 140};

        for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++)
        {
            MyColorPointType &pcl_p = (*iter);
            // int b = int(pcl_p.label);
            int b = 0;
            if (std::find(fil_list, fil_list + sizeof(fil_list) / sizeof(fil_list[0]), b) !=
                fil_list + sizeof(fil_list) / sizeof(fil_list[0]))
                roadcloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }
        if (roadcloud->empty() || roadcloud->points.empty())
        {
            return;
        }
        pcl::io::savePCDFileBinary(outputDir + "/" + "all_road.pcd", *roadcloud);

        //------------------------点云统计滤波---------------------------
        //        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_statistical(new pcl::PointCloud<pcl::PointXYZ>);
        //        pclFilter::StatisticalOutlierRemoval(roadcloud, cloud_statistical, 100, 1.0);

        //------------------------半径滤波---------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_statistical(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::RadiusOutlierRemoval(roadcloud, cloud_statistical, 0.2, 5);

        //------------------------均匀下采样---------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::UniformSampling(cloud_statistical, cloud, 0.1);
        // pcl::io::savePCDFileBinary(outputDir + "/" + "AAAA.pcd", *cloud);

        //------------------------计算法向量---------------------------
        std::cout << "->计算法向量" << std::endl;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
        normEst.setRadiusSearch(0.5);
        normEst.compute(*normals);

        std::cout << "->点云边缘估计" << std::endl;
        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
        boundEst.setInputCloud(cloud);
        boundEst.setInputNormals(normals);
        boundEst.setRadiusSearch(0.5);
        boundEst.setAngleThreshold(M_PI / 2);
        boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
        boundEst.compute(boundaries);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < cloud->points.size(); i++)
        {

            if (boundaries[i].boundary_point > 0)
            {
                cloud_boundary->push_back(cloud->points[i]);
            }
        }

        std::cout << "边界点个数:" << cloud_boundary->points.size() << std::endl;
        if (cloud_boundary->empty() || cloud_boundary->points.empty())
        {
            return;
        }
        pcl::io::savePCDFileBinary(outputDir + "/" + "boundaries.pcd", *cloud_boundary);
    }

    void ProcessRoadBoundary::get_ref_info(std::string parse_json, std::string outputDir, pcl::PointCloud<pcl::PointXYZ>::Ptr &laneSegCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor)
    {
        Utils::readRefLines_2(parse_json, refLaneBoundaryCloud);
        if (refLaneBoundaryCloud->empty() || refLaneBoundaryCloud->points.empty())
        {
            return;
        }
        pcl::io::savePCDFileBinary(outputDir + "/refLink.pcd", *refLaneBoundaryCloud);

        // 对点进行加密
        int nCount = refLaneBoundaryCloud->size();
        refLaneAnchorCloud->push_back(refLaneBoundaryCloud->at(0));
        pcl::PointXYZ pOld = refLaneBoundaryCloud->at(0);
        double tolerance = 3.0;
        double dRemainderLeth = tolerance;

        for (Int32 i = 1; i < nCount; i++)
        {
            double dx = pOld.x - refLaneBoundaryCloud->at(i).x;
            double dy = pOld.y - refLaneBoundaryCloud->at(i).y;
            double dz = pOld.z - refLaneBoundaryCloud->at(i).z;
            double dis = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dis == 1.0E-5)
            {
                continue;
            }
            if (dis < dRemainderLeth)
            {
                dRemainderLeth -= dis; // �����������һ������Ѱ��
                pOld = refLaneBoundaryCloud->at(i);
            }
            else if (dis == dRemainderLeth)
            {
                refLaneAnchorCloud->push_back(refLaneBoundaryCloud->at(i)); // ���ã��������Ϣ����������һ�߶�
                pOld = refLaneBoundaryCloud->at(i);
            }
            else
            {
                Eigen::Vector3f deltCoord = refLaneBoundaryCloud->at(i).getVector3fMap() - pOld.getVector3fMap();
                double dPercent = dRemainderLeth / dis;
                pcl::PointXYZ coord; // �����ȡ������
                coord.x = pOld.x + deltCoord[0] * dPercent;
                coord.y = pOld.y + deltCoord[1] * dPercent;
                coord.z = pOld.z + deltCoord[2] * dPercent;
                refLaneAnchorCloud->push_back(coord);
                dRemainderLeth = tolerance;
                pOld = coord;
                i--;
            }
        }

        refLaneAnchorCloud->push_back(refLaneBoundaryCloud->at(nCount - 1));

        //        //计算方向向量
        //        Eigen::Vector3f refDirection = Eigen::Vector3f::Zero();
        //        for(int i=1; i<refLaneBoundaryCloud->size(); i++){
        //            refDirection += refLaneBoundaryCloud->at(i).getVector3fMap() - refLaneBoundaryCloud->at(i-1).getVector3fMap();
        //        }
        //        refDirection.normalize();
        //        Eigen::Vector3f refCentroid = refLaneBoundaryCloud->at(refLaneBoundaryCloud->size()/2).getVector3fMap();
        //
        //        float minVal = 1e6, maxVal = -1e6;
        //        for(int i=0; i<laneSegCloud->size(); i++){
        //            float val = refDirection.dot(laneSegCloud->at(i).getVector3fMap()-refCentroid);
        //            if(val<minVal) minVal = val;
        //            if(val>maxVal) maxVal = val;
        //        }
        //        float resolution = 3.f;
        ////    int num = (maxVal - minVal)/resolution + 1;
        //        int num1 = maxVal/resolution + 1;
        //        int num2 = (fabs(minVal))/resolution + 1;
        //        for(int i=-num2; i<=num1; i++){
        //            Eigen::Vector3f pos = refCentroid + resolution*i*refDirection;
        //            refLaneAnchorCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
        //        }

        // pcl::io::savePCDFileBinary(outputDir+"/refLaneAnchorCloud.pcd", *refLaneAnchorCloud);
        kdtree_refLaneAnchor->setInputCloud(refLaneAnchorCloud); // 设置要搜索的点云，建立KDTree
    }

    // void ProcessRoadBoundary::run(std::string dataDir, std::string outputDir)
    //{
    //     std::string reconsMergeOutputDir = dataDir+"/reconstruction/output";
    //     std::string bevInfoJsonFile = reconsMergeOutputDir + "/bev_info.json";
    //     std::cout<<"bevInfoJsonFile: "<<bevInfoJsonFile<<std::endl;
    //     std::ifstream ifs_bevInfo(bevInfoJsonFile);
    //     if(!ifs_bevInfo.is_open()){
    //         ifs_bevInfo.close();
    //         return;
    //     }
    //     nlohmann::json bevInfo;
    //     bevInfo<<ifs_bevInfo;
    //     int utm_num = bevInfo["utm_num"];
    //     float pixelSize = bevInfo["pixel_size_m"];
    //     Eigen::Matrix3d T_utm_bev = Eigen::Matrix3d::Identity();
    //     T_utm_bev<<bevInfo["T_utm_bev"][0][0], bevInfo["T_utm_bev"][0][1], bevInfo["T_utm_bev"][0][2],
    //                bevInfo["T_utm_bev"][1][0], bevInfo["T_utm_bev"][1][1], bevInfo["T_utm_bev"][1][2],
    //                bevInfo["T_utm_bev"][2][0], bevInfo["T_utm_bev"][2][1], bevInfo["T_utm_bev"][2][2];
    //     std::cout<<"T_utm_bev:\n"<<T_utm_bev<<std::endl;
    //     Eigen::Vector2d UTM_base = T_utm_bev.block<2,1>(0,2);
    //     projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
    //     std::string utm_param = "+proj=utm";
    //     utm_param = utm_param + " +zone=" + std::to_string(utm_num) + "N +ellps=WGS84 +no_defs";
    //     projPJ g_utm = pj_init_plus(utm_param.data());
    //
    //     auto UTM2llh = [&](Eigen::Vector3d utmPos){
    //         Eigen::Vector3d llh = utmPos;
    //         pj_transform(g_utm, g_pWGS84, 1, 1, &llh(0), &llh(1), &llh(2));
    //         std::swap(llh(0), llh(1));
    //         return llh;
    //     };
    //
    //     Eigen::Vector3d init_llh = UTM2llh(Eigen::Vector3d(UTM_base(0), UTM_base(1), 0.f));
    //     std::cout<<"init_llh: "<<init_llh.transpose()<<std::endl;
    //
    //     Eigen::Matrix3d Cen = tools::Earth::Pos2Cne(init_llh).transpose();
    //     Eigen::Vector3d init_ecef = tools::Earth::LLH2ECEF(init_llh);
    //     auto ENU2UTM = [&](Eigen::Vector3d pos_enu){
    //         Eigen::Vector3d pos_ecef = init_ecef + Cen*pos_enu;
    //         Eigen::Vector3d pos_llh = tools::Earth::ECEF2LLH(pos_ecef);
    //         Eigen::Vector3d pos_utm(pos_llh(1), pos_llh(0), pos_llh(2));
    //         pj_transform(g_pWGS84, g_utm, 1, 1, &pos_utm(0), &pos_utm(1), &pos_utm(2));
    //         return pos_utm;
    //     };
    //
    //     std::string mapfusionDir = dataDir+"/mapfusion";
    //     std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> laneBoundaryClouds;
    //     std::ifstream ifs(mapfusionDir+"/fusion_map--1.txt");
    //     if(!ifs.is_open()){
    //         ifs.close();
    //         return;
    //     }
    //     std::string line;
    //     //ID x y z score type color marking ldm
    //     getline(ifs, line);
    //     std::string delimiter = " ";
    //     while(getline(ifs, line)){
    //         std::vector<std::string> vec;
    //         Utils::split(line, delimiter, vec);
    //         if(vec.size()<5){
    //             continue;
    //         }
    //         int id = std::stoi(vec[0]);
    //         int type = std::stoi(vec[5]);
    //         if(type != 1){
    //             continue;
    //         }
    //         if(laneBoundaryClouds.find(id)==laneBoundaryClouds.end()){
    //             laneBoundaryClouds[id] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //         }
    //         Eigen::Vector3d utmPos(std::stod(vec[1]), std::stod(vec[2]), 0.f);
    //         Eigen::Vector3d llh = UTM2llh(utmPos);
    //         Eigen::Vector3d pos = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
    //         laneBoundaryClouds[id]->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
    //     }
    //
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     int refLaneId = -1, refLaneSize = 0;
    //     for(auto iter=laneBoundaryClouds.begin(); iter!=laneBoundaryClouds.end(); iter++){
    //         if(iter->second->size() > refLaneSize){
    //             refLaneSize = iter->second->size();
    //             refLaneId = iter->first;
    //         }
    //     }
    //     for(int i=0; i<laneBoundaryClouds[refLaneId]->size(); i+=10){
    //         refLaneBoundaryCloud->push_back(laneBoundaryClouds[refLaneId]->at(i));
    //     }
    //     std::cout<<"ref laneBoundaryCloud size: "<<refLaneBoundaryCloud->size()<<std::endl;
    //     pcl::io::savePCDFileBinary(outputDir+"/refLaneBoundaryCloud.pcd", *refLaneBoundaryCloud);
    //     pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLane(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    //     kdtree_refLane->setInputCloud(refLaneBoundaryCloud); // 设置要搜索的点云，建立KDTree
    //
    //     std::string roadSegImgFile = reconsMergeOutputDir + "/bev_road_seg.png";
    //     cv::Mat img = cv::imread(roadSegImgFile, cv::IMREAD_GRAYSCALE);
    //     std::cout << "img rows: " << img.rows << " cols: " << img.cols << std::endl;
    //     // get road boundary
    //     // road:150, roadMark:255, road extension:100
    //     for (int i = 1; i < img.rows - 1; i++)
    //     {
    //         for (int j = 1; j < img.cols - 1; j++)
    //         {
    //             if (img.at<uchar>(i, j) == 255)
    //             {
    //                 img.at<uchar>(i, j) = 150;
    //             }
    //             else if (img.at<uchar>(i, j) == 100 &&
    //                         img.at<uchar>(i - 1, j) == 150 && img.at<uchar>(i, j - 1) == 150 && img.at<uchar>(i, j + 1) == 150 &&
    //                         img.at<uchar>(i + 1, j) == 150)
    //             {
    //                 img.at<uchar>(i, j) = 150;
    //             }
    //         }
    //     }
    //     cv::imwrite(outputDir+"/inpaint.png", img);
    //     auto determine = [&](int id_x, int id_y, int *adjacentPx, int &currentType, int &switchedNum)
    //     {
    //         if (img.at<uchar>(id_x, id_y) == 150)
    //         {
    //             if (currentType == 0)
    //             {
    //                 adjacentPx[currentType] += 1;
    //             }
    //             else if (switchedNum >= 2)
    //             {
    //                 return false;
    //             }
    //             else
    //             {
    //                 currentType = 0;
    //                 adjacentPx[currentType] += 1;
    //                 switchedNum += 1;
    //             }
    //         }
    //         else if (img.at<uchar>(id_x, id_y) == 100)
    //         {
    //             if (currentType == 1)
    //             {
    //                 adjacentPx[currentType] += 1;
    //             }
    //             else if (switchedNum >= 2)
    //             {
    //                 return false;
    //             }
    //             else
    //             {
    //                 currentType = 1;
    //                 adjacentPx[currentType] += 1;
    //                 switchedNum += 1;
    //             }
    //         }
    //         else
    //         {
    //             return false;
    //         }
    //         return true;
    //     };
    //     cv::Mat roadBoundaryImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     std::cout << "roadBoundaryImg rows: " << roadBoundaryImg.rows << " cols: " << roadBoundaryImg.cols << std::endl;
    //     for (int i = 1; i < img.rows - 1; i++)
    //     {
    //         for (int j = 1; j < img.cols - 1; j++)
    //         {
    //             int adjacentPx[2] = {0, 0};
    //             int currentType = 0;
    //             int switchedNum = 0;
    //             if (img.at<uchar>(i, j) != 150)
    //                 continue;
    //             if (img.at<uchar>(i - 1, j - 1) == 150)
    //             {
    //                 currentType = 0;
    //             }
    //             else if (img.at<uchar>(i - 1, j - 1) == 100)
    //             {
    //                 currentType = 1;
    //             }
    //             else
    //             {
    //                 continue;
    //             }
    //             adjacentPx[currentType] += 1;
    //             if (!determine(i - 1, j, adjacentPx, currentType, switchedNum))
    //                 continue;
    //             if (!determine(i - 1, j + 1, adjacentPx, currentType, switchedNum))
    //                 continue;
    //             if (!determine(i, j + 1, adjacentPx, currentType, switchedNum))
    //                 continue;
    //             if (!determine(i + 1, j + 1, adjacentPx, currentType, switchedNum))
    //                 continue;
    //             if (!determine(i + 1, j, adjacentPx, currentType, switchedNum))
    //                 continue;
    //             if (!determine(i + 1, j - 1, adjacentPx, currentType, switchedNum))
    //                 continue;
    //             if (!determine(i, j - 1, adjacentPx, currentType, switchedNum))
    //                 continue;
    //             if (adjacentPx[0] >= 3 && adjacentPx[1] >= 3){
    //                 Eigen::Vector3d utmPos(T_utm_bev(0,0)*j+T_utm_bev(0,1)*i+T_utm_bev(0,2),
    //                                        T_utm_bev(1,0)*j+T_utm_bev(1,1)*i+T_utm_bev(1,2), 0.f);
    //                 Eigen::Vector3d llh = UTM2llh(utmPos);
    //                 Eigen::Vector3d pos = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
    //                 cloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
    //             }
    //         }
    //     }
    //
    //     std::cout << "road Boundary cloud size: " << cloud->size() << std::endl;
    //     pcl::io::savePCDFileBinary(outputDir+"/totalRoadBoundary.pcd", *cloud);
    //
    //
    //     // cluster cloud
    //     std::vector<std::vector<int>> clusters;
    //     float radius = 1.f;
    //     int minPts = 3;
    //     Inference::DBSCAN(cloud, radius, minPts, clusters);
    //     std::vector<Inference::Segment> segments;
    //
    //     // generate segment
    //     for (int i = 0; i < clusters.size(); i++)
    //     {
    //         std::cout << i << " th cluster size: " << clusters[i].size() << std::endl;
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //         pcl::copyPointCloud(*cloud, clusters[i], *clusterCloud);
    //         int preSegmentsSize = segments.size();
    //         Inference::ExtractSegment(clusterCloud, segments, 4.f, 3);
    //         int currentSegmentsSize = segments.size();
    //         std::cout << i << " th cluster has " << currentSegmentsSize - preSegmentsSize << " segments" << std::endl;
    //     }
    //     if(segments.empty()){
    //         return;
    //     }
    //     for(int i=0; i<segments.size(); i++){
    //         if(segments[i].anchorIndices.size() > 10){
    //             segments[i].selectAnchor(5);
    //         }
    //         std::cout<<i<<" th segment anchorIndices size: "<<segments[i].anchorIndices.size()<<std::endl;
    //         //arrange direction to coincide with key pose trajectory
    //         segments[i].DecideOrientation(refLaneBoundaryCloud, kdtree_refLane);
    //     }
    //     std::vector<std::pair<int, float>> disp(segments.size());
    //     for(int i=0; i<segments.size(); i++){
    //         disp[i] = std::make_pair(i, 10000.f);
    //     }
    //     for(int i=0; i<segments.size(); i++){
    //         int ncount = 0;
    //         float accuDisp = 0.f;
    //         float radius = 20.f;
    //         for(auto anchorIndex : segments[i].anchorIndices){
    //             std::vector<int> neighbors;
    //             std::vector<float> pointRadiusSquaredDistance;
    //             kdtree_refLane->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
    //             if(neighbors.empty()){
    //                 continue;
    //             }
    //             int neighborIndex = neighbors.front();
    //             if(neighborIndex<1){
    //                 continue;
    //             }
    //             Eigen::Vector3f direction = refLaneBoundaryCloud->at(neighborIndex).getVector3fMap() -
    //                                             refLaneBoundaryCloud->at(neighborIndex-1).getVector3fMap();
    //             Eigen::Vector2f nx = direction.head(2).normalized();
    //             Eigen::Vector2f ny(-nx(1), nx(0));
    //             accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() -
    //                             refLaneBoundaryCloud->at(neighborIndex).getVector3fMap()).head(2).dot(ny);
    //             ncount++;
    //         }
    //         if(ncount > 0){
    //             disp[i].second = accuDisp/ncount;
    //         }
    //         std::cout<<i<<" th segment, disp: "<<disp[i].second<<std::endl;
    //     }
    //     Eigen::Matrix2d R_bev_utm = T_utm_bev.block<2,2>(0,0).inverse();
    //
    //     nlohmann::json obj;
    //     obj["base"]["lat"] = init_llh(0);
    //     obj["base"]["lon"] = init_llh(1);
    //     obj["base"]["alt"] = init_llh(2);
    //
    //     for(int i=0; i<segments.size(); i++){
    //         if(segments[i].cloud !=NULL && segments[i].cloud->size()>0){
    //             pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[i].cloud);
    //         }
    //
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //         pcl::copyPointCloud(*segments[i].cloud, segments[i].anchorIndices, *anchorCloud);
    //         if(anchorCloud!=NULL && anchorCloud->size()>0){
    //             pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_anchorCloud.pcd", *anchorCloud);
    //         }
    //
    //         segments[i].QpSpline();
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[i].smoothTrjCloud();
    //         if(trjCloud!=NULL && trjCloud->size()>0){
    //             pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_trjCloud.pcd", *trjCloud);
    //         }
    //
    //         for(int k=0; k<trjCloud->size(); k++){
    //             Eigen::Vector3d pos_enu = trjCloud->at(k).getVector3fMap().cast<double>();
    //             //std::cout<<"pos_enu: "<<pos_enu.transpose()<<std::endl;
    //             Eigen::Vector2d pos_utm = ENU2UTM(pos_enu).head(2);
    //             //std::cout<<"pos_utm: "<<pos_utm.transpose()<<std::endl;
    //             Eigen::Vector2d px = R_bev_utm*(pos_utm-UTM_base);
    //             //std::cout<<"px: "<<px.transpose()<<std::endl;
    //             cv::Point pt(px(0), px(1));
    //             cv::circle(img, pt, 3, cv::Scalar(255));
    //         }
    //         segments[i].addSegmentCoeff(obj, i);
    //     }
    //     cv::imwrite(outputDir+"/roadBoundary.png", img);
    //
    //     std::ofstream fid_laneBoundary(outputDir+"/roadBoundary.json");
    //     //fid_laneBoundary << std::setw(4) << obj << std::endl;
    //     fid_laneBoundary<< obj << std::endl;
    // }
    //
    // void ProcessRoadBoundary::run_yxx(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir)
    //{
    //     //假设utm_zone = 50
    //     int utm_num = 50;
    //     std::string parse_json = dataDir + "/parse_json.json";
    //
    //     //读取参数中的度带号，偏转点的utm坐标
    //     Eigen::Vector3d offset_utm;
    //     Utils::readParse(parse_json, utm_num, offset_utm);
    //
    //     std::string lidarFile = pcdPath;
    //     std::cout<<"lidarFile: "<<lidarFile<<std::endl;
    //     pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    //     pcl::io::loadPCDFile(lidarFile, *pc_ptr);
    //
    //     if(pc_ptr->empty())
    //         return;
    //
    //     //输入第一个点作为偏转点
    //     MyColorPointType pcl_p = (*pc_ptr->begin());
    //     Eigen::Vector3d firstPt = Eigen::Vector3d(pcl_p.x, pcl_p.y, pcl_p.z);
    //     Eigen::Vector3d init_llh = tools::Earth::UTM2llh(Eigen::Vector3d(firstPt(0), firstPt(1), 0.f),utm_num);
    //
    //     std::cout<<"init_llh: "<<init_llh.transpose()<<std::endl;
    //
    //     Eigen::Matrix3d Cen = tools::Earth::Pos2Cne(init_llh).transpose();
    //     Eigen::Vector3d init_ecef = tools::Earth::LLH2ECEF(init_llh);
    //
    //     //输入参考线
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     Utils::readRefLines(parse_json, refLaneBoundaryCloud, init_llh);
    //     pcl::io::savePCDFileBinary(outputDir+"/refLaneBoundaryCloud.pcd", *refLaneBoundaryCloud);
    //     pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLane(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    //     kdtree_refLane->setInputCloud(refLaneBoundaryCloud); // 设置要搜索的点云，建立KDTree
    //
    //     //计算方向向量
    //     Eigen::Vector3f refDirection = Eigen::Vector3f::Zero();
    //     for(int i=1; i<refLaneBoundaryCloud->size(); i++){
    //         refDirection += refLaneBoundaryCloud->at(i).getVector3fMap() - refLaneBoundaryCloud->at(i-1).getVector3fMap();
    //     }
    //     refDirection.normalize();
    //     Eigen::Vector3f refCentroid = refLaneBoundaryCloud->at(refLaneBoundaryCloud->size()/2).getVector3fMap();
    //
    //
    //     //输入道路点和extend点
    //     //label： "line": 80, "arrow_id": 100, "road_id": 0
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr roadcloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr extendcloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     int fil_list[]={10,20,30,40,120,150};
    //     int n = 0;
    //     for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++){
    //         MyColorPointType &pcl_p = (*iter);
    ////        std::cout<<n<<std::endl;
    ////        n++;
    //        if(pcl_p.label == 0)
    //        {
    //            Eigen::Vector3d llh = tools::Earth::UTM2llh(Eigen::Vector3d(pcl_p.x, pcl_p.y, pcl_p.z),utm_num);
    //            Eigen::Vector3d pos = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
    //            roadcloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
    //        }
    //        else
    //        {
    //            int b = int(pcl_p.label);
    //            if(std::find(fil_list,fil_list+sizeof(fil_list)/sizeof(fil_list[0]),b)!=fil_list+sizeof(fil_list)/sizeof(fil_list[0]))
    //            {
    //                Eigen::Vector3d llh = tools::Earth::UTM2llh(Eigen::Vector3d(pcl_p.x, pcl_p.y, pcl_p.z),utm_num);
    //                Eigen::Vector3d pos = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
    //                extendcloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
    //            }
    //        }
    //    }
    //    if(roadcloud->size() > 0 && extendcloud->size() > 0)
    //    {
    //        pcl::io::savePCDFileBinary(outputDir+"/roadCloud.pcd", *roadcloud);
    //        pcl::io::savePCDFileBinary(outputDir+"/extendcloud.pcd", *extendcloud);
    //    }
    //
    //    //根据边缘点检测提取路面铺设边缘
    //    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_extcloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    //    kdtree_extcloud->setInputCloud(extendcloud); // 设置要搜索的点云，建立KDTree
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    float radius1 = 0.1f;
    //    for (int i = 0; i < roadcloud->size(); ++i)
    //    {
    //        std::vector<int> pointIdxRadiusSearch;
    //        std::vector<float> pointRadiusSquaredDistance;
    //        kdtree_extcloud->radiusSearch(roadcloud->at(i), radius1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    //        if(pointIdxRadiusSearch.size() > 5)
    //            cloud->push_back(roadcloud->at(i));
    //    }
    //    if(cloud->empty())
    //        return;
    //    pcl::io::savePCDFileBinary(outputDir+"/roadCloudExtract.pcd", *cloud);
    //
    //    // cluster cloud
    //    std::vector<std::vector<int>> clusters;
    //    float radius = 1.f;
    //    int minPts = 3;
    //    Inference::DBSCAN(cloud, radius, minPts, clusters);
    //    std::vector<Inference::Segment> segments;
    //
    //    // generate segment
    //    for (int i = 0; i < clusters.size(); i++)
    //    {
    //        std::cout << i << " th cluster size: " << clusters[i].size() << std::endl;
    //        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //        pcl::copyPointCloud(*cloud, clusters[i], *clusterCloud);
    //        int preSegmentsSize = segments.size();
    //        Inference::ExtractSegment(clusterCloud, segments, 4.f, 3);
    //        int currentSegmentsSize = segments.size();
    //        std::cout << i << " th cluster has " << currentSegmentsSize - preSegmentsSize << " segments" << std::endl;
    //    }
    //    if(segments.empty()){
    //        return;
    //    }
    //    for(int i=0; i<segments.size(); i++){
    //        if(segments[i].anchorIndices.size() > 10){
    //            segments[i].selectAnchor(5);
    //        }
    //        std::cout<<i<<" th segment anchorIndices size: "<<segments[i].anchorIndices.size()<<std::endl;
    //        //arrange direction to coincide with key pose trajectory
    //        segments[i].DecideOrientation(refLaneBoundaryCloud, kdtree_refLane);
    //    }
    //    std::vector<std::pair<int, float>> disp(segments.size());
    //    for(int i=0; i<segments.size(); i++){
    //        disp[i] = std::make_pair(i, 10000.f);
    //    }
    //    for(int i=0; i<segments.size(); i++){
    //        int ncount = 0;
    //        float accuDisp = 0.f;
    //        float radius = 20.f;
    //        for(auto anchorIndex : segments[i].anchorIndices){
    //            std::vector<int> neighbors;
    //            std::vector<float> pointRadiusSquaredDistance;
    //            kdtree_refLane->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
    //            if(neighbors.empty()){
    //                continue;
    //            }
    //            int neighborIndex = neighbors.front();
    //            if(neighborIndex<1){
    //                continue;
    //            }
    //            Eigen::Vector3f direction = refLaneBoundaryCloud->at(neighborIndex).getVector3fMap() -
    //                                        refLaneBoundaryCloud->at(neighborIndex-1).getVector3fMap();
    //            Eigen::Vector2f nx = direction.head(2).normalized();
    //            Eigen::Vector2f ny(-nx(1), nx(0));
    //            accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() -
    //                         refLaneBoundaryCloud->at(neighborIndex).getVector3fMap()).head(2).dot(ny);
    //            ncount++;
    //        }
    //        if(ncount > 0){
    //            disp[i].second = accuDisp/ncount;
    //        }
    //        std::cout<<i<<" th segment, disp: "<<disp[i].second<<std::endl;
    //    }
    //
    //    nlohmann::json obj;
    //    obj["base"]["lat"] = init_llh(0);
    //    obj["base"]["lon"] = init_llh(1);
    //    obj["base"]["alt"] = init_llh(2);
    //
    //    for(int i=0; i<segments.size(); i++){
    //        if(segments[i].cloud !=NULL && segments[i].cloud->size()>0){
    //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[i].cloud);
    //        }
    //
    //        pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //        pcl::copyPointCloud(*segments[i].cloud, segments[i].anchorIndices, *anchorCloud);
    //        if(anchorCloud!=NULL && anchorCloud->size()>0){
    //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_anchorCloud.pcd", *anchorCloud);
    //        }
    //
    //        segments[i].QpSpline();
    //        pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[i].smoothTrjCloud();
    //        if(trjCloud!=NULL && trjCloud->size()>0){
    //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_trjCloud.pcd", *trjCloud);
    //        }
    //
    //        segments[i].addSegmentCoeff(obj, i);
    //    }
    //
    //    std::ofstream fid_laneBoundary(outputDir+"/roadBoundary.json");
    //    //fid_laneBoundary << std::setw(4) << obj << std::endl;
    //    fid_laneBoundary<< obj << std::endl;
    //}
    //
    // void ProcessRoadBoundary::run_yxx_2(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir)
    //{
    //    //假设utm_zone = 50
    //    int utm_num = 50;
    //    std::string parse_json = dataDir + "/parse_json.json";
    //
    //    //读取参数中的度带号，偏转点的utm坐标
    //    Eigen::Vector3d offset_utm;
    //    Utils::readParse(parse_json, utm_num, offset_utm);
    //
    //    std::string lidarFile = pcdPath;
    //    std::cout<<"lidarFile: "<<lidarFile<<std::endl;
    //    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    //    pcl::io::loadPCDFile(lidarFile, *pc_ptr);
    //
    //    //输入道路点和extend点
    //    //label： "line": 80, "arrow_id": 100, "road_id": 0
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr roadcloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr extendcloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    int fil_list[]={10,20,30,40,120,150};
    //    int n = 0;
    //    for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++){
    //        MyColorPointType &pcl_p = (*iter);
    ////        std::cout<<n<<std::endl;
    ////        n++;
    //        if(pcl_p.label == 0)
    //        {
    //            roadcloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
    //        }
    //        else
    //        {
    //            int b = int(pcl_p.label);
    //            if(std::find(fil_list,fil_list+sizeof(fil_list)/sizeof(fil_list[0]),b)!=fil_list+sizeof(fil_list)/sizeof(fil_list[0]))
    //            {
    //                extendcloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
    //            }
    //        }
    //    }
    //    if(roadcloud->size() > 0 && extendcloud->size() > 0)
    //    {
    //        pcl::io::savePCDFileBinary(outputDir+"/roadCloud.pcd", *roadcloud);
    //        pcl::io::savePCDFileBinary(outputDir+"/extendcloud.pcd", *extendcloud);
    //    }
    //
    //    //输入参考线
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    Utils::readRefLines_2(parse_json, refLaneBoundaryCloud);
    //    pcl::io::savePCDFileBinary(outputDir+"/refLaneBoundaryCloud.pcd", *refLaneBoundaryCloud);
    //
    //    //计算方向向量
    //    Eigen::Vector3f refDirection = Eigen::Vector3f::Zero();
    //    for(int i=1; i<refLaneBoundaryCloud->size(); i++){
    //        refDirection += refLaneBoundaryCloud->at(i).getVector3fMap() - refLaneBoundaryCloud->at(i-1).getVector3fMap();
    //    }
    //    refDirection.normalize();
    //    Eigen::Vector3f refCentroid = refLaneBoundaryCloud->at(refLaneBoundaryCloud->size()/2).getVector3fMap();
    //
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    float minVal = 1e6, maxVal = -1e6;
    //    for(int i=0; i<extendcloud->size(); i++){
    //        float val = refDirection.dot(extendcloud->at(i).getVector3fMap()-refCentroid);
    //        if(val<minVal) minVal = val;
    //        if(val>maxVal) maxVal = val;
    //    }
    //    float resolution = 3.f;
    ////    int num = (maxVal - minVal)/resolution + 1;
    //    int num1 = maxVal/resolution + 1;
    //    int num2 = (fabs(minVal))/resolution + 1;
    //    for(int i=-num2; i<=num1; i++){
    //        Eigen::Vector3f pos = refCentroid + resolution*i*refDirection;
    //        refLaneAnchorCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
    //    }
    //    pcl::io::savePCDFileBinary(outputDir+"/refLaneAnchorCloud.pcd", *refLaneAnchorCloud);
    //
    //    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    //    kdtree_refLaneAnchor->setInputCloud(refLaneAnchorCloud); // 设置要搜索的点云，建立KDTree
    //
    //
    //    //根据边缘点检测提取路面铺设边缘
    //    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_extcloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    //    kdtree_extcloud->setInputCloud(extendcloud); // 设置要搜索的点云，建立KDTree
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    float radius1 = 0.2f;
    //    for (int i = 0; i < roadcloud->size(); ++i)
    //    {
    //        std::vector<int> pointIdxRadiusSearch;
    //        std::vector<float> pointRadiusSquaredDistance;
    //        kdtree_extcloud->radiusSearch(roadcloud->at(i), radius1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    //        if(pointIdxRadiusSearch.size() > 3)
    //            cloud->push_back(roadcloud->at(i));
    //    }
    //    if(cloud->empty())
    //        return;
    //    pcl::io::savePCDFileBinary(outputDir+"/roadCloudExtract.pcd", *cloud);
    //
    //    // cluster cloud
    //    std::vector<std::vector<int>> clusters;
    //    float radius = 1.f;
    //    int minPts = 3;
    //    Inference::DBSCAN(cloud, radius, minPts, clusters);
    //    std::vector<Inference::Segment> segments;
    //
    //    // generate segment
    //    for (int i = 0; i < clusters.size(); i++)
    //    {
    //        std::cout << i << " th cluster size: " << clusters[i].size() << std::endl;
    //        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //        pcl::copyPointCloud(*cloud, clusters[i], *clusterCloud);
    //        int preSegmentsSize = segments.size();
    //        Inference::ExtractSegment(clusterCloud, segments, 4.f, 3);
    //        int currentSegmentsSize = segments.size();
    //        std::cout << i << " th cluster has " << currentSegmentsSize - preSegmentsSize << " segments" << std::endl;
    //    }
    //    if(segments.empty()){
    //        return;
    //    }
    //    for(int i=0; i<segments.size(); i++){
    //        if(segments[i].anchorIndices.size() > 10){
    //            segments[i].selectAnchor(5);
    //        }
    //        std::cout<<i<<" th segment anchorIndices size: "<<segments[i].anchorIndices.size()<<std::endl;
    //        //arrange direction to coincide with key pose trajectory
    //        segments[i].DecideOrientation(refLaneBoundaryCloud, kdtree_refLaneAnchor);
    //    }
    //    std::vector<std::pair<int, float>> disp(segments.size());
    //    for(int i=0; i<segments.size(); i++){
    //        disp[i] = std::make_pair(i, 10000.f);
    //    }
    //    for(int i=0; i<segments.size(); i++){
    //        int ncount = 0;
    //        float accuDisp = 0.f;
    //        float radius = 20.f;
    //        for(auto anchorIndex : segments[i].anchorIndices){
    //            std::vector<int> neighbors;
    //            std::vector<float> pointRadiusSquaredDistance;
    //            kdtree_refLaneAnchor->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
    //            if(neighbors.empty()){
    //                continue;
    //            }
    //            int neighborIndex = neighbors.front();
    //            if(neighborIndex<1){
    //                continue;
    //            }
    //            Eigen::Vector3f direction = refLaneAnchorCloud->at(neighborIndex).getVector3fMap() -
    //                    refLaneAnchorCloud->at(neighborIndex-1).getVector3fMap();
    //            Eigen::Vector2f nx = direction.head(2).normalized();
    //            Eigen::Vector2f ny(-nx(1), nx(0));
    //            accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() -
    //                    refLaneAnchorCloud->at(neighborIndex).getVector3fMap()).head(2).dot(ny);
    //            ncount++;
    //        }
    //        if(ncount > 0){
    //            disp[i].second = accuDisp/ncount;
    //        }
    //        std::cout<<i<<" th segment, disp: "<<disp[i].second<<std::endl;
    //    }
    //
    ////    nlohmann::json obj;
    ////    obj["base"]["lat"] = 0.0;
    ////    obj["base"]["lon"] = 0.0;
    ////    obj["base"]["alt"] = 0.0;
    //
    //    for(int i=0; i<segments.size(); i++){
    ////        if(segments[i].cloud !=NULL && segments[i].cloud->size()>0){
    ////            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[i].cloud);
    ////        }
    //
    //        pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //        pcl::copyPointCloud(*segments[i].cloud, segments[i].anchorIndices, *anchorCloud);
    ////        if(anchorCloud!=NULL && anchorCloud->size()>0){
    ////            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_anchorCloud.pcd", *anchorCloud);
    ////        }
    //
    //        segments[i].QpSpline();
    //        pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[i].smoothTrjCloud();
    //        if(trjCloud!=NULL && trjCloud->size()>0){
    //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_trjCloud.pcd", *trjCloud);
    //        }
    //
    ////        segments[i].addSegmentCoeff(obj, i);
    //    }
    //
    ////    std::ofstream fid_laneBoundary(outputDir+"/roadBoundary.json");
    ////    //fid_laneBoundary << std::setw(4) << obj << std::endl;
    ////    fid_laneBoundary<< obj << std::endl;
    //}
  
    void ProcessRoadBoundary::run_qzc_lane_center_in_road(std::string dataDir, std::string pcdPath, std::string refFile,
                                                  std::string outputDir, std::string label)
    {
        bool debug_log = false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lane_center(new pcl::PointCloud<pcl::PointXYZ>);
        std::map<int, int> indexMap;// 用于构建cloud_boundary与条件滤波后的点云的索引对应关系
        int rawCloudIndex = 0;

        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(pcdPath, *pc_ptr);
    
        // 条件滤波
        pcl::PointCloud<ConRmPointType>::Ptr filtercloud_lane_center(new pcl::PointCloud<ConRmPointType>);
        pclFilter::ConditionalRemoval(pc_ptr, filtercloud_lane_center, "cloud_bev_center_line", 1);  

        //统计滤波
        pcl::PointCloud<ConRmPointType>::Ptr laneSegCloud(new pcl::PointCloud<ConRmPointType>);
        pclFilter::StatisticalOutlierRemoval(filtercloud_lane_center, laneSegCloud, 50, 1.0f);
        if(laneSegCloud->empty()) {
            return;
        }
        pcl::io::savePCDFileBinary(outputDir + "/laneSegCloud_Statistical.pcd", *laneSegCloud);

        for (const auto& pcl_p : laneSegCloud->points) {
            cloud_lane_center->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
            indexMap[cloud_lane_center->size() - 1] = rawCloudIndex++;
        }
        if (cloud_lane_center->empty())
            return;

        // 获取参考线相关信息
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        get_ref_info(refFile, outputDir, cloud_lane_center, refLaneBoundaryCloud, refLaneAnchorCloud, kdtree_refLaneAnchor);

        std::cout << "[lane_center]filtered cloud size: " << cloud_lane_center->points.size() << std::endl;

        //聚类
        // TODO:qzc split curve in Y-style lane
        std::vector<std::vector<int>> clusters;
        float radius = 0.5f;
        int minPts = 20;
        Inference::DBSCAN(cloud_lane_center, radius, minPts, clusters);
        std::cout<<"[lane_center]cluster size: "<<clusters.size()<<std::endl;

        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i < clusters.size(); ++i) {
            if(clusters[i].size() < 10) //按照1米20个点 要求长度最少5米 修改背景：发现很多被误删的 将条件放宽，尽可能都参与建模
                continue;

            for (int j = 0; j < clusters[i].size(); j++) {
                pcl::PointXYZ curPXYZ = cloud_lane_center->at(clusters[i][j]);
                clustersCloud->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            }
        }

        pcl::io::savePCDFileBinary(outputDir + "/reg_clustersCloud.pcd", *clustersCloud);

        std::vector<Inference::Segment> segments;

        // generate segment
        for (int i = 0; i < clusters.size(); i++) {
            std::cout << "[lane_center] " << i << " th cluster size: " << clusters[i].size() << std::endl;
            if (clusters[i].size() < 10)
                continue;

            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud_lane_center, clusters[i], *clusterCloud);

            // 构建clusterCloud与统计滤波后的点云的索引对应关系
            std::map<int, int> clusterIndexMap;           
            std::map<int, bool> clusterIndexMap_is_segmented; 
            for (size_t j = 0; j < clusters[i].size(); ++j) {
                int cloudLaneCenterIndex = clusters[i][j];
                auto it = indexMap.find(cloudLaneCenterIndex);
                if (it != indexMap.end()) {
                    clusterIndexMap[j] = it->second;
                    clusterIndexMap_is_segmented[j] = false;
                } else {
                    // std::cout << "[lane_center] can not find Index in indexMap: " << cloudLaneCenterIndex << std::endl;
                }
            }
            int preSegmentsSize = segments.size();
            Inference::ExtractSegment(clusterCloud, segments, 4.f, 3, 2);

            // 更新segmentCloudIndexMap,构建segmentCloud与条件滤波后的点云的索引对应关系
            // average direction_rough
            Eigen::Vector2f direction_rough; 
            for (size_t k = preSegmentsSize; k < segments.size(); ++k) {
                for (size_t m = 0; m < segments[k].segmentCloudIndexMap.size(); ++m) {
                    int clusterIndex = segments[k].segmentCloudIndexMap[m];
                    auto it = clusterIndexMap.find(clusterIndex);
                    if (it != clusterIndexMap.end()) {
                        segments[k].segmentCloudIndexMap[m] = it->second;
                        clusterIndexMap_is_segmented[clusterIndex] = true;
                    } else {
                        // std::cout << "[lane_center] can not find Index in clusterIndexMap: " << clusterIndex << std::endl;
                    }
                }

                if(k == segments.size()-1){
                    direction_rough = segments[k].direction_rough;
                }
            }

            // 此处再聚类一次，针对分合流
            {
                std::vector<int> cluster_diverses;
                for (const auto& pair : clusterIndexMap_is_segmented)
                {
                    if (pair.second == false)
                    {
                        auto it = clusterIndexMap.find(pair.first);
                        if (it != clusterIndexMap.end()) {
                            cluster_diverses.push_back(it->second);
                        }
                    }
                }

                bool ret = ProcessSkeletonCluster(i, cloud_lane_center, cluster_diverses, indexMap, outputDir, segments, direction_rough);
            }

            int currentSegmentsSize = segments.size();
            if (debug_log)
            {
                std::cout << "[lane_center] " << i << " th cluster has " << currentSegmentsSize - preSegmentsSize << " segments" << std::endl;
            }
        }

        if (segments.empty()) {
            return;
        }

        for (int i = 0; i < segments.size(); i++) {
            //        if(segments[i].anchorIndices.size() > 10){
            //            segments[i].selectAnchor(5);
            //        }
            if (debug_log)
            {
                std::cout << "[lane_center] " << i << " th segment anchorIndices size: " << segments[i].anchorIndices.size() << std::endl;
            }
            // arrange direction to coincide with key pose trajectory
            segments[i].DecideOrientation(refLaneAnchorCloud, kdtree_refLaneAnchor);
        }
        std::vector<std::pair<int, float>> disp(segments.size());
        for (int i = 0; i < segments.size(); i++) {
            disp[i] = std::make_pair(i, 10000.f);
        }

        for (int i = 0; i < segments.size(); i++) {
            int ncount = 0;
            float accuDisp = 0.f;
            float radius = 20.f;
            for (auto anchorIndex : segments[i].anchorIndices) {
                std::vector<int> neighbors;
                std::vector<float> pointRadiusSquaredDistance;
                kdtree_refLaneAnchor->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
                if (neighbors.empty()) {
                    continue;
                }
                int neighborIndex = neighbors.front();
                if (neighborIndex < 1) {
                    continue;
                }
                Eigen::Vector3f direction = refLaneAnchorCloud->at(neighborIndex).getVector3fMap() -
                                            refLaneAnchorCloud->at(neighborIndex - 1).getVector3fMap();
                Eigen::Vector2f nx = direction.head(2).normalized();
                Eigen::Vector2f ny(-nx(1), nx(0));
                accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() -
                             refLaneAnchorCloud->at(neighborIndex).getVector3fMap())
                                .head(2)
                                .dot(ny);
                ncount++;
            }
            if (ncount > 0) {
                disp[i].second = accuDisp / ncount;
            }
            if (debug_log)
            {
                std::cout << "[lane_center] " << i << " th segment, disp: " << disp[i].second << std::endl;
            }
        }

        for (int i = 0; i < segments.size(); i++) {
            //        if(segments[i].cloud !=NULL && segments[i].cloud->size()>0){
            //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[i].cloud);
            //        }
            //
            //        pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
            //        pcl::copyPointCloud(*segments[i].cloud, segments[i].anchorIndices, *anchorCloud);
            //        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_anchorCloud.pcd", *anchorCloud);
            segments[i].QpSpline();
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[i].smoothTrjCloud2();

            pcl::PointCloud<PointElement>::Ptr lanecenter_output(new pcl::PointCloud<PointElement>);
            generateOutputCloud(i, trjCloud, segments[i], laneSegCloud, lanecenter_output, 12);// 12：车道中心线

            if (lanecenter_output != NULL && lanecenter_output->size() > 0) {
                pcl::io::savePCDFileBinary(outputDir + "/" + std::to_string(i) + "_trjCloud.pcd", *lanecenter_output);
            }
        }
    }


    void ProcessRoadBoundary::run_qzc_fin_in_road(std::string dataDir, std::string pcdPath, std::string refFile,
                                                  std::string outputDir, std::string label)
    {
        bool debug_log = false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        std::map<int, int> indexMap;// 用于构建cloud_boundary与条件滤波后的点云的索引对应关系
        int rawCloudIndex = 0;

        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(pcdPath, *pc_ptr);

        // 条件滤波
        pcl::PointCloud<ConRmPointType>::Ptr filtercloud_boundary(new pcl::PointCloud<ConRmPointType>);
        pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, "cloud_bev_label_1", 1);  

        //统计滤波
        pcl::PointCloud<ConRmPointType>::Ptr laneSegCloud(new pcl::PointCloud<ConRmPointType>);
        pclFilter::StatisticalOutlierRemoval(filtercloud_boundary, laneSegCloud, 50, 1.0f);
        if(laneSegCloud->empty()) {
            return;
        }
        pcl::io::savePCDFileBinary(outputDir + "/laneSegCloud_Statistical.pcd", *laneSegCloud);

        for (const auto& pcl_p : laneSegCloud->points) {
            cloud_boundary->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
            indexMap[cloud_boundary->size() - 1] = rawCloudIndex++;
        }
        if (cloud_boundary->empty())
            return;

        // 获取参考线相关信息
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        get_ref_info(refFile, outputDir, cloud_boundary, refLaneBoundaryCloud, refLaneAnchorCloud, kdtree_refLaneAnchor);

        std::cout << "[lane_boundary]filtered cloud size: " << cloud_boundary->points.size() << std::endl;

        //聚类
        // TODO:qzc split curve in Y-style lane
        std::vector<std::vector<int>> clusters;
        float radius = 0.5f;
        int minPts = 20;
        Inference::DBSCAN(cloud_boundary, radius, minPts, clusters);
        std::cout<<"[lane_boundary]cluster size: "<<clusters.size()<<std::endl;

        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i < clusters.size(); ++i) {
            if(clusters[i].size() < 10) //按照1米20个点 要求长度最少5米 修改背景：发现很多被误删的 将条件放宽，尽可能都参与建模
                continue;

            for (int j = 0; j < clusters[i].size(); j++) {
                pcl::PointXYZ curPXYZ = cloud_boundary->at(clusters[i][j]);
                clustersCloud->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            }
        }

        pcl::io::savePCDFileBinary(outputDir + "/reg_clustersCloud.pcd", *clustersCloud);

        std::vector<Inference::Segment> segments;

        // generate segment
        for (int i = 0; i < clusters.size(); i++) {
            if (debug_log)
            {
                std::cout << "[lane_boundary] " << i << " th cluster size: " << clusters[i].size() << std::endl;
            }
            
            if (clusters[i].size() < 10)
                continue;

            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud_boundary, clusters[i], *clusterCloud);

            // 构建clusterCloud与统计滤波后的点云的索引对应关系
            std::map<int, int> clusterIndexMap;            
            std::map<int, bool> clusterIndexMap_is_segmented;            
            for (size_t j = 0; j < clusters[i].size(); ++j) {
                int cloudBoundaryIndex = clusters[i][j];
                auto it = indexMap.find(cloudBoundaryIndex);
                if (it != indexMap.end()) {
                    clusterIndexMap[j] = it->second;
                    clusterIndexMap_is_segmented[j] = false;
                } else {
                    // std::cout << "[lane_boundary] can not find Index in indexMap: " << cloudBoundaryIndex << std::endl;
                }
            }
            int preSegmentsSize = segments.size();
            Inference::ExtractSegment(clusterCloud, segments, 4.f, 3, 2);

            // 更新segmentCloudIndexMap,构建segmentCloud与条件滤波后的点云的索引对应关系
            // average direction_rough
            Eigen::Vector2f direction_rough; 
            for (size_t k = preSegmentsSize; k < segments.size(); ++k) {
                for (size_t m = 0; m < segments[k].segmentCloudIndexMap.size(); ++m) {
                    int clusterIndex = segments[k].segmentCloudIndexMap[m];
                    auto it = clusterIndexMap.find(clusterIndex);
                    if (it != clusterIndexMap.end()) {
                        segments[k].segmentCloudIndexMap[m] = it->second;
                        clusterIndexMap_is_segmented[clusterIndex] = true;
                    } else {
                        // std::cout << "[lane_boundary] can not find Index in clusterIndexMap: " << clusterIndex << std::endl;
                    }
                }

                if(k == segments.size()-1){
                    direction_rough = segments[k].direction_rough;
                }
            }

            if(0) {
                for (int ii = 0; ii < segments.size(); ii++) {
                    if (ii == segments.size()-1)
                    {
                        if(segments[ii].cloud !=NULL && segments[ii].cloud->size()>0){
                            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(ii)+"_segmentCloud.pcd", *segments[ii].cloud);
                        }
                
                        pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
                        pcl::copyPointCloud(*segments[ii].cloud, segments[ii].anchorIndices, *anchorCloud);
                        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(ii)+"_anchorCloud.pcd", *anchorCloud);
                    }
                    
                }
            }

            // 此处再聚类一次，针对分合流
            {
                std::vector<int> cluster_diverses;
                for (const auto& pair : clusterIndexMap_is_segmented)
                {
                    if (pair.second == false)
                    {
                        auto it = clusterIndexMap.find(pair.first);
                        if (it != clusterIndexMap.end()) {
                            cluster_diverses.push_back(it->second);
                        }
                    }
                }

                bool ret = ProcessSkeletonCluster(i, cloud_boundary, cluster_diverses, indexMap, outputDir, segments, direction_rough);
            }

            int currentSegmentsSize = segments.size();
            if (debug_log)
            {
                std::cout << "[lane_boundary] " << i << " th cluster has " << currentSegmentsSize - preSegmentsSize << " segments" << std::endl;
            }
            
        }

        if (segments.empty()) {
            return;
        }

        for (int i = 0; i < segments.size(); i++) {
            //        if(segments[i].anchorIndices.size() > 10){
            //            segments[i].selectAnchor(5);
            //        }
            
            // std::cout << "[lane_boundary] " << i << " th segment anchorIndices size: " << segments[i].anchorIndices.size() << std::endl;

            // arrange direction to coincide with key pose trajectory
            segments[i].DecideOrientation(refLaneAnchorCloud, kdtree_refLaneAnchor);
        }
        std::vector<std::pair<int, float>> disp(segments.size());
        for (int i = 0; i < segments.size(); i++) {
            disp[i] = std::make_pair(i, 10000.f);
        }

        for (int i = 0; i < segments.size(); i++) {
            int ncount = 0;
            float accuDisp = 0.f;
            float radius = 20.f;
            for (auto anchorIndex : segments[i].anchorIndices) {
                std::vector<int> neighbors;
                std::vector<float> pointRadiusSquaredDistance;
                kdtree_refLaneAnchor->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
                if (neighbors.empty()) {
                    continue;
                }
                int neighborIndex = neighbors.front();
                if (neighborIndex < 1) {
                    continue;
                }
                Eigen::Vector3f direction = refLaneAnchorCloud->at(neighborIndex).getVector3fMap() -
                                            refLaneAnchorCloud->at(neighborIndex - 1).getVector3fMap();
                Eigen::Vector2f nx = direction.head(2).normalized();
                Eigen::Vector2f ny(-nx(1), nx(0));
                accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() -
                             refLaneAnchorCloud->at(neighborIndex).getVector3fMap())
                                .head(2)
                                .dot(ny);
                ncount++;
            }
            if (ncount > 0) {
                disp[i].second = accuDisp / ncount;
            }
            if (debug_log)
            {
                std::cout << "[lane_boundary] " << i << " th segment, disp: " << disp[i].second << std::endl;
            }
        }

        for (int i = 0; i < segments.size(); i++) {
            //        if(segments[i].cloud !=NULL && segments[i].cloud->size()>0){
            //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[i].cloud);
            //        }
            //
            //        pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
            //        pcl::copyPointCloud(*segments[i].cloud, segments[i].anchorIndices, *anchorCloud);
            //        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_anchorCloud.pcd", *anchorCloud);
            segments[i].QpSpline();
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[i].smoothTrjCloud2();

            pcl::PointCloud<PointElement>::Ptr laneline_output(new pcl::PointCloud<PointElement>);
            generateOutputCloud(i, trjCloud, segments[i], laneSegCloud, laneline_output, 11);// 11：车道线

            if (laneline_output != NULL && laneline_output->size() > 0) {
                pcl::io::savePCDFileBinary(outputDir + "/" + std::to_string(i) + "_trjCloud.pcd", *laneline_output);
            }
        }
    }


    void ProcessRoadBoundary::run_yxx_fin_in_road(std::string dataDir, std::string pcdPath, std::string refFile,
                                                  std::string outputDir, std::string label)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        if (label == "label")
        {
            // 进行点云边缘估计
            road_cloud_edge_precess(dataDir, pcdPath, refFile, outputDir);
            std::string edge_cloud = outputDir + "/" + "boundaries.pcd";
            std::ifstream infile(edge_cloud);
            if (!infile.good())
                return;

            pcl::io::loadPCDFile(edge_cloud, *cloud_boundary);
            std::cout << "->加载点云" << cloud_boundary->points.size() << "个" << std::endl;
        }
        else if (label == "cloud_line_seg")
        {
            pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
            pcl::io::loadPCDFile(pcdPath, *pc_ptr);
            pcl::PointCloud<MyColorPointType>::Ptr filtercloud_boundary(new pcl::PointCloud<MyColorPointType>);
            pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, label, 1);
            for (auto iter = filtercloud_boundary->begin(); iter != filtercloud_boundary->end(); iter++)
            {
                MyColorPointType &pcl_p = (*iter);
                cloud_boundary->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
            }
        }
        else if (label == "cloud_pano_seg")
        {
            pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
            pcl::io::loadPCDFile(pcdPath, *pc_ptr);
            pcl::PointCloud<MyColorPointType>::Ptr filtercloud_boundary(new pcl::PointCloud<MyColorPointType>);
            // pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, label, 3);
            pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, label, 24);
            for (auto iter = filtercloud_boundary->begin(); iter != filtercloud_boundary->end(); iter++)
            {
                MyColorPointType &pcl_p = (*iter);
                cloud_boundary->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
            }
        }
        else if (label == "cloud_bev_label")
        {
            // 坤哥37类别转换映射
            //  if (type >= 1 && type <= 13)
            //  { // 白色线类
            //      return 1;
            //  }
            //  if (type >= 14 && type <= 20)
            //  { // 黄色线类
            //      return 2;
            //  }
            //  if (type >= 21 && type <= 27)
            //  { // 道路边界类
            //      return 3;
            //  }
            //  if (type == 28)
            //  { // 停止线
            //      return 4;
            //  }
            //  if (type == 29)
            //  { // 人行道
            //      return 5;
            //  }
            /////**************************新cloud_bev_label***************************//////
            // int label_1 = 0;
            // {
            //     // clang-format off
            //     if (raw_id_37 >= 1 && raw_id_37 <= 20) label_1 = 1; // 车道线
            //     else if (raw_id_37 == 28)              label_1 = 2; // stopline
            //     else if (raw_id_37 == 29)              label_1 = 3; // crosswalk
            //     // clang-format on
            // }
            // int label_2 = 0;
            // {
            //     // clang-format off
            //     if (raw_id_37 >= 21 && raw_id_37 <= 27) label_2 = 1; // road_boundary
            //     else if (raw_id_37 == 36)               label_2 = 2; // lane_center
            //     // clang-format on
            // }
            // int color = 0;
            // {
            //     // clang-format off
            //     if  (raw_id_37 >= 7 && raw_id_37 <= 13)      color = 1; // white
            //     else if (raw_id_37 >= 14 && raw_id_37 <= 20) color = 2; // yellow
            //     // clang-format on
            // }
            // int shape = 0;
            // {
            //     // clang-format off
            //          if (raw_id_37 ==1  || raw_id_37 == 8  || raw_id_37 == 15) shape = 1; // solid
            //     else if (raw_id_37 ==2  || raw_id_37 == 9  || raw_id_37 == 16) shape = 2; // dashed
            //     else if (raw_id_37 ==3  || raw_id_37 == 10 || raw_id_37 == 17) shape = 3; // double_solid
            //     else if (raw_id_37 ==4  || raw_id_37 == 11 || raw_id_37 == 18) shape = 4; // double_dashed
            //     else if (raw_id_37 ==5  || raw_id_37 == 12 || raw_id_37 == 19) shape = 5; // left_dashed_right_solid
            //     else if (raw_id_37 ==6  || raw_id_37 == 13 || raw_id_37 == 20) shape = 6; // left_solid_right_dashed
            //     // clang-format on
            // }
            pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
            pcl::io::loadPCDFile(pcdPath, *pc_ptr);
            pcl::PointCloud<MyColorPointType>::Ptr filtercloud_boundary(new pcl::PointCloud<MyColorPointType>);
            // pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, "cloud_bev_label_2", 1);
            pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, "cloud_bev_label", 3);
            for (auto iter = filtercloud_boundary->begin(); iter != filtercloud_boundary->end(); iter++)
            {
                MyColorPointType &pcl_p = (*iter);
                cloud_boundary->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
            }
        }

        if (cloud_boundary->points.empty())
            return;

        // 获取参考线相关信息
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        get_ref_info(refFile, outputDir, cloud_boundary, refLaneBoundaryCloud, refLaneAnchorCloud, kdtree_refLaneAnchor);

        
        // 创建一个normal点云
        std::cout << "road boundary size: " << cloud_boundary->points.size() << std::endl;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        EigenSolverEstimation(cloud_boundary, refLaneAnchorCloud, kdtree_refLaneAnchor, 1.0, cloud_normals);

        //        string objFullPath = outputDir + "/" + "Normal.obj";
        //        ofstream ofs(objFullPath);
        //        std::vector<std::vector<pcl::PointXYZ>> lineVec;
        //        for (int j = 0; j < cloud_normals->size(); j++)
        //        {
        //            std::vector<pcl::PointXYZ> curNormals;
        //            pcl::PointXYZ curPt = cloud_boundary->at(j);
        //            curNormals.push_back(curPt);
        //
        //            pcl::PointXYZ disPt;
        //            disPt.x = curPt.x + 0.5 * cloud_normals->at(j).normal_x;
        //            disPt.y = curPt.y + 0.5 * cloud_normals->at(j).normal_y;
        //            disPt.z = curPt.z + 0.5 * cloud_normals->at(j).normal_z;
        //            curNormals.push_back(disPt);
        //
        //            ofs << std::fixed;
        //            ofs << "v " << curPt.x << " " << curPt.y << " " << curPt.z << endl;
        //            ofs << "v " << disPt.x << " " << disPt.y << " " << disPt.z << endl;
        //            lineVec.push_back(curNormals);
        //        }

        //        int K = 0;
        //        for (int j = 0; j < lineVec.size(); j++) {
        //            if (lineVec[j].size() <= 1) {
        //                continue;
        //            }
        //            ofs << "l ";
        //            for (int k = 0; k < lineVec[j].size(); k++) {
        //                ofs << ++K << " ";
        //            }
        //            ofs << endl;
        //        }
        //
        //        ofs.close();

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;                                                   // 创造区域生长分割对象
        reg.setMinClusterSize(10);                                                                            // 设置一个聚类需要的最小点数，聚类小于阈值的结果将被舍弃
        reg.setMaxClusterSize(1000000);                                                                       // 设置一个聚类需要的最大点数，聚类大于阈值的结果将被舍弃
        reg.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>)); // 设置搜索方法
                                                                                                              //        reg.setResidualThreshold (0.5); // 设置搜索的近邻点数目
        reg.setNumberOfNeighbours(20);
        reg.setInputCloud(cloud_boundary);            // 设置输入点云
        reg.setInputNormals(cloud_normals);           // 设置输入点云
        reg.setSmoothnessThreshold(5 / 180.0 * M_PI); // 设置平滑阀值
        reg.setCurvatureThreshold(1);                 // 设置曲率阀值

        // 以下两行用于启动分割算法，并返回聚类向量
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters); // 获取聚类的结果，分割结果保存在点云索引的向量中

        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i < clusters.size(); ++i)
        {
            for (int j = 0; j < clusters[i].indices.size(); j++)
            {
                pcl::PointXYZ curPXYZ = cloud_boundary->at(clusters[i].indices[j]);
                clustersCloud->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            }
        }
        std::cout << "save reg_clustersCloud " << std::endl;
        pcl::io::savePCDFileBinary(outputDir + "/reg_clustersCloud.pcd", *clustersCloud);

        std::vector<Inference::Segment> segments;

        // generate segment
        for (int i = 0; i < clusters.size(); i++)
        {
            std::cout << i << " th cluster size: " << clusters[i].indices.size() << std::endl;
            if (clusters[i].indices.size() < 10)
                continue;

            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud_boundary, clusters[i], *clusterCloud);
            int preSegmentsSize = segments.size();
            Inference::ExtractSegment(clusterCloud, segments, 4.f, 3, 2);
            int currentSegmentsSize = segments.size();
            std::cout << i << " th cluster has " << currentSegmentsSize - preSegmentsSize << " segments" << std::endl;
        }
        if (segments.empty())
        {
            return;
        }
        for (int i = 0; i < segments.size(); i++)
        {
            //        if(segments[i].anchorIndices.size() > 10){
            //            segments[i].selectAnchor(5);
            //        }
            std::cout << i << " th segment anchorIndices size: " << segments[i].anchorIndices.size() << std::endl;
            // arrange direction to coincide with key pose trajectory
            segments[i].DecideOrientation(refLaneAnchorCloud, kdtree_refLaneAnchor);
        }
        std::vector<std::pair<int, float>> disp(segments.size());
        for (int i = 0; i < segments.size(); i++)
        {
            disp[i] = std::make_pair(i, 10000.f);
        }
        for (int i = 0; i < segments.size(); i++)
        {
            int ncount = 0;
            float accuDisp = 0.f;
            float radius = 20.f;
            for (auto anchorIndex : segments[i].anchorIndices)
            {
                std::vector<int> neighbors;
                std::vector<float> pointRadiusSquaredDistance;
                kdtree_refLaneAnchor->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
                if (neighbors.empty())
                {
                    continue;
                }
                int neighborIndex = neighbors.front();
                if (neighborIndex < 1)
                {
                    continue;
                }
                Eigen::Vector3f direction = refLaneAnchorCloud->at(neighborIndex).getVector3fMap() -
                                            refLaneAnchorCloud->at(neighborIndex - 1).getVector3fMap();
                Eigen::Vector2f nx = direction.head(2).normalized();
                Eigen::Vector2f ny(-nx(1), nx(0));
                accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() -
                             refLaneAnchorCloud->at(neighborIndex).getVector3fMap())
                                .head(2)
                                .dot(ny);
                ncount++;
            }
            if (ncount > 0)
            {
                disp[i].second = accuDisp / ncount;
            }
            std::cout << i << " th segment, disp: " << disp[i].second << std::endl;
        }

        for (int i = 0; i < segments.size(); i++)
        {
            //        if(segments[i].cloud !=NULL && segments[i].cloud->size()>0){
            //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[i].cloud);
            //        }
            //
            //        pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
            //        pcl::copyPointCloud(*segments[i].cloud, segments[i].anchorIndices, *anchorCloud);
            //        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_anchorCloud.pcd", *anchorCloud);
            segments[i].QpSpline();
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[i].smoothTrjCloud2();
            if (trjCloud != NULL && trjCloud->size() > 0)
            {
                pcl::io::savePCDFileBinary(outputDir + "/" + std::to_string(i) + "_trjCloud.pcd", *trjCloud);
            }
        }
    }

    void ProcessRoadBoundary::run_yxx_all_road(std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir)
    {
        std::string pcdPath1 = "/home/test/data/102067/fsd_mapbuild_out/547739448/roadBoundaryFromSeg/AAAA.pcd";
        std::string pcdPath2 = "/home/test/data/102067/fsd_mapbuild_out/723041994/roadBoundaryFromSeg/AAAA.pcd";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(pcdPath1, *cloud1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(pcdPath2, *cloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud = cloud1;
        *cloud += *cloud2;
        // pcl::io::savePCDFileBinary(outputDir + "/" + "all_road_2.pcd", *cloud);

        //------------------------计算法向量---------------------------
        std::cout << "->计算法向量" << std::endl;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
        normEst.setRadiusSearch(0.5);
        normEst.compute(*normals);

        std::cout << "->点云边缘估计" << std::endl;
        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
        boundEst.setInputCloud(cloud);
        boundEst.setInputNormals(normals);
        boundEst.setRadiusSearch(0.5);
        boundEst.setAngleThreshold(M_PI / 2);
        boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
        boundEst.compute(boundaries);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < cloud->points.size(); i++)
        {

            if (boundaries[i].boundary_point > 0)
            {
                cloud_boundary->push_back(cloud->points[i]);
            }
        }

        std::cout << "边界点个数:" << cloud_boundary->points.size() << std::endl;
        if (cloud_boundary->empty() || cloud_boundary->points.empty())
        {
            return;
        }
        // pcl::io::savePCDFileBinary(outputDir + "/" + "all_boundaries.pcd", *cloud_boundary);

        //------------------------半径滤波---------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_statistical(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::RadiusOutlierRemoval(cloud_boundary, cloud_statistical, 0.5, 2);
        if (cloud_statistical->empty() || cloud_statistical->points.empty())
        {
            return;
        }
        // pcl::io::savePCDFileBinary(outputDir + "/" + "all_boundaries_RadiusOutlier.pcd", *cloud_statistical);

        // 获取参考线相关信息
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        get_ref_info(refFile, outputDir, cloud_statistical, refLaneBoundaryCloud, refLaneAnchorCloud, kdtree_refLaneAnchor);

        // 创建一个normal点云
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        EigenSolverEstimation(cloud_statistical, refLaneAnchorCloud, kdtree_refLaneAnchor, 1.0, cloud_normals);
        if (cloud_statistical->empty() || cloud_statistical->points.empty())
        {
            return;
        }
        // pcl::io::savePCDFileBinary(outputDir + "/" + "all_boundaries_filter.pcd", *cloud_statistical);

        // 针对点云进行聚类
        cloud = cloud_statistical;
        std::vector<std::vector<int>> clusters;
        float radius = 0.5;
        int minPts = 3;
        Inference::DBSCAN(cloud, radius, minPts, clusters);
        std::vector<Inference::Segment> segments;

        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i < clusters.size(); ++i)
        {
            for (int j = 0; j < clusters[i].size(); j++)
            {
                pcl::PointXYZ curPXYZ = cloud->at(clusters[i][j]);
                clustersCloud->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            }
        }
        if (clustersCloud->empty() || clustersCloud->points.empty())
        {
            return;
        }
        // pcl::io::savePCDFileBinary(outputDir + "/reg_clustersCloud.pcd", *clustersCloud);

        // generate segment
        for (int i = 0; i < clusters.size(); i++)
        {
            std::cout << i << " th cluster size: " << clusters[i].size() << std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud, clusters[i], *clusterCloud);
            int preSegmentsSize = segments.size();
            Inference::ExtractSegment(clusterCloud, segments, 4.f, 3);
            int currentSegmentsSize = segments.size();
            std::cout << i << " th cluster has " << currentSegmentsSize - preSegmentsSize << " segments" << std::endl;
        }
        if (segments.empty())
        {
            return;
        }
        for (int i = 0; i < segments.size(); i++)
        {
            if (segments[i].anchorIndices.size() > 10)
            {
                segments[i].selectAnchor(5);
            }
            std::cout << i << " th segment anchorIndices size: " << segments[i].anchorIndices.size() << std::endl;
            // arrange direction to coincide with key pose trajectory
            segments[i].DecideOrientation(refLaneBoundaryCloud, kdtree_refLaneAnchor);
        }
        std::vector<std::pair<int, float>> disp(segments.size());
        for (int i = 0; i < segments.size(); i++)
        {
            disp[i] = std::make_pair(i, 10000.f);
        }
        for (int i = 0; i < segments.size(); i++)
        {
            int ncount = 0;
            float accuDisp = 0.f;
            float radius = 20.f;
            for (auto anchorIndex : segments[i].anchorIndices)
            {
                std::vector<int> neighbors;
                std::vector<float> pointRadiusSquaredDistance;
                kdtree_refLaneAnchor->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
                if (neighbors.empty())
                {
                    continue;
                }
                int neighborIndex = neighbors.front();
                if (neighborIndex < 1)
                {
                    continue;
                }
                Eigen::Vector3f direction = refLaneAnchorCloud->at(neighborIndex).getVector3fMap() -
                                            refLaneAnchorCloud->at(neighborIndex - 1).getVector3fMap();
                Eigen::Vector2f nx = direction.head(2).normalized();
                Eigen::Vector2f ny(-nx(1), nx(0));
                accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() -
                             refLaneAnchorCloud->at(neighborIndex).getVector3fMap())
                                .head(2)
                                .dot(ny);
                ncount++;
            }
            if (ncount > 0)
            {
                disp[i].second = accuDisp / ncount;
            }
            std::cout << i << " th segment, disp: " << disp[i].second << std::endl;
        }

        //    nlohmann::json obj;
        //    obj["base"]["lat"] = 0.0;
        //    obj["base"]["lon"] = 0.0;
        //    obj["base"]["alt"] = 0.0;

        for (int i = 0; i < segments.size(); i++)
        {
            //        if(segments[i].cloud !=NULL && segments[i].cloud->size()>0){
            //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[i].cloud);
            //        }

            pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*segments[i].cloud, segments[i].anchorIndices, *anchorCloud);
            //        if(anchorCloud!=NULL && anchorCloud->size()>0){
            //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_anchorCloud.pcd", *anchorCloud);
            //        }

            segments[i].QpSpline();
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[i].smoothTrjCloud();
            if (trjCloud != NULL && trjCloud->size() > 0)
            {
                pcl::io::savePCDFileBinary(outputDir + "/" + std::to_string(i) + "_trjCloud.pcd", *trjCloud);
            }

            //        segments[i].addSegmentCoeff(obj, i);
        }
    }

    /*
        breif: add type, modified by ccj, 2024-12-26
    */
    void ProcessRoadBoundary::run_yxx_fin_add_type(std::string dataDir, std::string pcdPath, std::string refFile,
                                                  std::string outputDir, std::string label)
    {
        bool debug_log = false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        std::map<int, int> indexMap;// 用于构建cloud_boundary与条件滤波后的点云的索引对应关系
        int rawCloudIndex = 0;

        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(pcdPath, *pc_ptr);
        if (pcl::io::loadPCDFile(pcdPath, *pc_ptr) == -1) {
            std::cerr << "[road_boundary] Error: Unable to load PCD file: " << pcdPath << std::endl;
            return;
        }

        pcl::PointCloud<ConRmPointType>::Ptr filtercloud_boundary(new pcl::PointCloud<ConRmPointType>);
        // pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, "cloud_bev_label", 3);
        pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, "cloud_bev_center_line", 2);


        for (const auto& pcl_p : *filtercloud_boundary) {
            cloud_boundary->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
            indexMap[cloud_boundary->size() - 1] = rawCloudIndex++;
        }

        if (cloud_boundary->empty())
        {
            std::cerr << "[road_boundary] Error: cloud_boundary is empty after filtering." << std::endl;
            return;
        }

        // 获取参考线相关信息
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        get_ref_info(refFile, outputDir, cloud_boundary, refLaneBoundaryCloud, refLaneAnchorCloud, kdtree_refLaneAnchor);

        
        // 创建一个normal点云
        std::cout << "[road_boundary] cloud_boundary size: " << cloud_boundary->points.size() << std::endl;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        EigenSolverEstimationNew(cloud_boundary, refLaneAnchorCloud, kdtree_refLaneAnchor, 1.0, cloud_normals, indexMap);
        if (cloud_boundary->empty())
        {
            std::cerr << "[road_boundary] Error: cloud_boundary is empty after normal estimation." << std::endl;
            return;
        }

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;                                                   // 创造区域生长分割对象
        reg.setMinClusterSize(10);                                                                            // 设置一个聚类需要的最小点数，聚类小于阈值的结果将被舍弃
        reg.setMaxClusterSize(1000000);                                                                       // 设置一个聚类需要的最大点数，聚类大于阈值的结果将被舍弃
        reg.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>)); // 设置搜索方法
                                                                                                              //        reg.setResidualThreshold (0.5); // 设置搜索的近邻点数目
        reg.setNumberOfNeighbours(20);
        reg.setInputCloud(cloud_boundary);            // 设置输入点云
        reg.setInputNormals(cloud_normals);           // 设置输入点云
        reg.setSmoothnessThreshold(5 / 180.0 * M_PI); // 设置平滑阀值
        reg.setCurvatureThreshold(1);                 // 设置曲率阀值

        // 以下两行用于启动分割算法，并返回聚类向量
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters); // 获取聚类的结果，分割结果保存在点云索引的向量中

        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            for (size_t j = 0; j < clusters[i].indices.size(); j++)
            {
                pcl::PointXYZ curPXYZ = cloud_boundary->at(clusters[i].indices[j]);
                clustersCloud->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            }
        }
        pcl::io::savePCDFileBinary(outputDir + "/reg_clustersCloud.pcd", *clustersCloud);

        std::vector<Inference::Segment> segments;

        // generate segment
        std::cout << "[road_boundary] cluster size: " << clusters.size() << std::endl;
        for (size_t i = 0; i < clusters.size(); i++)
        {
            if (debug_log)
            {
                std::cout  << "[road_boundary] " << i << "th cluster size: " << clusters[i].indices.size() << std::endl;
            }
            if (clusters[i].indices.size() < 10)
                continue;

            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud_boundary, clusters[i], *clusterCloud);
            
            // 构建clusterCloud与条件滤波后的点云的索引对应关系
            std::map<int, int> clusterIndexMap;        
            std::map<int, bool> clusterIndexMap_is_segmented;    
            for (size_t j = 0; j < clusters[i].indices.size(); ++j) {
                int cloudBoundaryIndex = clusters[i].indices[j];
                auto it = indexMap.find(cloudBoundaryIndex);
                if (it != indexMap.end()) {
                    clusterIndexMap[j] = it->second;
                    clusterIndexMap_is_segmented[j] = false;
                } else {
                    // std::cout << "[road_boundary] can not find Index in indexMap: " << cloudBoundaryIndex << std::endl;
                }
            }

            int preSegmentsSize = segments.size();
            Inference::ExtractSegment(clusterCloud, segments, 4.f, 3, 2);

            // 更新segmentCloudIndexMap,构建segmentCloud与条件滤波后的点云的索引对应关系
            // average direction_rough
            Eigen::Vector2f direction_rough; 
            for (size_t k = preSegmentsSize; k < segments.size(); ++k) {
                for (size_t m = 0; m < segments[k].segmentCloudIndexMap.size(); ++m) {
                    int clusterIndex = segments[k].segmentCloudIndexMap[m];
                    auto it = clusterIndexMap.find(clusterIndex);
                    if (it != clusterIndexMap.end()) {
                        segments[k].segmentCloudIndexMap[m] = it->second;
                        clusterIndexMap_is_segmented[clusterIndex] = true;
                    } else {
                        // std::cout << "[road_boundary] can not find Index in clusterIndexMap: " << clusterIndex << std::endl;
                    }
                }

                if(k == segments.size()-1){
                    direction_rough = segments[k].direction_rough;
                }
            }

            // 此处再聚类一次，针对分合流
            {
                std::vector<int> cluster_diverses;
                for (const auto& pair : clusterIndexMap_is_segmented)
                {
                    if (pair.second == false)
                    {
                        auto it = clusterIndexMap.find(pair.first);
                        if (it != clusterIndexMap.end()) {
                            cluster_diverses.push_back(it->second);
                        }
                    }
                }

                bool ret = ProcessSkeletonCluster(i, cloud_boundary, cluster_diverses, indexMap, outputDir, segments, direction_rough);
            }

            int currentSegmentsSize = segments.size();
            if (debug_log)
            {
                std::cout  << "[road_boundary] "<< i << "th cluster has " << currentSegmentsSize - preSegmentsSize << " segments" << std::endl;
            }
        }
        if (segments.empty())
        {
            return;
        }
        for (size_t i = 0; i < segments.size(); i++)
        {
            //        if(segments[i].anchorIndices.size() > 10){
            //            segments[i].selectAnchor(5);
            //        }
            if (debug_log)
            {
                std::cout << "[road_boundary] " << i << "th segment anchorIndices size: " << segments[i].anchorIndices.size() << std::endl;
            }
            // arrange direction to coincide with key pose trajectory
            segments[i].DecideOrientation(refLaneAnchorCloud, kdtree_refLaneAnchor);
        }
        std::vector<std::pair<int, float>> disp(segments.size());
        for (size_t i = 0; i < segments.size(); i++)
        {
            disp[i] = std::make_pair(i, 10000.f);
        }
        for (size_t i = 0; i < segments.size(); i++)
        {
            int ncount = 0;
            float accuDisp = 0.f;
            float radius = 20.f;
            for (auto anchorIndex : segments[i].anchorIndices)
            {
                std::vector<int> neighbors;
                std::vector<float> pointRadiusSquaredDistance;
                kdtree_refLaneAnchor->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
                if (neighbors.empty())
                {
                    continue;
                }
                int neighborIndex = neighbors.front();
                if (neighborIndex < 1)
                {
                    continue;
                }
                Eigen::Vector3f direction = refLaneAnchorCloud->at(neighborIndex).getVector3fMap() -
                                            refLaneAnchorCloud->at(neighborIndex - 1).getVector3fMap();
                Eigen::Vector2f nx = direction.head(2).normalized();
                Eigen::Vector2f ny(-nx(1), nx(0));
                accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() -
                             refLaneAnchorCloud->at(neighborIndex).getVector3fMap())
                                .head(2)
                                .dot(ny);
                ncount++;
            }
            if (ncount > 0)
            {
                disp[i].second = accuDisp / ncount;
            }
            if (debug_log)
            {
                std::cout << "[road_boundary] "<< i << " th segment, disp: " << disp[i].second << std::endl;
            }
        }

        std::cout << "[road_boundary] segments size: " << segments.size() << std::endl;
        for (size_t i = 0; i < segments.size(); i++)
        {
            //        if(segments[i].cloud !=NULL && segments[i].cloud->size()>0){
            //            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[i].cloud);
            //        }
            //
            //        pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
            //        pcl::copyPointCloud(*segments[i].cloud, segments[i].anchorIndices, *anchorCloud);
            //        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_anchorCloud.pcd", *anchorCloud);
            segments[i].QpSpline();
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[i].smoothTrjCloud2();
            
            pcl::PointCloud<PointElement>::Ptr roadBoundary_output(new pcl::PointCloud<PointElement>);
            generateOutputCloud(i, trjCloud, segments[i], filtercloud_boundary, roadBoundary_output, 10); // 10：道路边界

            if (roadBoundary_output != NULL && roadBoundary_output->size() > 0)
            {
                pcl::io::savePCDFileBinary(outputDir + "/" + std::to_string(i) + "_trjCloud.pcd", *roadBoundary_output);
            }   
        }
    }


    /*
        breif: 生成输出点云, modified by ccj, 2024-12-26
    */
    void ProcessRoadBoundary::generateOutputCloud(int index, pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud, 
                                                Inference::Segment segment, 
                                                pcl::PointCloud<ConRmPointType>::Ptr rawCloud, 
                                                pcl::PointCloud<PointElement>::Ptr outputCloud,
                                                int ele_type)
    {
        outputCloud->clear();

        const auto& anchorIndices = segment.anchorIndices;
        size_t anchorCount = anchorIndices.size();

        pcl::KdTreeFLANN<ConRmPointType>::Ptr kdtree_rawcloud(new pcl::KdTreeFLANN<ConRmPointType>);
        kdtree_rawcloud->setInputCloud(rawCloud);
        
        float radius = 0.5f;
        std::vector<int> neighbors;
        std::vector<float> pointRadiusSquaredDistance;

        for (int i = 0; i < trjCloud->size(); ++i) {
            PointElement point;
            point.x = trjCloud->points[i].x;
            point.y = trjCloud->points[i].y;
            point.z = trjCloud->points[i].z;
            point.ele_type = ele_type;
            point.id = index;
            point.index = i;
            point.heading = 0;
            point.score = 0;

            if(ele_type==11){
                neighbors.clear();
                pointRadiusSquaredDistance.clear();
                ConRmPointType tmppoint;
                tmppoint.x = trjCloud->points[i].x;
                tmppoint.y = trjCloud->points[i].y;
                tmppoint.z = trjCloud->points[i].z;
                kdtree_rawcloud->nearestKSearch(tmppoint, 1, neighbors, pointRadiusSquaredDistance);
                // std::cout << "----------------- "<< i << " th trjCloud, neighbors: " << neighbors.size() << ", Distance: " << pointRadiusSquaredDistance[0] << std::endl;
                
                if (neighbors.empty() || neighbors.front() < 1 || pointRadiusSquaredDistance[0] > 0.4){
                    point.type1 = 0;
                    point.type2 = 0;
                } else {
                    // int j = segment.segmentCloudIndexMap[neighbors[0]];
                    int j = neighbors[0];
                    point.type1 = rawCloud->points[j].cloud_bev_shape;
                    point.type2 = rawCloud->points[j].cloud_bev_color;
                }
            } else {
                // 获得样条系数索引
                auto coeffsIndex = segment.trjCloudIndexMap[i];
                if (coeffsIndex < 0 || coeffsIndex >= anchorCount - 1) {
                    std::cerr << "Error: Invalid coeffs index." << std::endl;
                    continue; 
                }

                // 获得锚点在segment中的索引
                int startAnchorSegmentIdx = anchorIndices[coeffsIndex];
                int endAnchorSegmentIdx = anchorIndices[coeffsIndex + 1];

                // 遍历锚点对之间所有点，检查属性是否发生变化，若发生变化，取距离最近的点的属性作为输出点的属性
                bool hasChanged = false;
                size_t nearestIndex = startAnchorSegmentIdx;
                float minDistance = std::numeric_limits<float>::max();

                for (size_t j = startAnchorSegmentIdx; j <= endAnchorSegmentIdx; ++j) {
                    const auto& currentPoint = rawCloud->points[segment.segmentCloudIndexMap[j]];
                    float distance = std::sqrt(std::pow(currentPoint.x - point.x, 2) + 
                                                std::pow(currentPoint.y - point.y, 2) + 
                                                std::pow(currentPoint.z - point.z, 2));

                    if (distance < minDistance) {
                            minDistance = distance;
                            nearestIndex = j;
                    }
                    // 检查属性变化
                    if ((currentPoint.cloud_bev_shape != rawCloud->points[segment.segmentCloudIndexMap[startAnchorSegmentIdx]].cloud_bev_shape) ||
                        (currentPoint.cloud_bev_color != rawCloud->points[segment.segmentCloudIndexMap[startAnchorSegmentIdx]].cloud_bev_color)) {                    
                        hasChanged = true;
                    }
                }

                if (hasChanged) {
                    point.type1 = rawCloud->points[segment.segmentCloudIndexMap[nearestIndex]].cloud_bev_shape;
                    point.type2 = rawCloud->points[segment.segmentCloudIndexMap[nearestIndex]].cloud_bev_color;

                } else {
                    point.type1 = rawCloud->points[segment.segmentCloudIndexMap[startAnchorSegmentIdx]].cloud_bev_shape;
                    point.type2 = rawCloud->points[segment.segmentCloudIndexMap[startAnchorSegmentIdx]].cloud_bev_color;
                }
            }
            
            outputCloud->push_back(point);
        }
    }


    bool ProcessRoadBoundary::ProcessSkeletonCluster(int index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                                  const std::vector<int> &cluster_diverses,
                                                  const std::map<int, int>& indexMap,
                                                  std::string outputDir,
                                                  std::vector<Inference::Segment>& segments,
                                                  Eigen::Vector2f direction_rough)
    {
        if (cluster_diverses.size() > 50) {
            // 曲线分割为两部分，并将一定范围内的点云当做一条曲线
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud_in, cluster_diverses, *clusterCloud);

            // 过滤比较短的线
            auto wlh = Inference::GetWLH(clusterCloud);
            if (wlh.x() < 2 && wlh.y() < 2 && wlh.z() < 2) {
                return false;
            }

            if(0) {
                std::string save_name(outputDir + "/cluster_" + std::to_string(index) + ".pcd");
                pcl::io::savePCDFileBinary(save_name, *clusterCloud);
            }

            // 1. 计算质心
            int point_size = clusterCloud->points.size();
            Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
            for(int i = 0; i < point_size; i++) {
                centroid += Eigen::Vector2f(clusterCloud->at(i).x, clusterCloud->at(i).y);
            }
            centroid = centroid/point_size;

            // 2. 中心化数据
            // 3. 计算协方差矩阵
            std::vector<cv::Point2f> centered_points;
            Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
            for(int i = 0; i < point_size; i++){
                Eigen::Vector2f pos(clusterCloud->at(i).x, clusterCloud->at(i).y);
                Eigen::Vector2f diff = pos - centroid;
                cov += diff*diff.transpose();

                centered_points.push_back(cv::Point2f(diff.x(), diff.y()));
            }
            // cov = cov/point_size;
            // cov = (cov + cov.transpose())/2;
            // Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es;
            // es.compute(cov);
            // Eigen::Vector2f eigenvalues = es.eigenvalues();
            // Eigen::Matrix2f eigenvectors = es.eigenvectors();

            // 4. 求解特征值和特征向量, 主方向为特征向量
            // Eigen::Vector2f main_direction1 = eigenvectors.col(1); // 第一主方向
            // Eigen::Vector2f main_direction2 = eigenvectors.col(0); // 第二主方向

            Eigen::Vector2f main_direction1 = direction_rough;
            Eigen::Vector2f main_direction2 = Eigen::Vector2f(-direction_rough.y(), direction_rough.x());
            if (direction_rough.norm() < 1e-3)
            {
                return false;

                // main_direction1 = eigenvectors.col(1); // 第一主方向
                // main_direction2 = eigenvectors.col(0); // 第二主方向
            }
            

            // 5. 构建变换矩阵
            Eigen::Matrix2f transformation_matrix;

            transformation_matrix.col(0) = main_direction1; // 第一主方向
            transformation_matrix.col(1) = main_direction2; // 第二主方向

            // 6. 应用变换
            std::vector<cv::Point2f> transformed_points;
            for (const auto& point : centered_points) {
                Eigen::Vector2f vec(point.x, point.y);
                Eigen::Vector2f transformed_vec = transformation_matrix.inverse() * vec;
                transformed_points.push_back(cv::Point2f(transformed_vec(0), transformed_vec(1)));
            }

            if(0) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloud_test1(new pcl::PointCloud<pcl::PointXYZI>);
                for (const auto& point : transformed_points) {
                    clustersCloud_test1->push_back(pcl::PointXYZI(point.x, point.y, 0, 0));
                }

                // debug 
                Eigen::Vector2f vec = main_direction1 * 2;
                Eigen::Vector2f transformed_vec = transformation_matrix.inverse() * vec;
                clustersCloud_test1->push_back(pcl::PointXYZI(transformed_vec(0), transformed_vec(1), 0, 3));
                transformed_vec = transformation_matrix.inverse() * -vec;
                clustersCloud_test1->push_back(pcl::PointXYZI(transformed_vec(0), transformed_vec(1), 0, 3));
                clustersCloud_test1->push_back(pcl::PointXYZI(0, 0, 0, 3));

                std::string save_name_test1(outputDir + "/cluster_" + std::to_string(index) + "_normalize.pcd");
                pcl::io::savePCDFileBinary(save_name_test1, *clustersCloud_test1);
            }

            // 确定分段范围（以X轴主方向）
            float min_x = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            for (const auto& point : transformed_points) {
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
            }

            // 分段统计
            float seg_bin_len = 0.3;
            int num_bins = std::ceil((max_x - min_x) / seg_bin_len); // 分段数量
            if (num_bins < 1)
            {
                return false;
            }
            std::vector<std::vector<cv::Point2f>> bins(num_bins);
            
            std::vector<std::vector<int>> bins_index(num_bins); // map: bin_index, origin cloud index
            for(int i = 0; i < point_size; i++){
                const auto& point = transformed_points[i];
                int bin_index = static_cast<int>((point.x - min_x) / seg_bin_len);
                bin_index = std::min(bin_index, num_bins - 1); // 防止越界
                bins[bin_index].push_back(point);

                bins_index[bin_index].push_back(i);
            }

            // 输出每个分段的统计信息
            std::vector<cv::Point2f> center_line(num_bins);
            for (int i = 0; i < num_bins; ++i) {
                if (bins[i].size() > 0)
                {   
                    cv::Point2f mid_points(0, 0);
                    for (const auto& point : bins[i]) {
                        mid_points += point;
                    }
                    mid_points *= 1.0 / bins[i].size();
                    center_line[i] = mid_points;
                }
                // std::cout << "Segment " << i << ": " << segments[i].size() << " points" << std::endl;
            }

            if (outputDir.find("laneBoundaryFromSeg") != std::string::npos) {

                // std::cout << "************   Cross Segment   ************   " << std::endl ;
                // 计算分叉位置
                std::vector<float> oriBinDist(num_bins);
                // binDist.reserve(num_bins);
                // int startIndex=-1, endIndex=-1;
                // bool isY=false, isBreak=false;
                for (int i = 0; i < num_bins; ++i) {
                    if (bins[i].size() > 1)
                    {   
                        std::sort(bins[i].begin(), bins[i].end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                            return a.y < b.y;
                        });
                        
                        float dist = 0.0;
                        for (int j = 1; j < bins[i].size(); ++j) {
                            dist = max(dist, bins[i][j].y - bins[i][j-1].y);
                        }
                        oriBinDist[i] = dist;
                    }
                }
                
                auto FindCrossPoint = [oriBinDist, num_bins, bins, index, outputDir, seg_bin_len, center_line, centroid, transformation_matrix, clusterCloud](int start, int end){
                    // 重置相关变量
                    std::vector<float> binDist;
                    binDist.reserve(num_bins);
                    int startIndex=-1, endIndex=-1;
                    bool isY=false, isBreak=false;
                    
                    // 开始处理
                    for (int i = start; i < end; ++i) {
                        
                        float dist = oriBinDist[i];
                        int cur = binDist.size() - 1;
                        if (dist > 1.0) {
                            if (cur == -1 or i == num_bins-1) {  // 1.降序 -- 起点即首点情况
                                endIndex = i;
                                binDist.push_back(dist);
                                continue;
                            }

                            // 1.降序 -- a.起点有延长出的情况; b.起点区域有多个大于2.0距离; 
                            // 3.起始区域有小分叉干扰的情况, 类似于0, 0.2, 2.0, 2.1 ....
                            if ((0 <= cur && cur <= 2) && (fabs(dist - binDist[cur]) > 1.0 || (binDist[cur] > 2.0 && dist > 2.0))) {  
                                binDist.clear();
                                cur = binDist.size() - 1;
                                startIndex = -1;
                                endIndex = i;
                            }

                            if (cur >= 5 && (fabs(binDist[cur] - dist) > 0.5 or binDist[cur] > 2.0)) {    // 2.升序 -- a.末端有距离骤降or距离大于2.0,则停止生长
                                endIndex = i-1;
                                break;
                            }
                            binDist.push_back(dist);
                        } else if (dist < 0.1) {
                            if (cur >= 5) {  
                                if (binDist[0] > 1.0) {  // 1.降序情况 -- a.判断该点是否为异常值 决定startIndex位置
                                    binDist.push_back(dist);
                                    if (bins[i].size() <= 1) {
                                        startIndex = i-1;
                                    } else {
                                        startIndex = i;
                                    }
                                } else if (binDist[cur] > 1.0) {  // 2.升序 -- 升序序列中距离骤降, 判断前一点为序列的终点位置
                                    endIndex = i-1;
                                }
                                break;
                            } 
                            // if (bins[i].size() <= 1 and cur < 5) {
                            if ( cur < 5 ) { // 1.初始阶段 -- 判断是否起始有多个可间断0.000组合, 是的话清空列表, continue继续生长
                                binDist.clear();
                                cur = binDist.size() - 1;
                                startIndex = -1;
                                endIndex=-1;

                                if (bins[i].size() <= 1) {
                                    continue;
                                }
                            }
                            if (cur == 0 and binDist[cur] < 0.1) { // 1.初始阶段 -- 排空距离小于0.1的起始区域, 以找到准确的起点位置
                                binDist.pop_back();
                            }
                            startIndex = i;
                            binDist.push_back(dist);
                        } else {  // 0.1 <= dist < 1.0
                            if (cur >= 5 and fabs(binDist[cur] - dist) > 1.0) {    // 2.升序 -- 升序序列中距离骤降, 判断前一点为序列的终点位置
                                endIndex = i-1;
                                break;
                            }
                            if (cur == -1 or (binDist[0] > 1.0 and startIndex == -1)) {  // 起始位置 or 降序序列中不断更新startIndex
                                startIndex = i;
                            }
                            if (cur == 0 and binDist[cur] < 0.01) {  // 2.升序 -- a.判断前一个点是否为异常值 决定startIndex位置
                                startIndex = i;
                            }
                            binDist.push_back(dist);
                        }
                    }

                    // std::cout << "************   startIndex : " << startIndex << ", endIndex : " << endIndex << std::endl;
                    float deltaX = seg_bin_len*std::abs(endIndex-startIndex);
                    float deltaY, CPoint_X, CPoint_Y;

                    if (binDist.size() > 5) {
                        // if (binDist.back() > binDist.front()) {   // 解决渐变分合流的
                        //     deltaY = fabs(oriBinDist[startIndex] - oriBinDist[startIndex+]);
                        // }
                        deltaY = fabs(binDist.back() - binDist.front());
                        float slope = deltaY / deltaX;
                        bool cond1 = slope > 0.2 and (binDist.back() > 1.0 or binDist.front() > 1.0);  // 斜率大于0.2且有一端要大于1.0（但另一端要小于1.0，下面条件判断）
                        cond1 = cond1 and !(binDist.back() > 1.0 and binDist.front() > 1.0);  // 处理异常大距离情况
                        bool cond2 = (binDist.back() > 2.0 and binDist.front() < 0.5) or (binDist.back() < 0.5 and binDist.front() > 2.0);  // 首尾端分别大于2.0且另一端小于0.5
                        if (cond1 or cond2) {
                            isY = true;
                            
                            float x_pin = center_line[startIndex].x;
                            float dx = (oriBinDist[startIndex] < 0.2) ? 2.0 : 4.0;
                            if (binDist.back() > binDist.front()){
                                x_pin -= dx;
                            } else {
                                x_pin += dx;
                            }
                            Eigen::Vector2f vec(x_pin, center_line[startIndex].y);
                            Eigen::Vector2f original_vec = transformation_matrix * vec + Eigen::Vector2f(centroid.x(), centroid.y());

                            CPoint_X = original_vec(0);
                            CPoint_Y = original_vec(1);
                        }
                    }

                    // **************             保存  分叉点云           *******************
                        
                    if (isY) {
                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
                        // 读取点云文件
                        std::string save_name_test1(outputDir + "/../../crosspoint.pcd");
                        std::cout << '****************** ' << save_name_test1 << std::endl;

                        std::ifstream file(save_name_test1);
                        bool fileExist = file.good();
                        file.close();
                        if (fileExist) {
                            if (pcl::io::loadPCDFile<pcl::PointXYZI>(save_name_test1, *cloud) == -1) {
                                PCL_ERROR("Couldn't read the input PCD file\n");
                                return (-1);
                            }
                        }

                        pcl::PointXYZ tmppoint(CPoint_X, CPoint_Y, 0);
                        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_local(new pcl::KdTreeFLANN<pcl::PointXYZ>);
                        kdtree_local->setInputCloud(clusterCloud); // 设置要搜索的点云，建立KDTree
                        std::vector<int> neighbors;
                        std::vector<float> dist;
                        kdtree_local->nearestKSearch(tmppoint, 1, neighbors, dist);
                        double CPoint_Z = clusterCloud->at(neighbors[0]).z;

                        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
                        kdtree->setInputCloud(cloud); // 设置要搜索的点云，建立KDTree
                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquaredDistance;
                        float radius = 3.0;

                        // 进行半径搜索
                        pcl::PointXYZI pt(CPoint_X, CPoint_Y, CPoint_Z, 3);
                        if (cloud->empty() || kdtree->radiusSearch(pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0) {
                            // 没有距离小于3米的点，添加该点
                            cloud->push_back(pt);
                        }

                        // cloud->push_back(pcl::PointXYZI(CPoint_X, CPoint_Y, 0, 3));

                        // 保存更新后的点云到文件
                        if (pcl::io::savePCDFileBinary(save_name_test1, *cloud) == -1) {
                            PCL_ERROR("Couldn't save the output PCD file\n");
                            return (-1);
                        }
                    }
                    
                    if(0) {

                        // **************             保存  log信息           *******************
                        std::cout << outputDir << " -- " << index << "  " << "************   Cross Segment :  origin dist -- " ;
                        for (size_t i = 0; i < num_bins; ++i) {
                            std::cout << std::fixed << std::setprecision(6) << oriBinDist[i] << " ";
                        }
                        std::cout << std::endl;
                        
                        if (isY) {
                            std::cout << "************   Cross Segment : True  ------- ( " << CPoint_X << ",  " << CPoint_Y << " ) --------" << std::endl;
                            std::cout << "************   startIndex : " << startIndex << ", endIndex : " << endIndex << " , dist -- " << oriBinDist[startIndex] << std::endl;
                        } else {
                            std::cout << "************   Cross Segment : False ....." << std::endl;
                            std::cout << "************   startIndex : " << startIndex << ", endIndex : " << endIndex << " , dist -- " << oriBinDist[startIndex] << std::endl;
                        }

                        std::cout << " -----   Cross area dist ----- " ;
                        for (size_t i = 0; i < binDist.size(); ++i) {
                            std::cout << std::fixed << std::setprecision(6) << binDist[i] << " ";
                        }
                        std::cout << std::endl;
                    }

                    if (isY) {
                        return 1;
                    } else {
                        return -1;
                    }
                };

                // FindCrossPoint(0, num_bins);

                if (num_bins > 90) {
                    FindCrossPoint(0, num_bins/2);
                    FindCrossPoint(num_bins/2, num_bins);
                } else {
                    FindCrossPoint(0, num_bins);
                }
            }
            
            // 中心线以上的当做一条线；中心线以下的当做一条线
            std::vector<std::vector<cv::Point2f>> split_lane_line;
            std::vector<std::vector<int>> split_lane_line_index;
            split_lane_line.resize(2);
            split_lane_line_index.resize(2);
            for (int i = 0; i < num_bins; ++i) {
                if (bins[i].size() > 0)
                {   
                    cv::Point2f mid_points = center_line[i];
                    int point_size_in_bin = bins[i].size();
                    for (int j = 0; j < point_size_in_bin; ++j) {
                        const auto& point = bins[i][j];

                        float distance = cv::norm(point - mid_points);
                        if (distance < 0.3) {
                            split_lane_line[0].push_back(point);
                            split_lane_line[1].push_back(point);
                            split_lane_line_index[0].push_back(bins_index[i][j]);
                            split_lane_line_index[1].push_back(bins_index[i][j]);
                        } else if(point.y > mid_points.y) {
                            split_lane_line[0].push_back(point);
                            split_lane_line_index[0].push_back(bins_index[i][j]);
                        } else {
                            split_lane_line[1].push_back(point);
                            split_lane_line_index[1].push_back(bins_index[i][j]);
                        }
                    }
                }
            }

            // 7. 逆变换
            // 原始点
            std::vector<cv::Point2f> original_points;
            for (const auto& point : transformed_points) {
                Eigen::Vector2f vec(point.x, point.y);
                Eigen::Vector2f original_vec = transformation_matrix * vec + Eigen::Vector2f(centroid.x(), centroid.y());
                original_points.push_back(cv::Point2f(original_vec(0), original_vec(1)));
            }

            // 中心线
            std::vector<cv::Point2f> original_center_line_points;
            for (int i = 0; i < num_bins; ++i) {
                const auto& point = center_line[i];
                if (bins[i].size() > 0)
                {   
                    Eigen::Vector2f vec(point.x, point.y);
                    Eigen::Vector2f original_vec = transformation_matrix * vec + Eigen::Vector2f(centroid.x(), centroid.y());
                    original_center_line_points.push_back(cv::Point2f(original_vec(0), original_vec(1)));
                }
            }

            // 分割的线1
            std::vector<cv::Point2f> split_line_1;
            std::vector<int> split_line_1_input_index; // 最原始点云的index
            for (int i = 0; i < split_lane_line[0].size(); ++i) {
                const auto& point = split_lane_line[0][i];
                Eigen::Vector2f vec(point.x, point.y);
                Eigen::Vector2f original_vec = transformation_matrix * vec + Eigen::Vector2f(centroid.x(), centroid.y());
                split_line_1.push_back(cv::Point2f(original_vec(0), original_vec(1)));

                int origin_index = split_lane_line_index[0][i];
                split_line_1_input_index.push_back(cluster_diverses[origin_index]);
            }

            // 分割的线2
            std::vector<cv::Point2f> split_line_2;
            std::vector<int> split_line_2_input_index;
            for (int i = 0; i < split_lane_line[1].size(); ++i) {
                const auto& point = split_lane_line[1][i];
                Eigen::Vector2f vec(point.x, point.y);
                Eigen::Vector2f original_vec = transformation_matrix * vec + Eigen::Vector2f(centroid.x(), centroid.y());
                split_line_2.push_back(cv::Point2f(original_vec(0), original_vec(1)));
                int origin_index = split_lane_line_index[1][i];
                split_line_2_input_index.push_back(cluster_diverses[origin_index]);
            }

            if (0) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloud_test1(new pcl::PointCloud<pcl::PointXYZI>);
                // for (const auto& point : original_points) {
                //     clustersCloud_test1->push_back(pcl::PointXYZI(point.x, point.y, 0, 0));
                // }
                for (const auto& point : original_center_line_points) {
                    clustersCloud_test1->push_back(pcl::PointXYZI(point.x, point.y, 0, 1));
                }
                for (const auto& point : split_line_1) {
                    clustersCloud_test1->push_back(pcl::PointXYZI(point.x, point.y, 0, 2));
                }
                for (const auto& point : split_line_2) {
                    clustersCloud_test1->push_back(pcl::PointXYZI(point.x, point.y, 0, 3));
                }
                std::string save_name_test1(outputDir + "/cluster_" + std::to_string(index) + "_recover.pcd");
                pcl::io::savePCDFileBinary(save_name_test1, *clustersCloud_test1);
            }
            
            // 第一条线
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud2(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*cloud_in, split_line_1_input_index, *clusterCloud2);

                // 去除多余杂点
                std::vector<std::vector<int>> clusters;
                float radius = 0.3f;
                int minPts = 20;
                Inference::DBSCAN(clusterCloud2, radius, minPts, clusters);
                std::vector<int> split_line_1_input_index_modify;
                for (int i = 0; i < clusters.size(); i++) {
                    if (0)
                    {
                        std::cout << "[sub lane] " << i << " th cluster size: " << clusters[i].size() << std::endl;
                    }
                    
                    if (clusters[i].size() < 10)
                        continue;

                    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::copyPointCloud(*clusterCloud2, clusters[i], *clusterCloud);

                    // 过滤比较短的线
                    auto wlh = Inference::GetWLH(clusterCloud);
                    if (wlh.x() < 1 && wlh.y() < 1 && wlh.z() < 1) {
                        continue;
                    }

                    for (int j = 0; j < clusters[i].size(); j++)
                    {
                        int point_index = clusters[i][j];
                        split_line_1_input_index_modify.push_back(split_line_1_input_index[point_index]);
                    }
                }

                split_line_1_input_index = split_line_1_input_index_modify;
                if (split_line_1_input_index.size() > 10)
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud3(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::copyPointCloud(*cloud_in, split_line_1_input_index, *clusterCloud3);

                    // 构建 clusterCloud3 与统计滤波后的点云的索引对应关系
                    std::map<int, int> clusterIndexMap2;                    
                    for (size_t j = 0; j < split_line_1_input_index.size(); ++j) {
                        int cloudBoundaryIndex = split_line_1_input_index[j];
                        auto it = indexMap.find(cloudBoundaryIndex);
                        if (it != indexMap.end()) {
                            clusterIndexMap2[j] = it->second;
                        } else {
                            // std::cout << "[sub line] can not find Index in indexMap: " << cloudBoundaryIndex << std::endl;
                        }
                    }
                    
                    if (0) {
                        std::string save_name_test1(outputDir + "/debug_cluster_1.pcd");
                        pcl::io::savePCDFileBinary(save_name_test1, *clusterCloud3);
                    }
                    
                    int preSegmentsSize = segments.size();
                    Inference::ExtractSegment(clusterCloud3, segments, 4.f, 3, 2);

                    // 更新segmentCloudIndexMap,构建segmentCloud与条件滤波后的点云的索引对应关系
                    for (size_t k = preSegmentsSize; k < segments.size(); ++k) {
                        for (size_t m = 0; m < segments[k].segmentCloudIndexMap.size(); ++m) {
                            int clusterIndex = segments[k].segmentCloudIndexMap[m];
                            auto it = clusterIndexMap2.find(clusterIndex);
                            if (it != clusterIndexMap2.end()) {
                                segments[k].segmentCloudIndexMap[m] = it->second;
                            } else {
                                // std::cout << "[lane_boundary] can not find Index in clusterIndexMap2: " << clusterIndex << std::endl;
                            }
                        }
                    }
                }
            }

            // 第二条线
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud2(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*cloud_in, split_line_2_input_index, *clusterCloud2);

                // 去除多余杂点
                std::vector<std::vector<int>> clusters;
                float radius = 0.3f;
                int minPts = 20;
                Inference::DBSCAN(clusterCloud2, radius, minPts, clusters);
                std::vector<int> split_line_2_input_index_modify;
                for (int i = 0; i < clusters.size(); i++) {
                    if (0)
                    {
                        std::cout << "[sub lane2] " << i << " th cluster size: " << clusters[i].size() << std::endl;
                    }
                    
                    if (clusters[i].size() < 10)
                        continue;

                    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud3(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::copyPointCloud(*clusterCloud2, clusters[i], *clusterCloud);

                    // 过滤比较短的线
                    auto wlh = Inference::GetWLH(clusterCloud);
                    if (wlh.x() < 1 && wlh.y() < 1 && wlh.z() < 1) {
                        continue;
                    }

                    for (int j = 0; j < clusters[i].size(); j++)
                    {
                        int point_index = clusters[i][j];
                        split_line_2_input_index_modify.push_back(split_line_2_input_index[point_index]);
                    }
                }

                split_line_2_input_index = split_line_2_input_index_modify;
                if (split_line_2_input_index.size() > 10)
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud3(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::copyPointCloud(*cloud_in, split_line_2_input_index, *clusterCloud3);

                    // 构建 clusterCloud3 与统计滤波后的点云的索引对应关系
                    std::map<int, int> clusterIndexMap2;                    
                    for (size_t j = 0; j < split_line_2_input_index.size(); ++j) {
                        int cloudBoundaryIndex = split_line_2_input_index[j];
                        auto it = indexMap.find(cloudBoundaryIndex);
                        if (it != indexMap.end()) {
                            clusterIndexMap2[j] = it->second;
                        } else {
                            // std::cout << "[sub lane2] can not find Index in indexMap: " << cloudBoundaryIndex << std::endl;
                        }
                    }

                    if (0) {
                        std::string save_name_test1(outputDir + "/debug_cluster_2.pcd");
                        pcl::io::savePCDFileBinary(save_name_test1, *clusterCloud3);
                    }

                    int preSegmentsSize = segments.size();
                    Inference::ExtractSegment(clusterCloud3, segments, 4.f, 3, 2);

                    // 更新segmentCloudIndexMap,构建segmentCloud与条件滤波后的点云的索引对应关系
                    for (size_t k = preSegmentsSize; k < segments.size(); ++k) {
                        for (size_t m = 0; m < segments[k].segmentCloudIndexMap.size(); ++m) {
                            int clusterIndex = segments[k].segmentCloudIndexMap[m];
                            auto it = clusterIndexMap2.find(clusterIndex);
                            if (it != clusterIndexMap2.end()) {
                                segments[k].segmentCloudIndexMap[m] = it->second;
                            } else {
                                // std::cout << "[sub lane2] can not find Index in clusterIndexMap2: " << clusterIndex << std::endl;
                            }
                        }
                    }
                }
            }



            // 此处将含有交叉点的线段，二次进行 skeleton 分割
            // SkeletonCluster skeleton_cluster;
            // std::vector<std::vector<int>> clusters_skeleton; // skeleton 聚类后，新的索引坐标
            // std::vector<int> origin_cluster_index;

            // 如果需要分割，则进行 skeleton 分割
            // 提取中心线
            // 恢复为两条曲线
            // std::cout << "cluster i " << index << std::endl;
            // skeleton_cluster.SetInput(clusterCloud, outputDir, std::to_string(index));
            // if (skeleton_cluster.Process())
            // {
            //     auto &cluster_results = skeleton_cluster.GetResult();
            //     for (auto &one_cluster : cluster_results)
            //     {
            //         origin_cluster_index.clear();
            //         for (auto &one_index : one_cluster)
            //         {
            //             // 恢复原始的index
            //             origin_cluster_index.push_back(cluster_diverses[one_index]);
            //         }
            //         if (origin_cluster_index.size() < 10)
            //         {
            //             continue;
            //         }
            //         clusters_skeleton.push_back(origin_cluster_index);
            //     }
            // }
            
            // // 重新开始聚类
            // for (int i2 = 0; i2 < clusters_skeleton.size(); ++i2) {
            //     pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud2(new pcl::PointCloud<pcl::PointXYZ>);
            //     pcl::copyPointCloud(*cloud_in, clusters_skeleton[i2], *clusterCloud2);
            //     // 构建 clusterCloud2 与统计滤波后的点云的索引对应关系
            //     std::map<int, int> clusterIndexMap2;                    
            //     for (size_t j = 0; j < clusters_skeleton[i2].size(); ++j) {
            //         int cloudBoundaryIndex = clusters_skeleton[i2][j];
            //         auto it = indexMap.find(cloudBoundaryIndex);
            //         if (it != indexMap.end()) {
            //             clusterIndexMap2[j] = it->second;
            //         } else {
            //             std::cout << "[lane_boundary] can not find Index in indexMap: " << cloudBoundaryIndex << std::endl;
            //         }
            //     }


            //     int preSegmentsSize = segments.size();
            //     Inference::ExtractSegment(clusterCloud2, segments, 4.f, 3, 2);

            //     // 更新segmentCloudIndexMap,构建segmentCloud与条件滤波后的点云的索引对应关系
            //     for (size_t k = preSegmentsSize; k < segments.size(); ++k) {
            //         for (size_t m = 0; m < segments[k].segmentCloudIndexMap.size(); ++m) {
            //             int clusterIndex = segments[k].segmentCloudIndexMap[m];
            //             auto it = clusterIndexMap2.find(clusterIndex);
            //             if (it != clusterIndexMap2.end()) {
            //                 segments[k].segmentCloudIndexMap[m] = it->second;
            //             } else {
            //                 std::cout << "[lane_boundary] can not find Index in clusterIndexMap2: " << clusterIndex << std::endl;
            //             }
            //         }
            //     }
            // }
            return true;
        }
    } 



}

