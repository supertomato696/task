//
//
//

#include "processCrosswalk.h"
#include "pclFilter.h"
#include "cluster.h"
#include "Utils.h"
#include "TinBuild.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/GeometryAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Algorithm/GeometricFit.h"
#include "./include/CommonUtil.h"

#include <boost/filesystem.hpp>
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法
#include <pcl/sample_consensus/model_types.h>  //模型定义
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/filters/voxel_grid.h>            //基于体素网格化的滤波
#include <pcl/filters/extract_indices.h>       //索引提取
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/registration/icp.h>
#include <pcl/console/print.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
// #include <pcl/segmentation/supervoxel_clustering.h>
// #include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/common/intersections.h>
#include <pcl/common/pca.h>

#include <pcl/surface/convex_hull.h>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp> // 包含cv2eigen函数的头文件

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include "parallel_hashmap/phmap.h"

#include <ceres/ceres.h>

using namespace std;
namespace fs = std::filesystem;

#define CLUSTERMINDCOUNT 10000
namespace RoadMapping
{
    void processCrosswalk::run_yxx_fin_in(std::string middle_json_path, std::string dataDir, std::string pcdPath, std::string refFile,
                                          std::string outputDir, std::string data_type)
    {
        // 1、读取label CLASS_TYPE_90 = 90,         //crosswalk_line
        std::string lidarFile = pcdPath;
        std::cout << "lidarFile: " << lidarFile << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(lidarFile, *pc_ptr);

        pcl::PointCloud<ConRmPointType>::Ptr cloudfilter90(new pcl::PointCloud<ConRmPointType>);
        // pclFilter::ConditionalRemoval(pc_ptr, cloudfilter90, "cloud_bev_label_1", 3);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter90, "cloud_bev_label_2", 3);


        pcl::PointCloud<pcl::PointXYZ>::Ptr crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto iter = cloudfilter90->begin(); iter != cloudfilter90->end(); iter++)
        {
            ConRmPointType &pcl_p = (*iter);
            crosswalkCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, 0.0f));
        }
        if (!crosswalkCloud->empty())
            pcl::io::savePCDFileBinary(outputDir + "/" + "crosswalk_rawCloud.pcd", *crosswalkCloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr crosswalkCloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::VoxelGridSampling(crosswalkCloud, cloudVoxelGrid, 0.05f);
        pclFilter::StatisticalOutlierRemoval(cloudVoxelGrid, crosswalkCloud2, 50, 1.0);
        if (!crosswalkCloud2->empty())
            pcl::io::savePCDFileBinary(outputDir + "/" + "crosswalk_staticRmCloud.pcd", *crosswalkCloud2);

        // 2、对点云进行聚类以及去噪处理
        float radius;
        int minPtsSerch;
        // TODO: ccj
        if (data_type == "BYD_LIDAR_B" || data_type == "BYD_LIDAR_BEV_B") {
            radius = 1.0f;
            minPtsSerch = 15;
        }
        else {
            float avgDist = computeAverageNeighborDistance(crosswalkCloud2);
            radius = avgDist * 8.0f;
            minPtsSerch = std::max(15, static_cast<int>(avgDist * 15));
        }

        std::vector<std::vector<int>> clusters;
        Inference::DBSCAN(crosswalkCloud2, radius, minPtsSerch, clusters);
        std::cout << "extract clusters size: " << clusters.size() << "  radius: " << radius << "  minPtsSerch: " << minPtsSerch << std::endl;

        // 测试输出聚类后数据
        int minPts = 100;
        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i < clusters.size(); ++i)
        {
            for (int j = 0; j < clusters[i].size(); j++)
            {
                pcl::PointXYZ curPXYZ = crosswalkCloud2->at(clusters[i][j]);
                clustersCloudWithI->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            }
        }
        if (!clustersCloudWithI->empty())
            pcl::io::savePCDFileBinary(outputDir + "/" + "crosswalk_clustersCloud.pcd", *clustersCloudWithI);

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> allClusterCoordinates;
        // #pragma omp parallel for
        for (int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr originalClusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*crosswalkCloud2, clusters[clusterIndex], *originalClusterCloud);

            // 针对每个聚类进行去噪
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredClusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pclFilter::StatisticalOutlierRemoval(originalClusterCloud, filteredClusterCloud, 50, 1.0);
            std::cout << clusterIndex << " :cluster has points :" << filteredClusterCloud->size() << std::endl;
            if (filteredClusterCloud->size() < minPts)
            {
                continue;
            }

            // 去噪以后重新聚类
            std::vector<std::vector<int>> refinedClusters;
            Inference::DBSCAN(filteredClusterCloud, radius, minPtsSerch, refinedClusters);

            pcl::PointCloud<pcl::PointXYZI>::Ptr clustersWithIndex(new pcl::PointCloud<pcl::PointXYZI>);
            for (int i = 0; i < refinedClusters.size(); ++i)
            {
                for (int pointIndex = 0; pointIndex < refinedClusters[i].size(); pointIndex++)
                {
                    pcl::PointXYZ currentPoint = filteredClusterCloud->at(refinedClusters[i][pointIndex]);
                    clustersWithIndex->push_back(pcl::PointXYZI(currentPoint.x, currentPoint.y, currentPoint.z, i));
                }
            }
            if (!clustersWithIndex->empty())
                pcl::io::savePCDFileBinary(outputDir + "/" + to_string(clusterIndex) + "_crosswalk_clustersCloud.pcd", *clustersWithIndex);

            for (int i = 0; i < refinedClusters.size(); ++i)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr newclusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*filteredClusterCloud, refinedClusters[i], *newclusterCloud);           
                if (newclusterCloud->size() < minPts)
                {
                    continue;
                }

                // 快速凸包处理
                pcl::PointCloud<pcl::PointXYZ>::Ptr convexHullCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::ConvexHull<pcl::PointXYZ> convexHull;
                convexHull.setInputCloud(newclusterCloud);
                convexHull.reconstruct(*convexHullCloud);
                pcl::io::savePCDFileBinary(outputDir + "/" + to_string(clusterIndex) + "_" + to_string(i) + "_crosswalk_cloud_hull.pcd", *convexHullCloud);

                Engine::Base::Array<Engine::Geometries::Coordinate> hullCoordinates;
                // 矩形框生成
                extractBoxOptimized(convexHullCloud, hullCoordinates, 3.0, 3.0); 
                if (hullCoordinates.IsEmpty()){
                    continue;
                }

                std::cout << "after extract array: ";
                for (size_t j = 0; j < hullCoordinates.GetCount(); ++j) {
                    const auto& coord = hullCoordinates[j];
                    std::cout << "(" << coord.x << ", " << coord.y << ") ";
                }
                std::cout << std::endl;

                std::vector<Engine::Base::Array<Engine::Geometries::Coordinate>> splitResults;
                // 根据道路边界分割人行横道
                splitRect(dataDir, refFile, hullCoordinates, splitResults, 3.0, 3.0);
                std::cout << "result size: " << splitResults.size() << std::endl;
                if (!splitResults.empty()){
                    // #pragma omp critical
                    for (const auto& array : splitResults){
                        std::cout << "after split array: ";
                        for (size_t j = 0; j < array.GetCount(); ++j) {
                            const auto& coord = array[j];
                            std::cout << "(" << coord.x << ", " << coord.y << ") ";
                        }
                        std::cout << std::endl;

                        allClusterCoordinates.Add(array);
                    }
                }
            }
        }

        // afterProcess(middle_json_path, dataDir, outputDir, refFile, allClusterCoordinates);

        for (size_t i = 0; i < allClusterCoordinates.GetCount(); ++i) {
            const auto& array = allClusterCoordinates[i];
            std::cout << "after process array: ";
            for (size_t j = 0; j < array.GetCount(); ++j) {
                const auto& coord = array[j];
                std::cout << "(" << coord.x << ", " << coord.y << ") ";
            }
            std::cout << std::endl;
        }

        // 输出测试数据
        if (!allClusterCoordinates.IsEmpty())
            hdmap_build::CommonUtil::WriteToOBJ(allClusterCoordinates, outputDir, "crosswalk", true);
    }

    void processCrosswalk::get_crosswalk_by_stopline(std::string global_pcd_path, const Engine::Base::Array<Engine::Geometries::Coordinate> &lukou_center_pts, const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &stop_lines, std::string outputDir, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &edgePts, bool online)
    {
        std::cout << "------>人行横道建模：process get_crosswalk_by_stopline global_pcd_path: " << global_pcd_path << std::endl;
        _output = outputDir;
        pcl::PointCloud<pcl::PointXYZ>::Ptr bev_crosswalk_pcd(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<std::vector<int>> clustersnew;

        if (online)
        {
            if (lukou_center_pts.IsEmpty())
            {
                std::cout << "路口点为空！！！" << std::endl;
                return;
            }

            pcl::PointCloud<pcl::PointXYZ> crosswalkCloud;
            if (!filter_label_pcd(global_pcd_path, crosswalkCloud, outputDir, "label", 90))
            {
                return;
            }

            // 将点云转换为bev图片，并进行膨胀腐蚀 得到bev的点云
            std::cout << "label点云转换为bev图片，进行去噪和gap填充" << std::endl;

            output_bev(crosswalkCloud, 0.05, outputDir, bev_crosswalk_pcd);

            if (bev_crosswalk_pcd->points.empty())
                return;

            // 将点云进行聚类操作
            float radius = 1.0f;
            int minPtsSerch = 60;
            std::cout << "进行聚类 radius：" << radius << " minPtsSerch:" << minPtsSerch << std::endl;

            Inference::DBSCAN(bev_crosswalk_pcd, radius, minPtsSerch, clustersnew);

            // 测试输出聚类后数据
            pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
            for (int i = 0; i < clustersnew.size(); ++i)
            {
                if (clustersnew[i].size() < CLUSTERMINDCOUNT)
                    continue;
                for (int j = 0; j < clustersnew[i].size(); j++)
                {
                    pcl::PointXYZ curPXYZ = bev_crosswalk_pcd->at(clustersnew[i][j]);
                    clustersCloudWithI->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
                }
            }
            if (!clustersCloudWithI->empty())
                pcl::io::savePCDFileBinary(outputDir + "/" + "crosswalk_clustersCloud.pcd", *clustersCloudWithI);
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::io::loadPCDFile(outputDir + "/" + "crosswalk_clustersCloud.pcd", *clustersCloudWithI);
            std::map<int, std::vector<int>> mapCluster;
            for (int i = 0; i < clustersCloudWithI->points.size(); ++i)
            {
                pcl::PointXYZ newPclPt(clustersCloudWithI->points[i].x, clustersCloudWithI->points[i].y, clustersCloudWithI->points[i].z);
                bev_crosswalk_pcd->push_back(newPclPt);
                int intensity = int(clustersCloudWithI->points[i].intensity);
                mapCluster[intensity].push_back(i);
            }
            for (auto onecluster : mapCluster)
            {
                clustersnew.push_back(onecluster.second);
            }
        }

        // 输入路口点和停止线，并计算停止线与路口的关系
        inputLuKouCenterPts(lukou_center_pts);
        inputStoplines(stop_lines);

        // 将聚类点云与路口和停止线进行关联
        std::cout << "聚类点云与路口和停止线进行关联" << std::endl;
        cluster_pts_realtion(clustersnew, bev_crosswalk_pcd);

        Engine::Base::Array<ClustrCloud_Realtion_Stopline> resClusterBoxs;
        for (int i = 0; i < _crosswalk_info.GetCount(); ++i)
        {
            int index = _crosswalk_info[i].clusterIndex;
            std::cout << index << "：聚类提取外包框" << std::endl;
            std::cout << index << ":聚类中点是否正确:" << _crosswalk_info[i].isright << " 关联的路口点: " << _crosswalk_info[i].r_center_index << "  关联的停止线:" << _crosswalk_info[i].r_stoplineIndex << std::endl;

            if (_crosswalk_info[i].isright == false)
            {
                Engine::Base::Array<ClustrCloud_Realtion_Stopline> newClusterBoxs;
                deal_wrong(_crosswalk_info[i], newClusterBoxs);
                for (int j = 0; j < newClusterBoxs.GetCount(); ++j)
                {
                    if (!newClusterBoxs[j].boxpts.IsEmpty())
                    {
                        resClusterBoxs.Add(newClusterBoxs[j]);
                    }
                }
            }
            else if (_crosswalk_info[i].r_stoplineIndex >= 0 && _crosswalk_info[i].r_center_index >= 0) // 有中心点和停止线同时关联
            {
                deal_r_center_stopline(_crosswalk_info[i]);
                if (!_crosswalk_info[i].boxpts.IsEmpty())
                {
                    resClusterBoxs.Add(_crosswalk_info[i]);
                }
            }
            else if (_crosswalk_info[i].r_stoplineIndex < 0 && _crosswalk_info[i].r_center_index >= 0) // 只关联路口中心点
            {
                deal_r_center(_crosswalk_info[i]);
                if (!_crosswalk_info[i].boxpts.IsEmpty())
                {
                    resClusterBoxs.Add(_crosswalk_info[i]);
                }
            }
            else if (_crosswalk_info[i].r_stoplineIndex >= 0 && _crosswalk_info[i].r_center_index < 0) // 只关联停止线
            {
                deal_r_stopline(_crosswalk_info[i]);
                if (!_crosswalk_info[i].boxpts.IsEmpty())
                {
                    resClusterBoxs.Add(_crosswalk_info[i]);
                }
            }
            else if (_crosswalk_info[i].r_stoplineIndex < 0 && _crosswalk_info[i].r_center_index < 0) // 都不关联
            {
                deal_no_r(_crosswalk_info[i]);
                if (!_crosswalk_info[i].boxpts.IsEmpty())
                {
                    resClusterBoxs.Add(_crosswalk_info[i]);
                }
            }
        }

        // 对已经生成的外包框进行调整
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> oriedgePts;
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> icpedgePts;
        for (int i = 0; i < resClusterBoxs.GetCount(); i++)
        {
            resClusterBoxs[i].clusterIndex = i;
            oriedgePts.Add(resClusterBoxs[i].boxpts);
            fix_edge_icp(resClusterBoxs[i]);
            icpedgePts.Add(resClusterBoxs[i].icpboxpts);

            // 根据点云判断覆盖情况，覆盖过少的框删除，保准确率
            if (adjust_box(resClusterBoxs[i].clusterpts, resClusterBoxs[i].icpboxpts))
            {
                edgePts.Add(resClusterBoxs[i].icpboxpts);
            }
            else
            {
                std::cout << resClusterBoxs[i].clusterIndex << "聚类覆盖点云不全，将删除" << std::endl;
            }
        }
        hdmap_build::CommonUtil::WriteToOBJ(oriedgePts, outputDir, "ori_crosswalk", true);
        hdmap_build::CommonUtil::WriteToOBJ(icpedgePts, outputDir, "ori_icp_crosswalk", true);
        hdmap_build::CommonUtil::WriteToOBJ(edgePts, outputDir, "ori_adjust_box_crosswalk", true);
        //        //每个关联数据生成外包框
        //        for (int i = 0; i < arrCrosswalkRealtion.GetCount(); ++i) {
        //            //判断是否有停止线进行关联
        //            if (arrCrosswalkRealtion[i].stopline_realtion.IsEmpty())
        //                continue;
        //            std::cout << arrCrosswalkRealtion[i].clusterIndex <<"-->聚类点生成外包框"<< std::endl;
        //            std::cout<<arrCrosswalkRealtion[i].clusterIndex<<":关联的停止线:"<<arrCrosswalkRealtion[i].r_stoplineIndex<<std::endl;
        //
        //            //根据停止线计算朝向
        //            Engine::Geometries::Coordinate direction;
        //            get_stop_line_driver_dir(arrCrosswalkRealtion[i].stopline_realtion, lukou_center_pts, direction);
        //
        //            //test
        //            Engine::Base::Array<Engine::Geometries::Coordinate> stop_dir;
        //            stop_dir.Add(arrCrosswalkRealtion[i].middle_pt);
        //            stop_dir.Add(arrCrosswalkRealtion[i].middle_pt + direction * 3.0);
        //            Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> testPts;
        //            testPts.Add(stop_dir);
        //            hdmap_build::CommonUtil::WriteToOBJ(testPts, outputDir, to_string(arrCrosswalkRealtion[i].clusterIndex)+"_move_dir", true);
        //
        //            Engine::Base::Array<Engine::Geometries::Coordinate> one_box;
        //            check_edge(arrCrosswalkRealtion[i], direction, one_box);
        //            if (!one_box.IsEmpty())
        //            {
        //                edgePts.Add(one_box);
        //            }
        //        }
        std::cout << "------>人行横道建模完成<------" << std::endl;
    }

    void processCrosswalk::run_bev_label_crosswalk(std::string global_pcd_path, std::string bev_obj_dir, std::string outputDir, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &edgePts)
    {
        _output = outputDir;
        _VoxelGrid_size = 0.5f;
        std::cout << "开始处理点云数据" << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(global_pcd_path, *pc_ptr);
        if (pc_ptr->points.empty())
        {
            std::cout << "点云文件无数据：global_pcd_path:" << global_pcd_path << std::endl;
            return;
        }

        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter(new pcl::PointCloud<MyColorPointType>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ori_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_pano_seg", 16);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_bev_label_1", 3);
        for (auto iter = cloudfilter->begin(); iter != cloudfilter->end(); iter++)
        {
            MyColorPointType &pcl_p = (*iter);
            ori_crosswalkCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
            _crosswalkcloud.Add(Engine::Geometries::Coordinate(pcl_p.x, pcl_p.y, pcl_p.z));
        }
        if (ori_crosswalkCloud->empty() || ori_crosswalkCloud->points.empty())
        {
            std::cout << "无人行横道label点云：global_pcd_path:" << global_pcd_path << std::endl;
            return;
        }
        pcl::io::savePCDFileBinary(outputDir + "/ori_cloud_bev_label_crosswalk.pcd", *ori_crosswalkCloud);

        // 对点云进行体素滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::VoxelGridSampling(ori_crosswalkCloud, cloudVoxelGrid, _VoxelGrid_size);
        pcl::io::savePCDFileBinary(outputDir + "/ori_cloud_bev_label_crosswalk_Voxel.pcd", *cloudVoxelGrid);
        // //提取体素滤波后点云的边缘
        // std::cout << "提取人行横道边缘" << std::endl;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudhull(new pcl::PointCloud<pcl::PointXYZ>);
        // get_cloud_hull(cloudVoxelGrid, cloudhull);
        // if (cloudhull->empty() || cloudhull->points.empty())
        // {
        //     std::cout<<"人行横道边缘提取失败"<<std::endl;
        //     return;
        // }
        // pcl::io::savePCDFileBinary(outputDir+"/hull_cloud_bev_label_crosswalk.pcd",*cloudhull);

        ///////////////////bev数据////////////////////////
        std::cout << "开始处理bev数据" << std::endl;
        // bev_dir = "/home/lenovo/data/lidar_mapping_ws/55302/bev_obj/";
        if (!boost::filesystem::exists(bev_obj_dir))
        {
            return;
        }

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> bev_crosswalks;
        bev_crosswalk_obj(bev_obj_dir, outputDir, bev_crosswalks);
        if (bev_crosswalks.IsEmpty())
        {
            std::cout << "bev数据为空" << std::endl;
            return;
        }

        // 为每个多边形打分
        //  cal_bev_crosswalk_core(bev_crosswalks, cloudhull, 0.90);
        cal_bev_crosswalk_core(bev_crosswalks, cloudVoxelGrid, 0.90);

        // filter多余多边形
        filter_bev_polygons();

        // 提取全景分割点云：
        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter_pano_seg(new pcl::PointCloud<MyColorPointType>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pano_seg_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter_pano_seg, "cloud_pano_seg", 16);
        if (cloudfilter_pano_seg->empty())
        {
            cloudfilter_pano_seg = cloudfilter;
        }

        for (auto iter = cloudfilter_pano_seg->begin(); iter != cloudfilter_pano_seg->end(); iter++)
        {
            MyColorPointType &pcl_p = (*iter);
            pano_seg_crosswalkCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }
        // 建立搜索树
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr pano_seg_kdtree_cloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        pano_seg_kdtree_cloud->setInputCloud(pano_seg_crosswalkCloud); // 设置要搜索的点云，建立KDTree

        // 将polygon截取出的点云进行包装，提取box
        for (int i = 0; i < _bev_crosswalks_polygons.GetCount(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr in_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
            get_cloud_in_bevcrosswalk(_bev_crosswalks_polygons[i], pano_seg_crosswalkCloud, pano_seg_kdtree_cloud, in_crosswalkCloud);
            ClustrCloud_Realtion_Stopline one_clustercloud;
            if (Bev_Crosswalk_to_ClustrCloud_Realtion_Stopline(_bev_crosswalks_polygons[i], in_crosswalkCloud, one_clustercloud))
            {
                std::cout << "提取外边缘点：" << _bev_crosswalks_polygons[i].index << std::endl;
                // deal_no_r(one_clustercloud);

                // 使用check_edge2需要提前确定好人行横道进入点，以及朝向路口的移动方向
                Engine::Geometries::Coordinate trans_direction(one_clustercloud.fit_axis_normal.y, -one_clustercloud.fit_axis_normal.x, 0.0);
                one_clustercloud.move_normal = trans_direction;
                double length = 10.0;
                one_clustercloud.start_pt = one_clustercloud.middle_pt + trans_direction * length * (-1.0);
                check_edge2(one_clustercloud);

                std::cout << "外边缘点icp：" << _bev_crosswalks_polygons[i].index << std::endl;
                fix_edge_icp(one_clustercloud);
                if (one_clustercloud.icpboxpts.GetCount() == 5)
                {
                    edgePts.Add(one_clustercloud.icpboxpts);
                }
            }
        }
        hdmap_build::CommonUtil::WriteToOBJ(edgePts, _output, "final_bev_crosswalk", true);
    }

    void processCrosswalk::crosswalk_cloud_to_png(pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_crosswalk_pcd, cv::Mat &crosswalk_image, int &width, int &height)
    {
        pcl::PointXYZ min_p1, max_p1;
        pcl::getMinMax3D(*bev_crosswalk_pcd, min_p1, max_p1);

        //         _min_x = (floor((min_p1.x) / 10)) * 10;
        //         _min_y = (floor((min_p1.y) / 10)) * 10;
        //         int max_x = (ceil((max_p1.x) / 10)) * 10;
        //         int max_y = (ceil((max_p1.y) / 10)) * 10;

        //         width = (max_x - _min_x) / _pixel_size_m;
        //         height = (max_y - _min_y) / _pixel_size_m;

        //         // std::sort(point_cloud.begin(), point_cloud.end(),
        //         //           [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
        //         //           { return pt1.z < pt2.z; });

        // //        cv::Mat ground_lable_image(height, width, CV_8UC1, 0);
        //         crosswalk_image = cv::Mat::zeros(height, width, CV_8UC1);
        //         for (pcl::PointXYZ p : bev_crosswalk_pcd->points)
        //         {
        //             int x = round((p.x - _min_x + 0.0001) / _pixel_size_m);
        //             int y = round(height - (p.y - _min_y + 0.0001) / _pixel_size_m);
        //             if (x < 0 || x >= width || y < 0 || y >= height)
        //             {
        //                 continue;
        //             }

        //             crosswalk_image.at<uint8_t>(y, x) = 255;
        //         }
        //         cv::imwrite((_output + "/crosswalk.png"), crosswalk_image);
    }

    void processCrosswalk::image_to_cloud(const cv::Mat &crosswalk_image, pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_crosswalk_pcd)
    {
        // for (int i = 0 ; i < _width ; i ++)
        // {
        //     for(int j = 0 ; j < _height ; j ++)
        //     {
        //         if(crosswalk_image.at<uchar>(j,i) != 255)
        //             continue;
        //         double x = i * _pixel_size_m + _min_x;
        //         double y = (_height - j )* _pixel_size_m + _min_y;
        //         pcl::PointXYZ pt(float(x), float(y), 0);
        //         int K = 1;
        //         std::vector<int> Idx;
        //         std::vector<float> SquaredDistance;
        //         // kdtree_cloud.nearestKSearch(pt, K, Idx, SquaredDistance);
        //         // pt.z = ptr_cloud->at(Idx[0]).z;
        //         // bev_cloud->push_back(pt);
        //     }
        // }
    }

    void processCrosswalk::pts_geo_polygon(const Engine::Base::Array<Engine::Geometries::Coordinate> &plyCoords, Engine::Geometries::Polygon &resPolygon)
    {
        Engine::Base::Array<Engine::Geometries::Coordinate *> *newCoords = new Engine::Base::Array<Engine::Geometries::Coordinate *>();
        for (int i = 0; i < plyCoords.GetCount(); ++i)
        {
            Engine::Geometries::Coordinate *newPoint = new Engine::Geometries::Coordinate();
            newPoint->x = plyCoords[i].x;
            newPoint->y = plyCoords[i].y;
            newPoint->z = plyCoords[i].z;
            newCoords->Add(newPoint);
        }

        Engine::Geometries::LinearRing *shellLinearRing = new Engine::Geometries::LinearRing(newCoords);
        // if(!shellLinearRing->IsClosed())
        // {
        //     shellLinearRing->GetCoordinates()->Add(shellLinearRing->GetStartPoint());
        // }
        Engine::Geometries::Polygon polygon(shellLinearRing);
        resPolygon = polygon;
    }

    void processCrosswalk::get_cloud_in_bevcrosswalk(Bev_Crosswalk &bev_crosswalk, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pano_segCloud, const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &pts_in_cloud)
    {
        // 在中心点30米范围内检索全景分割的点
        std::vector<int> k_indices;
        std::vector<float> k_distances;
        pcl::PointXYZ newPt(bev_crosswalk.middlept.x, bev_crosswalk.middlept.y, bev_crosswalk.middlept.z);
        kdtree_cloud1->radiusSearch(newPt, 30.0, k_indices, k_distances);

        Engine::Geometries::Polygon bev_obj_polygon;
        pts_geo_polygon(bev_crosswalk.polygon_pts, bev_obj_polygon);

#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
        for (int i = 0; i < k_indices.size(); i++)
        {
            auto &pt = pano_segCloud->points[k_indices[i]];
            Engine::Geometries::Coordinate curPt(pt.x, pt.y, pt.z);
            if (bev_crosswalk.poly_enve.Intersects(curPt)) // 粗匹配
            {
                if (Engine::Geometries::GeometryAlgorithm::PtInPolygon(&bev_obj_polygon, &curPt))
                {
                    pts_in_cloud->points.push_back(pt);
                }
            }
        }
    }

    void processCrosswalk::get_Envelope(const Engine::Base::Array<Engine::Geometries::Coordinate> &arrPoints, Engine::Geometries::Envelope &envelope)
    {
        if (arrPoints.IsEmpty())
        {
            return;
        }
        Double dMinX, dMaxX, dMinY, dMaxY;
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

    void processCrosswalk::cal_fitaxis_height_width(Bev_Crosswalk &cal_length_width)
    {
        if (!Engine::Algorithm::GeometricFit::LeastSquareLineFit(cal_length_width.polygon_pts, cal_length_width.fit_s, cal_length_width.fit_e))
        {
            return;
        }

        cal_length_width.length = cal_length_width.fit_s.DistanceXY(cal_length_width.fit_e);
        cal_length_width.middlept = hdmap_build::CommonUtil::GetAveragePt(cal_length_width.polygon_pts);
        Engine::Geometries::Coordinate e1 = cal_length_width.fit_e - cal_length_width.fit_s;
        e1.z = 0.0;
        e1.Normalize();

        Engine::Geometries::Coordinate trans_direction(e1.y, -e1.x, 0.0);
        double maxdis = -99999999;
        double mindis = 99999999;
        for (int i = 0; i < cal_length_width.polygon_pts.GetCount(); i++)
        {
            /* code */
        }
    }

    void processCrosswalk::cal_bev_crosswalk_core(const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &bev_crosswalks_polygons, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull, double toler)
    {
        std::mutex mtx;
        std::cout << "------>对bev每个数据打分" << std::endl;
        if (_crosswalkcloud.IsEmpty())
        {
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_xy(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);
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
        // 生成边缘点检测kdtree
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        kdtree_cloud->setInputCloud(cloud_projected_xy); // 设置要搜索的点云，建立KDTree
        double piexl_area = _VoxelGrid_size * _VoxelGrid_size;

#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
        for (int i = 0; i < bev_crosswalks_polygons.GetCount(); i++)
        {
            // std::cout<<i<<std::endl;
            if (i % 10000 == 0)
            {
                double ndeal = double(i) / double(bev_crosswalks_polygons.GetCount());

                std::cout << "处理进度：" << int(ndeal * 100) << "%, " << i << " / " << bev_crosswalks_polygons.GetCount() << std::endl;
            }

            if (bev_crosswalks_polygons[i].GetCount() < 4)
            {
                continue;
            }

            // 面积最小为10 最大为250
            double area = hdmap_build::CommonUtil::ComputePolygonArea(bev_crosswalks_polygons[i]);
            if (area < 10.0 || area > 250)
            {
                continue;
            }

            Engine::Geometries::Coordinate middle_pt = hdmap_build::CommonUtil::GetAveragePt(bev_crosswalks_polygons[i]);

            // 在中心点30米范围内检索点,最小个数需满足 toler * area / 体素面积
            std::vector<int> k_indices;
            std::vector<float> k_distances;
            pcl::PointXYZ newPt(middle_pt.x, middle_pt.y, 0.0);
            kdtree_cloud->radiusSearch(newPt, 30.0, k_indices, k_distances);

            double all_num = area / piexl_area;
            double minPts = toler * all_num;

            if (k_indices.size() < minPts)
            {
                continue;
            }

            int matchnum = 0;
            Engine::Geometries::Envelope newEnvelope;
            get_Envelope(bev_crosswalks_polygons[i], newEnvelope);
            Engine::Geometries::Polygon bev_obj_polygon;
            pts_geo_polygon(bev_crosswalks_polygons[i], bev_obj_polygon);
            for (int j = 0; j < k_indices.size(); j++)
            {
                auto &pt = cloud_projected_xy->points[k_indices[j]];
                Engine::Geometries::Coordinate curPt(pt.x, pt.y, pt.z);
                if (newEnvelope.Intersects(curPt)) // 粗匹配
                {
                    if (Engine::Geometries::GeometryAlgorithm::PtInPolygon(&bev_obj_polygon, &curPt))
                    {
                        matchnum++;
                    }
                }
            }

            if (matchnum > minPts)
            {
                double score = matchnum / all_num;
                // std::cout<<score<< " ";
                Bev_Crosswalk new_bev_crosswalk;
                new_bev_crosswalk.index = i + 1;
                new_bev_crosswalk.isSave = true;
                new_bev_crosswalk.poly_enve = newEnvelope;
                new_bev_crosswalk.area = area;
                new_bev_crosswalk.score = score;
                new_bev_crosswalk.num_crosswalk_pts_in = matchnum;
                new_bev_crosswalk.polygon_pts = bev_crosswalks_polygons[i];
                new_bev_crosswalk.middlept = middle_pt;
                // std::lock_guard<std::mutex> lck(mtx);
                _bev_crosswalks_polygons.Add(new_bev_crosswalk);
            }
        }

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> bev_save_polygons;
        for (int i = 0; i < _bev_crosswalks_polygons.GetCount(); i++)
        {
            _bev_crosswalks_polygons[i].index = i + 1;
            // std::cout<< _bev_crosswalks_polygons[i].index << ":score-" << _bev_crosswalks_polygons[i].score << ",  ";
            bev_save_polygons.Add(_bev_crosswalks_polygons[i].polygon_pts);
        }

        hdmap_build::CommonUtil::WriteToOBJ(bev_save_polygons, _output, "score_filter_bev_crosswalk" + std::to_string(int(toler * 100)), true);
    }

    void processCrosswalk::filter_bev_polygons()
    {
        std::cout << "------>根据分数去重" << std::endl;
        for (int i = 0; i < _bev_crosswalks_polygons.GetCount(); i++)
        {
            if (!_bev_crosswalks_polygons[i].isSave)
            {
                continue;
            }
            for (int j = 0; j < _bev_crosswalks_polygons.GetCount(); j++)
            {
                if (!_bev_crosswalks_polygons[j].isSave)
                {
                    continue;
                }
                if (_bev_crosswalks_polygons[i].index == _bev_crosswalks_polygons[j].index)
                {
                    continue;
                }
                Engine::Geometries::Envelope Intersec;
                bool isOverlap = _bev_crosswalks_polygons[i].poly_enve.Intersection(_bev_crosswalks_polygons[j].poly_enve, Intersec);
                if (!isOverlap)
                {
                    continue;
                }

                // if(_bev_crosswalks_polygons[i].index == 101 && _bev_crosswalks_polygons[j].index == 108)
                // {
                //     std::cout << "101 : poly_enve GetMaxX GetMaxY GetMinX GetMinY"  << _bev_crosswalks_polygons[i].poly_enve.GetMaxX() << _bev_crosswalks_polygons[i].poly_enve.GetMaxY() << _bev_crosswalks_polygons[i].poly_enve.GetMinX()<< _bev_crosswalks_polygons[i].poly_enve.GetMinY()<< std::endl;
                //     std::cout << "108 : poly_enve GetMaxX GetMaxY GetMinX GetMinY"  << _bev_crosswalks_polygons[j].poly_enve.GetMaxX() << _bev_crosswalks_polygons[j].poly_enve.GetMaxY() << _bev_crosswalks_polygons[j].poly_enve.GetMinX()<< _bev_crosswalks_polygons[j].poly_enve.GetMinY()<< std::endl;
                //     std::cout << "Intersec : poly_enve GetMaxX GetMaxY GetMinX GetMinY"  << Intersec.GetMaxX() << Intersec.GetMaxY() << Intersec.GetMinX()<< Intersec.GetMinY()<< std::endl;
                // }

                double overlap1 = Intersec.GetArea() / _bev_crosswalks_polygons[i].area;
                double overlap2 = Intersec.GetArea() / _bev_crosswalks_polygons[j].area;
                if (overlap1 > 0.1 || overlap2 > 0.1)
                {
                    int n1 = _bev_crosswalks_polygons[i].num_crosswalk_pts_in;
                    int n2 = _bev_crosswalks_polygons[j].num_crosswalk_pts_in;

                    // 判断包含点数在相差不大的时候 以score为主 否则以点数为主
                    int diff = fabs(n1 - n2);
                    double diffRadio1 = double(diff) / double(n1);
                    double diffRadio2 = double(diff) / double(n2);
                    if (fabs(diffRadio1 - diffRadio2) < 0.02)
                    {
                        // 看哪个包含了更多的点
                        if (_bev_crosswalks_polygons[j].score > _bev_crosswalks_polygons[i].score)
                        {
                            // std::cout<< _bev_crosswalks_polygons[i].index << "败于" << _bev_crosswalks_polygons[j].index<<",  ";
                            _bev_crosswalks_polygons[i].isSave = false;
                            break;
                        }
                        else
                        {
                            // std::cout<< _bev_crosswalks_polygons[i].index << "胜于" << _bev_crosswalks_polygons[j].index<<",  ";
                            _bev_crosswalks_polygons[j].isSave = false;
                        }
                    }
                    else
                    {
                        // 看哪个包含了更多的点
                        if (n2 > n1)
                        {
                            // std::cout<< _bev_crosswalks_polygons[i].index << "败于" << _bev_crosswalks_polygons[j].index<<",  ";
                            _bev_crosswalks_polygons[i].isSave = false;
                            break;
                        }
                        else
                        {
                            // std::cout<< _bev_crosswalks_polygons[i].index << "胜于" << _bev_crosswalks_polygons[j].index<<",  ";
                            _bev_crosswalks_polygons[j].isSave = false;
                        }
                    }

                    /////////////////////////////////////////只是分数方案//////////////////////////////////////////////
                    // if(_bev_crosswalks_polygons[j].score > _bev_crosswalks_polygons[i].score)
                    // {
                    //     // std::cout<< _bev_crosswalks_polygons[i].index << "败于" << _bev_crosswalks_polygons[j].index<<",  ";
                    //     _bev_crosswalks_polygons[i].isSave = false;
                    //     break;
                    // }
                    // else
                    // {
                    //     // std::cout<< _bev_crosswalks_polygons[i].index << "胜于" << _bev_crosswalks_polygons[j].index<<",  ";
                    //     _bev_crosswalks_polygons[j].isSave = false;
                    // }
                }
            }
        }
        int n = _bev_crosswalks_polygons.GetCount();
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> bev_save_polygons;
        for (int i = n - 1; i >= 0; i--)
        {
            if (!_bev_crosswalks_polygons[i].isSave)
            {
                _bev_crosswalks_polygons.Delete(i);
            }
            else
            {
                bev_save_polygons.Add(_bev_crosswalks_polygons[i].polygon_pts);
                _bev_crosswalks_polygons[i].index = bev_save_polygons.GetCount();
            }
        }
        hdmap_build::CommonUtil::WriteToOBJ(bev_save_polygons, _output, "filter_bev_crosswalk", true);
    }

    bool processCrosswalk::Bev_Crosswalk_to_ClustrCloud_Realtion_Stopline(Bev_Crosswalk &bev_crosswalk, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pts_in_cloud, ClustrCloud_Realtion_Stopline &one_clustercloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
        if ((pts_in_cloud != NULL) && (pts_in_cloud->points.size() > 0))
        {
            crosswalkCloud->points = pts_in_cloud->points;
        }
        else
        {
            return false;
        }

        // //对数据进行统计滤波去噪：
        // pcl::PointCloud<pcl::PointXYZ>::Ptr filter_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pclFilter::StatisticalOutlierRemoval(crosswalkCloud, filter_crosswalkCloud, 50, 0.2);

        // 对框内点云膨胀腐蚀
        pcl::PointCloud<pcl::PointXYZ>::Ptr deal_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
        output_bev(*crosswalkCloud, 0.05, _output, deal_crosswalkCloud);
        if (deal_crosswalkCloud->empty() || deal_crosswalkCloud->points.size() < 1)
        {
            return false;
        }
        Engine::Base::Array<Engine::Geometries::Coordinate> clusterpts;
        for (int i = 0; i < deal_crosswalkCloud->points.size(); i++)
        {
            clusterpts.Add(Engine::Geometries::Coordinate(deal_crosswalkCloud->points[i].x, deal_crosswalkCloud->points[i].y, deal_crosswalkCloud->points[i].z));
        }

        // 用膨胀腐蚀后的点云提取凹壳
        hdmap_build::CommonUtil::WriteToTxt(clusterpts, _output, to_string(bev_crosswalk.index) + "_bev_pcd.txt", true);
        getConcaveHull(clusterpts, one_clustercloud.concavehull_ptr);
        one_clustercloud.clusterIndex = bev_crosswalk.index;
        one_clustercloud.middle_pt = hdmap_build::CommonUtil::GetAveragePt(bev_crosswalk.polygon_pts);
        one_clustercloud.clusterpts = clusterpts;

        // 计算拟合线
        Engine::Geometries::Coordinate ptStart, ptEnd;
        Engine::Algorithm::GeometricFit::LeastSquareLineFit(clusterpts, ptStart, ptEnd);
        one_clustercloud.fit_axis.start_pt = ptStart;
        one_clustercloud.fit_axis.end_pt = ptEnd;
        one_clustercloud.fit_axis_normal = ptEnd - ptStart;
        one_clustercloud.fit_axis_normal.z = 0.0;
        one_clustercloud.fit_axis_normal.Normalize();

        // Engine::Base::Array<Engine::Geometries::Coordinate> testpts;
        // testpts.Add(ptStart);
        // testpts.Add(ptEnd);
        // testpts.Add(one_clustercloud.middle_pt);
        // hdmap_build::CommonUtil::WriteToTxt(testpts, _output, to_string(bev_crosswalk.index)+"_test_bev_pcd.txt", true);
        return true;
    }

    void processCrosswalk::pcl_crosswalk(pcl::PointCloud<pcl::PointXYZ>::Ptr crosswalkCloud)
    {
        // 根据pcl进行点云膨胀腐蚀工作
        pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
        pmf.setInputCloud(crosswalkCloud);
        // 设置窗的大小以及切深，斜率信息
        pmf.setMaxWindowSize(10);
        pmf.setSlope(1.0);
        pmf.setInitialDistance(0.5f);
        pmf.setMaxDistance(1.5f);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pmf.extract(inliers->indices);

        // 从标号到点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(crosswalkCloud);
        extract.setIndices(inliers);
        extract.filter(*cloud_filtered);
    }

    void processCrosswalk::ExtractPointsPackage(const pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud, Engine::Base::Array<Engine::Geometries::Coordinate> &plyCroods)
    {
        Engine::Base::Array<Engine::Geometries::Coordinate> clusterCroods;
        for (auto iter = clusterCloud->begin(); iter != clusterCloud->end(); iter++)
        {
            Engine::Geometries::Coordinate newCrood(iter->x, iter->y, iter->z);
            clusterCroods.Add(newCrood);
        }

        // 提取单个聚类的外包络点集
        Engine::Geometries::BaseAlgorithm::PointsPackage(clusterCroods, plyCroods);
    }

    void processCrosswalk::ExtractBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud, Engine::Base::Array<Engine::Geometries::Coordinate> &plyCroods, double widthToler, double heightToler)
    {
        if (clusterCloud->size() < 4)
            return;

        Engine::Base::Array<Engine::Geometries::Coordinate> clusterCroods;
        for (auto iter = clusterCloud->begin(); iter != clusterCloud->end(); iter++)
        {
            Engine::Geometries::Coordinate newCrood(iter->x, iter->y, iter->z);
            clusterCroods.Add(newCrood);
        }

        Envelope rcBounds;
        Double dMinX, dMaxX, dMinY, dMaxY;
        dMinX = clusterCroods[0].x;
        dMaxX = clusterCroods[0].x;
        dMinY = clusterCroods[0].y;
        dMaxY = clusterCroods[0].y;

        for (int i = 1; i < clusterCroods.GetCount(); i++)
        {
            if (clusterCroods[i].x < dMinX)
            {
                dMinX = clusterCroods[i].x;
            }
            else if (clusterCroods[i].x > dMaxX)
            {
                dMaxX = clusterCroods[i].x;
            }

            if (clusterCroods[i].y < dMinY)
            {
                dMinY = clusterCroods[i].y;
            }
            else if (clusterCroods[i].y > dMaxY)
            {
                dMaxY = clusterCroods[i].y;
            }
        }

        rcBounds.SetMinX(dMinX);
        rcBounds.SetMaxX(dMaxX);
        rcBounds.SetMinY(dMinY);
        rcBounds.SetMaxY(dMaxY);

        if (rcBounds.GetWidth() < widthToler || rcBounds.GetHeight() < heightToler) // 包围框不满足要求的删除
            return;

        // 提取单个聚类的外包络点集
        Engine::Geometries::BaseAlgorithm::PointsPackage(clusterCroods, plyCroods);

        // 对提取的包围框点进行抽稀
        int npts = plyCroods.GetCount();
        plyCroods.Delete(npts - 1);
        hdmap_build::CommonUtil::Resample(plyCroods, 1.0);

        if (plyCroods.GetCount() < 3)
        {
            plyCroods.Clear();
            return;
        }
        plyCroods.Add(plyCroods[0]);

        // 调整为逆时针序列
        anticlockwisePolygon(plyCroods);
    }

    void processCrosswalk::anticlockwisePolygon(Engine::Base::Array<Engine::Geometries::Coordinate> &plyCroods) // 将多边形变化为逆时针序列
    {
        if (plyCroods.GetCount() < 4)
            return;

        Int32 flag = Engine::Geometries::BaseAlgorithm::PntMatchLine(plyCroods[0], plyCroods[2], plyCroods[1]);
        if (flag == 1) // 说明当前为顺时针
        {
            plyCroods.Reverse();
        }
    }

    void processCrosswalk::SACSegmentationLine(pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); // 创建采样一致性模型指针
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());                // 创建一个PointIndices结构体指针

        // 创建一个点云分割对象
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // 是否优化模型系数
        seg.setOptimizeCoefficients(true);
        // 设置模型　和　采样方法
        seg.setModelType(pcl::SACMODEL_LINE); // 直线模型
        seg.setMethodType(pcl::SAC_RANSAC);   // 随机采样一致性算法
        seg.setMaxIterations(1000);           // 迭代次数
        seg.setDistanceThreshold(0.01);       // 是否在平面上的阈值

        seg.setInputCloud(clusterCloud);      // 输入点云
        seg.segment(*inliers, *coefficients); // 分割　得到平面系数　已经在平面上的点的　索引

        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            return;
        }

        // 按照索引提取点云　　内点
        pcl::ExtractIndices<pcl::PointXYZ> extract_indices; // 索引提取器
        extract_indices.setIndices(inliers);                // 设置索引
        extract_indices.setInputCloud(clusterCloud);        // 设置输入点云

        extract_indices.filter(*output); // 提取对于索引的点云 内点
        std::cerr << "output point size : " << output->points.size() << std::endl;
    }

    bool processCrosswalk::filter_label_pcd(std::string global_pcd_path, pcl::PointCloud<pcl::PointXYZ> &crosswalkCloud, std::string outputDir, std::string ziduan, int value)
    {
        // 提取所有人行横道的点云 CLASS_TYPE_90 = 90,         //crosswalk_line
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(global_pcd_path, *pc_ptr);
        if (pc_ptr->points.empty())
        {
            std::cout << "点云文件无数据：global_pcd_path:" << global_pcd_path << std::endl;
            return false;
        }

        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter90(new pcl::PointCloud<MyColorPointType>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ori_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter90, ziduan, value);
        for (auto iter = cloudfilter90->begin(); iter != cloudfilter90->end(); iter++)
        {
            MyColorPointType &pcl_p = (*iter);
            ori_crosswalkCloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }
        if (ori_crosswalkCloud->empty())
        {
            std::cout << "无人行横道label点云：global_pcd_path:" << global_pcd_path << std::endl;
            return false;
        }
        pcl::io::savePCDFileBinary(outputDir + "/ori_crosswalk.pcd", *ori_crosswalkCloud);

        // 先按照2米范围聚类，每个聚类内进行统计滤波
        float radius = 2.0f;
        int minPtsSerch = 60;
        std::cout << "进行聚类 radius：" << radius << " minPtsSerch:" << minPtsSerch << std::endl;
        std::vector<std::vector<int>> clustersnew;
        Inference::DBSCAN(ori_crosswalkCloud, radius, minPtsSerch, clustersnew);

        pcl::PointCloud<pcl::PointXYZ>::Ptr allfilter_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < clustersnew.size(); ++i)
        {
            if (clustersnew[i].size() < CLUSTERMINDCOUNT)
                continue;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*ori_crosswalkCloud, clustersnew[i], *cluster_crosswalkCloud);

            // 进行统计滤波
            pcl::PointCloud<pcl::PointXYZ>::Ptr filter_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pclFilter::StatisticalOutlierRemoval(cluster_crosswalkCloud, filter_crosswalkCloud, 50, 1.0);
            if (filter_crosswalkCloud->empty())
            {
                continue;
            }
            // 将统计后滤波加入数据
            *allfilter_crosswalkCloud += *filter_crosswalkCloud;
        }

        // 进行统计滤波
        //        pcl::PointCloud<pcl::PointXYZ>::Ptr filter_crosswalkCloud(new pcl::PointCloud<pcl::PointXYZ>);
        //        pclFilter::StatisticalOutlierRemoval(ori_crosswalkCloud, filter_crosswalkCloud, 50, 1.0);
        if (allfilter_crosswalkCloud->empty())
        {
            std::cout << "统计滤波后点云为空" << std::endl;
            return false;
        }

        pcl::io::savePCDFileBinary(outputDir + "/filter_crosswalk.pcd", *allfilter_crosswalkCloud);
        crosswalkCloud = *allfilter_crosswalkCloud;

        return true;
    }

    bool processCrosswalk::get_stop_line_driver_dir(const Engine::Base::Array<Engine::Geometries::Coordinate> &stop_line, const Engine::Base::Array<Engine::Geometries::Coordinate> &lukou_center_pts, Engine::Geometries::Coordinate &direction)
    {
        if (stop_line.GetCount() != 2 || lukou_center_pts.IsEmpty())
            return false;

        // 寻找距离停止线最近的路口点
        Engine::Geometries::Coordinate *line_s = new Engine::Geometries::Coordinate(stop_line[0]);
        Engine::Geometries::Coordinate *line_e = new Engine::Geometries::Coordinate(stop_line[1]);
        double minDis = 9999999;
        int nearest_index = 0;
        Engine::Geometries::Coordinate nearest_proj_pt;
        for (int i = 0; i < lukou_center_pts.GetCount(); ++i)
        {
            Engine::Geometries::Coordinate *hit_test = new Engine::Geometries::Coordinate(lukou_center_pts[i]);
            Engine::Geometries::Coordinate proj_pt = Geometries::BaseAlgorithm::GetPtToLine(line_s, line_e, hit_test);
            double dis = proj_pt.DistanceXY(lukou_center_pts[i]);
            if (dis < minDis)
            {
                minDis = dis;
                nearest_index = i;
                nearest_proj_pt = proj_pt;
            }
        }

        // 确认人行横道移动方向
        Engine::Geometries::Coordinate moveDir = lukou_center_pts[nearest_index] - nearest_proj_pt;
        moveDir.z = 0.0;
        moveDir.Normalize();
        direction = moveDir;
    }

    void processCrosswalk::output_bev(pcl::PointCloud<pcl::PointXYZ> &point_cloud, double pixel_size_m, std::string output_folder, pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_cloud)
    {
        pcl::PointXYZ min_p1, max_p1;
        pcl::getMinMax3D(point_cloud, min_p1, max_p1);

        int min_x = (floor((min_p1.x) / 10)) * 10;
        int min_y = (floor((min_p1.y) / 10)) * 10;
        int max_x = (ceil((max_p1.x) / 10)) * 10;
        int max_y = (ceil((max_p1.y) / 10)) * 10;

        int width = (max_x - min_x) / pixel_size_m;
        int height = (max_y - min_y) / pixel_size_m;

        std::sort(point_cloud.begin(), point_cloud.end(),
                  [](pcl::PointXYZ pt1, pcl::PointXYZ pt2)
                  { return pt1.z < pt2.z; });

        //        cv::Mat ground_lable_image(height, width, CV_8UC1, 0);
        cv::Mat ground_lable_image = cv::Mat::zeros(height, width, CV_8UC1);
        for (pcl::PointXYZ p : point_cloud.points)
        {
            int x = round((p.x - min_x + 0.0001) / pixel_size_m);
            int y = round(height - (p.y - min_y + 0.0001) / pixel_size_m);
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }

            ground_lable_image.at<uint8_t>(y, x) = 255;
        }
        cv::imwrite((output_folder + "/crosswalk.png"), ground_lable_image);

        //        //自适应
        //        cv::Mat adaptiveThreshold_out;
        //        cv::adaptiveThreshold(ground_lable_image, adaptiveThreshold_out, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 11, -127.0);
        //        cv::imwrite((output_folder+ "/crosswalk_adaptiveThreshold.png"), adaptiveThreshold_out);
        //
        //        //高斯滤波
        //        cv::Mat GaussianBlur_out;
        //        cv::GaussianBlur(ground_lable_image, GaussianBlur_out, {21, 21}, 0, 0);
        //        cv::imwrite((output_folder+ "/crosswalk_GaussianBlur_out.png"), GaussianBlur_out);

        //        //先去噪--腐蚀+膨胀
        //        //腐蚀
        //        cv::Mat erode_element = getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
        //        cv::Mat erode_out;
        //        erode(ground_lable_image, erode_out, erode_element);
        //        cv::imwrite((output_folder+ "/crosswalk_quzao_fushi.png"), erode_out);
        //
        //        //腐蚀结果再膨胀
        //        cv::Mat dilate_element = getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
        //        cv::Mat dilate_out;
        //        cv::dilate(erode_out,dilate_out,dilate_element);
        //        cv::imwrite((output_folder+ "/crosswalk_quzao_pengzhang.png"), dilate_out);

        // 后填空--膨胀+腐蚀
        cv::Mat dilate_element_2 = getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20));
        cv::Mat dilate_out_2;
        cv::dilate(ground_lable_image, dilate_out_2, dilate_element_2);
        cv::imwrite((output_folder + "/crosswalk_tiankong_pengzhang.png"), dilate_out_2);

        cv::Mat erode_element_2 = getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20));
        cv::Mat erode_out_2;
        erode(dilate_out_2, erode_out_2, erode_element_2);
        cv::imwrite((output_folder + "/crosswalk_tikong_fushi.png"), erode_out_2);

        //        {
        //            std::ofstream file(output_folder+"/left_up_point.txt");
        //            file << min_x << " " << max_y;
        //            file.close();
        //        }

        // 结果转回点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ptr_cloud->points = point_cloud.points;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_cloud;
        kdtree_cloud.setInputCloud(ptr_cloud);
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                if (erode_out_2.at<uchar>(j, i) != 255)
                    continue;
                double x = i * pixel_size_m + min_x;
                double y = (height - j) * pixel_size_m + min_y;
                pcl::PointXYZ pt(float(x), float(y), 0);
                int K = 1;
                std::vector<int> Idx;
                std::vector<float> SquaredDistance;
                kdtree_cloud.nearestKSearch(pt, K, Idx, SquaredDistance);
                pt.z = ptr_cloud->at(Idx[0]).z;
                bev_cloud->push_back(pt);
            }
        }
        if (bev_cloud != NULL && bev_cloud->points.size() > 0)
        {
            pcl::io::savePCDFileBinary(output_folder + "/bev_crosswalk.pcd", *bev_cloud);
        }
    }

    void processCrosswalk::cluster_pts_realtion(const std::vector<std::vector<int>> &clustersnew, const pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_cloud)
    {

        //        int cluster_min_size = 10000;//5cm一个点并进行了填充处理，因此一个聚类内要求点数比较多
        for (int i = 0; i < clustersnew.size(); ++i)
        {
            if (clustersnew[i].size() < CLUSTERMINDCOUNT)
                continue;
            ClustrCloud_Realtion_Stopline curRealtionStrust;
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            Engine::Base::Array<Engine::Geometries::Coordinate> arrCroods;
            for (int j = 0; j < clustersnew[i].size(); j++)
            {
                pcl::PointXYZ curpt = bev_cloud->at(clustersnew[i][j]);
                x += curpt.x;
                y += curpt.y;
                z += curpt.z;
                clusterCloud->push_back(curpt);
                arrCroods.Add(Engine::Geometries::Coordinate(curpt.x, curpt.y, curpt.z));
            }
            curRealtionStrust.clusterIndex = i;
            curRealtionStrust.clusterpts = arrCroods;
            curRealtionStrust.middle_pt = Engine::Geometries::Coordinate(x / clustersnew[i].size(), y / clustersnew[i].size(), z / clustersnew[i].size());

            curRealtionStrust.isright = adjust_cluster(clusterCloud, curRealtionStrust.middle_pt);
            if (!curRealtionStrust.isright)
            {
                std::cout << i << ":聚类后的点云不符合要求，需重新处理" << std::endl;
            }
            else
            {
                // 计算拟合线
                Engine::Geometries::Coordinate ptStart, ptEnd;
                Engine::Algorithm::GeometricFit::LeastSquareLineFit(arrCroods, ptStart, ptEnd);
                curRealtionStrust.fit_axis.start_pt = ptStart;
                curRealtionStrust.fit_axis.end_pt = ptEnd;
                curRealtionStrust.fit_axis_normal = ptEnd - ptStart;
                curRealtionStrust.fit_axis_normal.z = 0.0;
                curRealtionStrust.fit_axis_normal.Normalize();
            }

            // 判断与此人行横道关联的路口点(最小距离要求小于50米)
            curRealtionStrust.r_center_index = -1;
            double minDis = 999999;
            for (int j = 0; j < _lukoupts.GetCount(); j++)
            {
                double dis = _lukoupts[j].DistanceXY(curRealtionStrust.middle_pt);
                if (dis < 50 && dis < minDis)
                {
                    minDis = dis;
                    curRealtionStrust.center_pt = _lukoupts[j];
                    curRealtionStrust.r_center_index = j;
                }
            }

            // 当中心点正确时寻找与人行横道距离最近的停止线 1、与自身拟合夹角<30° 2、到停止线的垂距<10 3、如果有中心点选择与停止线在路口方向投影向量最小的 反之，选择与停止线中点最近的
            curRealtionStrust.r_stoplineIndex = -1;
            if (curRealtionStrust.isright)
            {
                double min_dis_stopline = 999999;
                std::map<int, Engine::Base::Array<Stopline_Info>>::iterator it;
                it = _center_stoplines.find(curRealtionStrust.r_center_index);
                if (it != _center_stoplines.end())
                {
                    Engine::Base::Array<Stopline_Info> lukou_stoplines = it->second;
                    for (int j = 0; j < lukou_stoplines.GetCount(); ++j)
                    {
                        double dot = fabs(lukou_stoplines[j].stopline_Normal.DotProduct(curRealtionStrust.fit_axis_normal));
                        if (dot < 0.866)
                            continue;
                        double dis = Geometries::BaseAlgorithm3D::DisPtToLine(lukou_stoplines[j].start_pt, lukou_stoplines[j].end_pt, curRealtionStrust.middle_pt);
                        if (dis > 10.0)
                            continue;
                        double compareDis = 9999999;
                        if (curRealtionStrust.r_center_index < 0)
                        {
                            compareDis = curRealtionStrust.middle_pt.DistanceXY(lukou_stoplines[j].middle_pt);
                        }
                        else
                        {
                            Engine::Geometries::Coordinate e = curRealtionStrust.middle_pt - lukou_stoplines[j].middle_pt;
                            e.z = 0.0;
                            e.Normalize();
                            compareDis = e.DotProduct(lukou_stoplines[j].to_lulou_normal);
                            if (compareDis < 0)
                                continue;
                        }
                        if (compareDis < min_dis_stopline)
                        {
                            min_dis_stopline = compareDis;
                            curRealtionStrust.stopline_realtion = lukou_stoplines[j];
                            curRealtionStrust.r_stoplineIndex = lukou_stoplines[j].stopline_index;
                        }
                    }
                }
            }
            int isright = curRealtionStrust.isright;
            std::cout << i << ":聚类中点是否正确:" << isright << " 关联的路口点: " << curRealtionStrust.r_center_index << "  关联的停止线:" << curRealtionStrust.r_stoplineIndex << std::endl;
            _crosswalk_info.Add(curRealtionStrust);
        }
    }

    void processCrosswalk::check_edge(ClustrCloud_Realtion_Stopline &one_clustercloud)
    {
        Engine::Base::Array<Engine::Geometries::Coordinate> arrPts = one_clustercloud.clusterpts;
        if (arrPts.GetCount() < 2)
            return;

        Engine::Geometries::Coordinate direction = one_clustercloud.move_normal;
        Engine::Geometries::Coordinate endpts = one_clustercloud.middle_pt + direction * 5.0;

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> arrtestPts;
        Engine::Base::Array<Engine::Geometries::Coordinate> testmovePts;
        testmovePts.Add(one_clustercloud.middle_pt);
        testmovePts.Add(endpts);
        arrtestPts.Add(testmovePts);

        double up_disp = 0.05;
        double up_length = 5.0;
        int up_neighborhoodCount = 10;
        double right_disp = 0.10;
        double right_length = 30.0;
        int right_neighborhoodCount = 10;

        // up_line down_line(重新)
        int up_max_diff_index = -1;
        int down_max_diff_index = -1;
        get_diff_max_pts(one_clustercloud, direction, up_disp, up_length, up_neighborhoodCount, up_max_diff_index, down_max_diff_index);
        std::cout << "up_max_diff_index:" << up_max_diff_index << "---down_max_diff_index:" << down_max_diff_index << std::endl;

        // 向量顺时针旋转90度
        Engine::Geometries::Coordinate trans_direction(direction.y, -direction.x, direction.z);
        Engine::Base::Array<Engine::Geometries::Coordinate> testtransPts;
        testtransPts.Add(one_clustercloud.middle_pt);
        testtransPts.Add(one_clustercloud.middle_pt + trans_direction * 30.0);
        arrtestPts.Add(testtransPts);

        hdmap_build::CommonUtil::WriteToOBJ(arrtestPts, _output, to_string(one_clustercloud.clusterIndex) + "_move_dir", true);
        int right_max_diff_index = -1;
        int left_max_diff_index = -1;
        get_diff_max_pts(one_clustercloud, trans_direction, right_disp, right_length, right_neighborhoodCount, right_max_diff_index, left_max_diff_index);
        std::cout << "right_max_diff_index:" << up_max_diff_index << "---left_max_diff_index:" << down_max_diff_index << std::endl;

        if (up_max_diff_index < 0 || down_max_diff_index < 0 || right_max_diff_index < 0 || left_max_diff_index < 0)
            return;

        Engine::Geometries::Coordinate up_pt = one_clustercloud.middle_pt + direction * (up_max_diff_index + 1) * up_disp;
        Engine::Geometries::Coordinate down_pt = one_clustercloud.middle_pt + direction * (down_max_diff_index + 1) * up_disp * (-1.0);
        Engine::Geometries::Coordinate right_pt = one_clustercloud.middle_pt + trans_direction * (right_max_diff_index + 1) * right_disp;
        Engine::Geometries::Coordinate left_pt = one_clustercloud.middle_pt + trans_direction * (left_max_diff_index + 1) * right_disp * (-1.0);
        // Engine::Base::Array<Engine::Geometries::Coordinate> arrtest;
        // arrtest.Add(up_pt);
        // arrtest.Add(down_pt);
        // arrtest.Add(right_pt);
        // arrtest.Add(left_pt);
        // hdmap_build::CommonUtil::WriteToTxt(arrtest, "/home/test/data/27544/crosswalk/", "test_pts", true);
        Engine::Base::Array<Engine::Geometries::Coordinate> resEdgePts;
        Engine::Geometries::Coordinate left_up_pt = up_pt + trans_direction * (left_max_diff_index + 1) * right_disp * (-1.0);
        Engine::Geometries::Coordinate left_down_pt = down_pt + trans_direction * (left_max_diff_index + 1) * right_disp * (-1.0);
        Engine::Geometries::Coordinate right_down_pt = down_pt + trans_direction * (right_max_diff_index + 1) * right_disp;
        Engine::Geometries::Coordinate right_up_pt = up_pt + trans_direction * (right_max_diff_index + 1) * right_disp;
        resEdgePts.Add(left_up_pt);
        resEdgePts.Add(left_down_pt);
        resEdgePts.Add(right_down_pt);
        resEdgePts.Add(right_up_pt);
        resEdgePts.Add(left_up_pt);
        one_clustercloud.boxpts = resEdgePts;
    }

    void processCrosswalk::check_edge2(ClustrCloud_Realtion_Stopline &one_clustercloud) // 修改提取外包框策略
    {
        Engine::Base::Array<Engine::Geometries::Coordinate> arrPts = one_clustercloud.clusterpts;
        if (arrPts.GetCount() < 2)
            return;

        Engine::Geometries::Coordinate direction = one_clustercloud.move_normal;
        Engine::Geometries::Coordinate endpts = one_clustercloud.middle_pt + direction * 5.0;

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> arrtestPts;
        Engine::Base::Array<Engine::Geometries::Coordinate> testmovePts;
        testmovePts.Add(one_clustercloud.middle_pt);
        testmovePts.Add(endpts);
        arrtestPts.Add(testmovePts);

        double up_disp = 0.05;
        double up_length = 20.0;
        int up_neighborhoodCount = 10;
        double right_disp = 0.10;
        double right_length = 30.0;
        int right_neighborhoodCount = 10;

        // up_line down_line(重新)
        int up_max_diff_index = -1;
        int down_max_diff_index = -1;
        get_up_down_edge(one_clustercloud, direction, up_disp, up_length, up_neighborhoodCount, up_max_diff_index, down_max_diff_index);
        std::cout << "up_max_diff_index:" << up_max_diff_index << "---down_max_diff_index:" << down_max_diff_index << std::endl;

        // 向量顺时针旋转90度
        Engine::Geometries::Coordinate trans_direction(direction.y, -direction.x, direction.z);
        Engine::Base::Array<Engine::Geometries::Coordinate> testtransPts;
        testtransPts.Add(one_clustercloud.middle_pt);
        testtransPts.Add(one_clustercloud.middle_pt + trans_direction * 30.0);
        arrtestPts.Add(testtransPts);

        hdmap_build::CommonUtil::WriteToOBJ(arrtestPts, _output, to_string(one_clustercloud.clusterIndex) + "_move_dir", true);
        int right_max_diff_index = -1;
        int left_max_diff_index = -1;
        get_diff_max_pts(one_clustercloud, trans_direction, right_disp, right_length, right_neighborhoodCount, right_max_diff_index, left_max_diff_index);
        std::cout << "right_max_diff_index:" << right_max_diff_index << "---left_max_diff_index:" << left_max_diff_index << std::endl;

        if (up_max_diff_index < 0 || down_max_diff_index < 0 || right_max_diff_index < 0 || left_max_diff_index < 0)
            return;
        // 上下边的求解有变化
        Engine::Geometries::Coordinate up_pt = one_clustercloud.start_pt + direction * (up_max_diff_index - 3) * up_disp;
        Engine::Geometries::Coordinate down_pt = one_clustercloud.start_pt + direction * (down_max_diff_index + 5) * up_disp;

        Engine::Geometries::Coordinate right_pt = one_clustercloud.middle_pt + trans_direction * (right_max_diff_index + 1) * right_disp;
        Engine::Geometries::Coordinate left_pt = one_clustercloud.middle_pt + trans_direction * (left_max_diff_index + 1) * right_disp * (-1.0);
        //        Engine::Base::Array<Engine::Geometries::Coordinate> arrtest;
        //        arrtest.Add(up_pt);
        //        arrtest.Add(down_pt);
        //        arrtest.Add(right_pt);
        //        arrtest.Add(left_pt);
        //        hdmap_build::CommonUtil::WriteToTxt(arrtest, _output, std::to_string(one_clustercloud.clusterIndex)+"_"+"check_edge2", true);
        Engine::Base::Array<Engine::Geometries::Coordinate> resEdgePts;
        double up_move_dis = (up_pt - left_pt).DotProduct(direction);
        double down_move_dis = (left_pt - down_pt).DotProduct(direction);
        Engine::Geometries::Coordinate left_up_pt = left_pt + direction * up_move_dis;
        Engine::Geometries::Coordinate left_down_pt = left_pt + direction * down_move_dis * (-1.0);
        Engine::Geometries::Coordinate right_down_pt = right_pt + direction * down_move_dis * (-1.0);
        Engine::Geometries::Coordinate right_up_pt = right_pt + direction * up_move_dis;
        resEdgePts.Add(left_up_pt);
        resEdgePts.Add(left_down_pt);
        resEdgePts.Add(right_down_pt);
        resEdgePts.Add(right_up_pt);
        resEdgePts.Add(left_up_pt);
        one_clustercloud.boxpts = resEdgePts;
        //        hdmap_build::CommonUtil::WriteToTxt(one_clustercloud.boxpts, _output, std::to_string(one_clustercloud.clusterIndex)+"_"+"check_edge2_res", true);
    }

    void processCrosswalk::line_inter_disp(const Engine::Geometries::Coordinate &ori_pt, const Engine::Geometries::Coordinate &direction, double disp, double length, Engine::Base::Array<Engine::Geometries::Coordinate> &res_pts)
    {
        int n = int(length / disp);
        if (n < 1)
            return;
        for (int i = 0; i <= n; ++i)
        {
            double dis = i * disp;
            Engine::Geometries::Coordinate newPt = ori_pt + direction * dis;
            res_pts.Add(newPt);
        }
    }

    int processCrosswalk::diff_max(const Engine::Base::Array<int> &arrCount, int neighborhoodCount)
    {
        int n = arrCount.GetCount();
        if (neighborhoodCount >= (n / 2)) // 没办法进行邻域计算
            neighborhoodCount = 0;
        int res = -1;
        int maxdiff = 0;
        //        for (int i = 0; i < arrCount.GetCount()-1; ++i) {
        //            int diff = fabs(arrCount[i]-arrCount[i+1]);
        //            if (diff > maxdiff)
        //            {
        //                maxdiff = diff;
        //                res = i;
        //            }
        //        }
        for (int i = neighborhoodCount; i < n - neighborhoodCount; ++i)
        {
            int before_i_count = 0;
            for (int j = i; j >= (i - neighborhoodCount); j--)
            {
                before_i_count += arrCount[j];
            }
            int after_i_count = 0;
            for (int j = i; j <= (i + neighborhoodCount); ++j)
            {
                after_i_count += arrCount[j];
            }

            int diff = before_i_count - after_i_count;
            if (diff > maxdiff)
            {
                maxdiff = diff;
                res = i;
            }
        }
        return res;
    }

    int processCrosswalk::diff_min(const Engine::Base::Array<int> &arrCount, int neighborhoodCount)
    {
        int n = arrCount.GetCount();
        if (neighborhoodCount >= (n / 2)) // 没办法进行邻域计算
            neighborhoodCount = 0;
        int res = -1;
        int mindiff = 999999999;
        //        for (int i = 0; i < arrCount.GetCount()-1; ++i) {
        //            int diff = fabs(arrCount[i]-arrCount[i+1]);
        //            if (diff > maxdiff)
        //            {
        //                maxdiff = diff;
        //                res = i;
        //            }
        //        }
        for (int i = neighborhoodCount; i < n - neighborhoodCount; ++i)
        {
            int before_i_count = 0;
            for (int j = i; j >= (i - neighborhoodCount); j--)
            {
                before_i_count += arrCount[j];
            }
            int after_i_count = 0;
            for (int j = i; j <= (i + neighborhoodCount); ++j)
            {
                after_i_count += arrCount[j];
            }

            int diff = before_i_count - after_i_count;
            if (diff < mindiff)
            {
                mindiff = diff;
                res = i;
            }
        }
        return res;
    }

    void processCrosswalk::get_diff_max_pts(const ClustrCloud_Realtion_Stopline &one_clustercloud, const Engine::Geometries::Coordinate &direction, double disp, double length, int neighborhoodCount, int &upindex, int &downindex)
    {
        int nSegment = int(length / disp);
        Engine::Base::Array<int> upProjptsCount(nSegment);
        Engine::Base::Array<int> downProjptsCount(nSegment);
        Engine::Geometries::Coordinate middle_pt = one_clustercloud.middle_pt;
        Engine::Base::Array<Engine::Geometries::Coordinate> arrPts = one_clustercloud.clusterpts;
        for (int i = 0; i < arrPts.GetCount(); ++i)
        {
            //            if (fabs(arrPts[i].x + 9.55) < 0.01 && fabs(arrPts[i].y + 2.60) < 0.01)
            //            {
            //                int m = 0;
            //            }
            Engine::Geometries::Coordinate e = arrPts[i] - middle_pt;
            e.z = 0.0;
            double measure = e.DotProduct(direction);
            int index = int(fabs(measure) / disp);
            //            std::cout << "measure:" << measure <<"index:"<< index <<std::endl;
            if (index >= nSegment)
                continue;
            if (measure > 0)
            {
                upProjptsCount[index]++;
            }
            else
            {
                downProjptsCount[index]++;
            }
        }
        upindex = diff_max(upProjptsCount, neighborhoodCount);
        downindex = diff_max(downProjptsCount, neighborhoodCount);
    }

    void processCrosswalk::get_up_down_edge(ClustrCloud_Realtion_Stopline &one_clustercloud, const Engine::Geometries::Coordinate &direction, double disp, double length, int neighborhoodCount, int &upindex, int &downindex)
    {
        int nSegment = int(2.0 * length / disp); // 确保能覆盖整个人行横道的宽
        Engine::Base::Array<Engine::Base::Array<int>> upProjptsCount(nSegment);
        Engine::Geometries::Coordinate middle_pt = one_clustercloud.middle_pt;
        //        Engine::Geometries::Coordinate direction = one_clustercloud.move_normal;
        //        one_clustercloud.start_pt = one_clustercloud.middle_pt + direction * length * (-1.0);
        Engine::Base::Array<Engine::Geometries::Coordinate> arrPts = one_clustercloud.clusterpts;
        for (int i = 0; i < arrPts.GetCount(); ++i)
        {
            //            if (fabs(arrPts[i].x + 9.55) < 0.01 && fabs(arrPts[i].y + 2.60) < 0.01)
            //            {
            //                int m = 0;
            //            }
            Engine::Geometries::Coordinate e = arrPts[i] - one_clustercloud.start_pt;
            e.z = 0.0;
            double measure = e.DotProduct(direction);
            if (measure < 0)
                continue;
            int index = int(fabs(measure) / disp);
            //            std::cout << "measure:" << measure <<"index:"<< index <<std::endl;
            if (index >= nSegment)
                continue;
            upProjptsCount[index].Add(i);
        }
        Engine::Base::Array<int> projectsCountArr;
        for (int i = 0; i < nSegment; ++i)
        {
            projectsCountArr.Add(upProjptsCount[i].GetCount());
        }
        int minIndex = diff_min(projectsCountArr, neighborhoodCount);
        if ((minIndex + 1) * disp > 10.0) // 计算的边缘距离停止线过远放弃
            return;
        if ((minIndex + 1) * disp < 0.8)
            return; // 距离停止线过近也删除
        int maxIndex = diff_max(projectsCountArr, neighborhoodCount);
        if (maxIndex < minIndex)
            return;
        double weight = (maxIndex - minIndex) * disp;
        std::cout << "计算的宽度: " << weight << "  过长将强制转换为6.5米" << std::endl;
        if (weight > 6.5) // 计算的宽度大于6.5米，强制转化为6.5米
        {
            maxIndex = minIndex + int(6.5 / disp);
        }
        upindex = maxIndex;
        downindex = minIndex;
        std::cout << "upindex:" << upindex << "--downindex:" << downindex << std::endl;
        Engine::Base::Array<Engine::Geometries::Coordinate> arrClipPts;
        for (int i = minIndex; i <= maxIndex; ++i)
        {
            for (int j = 0; j < upProjptsCount[i].GetCount(); ++j)
            {
                int index_pt = upProjptsCount[i][j];
                arrClipPts.Add(arrPts[index_pt]);
            }
        }
        one_clustercloud.clusterpts = arrClipPts;
        one_clustercloud.middle_pt = hdmap_build::CommonUtil::GetAveragePt(arrClipPts);
    }

    bool processCrosswalk::adjust_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &clusterCloud, const Engine::Geometries::Coordinate &midPt)
    {
        // 根据中心点初步判断聚类是否正确
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_cloud;
        kdtree_cloud.setInputCloud(clusterCloud);
        double r = 0.5;
        std::vector<int> Idx;
        std::vector<float> SquaredDistance;
        pcl::PointXYZ midpt(midPt.x, midPt.y, midPt.z);
        kdtree_cloud.radiusSearch(midpt, r, Idx, SquaredDistance);
        std::cout << "中心点0.5米范围内找到的点个数：" << Idx.size() << std::endl;
        if (Idx.size() > 100)
            return true;
        else
            return false;
    }

    void processCrosswalk::inputLuKouCenterPts(const Engine::Base::Array<Engine::Geometries::Coordinate> &lukou_center_pts)
    {
        _lukoupts = lukou_center_pts;
    }

    void processCrosswalk::inputStoplines(const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &stop_lines)
    {
        Engine::Base::Array<Stopline_Info> stop_infos;
        for (int i = 0; i < stop_lines.GetCount(); ++i)
        {
            Stopline_Info stop_info;
            stop_info.stopline_index = i;
            stop_info.start_pt = stop_lines[i][0];
            stop_info.end_pt = stop_lines[i][1];
            double dis = stop_info.start_pt.DistanceXY(stop_info.end_pt);
            if (dis > 15.0)
            {
                std::cout << i << ":停止线大于15米，不作为参考" << std::endl;
                continue;
            }
            stop_info.middle_pt = (stop_info.start_pt + stop_info.end_pt) * 0.5;
            Engine::Geometries::Coordinate normal = stop_info.end_pt - stop_info.start_pt;
            normal.z = 0;
            normal.Normalize();
            stop_info.stopline_Normal = normal;
            stop_info.center_index = -1;
            // 找最近的且距离小于50米的路口点
            double minDis = 9999999;
            for (int j = 0; j < _lukoupts.GetCount(); ++j)
            {
                double dis = _lukoupts[j].DistanceXY(stop_info.middle_pt);
                if (dis < 100 && dis < minDis)
                {
                    minDis = dis;
                    stop_info.center_index = j;
                    stop_info.center_pt = _lukoupts[j];
                }
            }
            // 如有路口点 需将停止线朝向路口的法向量计算出来
            if (stop_info.center_index >= 0)
            {
                Engine::Geometries::Coordinate *line_s = new Engine::Geometries::Coordinate(stop_info.start_pt);
                Engine::Geometries::Coordinate *line_e = new Engine::Geometries::Coordinate(stop_info.end_pt);
                Engine::Geometries::Coordinate *hit_test = new Engine::Geometries::Coordinate(stop_info.center_pt);
                Engine::Geometries::Coordinate proj_pt = Geometries::BaseAlgorithm::GetPtToLine(line_s, line_e, hit_test);
                stop_info.center_pt_porject = proj_pt;
                Engine::Geometries::Coordinate e = stop_info.center_pt - proj_pt;
                e.z = 0.0;
                e.Normalize();
                stop_info.to_lulou_normal = e;
            }
            _center_stoplines[stop_info.center_index].Add(stop_info);
            stop_infos.Add(stop_info);
        }
        _stoplines = stop_infos;
    }

    void processCrosswalk::getProjNormal(Engine::Geometries::Coordinate s_pt, Engine::Geometries::Coordinate e_pt, Engine::Geometries::Coordinate hit_pt, Engine::Geometries::Coordinate &proj_pt, Engine::Geometries::Coordinate &proj_normal)
    {
        Engine::Geometries::Coordinate *line_s = new Engine::Geometries::Coordinate(s_pt);
        Engine::Geometries::Coordinate *line_e = new Engine::Geometries::Coordinate(e_pt);
        Engine::Geometries::Coordinate *hit_test = new Engine::Geometries::Coordinate(hit_pt);
        proj_pt = Geometries::BaseAlgorithm::GetPtToLine(line_s, line_e, hit_test);

        proj_normal = hit_pt - proj_pt;
        proj_normal.z = 0.0;
        proj_normal.Normalize();
    }

    void processCrosswalk::deal_wrong(ClustrCloud_Realtion_Stopline &one_clustercloud, Engine::Base::Array<ClustrCloud_Realtion_Stopline> &newClusterArr)
    {
        Engine::Base::Array<Stopline_Info> r_stopline = _center_stoplines[one_clustercloud.r_center_index];
        if (one_clustercloud.r_center_index < 0)
        {
            // 计算与其相关的停止线
            for (int i = 0; i < r_stopline.GetCount(); ++i)
            {
                double dis = one_clustercloud.middle_pt.DistanceXY(r_stopline[i].middle_pt);
                if (dis > 30)
                    continue;
                getProjNormal(r_stopline[i].start_pt, r_stopline[i].end_pt, one_clustercloud.middle_pt, one_clustercloud.start_pt, r_stopline[i].to_lulou_normal);
                Engine::Base::Array<Engine::Geometries::Coordinate> clippts;
                // 对于聚类较大的数据，停止线需在中心点与停止线中间
                clipPtsByMiddleptStopline(one_clustercloud, r_stopline[i], clippts);
                if (clippts.GetCount() < CLUSTERMINDCOUNT)
                    continue;

                std::string name = to_string(one_clustercloud.clusterIndex) + "_" + to_string(clippts.GetCount());
                hdmap_build::CommonUtil::WriteToTxt(clippts, _output, name, true);

                ClustrCloud_Realtion_Stopline newcluster;
                newcluster = one_clustercloud;
                newcluster.clusterpts = clippts;
                newcluster.middle_pt = hdmap_build::CommonUtil::GetAveragePt(clippts);
                newcluster.stopline_realtion = r_stopline[i];
                newcluster.r_stoplineIndex = r_stopline[i].stopline_index;
                newcluster.move_normal = r_stopline[i].to_lulou_normal;
                newcluster.isright = true;

                getConcaveHull(clippts, newcluster.concavehull_ptr);
                check_edge2(newcluster);
                newClusterArr.Add(newcluster);
            }
        }
        else
        {
            for (int i = 0; i < r_stopline.GetCount(); ++i)
            {
                Engine::Base::Array<Engine::Geometries::Coordinate> clippts;
                clipPtsByStopline(one_clustercloud.clusterpts, r_stopline[i], clippts);
                if (clippts.GetCount() < CLUSTERMINDCOUNT)
                    continue;
                ClustrCloud_Realtion_Stopline newcluster;
                newcluster = one_clustercloud;
                newcluster.clusterpts = clippts;
                newcluster.middle_pt = hdmap_build::CommonUtil::GetAveragePt(clippts);
                newcluster.stopline_realtion = r_stopline[i];
                newcluster.r_stoplineIndex = r_stopline[i].stopline_index;
                newcluster.move_normal = r_stopline[i].to_lulou_normal;
                newcluster.start_pt = r_stopline[i].center_pt_porject;
                newcluster.isright = true;

                getConcaveHull(clippts, newcluster.concavehull_ptr);
                check_edge2(newcluster);
                newClusterArr.Add(newcluster);
            }
        }
    }

    void processCrosswalk::deal_r_center_stopline(ClustrCloud_Realtion_Stopline &one_clustercloud) // 处理既有中心点也有停止线相关联的人行横道
    {
        // 将停止线朝向路口的单位向量作为移动方向
        Engine::Base::Array<Engine::Geometries::Coordinate> newPts;
        clipPtsByStopline(one_clustercloud.clusterpts, one_clustercloud.stopline_realtion, newPts);
        if (newPts.GetCount() < CLUSTERMINDCOUNT)
            return;
        one_clustercloud.clusterpts = newPts;
        one_clustercloud.middle_pt = hdmap_build::CommonUtil::GetAveragePt(newPts);
        one_clustercloud.move_normal = one_clustercloud.stopline_realtion.to_lulou_normal;
        // check_edge(one_clustercloud);
        ///////////////////////////////////////////////////////////////////
        getConcaveHull(newPts, one_clustercloud.concavehull_ptr);
        one_clustercloud.start_pt = one_clustercloud.stopline_realtion.center_pt_porject;
        check_edge2(one_clustercloud);
    }
    void processCrosswalk::deal_r_center(ClustrCloud_Realtion_Stopline &one_clustercloud)
    {
        // 路口中心点到拟合轴线的的单位向量作为移动方向
        Engine::Geometries::Coordinate proj_pt;
        getProjNormal(one_clustercloud.fit_axis.start_pt, one_clustercloud.fit_axis.end_pt, one_clustercloud.center_pt, proj_pt, one_clustercloud.move_normal);
        check_edge(one_clustercloud);
    }
    void processCrosswalk::deal_r_stopline(ClustrCloud_Realtion_Stopline &one_clustercloud)
    {
        // 将停止线朝向中点的方向作为移动方向
        Engine::Geometries::Coordinate proj_pt;
        getProjNormal(one_clustercloud.stopline_realtion.start_pt, one_clustercloud.stopline_realtion.end_pt, one_clustercloud.middle_pt, proj_pt, one_clustercloud.move_normal);
        one_clustercloud.stopline_realtion.to_lulou_normal = one_clustercloud.move_normal;

        Engine::Base::Array<Engine::Geometries::Coordinate> newPts;
        clipPtsByStopline(one_clustercloud.clusterpts, one_clustercloud.stopline_realtion, newPts);
        if (newPts.GetCount() < CLUSTERMINDCOUNT)
            return;
        one_clustercloud.clusterpts = newPts;
        one_clustercloud.middle_pt = hdmap_build::CommonUtil::GetAveragePt(newPts);

        // check_edge(one_clustercloud);
        ///////////////////////////////////////////////////////////////////
        getConcaveHull(newPts, one_clustercloud.concavehull_ptr);
        one_clustercloud.start_pt = proj_pt;
        check_edge2(one_clustercloud);
    }

    void processCrosswalk::deal_no_r(ClustrCloud_Realtion_Stopline &one_clustercloud) // 处理只有未有任何关联的人行横道
    {
        // 将本身轴线的法向量作为移动方向
        Engine::Geometries::Coordinate trans_direction(one_clustercloud.fit_axis_normal.y, -one_clustercloud.fit_axis_normal.x, 0.0);
        one_clustercloud.move_normal = trans_direction;
        check_edge(one_clustercloud);
    }

    void processCrosswalk::clipPtsByStopline(const Engine::Base::Array<Engine::Geometries::Coordinate> &ori_cluster_pts, Stopline_Info stopline, Engine::Base::Array<Engine::Geometries::Coordinate> &clipPts)
    {
        Engine::Geometries::Coordinate stopline_middle = stopline.middle_pt;
        for (int i = 0; i < ori_cluster_pts.GetCount(); ++i)
        {
            Engine::Geometries::Coordinate diff = ori_cluster_pts[i] - stopline_middle;
            diff.z = 0.0;
            double dis = diff.DotProduct(stopline.to_lulou_normal);
            if (dis < 0 || dis > 13)
                continue;
            clipPts.Add(ori_cluster_pts[i]);
        }
    }

    void processCrosswalk::clipPtsByMiddleptStopline(const ClustrCloud_Realtion_Stopline &one_clustercloud, Stopline_Info stopline, Engine::Base::Array<Engine::Geometries::Coordinate> &clipPts)
    {
        // 对于聚类较大的数据，停止线需在中心点与停止线中间
        double disMax = one_clustercloud.middle_pt.DistanceXY(one_clustercloud.start_pt);
        for (int k = 0; k < one_clustercloud.clusterpts.GetCount(); ++k)
        {
            Engine::Geometries::Coordinate diff = one_clustercloud.clusterpts[k] - one_clustercloud.start_pt;
            diff.z = 0.0;
            double dis = diff.DotProduct(stopline.to_lulou_normal);
            if (dis < 0 || dis > disMax)
                continue;
            clipPts.Add(one_clustercloud.clusterpts[k]);
        }
    }

    void processCrosswalk::fix_edge_icp(ClustrCloud_Realtion_Stopline &one_clustercloud)
    {
        if (one_clustercloud.clusterpts.IsEmpty() || one_clustercloud.boxpts.IsEmpty())
            return;
        Engine::Base::Array<Engine::Geometries::Coordinate> interPts = one_clustercloud.boxpts;
        std::vector<int> ori_pts_index;
        if (!getInterpolationLine(interPts, ori_pts_index, 0.05, 1))
            return;
        if (one_clustercloud.concavehull_ptr == NULL || one_clustercloud.concavehull_ptr->points.empty())
        {
            getConcaveHull(one_clustercloud.clusterpts, one_clustercloud.concavehull_ptr);
        }
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < one_clustercloud.clusterpts.GetCount(); ++i)
        {
            cloud_1_ptr->push_back(pcl::PointXYZ(one_clustercloud.clusterpts[i].x, one_clustercloud.clusterpts[i].y, one_clustercloud.clusterpts[i].z));
        }
        pcl::io::savePCDFileBinary(_output + "/" + to_string(one_clustercloud.clusterIndex) + "_crosswalk_cloud.pcd", *cloud_1_ptr);

        //        //求聚类点云的凹包
        //        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        //        pcl::ConcaveHull<pcl::PointXYZ> chull;  //创建多边形提取对象
        //        chull.setInputCloud (cloud_1_ptr);  //设置输入点云为投影后点云cloud_projected
        //        chull.setAlpha (0.1);                   //设置alpha值为0.1
        //        chull.reconstruct (*cloud_hull);       //重建提取创建凹多边形
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull = one_clustercloud.concavehull_ptr;
        pcl::io::savePCDFileBinary(_output + "/" + to_string(one_clustercloud.clusterIndex) + "_crosswalk_cloud_hull.pcd", *cloud_hull);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < interPts.GetCount(); ++i)
        {
            cloud_2_ptr->push_back(pcl::PointXYZ(interPts[i].x, interPts[i].y, interPts[i].z));
        }

        icp.setInputTarget(cloud_hull);  // 参照点
        icp.setInputSource(cloud_2_ptr); // 待移动点
        icp.setMaxCorrespondenceDistance(5);
        // // 设置最大迭代次数(默认 1)
        icp.setMaximumIterations(300);
        // // 设置 两次变换矩阵之间的差值 (默认 2)
        // icp.setTransformationEpsilon(1e-8);
        // // 设置 均方误差(默认 3)
        // icp.setEuclideanFitnessEpsilon(1);
        icp.setUseReciprocalCorrespondences(1);
        icp.align(*cloud_2_ptr);
        for (int i = 0; i < ori_pts_index.size(); ++i)
        {
            pcl::PointXYZ fixPt = cloud_2_ptr->at(ori_pts_index[i]);
            one_clustercloud.icpboxpts.Add(Engine::Geometries::Coordinate(fixPt.x, fixPt.y, fixPt.z));
        }
    }

    bool processCrosswalk::bev_obj_icp(Engine::Base::Array<Engine::Geometries::Coordinate> &bev_obj, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull)
    {
        std::vector<int> ori_pts_index;
        Engine::Base::Array<Engine::Geometries::Coordinate> interPts = bev_obj;
        if (!getInterpolationLine(interPts, ori_pts_index, 0.05, 1))
            return false;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < interPts.GetCount(); ++i)
        {
            cloud_2_ptr->push_back(pcl::PointXYZ(interPts[i].x, interPts[i].y, interPts[i].z));
        }

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
        icp.setInputTarget(cloud_hull);  // 参照点
        icp.setInputSource(cloud_2_ptr); // 待移动点
        icp.setMaxCorrespondenceDistance(5);
        // // 设置最大迭代次数(默认 1)
        icp.setMaximumIterations(300);
        // // 设置 两次变换矩阵之间的差值 (默认 2)
        // icp.setTransformationEpsilon(1e-8);
        // 设置 均方误差(默认 3)
        icp.setEuclideanFitnessEpsilon(1);
        icp.setUseReciprocalCorrespondences(1);
        icp.align(*cloud_2_ptr);

        if (icp.hasConverged())
        {
            std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
            bev_obj.Clear();
            for (int i = 0; i < ori_pts_index.size(); ++i)
            {
                pcl::PointXYZ fixPt = cloud_2_ptr->at(ori_pts_index[i]);
                bev_obj.Add(Engine::Geometries::Coordinate(fixPt.x, fixPt.y, fixPt.z));
            }
            return true;
        }
        return false;
    }

    bool processCrosswalk::adjust_box(const Engine::Base::Array<Engine::Geometries::Coordinate> &clusterpts, const Engine::Base::Array<Engine::Geometries::Coordinate> &boxpts)
    {
        if (boxpts.GetCount() != 5)
            return false;
        double d1 = boxpts[0].DistanceXY(boxpts[1]);
        double d2 = boxpts[1].DistanceXY(boxpts[2]);
        // std::cout<< d1 << "  d2:"<< d2 <<std::endl;
        double n = d1 * d2 / 0.0025;
        if (n < 1)
            return false;
        Engine::Geometries::Coordinate sPt = boxpts[0];
        Engine::Geometries::Coordinate ePt1 = boxpts[1];
        Engine::Geometries::Coordinate ePt2 = boxpts[3];
        Engine::Geometries::Coordinate e1 = ePt1 - sPt;
        e1.z = 0;
        e1.Normalize();
        Engine::Geometries::Coordinate e2 = ePt2 - sPt;
        e2.Normalize();
        e2.z = 0;
        double filterNum = 0;
        for (int i = 0; i < clusterpts.GetCount(); ++i)
        {
            Engine::Geometries::Coordinate e = clusterpts[i] - sPt;
            e.z = 0;
            double dis1 = e.DotProduct(e1);
            if (dis1 < 0 || dis1 > d1)
                continue;

            double dis2 = e.DotProduct(e2);
            if (dis2 < 0 || dis2 > d2)
                continue;
            filterNum += 1;
        }
        double radio = filterNum / n;
        std::cout << "  n:" << n << " filterNum:" << filterNum << "  radio:" << radio << std::endl;
        if (radio < 0.6)
            return false;
        return true;
    }

    bool processCrosswalk::getInterpolationLine(Engine::Base::Array<Engine::Geometries::Coordinate> &coordinates, std::vector<int> &ori_pts_index, double interval, int save_ori_pts)
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
        if (save_ori_pts)
        {
            newLine.Add(coordinates[0]);
            ori_pts_index.push_back(0);
            for (int i = 0; i < preLine.GetCount() - 1; i++)
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
                    point.x = startPoint.x + (endPoint.x - startPoint.x) * (dis) / length;
                    point.y = startPoint.y + (endPoint.y - startPoint.y) * (dis) / length;
                    point.z = startPoint.z + (endPoint.z - startPoint.z) * (dis) / length;
                    newLine.Add(point);
                }
                newLine.Add(endPoint);
                ori_pts_index.push_back(newLine.GetCount() - 1);
            }
        }
        else
        {
            newLine.Add(coordinates[0]);
            for (int i = 0; i < preLine.GetCount() - 1;)
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

                point.x = startPoint.x + (endPoint.x - startPoint.x) * (thresholld) / length;
                point.y = startPoint.y + (endPoint.y - startPoint.y) * (thresholld) / length;
                point.z = startPoint.z + (endPoint.z - startPoint.z) * (thresholld) / length;
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

    void processCrosswalk::getConcaveHull(const Engine::Base::Array<Engine::Geometries::Coordinate> &clusterpts, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull)
    {
        if (clusterpts.IsEmpty())
            return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < clusterpts.GetCount(); ++i)
        {
            cloud_1_ptr->push_back(pcl::PointXYZ(clusterpts[i].x, clusterpts[i].y, clusterpts[i].z));
        }

        // 求聚类点云的凹包
        pcl::ConcaveHull<pcl::PointXYZ> chull; // 创建多边形提取对象
        chull.setInputCloud(cloud_1_ptr);      // 设置输入点云为投影后点云cloud_projected
        chull.setAlpha(0.1);                   // 设置alpha值为0.1
        chull.reconstruct(*output);            // 重建提取创建凹多边形
        cloud_hull = output;
    }

    void processCrosswalk::get_cloud_hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_crosswalk_pcd, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull)
    {
        // 求聚类点云的凹包
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConcaveHull<pcl::PointXYZ> chull;  // 创建多边形提取对象
        chull.setInputCloud(bev_crosswalk_pcd); // 设置输入点云为投影后点云cloud_projected
        chull.setAlpha(0.1);                    // 设置alpha值为0.1
        chull.reconstruct(*output);             // 重建提取创建凹多边形
        cloud_hull = output;
    }

    void processCrosswalk::ReadObj(std::string filepath, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts)
    {
        std::ifstream infile(filepath);
        if (!infile.good())
            return; // 检查文件的存在与否
        std::ifstream ifs(filepath.c_str());
        if (!ifs.is_open())
            return;
        std::vector<Engine::Geometries::Coordinate> vecCoordinates;
        std::string ss;
        while (std::getline(ifs, ss))
        {
            char *sz = const_cast<char *>(ss.data());
            char *str = strtok(sz, " ");
            // vertex
            if (strcmp(str, "v") == 0)
            {
                Engine::Geometries::Coordinate c;
                str = strtok(nullptr, " "); // x
                c.x = atof(str);
                str = strtok(nullptr, " "); // y
                c.y = atof(str);
                str = strtok(nullptr, " "); // z
                c.z = atof(str);
                vecCoordinates.push_back(c);
            }
            else
            {
                str = strtok(nullptr, " "); // first index
                int start_index = atoi(str);
                int end_index = -1;
                while (str != nullptr)
                {
                    end_index = atoi(str);
                    str = strtok(nullptr, " ");
                }
                int size = end_index - start_index + 1;
                Engine::Base::Array<Engine::Geometries::Coordinate> vecLine(size);
                std::copy(vecCoordinates.begin() + start_index - 1, vecCoordinates.begin() + end_index,
                          vecLine.Begin());
                arrArrPts.Add(vecLine);
            }
        }
    }

    void processCrosswalk::bev_crosswalk_obj(const std::string bev_obj_floder, const std::string out_obj_floder, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &bev_crosswalks)
    {
        if (!boost::filesystem::exists(bev_obj_floder))
        {
            return;
        }

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> ori_bev_crosswalks;

        boost::filesystem::path path(bev_obj_floder);
        for (const auto &iter : boost::filesystem::directory_iterator(path))
        {
            if (boost::filesystem::is_directory(iter.path()))
            {
                std::string bev_crosswalk_obj = iter.path().string() + "/utm_crosswalks_bev.obj";
                if (boost::filesystem::exists(bev_crosswalk_obj))
                {
                    std::cout << "读取bev crosswalk:" << bev_crosswalk_obj << std::endl;
                    ReadObj(bev_crosswalk_obj, ori_bev_crosswalks);
                }
            }
        }
        if (!ori_bev_crosswalks.IsEmpty())
        {
            for (int i = 0; i < ori_bev_crosswalks.GetCount(); i++)
            {
                int n = ori_bev_crosswalks[i].GetCount();
                if (n < 3)
                {
                    continue;
                }

                // 将多边形进行闭合
                Engine::Geometries::Coordinate s = ori_bev_crosswalks[i][0];
                Engine::Geometries::Coordinate e = ori_bev_crosswalks[i][n - 1];
                if (s.DistanceXY(e) > 0.5)
                {
                    ori_bev_crosswalks[i].Add(s);
                }
                else
                {
                    ori_bev_crosswalks[i][n - 1] = ori_bev_crosswalks[i][0];
                }

                bev_crosswalks.Add(ori_bev_crosswalks[i]);
            }

            hdmap_build::CommonUtil::WriteToOBJ(bev_crosswalks, out_obj_floder, "bev_utm_crosswalk", true);
        }
    }

    // 计算平均邻域距离
    float processCrosswalk::computeAverageNeighborDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        float totalDist = 0.0f;
        #pragma omp parallel for reduction(+:totalDist)
        for (size_t i = 0; i < cloud->size(); ++i) {
            std::vector<int> indices(2);
            std::vector<float> sqrDists(2);
            if (kdtree.nearestKSearch(cloud->points[i], 2, indices, sqrDists) > 1) {
                totalDist += std::sqrt(sqrDists[1]);
            }
        }
        return totalDist / cloud->size();
    }

    MinAreaRect processCrosswalk::findMinAreaRect(const std::vector<Eigen::Vector2f>& points) {
        MinAreaRect rect;
        rect.area = std::numeric_limits<float>::max();
        
        // 遍历所有边作为候选方向
        for (size_t i = 0; i < points.size(); ++i) {
            size_t j = (i + 1) % points.size();
            Eigen::Vector2f edge = points[j] - points[i];
            Eigen::Vector2f axis = edge.normalized();
            Eigen::Vector2f perpendicular(-axis.y(), axis.x());

            // 计算投影极值
            float minProj = std::numeric_limits<float>::max();
            float maxProj = -std::numeric_limits<float>::max();
            float minPerp = std::numeric_limits<float>::max();
            float maxPerp = -std::numeric_limits<float>::max();
            for (const auto& p : points) {
                float proj = p.dot(axis);
                float perp = p.dot(perpendicular);
                minProj = std::min(minProj, proj);
                maxProj = std::max(maxProj, proj);
                minPerp = std::min(minPerp, perp);
                maxPerp = std::max(maxPerp, perp);
            }

            // 计算当前矩形面积
            float width = maxProj - minProj;
            float height = maxPerp - minPerp;
            float area = width * height;
            
            // 更新最小矩形
            if (area < rect.area) {
                rect.vertices[0] = axis * minProj + perpendicular * minPerp;
                rect.vertices[1] = axis * maxProj + perpendicular * minPerp;
                rect.vertices[2] = axis * maxProj + perpendicular * maxPerp;
                rect.vertices[3] = axis * minProj + perpendicular * maxPerp;
                rect.area = area;
            }
        }
        return rect;
    }

    // 可以用hdmap_build::CommonUtil::computePolygonArea
    float processCrosswalk::computePolygonArea(const std::vector<Eigen::Vector2f>& points) {
        float area = 0.0f;
        for (size_t i = 0; i < points.size(); ++i) {
            size_t j = (i + 1) % points.size();
            area += points[i].x() * points[j].y() - points[j].x() * points[i].y();
        }
        return std::abs(area) * 0.5f;
    }    
    
    MinAreaRect processCrosswalk::optimizeRectangle(
        const std::vector<Eigen::Vector2f>& hullPoints,
        float polyArea, 
        float lambda) 
    {
        // Step 1: 获取初始解
        MinAreaRect initial = findMinAreaRect(hullPoints);
        BoxParams params;

        // 初始化参数数组
        Eigen::Vector2f center = (initial.vertices[0] + initial.vertices[2]) * 0.5f;
        params.data[0] = center.x();  // x
        params.data[1] = center.y();  // y

        // 计算初始角度并规范化到 [-π/2, π/2]
        float dx = initial.vertices[1].x() - initial.vertices[0].x();
        float dy = initial.vertices[1].y() - initial.vertices[0].y();
        float initial_theta = atan2(dy, dx);
        initial_theta = fmod(initial_theta + M_PI, 2 * M_PI) - M_PI;  // 规范化到 [-π, π)
        if (initial_theta > M_PI / 2) {
            initial_theta -= M_PI;
        } else if (initial_theta < -M_PI / 2) {
            initial_theta += M_PI;
        }
        params.data[2] = initial_theta;

        // 初始化 a 和 b，并确保 a >= b
        float initial_a = (initial.vertices[1] - initial.vertices[0]).norm() * 0.5;
        float initial_b = (initial.vertices[2] - initial.vertices[1]).norm() * 0.5;
        if (initial_a < initial_b) {
            std::swap(initial_a, initial_b);
            params.data[2] += M_PI / 2.0;  // 调整角度
        }
        params.data[3] = initial_b;        // b
        params.data[4] = initial_a - initial_b;  // delta (a = b + delta)

        // Step 2: 构建优化问题
        ceres::Problem problem;
        
        // 添加残差块
        const int sampleStep = std::max(1, (int)hullPoints.size() / 20);
        for (size_t i = 0; i < hullPoints.size(); i += sampleStep) {
            Eigen::Vector2d pt = hullPoints[i].cast<double>();
            ceres::CostFunction* costFunc = 
                new ceres::AutoDiffCostFunction<BoxCostFunction, 1, 5>(
                    new BoxCostFunction(pt, polyArea, lambda));
            problem.AddResidualBlock(costFunc, new ceres::CauchyLoss(0.1), params.data);
        }

        // 设置参数边界
        problem.SetParameterLowerBound(params.data, 3, 0.01);  // b 的下界
        problem.SetParameterLowerBound(params.data, 4, 0.0);   // delta >= 0
        problem.SetParameterUpperBound(params.data, 3, 2.0 * params.data[3]);
        problem.SetParameterUpperBound(params.data, 4, 2.0 * params.data[4]);

        // Step 3: 执行优化
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 50;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Step 4: 生成结果矩形
        MinAreaRect result;
        const double x = params.data[0];
        const double y = params.data[1];
        double theta = params.data[2];
        const double b = params.data[3];
        const double delta = params.data[4];
        const double a = b + delta;

        // 确保 theta 在 [-π, π)
        theta = fmod(theta + M_PI, 2 * M_PI) - M_PI;

        // 计算旋转矩阵
        const double cosT = cos(theta);
        const double sinT = sin(theta);

        // 生成顶点
        result.vertices[0] = Eigen::Vector2f(
            x - a * cosT + b * sinT,
            y - a * sinT - b * cosT
        );
        result.vertices[1] = Eigen::Vector2f(
            x + a * cosT + b * sinT,
            y + a * sinT - b * cosT
        );
        result.vertices[2] = Eigen::Vector2f(
            x + a * cosT - b * sinT,
            y + a * sinT + b * cosT
        );
        result.vertices[3] = Eigen::Vector2f(
            x - a * cosT - b * sinT,
            y - a * sinT + b * cosT
        );
        result.area = 4.0 * a * b;

        return result;
    }

    void processCrosswalk::extractBoxOptimized(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud,
        Engine::Base::Array<Engine::Geometries::Coordinate>& plyCroods,
        double widthToler,
        double heightToler) 
    {
        // -------------------- 1. 输入检查 --------------------
        if (clusterCloud->size() < 4) return;

        std::vector<Eigen::Vector2f> points2D;
        for (const auto& p : *clusterCloud) {
            points2D.emplace_back(p.x, p.y);
        }
        
        // -------------------- 2. 优化矩形计算 --------------------
        const float polyArea = computePolygonArea(points2D);
        MinAreaRect rect = optimizeRectangle(points2D, polyArea, 0.15);

        // -------------------- 3. 尺寸过滤 --------------------
        const float width = (rect.vertices[1] - rect.vertices[0]).norm();
        const float height = (rect.vertices[2] - rect.vertices[1]).norm();
        if (width < widthToler || height < heightToler) return;

        // -------------------- 4. 生成3D矩形框 --------------------
        float sumZ = 0.0f;
        float avgZ = 0.0f;
        for (const auto& p : *clusterCloud) {
            sumZ += p.z;
        }  
        avgZ = sumZ / clusterCloud->points.size();      
        for (const auto& v : rect.vertices) {
            plyCroods.Add(Engine::Geometries::Coordinate(v.x(), v.y(), avgZ));
        }

        // -------------------- 5. 顶点排序 --------------------
        anticlockwisePolygon(plyCroods);
    }

    bool processCrosswalk::getSingleSegmentIntersections(
        const Engine::Base::Array<Engine::Geometries::Coordinate>& rect,
        const Engine::Geometries::Coordinate& c1,
        const Engine::Geometries::Coordinate& c2,
        std::vector<std::pair<Engine::Geometries::Coordinate, int>>& intersections) 
    {
        const double eps = 1e-5;
        intersections.clear();

        // 计算每条边的长度
        std::vector<double> edge_lengths(4);
        for (int edge = 0; edge < 4; ++edge) {
            const auto& a = rect[edge];
            const auto& b = rect[(edge+1)%4];
            edge_lengths[edge] = std::hypot(b.x - a.x, b.y - a.y);
        }
        
        // 找出最长的两条边
        double max_length = *std::max_element(edge_lengths.begin(), edge_lengths.end());
        std::vector<int> long_edges;
        for (int edge = 0; edge < 4; ++edge) {
            if (std::abs(edge_lengths[edge] - max_length) < eps) {
                long_edges.push_back(edge);
            }
        }

        if (long_edges.empty()) return false;

        pcl::ModelCoefficients line_query;
        line_query.values.resize(6);
        line_query.values[0] = static_cast<float>(c1.x);
        line_query.values[1] = static_cast<float>(c1.y);
        line_query.values[2] = 0.0f;
        line_query.values[3] = static_cast<float>(c2.x - c1.x);
        line_query.values[4] = static_cast<float>(c2.y - c1.y);
        line_query.values[5] = 0.0f;

        for (int edge : long_edges) {
            const auto& a = rect[edge];
            const auto& b = rect[(edge+1)%4];

            pcl::ModelCoefficients line_edge;
            line_edge.values.resize(6);
            line_edge.values[0] = static_cast<float>(a.x);
            line_edge.values[1] = static_cast<float>(a.y);
            line_edge.values[2] = 0.0f;
            line_edge.values[3] = static_cast<float>(b.x - a.x);
            line_edge.values[4] = static_cast<float>(b.y - a.y);
            line_edge.values[5] = 0.0f;

            Eigen::Vector4f intersection;
            const double sqr_eps = eps * eps;
            bool has_intersection = pcl::lineWithLineIntersection(
                line_edge, line_query, intersection, sqr_eps);

            if (!has_intersection) continue;

            Engine::Geometries::Coordinate inter(
                intersection[0], intersection[1], 0.0);

            // 检查交点是否在矩形边的参数范围内 (0 <= t <= 1)
            double t = 0.0;
            const double dx_edge = b.x - a.x;
            const double dy_edge = b.y - a.y;
            if (std::abs(dx_edge) > eps) {
                t = (inter.x - a.x) / dx_edge;
            } else if (std::abs(dy_edge) > eps) {
                t = (inter.y - a.y) / dy_edge;
            } else {
                continue;
            }

            // 扩展参数范围容忍浮点误差
            const bool on_edge = (t >= -eps) && (t <= 1.0 + eps);

            // 检查交点是否在查询线段范围内 (0 <= s <= 1)
            double s = 0.0;
            const double dx_query = c2.x - c1.x;
            const double dy_query = c2.y - c1.y;
            if (std::abs(dx_query) > eps) {
                s = (inter.x - c1.x) / dx_query;
            } else if (std::abs(dy_query) > eps) {
                s = (inter.y - c1.y) / dy_query;
            } else {
                continue;
            }

            const bool on_segment = (s >= -eps) && (s <= 1.0 + eps);

            if (on_edge && on_segment) {
                bool is_center = (t >= 0.25) && (t <= 0.75);
                if (!is_center) {
                    continue;
                }

                if (t < 0.0) inter = a;
                if (t > 1.0) inter = b;
                
                intersections.emplace_back(inter, edge);
            }
        }

        // 去重处理
        auto comparator = [eps](const auto& p1, const auto& p2) {
            return std::hypot(p1.first.x - p2.first.x, 
                            p1.first.y - p2.first.y) < eps;
        };
        auto last = std::unique(intersections.begin(), intersections.end(), comparator);
        intersections.erase(last, intersections.end());

        return !intersections.empty();
    }

    void processCrosswalk::collectIntersections(
        const Engine::Base::Array<Engine::Geometries::Coordinate>& rect,
        const pcl::PointCloud<PointElement>::Ptr& line_pcd,
        std::vector<std::pair<Engine::Geometries::Coordinate, int>>& intersections) {
        for (size_t i = 0; i < line_pcd->size() - 1; ++i) {
            auto& p1 = line_pcd->at(i);
            auto& p2 = line_pcd->at(i + 1);

            Engine::Geometries::Coordinate c1(p1.x, p1.y, 0);
            Engine::Geometries::Coordinate c2(p2.x, p2.y, 0);

            std::vector<std::pair<Engine::Geometries::Coordinate, int>> local_intersections;
            bool has_intersection = getSingleSegmentIntersections(rect, c1, c2, local_intersections);
            
            if (has_intersection) {
                intersections.insert(intersections.end(), local_intersections.begin(), local_intersections.end());
            }
        }
    }

    bool processCrosswalk::validateOppositeEdges(
        const std::vector<std::pair<Engine::Geometries::Coordinate, int>>& intersections,
        std::pair<Engine::Geometries::Coordinate, Engine::Geometries::Coordinate>& result) {
        const auto& p1 = intersections[0];
        const auto& p2 = intersections[1];
        const int e1 = p1.second;
        const int e2 = p2.second;
        
        const bool is_vertical_pair = (e1 == 0 && e2 == 2) || (e1 == 2 && e2 == 0);
        const bool is_horizontal_pair = (e1 == 1 && e2 == 3) || (e1 == 3 && e2 == 1);
        
        if (!is_vertical_pair && !is_horizontal_pair) return false;

        result.first = p1.first;
        result.second = p2.first;
    
        return true;
    }

    void processCrosswalk::computeEdgeUV(const Engine::Geometries::Coordinate& A, const Engine::Geometries::Coordinate& AB, 
            const Engine::Geometries::Coordinate& AD, const Engine::Geometries::Coordinate& p, double& u, double& v) {
        const double AB_len_sq = AB.x*AB.x + AB.y*AB.y;
        const double AD_len_sq = AD.x*AD.x + AD.y*AD.y;
        
        // 参数化到AB轴（u坐标）
        u = ((p.x - A.x)*AB.x + (p.y - A.y)*AB.y) / AB_len_sq;
        
        // 参数化到AD轴（v坐标）
        v = ((p.x - A.x)*AD.x + (p.y - A.y)*AD.y) / AD_len_sq;

        // 强制对齐到最近的边（基于原始几何约束）
        constexpr double edge_threshold = 1e-5;
        if (u < edge_threshold) u = 0.0;
        else if (u > 1.0 - edge_threshold) u = 1.0;
        if (v < edge_threshold) v = 0.0;
        else if (v > 1.0 - edge_threshold) v = 1.0;
    }

    std::vector<Array<Engine::Geometries::Coordinate>> processCrosswalk::splitWithGap(const Array<Engine::Geometries::Coordinate>& originalRect,
        const std::vector<std::pair<Engine::Geometries::Coordinate, Engine::Geometries::Coordinate>>& intersectionPairs) 
    {
        constexpr double gap_cm = 20.0; 
        constexpr double epsilon = 1e-6;

        if (originalRect.GetCount() != 4) return {originalRect};
        const auto& A = originalRect[0], B = originalRect[1], C = originalRect[2], D = originalRect[3];
        
        // 计算基底向量
        const Engine::Geometries::Coordinate AB = B - A;
        const Engine::Geometries::Coordinate AD = D - A;
        const double AB_len = std::hypot(AB.x, AB.y);
        const double AD_len = std::hypot(AD.x, AD.y);
        
        // 转换为参数化空间的间隙
        const double gap_u = gap_cm / 100.0 / AB_len;
        const double gap_v = gap_cm / 100.0 / AD_len;

        // 确定点所在的边
        auto determineEdge = [epsilon](double u, double v) -> std::string {
            if (std::abs(v) < epsilon && u >= -epsilon && u <= 1 + epsilon) return "AB";
            if (std::abs(u - 1) < epsilon && v >= -epsilon && v <= 1 + epsilon) return "BC";
            if (std::abs(v - 1) < epsilon && u >= -epsilon && u <= 1 + epsilon) return "CD";
            if (std::abs(u) < epsilon && v >= -epsilon && v <= 1 + epsilon) return "AD";
            return "Unknown";
        };

        // 收集切割线参数
        std::vector<double> u_splits, v_splits;
        for (const auto& [p1, p2] : intersectionPairs) {
            double u1, v1, u2, v2;
            computeEdgeUV(A, AB, AD, p1, u1, v1);
            computeEdgeUV(A, AB, AD, p2, u2, v2);

            std::string edge1 = determineEdge(u1, v1);
            std::string edge2 = determineEdge(u2, v2);

            // 根据边对确定分割方向
            if ((edge1 == "AB" && edge2 == "CD") || (edge1 == "CD" && edge2 == "AB")) {
                u_splits.push_back((u1 + u2) * 0.5);
            } else if ((edge1 == "AD" && edge2 == "BC") || (edge1 == "BC" && edge2 == "AD")) {
                v_splits.push_back((v1 + v2) * 0.5);
            } else {
                if (std::abs(u1 - u2) < epsilon) {
                    u_splits.push_back((u1 + u2) * 0.5);
                } else {
                    v_splits.push_back((v1 + v2) * 0.5);
                }
            }
        }

        // 切割参数处理模板
        auto processSplits = [](auto& splits, double gap) {
            std::sort(splits.begin(), splits.end());
            splits.erase(std::unique(splits.begin(), splits.end()), splits.end());
            
            if (splits.empty()) return std::vector<std::pair<double, double>>{{0.0, 1.0}};
            
            std::vector<std::pair<double, double>> intervals;
            double prev = 0.0;
            for (double split : splits) {
                intervals.emplace_back(prev + gap/2, split - gap/2);
                prev = split + gap/2;
            }
            intervals.emplace_back(prev + gap/2, 1.0 - gap/2);
            
            intervals.erase(
                std::remove_if(intervals.begin(), intervals.end(),
                    [](const auto& p) { return p.first >= p.second; }),
                intervals.end()
            );
            return intervals;
        };

        // 生成带间隙的切割区间
        const auto u_intervals = processSplits(u_splits, gap_u);
        const auto v_intervals = processSplits(v_splits, gap_v);

        // 生成所有子矩形
        std::vector<Array<Engine::Geometries::Coordinate>> result;
        for (const auto& [u_start, u_end] : u_intervals) {
            for (const auto& [v_start, v_end] : v_intervals) {
                const Engine::Geometries::Coordinate p1 = A + AB*u_start + AD*v_start;
                const Engine::Geometries::Coordinate p2 = A + AB*u_end   + AD*v_start;
                const Engine::Geometries::Coordinate p3 = A + AB*u_end   + AD*v_end;
                const Engine::Geometries::Coordinate p4 = A + AB*u_start + AD*v_end;
                
                Engine::Base::Array<Engine::Geometries::Coordinate> array;
                array.Add(p1);
                array.Add(p2);
                array.Add(p3);
                array.Add(p4);

                result.emplace_back(array);
            }
        }

        return result.empty() ? std::vector<Array<Engine::Geometries::Coordinate>>{originalRect} : result;
    }
 
    void processCrosswalk::splitRect(
        const std::string dataDir, 
        const std::string refFile,
        const Engine::Base::Array<Engine::Geometries::Coordinate>& inputRect,
        std::vector<Engine::Base::Array<Engine::Geometries::Coordinate>>& outputRects,
        double widthToler, double heightToler) {

        // 读取道路边界线
        std::ifstream ifs_parseInfo(refFile);
        if(!ifs_parseInfo.is_open()){
            ifs_parseInfo.close();
            return;
        }

        nlohmann::json parseInfo = nlohmann::json::parse(ifs_parseInfo);
        std::vector<std::string> links = parseInfo["links"];

        std::vector<std::string> files;
        #pragma omp parallel
        {
            std::vector<std::string> private_files;
            #pragma omp for nowait
            for (int i = 0; i < links.size(); ++i) {
                std::string laneBoundaryFromSeg = dataDir+"/" + links[i] + "/roadBoundaryFromSeg";
                std::vector<std::string> file_names;
                Utils::getFiles(laneBoundaryFromSeg, file_names, [](std::string str)->bool{
                    return boost::algorithm::ends_with(str, "_trjCloud.pcd");
                });
                for (auto& filename : file_names) {
                    private_files.emplace_back(laneBoundaryFromSeg + "/" + filename);
                }
            }
            #pragma omp critical
            files.insert(files.end(), private_files.begin(), private_files.end());
        }

        // 计算中心和长边长度
        double width = inputRect[0].DistanceXY(inputRect[1]);
        double height = inputRect[0].DistanceXY(inputRect[3]);
        double longEdgeLength = std::max(width, height);
        Engine::Geometries::Coordinate center(
            (inputRect[0].x + inputRect[1].x + inputRect[2].x + inputRect[3].x) / 4.0,
            (inputRect[0].y + inputRect[1].y + inputRect[2].y + inputRect[3].y) / 4.0,
            0
        );

        // 过滤道路边界
        std::vector<pcl::PointCloud<PointElement>::Ptr> relevantClouds;
        #pragma omp parallel
        {
            std::vector<pcl::PointCloud<PointElement>::Ptr> local_clouds;
            #pragma omp for nowait
            for (size_t i = 0; i < files.size(); ++i) {
                pcl::PointCloud<PointElement>::Ptr cloud(new pcl::PointCloud<PointElement>);
                if (pcl::io::loadPCDFile(files[i], *cloud) != -1) {
                    int valid = 0;
                    for (const auto& point : *cloud) {
                        double dx = point.x - center.x;
                        double dy = point.y - center.y;
                        if (std::hypot(dx, dy) <= longEdgeLength/2) {
                            if (++valid >= 2) {
                                local_clouds.push_back(cloud);
                                break;
                            }
                        }
                    }
                }
            }
            #pragma omp critical
            relevantClouds.insert(relevantClouds.end(), local_clouds.begin(), local_clouds.end());
        }

        // 收集所有有效交点
        std::vector<std::pair<Engine::Geometries::Coordinate, Engine::Geometries::Coordinate>> allIntersectionPairs;
        #pragma omp parallel
        {
            std::vector<std::pair<Engine::Geometries::Coordinate, Engine::Geometries::Coordinate>> local_pairs;
            #pragma omp for nowait
            for (size_t i = 0; i < relevantClouds.size(); ++i) {
                std::vector<std::pair<Engine::Geometries::Coordinate, int>> intersections;
                collectIntersections(inputRect, relevantClouds[i], intersections);
                
                if (intersections.size() >= 2) {
                    std::pair<Engine::Geometries::Coordinate, Engine::Geometries::Coordinate> validPair;
                    if (validateOppositeEdges(intersections, validPair)) {
                        local_pairs.push_back(validPair);
                    }
                }
            }
            #pragma omp critical
            allIntersectionPairs.insert(allIntersectionPairs.end(), local_pairs.begin(), local_pairs.end());
        }

        // 切割并过滤矩形
        auto splitRects = splitWithGap(inputRect, allIntersectionPairs);
        #pragma omp parallel
        {
            std::vector<Engine::Base::Array<Engine::Geometries::Coordinate>> local_rects;
            #pragma omp for nowait
            for (size_t i = 0; i < splitRects.size(); ++i) {
                float w = splitRects[i][0].DistanceXY(splitRects[i][1]);
                float h = splitRects[i][0].DistanceXY(splitRects[i][3]);
                if (w > widthToler && h > heightToler) {
                    local_rects.push_back(splitRects[i]);
                }
            }
            #pragma omp critical
            outputRects.insert(outputRects.end(), local_rects.begin(), local_rects.end());
        }
    }

    void processCrosswalk::afterProcess(const std::string middle_json_path, const std::string& dataDir, const std::string& outputDir, const std::string& refFile,
                    Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>>& crosswalks) {
        
        // 读取路口中心点
        std::string file_name = middle_json_path;
        std::ifstream json_file(file_name);
        if (!json_file.is_open()) {
            std::cout << "无法打开json文件: " << file_name << std::endl;
            return;
        }

        nlohmann::json root = nlohmann::json::parse(json_file);
        if (!root["middle"].contains("site_center") || !root["middle"]["site_center"].is_array()) {
            std::cout << "[stopline]site_center数据结构不正确" << std::endl;
            return;
        }
        const auto &site_data = root["middle"]["site_center"]; 
        pcl::PointXYZ site_center(site_data[0], site_data[1], 0.0f);     

        // 读取车道线
        std::ifstream ifs_parseInfo(refFile);
        if (!ifs_parseInfo.is_open()) {
            return;
        }

        nlohmann::json parseInfo = nlohmann::json::parse(ifs_parseInfo);
        std::vector<std::string> links = parseInfo["links"];

        std::vector<std::string> files;
        pcl::PointCloud<pcl::PointXYZI>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZI>);

        #pragma omp parallel for
        for (const auto& link : links) {
            std::string laneBoundaryFromSeg = dataDir+"/" + link + "/laneBoundaryFromSeg";
            std::vector<std::string> file_names;
            Utils::getFiles(laneBoundaryFromSeg, file_names, [](std::string str)->bool{
                return boost::algorithm::ends_with(str, "_trjCloud.pcd");
            });

            #pragma omp critical
            for (auto filename : file_names) {
                std::string file_path = laneBoundaryFromSeg + "/" + filename;
                files.push_back(file_path);
            }
        }

        int line_id = 0;
        for (const auto& file : files) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile(file, *cloud) == -1) continue;

            for (const auto& point : *cloud) {
                refLaneBoundaryCloud->push_back(pcl::PointXYZI(point.x, point.y, point.z, line_id));
            }
            line_id++;
        }

        std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> lane_clouds_cache;
        for (const auto& pt : *refLaneBoundaryCloud) {
            auto& cloud = lane_clouds_cache.emplace(
                static_cast<int>(pt.intensity), std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()
            ).first->second;
            cloud->emplace_back(pt.x, pt.y, 0.0f);
        }

        // 读取停止线
        std::string stoplineFile = outputDir + "/object_stopline.pcd";
        pcl::PointCloud<PointElement>::Ptr stopline_pcd(new pcl::PointCloud<PointElement>);
        if (pcl::io::loadPCDFile(stoplineFile, *stopline_pcd) == -1) {
            std::cout << "Failed to read PCD: " << stoplineFile << std::endl;
            return;
        }

        // 预处理停止线
        Engine::Base::Array<std::pair<PointElement, PointElement>> stoplines;
        for (size_t i = 0; i < stopline_pcd->size(); i += 2) {
            if (i + 1 >= stopline_pcd->size()) break;
            stoplines.Add(std::make_pair(stopline_pcd->points[i], stopline_pcd->points[i + 1]));
        }

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> correctedCrosswalks = crosswalks;
        // 对每一个crosswalk，找到最近的停止线并进行校正
        for (size_t i = 0; i < correctedCrosswalks.GetCount(); i ++) {
            auto& crosswalk = correctedCrosswalks[i];
            std::cout << "raw crosswalk[" << i << "]: ";
            for (size_t j = 0; j < crosswalk.GetCount(); ++j) {
                const auto& coord = crosswalk[j];
                std::cout << "(" << coord.x << ", " << coord.y << ") ";
            }
            std::cout << std::endl;

            // 获取附近车道线的主方向
            auto nearby_lines = findLinesWithDynamicThreshold(crosswalk, site_center, lane_clouds_cache);
            if (nearby_lines.line_ids.empty()) continue;

            auto main_direction = filterLinesByDirection(std::move(nearby_lines), 30.0f).main_direction;
            if (main_direction.norm() < 1e-6) continue;

            // 找到最近的且方向与车道线主方向近似垂直的停止线
            std::pair<PointElement, PointElement> nearestStopline;
            double minDistance = std::numeric_limits<double>::max();
            bool found = false;

            for (size_t j = 0; j < stoplines.GetCount(); ++j) {
                const auto& stopline = stoplines[j];
                Eigen::Vector3f stoplineDir = calculateDirection(stopline);
                // 计算crosswalk四个角点与停止线两个端点之间最近的距离
                double distance = calculateMinCornerDistanceToStopline(crosswalk, stopline);
                if (distance > 10.0) continue;
                std::cout << "distance[" << i << "]" << "[" << j << "] " << distance << std::endl;


                double angleDiff = std::abs(std::atan2(stoplineDir.y(), stoplineDir.x()) - std::atan2(main_direction.y(), main_direction.x()));
                angleDiff = std::min(angleDiff, 2 * M_PI - angleDiff);
                std::cout << "angle[" << i << "]" << "[" << j << "]" << angleDiff << " , " << std::abs(angleDiff - M_PI / 2) << std::endl;
                
                if (std::abs(angleDiff - M_PI / 2) > M_PI / 18) continue;

                if (distance < minDistance) {
                    minDistance = distance;
                    nearestStopline = stopline;
                    found = true;
                }
            }

            if (!found) continue;

            std::cout << "nearest stopline: (" << nearestStopline.first.x << ", " << nearestStopline.first.y << ") - (" << nearestStopline.second.x << ", " << nearestStopline.second.y << ")" << std::endl;

            // 校正人行横道
            correctCrosswalk(crosswalk, nearestStopline);
            std::cout << "after corrected: ";
            for (size_t j = 0; j < crosswalk.GetCount(); ++j) {
                const auto& coord = crosswalk[j];
                std::cout << "(" << coord.x << ", " << coord.y << ") ";
            }
            std::cout << std::endl;

            // 调整人行横道与停止线的距离
            adjustCrosswalkDistance(crosswalk, nearestStopline);
            std::cout << "after adjusted: ";
            for (size_t j = 0; j < crosswalk.GetCount(); ++j) {
                const auto& coord = crosswalk[j];
                std::cout << "(" << coord.x << ", " << coord.y << ") ";
            }
            std::cout << std::endl;
        }
        crosswalks = correctedCrosswalks;
    }

    processCrosswalk::NearbyLinesResult processCrosswalk::findNearbyLinesWithMinPoints(
        const pcl::PointXYZ& target_point,
        const pcl::PointXYZ& site_center,
        double threshold,
        const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache,
        int minPts)
    {
        NearbyLinesResult result;
        const double squared_threshold = threshold * threshold;
        const auto& tx = target_point.x;
        const auto& ty = target_point.y;
        const auto& tz = target_point.z;

        const double dtx = tx - site_center.x;
        const double dty = ty - site_center.y;
        const double dtz = tz - site_center.z;
        double distance_target_to_site_center = std::sqrt(dtx * dtx + dty * dty + dtz * dtz); 
        if (distance_target_to_site_center > 60) 
            distance_target_to_site_center = distance_target_to_site_center -15;

        // std::vector<std::pair<int, size_t>> line_sizes;
        for (const auto& [line_id, cloud] : lane_clouds_cache) {
            if (!cloud || cloud->empty()) continue;

            auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            for (const auto& pt : *cloud) {
                const double dx = pt.x - tx;
                const double dy = pt.y - ty;
                const double dz = pt.z - tz;
                const double distance_to_target_squared = dx * dx + dy * dy + dz * dz; 

                if (distance_to_target_squared <= squared_threshold) {
                    const double dsx = pt.x - site_center.x;
                    const double dsy = pt.y - site_center.y;
                    const double dsz = pt.z - site_center.z;
                    const double distance_to_site_center_squared = dsx * dsx + dsy * dsy + dsz * dsz;            
                    if (distance_to_site_center_squared > distance_target_to_site_center * distance_target_to_site_center) {
                        filtered_cloud->push_back(pt);
                    }
                }
            }

            if (filtered_cloud->size() >= 2) {
                result.line_ids.push_back(line_id);
                result.line_points[line_id] = filtered_cloud;
                // line_sizes.emplace_back(line_id, filtered_cloud->size());
            }
        }

        // // 如果找到的线条数量大于3，保留点数最多的前三条
        // if (line_sizes.size() > 3) {
        //     std::sort(line_sizes.begin(), line_sizes.end(), [](const auto& a, const auto& b) {
        //         return a.second > b.second;
        //     });

        //     // 只保留前三条
        //     result.line_ids = {line_sizes[0].first, line_sizes[1].first, line_sizes[2].first};
        //     std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> new_line_points;
        //     for (const auto& [line_id, _] : line_sizes) {
        //         if (new_line_points.size() < 3) {
        //             new_line_points[line_id] = result.line_points[line_id];
        //         }
        //     }
        //     result.line_points = std::move(new_line_points);
        // }

        return result;
    }

    processCrosswalk::NearbyLinesResult processCrosswalk::findLinesWithDynamicThreshold(
        const Engine::Base::Array<Engine::Geometries::Coordinate> crosswalk,
        const pcl::PointXYZ& site_center,
        const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache)
    {
        Engine::Geometries::Coordinate crosswalk_center = calculateCenter(crosswalk);
        pcl::PointXYZ target_point(crosswalk_center.x, crosswalk_center.y, 0.0f);

        double maxEdge = std::numeric_limits<double>::lowest();
        for (size_t i = 0; i < 4; ++i) {
            size_t next = (i + 1) % 4;
            double distance = std::sqrt(std::pow(crosswalk[next].x - crosswalk[i].x,2) + std::pow(crosswalk[next].y - crosswalk[i].y,2));   
            if (distance > maxEdge) {
                maxEdge = distance;
            }
        }

        double current_threshold = maxEdge;
        double max_threshold = 2*maxEdge;

        while (current_threshold <= max_threshold) {
            auto result = findNearbyLinesWithMinPoints(target_point, site_center, current_threshold, lane_clouds_cache, 2);
            if (result.line_ids.size() >= 3) return result;
            current_threshold *= 1.5;
        }
        return findNearbyLinesWithMinPoints(target_point, site_center, max_threshold, lane_clouds_cache, 2);
    }

    processCrosswalk::DirectionFilterResult processCrosswalk::filterLinesByDirection(
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

    // 计算中心点
    Engine::Geometries::Coordinate processCrosswalk::calculateCenter(const Engine::Base::Array<Engine::Geometries::Coordinate>& object) {
        double x = 0.0, y = 0.0, z = 0.0;
        for (size_t i = 0; i < object.GetCount(); i++){
            x += object[i].x;
            y += object[i].y;
            z += object[i].z;
        }
        x /= object.GetCount();
        y /= object.GetCount();
        z /= object.GetCount();

        return Engine::Geometries::Coordinate{x, y, z};
    }

    // 计算方向向量
    Eigen::Vector3f processCrosswalk::calculateDirection(const std::pair<PointElement, PointElement>& stopline) {
        PointElement start = stopline.first;
        PointElement end = stopline.second;

        // 计算方向向量
        Eigen::Vector3f direction;
        direction.x() = end.x - start.x;
        direction.y() = end.y - start.y;
        direction.z() = 0.0f;

        // 归一化方向向量
        double length = std::sqrt(direction.x() * direction.x()  + direction.y() * direction.y());
        if (length > 0) {
            direction.x() /= length;
            direction.y() /= length;
        }

        return direction;
    }

    // 计算角点到停止线的欧氏距离
    double processCrosswalk::calculateMinCornerDistanceToStopline(const Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk, const std::pair<PointElement, PointElement>& stopline) {
        const auto& lineStart = stopline.first;
        const auto& lineEnd = stopline.second;
        double minDistance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < crosswalk.GetCount(); ++i) {
            auto point = crosswalk[i];
            double distanceToStart = std::sqrt(std::pow(point.x - lineStart.x, 2) + std::pow(point.y - lineStart.y, 2));
            double distanceToEnd = std::sqrt(std::pow(point.x - lineEnd.x, 2) + std::pow(point.y - lineEnd.y, 2));
            
            if (distanceToStart < minDistance) {
                minDistance = distanceToStart;
            }

            if (distanceToEnd < minDistance) {
                minDistance = distanceToEnd;
            }
        }

        return minDistance;
    }

    // 计算点到线的垂直距离
    double processCrosswalk::calculateDistanceToStopline(
        const Engine::Geometries::Coordinate& point,
        const std::pair<PointElement, PointElement>& stopline) {
        
        double x0 = point.x, y0 = point.y;
        double x1 = stopline.first.x, y1 = stopline.first.y;
        double x2 = stopline.second.x, y2 = stopline.second.y;

        double abx = x2 - x1;
        double aby = y2 - y1;

        double apx = x0 - x1;
        double apy = y0 - y1;

        double cross = apx * aby - apy * abx;

        double ab_length = std::hypot(abx, aby);

        if (ab_length == 0.0) {
            return std::hypot(apx, apy);
        }

        // 计算垂直距离
        // double perpendicular_distance = std::abs(cross) / ab_length;
        double perpendicular_distance = cross / ab_length;
        return perpendicular_distance;
    }

    // 找距离停止线最近的边
    int processCrosswalk::find_closest_edge(const Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk, const std::pair<PointElement, PointElement>& stopline) {
        size_t nearestCornerPoint = 0;
        double minDistance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < 4; ++i) {
            double distance = calculateDistanceToStopline(crosswalk[i], stopline);            
            // if (distance < minDistance) {
            //     minDistance = distance;
            //     nearestCornerPoint = i;
            // }
            if (std::abs(distance) < minDistance) {
                minDistance = std::abs(distance);
                nearestCornerPoint = i;
            }            
        }

        size_t nextCorner = (nearestCornerPoint + 1) % 4;
        size_t prevCorner = (nearestCornerPoint + 3) % 4;

        double prevEdgeAvgDist = (
            std::abs(calculateDistanceToStopline(crosswalk[prevCorner], stopline)) +
            std::abs(calculateDistanceToStopline(crosswalk[nearestCornerPoint], stopline))
        ) / 2.0;

        double nextEdgeAvgDist = (
            std::abs(calculateDistanceToStopline(crosswalk[nearestCornerPoint], stopline)) +
            std::abs(calculateDistanceToStopline(crosswalk[nextCorner], stopline))
        ) / 2.0;

        size_t nearestEdgeStart = (prevEdgeAvgDist < nextEdgeAvgDist) ? prevCorner : nearestCornerPoint;
    
        return nearestEdgeStart;
    }

    // 对点进行坐标转换
    void processCrosswalk::rotatePoint(Engine::Geometries::Coordinate& point, const Engine::Geometries::Coordinate& center, double angle) {
        double x = point.x - center.x;
        double y = point.y - center.y;
        point.x = x * std::cos(angle) - y * std::sin(angle) + center.x;
        point.y = x * std::sin(angle) + y * std::cos(angle) + center.y;
    }

    void processCrosswalk::correctCrosswalk(
        Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk,
        const std::pair<PointElement, PointElement>& stopline) {
        
        if (crosswalk.GetCount() != 4) {
            std::cout << "Crosswalk is not a rectangle. Skipping correction.\n";
            return;
        }

        double stoplineDx = stopline.second.x - stopline.first.x;
        double stoplineDy = stopline.second.y - stopline.first.y;
        double stoplineAngle = std::atan2(stoplineDy, stoplineDx);

        size_t nearestEdgeStart = find_closest_edge(crosswalk, stopline);
        std::cout << "Nearest edge: " << nearestEdgeStart << "\n";

        double nearestEdgeDx = crosswalk[(nearestEdgeStart + 1) % 4].x - crosswalk[nearestEdgeStart].x;
        double nearestEdgeDy = crosswalk[(nearestEdgeStart + 1) % 4].y - crosswalk[nearestEdgeStart].y;
        double nearestEdgeAngle = std::atan2(nearestEdgeDy, nearestEdgeDx);

        // 计算旋转角度
        double rotateAngle = stoplineAngle - nearestEdgeAngle;   
        if (rotateAngle > M_PI) rotateAngle -= 2 * M_PI;
        if (rotateAngle < -M_PI) rotateAngle += 2 * M_PI;

        // 如果角度差接近 180°，选择反向对齐（旋转 angle - 180°）
        if (std::abs(rotateAngle) > M_PI / 2) {
            rotateAngle = (rotateAngle > 0) ? rotateAngle - M_PI : rotateAngle + M_PI;
        }

        std::cout << "Stopline angle: " << stoplineAngle * 180 / M_PI << " degrees\n";
        std::cout << "Nearest edge angle: " << nearestEdgeAngle * 180 / M_PI << " degrees\n";
        std::cout << "Rotate angle: " << rotateAngle * 180 / M_PI << " degrees\n";

        // 计算矩形的中心点
        Engine::Geometries::Coordinate center = calculateCenter(crosswalk);

        // 旋转整个矩形，使最近的边与停止线平行
        for (size_t i = 0; i < crosswalk.GetCount(); ++i) {
            rotatePoint(crosswalk[i], center, rotateAngle);
        }
    } 

    void processCrosswalk::moveCrosswalk(Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk, const std::pair<PointElement, PointElement>& stopline, double offset) {
        double dx = stopline.second.x - stopline.first.x;
        double dy = stopline.second.y - stopline.first.y;
        double length = std::sqrt(dx * dx + dy * dy);

        // 计算逆时针旋转90度的垂直向量
        double perpDx = dy / length;
        double perpDy = -dx / length;

        for (size_t i = 0; i < crosswalk.GetCount(); i++){
            crosswalk[i].x += perpDx * offset;
            crosswalk[i].y += perpDy * offset;
        }
    }

    // corner case：人行横道中心在停止线上方，最近邻边在停止线下方，按照当前逻辑，是往下移动的。
    void processCrosswalk::adjustCrosswalkDistance(Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk, const std::pair<PointElement, PointElement>& stopline) {
        size_t nearestEdgeStart = find_closest_edge(crosswalk, stopline);
        double nearestEdgeCenterX = (crosswalk[(nearestEdgeStart + 1) % 4].x + crosswalk[nearestEdgeStart].x) / 2;
        double nearestEdgeCenterY = (crosswalk[(nearestEdgeStart + 1) % 4].y + crosswalk[nearestEdgeStart].y) / 2;
        Engine::Geometries::Coordinate nearestEdgeCenter{nearestEdgeCenterX, nearestEdgeCenterY, 0.0};
        double distance = calculateDistanceToStopline(nearestEdgeCenter, stopline);
 
        double nearestOppoEdgeCenterX = (crosswalk[(nearestEdgeStart + 2) % 4].x + crosswalk[(nearestEdgeStart + 3) % 4].x) / 2;
        double nearestOppoEdgeCenterY = (crosswalk[(nearestEdgeStart + 2) % 4].y + crosswalk[(nearestEdgeStart + 3) % 4].y) / 2;
        Engine::Geometries::Coordinate nearestOppoEdgeCenter{nearestOppoEdgeCenterX, nearestOppoEdgeCenterY, 0.0};
        double oppoDistance = calculateDistanceToStopline(nearestOppoEdgeCenter, stopline);        
        std::cout << "nearestEdgeStart: " << nearestEdgeStart << ", Distance to stopline: " << distance << ", Oppo Distance to stopline: " << oppoDistance << "\n";

        if (distance < 0.0) {
            if (oppoDistance > 0.0) {
                moveCrosswalk(crosswalk, stopline, (1.0 - distance));          
            } else {
                if (std::abs(distance) < 1.0) {
                    moveCrosswalk(crosswalk, stopline, -(1.0 + distance));
                } else if (std::abs(distance) > 3.0) {
                    moveCrosswalk(crosswalk, stopline, -(3.0 + distance));
                }
            }
        } else {
            if (oppoDistance < 0.0) {
                moveCrosswalk(crosswalk, stopline, (1.0 - oppoDistance));  
            } else {
                if (distance < 1.0) {
                    moveCrosswalk(crosswalk, stopline, 1.0 - distance);
                } else if (distance > 3.0) {
                    moveCrosswalk(crosswalk, stopline, 3.0 - distance);
                } 
            }       
        }
    }


}