#include "json.hpp"
#include "pclPtType.h"
#include "pclFilter.h"
#include "Utils.h"
#include "cluster.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "skeleton_cluster.hpp"

#include <chrono>
#include <cmath>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/boundary.h>             //边界提取
// #include <pcl/sample_consensus/method_types.h> //随机参数估计方法
// #include <pcl/sample_consensus/model_types.h>  //模型定义
// #include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
// #include <pcl/filters/extract_indices.h>       //索引提取
// #include <pcl/segmentation/region_growing.h>

// processSiteBoundary
// 处理路口附近的边界线
//  参数1：输入要合并的文件夹地址 参数2：输出文件全路径
int main(int argc, char *argv[])
{
    std::string pcd_file = argv[1];     // 全局点云
    std::string middlejson = argv[2];  // 建模输出的输入任务json文件，主要为了获取link信息
    std::string output_dir = argv[3];  // 输出文件路径

    // std::string dataDir = "/home/lenovo/data/lidar_mapping_ws/55267/model_pcd/global_cloud.pcd";
    // std::string middlejson = "/home/lenovo/data/lidar_mapping_ws/55267/fsd_mapbuild_out/fsd_mapbuild_task.json";
    // std::string outpcdPath = "/home/lenovo/data/lidar_mapping_ws/55267/fsd_mapbuild_out/laneboundary.pcd";

    nlohmann::json parseJosn;
    std::ifstream ifs_json(middlejson);
    if (!ifs_json.is_open())
    {
        ifs_json.close();
        std::cerr << "参数文件打开失败！！！" << std::endl;
        return 1;
    }
    parseJosn << ifs_json;

    std::string custom_site_poly_50m = parseJosn["custom_site_poly_50m"];
    if (custom_site_poly_50m == "")
    {
        std::cerr << "没有路口中心点，不处理！！！" << std::endl;
        return 1;
    }
    // std::cout << "custom_site_poly_50m " << custom_site_poly_50m << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr range_poly(new pcl::PointCloud<pcl::PointXYZ>);
    Utils::GisPolygon new_line;
    boost::geometry::read_wkt(custom_site_poly_50m, new_line);
    for (const auto &pt : new_line.outer())
    {
        double x = pt.get<0>();
        double y = pt.get<1>();
        double z = pt.get<2>();
        range_poly->push_back(pcl::PointXYZ(x, y, z));
    }
    pcl::io::savePCDFileBinary(output_dir + "/range_poly.pcd", *range_poly);

    auto start1 = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    if (pcl::io::loadPCDFile(pcd_file, *pc_ptr) == -1) {
        std::cerr << "[road_boundary] Error: Unable to load PCD file: " << pcd_file << std::endl;
        return 1;
    }
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1-start1);
    std::cout << "readpcd time: " << duration1.count() / 1000.0 << std::endl;

    // 点云切割
    pcl::PointXYZ min_p1, max_p1;
    pcl::getMinMax3D(*range_poly, min_p1, max_p1);
    min_p1.z = -10;
    max_p1.z = 10;
    std::cout << " min_p1: " << min_p1.x << " " << min_p1.y << " " << min_p1.z << " max_p1: " << max_p1.x << " " << max_p1.y << " " << max_p1.z <<  std::endl;

    // {
    //     auto start1 = std::chrono::high_resolution_clock::now();
    //     pcl::PointCloud<MyColorPointType>::Ptr cropped_cloud(new pcl::PointCloud<MyColorPointType>);
    //     pcl::CropBox<MyColorPointType> crop_filter;
    //     crop_filter.setInputCloud(pc_ptr);// 设置待滤波点云
    //     crop_filter.setMin(min_p1.getVector4fMap());
    //     crop_filter.setMax(max_p1.getVector4fMap());
    //     crop_filter.filter(*cropped_cloud); // 执行滤波，保存滤波结果于cloud_filtered
    //     // pcl::io::savePCDFileBinary(output_dir + "/boundary_cropped.pcd", *cropped_cloud);

    //     // 点云过滤
    //     pcl::PointCloud<ConRmPointType>::Ptr filtercloud_boundary(new pcl::PointCloud<ConRmPointType>);
    //     pclFilter::ConditionalRemoval(cropped_cloud, filtercloud_boundary, "cloud_bev_center_line", 2);

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    //     for (const auto& pcl_p : *filtercloud_boundary) {
    //         cloud_boundary->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
    //         // indexMap[cloud_boundary->size() - 1] = rawCloudIndex++;
    //     }
    //     auto end1 = std::chrono::high_resolution_clock::now();
    //     auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1-start1);
    //     std::cout << "method1: " << duration1.count() / 1000.0 << std::endl;
    //     pcl::io::savePCDFileBinary(output_dir + "/boundary_all1.pcd", *cloud_boundary);
    // }

    auto start2 = std::chrono::high_resolution_clock::now();
    // 点云过滤
    pcl::PointCloud<ConRmPointType>::Ptr filtercloud_boundary(new pcl::PointCloud<ConRmPointType>);
    pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, "cloud_bev_center_line", 2);

    pcl::PointCloud<ConRmPointType>::Ptr cropped_cloud(new pcl::PointCloud<ConRmPointType>);
    pcl::CropBox<ConRmPointType> crop_filter;
    crop_filter.setInputCloud(filtercloud_boundary);// 设置待滤波点云
    crop_filter.setMin(min_p1.getVector4fMap());
    crop_filter.setMax(max_p1.getVector4fMap());
    crop_filter.filter(*cropped_cloud); // 执行滤波，保存滤波结果于cloud_filtered
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pcl_p : *cropped_cloud) {
        cloud_boundary->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        // indexMap[cloud_boundary->size() - 1] = rawCloudIndex++;
    }
    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2-start2);
    std::cout << "method2: " << duration2.count() / 1000.0 << std::endl;

    if (cloud_boundary->empty()) {
        std::cerr << "[road_boundary] Error: cloud_boundary is empty after filtering." << std::endl;
        return 1;
    }
    // pcl::io::savePCDFileBinary(output_dir + "/boundary_all2.pcd", *cloud_boundary);

    // TODO：qzc
    // 统计滤波
    // pcl::PointCloud<pcl::PointXYZ>::Ptr laneSegCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pclFilter::StatisticalOutlierRemoval(cloud_boundary, laneSegCloud, 50, 0.1f);
    // if(laneSegCloud->empty()) {
    //     return 1;
    // }

    std::vector<std::vector<int>> clusters;
    float radius = 0.5f;
    int minPts = 50;
    Inference::DBSCAN(cloud_boundary, radius, minPts, clusters);
    std::cout<<"[lane_boundary]cluster size: "<<clusters.size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr debug_clustersCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < clusters.size(); ++i) {
        if(clusters[i].size() < 10)
            continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr clustersCloud_i(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr debug_clustersCloud_i(new pcl::PointCloud<pcl::PointXYZI>);
        for (int j = 0; j < clusters[i].size(); j++) {
            pcl::PointXYZ curPXYZ = cloud_boundary->at(clusters[i][j]);
            debug_clustersCloud->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            debug_clustersCloud_i->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            
            clustersCloud_i->push_back(pcl::PointXYZ(curPXYZ.x, curPXYZ.y, curPXYZ.z));
        }
        pcl::io::savePCDFileBinary(output_dir + "/reg_clustersCloud_"+std::to_string(i)+".pcd", *debug_clustersCloud_i);

        {
            std::string output_dir_in(output_dir+"/");
            std::string cluster_index(std::to_string(i));

            RoadMapping::SkeletonCluster skeleton_cluster;
            // 如果需要分割，则进行 skeleton 分割
            std::map<int, int> indexMap;
            skeleton_cluster.SetInput(clustersCloud_i, output_dir_in, cluster_index);
            // if (skeleton_cluster.Process())
            if (skeleton_cluster.ProcessBoundary()) {
                auto &cluster_results = skeleton_cluster.GetResult();
                for (auto &one_cluster : cluster_results) {
                    std::cout << " " << one_cluster.size()<< std::endl;
                }
            }
        }
    }

    pcl::io::savePCDFileBinary(output_dir + "/reg_clustersCloud.pcd", *debug_clustersCloud);

    return 0;
}
