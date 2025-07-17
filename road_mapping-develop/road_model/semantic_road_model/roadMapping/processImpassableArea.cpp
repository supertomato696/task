//
//
//

#include "processImpassableArea.h"
#include "pclFilter.h"
#include "cluster.h"
#include "pclPtType.h"

#include "Geometries/BaseAlgorithm.h"
#include "Geometries/GeometryAlgorithm.h"
#include "Geometries/LinearRing.h"
#include "Geometries/Polygon.h"

#include "./include/CloudAlgorithm.h"
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

// int shape = 0;
// {
//     if (raw_bev_id == 1 || raw_bev_id == 8  || raw_bev_id == 15) shape = 1;  // solid
//     else if (raw_bev_id == 2 || raw_bev_id == 9  || raw_bev_id == 16) shape = 2;  // dashed
//     else if (raw_bev_id == 3 || raw_bev_id == 10 || raw_bev_id == 17) shape = 3;  // double_solid
//     else if (raw_bev_id == 4 || raw_bev_id == 11 || raw_bev_id == 18) shape = 4;  // double_dashed
//     else if (raw_bev_id == 5 || raw_bev_id == 12 || raw_bev_id == 19) shape = 5;  // left_dashed_right_solid
//     else if (raw_bev_id == 6 || raw_bev_id == 13 || raw_bev_id == 20) shape = 6;  // left_solid_right_dashed
//     else if (raw_bev_id == 7 || raw_bev_id == 14)                     shape = 7;  // unset_type
//     else if (raw_bev_id == 30)                                        shape = 8;  // polygon_unknow_kind
//     else if (raw_bev_id == 31)                                        shape = 9;  // polygon_flowerbed
//     else if (raw_bev_id == 32)                                        shape = 10; // polygon_sentry_box
//     else if (raw_bev_id == 33)                                        shape = 11; // polygon_physical_safe_island
//     else if (raw_bev_id == 34)                                        shape = 12; // polygon_linear_safe_island
// }
// clang-format on

namespace RoadMapping
{
    void processImpassableArea::run_impassable_area_process(std::string pcdPath, std::string outputDir)
    {
        if (!boost::filesystem::exists(pcdPath))
        {
            std::cout << "全局点云不存在：" << pcdPath << std::endl;
            return;
        }

        pcl::PointCloud<MyColorPointType>::Ptr cloud_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(pcdPath, *cloud_ptr);

        // polygon_unknow_kind shape = 8
        pcl::PointCloud<MyColorPointType>::Ptr unknow_kind_cloud_ptr(new pcl::PointCloud<MyColorPointType>);
        pclFilter::ConditionalRemoval(cloud_ptr, unknow_kind_cloud_ptr, "cloud_bev_shape", 8);
        run_polygon_area(unknow_kind_cloud_ptr, 8, outputDir);

        // polygon_flowerbed shape = 9
        pcl::PointCloud<MyColorPointType>::Ptr flowerbed_cloud_ptr(new pcl::PointCloud<MyColorPointType>);
        pclFilter::ConditionalRemoval(cloud_ptr, flowerbed_cloud_ptr, "cloud_bev_shape", 9);
        run_polygon_area(flowerbed_cloud_ptr, 9, outputDir);

        // polygon_sentry_box shape = 10
        pcl::PointCloud<MyColorPointType>::Ptr sentry_box_cloud_ptr(new pcl::PointCloud<MyColorPointType>);
        pclFilter::ConditionalRemoval(cloud_ptr, sentry_box_cloud_ptr, "cloud_bev_shape", 10);
        run_polygon_area(sentry_box_cloud_ptr, 10, outputDir);

        // polygon_physical_safe_island shape = 11
        pcl::PointCloud<MyColorPointType>::Ptr physical_safe_island_cloud_ptr(new pcl::PointCloud<MyColorPointType>);
        pclFilter::ConditionalRemoval(cloud_ptr, physical_safe_island_cloud_ptr, "cloud_bev_shape", 11);
        run_polygon_area(physical_safe_island_cloud_ptr, 11, outputDir);

        // polygon_linear_safe_island shape = 12
        pcl::PointCloud<MyColorPointType>::Ptr linear_safe_island_cloud_ptr(new pcl::PointCloud<MyColorPointType>);
        pclFilter::ConditionalRemoval(cloud_ptr, linear_safe_island_cloud_ptr, "cloud_bev_shape", 12);
        run_polygon_area(linear_safe_island_cloud_ptr, 12, outputDir);
    }

    void processImpassableArea::run_circle_area(const pcl::PointCloud<MyColorPointType>::Ptr &cloudPtr, int type, std::string output_dir)
    {
        return;
    }

    void processImpassableArea::run_polygon_area(const pcl::PointCloud<MyColorPointType>::Ptr &cloudPtr, int type, std::string output_dir)
    {
        if (cloudPtr->empty())
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloudptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_type_trans(cloudPtr, pcl_cloudptr);

        // 将点进行xoy平面投影
        for (auto &pt : pcl_cloudptr->points)
        {
            pt.z = 0.0;
        }

        // 进行聚类
        float radius = 1.0f;
        int minPtsSerch = 10;
        std::vector<std::vector<int>> clusters;
        Inference::DBSCAN(pcl_cloudptr, radius, minPtsSerch, clusters);
        std::cout << "extract clusters size: " << clusters.size() << std::endl;

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> edge_lines;
        // 每个聚类进行边缘提取
        for (int index = 0; index < clusters.size(); index++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudOri(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*pcl_cloudptr, clusters[index], *clusterCloudOri);

            // 提取聚类边缘点
            pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // 求聚类点云的凹包
            pcl::ConcaveHull<pcl::PointXYZ> chull; // 创建多边形提取对象
            chull.setInputCloud(clusterCloudOri);  // 设置输入点云为投影后点云cloud_projected
            chull.setAlpha(0.1);                   // 设置alpha值为0.1
            chull.reconstruct(*edge_cloud);        // 重建提取创建凹多边形

            if (edge_cloud->empty() || edge_cloud->points.empty())
            {
                continue;
            }

            Engine::Base::Array<Engine::Geometries::Coordinate> chull_pts_arr;
            for (auto &pt : edge_cloud->points)
            {
                chull_pts_arr.Add(Engine::Geometries::Coordinate(pt.x, pt.y, pt.z));
            }

            // 根据边缘点求取凸包
            Engine::Base::Array<Engine::Geometries::Coordinate> edge_pts;
            Engine::Geometries::BaseAlgorithm::PointsPackage(chull_pts_arr, edge_pts);

            if (edge_pts.GetCount() < 3)
            {
                continue;
            }

            // 将多边形转换为逆时针
            anticlockwisePolygon(edge_pts);

            edge_lines.Add(edge_pts);
        }

        if (!edge_lines.IsEmpty())
        {
            pcl::KdTreeFLANN<MyColorPointType>::Ptr kd_cloud_ptr(new pcl::KdTreeFLANN<MyColorPointType>);
            kd_cloud_ptr->setInputCloud(cloudPtr);
            for (int i = 0; i < edge_lines.GetCount(); i++)
            {
                int n = edge_lines[i].GetCount();
                for (int j = 0; j < n; j++)
                {
                    MyColorPointType search_pcl;
                    search_pcl.x = edge_lines[i][j].x;
                    search_pcl.y = edge_lines[i][j].y;
                    search_pcl.z = edge_lines[i][j].z;

                    int K = 1;
                    std::vector<int> Idx;
                    std::vector<float> SquaredDistance;
                    kd_cloud_ptr->nearestKSearch(search_pcl, K, Idx, SquaredDistance);
                    edge_lines[i][j].z = cloudPtr->points.at(Idx[0]).z;
                }
                edge_lines[i][0] = edge_lines[i][n - 1];
            }

            hdmap_build::CommonUtil::WriteToOBJ(edge_lines, output_dir, "impassablearea_" + std::to_string(type), true);
        }
    }

    void processImpassableArea::pcl_type_trans(const pcl::PointCloud<MyColorPointType>::Ptr &cloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_cloudptr)
    {
        if (cloudPtr->empty())
        {
            return;
        }
        for (auto &pt : cloudPtr->points)
        {
            pcl_cloudptr->points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }
    }

    void processImpassableArea::anticlockwisePolygon(Engine::Base::Array<Engine::Geometries::Coordinate> &edge_pts) // 将多边形转换为逆时针
    {
        // 根据可以选取X或者Y值中最大或者最小的点，这个点必然是凸点
        //  p12 x p23 = (x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)
        //  为正时，p1-p2-p3 路径的走向为逆时针，
        //  为负时，p1-p2-p3 走向为顺时针，
        //  为零时，p1-p2-p3 所走的方向不变，亦即三点在一直线上。
        int n = edge_pts.GetCount();
        if (n < 4)
        {
            return;
        }
        double minx = edge_pts[0].x;
        int minx_index = 0;

        double flag = 0;
        for (int i = 1; i < n; i++)
        {
            if (edge_pts[i].x < minx)
            {
                minx = edge_pts[i].x;
                minx_index = i;
            }
        }
        Engine::Geometries::Coordinate p1, p3;
        Engine::Geometries::Coordinate p2 = edge_pts[minx_index];
        if (minx_index == 0)
        {
            p1 = edge_pts[n - 1];
            p3 = edge_pts[minx_index + 1];
        }
        else if (minx_index == n - 1)
        {
            p1 = edge_pts[minx_index - 1];
            p3 = edge_pts[0];
        }
        else
        {
            p1 = edge_pts[minx_index - 1];
            p3 = edge_pts[minx_index + 1];
        }

        flag = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);
        if (flag < 0.0)
        {
            edge_pts.Reverse();
        }
    }
}
