//
//
//

#include "processLuKouPly.h"
#include "pclFilter.h"
#include "cluster.h"
#include "pclPtType.h"

#include "Geometries/BaseAlgorithm.h"
#include "Geometries/GeometryAlgorithm.h"
#include "Geometries/LinearRing.h"
#include "Geometries/Polygon.h"

#include "./include/CloudAlgorithm.h"
#include "./include/CommonUtil.h"
#include "./include/RoadTopoBuild.h"

#include <boost/filesystem.hpp>
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法
#include <pcl/sample_consensus/model_types.h>  //模型定义
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/filters/voxel_grid.h>            //基于体素网格化的滤波
#include <pcl/filters/extract_indices.h>       //索引提取
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/registration/icp.h>
#include <pcl/console/print.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

namespace RoadMapping
{

    void processLuKouPly::run_yxx_fin_in(std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir)
    {
        // 1、读取label CLASS_TYPE_140 = 140,  //stop_line
        std::string lidarFile = pcdPath;
        std::cout << "lidarFile: " << lidarFile << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(lidarFile, *pc_ptr);

        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter(new pcl::PointCloud<MyColorPointType>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "label", 0);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (auto iter = cloudfilter->begin(); iter != cloudfilter->end(); iter++)
        {
            MyColorPointType &pcl_p = (*iter);
            cloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }

        //*根据人行横道确认路口大致范围*//
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> arrArrPts;
        std::string crossObjPath = outputDir + "/crosswalk.obj";
        ReadObj(crossObjPath, arrArrPts);
        Engine::Base::Array<Engine::Geometries::Coordinate> crossMidPts;
        GetAvePnts(arrArrPts, crossMidPts);

        // 1.根据人行横道中点进行聚类（100米）
        Engine::Base::Array<Engine::Base::Array<Engine::Base::Int32>> cluster_indices;
        hdmap_build::RoadTopoBuild::ConnectedComponent(crossMidPts, cluster_indices, 100.0);

        // 2.根据聚类得到路口的大致范围
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> lukouPlyPts;
        for (int i = 0; i < cluster_indices.GetCount(); ++i)
        {
            if (cluster_indices[i].GetCount() < 2)
                continue; // 只有一个人行横道暂且不提路口面

            Engine::Base::Array<Engine::Geometries::Coordinate> arrCrosswalkPts;
            for (int j = 0; j < cluster_indices[i].GetCount(); ++j)
            {
                int index = cluster_indices[i][j];
                arrCrosswalkPts.Add(arrArrPts[index]);
            }

            Engine::Base::Array<Engine::Geometries::Coordinate> arrPackgePoints;
            Engine::Geometries::BaseAlgorithm::PointsPackage(arrCrosswalkPts, arrPackgePoints);

            double offsetDis = 0.5;
            Engine::Base::Array<Engine::Geometries::Coordinate> arrOffsetPlyPts;
            GetPlyOffset(arrPackgePoints, arrOffsetPlyPts, offsetDis);
            RemovePntsByDis(arrOffsetPlyPts, 1.0); // 删除线之间角度大于160的点
            if (arrOffsetPlyPts.GetCount() > 3)
                lukouPlyPts.Add(arrOffsetPlyPts);
        }
        hdmap_build::CommonUtil::WriteToOBJ(lukouPlyPts, outputDir, "lukou", true);
    }

    void processLuKouPly::run_bev_lukouboundary(std::string dataDir, std::string global_pcd_path, std::string bev_obj_dir, std::string outputDir)
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr ori_bevlukoucloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_bev_label_2", 3);
        for (auto iter = cloudfilter->begin(); iter != cloudfilter->end(); iter++)
        {
            MyColorPointType &pcl_p = (*iter);
            ori_bevlukoucloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }
        if (ori_bevlukoucloud->empty() || ori_bevlukoucloud->points.empty())
        {
            std::cout << "无路口bev_label点云：" << global_pcd_path << std::endl;
            return;
        }
        pcl::io::savePCDFileBinary(outputDir + "/ori_cloud_bev_label_lukou.pcd", *ori_bevlukoucloud);

        // 对点云进行体素滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::VoxelGridSampling(ori_bevlukoucloud, cloudVoxelGrid, _VoxelGrid_size);
        pcl::io::savePCDFileBinary(outputDir + "/ori_cloud_bev_label_lukou_Voxel.pcd", *cloudVoxelGrid);

        ///////////////////bev数据////////////////////////
        std::cout << "开始处理bev数据" << std::endl;
        if (!boost::filesystem::exists(bev_obj_dir))
        {
            return;
        }

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> ori_bev_lukou_polygons;
        bev_lukou_obj(bev_obj_dir, outputDir, ori_bev_lukou_polygons);
        if (ori_bev_lukou_polygons.IsEmpty())
        {
            std::cout << "bev数据为空" << std::endl;
            return;
        }

        // 为每个多边形打分
        cal_bev_polygon_core(ori_bev_lukou_polygons, cloudVoxelGrid, 0.70);

        // filter多余多边形
        filter_bev_polygons();
    }

    void processLuKouPly::run_cloud_bev_lukoubd(std::string global_pcd_path, std::string outputDir)
    {
        // 采用cloud_bev_lable_2 中 value=3的点云生成路口外边界
        std::cout << "开始处理点云数据" << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(global_pcd_path, *pc_ptr);
        if (pc_ptr->points.empty())
        {
            std::cout << "点云文件无数据：global_pcd_path:" << global_pcd_path << std::endl;
            return;
        }

        pcl::PointCloud<ConRmPointType>::Ptr cloudfilter(new pcl::PointCloud<ConRmPointType>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ori_bevlukoucloud(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_bev_label_2", 3);
        for (auto iter = cloudfilter->begin(); iter != cloudfilter->end(); iter++)
        {
            ConRmPointType &pcl_p = (*iter);
            ori_bevlukoucloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }
        if (ori_bevlukoucloud->empty() || ori_bevlukoucloud->points.empty())
        {
            std::cout << "无路口bev_label点云：" << global_pcd_path << std::endl;
            return;
        }
        pcl::io::savePCDFileBinary(outputDir + "/ori_cloud_bev_label_lukou.pcd", *ori_bevlukoucloud);

        // 对点云进行体素滤波
        float leaf_size = 0.2f;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::VoxelGridSampling(ori_bevlukoucloud, cloud_voxel, leaf_size);

        // 对路口进行聚类
        float radius = 1.0f;
        int minPtsSerch = 10;
        std::vector<std::vector<int>> clusters;
        Inference::DBSCAN(cloud_voxel, radius, minPtsSerch, clusters);
        std::cout << "extract clusters size: " << clusters.size() << std::endl;

        // 测试输出聚类后数据
        int minPts = 3000;
        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i < clusters.size(); ++i)
        {
            if (clusters[i].size() < minPts)
            {
                continue;
            }
            for (int j = 0; j < clusters[i].size(); j++)
            {
                pcl::PointXYZ curPXYZ = cloud_voxel->at(clusters[i][j]);
                clustersCloudWithI->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            }
        }
        if (clustersCloudWithI->empty() || clustersCloudWithI->points.empty())
        {
            return;
        }

        pcl::io::savePCDFileBinary(outputDir + "/" + "lukou_clustersCloud.pcd", *clustersCloudWithI);

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> edges_polygons;
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> edges_polygons_opencv;
        for (int index = 0; index < clusters.size(); index++)
        {
            if (clusters[index].size() < minPts)
            {
                continue;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloudOri(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud_voxel, clusters[index], *clusterCloudOri);

            Engine::Base::Array<Engine::Geometries::Coordinate> one_lukou_boundary_bycloudedge;
            get_lukou_by_cloudedge(clusterCloudOri, one_lukou_boundary_bycloudedge);
            if (one_lukou_boundary_bycloudedge.GetCount() > 4)
            {
                edges_polygons.Add(one_lukou_boundary_bycloudedge);
            }

            // Engine::Base::Array<Engine::Geometries::Coordinate> one_lukou_boundary_bycontours;
            // get_lukou_by_opencvcontours(leaf_size, clusterCloudOri, one_lukou_boundary_bycontours, index, outputDir);
            // edges_polygons_opencv.Add(one_lukou_boundary_bycontours);
        }

        hdmap_build::CommonUtil::WriteToOBJ(edges_polygons, outputDir, "lukou", true);
        // hdmap_build::CommonUtil::WriteToOBJ(edges_polygons_opencv, outputDir, "lukou_opencv", true);
    }

    void processLuKouPly::get_lukou_by_cloudedge(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cluster, Engine::Base::Array<Engine::Geometries::Coordinate> &edge_pts)
    {
        // 提取聚类边缘点
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // 求聚类点云的凹包
        pcl::ConcaveHull<pcl::PointXYZ> chull; // 创建多边形提取对象
        chull.setInputCloud(cloud_cluster);    // 设置输入点云为投影后点云cloud_projected
        chull.setAlpha(0.1);                   // 设置alpha值为0.1
        chull.reconstruct(*edge_cloud);        // 重建提取创建凹多边形

        if (edge_cloud->empty() || edge_cloud->points.empty())
        {
            return;
        }

        Engine::Base::Array<Engine::Geometries::Coordinate> chull_pts_arr;
        for (auto &pt : edge_cloud->points)
        {
            chull_pts_arr.Add(Engine::Geometries::Coordinate(pt.x, pt.y, pt.z));
        }

        // 根据边缘点求取凸包
        Engine::Geometries::BaseAlgorithm::PointsPackage(chull_pts_arr, edge_pts);
        if (edge_pts.GetCount() < 3)
        {
            edge_pts.Clear();
        }

        // 删除长度比较短的线
        RemovePntsByDis(chull_pts_arr, 0.50);

        if (edge_pts.GetCount() < 3)
        {
            edge_pts.Clear();
            return;
        }

        // 将多边形转换为逆时针
        anticlockwisePolygon(edge_pts);
    }

    void processLuKouPly::get_lukou_by_opencvcontours(float voxelGrid_size, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cluster, Engine::Base::Array<Engine::Geometries::Coordinate> &edge_pts, int index, std::string outputDir)
    {
        // 将聚类点放到opencv图像中
        double pixel_size_m = voxelGrid_size;
        pcl::PointXYZ min_p1, max_p1;
        pcl::getMinMax3D(*cloud_cluster, min_p1, max_p1);

        int min_x = (floor((min_p1.x) / 10)) * 10;
        int min_y = (floor((min_p1.y) / 10)) * 10;
        int max_x = (ceil((max_p1.x) / 10)) * 10;
        int max_y = (ceil((max_p1.y) / 10)) * 10;

        int width = (max_x - min_x) / pixel_size_m;
        int height = (max_y - min_y) / pixel_size_m;

        cv::Mat cloud_image = cv::Mat::zeros(height, width, CV_8UC1);
        for (pcl::PointXYZ p : cloud_cluster->points)
        {
            int x = round((p.x - min_x + 0.0001) / pixel_size_m);
            int y = round(height - (p.y - min_y + 0.0001) / pixel_size_m);
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }

            cloud_image.at<uint8_t>(y, x) = 255;
        }
        cv::imwrite((outputDir + "/lukou_" + std::to_string(index) + "_bev.png"), cloud_image);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(cloud_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // std::cout << index << "找到的轮廓个数为："<< contours.size() << std::endl;

        // 填充看效果
        cv::Mat maxConnectedArea = cv::Mat::zeros(height, width, CV_8UC1);
        for (size_t i = 0; i < contours.size(); i++)
        {
            std::vector<std::vector<cv::Point>> vvContour{contours[i]};
            cv::fillPoly(maxConnectedArea, vvContour, cv::Scalar(255));
        }

        // 进行膨胀腐蚀
        cv::erode(maxConnectedArea, maxConnectedArea, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        cv::dilate(maxConnectedArea, maxConnectedArea, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

        cv::dilate(maxConnectedArea, maxConnectedArea, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        cv::erode(maxConnectedArea, maxConnectedArea, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        cv::imwrite((outputDir + "/lukou_" + std::to_string(index) + "_bev_fillpolygon.png"), maxConnectedArea);

        contours.clear();
        cv::findContours(maxConnectedArea, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // std::cout << index << "经过膨胀腐蚀后，找到的轮廓个数为："<< contours.size() << std::endl;

        cv::Mat drawing = cv::Mat::zeros(maxConnectedArea.size(), CV_8UC3);
        for (int i = 0; i < contours.size(); i++)
        {
            cv::Scalar color(0, 255, 0); //=Scalar color=Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255))
            cv::drawContours(drawing, contours, i, color, 1);
        }

        // cv::drawContours(image_binary, contours, -1, cv::Scalar(255), 1);
        // 选取面积最大的轮廓
        double max_area = 0.0;
        int max_index = -1;
        for (int i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > max_area)
            {
                max_area = area;
                max_index = i;
            }
        }

        if (max_index < 0)
        {
            return;
        }

        // 对轮廓进行近似处理
        std::vector<cv::Point> cnt = contours[max_index];
        std::vector<cv::Point> approx;
        cv::approxPolyDP(cnt, approx, 3, True);

        std::vector<std::vector<cv::Point>> approx_vec;
        approx_vec.push_back(approx);

        cv::Mat drawing2 = cv::Mat::zeros(maxConnectedArea.size(), CV_8UC3);
        cv::Scalar color(0, 255, 0);
        cv::drawContours(drawing2, approx_vec, 0, color, 2);
        cv::imwrite((outputDir + "/lukou_" + std::to_string(index) + "_bev_contours_epsilon.png"), drawing2);

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_cloud;
        kdtree_cloud.setInputCloud(cloud_cluster);

        for (int i = 0; i < approx.size(); i++)
        {
            cv::Point cur_pt = approx[i];
            double x = cur_pt.x * pixel_size_m + min_x;
            double y = (height - cur_pt.y) * pixel_size_m + min_y;
            pcl::PointXYZ pt(float(x), float(y), 0);
            int K = 1;
            std::vector<int> Idx;
            std::vector<float> SquaredDistance;
            kdtree_cloud.nearestKSearch(pt, K, Idx, SquaredDistance);
            pt.z = cloud_cluster->points.at(Idx[0]).z;

            edge_pts.Add(Engine::Geometries::Coordinate(x, y, pt.z));
        }

        int n = edge_pts.GetCount();
        if (n < 3)
        {
            return;
        }

        Engine::Geometries::Coordinate s = edge_pts[0];
        Engine::Geometries::Coordinate e = edge_pts[n - 1];
        if (s.DistanceXY(e) > 0.5)
        {
            edge_pts.Add(s);
        }
        else
        {
            edge_pts[n - 1] = edge_pts[0];
        }

        anticlockwisePolygon(edge_pts);
    }

    void processLuKouPly::anticlockwisePolygon(Engine::Base::Array<Engine::Geometries::Coordinate> &edge_pts) // 将多边形转换为逆时针
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

    void processLuKouPly::ReadObj(std::string filepath, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts)
    {
        std::ifstream infile(filepath);
        if (!infile.good())
            return; // 检查文件的存在与否

        std::ifstream ifs(filepath.c_str());
        if (!ifs.is_open())
            return;

        std::vector<Engine::Geometries::Coordinate> vecCoordinates;
        char sz[10240];
        while (ifs.getline(sz, 10240))
        {
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
                std::copy(vecCoordinates.begin() + start_index - 1, vecCoordinates.begin() + end_index, vecLine.Begin());
                arrArrPts.Add(vecLine);
            }
        }
    }

    void processLuKouPly::GetAvePnts(Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts, Engine::Base::Array<Engine::Geometries::Coordinate> &avePnts)
    {
        for (int i = 0; i < arrArrPts.GetCount(); ++i)
        {
            double x = 0.0, y = 0.0, z = 0.0;
            int n = arrArrPts[i].GetCount();
            for (int j = 0; j < n; ++j)
            {
                x += arrArrPts[i][j].x;
                y += arrArrPts[i][j].y;
                z += arrArrPts[i][j].z;
            }
            x = x / n;
            y = y / n;
            z = z / n;
            avePnts.Add(Engine::Geometries::Coordinate(x, y, z));
        }
    }

    void processLuKouPly::GetPlyOffset(Engine::Base::Array<Engine::Geometries::Coordinate> &oriPlyPts, Engine::Base::Array<Engine::Geometries::Coordinate> &offsetPlyPts, double offsetDis)
    {
        int n = oriPlyPts.GetCount();
        if (n < 3)
            return;
        if (offsetDis <= 0)
            return;

        Engine::Base::Array<Engine::Geometries::Coordinate *> *coords = new Base::Array<Geometries::Coordinate *>;
        for (int i = 0; i < oriPlyPts.GetCount(); ++i)
        {
            coords->Add(new Engine::Geometries::Coordinate(oriPlyPts[i]));
        }
        Engine::Geometries::LinearRing lr(coords);
        Engine::Geometries::Polygon pPolygon(new Engine::Geometries::LinearRing(lr));

        Engine::Geometries::Polygon *newOffsetPly = new Engine::Geometries::Polygon(pPolygon);
        Engine::Base::Double dis = offsetDis;
        newOffsetPly = Engine::Geometries::GeometryAlgorithm::GenerateOffsetPolygon(&pPolygon, dis);
        if (newOffsetPly != NULL)
        {
            Engine::Base::Array<Engine::Geometries::Coordinate *> *coords = newOffsetPly->GetExteriorRing()->GetCoordinates();
            for (int i = 0; i < coords->GetCount(); ++i)
                offsetPlyPts.Add(Engine::Geometries::Coordinate(coords->GetAt(i)->x, coords->GetAt(i)->y, coords->GetAt(i)->z));
        }
    }

    void processLuKouPly::RemovePntsByAngle(Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts, double angle)
    {
        // 适用于首尾相接的polygon
        if (arrPts.GetCount() < 4)
            return;

        for (int i = 1; i < arrPts.GetCount() - 1; ++i)
        {
            Engine::Geometries::Coordinate curPt = arrPts[i];
            Engine::Geometries::Coordinate prePt = arrPts[i - 1];
            Engine::Geometries::Coordinate nexPt = arrPts[i + 1];
            double value = Engine::Geometries::BaseAlgorithm::ComputeAngle(prePt, curPt, nexPt);
            double anglePt = fabs(value * 180 / M_PI);
            if (anglePt > angle)
            {
                arrPts.Delete(i);
                i--;
            }
        }
    }

    void processLuKouPly::RemovePntsByDis(Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts, double minDis)
    {
        // 适用于首尾相接的polygon
        if (arrPts.GetCount() < 4)
            return;

        for (int i = 0; i < arrPts.GetCount() - 1; ++i)
        {
            Engine::Geometries::Coordinate curPt = arrPts[i];
            Engine::Geometries::Coordinate nexPt = arrPts[i + 1];
            double dis = curPt.Distance(nexPt);
            if (dis < minDis)
            {
                if (i == 0)
                {
                    arrPts.Delete(i + 1);
                    i--;
                }
                else
                {
                    arrPts.Delete(i);
                    i--;
                }
            }
        }
    }

    void processLuKouPly::pts_geo_polygon(const Engine::Base::Array<Engine::Geometries::Coordinate> &plyCoords, Engine::Geometries::Polygon &resPolygon)
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

    void processLuKouPly::get_Envelope(const Engine::Base::Array<Engine::Geometries::Coordinate> &arrPoints, Engine::Geometries::Envelope &envelope)
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

    void processLuKouPly::cal_bev_polygon_core(const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &bev_crosswalks_polygons, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull, double toler)
    {
        std::mutex mtx;
        std::cout << "------>对bev每个数据打分" << std::endl;

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
            if (area < 10.0)
            {
                continue;
            }

            Engine::Geometries::Coordinate middle_pt = hdmap_build::CommonUtil::GetAveragePt(bev_crosswalks_polygons[i]);

            // 在中心点30米范围内检索点,最小个数需满足 toler * area / 体素面积
            std::vector<int> k_indices;
            std::vector<float> k_distances;
            pcl::PointXYZ newPt(middle_pt.x, middle_pt.y, 0.0);
            kdtree_cloud->radiusSearch(newPt, 50.0, k_indices, k_distances);

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
                BevPolygon new_bev_crosswalk;
                new_bev_crosswalk.index = i + 1;
                new_bev_crosswalk.isSave = true;
                new_bev_crosswalk.poly_enve = newEnvelope;
                new_bev_crosswalk.area = area;
                new_bev_crosswalk.score = score;
                new_bev_crosswalk.num_crosswalk_pts_in = matchnum;
                new_bev_crosswalk.polygon_pts = bev_crosswalks_polygons[i];
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

        hdmap_build::CommonUtil::WriteToOBJ(bev_save_polygons, _output, "score_filter_bev_lukou" + std::to_string(int(toler * 100)), true);
    }

    void processLuKouPly::filter_bev_polygons()
    {
        std::cout << "------>根据分数去重" << std::endl;
        for (int i = 0; i < _bev_lukou_bds_polys.GetCount(); i++)
        {
            if (!_bev_lukou_bds_polys[i].isSave)
            {
                continue;
            }
            for (int j = 0; j < _bev_lukou_bds_polys.GetCount(); j++)
            {
                if (!_bev_lukou_bds_polys[j].isSave)
                {
                    continue;
                }
                if (_bev_lukou_bds_polys[i].index == _bev_lukou_bds_polys[j].index)
                {
                    continue;
                }
                Engine::Geometries::Envelope Intersec;
                bool isOverlap = _bev_lukou_bds_polys[i].poly_enve.Intersection(_bev_lukou_bds_polys[j].poly_enve, Intersec);
                if (!isOverlap)
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
                if (overlap1 > 0.1 || overlap2 > 0.1)
                {
                    int n1 = _bev_lukou_bds_polys[i].num_crosswalk_pts_in;
                    int n2 = _bev_lukou_bds_polys[j].num_crosswalk_pts_in;

                    // 判断包含点数在相差不大的时候 以score为主 否则以点数为主
                    int diff = fabs(n1 - n2);
                    double diffRadio1 = double(diff) / double(n1);
                    double diffRadio2 = double(diff) / double(n2);
                    if (fabs(diffRadio1 - diffRadio2) < 0.02)
                    {
                        // 看哪个包含了更多的点
                        if (_bev_lukou_bds_polys[j].score > _bev_lukou_bds_polys[i].score)
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
                        // 看哪个包含了更多的点
                        if (n2 > n1)
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
        for (int i = n - 1; i >= 0; i--)
        {
            if (!_bev_lukou_bds_polys[i].isSave)
            {
                _bev_lukou_bds_polys.Delete(i);
            }
            else
            {
                bev_save_polygons.Add(_bev_lukou_bds_polys[i].polygon_pts);
                _bev_lukou_bds_polys[i].index = bev_save_polygons.GetCount();
            }
        }
        hdmap_build::CommonUtil::WriteToOBJ(bev_save_polygons, _output, "lukou", true);
    }

    void processLuKouPly::bev_lukou_obj(const std::string bev_obj_floder, const std::string out_obj_floder, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &bev_lukou_polygons)
    {
        if (!boost::filesystem::exists(bev_obj_floder))
        {
            return;
        }

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> ori_bev_lukou_polygons;

        boost::filesystem::path path(bev_obj_floder);
        for (const auto &iter : boost::filesystem::directory_iterator(path))
        {
            if (boost::filesystem::is_directory(iter.path()))
            {
                std::string bev_lukou_pcd = iter.path().string() + "/lukouboundary_bev.pcd";
                if (boost::filesystem::exists(bev_lukou_pcd))
                {
                    std::cout << "读取bev crosswalk:" << bev_lukou_pcd << std::endl;
                    ReadObjFromPcd(bev_lukou_pcd, ori_bev_lukou_polygons);
                }
            }
        }
        if (!ori_bev_lukou_polygons.IsEmpty())
        {
            for (int i = 0; i < ori_bev_lukou_polygons.GetCount(); i++)
            {
                int n = ori_bev_lukou_polygons[i].GetCount();
                if (n < 3)
                {
                    continue;
                }

                // 将多边形进行闭合
                Engine::Geometries::Coordinate s = ori_bev_lukou_polygons[i][0];
                Engine::Geometries::Coordinate e = ori_bev_lukou_polygons[i][n - 1];
                if (s.DistanceXY(e) > 0.5)
                {
                    ori_bev_lukou_polygons[i].Add(s);
                }
                else
                {
                    ori_bev_lukou_polygons[i][n - 1] = ori_bev_lukou_polygons[i][0];
                }

                bev_lukou_polygons.Add(ori_bev_lukou_polygons[i]);
            }

            hdmap_build::CommonUtil::WriteToOBJ(bev_lukou_polygons, out_obj_floder, "bev_utm_lukou_polygon", true);
        }
    }

    void processLuKouPly::ReadObjFromPcd(std::string bev_lukou_pcd, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts)
    {
        if (!boost::filesystem::exists(bev_lukou_pcd))
        {
            std::cout << "文件不存在:" << bev_lukou_pcd << std::endl;
            return; // 检查文件的存在与否
        }

        pcl::PointCloud<PointElement>::Ptr pc_ptr(new pcl::PointCloud<PointElement>);
        pcl::io::loadPCDFile(bev_lukou_pcd, *pc_ptr);

        std::map<int, Engine::Base::Array<Engine::Geometries::Coordinate>> line_map;
        for (auto &pt : pc_ptr->points)
        {
            Engine::Geometries::Coordinate newCoord(pt.x, pt.y, pt.z);
            line_map[pt.id].Add(newCoord);
        }
        for (auto &line : line_map)
        {
            if (line.second.GetCount() > 3)
            {
                arrArrPts.Add(line.second);
            }
        }
    }

    float processLuKouPly::pointToCenterDis(float x1, float y1, float x2, float y2) {
        float dis = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        return dis;
    }

    void processLuKouPly::generate_lukou_boundary_manual(std::string middle_json_path, std::string global_pcd_path, std::string outputDir)
    {
        // 从task_info_damo.json里读取site_center中心点坐标（已从GCJ02坐标系转换为utm坐标系）
        std::string file_name = middle_json_path;
        std::ifstream json_file(file_name);
        if (!json_file.is_open()) {
            std::cout << "[lukou]无法打开json文件: " << file_name << std::endl;
            return;
        }

        nlohmann::json root = nlohmann::json::parse(json_file);
        if (!root["middle"].contains("site_center") || !root["middle"]["site_center"].is_array()) {
            std::cout << "[lukou]site_center数据结构不正确" << std::endl;
            return;
        }

        const auto &site_center_obj = root["middle"]["site_center"];
        double site_center_utm_x = site_center_obj[0];
        double site_center_utm_y = site_center_obj[1];
        std::cout << "[lukou]site_center: " << site_center_utm_x << ", " << site_center_utm_y << std::endl;

        // 读取site_center 50m范围内的停止线和人行横道点云
        std::cout << "[lukou]开始处理点云数据" << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        if (pcl::io::loadPCDFile(global_pcd_path, *pc_ptr) == -1) {
            std::cout << "[lukou]点云文件无数据: global_pcd_path:" << global_pcd_path << std::endl;
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<ConRmPointType>::Ptr cloudCrosswalk(new pcl::PointCloud<ConRmPointType>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudCrosswalk, "cloud_bev_label_2", 3); // 人行横道
        
        pcl::PointCloud<ConRmPointType>::Ptr cloudStopLine(new pcl::PointCloud<ConRmPointType>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudStopLine, "cloud_bev_label", 4); // 停止线

        
        for (const auto& point : cloudCrosswalk->points) {
            if (pointToCenterDis(site_center_utm_x, site_center_utm_y, point.x, point.y) <= 50.0) {
                cloudFiltered->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
            }
        }

        for (const auto& point : cloudStopLine->points) {
            if (pointToCenterDis(site_center_utm_x, site_center_utm_y, point.x, point.y) <= 50.0) {
                cloudFiltered->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
            }
        }

        if (cloudFiltered->points.empty() || cloudFiltered->points.size() < 3) {
            std::cout << "[lukou]无法计算凸包，点云文件无数据或点数不足" << std::endl;
            return;
        }

        // 计算cloudFiltered的凸包
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHull(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::ConcaveHull<pcl::PointXYZ> convexHull;
        pcl::ConvexHull<pcl::PointXYZ> convexHull;
        convexHull.setInputCloud(cloudFiltered);
        // convexHull.setAlpha(0.1);
        convexHull.reconstruct(*cloudHull);

        if (cloudHull->points.empty()) {
            std::cout << "[lukou]凸包为空" << std::endl;
            return;
        }

        Engine::Base::Array<Engine::Geometries::Coordinate> chull_pts_arr;
        for (const auto &pt : cloudHull->points)
        {
            chull_pts_arr.Add(Engine::Geometries::Coordinate(pt.x, pt.y, pt.z));
        }

        // 使用PointsPackage函数进行点打包
        Engine::Base::Array<Engine::Geometries::Coordinate> edge_pts;
        Engine::Geometries::BaseAlgorithm::PointsPackage(chull_pts_arr, edge_pts);
        if (edge_pts.GetCount() < 3)
        {
            edge_pts.Clear();
            return;
        }

        // 将多边形转换为逆时针
        anticlockwisePolygon(edge_pts);

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> edges_polygons;
        if (edge_pts.GetCount() > 4)
        {
            edges_polygons.Add(edge_pts);
        }

        hdmap_build::CommonUtil::WriteToOBJ(edges_polygons, outputDir, "lukou", true);
    }    

}

