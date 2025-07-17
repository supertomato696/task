#pragma once
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "cluster.h"
#include "pclPtType.h"
#include "Geometries/BaseAlgorithm.h"

namespace RoadMapping{

class ProcessRoadBoundary{
public:
//    static void run(std::string dataDir, std::string outputDir);
//    static void run_yxx(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir);
//    static void run_yxx_2(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir);
    static void run_qzc_lane_center_in_road(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir, std::string label="label");
    static void run_qzc_fin_in_road(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir, std::string label="label");
    static void run_yxx_fin_in_road(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir, std::string label="label");
    static void run_yxx_all_road(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir); //尝试从fin_in中所有road点提取道路边界
    static void EigenSolverEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr &laneSegCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor, double radius, pcl::PointCloud <pcl::Normal>::Ptr& cloud_normals);
    static void road_cloud_edge_precess(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir);
    static void get_ref_info(std::string parse_json,std::string outputDir, pcl::PointCloud<pcl::PointXYZ>::Ptr &laneSegCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud,pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor);
    static void run_yxx_fin_add_type(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir, std::string label="label");
    static void EigenSolverEstimationNew(pcl::PointCloud<pcl::PointXYZ>::Ptr &Cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor, double radius, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, std::map<int, int>& indexMap);
    static void generateOutputCloud(int index, pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud, Inference::Segment segment, pcl::PointCloud<ConRmPointType>::Ptr rawCloud, pcl::PointCloud<PointElement>::Ptr outputCloud, int ele_type);

    static bool ProcessSkeletonCluster(int index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                        const std::vector<int> &clusters,
                                        const std::map<int, int>& indexMap,
                                        std::string outputDir,
                                        std::vector<Inference::Segment>& segments,
                                        Eigen::Vector2f direction_rough);
};

} //namespace RoadMapping