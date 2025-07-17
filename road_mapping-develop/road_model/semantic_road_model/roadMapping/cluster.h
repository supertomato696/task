#pragma once
#include <vector>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "json.hpp"


namespace Inference{
Eigen::Vector3f GetWLH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void BFS(const cv::Mat &img, std::vector<std::vector<std::pair<int,int>>> &clusters, int typeId);

void DBSCAN(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius, int minPts, 
            std::vector<std::vector<int>> &clusters);

pcl::PointCloud<pcl::PointXYZ>::Ptr SampleByGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution);    

struct Segment{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    //std::vector<double> ts;
    std::vector<int> anchorIndices;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> mCoeffs_x; //optimized x coeff
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> mCoeffs_y; //optimized y coeff
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> mCoeffs_z; //optimized z coeff
    std::map<int, int> segmentCloudIndexMap;//segment cloud中每个点与输入点云的索引对应关系，first:segment cloud的索引，second:输入点云的索引
    std::map<int, int> trjCloudIndexMap;//trajectory cloud中每个点与样条系数组的索引对应关系，first:trajectory cloud的索引，second:三次样条系数组的索引
    Eigen::Vector2f direction_rough;

    Segment():cloud(new pcl::PointCloud<pcl::PointXYZ>){}
    void DecideOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseCloud,
                           const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree);
    bool QpSpline();
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothTrjCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothTrjCloud2();
    void selectAnchor(int stride);
    void addSegmentCoeff(nlohmann::json &obj, int segmentId);
    void addSegmentCoeff_straightLine(nlohmann::json &obj, int segmentId);
};

void ExtractSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Segment> &segments,
                    float radius = 2.f, int minPts = 3, int minAnchorNum = 5);

} //namespace Inference

