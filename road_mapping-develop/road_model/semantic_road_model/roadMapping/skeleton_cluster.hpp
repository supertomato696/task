#pragma once
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/kdtree/kdtree_flann.h>
// #include "cluster.h"
#include "pclPtType.h"
#include "cluster.h"
// #include "Geometries/BaseAlgorithm.h"

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include <functional>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>

// 为cv::Point定义哈希函数
namespace std
{
    template <>
    struct hash<cv::Point>
    {
        std::size_t operator()(const cv::Point &pt) const
        {
            return std::hash<int>()(pt.x) ^ std::hash<int>()(pt.y);
            // std::size_t hash_x = std::hash<int>()(pt.x);
            // std::size_t hash_y = std::hash<int>()(pt.y);
            // return hash_x ^ (hash_y << 1);
        }
    };
} // namespace std


namespace RoadMapping{


#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_BLUE "\033[34m"
#define COLOR_DEFAULT "\033[0m"

class SkeletonCluster
{
private:
    cv::Mat bev_img_;
    cv::Mat bev_img_z_;
    std::string output_dir_;
    std::string cluster_index_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud_ptr_;
    std::vector<std::vector<int>> cluster_indexs; // skeleton 聚类的索引坐标

    std::unordered_map<cv::Point, int> m_xy_index_to_cloud_index_;

    float buffer_ = 0.5;
    float resolution_ = 0.05;
    float g_x_min_;
    float g_x_max_;
    float g_y_min_;
    float g_y_max_;
    int x_bins_;
    int y_bins_;

    Eigen::Vector2i min_index_xy_;
    Eigen::Vector2i max_index_xy_;

    // bool save_image_ = false;
    bool save_image_ = true;
public:
    SkeletonCluster();
    ~SkeletonCluster();

    void SetInput(pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud, std::string output_dir = "debug", std::string cluster_index = "0");

    void Clear(){
        clusterCloud_ptr_.reset();
        cluster_indexs.clear();
        m_xy_index_to_cloud_index_.clear();
        // m_xy_index_to_cloud_index_.swap(std::unordered_map<cv::Point, int>());
        std::unordered_map<cv::Point, int>().swap(m_xy_index_to_cloud_index_);	
    }

    bool Process();
    bool Process2();
    bool ProcessBoundary();
    std::vector<std::vector<int>>& GetResult() {
        return cluster_indexs;
    };
private:
    void GenBEV();
    void GenBEV_V2();
    bool RefineEdge(int refine_mode, const std::vector<std::vector<cv::Point>>& edges_in, std::vector<std::vector<cv::Point>>& edges, cv::Point2f& centroid_out);
    pcl::PointXYZI BevPixelToPoint(cv::Point& p_in, int index);
    PointElement BevPixelToPoint(cv::Point& p_in, int ele_type, int cluster_id, int point_index, bool use_z = true);
    int SkeletonGetEndPt(const cv::Mat& skeletonImg, std::vector<cv::Point>& endPoints);
    int SkeletonGetJunctionPt(const cv::Mat& biImg, std::vector<cv::Point>& ptPairVec, std::vector<cv::Point>& endPoints);
    cv::Mat ExtractSkeleton(const cv::Mat& input);
    std::vector<std::vector<cv::Point>> SplitSkeleton(const cv::Mat &skeleton,
                                                  const std::vector<cv::Point> &endPoints,
                                                  const std::vector<cv::Point> &junctionPoints);
    cv::Scalar GetCvColor(float size, float index);

    inline Eigen::Vector2i position_xy_to_5cm_index_xy(const Eigen::Vector2f &p_xy)
    {
        return Eigen::Vector2i(int(round((p_xy[0] + 50000) * 20)), int(round((p_xy[1] + 50000) * 20)));
    }

    inline Eigen::Vector2f index_xy_to_position_xy(const Eigen::Vector2i &index)
    {
        return Eigen::Vector2f(double(index[0]) / 20 - 50000, double(index[1]) / 20 - 50000);
    }

    inline Eigen::Vector2i position_xy_to_5cm_index_xy(float x, float y, int x_offset, int y_offset)
    {
        return Eigen::Vector2i(int(round((x + x_offset) * 20)), int(round((y + y_offset) * 20)));
    }

    inline Eigen::Vector2f index_xy_to_position_xy(int x, int y, int x_offset, int y_offset)
    {
        return Eigen::Vector2f(double(x) / 20 - x_offset, double(y) / 20 - y_offset);
    }

};

}