#pragma once

#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "pclPtType.h"

namespace RoadMapping{

struct LineSection{
    std::vector<Eigen::Vector3f> pts;
    Eigen::Vector3f leftPt;
    Eigen::Vector3f rightPt;
    Eigen::Vector3f leftTruncatePt;
    Eigen::Vector3f rightTruncatePt;
    bool flag_tracked = false;
};

struct Sector{
    std::map<int, LineSection> lineSections;
    std::vector<LineSection> mergedLineSections;
    std::vector<int> prevIndices;
    std::vector<int> nextIndices;
};

class PostProcessLaneBoundary{
public:
    static void run(std::string dataDir, std::string outputDir);
    static void run_yxx(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir);
    static void run_yxx_2(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir,std::string label="label");
    static void get_ref_info(std::string parse_json,std::string outputDir, pcl::PointCloud<pcl::PointXYZ>::Ptr &laneSegCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud,pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor);
    static void lane_extract_base(std::string outputDir,std::string outnamechar,pcl::PointCloud<pcl::PointXYZ>::Ptr &laneSegCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor);
    static float quadArea(LineSection &a, LineSection &b);
    static void MergeLineSection(Sector &sector, Eigen::Vector3f refDirection);
    static bool estimateDirection(LineSection &lineSection, int classId, Sector &prevSector, Sector &nextSector, Eigen::Vector3f refDirection);
    static cv::Mat bin_img_to_skel_img(cv::Mat &bin_img);
    static cv::Mat bin_img_remove_small_region(cv::Mat &bin_img, int min_area);
    static void process_lane(std::string outputDir,std::string parse_json,std::string outnamechar,pcl::PointCloud<MyColorPointType>::Ptr &laneSegCloud);
};

} //namespace RoadMapping