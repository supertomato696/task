//
//
//

#ifndef ROADMAPPING_PROCESSSTOPLINE_H
#define ROADMAPPING_PROCESSSTOPLINE_H

#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pclPtType.h"
#include "Base/Array.h"
#include "Geometries/Envelope.h"
#include "Geometries/Coordinate.h"

namespace RoadMapping
{

    class processStopline
    {
        struct StoplineGeo
        {
            Engine::Geometries::Envelope enve; // 外包框2d
            vector<Eigen::Vector3f> plypts;
            bool flag; // 是否处理
            int index; // 原始index
            //        double area;//面积
        };

        struct NearbyLinesResult {
            std::vector<int> line_ids;  // 符合条件的线段ID
            std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> line_points;  // 各线段在阈值内的点云
        };

        struct DirectionFilterResult {
            std::vector<int> filtered_lines;  // 过滤后的线段ID
            Eigen::Vector3f main_direction;   // 最终主方向（单位向量）
        };

    public:
        static void run_yxx_fin_in(std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir);

        static void run_yxx_fin_add_type(std::string middle_json_path, std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir, std::string data_type);

        static void fit_line(const Engine::Base::Array<Engine::Geometries::Coordinate> &vecPoints, Engine::Geometries::Coordinate &pntStart, Engine::Geometries::Coordinate &pntEnd);

        static void run_extract_guide_line(std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir);

        static void objsToPointElementPCD(const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrLines, std::string pcdpath, uint16_t ele_type = 0, uint16_t type1 = 0, uint16_t type2 = 0, uint16_t type3 = 0, float heading = 0.0, float score = 1.0);

        static void generateOutputCloud(int index, float yaw, int arrow_type, pcl::PointCloud<PointElement>::Ptr &outputCloud, Engine::Base::Array<Engine::Geometries::Coordinate> &LinePts, float score, int ele_type);

        static int getClusterBevShape(const std::vector<int>& cluster, const std::map<int, int>& indexMap, const pcl::PointCloud<ConRmPointType>::Ptr& cloudfilter);

        static float computeProjectionOverlap(float a1, float a2, float b1, float b2);
        static float computeOverlap(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2,
                        const Eigen::Vector3f& other_point1, const Eigen::Vector3f& other_point2,
                        const Eigen::Vector3f& line_vector);
        static pcl::PointCloud<PointElement>::Ptr afterProcess(std::string middle_json_path, std::string dataDir, std::string refFile, pcl::PointCloud<PointElement>::Ptr stopline);
        
        static NearbyLinesResult findNearbyLinesWithMinPoints(const pcl::PointXYZ& target_point, double threshold,
            const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache, int min_points = 2);  
        static NearbyLinesResult findLinesWithDynamicThreshold(const pcl::PointXYZ& target_point, double initial_threshold,
            double max_threshold, const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache);
        static DirectionFilterResult filterLinesByDirection(const NearbyLinesResult& candidates, float angle_threshold_deg = 30.0f);     
    
    
    };
} // namespace RoadMapping

#endif // ROADMAPPING_PROCESSSTOPLINE_H
