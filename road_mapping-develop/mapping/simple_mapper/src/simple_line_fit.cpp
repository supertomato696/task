#include <Eigen/Geometry>
#include <array>
#include <random>
#include <vector>
#include <thread>
#include <istream>
#include <unordered_set>
#include <map>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/common/impl/common.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <malloc.h>
#include "include/log.h"
#include "include/util.h"
#include "include/json.h"
#include "parallel_hashmap/phmap.h"
#include "include/sxf_cluster.h"

// clang-format off
struct EIGEN_ALIGN16 ModelPointType
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;
    uint16_t label;
    uint8_t cloud_pano_seg;
    uint8_t cloud_line_seg;
    uint16_t distance_x_cm;
    uint16_t distance_y_cm;
    uint8_t cloud_bev_label;
    uint8_t cloud_bev_label_1;
    uint8_t cloud_bev_label_2;
    uint8_t cloud_bev_color;
    uint8_t cloud_bev_shape;
    uint8_t cloud_bev_label_score;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(ModelPointType,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint8_t, cloud_pano_seg, cloud_pano_seg)
                                  (uint8_t, cloud_line_seg, cloud_line_seg)
                                  (uint16_t, distance_x_cm, distance_x_cm)
                                  (uint16_t, distance_y_cm, distance_y_cm)
                                  (uint8_t, cloud_bev_label, cloud_bev_label)
                                  (uint8_t, cloud_bev_label_1, cloud_bev_label_1)
                                  (uint8_t, cloud_bev_label_2, cloud_bev_label_2)
                                  (uint8_t, cloud_bev_color, cloud_bev_color)
                                  (uint8_t, cloud_bev_shape, cloud_bev_shape)
                                  (uint8_t, cloud_bev_label_score, cloud_bev_label_score))
// clang-format on

class Line
{
public:
    Line() {}
    ~Line() {}
    std::string pcd_path;
    std::string line_id;
};

class System
{
public:
    System() {}
    ~System() {}

    std::string input_json_path;
    std::string output_folder_path;
    std::vector<Line> line_list;
    std::string flag;

    void read_json()
    {
        nlohmann::json j = nlohmann::json::parse(std::ifstream(input_json_path));
        for (auto one : j["lines"])
        {
            Line line;
            line.line_id = one["line_id"];
            line.pcd_path = one["pcd_path"];

            line_list.push_back(line);
        }
        LOG_INFO("完成读入 json, 线数：{}", line_list.size());
    }

    void sxf_line_fit(Line &line)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(line.pcd_path, *clusterCloud);

        float resolution = 0.2f;
        clusterCloud = SampleByGrid(clusterCloud, resolution);
        std::vector<Segment> segments;
        ExtractSegment(clusterCloud, segments);
        LOG_INFO("segments.size() {}", segments.size());

        for (int j = 0; j < segments.size(); j++)
        {

            pcl::io::savePCDFileBinary(path_join(output_folder_path, line.line_id + "_" + std::to_string(j) + "_segmentCloud.pcd"), *segments[j].cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*segments[j].cloud, segments[j].anchorIndices, *anchorCloud);
            pcl::io::savePCDFileBinary(path_join(output_folder_path, line.line_id + "_" + std::to_string(j) + "_anchorCloud.pcd"), *segments[j].cloud);
            segments[j].QpSpline();
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[j].smoothTrjCloud();
            pcl::io::savePCDFileBinary(path_join(output_folder_path, line.line_id + "_" + std::to_string(j) + "_trjCloud.pcd"), *segments[j].cloud);
        }
    }

    void run_all_line_fit()
    {
        for (Line &line : line_list)
        {
            sxf_line_fit(line);
        }
    }
};

int main(int argc, char **argv)
{
    ArgParser arg_parser;
    arg_parser.add<std::string>("input", '\0', "输入 json path", true, "");
    arg_parser.add<std::string>("output", '\0', "输出文件夹", true, "");
    arg_parser.parse_check(argc, argv);

    System system;
    system.input_json_path = arg_parser.get<std::string>("input");
    system.output_folder_path = arg_parser.get<std::string>("output");
    add_folder_if_not_exist(path_join(system.output_folder_path, "debug"));

    auto logger = Logger::instance();
    logger->setLogLevel(spdlog::level::level_enum::info, spdlog::level::level_enum::trace);
    logger->setup(path_join(system.output_folder_path, "log", _time_str_s() + "_simple_line_fit.log"), "simple_line_fit");

    LOG_INFO("============================================");
    LOG_INFO("  Hello simple line fit");
    LOG_INFO("  input:{}", system.input_json_path);
    LOG_INFO("  output:{}", system.output_folder_path);
    LOG_INFO("============================================");

    system.read_json();
    system.run_all_line_fit();

    LOG_INFO("Finish simple line fit");
    return 0;
}
