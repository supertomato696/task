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
#include "include/types.h"

void output_bev(pcl::PointCloud<MyColorPointType> &point_cloud, double pixel_size_m, std::string output_folder)
{
    add_folder_if_not_exist(output_folder);
    MyColorPointType min_p1, max_p1;
    pcl::getMinMax3D(point_cloud, min_p1, max_p1);

    int min_x = int(min_p1.x - 2);
    int min_y = int(min_p1.y - 2);
    int max_x = int(max_p1.x + 2);
    int max_y = int(max_p1.y + 2);

    int width = (max_x - min_x) / pixel_size_m;
    int height = (max_y - min_y) / pixel_size_m;

    std::sort(point_cloud.begin(), point_cloud.end(),
              [](MyColorPointType pt1, MyColorPointType pt2)
              { return pt1.z < pt2.z; });

    cv::Mat rgb_image = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat ground_lable_image(height, width, CV_8UC1, 255);

    for (MyColorPointType p : point_cloud.points)
    {
        int x = round((p.x - min_x + 0.0001) / pixel_size_m);
        int y = round(height - (p.y - min_y + 0.0001) / pixel_size_m);
        if (rgb_image.at<cv::Vec3b>(y, x)[0] != 0)
        {
            continue;
        }
        else
        {
            rgb_image.at<cv::Vec3b>(y, x)[0] = p.b;
            rgb_image.at<cv::Vec3b>(y, x)[1] = p.g;
            rgb_image.at<cv::Vec3b>(y, x)[2] = p.r;
        }
        // is_road 放到了 a 字段中，bit_0: 感知当前车道地面, bit_1: ransac 地面, bit_0 and bit_1 感知且RANSAC路面
        if ((p.a & 3) == 3)
        {
            ground_lable_image.at<uint8_t>(y, x) = p.label;
        }
    }
    cv::imwrite(path_join(output_folder, "rgb_image.png"), rgb_image);
    cv::imwrite(path_join(output_folder, "ground_lable.png"), ground_lable_image);

    {
        std::ofstream file(path_join(output_folder, "left_up_point.txt"));
        file << min_x << " " << max_y;
        file.close();
    }
}

int main(int argc, char **argv)
{
    ArgParser arg_parser;
    arg_parser.add<std::string>("input", '\0', "输入 pcd path", true, "");
    arg_parser.add<std::string>("output", '\0', "输出文件夹", true, "");
    arg_parser.add<std::string>("input2", '\0', "输入 pcd2 path", false, "");
    arg_parser.parse_check(argc, argv);

    std::string input_pcd_path = arg_parser.get<std::string>("input");
    std::string output_folder = arg_parser.get<std::string>("output");
    std::string input_pcd_path2 = arg_parser.get<std::string>("input2");

    // spdlog
    auto logger = Logger::instance();
    logger->setLogLevel(spdlog::level::level_enum::info,
                        spdlog::level::level_enum::trace);
    std::string logFile = output_folder + "/log/" + _time_str_s() + "_cloud_to_image.log";
    logger->setup(logFile, "cloud_to_image");

    LOG_INFO("============================================");
    LOG_INFO("  Hello cloud_to_image");
    LOG_INFO("  input_pcd_path:{}", input_pcd_path);
    LOG_INFO("  input_pcd_path2:{}", input_pcd_path2);
    LOG_INFO("  output_folder:{}", output_folder);
    LOG_INFO("============================================");

    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::io::loadPCDFile(input_pcd_path, *pc_ptr);
    if (input_pcd_path2 != "")
    {
        pcl::PointCloud<MyColorPointType>::Ptr pc2_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(input_pcd_path2, *pc2_ptr);
        *pc_ptr += *pc2_ptr;
    }

    LOG_INFO("开始生成 road_bev 图片");
    output_bev(*pc_ptr, 0.05, output_folder);
    return 0;
}