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
#include <opencv2/ximgproc.hpp>
#include <malloc.h>
#include "include/log.h"
#include "include/util.h"
#include "include/json.h"
#include "parallel_hashmap/phmap.h"

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

static cv::Mat bin_img_dilate_erode(cv::Mat &bin_img, int size)
{
    // 输入输出 单通道 前景白色 背景黑色
    cv::Mat out;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
    cv::dilate(bin_img, out, element); // 膨胀
    cv::erode(out, out, element);      // 腐蚀
    return out;
}

static cv::Mat bin_img_erode_dilate(cv::Mat &bin_img, int size)
{
    // 输入输出 单通道 前景白色 背景黑色
    cv::Mat out;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
    cv::erode(bin_img, out, element); // 腐蚀
    cv::dilate(out, out, element);    // 膨胀
    return out;
}

static cv::Mat bin_img_remove_small_region(cv::Mat &bin_img, int min_area)
{
    // 输入输出 单通道 前景白色 背景黑色
    cv::Mat out(bin_img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat stats, centroids, labelImage, imshow_mat;
    int nLabels = cv::connectedComponentsWithStats(
        bin_img, labelImage, stats, centroids, 8);
    cv::Mat mask(labelImage.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat surfSup = stats.col(4) >= min_area;
    int tmp_label;
    for (int i = 1; i < bin_img.rows; i++)
    {
        for (int j = 1; j < bin_img.cols; j++)
        {
            tmp_label = labelImage.at<int>(i, j);
            mask.at<char>(i, j) = (char)surfSup.at<char>(tmp_label, 0);
        }
    }
    bin_img.copyTo(out, mask);
    return out;
}

static cv::Mat bin_img_to_skel_img(const cv::Mat &bin_img)
{
    // 输入输出 单通道 前景白色 背景黑色
    cv::Mat img = bin_img.clone();
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    bool done;
    do
    {
        cv::erode(img, eroded, element);
        cv::dilate(eroded, temp, element); // temp = open(img)
        cv::subtract(img, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(img);
        done = (cv::countNonZero(img) == 0);
    } while (!done);
    return skel;
}

class System
{
public:
    System() {}
    ~System() {}

    std::string input_pcd_path;
    std::string output_folder_path;
};

int main(int argc, char **argv)
{
    ArgParser arg_parser;
    arg_parser.add<std::string>("input", '\0', "输入 pcd path", true, "");
    arg_parser.add<std::string>("output", '\0', "输出文件夹", true, "");
    arg_parser.parse_check(argc, argv);

    System system;
    system.input_pcd_path = arg_parser.get<std::string>("input");
    system.output_folder_path = arg_parser.get<std::string>("output");
    add_folder_if_not_exist(path_join(system.output_folder_path, "debug"));

    auto logger = Logger::instance();
    logger->setLogLevel(spdlog::level::level_enum::info, spdlog::level::level_enum::trace);
    logger->setup(path_join(system.output_folder_path, "log", _time_str_s() + "_center_line_extract.log"), "center_line_extract");

    LOG_INFO("============================================");
    LOG_INFO("  Hello center line extract");
    LOG_INFO("  input:{}", system.input_pcd_path);
    LOG_INFO("  output:{}", system.output_folder_path);
    LOG_INFO("============================================");

    pcl::PointCloud<ModelPointType>::Ptr pc_ptr(new pcl::PointCloud<ModelPointType>);
    pcl::io::loadPCDFile(system.input_pcd_path, *pc_ptr);

    ModelPointType min_xyz;
    ModelPointType max_xyz;
    pcl::getMinMax3D(*pc_ptr, min_xyz, max_xyz);

    double pixel_cell = 0.05;
    int canvas_width = (max_xyz.x - min_xyz.x) / pixel_cell;
    int canvas_height = (max_xyz.y - min_xyz.y) / pixel_cell;

    cv::Mat raw_center_line;
    {
        raw_center_line = cv::Mat::zeros(canvas_height, canvas_width, CV_8UC1);
        for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++)
        {
            ModelPointType &p = (*iter);
            if (p.cloud_bev_label_1 == 2)
            {
                int canvas_x = canvas_width - round((max_xyz.x - p.x) / pixel_cell);
                int canvas_y = round((max_xyz.y - p.y) / pixel_cell);
                raw_center_line.at<uint8_t>(canvas_y, canvas_x) = 255;
            }
        }
        cv::imwrite(path_join(system.output_folder_path, "1-raw_center_line.png"), raw_center_line);
    }

    cv::Mat raw_center_line_dilate_erode;
    {
        raw_center_line_dilate_erode = bin_img_erode_dilate(raw_center_line, 5);
        raw_center_line_dilate_erode = bin_img_dilate_erode(raw_center_line_dilate_erode, 41);
        cv::imwrite(path_join(system.output_folder_path, "2-raw_center_line_dilate_erode.png"), raw_center_line_dilate_erode);
    }

    cv::Mat center_line_rm_small;
    {
        LOG_INFO("do 移除小区域");
        center_line_rm_small = bin_img_remove_small_region(raw_center_line_dilate_erode, 1000);
        cv::imwrite(path_join(system.output_folder_path, "3-center_line_rm_small.png"), center_line_rm_small);
    }

    cv::Mat ca_id_map; // 联通区域 ID 图，int 类型, 0 为背景
    int ca_num;        // 连通域个数，包含背景0
    {
        LOG_INFO("do 联通区域");
        std::vector<cv::Vec3b> temp_zone_id_to_color;
        cv::Mat centroids, stats;
        ca_num = connectedComponentsWithStats(raw_center_line_dilate_erode, ca_id_map, stats, centroids, 8);
        temp_zone_id_to_color.resize(ca_num + 1);
        temp_zone_id_to_color[0] = cv::Vec3b(0, 0, 0);
        for (int i = 1; i <= ca_num; i++)
        {
            temp_zone_id_to_color[i] = cv::Vec3b(rand() % 200 + 1, rand() % 200 + 1, rand() % 200 + 1);
        }
        cv::Mat center_line_ca(ca_id_map.size(), CV_8UC3);
        for (int y = 0; y < ca_id_map.rows; y++)
        {
            for (int x = 0; x < ca_id_map.cols; x++)
            {
                center_line_ca.at<cv::Vec3b>(y, x) = temp_zone_id_to_color[ca_id_map.at<int>(y, x)];
            }
        }
        cv::imwrite(path_join(system.output_folder_path, "4-center_line_ca.png"), center_line_ca);
    }

    // {
    //     // 输入输出 单通道 前景白色 背景黑色
    //     int size = 21;
    //     cv::Mat out;
    //     cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
    //     cv::erode(raw_center_line_dilate_erode, out, element); // 腐蚀
    //     raw_center_line_dilate_erode.setTo(0, out > 0);
    //     cv::imwrite(path_join(system.output_folder_path, "4.5-center_line_thin.png"), raw_center_line_dilate_erode);
    // }

    cv::Mat center_line_skel;
    {
        LOG_INFO("do 骨架提取");
        cv::ximgproc::thinning(raw_center_line_dilate_erode, center_line_skel);
        // center_line_skel = bin_img_to_skel_img(raw_center_line_dilate_erode);
        cv::imwrite(path_join(system.output_folder_path, "5-center_line_skel.png"), center_line_skel);
    }

    LOG_INFO("Finish center line extract");
    return 0;
}

/*
/home/test/code/fsd_map_bev_develop/mapping/simple_mapper/bin/center_line_extract --input=/home/test/bev_offline/29960/bev_mapping/flat_mapping/ground.pcd --output=/home/test/bev_offline/29960/bev_mapping
*/
