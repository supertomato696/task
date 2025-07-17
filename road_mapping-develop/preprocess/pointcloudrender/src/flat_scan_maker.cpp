#include <algorithm>
#include <array>
#include <random>
#include <vector>
#include <thread>
#include <istream>
#include <unordered_set>
#include <map>
#include <stdlib.h>
#include <set>
#include <cmath>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <omp.h>
#include <malloc.h>

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
#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_stick.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle3d.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/impl/prosac.hpp>
#include <pcl/sample_consensus/impl/rmsac.hpp>
#include <pcl/sample_consensus/impl/msac.hpp>
#include <pcl/sample_consensus/impl/mlesac.hpp>
#include <pcl/sample_consensus/impl/rransac.hpp>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/impl/lmeds.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

#include "include/log.h"
#include "include/util.h"
#include "include/json.h"
#include "parallel_hashmap/phmap.h"
#include "bev_label_map.h"

// struct EIGEN_ALIGN16 RenderPointType
// {
//     PCL_ADD_POINT4D;
//     PCL_ADD_RGB; // a: bit_0: 感知当前路面, bit_1: RANSAC 路面, bit_2 PCA 平面, bit_3 PCA 直线, bit_6 night, bit_7 bad_point
//     PCL_ADD_INTENSITY;
//     uint16_t ring;
//     uint16_t label;
//     uint16_t distance_x_cm;
//     uint16_t distance_y_cm;
//     uint8_t geom_type; // bit_0: 感知当前路面, bit_1: RANSAC 路面, bit_2 PCA 平面, bit_3 PCA 直线, bit_6 night, bit_7 bad_point
//     uint8_t cloud_pano_seg;
//     uint8_t cloud_line_seg;
//     uint8_t cloud_bev_label;
//     uint8_t cloud_bev_label_1;
//     uint8_t cloud_bev_label_2;
//     uint8_t cloud_bev_color;
//     uint8_t cloud_bev_shape;
//     uint8_t cloud_bev_center_line;
//     uint8_t cloud_bev_traffic_light;
//     uint8_t cloud_bev_center_line_score;
//     PCL_MAKE_ALIGNED_OPERATOR_NEW
// };

// // clang-format off
// POINT_CLOUD_REGISTER_POINT_STRUCT(RenderPointType,
//                                   (float, x, x)(float, y, y)(float, z, z)
//                                   (float, rgb, rgb)
//                                   (float, intensity, intensity)
//                                   (uint16_t, ring, ring)
//                                   (uint16_t, label, label)
//                                   (uint16_t, distance_x_cm, distance_x_cm)
//                                   (uint16_t, distance_y_cm, distance_y_cm)
//                                   (uint8_t, geom_type, geom_type)
//                                   (uint8_t, cloud_pano_seg, cloud_pano_seg)
//                                   (uint8_t, cloud_line_seg, cloud_line_seg)
//                                   (uint8_t, cloud_bev_label, cloud_bev_label)
//                                   (uint8_t, cloud_bev_label_1, cloud_bev_label_1)
//                                   (uint8_t, cloud_bev_label_2, cloud_bev_label_2)
//                                   (uint8_t, cloud_bev_color, cloud_bev_color)
//                                   (uint8_t, cloud_bev_shape, cloud_bev_shape)
//                                   (uint8_t, cloud_bev_center_line, cloud_bev_center_line)
//                                   (uint8_t, cloud_bev_traffic_light, cloud_bev_traffic_light)
//                                   (uint8_t, cloud_bev_center_line_score, cloud_bev_center_line_score))
// // clang-format on

// struct EIGEN_ALIGN16 RenderPointTypeNew
// {
//     PCL_ADD_POINT4D;
//     PCL_ADD_RGB;
//     PCL_ADD_INTENSITY;
//     uint16_t distance_x_cm;
//     uint16_t distance_y_cm;
//     uint8_t cloud_bev_label;
//     uint32_t cloud_bev_label_shape;
//     uint8_t cloud_bev_label_color;    
//     uint8_t cloud_bev_label_1;
//     uint32_t cloud_bev_label_1_shape;
//     uint8_t cloud_bev_label_1_color;
//     uint8_t cloud_bev_label_2;
//     uint32_t cloud_bev_label_2_shape;
//     uint8_t cloud_bev_label_2_color;
//     uint8_t cloud_bev_center_line;
//     uint32_t cloud_bev_center_line_shape;
//     uint8_t cloud_bev_center_line_color;
//     // uint8_t cloud_bev_traffic_light;
//     // uint8_t cloud_bev_traffic_light_shape;
//     // uint8_t cloud_bev_traffic_light_color;
//     uint8_t cloud_bev_center_line_score;
//     PCL_MAKE_ALIGNED_OPERATOR_NEW
// };

// // clang-format off 
// // 注册的参数个数被限制在20个之内，暂不处理traffic_light
// POINT_CLOUD_REGISTER_POINT_STRUCT(RenderPointTypeNew,
//                                   (float, x, x)(float, y, y)(float, z, z)
//                                   (float, rgb, rgb)
//                                   (float, intensity, intensity)
//                                   (uint16_t, distance_x_cm, distance_x_cm)
//                                   (uint16_t, distance_y_cm, distance_y_cm)
//                                   (uint8_t, cloud_bev_label, cloud_bev_label)
//                                   (uint32_t, cloud_bev_label_shape, cloud_bev_label_shape)
//                                   (uint8_t, cloud_bev_label_color, cloud_bev_label_color)
//                                   (uint8_t, cloud_bev_label_1, cloud_bev_label_1)
//                                   (uint32_t, cloud_bev_label_1_shape, cloud_bev_label_1_shape)
//                                   (uint8_t, cloud_bev_label_1_color, cloud_bev_label_1_color)
//                                   (uint8_t, cloud_bev_label_2, cloud_bev_label_2)
//                                   (uint32_t, cloud_bev_label_2_shape, cloud_bev_label_2_shape)
//                                   (uint8_t, cloud_bev_label_2_color, cloud_bev_label_2_color)
//                                   (uint8_t, cloud_bev_center_line, cloud_bev_center_line)
//                                   (uint32_t, cloud_bev_center_line_shape, cloud_bev_center_line_shape)
//                                   (uint8_t, cloud_bev_center_line_color, cloud_bev_center_line_color)
//                                 //   (uint8_t, cloud_bev_traffic_light, cloud_bev_traffic_light)
//                                 //   (uint8_t, cloud_bev_traffic_light_shape, cloud_bev_traffic_light_shape)
//                                 //   (uint8_t, cloud_bev_traffic_light_color, cloud_bev_traffic_light_color)
//                                   (uint8_t, cloud_bev_center_line_score, cloud_bev_center_line_score))
// // clang-format on


struct EIGEN_ALIGN16 ModelPointTypeNew
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;
    uint8_t cloud_bev_label;
    uint32_t cloud_bev_label_shape;
    uint8_t cloud_bev_label_color;    
    uint8_t cloud_bev_label_1;
    uint32_t cloud_bev_label_1_shape;
    uint8_t cloud_bev_label_1_color;
    uint8_t cloud_bev_label_2;
    uint32_t cloud_bev_label_2_shape;
    uint8_t cloud_bev_label_2_color;
    uint8_t cloud_bev_center_line;
    uint32_t cloud_bev_center_line_shape;
    uint8_t cloud_bev_center_line_color;
    // uint8_t cloud_bev_traffic_light;
    // uint8_t cloud_bev_traffic_light_shape;
    // uint8_t cloud_bev_traffic_light_color;
    uint8_t cloud_bev_label_score;
    uint8_t cloud_bev_center_line_score;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ModelPointTypeNew,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
                                  (uint8_t, cloud_bev_label, cloud_bev_label)
                                  (uint32_t, cloud_bev_label_shape, cloud_bev_label_shape)
                                  (uint8_t, cloud_bev_label_color, cloud_bev_label_color)
                                  (uint8_t, cloud_bev_label_1, cloud_bev_label_1)
                                  (uint32_t, cloud_bev_label_1_shape, cloud_bev_label_1_shape)
                                  (uint8_t, cloud_bev_label_1_color, cloud_bev_label_1_color)
                                  (uint8_t, cloud_bev_label_2, cloud_bev_label_2)
                                  (uint32_t, cloud_bev_label_2_shape, cloud_bev_label_2_shape)
                                  (uint8_t, cloud_bev_label_2_color, cloud_bev_label_2_color)
                                  (uint8_t, cloud_bev_center_line, cloud_bev_center_line)
                                  (uint32_t, cloud_bev_center_line_shape, cloud_bev_center_line_shape)
                                  (uint8_t, cloud_bev_center_line_color, cloud_bev_center_line_color)
                                //   (uint8_t, cloud_bev_traffic_light, cloud_bev_traffic_light)
                                //   (uint8_t, cloud_bev_traffic_light_shape, cloud_bev_traffic_light_shape)
                                //   (uint8_t, cloud_bev_traffic_light_color, cloud_bev_traffic_light_color)
                                  (uint8_t, cloud_bev_label_score, cloud_bev_label_score)
                                  (uint8_t, cloud_bev_center_line_score, cloud_bev_center_line_score))
// clang-format on


class Frame
{
public:
    Frame() {}
    ~Frame() {}

    uint64_t stamp_ms = 0;
    std::string bev_label_path = "";
    std::vector<int> good_bev_line_id;

    std::string day_or_night = "";
    bool has_bev_label = false;
};

class System
{
public:
    System() {}
    ~System() {}

    std::string data_type_ = "";
    std::string data_set_path;
    std::string output_folder;
    std::string trail_id;
    std::string day_or_night;
    std::vector<Frame> frame_list;
    float veh_max_x = 10;
    float veh_min_x = 0;
    float veh_max_y = 10;
    float veh_min_y = -10;
    // float veh_max_x = 10;
    // float veh_min_x = 3;
    // float veh_max_y = 8;
    // float veh_min_y = -8;
    float veh_max_z = 1;
    float veh_min_z = 0;
    float pixel_size = 0.05;
    int mat_width = (veh_max_y - veh_min_y) / pixel_size;
    int mat_height = (veh_max_x - veh_min_x) / pixel_size;

    void read_json()
    {
        nlohmann::json ds_json;
        std::ifstream ifs(data_set_path);
        ifs >> ds_json;
        trail_id = ds_json["header"]["trail_id"];
        day_or_night = ds_json["header"]["tag"][0];
        data_type_ = ds_json["header"]["data_type"];
        // frame信息
        for (auto one : ds_json["data"])
        {
            Frame frame;
            frame.day_or_night = day_or_night;
            frame.stamp_ms = one["stamp_ms"];
            frame.bev_label_path = one["bev_label_path"];
            frame.has_bev_label = !frame.bev_label_path.empty();
            for (int one_id : one["good_bev_line_id"])
            {
                frame.good_bev_line_id.push_back(one_id);
            }
            frame_list.push_back(frame);
        }
        LOG_INFO("数据类型：{}, 完成读入 trail id: {}, 白天 or 夜晚: {}, 帧数: {}", data_type_, trail_id, day_or_night, frame_list.size());
        if (frame_list.size() > 0) {
            LOG_INFO("完成读入 frame.has_bev_label: {}", int(frame_list[0].has_bev_label));
        }
    }

    void init_process()
    {
        add_folder_if_not_exist(output_folder);
    }

    inline Eigen::Vector2i position_to_index2d(Eigen::Vector2f position)
    {
        Eigen::Vector2i index;
        index.x() = round((veh_max_y - position.y()) / pixel_size);
        index.y() = round((veh_max_x - position.x()) / pixel_size);
        return index;
    }

    inline Eigen::Vector2i position_to_index2d(Eigen::Vector2f position, float extra_offset)
    {
        Eigen::Vector2i index;
        index.x() = round((veh_max_y + extra_offset - position.y()) / pixel_size);
        index.y() = round((veh_max_x + extra_offset - position.x()) / pixel_size);
        return index;
    }

    inline Eigen::Vector3i position_to_index3d(Eigen::Vector3f position)
    {
        Eigen::Vector3i index;
        index.x() = round((veh_max_y - position.y()) / pixel_size);
        index.y() = round((veh_max_x - position.x()) / pixel_size);
        index.z() = round((veh_max_z - position.z()) / pixel_size);
        return index;
    }

    Eigen::Vector2f index2d_to_position(const Eigen::Vector2i index)
    {
        Eigen::Vector2f position;
        position.x() = veh_max_x - index.y() * pixel_size;
        position.y() = veh_max_y - index.x() * pixel_size;
        return position;
    }

    Eigen::Vector3f index3d_to_position(const Eigen::Vector3i index)
    {
        Eigen::Vector3f position;
        position.x() = veh_max_x - index.y() * pixel_size;
        position.y() = veh_max_y - index.x() * pixel_size;
        position.z() = veh_max_z - index.z() * pixel_size;
        return position;
    }

    std::vector<Eigen::Vector2f> densify_2d_line(const std::vector<Eigen::Vector2f> &old_line, const float step)
    {
        std::vector<Eigen::Vector2f> new_line;
        if (old_line.size() == 0)
        {
            return new_line;
        }
        if (old_line.size() == 1)
        {
            new_line.push_back(old_line[0]);
            return new_line;
        }
        new_line.push_back(old_line[0]);
        for (int i = 1; i < old_line.size(); ++i)
        {
            auto &one = old_line[i];
            while ((one - new_line.back()).norm() > step)
            {
                float yaw = std::atan2(one.y() - new_line.back().y(), one.x() - new_line.back().x());
                new_line.emplace_back(new_line.back().x() + step * std::cos(yaw), new_line.back().y() + step * std::sin(yaw));
            }
            new_line.push_back(old_line[i]);
        }
        return new_line;
    }

    std::vector<Eigen::Vector2f> fill_the_crosswalk(std::vector<Eigen::Vector2f> &dense_line)
    {
        dense_line.emplace_back(dense_line.front());
        std::vector<Eigen::Vector2f> line = densify_2d_line(dense_line, 0.05);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
        for (auto &p : line)
        {
            tmp->emplace_back(p.x(), p.y(), 0);
        }

        pcl::PointXYZ min_p1, max_p1;
        pcl::getMinMax3D(*tmp, min_p1, max_p1);
        int min_x = int(min_p1.x - 2);
        int min_y = int(min_p1.y - 2);
        int max_x = int(max_p1.x + 2);
        int max_y = int(max_p1.y + 2);
        int width = (max_x - min_x) / 0.05;
        int height = (max_y - min_y) / 0.05;

        cv::Mat crosswalk_image = cv::Mat::zeros(height, width, CV_8UC1);
        cv::Mat crosswalk_full_image = cv::Mat::zeros(height, width, CV_8UC1);
        for (auto &p : tmp->points)
        {
            int x = round((p.x - min_x) / 0.05);
            int y = round(height - (p.y - min_y) / 0.05);
            crosswalk_image.at<uint8_t>(y, x) = 255;
        }

        // TODO：对闭合多边形（如路口面）不做内部填充
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(crosswalk_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        for (size_t k = 0; k < contours.size(); k++)
        {
            std::vector<std::vector<cv::Point>> vvContour{contours[k]};
            cv::fillPoly(crosswalk_full_image, vvContour, cv::Scalar(255));
        }
        cv::erode(crosswalk_full_image, crosswalk_full_image, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        std::vector<Eigen::Vector2f> out_line;
        for (int k = 0; k < crosswalk_full_image.rows; k++)
        {
            for (int j = 0; j < crosswalk_full_image.cols; j++)
            {
                if (crosswalk_full_image.at<uchar>(k, j) == 255)
                {
                    double x = j * 0.05 + min_x;
                    double y = (height - k) * 0.05 + min_y;
                    out_line.emplace_back(x, y);
                }
            }
        }

        return out_line;
    }

    struct Point2DHash
    {
        std::size_t operator()(const Eigen::Vector2i &p) const
        {
            return (p.x() << 20) + p.y();
        }
    };

    struct Point3DHash
    {
        std::size_t operator()(const Eigen::Vector3i &p) const
        {
            return (p.x() << 40) + (p.y() << 20) + p.z();
        }
    };

    // void add_bev_label_ground(std::string bev_label_path,
    //                           phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<float, 7, 1>, Point2DHash> &cloud_bev_label_dict_ground,
    //                           phmap::flat_hash_map<Eigen::Vector2i, float, Point2DHash> &center_line_score_dict)
    // {

    //     nlohmann::json j = nlohmann::json::parse(std::ifstream(bev_label_path));
    //     std::string cat_version = j.contains("cat_version") ? j["cat_version"] : "v1_37";

    //     int instance_n = j["detail"]["instance_list"].size();

    //     for (int instance_id = 0; instance_id < instance_n; instance_id++)
    //     {
    //         auto one_instance = j["detail"]["instance_list"][instance_id];
    //         int raw_bev_id = one_instance["attrs"]["type"]; // 原始 37 类bev label id
    //         float score = one_instance["attrs"]["score"];

    //         int label_0 = 0;
    //         int label_1 = 0;
    //         int label_2 = 0;
    //         int color = 0;
    //         int shape = 0;
    //         int center_line = 0;
    //         int traffic_light = 0;
    //         bev_label_map(cat_version, raw_bev_id, label_0, label_1, label_2, color, shape, center_line, traffic_light);
    //         // if (label_1 == 1 || label_1 == 2 || label_2 == 2) // 如果是线类，检查是否是好线
    //         // {
    //         //     auto it = std::find(frame.good_bev_line_id.begin(), frame.good_bev_line_id.end(), instance_id);
    //         //     if (it == frame.good_bev_line_id.end())
    //         //     {
    //         //         continue;
    //         //     }
    //         // }

    //         // 获取加粗填充稠密线
    //         std::vector<Eigen::Vector2f> dense_line;
    //         std::vector<Eigen::Vector2f> dense_center_line;
    //         std::vector<float> dense_center_line_score;
    //         {
    //             std::vector<Eigen::Vector2f> sparse_line;
    //             if (one_instance["data"].contains("points") && one_instance["data"]["points"].is_array() && !one_instance["data"]["points"].empty())
    //             {
    //                 for (const auto &one_point : one_instance["data"]["points"])
    //                 {
    //                     sparse_line.emplace_back(one_point[0], one_point[1]);
    //                 }
    //             }
    //             else if (one_instance["data"].contains("3dbox") && one_instance["data"]["3dbox"].is_array() && !one_instance["data"]["3dbox"].empty())
    //             {
    //                 sparse_line.emplace_back(one_instance["data"]["3dbox"][0][0], one_instance["data"]["3dbox"][0][1]);
    //                 sparse_line.emplace_back(one_instance["data"]["3dbox"][2][0], one_instance["data"]["3dbox"][2][1]);
    //                 sparse_line.emplace_back(one_instance["data"]["3dbox"][6][0], one_instance["data"]["3dbox"][6][1]);
    //                 sparse_line.emplace_back(one_instance["data"]["3dbox"][4][0], one_instance["data"]["3dbox"][4][1]);
    //             }
    //             else
    //             {
    //                 LOG_WARN("缺少有效字段");
    //             }

    //             std::vector<Eigen::Vector2f> dense_line_0 = densify_2d_line(sparse_line, 0.05);
    //             if (label_1 == 3 || label_1 == 4 || label_1 == 5 || label_2 == 3) // 如果是人行道/箭头/路口区域，填充内部
    //             {
    //                 dense_line = fill_the_crosswalk(dense_line_0);
    //             }
    //             else // 如果是线类，加粗两侧
    //             {
    //                 for (auto &p : dense_line_0)
    //                     for (int dx = -1; dx <= 1; dx++)
    //                         for (int dy = -1; dy <= 1; dy++)
    //                             dense_line.push_back(p + 0.05 * Eigen::Vector2f(dx, dy));
    //             }
    //             if (center_line == 1) // 如果是center line类，加粗两侧
    //             {
    //                 for (auto &p : dense_line_0)
    //                     for (int dx = -5; dx <= 5; dx++)
    //                         for (int dy = -5; dy <= 5; dy++)
    //                         {
    //                             dense_center_line.push_back(p + 0.05 * Eigen::Vector2f(dx, dy));
    //                             dense_center_line_score.push_back(exp(-0.001 * (dx * dx + dy * dy)));
    //                         }
    //             }
    //         }

    //         // 更新dict
    //         for (auto &p : dense_line)
    //         {
    //             Eigen::Matrix<float, 7, 1> cloud_bev_label;
    //             cloud_bev_label[0] = label_0;
    //             cloud_bev_label[1] = label_1;
    //             cloud_bev_label[2] = label_2;
    //             cloud_bev_label[3] = color;
    //             cloud_bev_label[4] = shape;
    //             cloud_bev_label[5] = center_line;
    //             cloud_bev_label[6] = traffic_light;
    //             auto key = position_to_index2d(p);
    //             if (cloud_bev_label_dict_ground.find(key) == cloud_bev_label_dict_ground.end())
    //             {
    //                 cloud_bev_label_dict_ground[key] = cloud_bev_label;
    //             }
    //             if (cloud_bev_label_dict_ground[key][0] == 0)
    //             {
    //                 cloud_bev_label_dict_ground[key][0] = cloud_bev_label[0];
    //             }
    //             if (cloud_bev_label_dict_ground[key][1] == 0)
    //             {
    //                 cloud_bev_label_dict_ground[key][1] = cloud_bev_label[1];
    //             }
    //             if (cloud_bev_label_dict_ground[key][2] == 0)
    //             {
    //                 cloud_bev_label_dict_ground[key][2] = cloud_bev_label[2];
    //             }
    //             if (cloud_bev_label_dict_ground[key][3] == 0)
    //             {
    //                 cloud_bev_label_dict_ground[key][3] = cloud_bev_label[3];
    //             }
    //             if (cloud_bev_label_dict_ground[key][4] == 0)
    //             {
    //                 cloud_bev_label_dict_ground[key][4] = cloud_bev_label[4];
    //             }
    //             if (cloud_bev_label_dict_ground[key][5] == 0)
    //             {
    //                 cloud_bev_label_dict_ground[key][5] = cloud_bev_label[5];
    //             }
    //             if (cloud_bev_label_dict_ground[key][6] == 0)
    //             {
    //                 cloud_bev_label_dict_ground[key][6] = cloud_bev_label[6];
    //             }
    //         }

    //         if (dense_center_line_score.size() > 0)
    //         {
    //             int n = dense_center_line_score.size();
    //             for (int i = 0; i < n; i++)
    //             {
    //                 Eigen::Vector2f p = dense_center_line[i];
    //                 auto key = position_to_index2d(p);
    //                 float score = dense_center_line_score[i];
    //                 if (center_line_score_dict.find(key) == center_line_score_dict.end())
    //                 {
    //                     center_line_score_dict[key] = score;
    //                 }
    //                 else if (center_line_score_dict[key] < score)
    //                 {
    //                     center_line_score_dict[key] = score;
    //                 }
    //             }
    //         }
    //     }
    // }

    // void add_bev_label_air(std::string bev_label_path,
    //                        phmap::flat_hash_map<Eigen::Vector3i, Eigen::Matrix<float, 1, 1>, Point3DHash> &cloud_bev_label_dict_air)
    // {
    //     nlohmann::json j = nlohmann::json::parse(std::ifstream(bev_label_path));
    //     std::string cat_version = j.contains("cat_version") ? j["cat_version"] : "v1_37";

    //     for (auto &one_instance : j["detail"]["instance_list"]) // 遍历每个BEV实体
    //     {
    //         if (!one_instance["data"].contains("3dbox") || one_instance["data"]["3dbox"].empty()) // 如果是 3dbox 类
    //         {
    //             continue;
    //         }

    //         int raw_bev_id = one_instance["attrs"]["type"]; // 原始 bev label id

    //         int label_0 = 0;
    //         int label_1 = 0;
    //         int label_2 = 0;
    //         int color = 0;
    //         int shape = 0;
    //         int center_line = 0;
    //         int traffic_light = 0;
    //         bev_label_map(cat_version, raw_bev_id, label_0, label_1, label_2, color, shape, center_line, traffic_light);

    //         std::vector<Eigen::Vector3f> dense_points;
    //         if (traffic_light >= 0) // 如果是空中 3dbox (为mmt数据修改成>=0，原本是大于0)
    //         {
    //             float min_x = one_instance["data"]["3dbox"][0][0];
    //             float min_y = one_instance["data"]["3dbox"][0][1];
    //             float min_z = one_instance["data"]["3dbox"][0][2];
    //             float max_x = one_instance["data"]["3dbox"][7][0];
    //             float max_y = one_instance["data"]["3dbox"][7][1];
    //             float max_z = one_instance["data"]["3dbox"][7][2];
    //             for (float x = min_x; x <= max_x; x += 0.05)
    //             {
    //                 for (float y = min_y; y <= max_y; y += 0.05)
    //                 {
    //                     for (float z = min_z; z <= max_z; z += 0.05)
    //                     {
    //                         dense_points.emplace_back(x, y, z);
    //                     }
    //                 }
    //             }
    //         }

    //         // 更新dict
    //         for (auto &p : dense_points)
    //         {
    //             if (p.x() > veh_max_x || p.x() < veh_min_x || p.y() > veh_max_y || p.y() < veh_min_y || p.z() > veh_max_z || p.z() < veh_min_z)
    //             {
    //                 continue;
    //             }

    //             auto key = position_to_index3d(p);
    //             if (cloud_bev_label_dict_air.find(key) == cloud_bev_label_dict_air.end())
    //             {
    //                 cloud_bev_label_dict_air[key] = Eigen::Matrix<float, 1, 1>::Zero();
    //             }
    //             if (cloud_bev_label_dict_air[key][0] == 0)
    //             {
    //                 cloud_bev_label_dict_air[key][0] = traffic_light;
    //             }
    //         }
    //     }
    // }

    // void make_one_flat_scan(Frame &frame)
    // {
    //     if (!frame.has_bev_label) // 强度处理
    //     {
    //         return;
    //     }
    //     phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<float, 7, 1>, Point2DHash> cloud_bev_label_dict_ground; // label-old, label1, label2, color, shape
    //     phmap::flat_hash_map<Eigen::Vector3i, Eigen::Matrix<float, 1, 1>, Point3DHash> cloud_bev_label_dict_air;    // traffic_light
    //     phmap::flat_hash_map<Eigen::Vector2i, float, Point2DHash> center_line_score_dict;                           //

    //     add_bev_label_ground(frame.bev_label_path, cloud_bev_label_dict_ground, center_line_score_dict);
    //     add_bev_label_air(frame.bev_label_path, cloud_bev_label_dict_air);

    //     // 生成平面BEV点云
    //     pcl::PointCloud<RenderPointType>::Ptr flat_scan_ptr(new pcl::PointCloud<RenderPointType>);
    //     for (float x = veh_min_x; x <= veh_max_x; x += pixel_size)
    //     {
    //         for (float y = veh_min_y; y <= veh_max_y; y += pixel_size)
    //         {
    //             Eigen::Vector2f position_xy(x, y);
    //             Eigen::Vector2i index_xy = position_to_index2d(position_xy);
    //             RenderPointType p;
    //             if (cloud_bev_label_dict_ground.find(index_xy) == cloud_bev_label_dict_ground.end())
    //             {
    //                 p.cloud_bev_label = 0;
    //                 p.cloud_bev_label_1 = 0;
    //                 p.cloud_bev_label_2 = 0;
    //                 p.cloud_bev_color = 0;
    //                 p.cloud_bev_shape = 0;
    //                 p.cloud_bev_center_line = 0;
    //                 p.cloud_bev_traffic_light = 0;
    //             }
    //             else
    //             {
    //                 auto &cloud_bev_label = cloud_bev_label_dict_ground[index_xy];
    //                 p.cloud_bev_label = cloud_bev_label[0];
    //                 p.cloud_bev_label_1 = cloud_bev_label[1];
    //                 p.cloud_bev_label_2 = cloud_bev_label[2];
    //                 p.cloud_bev_color = cloud_bev_label[3];
    //                 p.cloud_bev_shape = cloud_bev_label[4];
    //                 p.cloud_bev_center_line = cloud_bev_label[5];
    //                 p.cloud_bev_traffic_light = cloud_bev_label[6];
    //             }
    //             if (center_line_score_dict.find(index_xy) == center_line_score_dict.end())
    //             {
    //                 p.cloud_bev_center_line_score = 0;
    //             }
    //             else
    //             {
    //                 p.cloud_bev_center_line_score = (center_line_score_dict[index_xy]) * 100;
    //             }
    //             p.x = x;
    //             p.y = y;
    //             p.z = -0.385;
    //             p.a = 3;
    //             if (frame.day_or_night == "night") // 夜间
    //             {
    //                 p.a += 0b01000000;
    //             }
    //             p.distance_x_cm = round(p.x * 100);
    //             p.distance_y_cm = round(fabs(p.y) * 100);
    //             flat_scan_ptr->push_back(p);
    //         }
    //     }

    //     for (auto &[k, v] : cloud_bev_label_dict_air)
    //     {
    //         Eigen::Vector3f p_xzy = index3d_to_position(k);
    //         RenderPointType p;
    //         p.x = p_xzy.x();
    //         p.y = p_xzy.y();
    //         p.z = p_xzy.z();
    //         p.a = 0;
    //         if (frame.day_or_night == "night") // 夜间
    //         {
    //             p.a += 0b01000000;
    //         }
    //         p.distance_x_cm = round(p.x * 100);
    //         p.distance_y_cm = round(fabs(p.y) * 100);
    //         p.cloud_bev_traffic_light = v[0];
    //         flat_scan_ptr->push_back(p);
    //     }

    //     // TODO: 过滤点云
    //     // pcl::PointCloud<RenderPointType>::Ptr filtered_scan_ptr(new pcl::PointCloud<RenderPointType>);
    //     // for (size_t i = 0; i < flat_scan_ptr->points.size(); ++i) {
    //     //     if (flat_scan_ptr->points[i].cloud_bev_label == 0 &&
    //     //         flat_scan_ptr->points[i].cloud_bev_label_1 == 0 &&
    //     //         flat_scan_ptr->points[i].cloud_bev_label_2 == 0 &&
    //     //         flat_scan_ptr->points[i].cloud_bev_center_line == 0 &&
    //     //         flat_scan_ptr->points[i].cloud_bev_traffic_light == 0) {
    //     //         continue; // 跳过该点
    //     //     }

    //     //     filtered_scan_ptr->points.push_back(flat_scan_ptr->points[i]);
    //     // }

    //     // if (filtered_scan_ptr->points.size() > 0)
    //     // {
    //     //     pcl::io::savePCDFileBinaryCompressed(path_join(output_folder, str_format("%s.pcd", std::to_string(frame.stamp_ms).c_str())), *filtered_scan_ptr);
    //     // }

    //     if (flat_scan_ptr->points.size() > 0)
    //     {
    //         pcl::io::savePCDFileBinaryCompressed(path_join(output_folder, str_format("%s.pcd", std::to_string(frame.stamp_ms).c_str())), *flat_scan_ptr);
    //     }
    // }

    void add_bev_label_ground_line(std::vector<Eigen::Vector2f> sparse_line, Eigen::Matrix<LabelLayerInfo, 5, 1> cloud_bev_label, 
        bool center_line_flag, bool arrow_flag,float extra_offset, 
        phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<LabelLayerInfo, 5, 1>, Point2DHash> &cloud_bev_label_dict, 
        phmap::flat_hash_map<Eigen::Vector2i, float, Point2DHash> &center_line_score_dict)
    {
        if (sparse_line.empty()) {
            LOG_WARN("sparse_line is empty, no ground line to add.");
            return;
        }

        // 获取加粗填充稠密线
        std::vector<Eigen::Vector2f> dense_line;
        std::vector<Eigen::Vector2f> dense_center_line;
        std::vector<float> dense_center_line_score;

        std::vector<Eigen::Vector2f> dense_line_0 = densify_2d_line(sparse_line, 0.05);
        if (arrow_flag) {
            dense_line = fill_the_crosswalk(dense_line_0);
        } else {
            for (auto &p : dense_line_0) {
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        dense_line.push_back(p + 0.05 * Eigen::Vector2f(dx, dy));
                    }
                }
            }
        }

        if (center_line_flag) {
            for (auto &p : dense_line_0){
                for (int dx = -5; dx <= 5; dx++) {
                    for (int dy = -5; dy <= 5; dy++) {
                        dense_center_line.push_back(p + 0.05 * Eigen::Vector2f(dx, dy));
                        dense_center_line_score.push_back(exp(-0.001 * (dx * dx + dy * dy)));
                    }
                }
            }
        }

        // 更新dict
        for (auto &p : dense_line) {
            auto key = position_to_index2d(p, extra_offset);
            auto &label = cloud_bev_label_dict[key];
            if (label[0].label == 0) label[0] = cloud_bev_label[0];
            if (label[1].label == 0) label[1] = cloud_bev_label[1];
            if (label[2].label == 0) label[2] = cloud_bev_label[2];
            if (label[3].label == 0) label[3] = cloud_bev_label[3];
            if (label[4].label == 0) label[4] = cloud_bev_label[4];
        }

        for (size_t i = 0; i < dense_center_line.size(); ++i) {
            auto key = position_to_index2d(dense_center_line[i], extra_offset);
            float score = dense_center_line_score[i];
            if (center_line_score_dict.find(key) == center_line_score_dict.end()) {
                center_line_score_dict[key] = score;
            } else if (center_line_score_dict[key] < score) {
                center_line_score_dict[key] = score;
            }
        }
    }

    void add_bev_label_ground_plane(std::vector<Eigen::Vector2f> sparse_line, Eigen::Matrix<LabelLayerInfo, 5, 1> cloud_bev_label, 
        float extra_offset, phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<LabelLayerInfo, 5, 1>, Point2DHash> &cloud_bev_label_dict)
    {
        if (sparse_line.empty()) {
            LOG_WARN("sparse_line is empty, no ground plane to add.");
            return;
        }

        std::vector<Eigen::Vector2f> dense_line_0 = densify_2d_line(sparse_line, 0.05);
        std::vector<Eigen::Vector2f> dense_line = fill_the_crosswalk(dense_line_0);

        for (const auto& p : dense_line) {
            auto key = position_to_index2d(p, extra_offset);
            auto& label = cloud_bev_label_dict[key];

            if (label[0].label == 0) label[0] = cloud_bev_label[0];
            if (label[1].label == 0) label[1] = cloud_bev_label[1];
            if (label[2].label == 0) label[2] = cloud_bev_label[2];
            if (label[3].label == 0) label[3] = cloud_bev_label[3];
            if (label[4].label == 0) label[4] = cloud_bev_label[4];
        }
    }

    void add_bev_label_ground_new(Frame &frame, const std::string &bev_label_path,
                              phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<LabelLayerInfo, 5, 1>, Point2DHash> &cloud_bev_label_dict_line,
                              phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<LabelLayerInfo, 5, 1>, Point2DHash> &cloud_bev_label_dict_plane,    
                              phmap::flat_hash_map<Eigen::Vector2i, float, Point2DHash> &center_line_score_dict)
    {
        std::ifstream ifs(bev_label_path);
        if (!ifs.is_open()) {
            throw std::runtime_error("无法打开文件: " + bev_label_path);
        }

        nlohmann::json j = nlohmann::json::parse(ifs);
        std::string cat_version = j.value("cat_version", "BYD_BEV");

        auto &instance_list = j["detail"]["instance_list"];
        for (int instance_id = 0; instance_id < instance_list.size(); ++instance_id)
        {
            auto &one_instance = instance_list[instance_id];
            std::vector<int> raw_bev_ids = one_instance["attrs"]["type"];
            float score = one_instance["attrs"]["score"];

            std::vector<Eigen::Vector2f> sparse_line;
            if (one_instance["data"].contains("points") && one_instance["data"]["points"].is_array() && !one_instance["data"]["points"].empty()) {
                for (const auto &one_point : one_instance["data"]["points"])
                {
                    sparse_line.emplace_back(one_point[0], one_point[1]);
                }
            } else if (one_instance["data"].contains("3dbox") && one_instance["data"]["3dbox"].is_array() && !one_instance["data"]["3dbox"].empty()) {
                sparse_line.emplace_back(one_instance["data"]["3dbox"][0][0], one_instance["data"]["3dbox"][0][1]);
                sparse_line.emplace_back(one_instance["data"]["3dbox"][2][0], one_instance["data"]["3dbox"][2][1]);
                sparse_line.emplace_back(one_instance["data"]["3dbox"][6][0], one_instance["data"]["3dbox"][6][1]);
                sparse_line.emplace_back(one_instance["data"]["3dbox"][4][0], one_instance["data"]["3dbox"][4][1]);
            } else {
                // LOG_WARN("缺少有效字段");
                continue;
            }

            LabelLayerInfo label_0, label_1, label_2, center_line, traffic_light;
            bev_label_map_new(cat_version, raw_bev_ids, label_0, label_1, label_2, center_line, traffic_light);

            Eigen::Matrix<LabelLayerInfo, 5, 1> cloud_bev_label;
            cloud_bev_label << label_0, label_1, label_2, center_line, traffic_light;
            // debug
            // if (cloud_bev_label[0].label != 0)
            //     cloud_bev_label[0].label += score*10;
            // if (cloud_bev_label[1].label != 0)
            //     cloud_bev_label[1].label += score*10;
            // if (cloud_bev_label[2].label != 0)
            //     cloud_bev_label[2].label += score*10;
            // if (cloud_bev_label[3].label != 0)
            //     cloud_bev_label[3].label += score*10;
            // if (cloud_bev_label[4].label != 0)
            //     cloud_bev_label[4].label += score*10;

            if (label_0.label == 3 || label_1.label == 5 || label_2.label == 3) // 如果是人行道/导流区/路口
            {
                if(1) {
                    add_bev_label_ground_plane(sparse_line, cloud_bev_label, 10.0, cloud_bev_label_dict_plane);
                } else {
                    // if (data_type_ == "MMT_RC") {
                    //     add_bev_label_ground_plane(sparse_line, cloud_bev_label, 10.0, cloud_bev_label_dict_plane);
                    // } else if (data_type_ == "BYD_BEV" || data_type_ == "BYD_LIDAR_B") {
                    //     add_bev_label_ground_plane(sparse_line, cloud_bev_label, 0.0, cloud_bev_label_dict_plane);
                    // }
                }
            } else {
                if (label_1.label == 1 || center_line.label == 1) // || label_0.label == 4 // 如果是车道线/中心线(/停止线)需要检查一下是否是好线
                {
                    if (std::find(frame.good_bev_line_id.begin(), frame.good_bev_line_id.end(), instance_id) == frame.good_bev_line_id.end())
                    {
                        continue;
                    }                    
                }

                bool center_line_flag = (center_line.label == 1);
                bool arrow_flag = (label_1.label == 4);
                add_bev_label_ground_line(sparse_line, cloud_bev_label, center_line_flag, arrow_flag, 0.0, cloud_bev_label_dict_line, center_line_score_dict);
            }
        }
    }

    void add_bev_label_air_new(std::string bev_label_path,
                           phmap::flat_hash_map<Eigen::Vector3i, Eigen::Matrix<LabelLayerInfo, 1, 1>, Point3DHash> &cloud_bev_label_dict_air)
    {
        nlohmann::json j = nlohmann::json::parse(std::ifstream(bev_label_path));
        std::string cat_version = j.contains("cat_version") ? j["cat_version"] : "v1_37";

        for (auto &one_instance : j["detail"]["instance_list"]) // 遍历每个BEV实体
        {
            if (!one_instance["data"].contains("3dbox") || one_instance["data"]["3dbox"].empty()) // 如果是 3dbox 类
            {
                continue;
            }
            std::vector<int> raw_bev_ids = one_instance["attrs"]["type"];

            LabelLayerInfo label_0, label_1, label_2, center_line, traffic_light;
            bev_label_map_new(cat_version, raw_bev_ids, label_0, label_1, label_2, center_line, traffic_light);

            std::vector<Eigen::Vector3f> dense_points;
            if (traffic_light.label > 0) // 如果是空中 3dbox
            {
                float min_x = one_instance["data"]["3dbox"][0][0];
                float min_y = one_instance["data"]["3dbox"][0][1];
                float min_z = one_instance["data"]["3dbox"][0][2];
                float max_x = one_instance["data"]["3dbox"][7][0];
                float max_y = one_instance["data"]["3dbox"][7][1];
                float max_z = one_instance["data"]["3dbox"][7][2];
                for (float x = min_x; x <= max_x; x += 0.05)
                {
                    for (float y = min_y; y <= max_y; y += 0.05)
                    {
                        for (float z = min_z; z <= max_z; z += 0.05)
                        {
                            dense_points.emplace_back(x, y, z);
                        }
                    }
                }
            }

            // 更新dict
            for (auto &p : dense_points)
            {
                if (p.x() > veh_max_x || p.x() < veh_min_x || p.y() > veh_max_y || p.y() < veh_min_y || p.z() > veh_max_z || p.z() < veh_min_z)
                {
                    continue;
                }

                auto key = position_to_index3d(p);
                if (cloud_bev_label_dict_air.find(key) == cloud_bev_label_dict_air.end())
                {
                    cloud_bev_label_dict_air[key] = Eigen::Matrix<LabelLayerInfo, 1, 1>();
                    cloud_bev_label_dict_air[key](0, 0) = LabelLayerInfo();
                }
                if (cloud_bev_label_dict_air[key][0].label == 0)
                {
                    cloud_bev_label_dict_air[key][0] = traffic_light;
                }
            }
        }
    }

    void generate_bev_scan(phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<LabelLayerInfo, 5, 1>, Point2DHash> &cloud_bev_label_dict, 
        phmap::flat_hash_map<Eigen::Vector2i, float, Point2DHash> *center_line_score_dict_ptr,
        float extra_offset, std::string day_or_night, bool line_flag,
        pcl::PointCloud<ModelPointTypeNew>::Ptr flat_scan_ptr,
        phmap::flat_hash_map<Eigen::Vector2i, int, Point2DHash> &point_index_map) 
    {
        if (!flat_scan_ptr) {
            throw std::invalid_argument("flat_scan_ptr cannot be nullptr");
        }
        float min_x = veh_min_x;
        float max_x = veh_max_x + extra_offset;
        float min_y = veh_min_y - extra_offset;
        float max_y = veh_max_y + extra_offset;

        for (float x = min_x; x <= max_x; x += pixel_size)
        {
            for (float y = min_y; y <= max_y; y += pixel_size)
            {
                Eigen::Vector2f position_xy(x, y);
                Eigen::Vector2i index_xy = position_to_index2d(position_xy, extra_offset);
                ModelPointTypeNew p;

                auto label_it = cloud_bev_label_dict.find(index_xy);
                if (label_it == cloud_bev_label_dict.end())
                {
                    if (line_flag){
                        p.cloud_bev_label = 0;
                        p.cloud_bev_label_shape = 0;
                        p.cloud_bev_label_color = 0;
                        p.cloud_bev_label_1 = 0;
                        p.cloud_bev_label_1_shape = 0;
                        p.cloud_bev_label_1_color = 0;
                        p.cloud_bev_label_2 = 0;
                        p.cloud_bev_label_2_shape = 0;
                        p.cloud_bev_label_2_color = 0;
                        p.cloud_bev_center_line = 0;
                        p.cloud_bev_center_line_shape = 0;
                        p.cloud_bev_center_line_score = 0;
                        // p.cloud_bev_traffic_light = 0;
                        // p.cloud_bev_traffic_light_shape = 0;
                        // p.cloud_bev_traffic_light_score = 0;
                    } else {
                        continue;
                    }                    
                }
                else
                {
                    const auto &cloud_bev_label = label_it->second;
                    p.cloud_bev_label = cloud_bev_label[0].label;
                    p.cloud_bev_label_shape = cloud_bev_label[0].shape;
                    p.cloud_bev_label_color = cloud_bev_label[0].color;
                    p.cloud_bev_label_1 = cloud_bev_label[1].label;
                    p.cloud_bev_label_1_shape = cloud_bev_label[1].shape;
                    p.cloud_bev_label_1_color = cloud_bev_label[1].color;                    
                    p.cloud_bev_label_2 = cloud_bev_label[2].label;
                    p.cloud_bev_label_2_shape = cloud_bev_label[2].shape;
                    p.cloud_bev_label_2_color = cloud_bev_label[2].color;
                    p.cloud_bev_center_line = cloud_bev_label[3].label;
                    p.cloud_bev_center_line_shape = cloud_bev_label[3].shape;
                    p.cloud_bev_center_line_color = cloud_bev_label[3].color;
                    // p.cloud_bev_traffic_light = cloud_bev_label[4].label;
                    // p.cloud_bev_traffic_light_shape = cloud_bev_label[4].shape;
                    // p.cloud_bev_traffic_light_score = cloud_bev_label[4].color;                     
                }

                if (center_line_score_dict_ptr)
                {
                    auto score_it = center_line_score_dict_ptr->find(index_xy);
                    if (score_it == center_line_score_dict_ptr->end())
                    {
                        p.cloud_bev_center_line_score = 0;
                    }
                    else
                    {
                        p.cloud_bev_center_line_score = score_it->second * 100;
                    }
                } else {
                    p.cloud_bev_center_line_score = 0;
                }

                p.x = x;
                p.y = y;
                // p.z = -0.385;
                p.z = 0;
                p.a = (day_or_night == "night") ? (3 + 0b01000000) : 3;
                // p.distance_x_cm = round(p.x * 100);
                // p.distance_y_cm = round(fabs(p.y) * 100);

                Eigen::Vector2i origin_index(round(p.x / pixel_size), round(p.y / pixel_size));
                auto point_it = point_index_map.find(origin_index);
                if (point_it == point_index_map.end())
                {
                    // 如果点不存在，则添加点并更新映射
                    flat_scan_ptr->push_back(p);
                    point_index_map[origin_index] = flat_scan_ptr->size() - 1;
                }
                else
                {
                    // 如果点已经存在，则更新该点的属性
                    ModelPointTypeNew &existing_point = (*flat_scan_ptr)[point_it->second];
                    existing_point.cloud_bev_label += p.cloud_bev_label;
                    existing_point.cloud_bev_label_shape += p.cloud_bev_label_shape;
                    existing_point.cloud_bev_label_color += p.cloud_bev_label_color;
                    existing_point.cloud_bev_label_1 += p.cloud_bev_label_1;
                    existing_point.cloud_bev_label_1_shape += p.cloud_bev_label_1_shape;
                    existing_point.cloud_bev_label_1_color += p.cloud_bev_label_1_color;
                    existing_point.cloud_bev_label_2 += p.cloud_bev_label_2;
                    existing_point.cloud_bev_label_2_shape += p.cloud_bev_label_2_shape;
                    existing_point.cloud_bev_label_2_color += p.cloud_bev_label_2_color;
                    existing_point.cloud_bev_center_line += p.cloud_bev_center_line;
                    existing_point.cloud_bev_center_line_shape += p.cloud_bev_center_line_shape;
                    existing_point.cloud_bev_center_line_score += p.cloud_bev_center_line_score;
                }
            }
        }
    }

    void make_one_flat_scan_new(Frame &frame)
    {
        if (!frame.has_bev_label)
        {
            return;
        }

        phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<LabelLayerInfo, 5, 1>, Point2DHash> cloud_bev_label_dict_line; // 车道线，中心线等线类
        phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<LabelLayerInfo, 5, 1>, Point2DHash> cloud_bev_label_dict_plane; // 人行横道，箭头，导流区，路口等面类
        // phmap::flat_hash_map<Eigen::Vector3i, Eigen::Matrix<LabelLayerInfo, 1, 1>, Point3DHash> cloud_bev_label_dict_air; // traffic_light
        phmap::flat_hash_map<Eigen::Vector2i, float, Point2DHash> center_line_score_dict; 

        add_bev_label_ground_new(frame, frame.bev_label_path, cloud_bev_label_dict_line, cloud_bev_label_dict_plane, center_line_score_dict);
        // add_bev_label_air_new(frame.bev_label_path, cloud_bev_label_dict_air);

        // 生成平面BEV点云
        pcl::PointCloud<ModelPointTypeNew>::Ptr flat_scan_ptr(new pcl::PointCloud<ModelPointTypeNew>);
        phmap::flat_hash_map<Eigen::Vector2i, int, Point2DHash> point_index_map;


        generate_bev_scan(cloud_bev_label_dict_line, &center_line_score_dict, 0.0, frame.day_or_night, true, flat_scan_ptr, point_index_map);
        if(1) {
           generate_bev_scan(cloud_bev_label_dict_plane, NULL, 10.0, frame.day_or_night, false, flat_scan_ptr, point_index_map);
        } else {
            // if (data_type_ == "MMT_RC") {
            //     generate_bev_scan(cloud_bev_label_dict_plane, NULL, 10.0, frame.day_or_night, false, flat_scan_ptr, point_index_map);
            // } else if (data_type_ == "BYD_BEV" || data_type_ == "BYD_LIDAR_B") {
            //     generate_bev_scan(cloud_bev_label_dict_plane, NULL, 10.0, frame.day_or_night, false, flat_scan_ptr, point_index_map);
            // }
        }

        
        

        // for (auto &[k, v] : cloud_bev_label_dict_air)
        // {
        //     Eigen::Vector3f p_xzy = index3d_to_position(k);
        //     ModelPointTypeNew p;
        //     p.x = p_xzy.x();
        //     p.y = p_xzy.y();
        //     p.z = p_xzy.z();
        //     p.a = 0;
        //     if (frame.day_or_night == "night") // 夜间
        //     {
        //         p.a += 0b01000000;
        //     }
        //     // p.distance_x_cm = round(p.x * 100);
        //     // p.distance_y_cm = round(fabs(p.y) * 100);
        //     p.cloud_bev_traffic_light = v[0].label;
        //     p.cloud_bev_traffic_light_shape = v[0].shape;
        //     p.cloud_bev_traffic_light_score = v[0].color; 
        //     flat_scan_ptr->push_back(p);
        // }

        if (flat_scan_ptr->points.size() > 0)
        {
            pcl::io::savePCDFileBinaryCompressed(path_join(output_folder, str_format("%s.pcd", std::to_string(frame.stamp_ms).c_str())), *flat_scan_ptr);
        }
    }


    void run_process()
    {
        LOG_INFO("启动 flat scan maker");
        int finish_p = 0;
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
// #pragma omp parallel for num_threads(int(std::thread::hardware_concurrency()))
        for (auto &one_frame : frame_list)
        {
            finish_p++;
            if (finish_p % 50 == 0)
            {
                LOG_INFO("flat scan maker 进度{}/{}", finish_p, frame_list.size());
            }
            // make_one_flat_scan(one_frame);
            make_one_flat_scan_new(one_frame);
        }
    }
};

int main(int argc, char **argv)
{
    std::cout << "===Hello flat scan maker ===" << std::endl;
    ArgParser arg_parser;
    arg_parser.add<std::string>("data_set_path", '\0', "输入 txt path", true, "");
    arg_parser.add<std::string>("output_folder", '\0', "输出文件夹", true, "");
    arg_parser.parse_check(argc, argv);

    System system;
    system.data_set_path = arg_parser.get<std::string>("data_set_path");
    system.output_folder = arg_parser.get<std::string>("output_folder");

    auto logger = Logger::instance();
    logger->setLogLevel(spdlog::level::level_enum::info, spdlog::level::level_enum::trace);
    logger->setup(system.output_folder + "/log/" + _time_str_s() + "_flat_scan_maker.log", "flat_scan_maker");

    LOG_INFO("============================================");
    LOG_INFO("  Hello flat_scan_maker");
    LOG_INFO("  data_set_path:{}", system.data_set_path);
    LOG_INFO("  output_folder:{}", system.output_folder);
    LOG_INFO("============================================");

    system.read_json();
    system.init_process();
    system.run_process();

    LOG_INFO("完成 flat_scan_maker");
    return 0;
}

// /home/test/code/lidar-mapping/pointcloudrender/bin/pointcloud_render --data_set_path=/home/test/lidar_mapping_ws/fs_31974/data/5431676614279/stage_1_data_set.json --output_folder=/home/test/lidar_mapping_ws/fs_31974/data/5431676614279
// /home/test/code/fsd_map_road_model_dev/preprocess/pointcloudrender/bin/scan_render --data_set_path=/home/test/lidar_mapping_ws/26363/data/6171673476216/stage_1_data_set.json --output_folder=/home/test/lidar_mapping_ws/26363/data/6171673476216
