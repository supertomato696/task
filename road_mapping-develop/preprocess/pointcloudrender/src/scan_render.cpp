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
#include "include/parse_seg.h"
#include "include/json.h"
#include "include/types.h"
#include "include/mulls/pca_feature.h"
#include "parallel_hashmap/phmap.h"
#include "bev_label_map.h"

struct EIGEN_ALIGN16 RawPointType
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(RawPointType,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, time, time))
// clang-format on

struct EIGEN_ALIGN16 RenderPointType
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB; // a: bit_0: 感知当前路面, bit_1: RANSAC 路面, bit_2 PCA 平面, bit_3 PCA 直线, bit_6 night, bit_7 bad_point
    PCL_ADD_INTENSITY;
    uint16_t ring;
    uint16_t label;
    uint16_t distance_x_cm;
    uint16_t distance_y_cm;
    uint8_t geom_type; // bit_0: 感知当前路面, bit_1: RANSAC 路面, bit_2 PCA 平面, bit_3 PCA 直线, bit_6 night, bit_7 bad_point
    uint8_t cloud_pano_seg;
    uint8_t cloud_line_seg;
    uint8_t cloud_bev_label;
    uint8_t cloud_bev_label_1;
    uint8_t cloud_bev_label_2;
    uint8_t cloud_bev_color;
    uint8_t cloud_bev_shape;
    uint8_t cloud_bev_center_line;
    uint8_t cloud_bev_traffic_light;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(RenderPointType,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label)
                                  (uint16_t, distance_x_cm, distance_x_cm)
                                  (uint16_t, distance_y_cm, distance_y_cm)
                                  (uint8_t, geom_type, geom_type)
                                  (uint8_t, cloud_pano_seg, cloud_pano_seg)
                                  (uint8_t, cloud_line_seg, cloud_line_seg)
                                  (uint8_t, cloud_bev_label, cloud_bev_label)
                                  (uint8_t, cloud_bev_label_1, cloud_bev_label_1)
                                  (uint8_t, cloud_bev_label_2, cloud_bev_label_2)
                                  (uint8_t, cloud_bev_color, cloud_bev_color)
                                  (uint8_t, cloud_bev_shape, cloud_bev_shape)
                                  (uint8_t, cloud_bev_center_line, cloud_bev_center_line)
                                  (uint8_t, cloud_bev_traffic_light, cloud_bev_traffic_light))
// clang-format on

class Frame
{
public:
    Frame() {}
    ~Frame() {}

    uint64_t stamp_ms = 0;
    std::string scan_path = "";
    std::string front_image_path = "";
    std::string perception_data_path = ""; // 车端前视全景分割
    std::string cloud_line_seg_path = "";  // 云端前视车道线分割
    std::string cloud_pano_seg_path = "";  // 云端前视全景分割
    std::string bev_label_path = "";

    bool has_front_image = false;
    bool has_car_pano_seg = false;
    bool has_cloud_pano_seg = false;
    bool has_cloud_line_seg = false;
    bool has_bev_label = false;
};

class System
{
public:
    System() {}
    ~System() {}

    int ins_bad_cnt = 0;

    bool is_new_firmware = false;
    bool is_day = true;

    std::string data_set_path;
    std::string output_folder;
    std::string lidar_rgb_folder;
    std::string veh_scan_folder;
    std::string raw_seg_image_folder;
    std::string un_seg_image_folder;
    std::string un_rgb_image_folder;
    std::string un_cloud_front_line_folder;
    std::string un_cloud_pano_seg_folder;

    std::string trail_id;
    Eigen::Quaterniond q_veh_lidar;
    Eigen::Vector3d t_veh_lidar;
    Eigen::Matrix4d T_veh_lidar;
    Eigen::Matrix4d T_lidar_veh;

    Eigen::Quaterniond q_veh_cam;
    Eigen::Vector3d t_veh_cam;
    Eigen::Matrix4d T_veh_cam;
    Eigen::Matrix4d T_cam_veh;
    Eigen::Matrix4d T_cam_lidar;
    Eigen::Matrix3d raw_cam_K;
    std::vector<double> cam_k1234;
    std::vector<double> cam_k12p12k3456;
    std::string cam_dist_type;
    int raw_cam_width;
    int raw_cam_height;
    float x_max;
    float y_min;
    float y_max;
    float z_max;

    // 鱼眼相机的去畸变
    cv::Mat undist_map_1, undist_map_2;
    cv::Mat new_cam_K;
    int new_cam_width;
    int new_cam_height;
    std::vector<Frame> frame_list;

    void read_json()
    {
        nlohmann::json ds_json;
        std::ifstream ifs(data_set_path);
        ifs >> ds_json;
        trail_id = ds_json["header"]["trail_id"];
        // lidar 外参
        {
            t_veh_lidar = ds_json["header"]["t_veh_lidar"].get<Eigen::Vector3d>();
            q_veh_lidar = ds_json["header"]["q_veh_lidar"].get<Eigen::Quaterniond>();
            T_veh_lidar = qab_tab_to_Tab(q_veh_lidar, t_veh_lidar);
            T_lidar_veh = T_veh_lidar.inverse();
        }
        // 相机外参
        {
            t_veh_cam = ds_json["header"]["t_veh_cam"].get<Eigen::Vector3d>();
            q_veh_cam = ds_json["header"]["q_veh_cam"].get<Eigen::Quaterniond>();
            T_veh_cam = qab_tab_to_Tab(q_veh_cam, t_veh_cam);
            T_cam_veh = T_veh_cam.inverse();
            T_cam_lidar = T_veh_cam.inverse() * T_veh_lidar;
        }
        // 相机内参
        {
            if (ds_json["header"]["cam_dist_type"] == "equidistant")
            {
                cam_dist_type = ds_json["header"]["cam_dist_type"];
                for (int i = 0; i < 4; i++)
                {
                    cam_k1234.push_back(ds_json["header"]["cam_k1234"][i]);
                }
            }
            if (ds_json["header"]["cam_dist_type"] == "radtan")
            {
                cam_dist_type = ds_json["header"]["cam_dist_type"];
                for (int i = 0; i < 8; i++)
                {
                    cam_k12p12k3456.push_back(ds_json["header"]["cam_k12p12k3456"][i]);
                }
            }
            raw_cam_height = ds_json["header"]["cam_height"];
            raw_cam_width = ds_json["header"]["cam_width"];
            raw_cam_K << ds_json["header"]["cam_fxycxy"][0], 0, ds_json["header"]["cam_fxycxy"][2],
                0, ds_json["header"]["cam_fxycxy"][1], ds_json["header"]["cam_fxycxy"][3],
                0, 0, 1;
        }
        // day or night
        {
            for (auto &one : ds_json["header"]["tag"])
            {
                if (one == "day")
                {
                    is_day = true;
                    break;
                }
                if (one == "night")
                {
                    is_day = false;
                    break;
                }
            }
        }
        // frame信息
        for (auto one : ds_json["data"])
        {
            Frame frame;
            frame.stamp_ms = one["stamp_ms"];
            frame.scan_path = one["scan_path"];
            frame.front_image_path = one["front_image_path"];
            frame.bev_label_path = one["bev_label_path"];
            frame.perception_data_path = one["perception_data_path"];
            frame.cloud_pano_seg_path = one["cloud_pano_seg_path"];
            frame.cloud_line_seg_path = one["cloud_line_seg_path"];

            frame.has_front_image = !frame.front_image_path.empty();
            frame.has_car_pano_seg = !frame.perception_data_path.empty();
            frame.has_cloud_pano_seg = !frame.cloud_pano_seg_path.empty();
            frame.has_cloud_line_seg = !frame.cloud_line_seg_path.empty();
            frame.has_bev_label = !frame.bev_label_path.empty();

            frame_list.push_back(frame);

            if (one["ins_position_type"] != "NARROW_INT")
                ins_bad_cnt++;
        }
        LOG_INFO("完成读入 trail id:, {}, 帧数: {}", trail_id, frame_list.size());
    }

    bool check_new_firmware_(const pcl::PointCloud<RawPointType> &pc)
    {
        std::set<int> s;
        for (const auto &p : pc.points)
        {
            int offset = static_cast<int>(p.time);
            if (s.count(offset) == 0)
                s.insert(offset);
        }

        std::vector<int> v(s.begin(), s.end());
        for (size_t i = 1; i < v.size(); i++)
        {
            if (v[i] - v[i - 1] == 1)
            {
                return true; // 新固件在原有的时间戳上+1
            }
        }
        return false;
    }

    void init_process()
    {
        lidar_rgb_folder = add_folder_if_not_exist(path_join(output_folder, "lidar_rgb"));
        veh_scan_folder = add_folder_if_not_exist(path_join(output_folder, "veh_scan"));
        raw_seg_image_folder = add_folder_if_not_exist(path_join(output_folder, "image_seg"));
        un_seg_image_folder = add_folder_if_not_exist(path_join(output_folder, "un_image_seg"));
        un_rgb_image_folder = add_folder_if_not_exist(path_join(output_folder, "un_image"));
        un_cloud_front_line_folder = add_folder_if_not_exist(path_join(output_folder, "un_cloud_front_line"));
        un_cloud_pano_seg_folder = add_folder_if_not_exist(path_join(output_folder, "un_cloud_pano_seg"));
        add_folder_if_not_exist(path_join(output_folder, "bev_flat_scan"));

        if (!frame_list.empty())
        {
            auto it = frame_list.begin();
            const std::string scan_path = it->scan_path;
            pcl::PointCloud<RawPointType>::Ptr in_ptr(new pcl::PointCloud<RawPointType>);
            pcl::io::loadPCDFile(scan_path, *in_ptr);
            is_new_firmware = check_new_firmware_(*in_ptr);
        }
        LOG_INFO("激光雷达新固件? {}", is_new_firmware);

        cv::Size raw_img_size(raw_cam_width, raw_cam_height);
        cv::Mat raw_cam_K_cvmat;
        cv::eigen2cv(raw_cam_K, raw_cam_K_cvmat);

        new_cam_width = 1920;
        new_cam_height = 530;
        new_cam_K = (cv::Mat_<double>(3, 3) << 560, 0, 960, 0, 560, 360, 0, 0, 1);

        if (cam_dist_type == "equidistant")
        {
            cv::Mat distort_param = (cv::Mat_<double>(1, 4) << cam_k1234[0], cam_k1234[1], cam_k1234[2], cam_k1234[3]);
            cv::fisheye::initUndistortRectifyMap(raw_cam_K_cvmat, distort_param, cv::Matx33d::eye(),
                                                 new_cam_K, cv::Size(new_cam_width, new_cam_height),
                                                 CV_16SC2, undist_map_1, undist_map_2);
        }
        else if (cam_dist_type == "radtan")
        {
            cv::Mat distort_param = (cv::Mat_<double>(1, 8) << cam_k12p12k3456[0], cam_k12p12k3456[1], cam_k12p12k3456[2], cam_k12p12k3456[3],
                                     cam_k12p12k3456[4], cam_k12p12k3456[5], cam_k12p12k3456[6], cam_k12p12k3456[7]);
            cv::initUndistortRectifyMap(raw_cam_K_cvmat, distort_param, cv::Matx33d::eye(),
                                        new_cam_K, cv::Size(new_cam_width, new_cam_height),
                                        CV_16SC2, undist_map_1, undist_map_2);
        }
        else
        {
            LOG_ERROR("未知的相机畸变类型");
        }
        LOG_INFO("new_img_size: {} {}", new_cam_width, new_cam_height);

        {
            LOG_INFO("输出去畸变后图片内参");
            nlohmann::json un_image_param_json;
            un_image_param_json["width"] = new_cam_width;
            un_image_param_json["height"] = new_cam_height;
            un_image_param_json["fxycxy"][0] = new_cam_K.at<double>(0, 0);
            un_image_param_json["fxycxy"][1] = new_cam_K.at<double>(1, 1);
            un_image_param_json["fxycxy"][2] = new_cam_K.at<double>(0, 2);
            un_image_param_json["fxycxy"][3] = new_cam_K.at<double>(1, 2);
            std::ofstream o(path_join(output_folder, "un_image_param.json"));
            o << un_image_param_json;
        }

        {
            // todo：INS异常率 不应在此环节分析
            LOG_INFO("输出固件信息及INS异常率");
            nlohmann::json firmware_ins_param_json;
            firmware_ins_param_json["firmware"] = is_new_firmware;
            firmware_ins_param_json["bad_ins"] = ins_bad_cnt;
            firmware_ins_param_json["all_ins"] = frame_list.size();
            std::ofstream o(path_join(output_folder, "firmware_ins_param_json.json"));
            o << firmware_ins_param_json;
        }
    }

    cv::Mat keep_max_region(cv::Mat &bin_img)
    {
        // 输入输出 单通道 前景白色 背景黑色
        cv::Mat stats, centroids, labelImage, imshow_mat;
        int nLabels = cv::connectedComponentsWithStats(bin_img, labelImage, stats, centroids, 8);

        std::map<int, int> label_list;
        for (int y = new_cam_height * 0.7; y <= new_cam_height; y++)
        {
            for (int x = new_cam_width * 0.25; x <= new_cam_width * 0.75; x++)
            {
                int label_idx = labelImage.at<int>(y, x);
                if (label_idx == 0)
                {
                    continue;
                }
                if (label_list.count(label_idx))
                {
                    label_list[label_idx] += 1;
                }
                else
                {
                    label_list.emplace(label_idx, 1);
                }
            }
        }

        int max_area = 0;
        int max_idx = 0;
        for (auto &label_item : label_list)
        {
            if (label_item.second > max_area)
            {
                max_area = label_item.second;
                max_idx = label_item.first;
            }
        }

        cv::Mat out(bin_img.size(), CV_8UC1, cv::Scalar(0));
        out.setTo(255, (labelImage == max_idx));
        return out;
    }

    void proj_rgb_and_seg(Frame &frame,
                          pcl::PointCloud<RawPointType>::Ptr input_scan_ptr,
                          pcl::PointCloud<RenderPointType>::Ptr output_scan_ptr,
                          cv::Mat &un_rgb_image,
                          cv::Mat &un_car_pano_seg,
                          cv::Mat &un_cloud_pano_seg,
                          cv::Mat &un_cloud_front_line)
    {
        cv::Mat moving_obj_mask = cv::Mat::zeros(new_cam_height, new_cam_width, CV_8UC1); // 动态mask
        {
            if (frame.has_cloud_pano_seg)
            {
                moving_obj_mask.setTo(255, (un_cloud_pano_seg == 70));
                moving_obj_mask.setTo(255, (un_cloud_pano_seg == 80));
                moving_obj_mask.setTo(255, (un_cloud_pano_seg == 90));
                moving_obj_mask.setTo(255, (un_cloud_pano_seg == 100));
                moving_obj_mask.setTo(255, (un_cloud_pano_seg == 110));
                moving_obj_mask.setTo(255, (un_cloud_pano_seg == 120));
                moving_obj_mask.setTo(255, (un_cloud_pano_seg == 130));
                moving_obj_mask.setTo(255, (un_cloud_pano_seg == 140));
            }
            if (frame.has_car_pano_seg)
            {
                moving_obj_mask.setTo(255, (un_car_pano_seg == 50));
                moving_obj_mask.setTo(255, (un_car_pano_seg == 60));
                moving_obj_mask.setTo(255, (un_car_pano_seg == 70));
            }
            if (frame.has_cloud_pano_seg || frame.has_car_pano_seg)
            {
                cv::dilate(moving_obj_mask, moving_obj_mask, getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11)));
            }
        }

        cv::Mat percetion_road_mask = cv::Mat::zeros(new_cam_height, new_cam_width, CV_8UC1); // 路面 mask
        {
            if (frame.has_cloud_pano_seg)
            {
                percetion_road_mask.setTo(255, (un_cloud_pano_seg == 0));
                percetion_road_mask.setTo(255, (un_cloud_pano_seg == 10));
                percetion_road_mask.setTo(255, (un_cloud_pano_seg == 150));
                percetion_road_mask.setTo(255, (un_cloud_pano_seg == 160));
                percetion_road_mask.setTo(255, (un_cloud_pano_seg == 170));
                percetion_road_mask.setTo(255, (un_cloud_pano_seg == 180));
                percetion_road_mask.setTo(255, (un_cloud_pano_seg == 190));
                percetion_road_mask.setTo(255, (un_cloud_pano_seg == 210));
            }
            else if (frame.has_car_pano_seg)
            {
                percetion_road_mask.setTo(255, (un_car_pano_seg == 0));
                percetion_road_mask.setTo(255, (un_car_pano_seg == 80));
                percetion_road_mask.setTo(255, (un_car_pano_seg == 90));
                percetion_road_mask.setTo(255, (un_car_pano_seg == 100));
                percetion_road_mask.setTo(255, (un_car_pano_seg == 110));
                percetion_road_mask.setTo(255, (un_car_pano_seg == 120));
                percetion_road_mask.setTo(255, (un_car_pano_seg == 140));
                percetion_road_mask.setTo(255, (un_car_pano_seg == 150));
            }
            if (frame.has_cloud_pano_seg || frame.has_car_pano_seg)
            {
                for (int y = new_cam_height - 10; y > 370; y -= 10) // 380 约为地平线的高度
                {
                    cv::Mat small(percetion_road_mask, cv::Rect(0, y, new_cam_width, 10));
                    int size = ceil((y - 380) * 0.2 + 0.01);
                    cv::dilate(small, small, getStructuringElement(cv::MORPH_RECT, cv::Size(size, 1)));
                }
                percetion_road_mask = keep_max_region(percetion_road_mask);
            }
        }

        // // 腐蚀车道线
        // {
        //     cv::Mat src = un_car_pano_seg.clone();
        //     for (int y = 1; y < src.rows - 1; y++)
        //     {
        //         for (int x = 1; x < src.cols - 1; x++)
        //         {
        //             if ((src.at<uchar>(y, x) >= 80 && src.at<uchar>(y, x) <= 120) && (src.at<uchar>(y - 1, x) == 0 || src.at<uchar>(y + 1, x) == 0 || src.at<uchar>(y, x - 1) == 0 || src.at<uchar>(y, x + 1) == 0))
        //             {
        //                 un_car_pano_seg.at<uchar>(y, x) = 0;
        //             }
        //         }
        //     }
        // }
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.5))

        // 投影
        for (auto &p : input_scan_ptr->points)
        {
            Eigen::Vector4d t_cam_p = T_cam_veh * Eigen::Vector4d(p.x, p.y, p.z, 1);
            int u = round(t_cam_p[0] / t_cam_p[2] * new_cam_K.at<double>(0, 0) + new_cam_K.at<double>(0, 2));
            int v = round(t_cam_p[1] / t_cam_p[2] * new_cam_K.at<double>(1, 1) + new_cam_K.at<double>(1, 2));

            if (u < 0 || u >= new_cam_width || v < 0 || v >= new_cam_height)
            {
                continue;
            }
            if (moving_obj_mask.at<uint8_t>(v, u) == 255)
            {
                continue;
            }

            RenderPointType color_p;
            color_p.x = p.x;
            color_p.y = p.y;
            color_p.z = p.z;
            color_p.ring = p.ring;
            color_p.intensity = p.intensity;
            color_p.distance_x_cm = round(p.x * 100);
            color_p.distance_y_cm = round(fabs(p.y) * 100);
            color_p.a = 0;
            color_p.geom_type = 0;
            if (static_cast<int>(p.time) & 0x01) // 判断最后一位的奇偶性
            {
                color_p.a |= 0b10000000; // bad point
            }
            if (is_day == false)
            {
                color_p.a |= 0b01000000; // night
            }

            cv::Vec3b bgr = un_rgb_image.at<cv::Vec3b>(v, u);
            color_p.r = bgr[2];
            color_p.g = bgr[1];
            color_p.b = bgr[0];
            color_p.label = un_car_pano_seg.at<uint8_t>(v, u);
            color_p.cloud_pano_seg = un_cloud_pano_seg.at<uint8_t>(v, u) / 10;
            color_p.cloud_line_seg = un_cloud_front_line.at<uint8_t>(v, u) / 10;

            // a 字段：bit_0: 感知路面, bit_1: RANSAC 路面
            if ((v > 370) && (percetion_road_mask.at<uint8_t>(v, u) == 255))
            {
                color_p.a |= 0b00000001; // set bit_0
                color_p.geom_type |= 0b00000001;
            }

            output_scan_ptr->points.push_back(color_p);
        }
        output_scan_ptr->width = output_scan_ptr->size();
    }

    void ransac_road(Frame &frame, pcl::PointCloud<RenderPointType>::Ptr &scan_ptr)
    {
        pcl::PointIndices::Ptr down_index(new pcl::PointIndices);
        for (size_t i = 0; i < scan_ptr->points.size(); i++)
        {
            if (scan_ptr->points[i].z < 0)
            {
                down_index->indices.push_back(i);
            }
        }

        pcl::PointIndices::Ptr plane_index(new pcl::PointIndices);
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::SACSegmentation<RenderPointType> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.2);
            seg.setInputCloud(scan_ptr);
            seg.setIndices(down_index);
            seg.segment(*plane_index, *coefficients);
        }
        if (frame.has_cloud_pano_seg || frame.has_car_pano_seg)
        {
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.5))
            for (auto &i : plane_index->indices)
            {
                scan_ptr->points[i].a |= 0b00000010; // set bit_1 RANSAC 路面
                scan_ptr->points[i].geom_type |= 0b00000010;
            }
        }
        else
        {
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.5))
            for (auto &i : plane_index->indices)
            {
                scan_ptr->points[i].a |= 0b00000011; // set bit_1 RANSAC 路面
                scan_ptr->points[i].geom_type |= 0b00000011;
            }
        }

        // for (auto &p : scan_ptr->points)
        // {
        //     p.ring = p.a;
        // }
    }

    float down_intensity(uint16_t ring, float y, float intensity)
    {
        float scale_ring = 1 / (1 + exp((ring - 127) / 6.0));
        float scale_y = 1.0;
        if (ring >= 100)
        {
            scale_y = 1 / (1 + exp(-abs(y) / 2.0));
        }
        return ceil(intensity * scale_ring * scale_y); // 根据以上两个系数，对强度进行弱化
    }

    float adjust_distribution(float intensity, float mid_intensity, float max_intensity)
    {
        float maxLimit = 255.0; // 默认一帧的点云最大强度为255
        intensity = std::min(maxLimit, intensity);
        mid_intensity = std::min(maxLimit - 1, mid_intensity);
        max_intensity = std::min(maxLimit, max_intensity);
        float res = 0;
        if (intensity <= mid_intensity)
        {
            res = intensity / mid_intensity * 30; // 中间强度以下的强度都分布在30以下
        }
        else
        {
            res = (intensity - mid_intensity) / (max_intensity - mid_intensity) * (255 - 30) + 30; // 中间强度以上的都分布在30以上
        }
        return res;
    }

    void intensity_and_distance(pcl::PointCloud<RenderPointType>::Ptr &scan_ptr)
    {
        float max_intensity = 5; // 该帧点云的最大强度，默认为5
        float mid_intensity = 0; // 该帧点云的中间强度，中间强度的作用是使强度小于中间强度的分布在前半段，大于中间强度的分布在后半段
        std::map<int, int> intensity_dict;
        for (auto &p : scan_ptr->points)
        {
            max_intensity = std::max(max_intensity, (p.intensity));
            int inten_v = round(p.intensity);
            if (intensity_dict.end() != intensity_dict.find(inten_v))
            {
                intensity_dict[inten_v]++;
            }
            else
            {
                intensity_dict.insert(std::pair<int, int>(inten_v, 1));
            }
        }

        float nums = scan_ptr->points.size();
        int cur_nums = 0;
        for (auto it = intensity_dict.begin(); it != intensity_dict.end(); it++)
        {
            cur_nums += it->second;
            if (float(cur_nums) / nums > 0.95) // 代表9.5成的点分布在前半段
            {
                mid_intensity = it->first; // 中间强度的选择取决于该强度的数量，目的是让大部分点位于前半段
                break;
            }
        }
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.5))
        for (auto &p : scan_ptr->points)
        {
            p.intensity = down_intensity(p.ring, p.y, p.intensity);
            p.intensity = adjust_distribution(p.intensity, mid_intensity, max_intensity); // 根据该帧点云的中间强度、最大强度、该点强度，重新调整该点的强度

            { // 将距离藏到intensity小数点后
                double distance = sqrt(pow(p.x, 2) + pow(p.y, 2));
                if (distance > 200)
                {
                    distance = 200;
                }
                if (!is_day) // 如果是黑天，距离+200m
                {
                    distance += 200;
                }
                p.intensity = round(p.intensity) + (distance / 10000);
            }
        }
    }

    void rm_new_firmware_bad_points(pcl::PointCloud<RenderPointType>::Ptr &scan_ptr)
    {
        if (!is_new_firmware)
        {
            return;
        }
        for (auto &p : scan_ptr->points)
        {
            if (p.a & 0b10000000) // bad point
            {
                p.x = std::numeric_limits<float>::quiet_NaN();
            }
        }
        scan_ptr->is_dense = false;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*scan_ptr, *scan_ptr, indices);
    }

    void pca_plane_and_line(Frame &frame, pcl::PointCloud<RenderPointType>::Ptr &scan_ptr)
    {
        PcaFeatureExtraction<RenderPointType> pca_extractor;
        pca_extractor.Extract(scan_ptr);
        struct Point3DHash
        {
            std::size_t operator()(const std::tuple<int, int, int> &p) const
            {
                return (std::get<2>(p) << 40) + (std::get<1>(p) << 20) + std::get<0>(p);
            }
        };
        phmap::flat_hash_map<std::tuple<int, int, int>, uint8_t, Point3DHash> pca_plane_feature;
        for (auto &p : *pca_extractor.feature_clouds[0])
        {
            auto key = std::make_tuple(round(p.x / 0.1), round(p.y / 0.1), round(p.z / 0.1));
            pca_plane_feature.insert(std::make_pair(key, 1));
        }
        for (auto &p : *pca_extractor.feature_clouds[1])
        {
            auto key = std::make_tuple(round(p.x / 0.1), round(p.y / 0.1), round(p.z / 0.1));
            pca_plane_feature.insert(std::make_pair(key, 1));
        }
        phmap::flat_hash_map<std::tuple<int, int, int>, uint8_t, Point3DHash> pca_line_feature;
        for (auto &p : *pca_extractor.feature_clouds[2])
        {
            auto key = std::make_tuple(round(p.x / 0.1), round(p.y / 0.1), round(p.z / 0.1));
            pca_line_feature.insert(std::make_pair(key, 1));
        }
        for (auto &p : *pca_extractor.feature_clouds[3])
        {
            auto key = std::make_tuple(round(p.x / 0.1), round(p.y / 0.1), round(p.z / 0.1));
            pca_line_feature.insert(std::make_pair(key, 1));
        }
        for (auto &p : *scan_ptr)
        {
            auto key = std::make_tuple(round(p.x / 0.1), round(p.y / 0.1), round(p.z / 0.1));
            if (pca_plane_feature.find(key) != pca_plane_feature.end())
            {
                p.a |= 0b00000100; // set bit_2 PCA 平面
                p.geom_type |= 0b00000100;
            }
            if (pca_line_feature.find(key) != pca_line_feature.end())
            {
                p.a |= 0b00001000; // set bit_3 PCA 直线
                p.geom_type |= 0b00001000;
            }
        }
    }

    float veh_max_x = 50;
    float veh_min_x = 0;
    float veh_max_y = 15;
    float veh_min_y = -15;
    float veh_max_z = 10;
    float veh_min_z = 0;
    float pixel_size = 0.05;

    inline Eigen::Vector2i position_to_index2d(Eigen::Vector2f position)
    {
        Eigen::Vector2i index;
        index.x() = round((veh_max_y - position.y()) / pixel_size);
        index.y() = round((veh_max_x - position.x()) / pixel_size);
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
        position.x() = veh_max_y - index.y() * pixel_size;
        position.y() = veh_max_x - index.x() * pixel_size;
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

    void add_bev_label_ground(std::string bev_label_path,
                              phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<float, 7, 1>, Point2DHash> &cloud_bev_label_dict_ground)
    {
        nlohmann::json j = nlohmann::json::parse(std::ifstream(bev_label_path));
        std::string cat_version = j.contains("cat_version") ? j["cat_version"] : "v1_37";

        for (auto &one_instance : j["detail"]["instance_list"]) // 遍历每个BEV实体
        {
            int raw_bev_id = one_instance["attrs"]["type"]; // 原始 bev label id
            float score = one_instance["attrs"]["score"];

            int label_0 = 0;
            int label_1 = 0;
            int label_2 = 0;
            int color = 0;
            int shape = 0;
            int center_line = 0;
            int traffic_light = 0;
            bev_label_map(cat_version, raw_bev_id, label_0, label_1, label_2, color, shape, center_line, traffic_light);

            std::vector<Eigen::Vector2f> dense_points;

            if (label_1 == 3 || label_1 == 4 || label_1 == 5 || label_2 == 3) // 如果是地面 polygon
            {
                std::vector<Eigen::Vector2f> sparse_polygon_line;
                for (const auto &one_point : one_instance["data"]["points"])
                {
                    sparse_polygon_line.emplace_back(one_point[0], one_point[1]);
                }
                std::vector<Eigen::Vector2f> dense_polygon_line = densify_2d_line(sparse_polygon_line, 0.05); // 稠密化
                if (label_1 == 3 || label_1 == 4 || label_1 == 5 || label_2 == 3)                             // 如果是人行道/箭头/路口区域，填充内部
                {
                    dense_points = fill_the_crosswalk(dense_polygon_line);
                }
            }
            else if (traffic_light > 0) // 如果是空中 3dbox
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
                        dense_points.emplace_back(x, y);
                    }
                }
            }
            else // 如果是地面线类
            {
                std::vector<Eigen::Vector2f> sparse_line;
                for (const auto &one_point : one_instance["data"]["points"])
                {
                    sparse_line.emplace_back(one_point[0], one_point[1]);
                }
                std::vector<Eigen::Vector2f> dense_line_0 = densify_2d_line(sparse_line, 0.05); // 稠密化
                for (auto &p : dense_line_0)
                    for (int dx = -1; dx <= 1; dx++)
                        for (int dy = -1; dy <= 1; dy++)
                            dense_points.push_back(p + 0.05 * Eigen::Vector2f(dx, dy));
            }

            // 更新dict
            for (auto &p : dense_points)
            {
                if (p.x() > veh_max_x || p.x() < veh_min_x || p.y() > veh_max_y || p.y() < veh_min_y)
                {
                    continue;
                }
                auto key = position_to_index2d(p);

                if (cloud_bev_label_dict_ground.find(key) == cloud_bev_label_dict_ground.end())
                {
                    cloud_bev_label_dict_ground[key] = Eigen::Matrix<float, 7, 1>::Zero();
                }
                if (cloud_bev_label_dict_ground[key][0] == 0)
                {
                    cloud_bev_label_dict_ground[key][0] = label_0;
                }
                if (cloud_bev_label_dict_ground[key][1] == 0)
                {
                    cloud_bev_label_dict_ground[key][1] = label_1;
                }
                if (cloud_bev_label_dict_ground[key][2] == 0)
                {
                    cloud_bev_label_dict_ground[key][2] = label_2;
                }
                if (cloud_bev_label_dict_ground[key][3] == 0)
                {
                    cloud_bev_label_dict_ground[key][3] = color;
                }
                if (cloud_bev_label_dict_ground[key][4] == 0)
                {
                    cloud_bev_label_dict_ground[key][4] = shape;
                }
                if (cloud_bev_label_dict_ground[key][5] == 0)
                {
                    cloud_bev_label_dict_ground[key][5] = center_line;
                }
                // if (cloud_bev_label_dict_ground[key][6] == 0)
                // {
                //     cloud_bev_label_dict_ground[key][6] = traffic_light;
                // }
            }
        }
    }

    void add_bev_label_air(std::string bev_label_path,
                           phmap::flat_hash_map<Eigen::Vector3i, Eigen::Matrix<float, 1, 1>, Point3DHash> &cloud_bev_label_dict_air)
    {
        nlohmann::json j = nlohmann::json::parse(std::ifstream(bev_label_path));
        std::string cat_version = j.contains("cat_version") ? j["cat_version"] : "v1_37";

        for (auto &one_instance : j["detail"]["instance_list"]) // 遍历每个BEV实体
        {
            if (!one_instance["data"].contains("3dbox")) // 如果是 3dbox 类
            {
                continue;
            }

            int raw_bev_id = one_instance["attrs"]["type"]; // 原始 bev label id

            int label_0 = 0;
            int label_1 = 0;
            int label_2 = 0;
            int color = 0;
            int shape = 0;
            int center_line = 0;
            int traffic_light = 0;
            bev_label_map(cat_version, raw_bev_id, label_0, label_1, label_2, color, shape, center_line, traffic_light);

            std::vector<Eigen::Vector3f> dense_points;
            if (traffic_light > 0) // 如果是空中 3dbox
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
                    cloud_bev_label_dict_air[key] = Eigen::Matrix<float, 1, 1>::Zero();
                }
                if (cloud_bev_label_dict_air[key][0] == 0)
                {
                    cloud_bev_label_dict_air[key][0] = traffic_light;
                }
            }
        }
    }

    void add_bev_label(Frame &frame, pcl::PointCloud<RenderPointType>::Ptr &scan_ptr)
    {
        if (!frame.has_bev_label)
        {
            return;
        }

        phmap::flat_hash_map<Eigen::Vector2i, Eigen::Matrix<float, 7, 1>, Point2DHash> cloud_bev_label_dict_ground; // label-old, label1, label2, color, shape, center_line, traffic_light
        phmap::flat_hash_map<Eigen::Vector3i, Eigen::Matrix<float, 1, 1>, Point3DHash> cloud_bev_label_dict_air;    // traffic_light

        add_bev_label_ground(frame.bev_label_path, cloud_bev_label_dict_ground);
        add_bev_label_air(frame.bev_label_path, cloud_bev_label_dict_air);

#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.5))
        for (auto &p : scan_ptr->points)
        {
            if ((p.geom_type & 3) == 3) // 地面点
            {
                Eigen::Vector2f position_xy(p.x, p.y);
                Eigen::Vector2i index_xy = position_to_index2d(position_xy);
                if (cloud_bev_label_dict_ground.find(index_xy) == cloud_bev_label_dict_ground.end())
                {
                    p.cloud_bev_label = 0;
                    p.cloud_bev_label_1 = 0;
                    p.cloud_bev_label_2 = 0;
                    p.cloud_bev_color = 0;
                    p.cloud_bev_shape = 0;
                    p.cloud_bev_center_line = 0;
                    p.cloud_bev_traffic_light = 0;
                }
                else
                {
                    auto &cloud_bev_label = cloud_bev_label_dict_ground[index_xy];
                    p.cloud_bev_label = cloud_bev_label[0];
                    p.cloud_bev_label_1 = cloud_bev_label[1];
                    p.cloud_bev_label_2 = cloud_bev_label[2];
                    p.cloud_bev_color = cloud_bev_label[3];
                    p.cloud_bev_shape = cloud_bev_label[4];
                    p.cloud_bev_center_line = cloud_bev_label[5];
                    p.cloud_bev_traffic_light = cloud_bev_label[6];
                }
            }
            else // 空中点
            {
                Eigen::Vector3f position_xyz(p.x, p.y, p.z);
                Eigen::Vector3i index_xyz = position_to_index3d(position_xyz);
                if (cloud_bev_label_dict_air.find(index_xyz) == cloud_bev_label_dict_air.end())
                {
                    p.cloud_bev_traffic_light = 0;
                }
                else
                {
                    auto &cloud_bev_label = cloud_bev_label_dict_air[index_xyz];
                    p.cloud_bev_traffic_light = cloud_bev_label[0];
                }
            }
        }
    }

    void process_one_frame(Frame &frame)
    {
        if (file_exists(path_join(veh_scan_folder, str_format("%s.pcd", std::to_string(frame.stamp_ms).c_str()))))
        {
            return; // 已处理过则跳过
        }

        pcl::PointCloud<RawPointType>::Ptr raw_scan_ptr(new pcl::PointCloud<RawPointType>);
        pcl::io::loadPCDFile(frame.scan_path, *raw_scan_ptr);                // 读入原始scan
        pcl::transformPointCloud(*raw_scan_ptr, *raw_scan_ptr, T_veh_lidar); // 转为车体系

        // RGB和分割图去畸变
        cv::Mat raw_car_pano_seg;
        cv::Mat un_rgb_image;
        cv::Mat un_car_pano_seg = cv::Mat(new_cam_height, new_cam_width, CV_8UC1, 255);
        cv::Mat un_cloud_pano_seg = cv::Mat(new_cam_height, new_cam_width, CV_8UC1, 255);
        cv::Mat un_cloud_front_line = cv::Mat(new_cam_height, new_cam_width, CV_8UC1, 255);
        if (frame.has_front_image)
        {
            cv::Mat raw_rgb_image = cv::imread(frame.front_image_path);
            if (raw_rgb_image.empty())
            {
                LOG_WARN("raw_rgb_image.empty() {}", frame.front_image_path);
                frame.has_front_image = false;
            }
            else
            {
                cv::remap(raw_rgb_image, un_rgb_image, undist_map_1, undist_map_2, cv::INTER_LINEAR);
            }
        }
        if (frame.has_car_pano_seg)
        {
            perception_data_to_png(frame.perception_data_path, raw_car_pano_seg); // 解析点云分割图
            if (raw_car_pano_seg.empty())
            {
                LOG_WARN("raw_car_pano_seg.empty() {}", frame.perception_data_path);
                frame.has_car_pano_seg = false;
            }
            else
            {
                cv::remap(raw_car_pano_seg, un_car_pano_seg, undist_map_1, undist_map_2, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 255);
            }
        }
        if (frame.has_cloud_pano_seg)
        {
            cv::Mat raw_cloud_pano_seg = cv::imread(frame.cloud_pano_seg_path, cv::IMREAD_GRAYSCALE);
            if (raw_cloud_pano_seg.empty())
            {
                LOG_WARN("raw_cloud_pano_seg.empty() {}", frame.cloud_pano_seg_path)
                frame.has_cloud_pano_seg = false;
            }
            else
            {
                cv::remap(raw_cloud_pano_seg, un_cloud_pano_seg, undist_map_1, undist_map_2, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 255);
                un_cloud_pano_seg = un_cloud_pano_seg * 10;
            }
        }
        if (frame.has_cloud_line_seg)
        {
            cv::Mat raw_cloud_line_seg = cv::imread(frame.cloud_line_seg_path, cv::IMREAD_GRAYSCALE);
            if (raw_cloud_line_seg.empty())
            {
                LOG_WARN("raw_cloud_line_seg.empty() {}", frame.cloud_line_seg_path)
                frame.has_cloud_line_seg = false;
            }
            else
            {
                cv::remap(raw_cloud_line_seg, un_cloud_front_line, undist_map_1, undist_map_2, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 255);
                un_cloud_front_line = un_cloud_front_line * 10;
            }
        }

        pcl::PointCloud<RenderPointType>::Ptr color_scan_ptr(new pcl::PointCloud<RenderPointType>);
        proj_rgb_and_seg(frame, raw_scan_ptr, color_scan_ptr, un_rgb_image, un_car_pano_seg, un_cloud_pano_seg, un_cloud_front_line); // 投影着色
        ransac_road(frame, color_scan_ptr);                                                                                           // ransac 地面
        // pca_plane_and_line(frame, color_scan_ptr);// 提 pca 特征
        intensity_and_distance(color_scan_ptr); // 强度处理
        if (frame.has_bev_label)                // 强度处理
        {
            add_bev_label(frame, color_scan_ptr);
        }

        { // 输出 lidar 系 scan
            pcl::PointCloud<RenderPointType>::Ptr lidar_rgb_ptr(new pcl::PointCloud<RenderPointType>);
            pcl::transformPointCloud(*color_scan_ptr, *lidar_rgb_ptr, T_lidar_veh);
            if (is_new_firmware)
            {
                rm_new_firmware_bad_points(lidar_rgb_ptr); // 给晓佳的scan数据去除坏点
            }
            pcl::io::savePCDFileBinaryCompressed(path_join(lidar_rgb_folder, str_format("%s.pcd", std::to_string(frame.stamp_ms).c_str())), *lidar_rgb_ptr);
        }

        { // 输出 veh 系 scan
            pcl::PointCloud<RenderPointType>::Ptr veh_scan_ptr(new pcl::PointCloud<RenderPointType>);
            pcl::ConditionalRemoval<RenderPointType> condrem;
            pcl::ConditionAnd<RenderPointType>::Ptr range_cond_ptr(new pcl::ConditionAnd<RenderPointType>());
            range_cond_ptr->addComparison(pcl::FieldComparison<RenderPointType>::ConstPtr(
                new pcl::FieldComparison<RenderPointType>("x", pcl::ComparisonOps::LT, x_max)));
            range_cond_ptr->addComparison(pcl::FieldComparison<RenderPointType>::ConstPtr(
                new pcl::FieldComparison<RenderPointType>("y", pcl::ComparisonOps::LT, y_max)));
            range_cond_ptr->addComparison(pcl::FieldComparison<RenderPointType>::ConstPtr(
                new pcl::FieldComparison<RenderPointType>("y", pcl::ComparisonOps::GT, y_min)));
            range_cond_ptr->addComparison(pcl::FieldComparison<RenderPointType>::ConstPtr(
                new pcl::FieldComparison<RenderPointType>("z", pcl::ComparisonOps::LT, z_max)));
            condrem.setCondition(range_cond_ptr);
            condrem.setInputCloud(color_scan_ptr);
            condrem.filter(*veh_scan_ptr);
            custom_voxel_filter(*veh_scan_ptr, 0.05, 0.05, 0.05);
            pcl::io::savePCDFileBinaryCompressed(path_join(veh_scan_folder, str_format("%s.pcd", std::to_string(frame.stamp_ms).c_str())), *veh_scan_ptr);
        }

        { // 输出去畸变图片
            cv::imwrite(path_join(un_rgb_image_folder, str_format("%s.jpg", std::to_string(frame.stamp_ms).c_str())), un_rgb_image);
            if (frame.has_car_pano_seg)
            {
                cv::imwrite(path_join(raw_seg_image_folder, str_format("%s.png", std::to_string(frame.stamp_ms).c_str())), raw_car_pano_seg); // 输出去畸变前视图片
                cv::imwrite(path_join(un_seg_image_folder, str_format("%s.png", std::to_string(frame.stamp_ms).c_str())), un_car_pano_seg);   // 输出去畸变分割图
            }
            if (frame.has_cloud_pano_seg)
            {
                cv::imwrite(path_join(un_cloud_pano_seg_folder, str_format("%s.png", std::to_string(frame.stamp_ms).c_str())), un_cloud_pano_seg); // 输出去畸变分割图
            }
            if (frame.has_cloud_line_seg)
            {
                cv::imwrite(path_join(un_cloud_front_line_folder, str_format("%s.png", std::to_string(frame.stamp_ms).c_str())), un_cloud_front_line); // 输出去畸变分割图
            }
        }
    }

    void run_process()
    {
        LOG_INFO("启动点云处理");
        int finish_p = 0;
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.5))
        for (auto &one_frame : frame_list)
        {
            finish_p++;
            if (finish_p % 10 == 0)
            {
                LOG_INFO("点云着色进度{}/{}", finish_p, frame_list.size());
            }
            process_one_frame(one_frame);
        }
    }
};

int main(int argc, char **argv)
{
    std::cout << "===Hello scan render===" << std::endl;
    ArgParser arg_parser;
    arg_parser.add<std::string>("data_set_path", '\0', "输入 txt path", true, "");
    arg_parser.add<std::string>("output_folder", '\0', "输出文件夹", true, "");
    arg_parser.add<std::string>("x_max", '\0', "veh_scan x_max m", false, "50");
    arg_parser.add<std::string>("y_min", '\0', "veh_scan y_min m", false, "-50");
    arg_parser.add<std::string>("y_max", '\0', "veh_scan y_max m", false, "50");
    arg_parser.add<std::string>("z_max", '\0', "veh_scan z_max m", false, "10");
    arg_parser.parse_check(argc, argv);

    System system;
    system.data_set_path = arg_parser.get<std::string>("data_set_path");
    system.output_folder = arg_parser.get<std::string>("output_folder");
    system.x_max = stof(arg_parser.get<std::string>("x_max"));
    system.y_min = stof(arg_parser.get<std::string>("y_min"));
    system.y_max = stof(arg_parser.get<std::string>("y_max"));
    system.z_max = stof(arg_parser.get<std::string>("z_max"));

    auto logger = Logger::instance();
    logger->setLogLevel(spdlog::level::level_enum::info, spdlog::level::level_enum::trace);
    logger->setup(system.output_folder + "/log/" + _time_str_s() + "_render.log", "scan render");

    LOG_INFO("============================================");
    LOG_INFO("  Hello scan render");
    LOG_INFO("  data_set_path:{}", system.data_set_path);
    LOG_INFO("  output_folder:{}", system.output_folder);
    LOG_INFO("============================================");

    system.read_json();
    system.init_process();
    system.run_process();

    LOG_INFO("完成 scan render");
    return 0;
}

// /home/test/code/lidar-mapping/pointcloudrender/bin/pointcloud_render --data_set_path=/home/test/lidar_mapping_ws/fs_31974/data/5431676614279/stage_1_data_set.json --output_folder=/home/test/lidar_mapping_ws/fs_31974/data/5431676614279
// /home/test/code/fsd_map_road_model_dev/preprocess/pointcloudrender/bin/scan_render --data_set_path=/home/test/lidar_mapping_ws/26363/data/6171673476216/stage_1_data_set.json --output_folder=/home/test/lidar_mapping_ws/26363/data/6171673476216
