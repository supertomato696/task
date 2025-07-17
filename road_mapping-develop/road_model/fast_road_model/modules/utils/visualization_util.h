

#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#include "opencv2/opencv.hpp"
#include "utils/common_util.h"
#include "utils/log_util.h"
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 PointXYZ_OPT {
    PCL_ADD_POINT4D
    uint16_t ins_id;
    uint16_t index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
    friend std::ostream& operator << (std::ostream& os, const PointXYZ_OPT& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZ_OPT,
                                   (float, x, x) (float, y, y)(float, z, z)
                                   (uint16_t, ins_id, ins_id) (uint16_t, index, index) 
                                  )

struct EIGEN_ALIGN16 PointLabel // 强制SSE填充以获得正确的内存对齐
{
    PCL_ADD_POINT4D; // 添加XYZ
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY; // 添加强度
    uint16_t label;
    uint8_t cloud_pano_seg;
    uint8_t cloud_line_seg;
    uint8_t cloud_bev_label;
    uint16_t distance_x_cm;
    uint16_t distance_y_cm;
    uint8_t cloud_bev_label_score;
    uint8_t cloud_bev_label_1;
    uint8_t cloud_bev_label_2;
    uint8_t cloud_bev_color;
    uint8_t cloud_bev_shape;
    uint8_t opt_label;
    float intensity_opt;
    float score;
    int8_t status = 0;
    PCL_MAKE_ALIGNED_OPERATOR_NEW // 确保新的分配器内存是对齐的
    friend std::ostream& operator << (std::ostream& os, const PointLabel& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointLabel,
        (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)
        (float, intensity, intensity)
        (uint16_t, label, label)
        (uint8_t, cloud_pano_seg, cloud_pano_seg)
        (uint8_t, cloud_line_seg, cloud_line_seg)
        (uint8_t, cloud_bev_label, cloud_bev_label)
        (uint16_t, distance_x_cm, distance_x_cm)
        (uint16_t, distance_y_cm, distance_y_cm)
        (uint8_t, cloud_bev_label_score, cloud_bev_label_score)
        (uint8_t, cloud_bev_label_1, cloud_bev_label_1)
        (uint8_t, cloud_bev_label_2, cloud_bev_label_2)
        (uint8_t, cloud_bev_color, cloud_bev_color)
        (uint8_t, cloud_bev_shape, cloud_bev_shape)
        (uint8_t, opt_label, opt_label)
        (float, intensity_opt, intensity_opt)
        (float, score, score)
        )


struct EIGEN_ALIGN16 PointElement // 强制SSE填充以获得正确的内存对齐
{
    PCL_ADD_POINT4D; // 添加XYZ
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY; // 添加强度
    uint16_t ele_type; // 要素类型
    uint32_t id; // 实例id
    uint32_t index; // 同一个下面点的顺序
    uint32_t type1; // 扩展类型1
    uint32_t type2; // 扩展类型2
    uint32_t type3; // 扩展类型3
    float heading; // yaw角
    float score; // 置信度
    PCL_MAKE_ALIGNED_OPERATOR_NEW // 确保新的分配器内存是对齐的
    friend std::ostream& operator << (std::ostream& os, const PointElement& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointElement,
        (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)
        (uint16_t, ele_type, ele_type)
        (uint32_t, id, id)
        (uint32_t, index, index)
        (uint32_t, type1, type1)
        (uint32_t, type2, type2)
        (uint32_t, type3, type3)
        (float, heading, heading)
        (float, score, score)
        )


namespace fsdmap {
namespace utils {

//using CloudPoint = PointXYZIRT;
using CloudPoint = PointLabel;
// using CloudPoint = pcl::PointXYZI;
using RGBPoint = pcl::PointXYZRGB;
using Cloud = pcl::PointCloud<CloudPoint>;
using CloudPtr = pcl::PointCloud<CloudPoint>::Ptr;
using ElementCloud = pcl::PointCloud<PointElement>;
using ElementCloudPtr = pcl::PointCloud<PointElement>::Ptr;
using RGBCloudPtr = pcl::PointCloud<RGBPoint>::Ptr;

struct DisplayScope : public std::enable_shared_from_this<DisplayScope> {
    DisplayScope() {}
    DisplayScope(double x, double y, Eigen::Vector3d &center_pos)
        : x_radius(x), y_radius(y), center_pos(center_pos) {
            resolution = 1;
            dir = {0, 0, 0};
        }
    DisplayScope(double x, double y, Eigen::Vector3d &center_pos, double resolution)
        : x_radius(x), y_radius(y), center_pos(center_pos), resolution(resolution) {
            set_resolution(resolution);
            dir = {0, 0, 0};
        }

    virtual ~DisplayScope() {}
    double x_radius;
    double y_radius;
    double resolution;
    // {pitch, roll, yaw}
    Eigen::Vector3d dir;
    Eigen::Vector3d center_pos;
    void set_resolution(double resolution) {
        if (resolution == 0.0) {
            return;
        }
        this->resolution = resolution;
        x_radius /= resolution;
        y_radius /= resolution;
    }

    bool in_scope(Eigen::Vector3d &center_pos) {

    }
};

struct RGB {
    int r;
    int g;
    int b;
};

struct LogElement {
    Eigen::Vector3d pos;
    // Eigen::Vector3d tar_pos;
    cv::Point tar_pt;
    PointLabel label;
    cv::Scalar color;
};

struct DisplayInfo : public std::enable_shared_from_this<DisplayInfo> {
    enum LOG_TYPE {
        POINT,
        LINE,
        LINE_INDEX,
        TEXT,
        POLYGEN,
    };

    std::string log_name;
    std::string desc;

    int type;
    cv::Scalar color = {255, 255, 255};
    std::vector<LogElement> path;

    CloudPtr cache_cloud;
    bool has_cache_pcd = false;
    bool has_cache_img = false;
    // std::vector<Eigen::Vector3d> path;
    // std::vector<PointLabel> labels;

    LogElement& add(Eigen::Vector3d &pos, double z_delta=0, int label=0) {
        LogElement ele;
        ele.pos = pos;
        // ele.pos.z() = 0;
        // ele.pos.z() += z_delta;
        ele.label.label = label;
        path.emplace_back(ele);
        // path.back().z() += z_delta;
        // labels.resize(path.size());
        // labels.back().label = label;
        return path.back();
    }
    
    LogElement& add(double x, double y, double z=0, int label=0) {
        LogElement ele;
        ele.pos = {x, y, z};
        ele.label.label = label;
        path.emplace_back(ele);

        // path.push_back({x, y, z});
        // labels.resize(path.size());
        // labels.back().label = label;
        return path.back();
    }
    
    template<typename FormatString, typename... Args>
    void init(utils::DisplayInfo::LOG_TYPE type, const FormatString &desc, const Args &... args) {
        this->type = type;
        this->desc = utils::fmt(desc, args...);
    }

    void init(DisplayInfo* raw_log) {
        this->type = raw_log->type;
        this->path = raw_log->path;
        this->color = raw_log->color;
        this->log_name = raw_log->log_name;
        this->desc = raw_log->desc;
    }
};


template<typename T>
int save_data_to_file(const char * file_name, std::vector<T> &data_list) {
    std::ofstream ofs;
    ofs.open(file_name, std::ios::out);
    ResPtr file_ptr([&ofs](){ofs.close();});
    if(!ofs.is_open()) {
        LOG_ERROR("open {} failed", file_name);
        return fsdmap::FAIL;
    }
    ofs << T::title() << std::endl;
    ofs << T::meta() << std::endl;
    for (auto &row : data_list) {
        ofs << row.string() << std::endl;
    }
    return fsdmap::SUCC;
}

float rgb_2_float(cv::Scalar &rgb);

RGB tran_intensity(float intensity);

void trans_display_local_pos(DisplayScope &box, Eigen::Vector3d &pos, bool need_scale=true);

void gen_display_image(cv::Mat &im, DisplayScope &box, DisplayInfo* log);

void gen_display_image(cv::Mat &im, DisplayScope &box, std::vector<DisplayInfo*> &log_list);

void save_display_image(const char * file_name, DisplayScope &box, DisplayInfo* log);

void save_display_image(const char * file_name, DisplayScope &box, std::vector<DisplayInfo*> &log_list);

void sample_line_to_cloud_point(CloudPtr &tar_cloud, 
        Eigen::Vector3d &start_pt, Eigen::Vector3d& end_pt, LogElement &ele);

void sample_cycle_to_cloud_point(CloudPtr &tar_cloud, Eigen::Vector3d &pt, double radius, 
        LogElement &ele);

int64_t save_display_pcd(CloudPtr &tar_cloud, DisplayScope &box, DisplayInfo* log);

int64_t save_display_pcd(CloudPtr &tar_cloud, DisplayScope &box, std::vector<DisplayInfo*> &log_list);

int64_t save_display_pcd(const char * file_name, DisplayScope &box, DisplayInfo* log);

int64_t save_display_pcd(const char * file_name, DisplayScope &box, std::vector<DisplayInfo*> &log_list);

int64_t save_display_pcd(RGBCloudPtr &tar_cloud, DisplayScope &box, CloudPtr &src_cloud, 
        utils::ThreadPoolProxy* pool=NULL);

int64_t save_display_pcd(CloudPtr &tar_cloud, DisplayScope &box, CloudPtr &src_cloud,
        utils::ThreadPoolProxy* pool=NULL);

}
}
