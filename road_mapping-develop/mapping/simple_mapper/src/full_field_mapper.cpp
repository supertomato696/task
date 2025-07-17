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
#include <pcl/filters/impl/crop_hull.hpp>
#include <pcl/surface/impl/concave_hull.hpp>
#include <pcl/surface/impl/convex_hull.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <malloc.h>
#include "include/log.h"
#include "include/util.h"
#include "include/json.h"
#include "parallel_hashmap/phmap.h"

struct EIGEN_ALIGN16 RenderPointType
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;
    uint16_t label;
    uint16_t distance_x_cm;
    uint16_t distance_y_cm;
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
                                  (uint16_t, label, label)
                                  (uint16_t, distance_x_cm, distance_x_cm)
                                  (uint16_t, distance_y_cm, distance_y_cm)
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
    uint8_t cloud_bev_center_line;
    uint8_t cloud_bev_traffic_light;
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
                                  (uint8_t, cloud_bev_center_line, cloud_bev_center_line)
                                  (uint8_t, cloud_bev_traffic_light, cloud_bev_traffic_light)
                                  (uint8_t, cloud_bev_label_score, cloud_bev_label_score))
// clang-format on

class Frame
{
public:
    Frame() {}
    ~Frame() {}

    std::string cloud_path;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Eigen::Matrix4d T;
};

#define RGBI_NUM 4
#define CAR_PANO_SEG_NUM 16
#define CLOUD_PANO_NUM 23
#define CLOUD_LINE_NUM 4
#define CLOUD_BEV_LABEL_NUM 7
#define CLOUD_BEV_LABEL_1_NUM 6
#define CLOUD_BEV_LABEL_2_NUM 4
#define CLOUD_BEV_COLOR_NUM 3
#define CLOUD_BEV_SHAPE_NUM 13
#define CLOUD_BEV_CENTER_LINE_NUM 3
#define CLOUD_BEV_TRAFFIC_LIGHT_NUM 8

struct Cm5VoxelCell
{
    Cm5VoxelCell() {}
    uint64_t index = 0;
    float weight = 0;
    float z = 0;
    float max_weight = 0;
    Eigen::Matrix<float, RGBI_NUM, 1> rgbi = Eigen::MatrixXf::Zero(RGBI_NUM, 1);
    Eigen::Matrix<float, CAR_PANO_SEG_NUM, 1> car_pano_seg_counter = Eigen::MatrixXf::Zero(CAR_PANO_SEG_NUM, 1);
    Eigen::Matrix<float, CLOUD_PANO_NUM, 1> cloud_pano_seg_counter = Eigen::MatrixXf::Zero(CLOUD_PANO_NUM, 1);
    Eigen::Matrix<float, CLOUD_LINE_NUM, 1> cloud_line_seg_counter = Eigen::MatrixXf::Zero(CLOUD_LINE_NUM, 1);
    Eigen::Matrix<float, CLOUD_BEV_LABEL_NUM, 1> cloud_bev_label_counter = Eigen::MatrixXf::Zero(CLOUD_BEV_LABEL_NUM, 1);
    Eigen::Matrix<float, CLOUD_BEV_LABEL_1_NUM, 1> cloud_bev_label_1_counter = Eigen::MatrixXf::Zero(CLOUD_BEV_LABEL_1_NUM, 1);
    Eigen::Matrix<float, CLOUD_BEV_LABEL_2_NUM, 1> cloud_bev_label_2_counter = Eigen::MatrixXf::Zero(CLOUD_BEV_LABEL_2_NUM, 1);
    Eigen::Matrix<float, CLOUD_BEV_COLOR_NUM, 1> cloud_bev_color_counter = Eigen::MatrixXf::Zero(CLOUD_BEV_COLOR_NUM, 1);
    Eigen::Matrix<float, CLOUD_BEV_SHAPE_NUM, 1> cloud_bev_shape_counter = Eigen::MatrixXf::Zero(CLOUD_BEV_SHAPE_NUM, 1);
    Eigen::Matrix<float, CLOUD_BEV_CENTER_LINE_NUM, 1> cloud_bev_center_line_counter = Eigen::MatrixXf::Zero(CLOUD_BEV_CENTER_LINE_NUM, 1);
    Eigen::Matrix<float, CLOUD_BEV_TRAFFIC_LIGHT_NUM, 1> cloud_bev_traffic_light_counter = Eigen::MatrixXf::Zero(CLOUD_BEV_TRAFFIC_LIGHT_NUM, 1);

    ModelPointType to_point(uint64_t index_x, uint64_t index_y)
    {
        ModelPointType out_p;
        out_p.x = double(index_x) / 20 - 50000;
        out_p.y = double(index_y) / 20 - 50000;
        out_p.z = z;
        out_p.r = round(rgbi[0]);
        out_p.g = round(rgbi[1]);
        out_p.b = round(rgbi[2]);
        out_p.intensity = rgbi[3];

        int col_index = 0, row_index = 0;
        car_pano_seg_counter.maxCoeff(&row_index, &col_index);
        out_p.label = row_index;
        if (out_p.label < 25)
        {
            out_p.label *= 10;
        }

        cloud_pano_seg_counter[0] = cloud_pano_seg_counter[0] / 2;
        cloud_pano_seg_counter.maxCoeff(&row_index, &col_index);
        out_p.cloud_pano_seg = row_index;

        cloud_line_seg_counter[0] = cloud_line_seg_counter[0] / 2;
        cloud_line_seg_counter.maxCoeff(&row_index, &col_index);
        out_p.cloud_line_seg = row_index;

        cloud_bev_label_counter[0] = cloud_bev_label_counter[0] / 20;
        float max_score = cloud_bev_label_counter.maxCoeff(&row_index, &col_index);
        out_p.cloud_bev_label = row_index;
        float sum_score = cloud_bev_label_counter.sum();
        if (sum_score <= 0)
        {
            out_p.cloud_bev_label_score = 0;
        }
        else if (out_p.cloud_bev_label == 0)
        {
            out_p.cloud_bev_label_score = 0;
        }
        else
        {
            out_p.cloud_bev_label_score = round((max_score / sum_score) * 100);
        }

        cloud_bev_label_1_counter[0] = cloud_bev_label_1_counter[0] / 20;
        cloud_bev_label_1_counter.maxCoeff(&row_index, &col_index);
        out_p.cloud_bev_label_1 = row_index;

        if (out_p.cloud_bev_label_1 > 0)
        {
            cloud_bev_color_counter[0] = 0;
            cloud_bev_shape_counter[0] = 0;
        }

        cloud_bev_label_2_counter[0] = cloud_bev_label_2_counter[0] / 20;
        cloud_bev_label_2_counter.maxCoeff(&row_index, &col_index);
        out_p.cloud_bev_label_2 = row_index;

        cloud_bev_color_counter.maxCoeff(&row_index, &col_index);
        out_p.cloud_bev_color = row_index;

        cloud_bev_shape_counter.maxCoeff(&row_index, &col_index);
        out_p.cloud_bev_shape = row_index;

        cloud_bev_center_line_counter[0] = cloud_bev_center_line_counter[0] / 20;
        cloud_bev_center_line_counter.maxCoeff(&row_index, &col_index);
        out_p.cloud_bev_center_line = row_index;

        cloud_bev_traffic_light_counter[0] = cloud_bev_traffic_light_counter[0] / 20;
        cloud_bev_traffic_light_counter.maxCoeff(&row_index, &col_index);
        out_p.cloud_bev_traffic_light = row_index;
        return out_p;
    }
};

class Cm5VoxelMap
{
public:
    struct Vector2iHash
    {
        inline std::size_t operator()(const Eigen::Vector2i &p) const
        {
            return (p[0] << 32) + p[1];
        }
    };

    uint64_t cell_num = 0;
    int slice_num;
    std::vector<std::mutex> mtx_list;
    std::vector<phmap::flat_hash_map<Eigen::Vector2i, Cm5VoxelCell, Vector2iHash>> hash_map_list;

    Cm5VoxelMap()
    {
        slice_num = 64;
        std::vector<std::mutex> list(slice_num);
        mtx_list.swap(list);
        hash_map_list.resize(slice_num);
    };

    inline uint64_t slice_id(const Eigen::Vector2i &p)
    {
        return (p[0] + p[1]) & 0b111111;
    }

    bool have_key(Eigen::Vector2i &p)
    {
        auto &hash_map = hash_map_list[slice_id(p)];
        return hash_map.find(p) != hash_map.end();
    }

    bool get(const Eigen::Vector2i &p, Cm5VoxelCell &cell_out)
    {
        auto id = slice_id(p);
        std::lock_guard<std::mutex> lck(mtx_list[id]);
        auto &hash_map = hash_map_list[id];
        if (hash_map.find(p) != hash_map.end())
        {
            cell_out = hash_map[p];
            return true;
        }
        else
        {
            return false;
        }
    }

    bool get_with_z(const Eigen::Vector2i &p, double z, Cm5VoxelCell &cell_out)
    {
        auto &hash_map = hash_map_list[slice_id(p)];
        if (hash_map.find(p) != hash_map.end())
        {
            if (fabs(hash_map[p].z - z) > 1)
            {
                return false;
            }
            cell_out = hash_map[p];
            return true;
        }
        else
        {
            return false;
        }
    }

    inline bool update(const Eigen::Vector2i &p, const RenderPointType &point, const float &point_weight)
    {
        auto id = slice_id(p);
        std::lock_guard<std::mutex> lck(mtx_list[id]);
        auto &hash_map = hash_map_list[id];
        Cm5VoxelCell cell_in;
        auto r = hash_map.insert(std::make_pair(p, cell_in));
        if (r.second == false)
        {
            Cm5VoxelCell &cell = r.first->second;
            if (fabs(cell.z - point.z) > 2.0)
            {
                return false;
            }
            cell.z = (cell.z * cell.weight + point.z * point_weight) / (cell.weight + point_weight);
            cell.weight = cell.weight + point_weight;
            if (point_weight > cell.max_weight)
            {
                cell.rgbi = Eigen::Matrix<float, RGBI_NUM, 1>(point.r, point.g, point.b, point.intensity);
                cell.max_weight = point_weight;
            }

            uint8_t car_pano_seg_id = int(point.label * 0.1);
            if (car_pano_seg_id < CAR_PANO_SEG_NUM)
            {
                cell.car_pano_seg_counter[car_pano_seg_id] += point_weight;
            }
            if (point.cloud_pano_seg < CLOUD_PANO_NUM)
            {
                cell.cloud_pano_seg_counter[point.cloud_pano_seg] += point_weight;
            }
            if (point.cloud_line_seg < CLOUD_LINE_NUM)
            {
                cell.cloud_line_seg_counter[point.cloud_line_seg] += point_weight;
            }
            if (point.cloud_bev_label < CLOUD_BEV_LABEL_NUM)
            {
                cell.cloud_bev_label_counter[point.cloud_bev_label] += point_weight;
            }
            if (point.cloud_bev_label_1 < CLOUD_BEV_LABEL_1_NUM)
            {
                cell.cloud_bev_label_1_counter[point.cloud_bev_label_1] += point_weight;
            }
            if (point.cloud_bev_label_2 < CLOUD_BEV_LABEL_2_NUM)
            {
                cell.cloud_bev_label_2_counter[point.cloud_bev_label_2] += point_weight;
            }
            if (point.cloud_bev_color < CLOUD_BEV_COLOR_NUM)
            {
                cell.cloud_bev_color_counter[point.cloud_bev_color] += point_weight;
            }
            if (point.cloud_bev_shape < CLOUD_BEV_SHAPE_NUM)
            {
                cell.cloud_bev_shape_counter[point.cloud_bev_shape] += point_weight;
            }
            if (point.cloud_bev_center_line < CLOUD_BEV_CENTER_LINE_NUM)
            {
                cell.cloud_bev_center_line_counter[point.cloud_bev_center_line] += point_weight;
            }
            if (point.cloud_bev_traffic_light < CLOUD_BEV_TRAFFIC_LIGHT_NUM)
            {
                cell.cloud_bev_traffic_light_counter[point.cloud_bev_traffic_light] += point_weight;
            }
        }
        else
        {
            Cm5VoxelCell &cell = r.first->second;
            cell.z = point.z;
            cell.rgbi = Eigen::Matrix<float, RGBI_NUM, 1>(point.r, point.g, point.b, point.intensity);
            cell.weight = point_weight;
            cell.max_weight = point_weight;
            uint8_t car_pano_seg_id = int(point.label * 0.1);
            if (car_pano_seg_id < CAR_PANO_SEG_NUM)
            {
                cell.car_pano_seg_counter[car_pano_seg_id] = point_weight;
            }
            if (point.cloud_pano_seg < CLOUD_PANO_NUM)
            {
                cell.cloud_pano_seg_counter[point.cloud_pano_seg] = point_weight;
            }
            if (point.cloud_line_seg < CLOUD_LINE_NUM)
            {
                cell.cloud_line_seg_counter[point.cloud_line_seg] = point_weight;
            }
            if (point.cloud_bev_label < CLOUD_BEV_LABEL_NUM)
            {
                cell.cloud_bev_label_counter[point.cloud_bev_label] = point_weight;
            }
            if (point.cloud_bev_label_1 < CLOUD_BEV_LABEL_1_NUM)
            {
                cell.cloud_bev_label_1_counter[point.cloud_bev_label_1] = point_weight;
            }
            if (point.cloud_bev_label_2 < CLOUD_BEV_LABEL_2_NUM)
            {
                cell.cloud_bev_label_2_counter[point.cloud_bev_label_2] = point_weight;
            }
            if (point.cloud_bev_color < CLOUD_BEV_COLOR_NUM)
            {
                cell.cloud_bev_color_counter[point.cloud_bev_color] = point_weight;
            }
            if (point.cloud_bev_shape < CLOUD_BEV_SHAPE_NUM)
            {
                cell.cloud_bev_shape_counter[point.cloud_bev_shape] = point_weight;
            }
            if (point.cloud_bev_center_line < CLOUD_BEV_CENTER_LINE_NUM)
            {
                cell.cloud_bev_center_line_counter[point.cloud_bev_center_line] = point_weight;
            }
            if (point.cloud_bev_traffic_light < CLOUD_BEV_TRAFFIC_LIGHT_NUM)
            {
                cell.cloud_bev_traffic_light_counter[point.cloud_bev_traffic_light] = point_weight;
            }
        }
        return true;
    }

    void add_index()
    {
        LOG_INFO("add_index()");
        cell_num = 0;
        for (auto &hash_map : hash_map_list)
        {
            for (auto &[k, cell] : hash_map)
            {
                cell.index = cell_num;
                cell_num++;
            }
        }
        LOG_INFO("cell_num {}", cell_num);
    }
};

struct M2VoxelCell
{
    M2VoxelCell() {}
    M2VoxelCell(const float &weight_, const float &z_) : weight(weight_), z(z_) {}
    float weight = 0;
    float z = 0;
};

class M2VoxelMap
{
public:
    M2VoxelMap()
    {
        slice_num = 64;
        std::vector<std::mutex> list(slice_num);
        mtx_list.swap(list);
        hash_map_list.resize(slice_num);
    };

    struct Vector2iHash
    {
        inline std::size_t operator()(const Eigen::Vector2i &p) const
        {
            return (p[0] << 32) + p[1];
        }
    };

    inline uint64_t slice_id(const Eigen::Vector2i &p)
    {
        return (p[0] + p[1]) & 0b111111;
    }

    bool have_key(Eigen::Vector2i &p)
    {
        auto &hash_map = hash_map_list[slice_id(p)];
        return hash_map.find(p) != hash_map.end();
    }

    void update(Eigen::Vector2i &p, const M2VoxelCell &cell_in)
    {
        auto id = slice_id(p);
        std::lock_guard<std::mutex> lck(mtx_list[id]);
        auto &hash_map = hash_map_list[id];
        if (hash_map.find(p) != hash_map.end())
        {
            M2VoxelCell &cell = hash_map[p];
            cell.z = (cell.z * cell.weight + cell_in.z * cell_in.weight) / (cell.weight + cell_in.weight);
            cell.weight = cell.weight + cell_in.weight;
        }
        else
        {
            hash_map[p] = cell_in;
        }
    }

    bool closed_to(const Eigen::Vector2i &p, const M2VoxelCell &cell_in)
    {
        auto id = slice_id(p);
        std::lock_guard<std::mutex> lck(mtx_list[id]);
        auto &hash_map = hash_map_list[id];
        auto r = hash_map.insert(std::make_pair(p, cell_in));

        if (r.second == true)
        {
            return true;
        }
        else
        {
            M2VoxelCell &cell = r.first->second;
            if (fabs(cell.z - cell_in.z) < 2.0)
            {
                // cell.z = (cell.z * cell.weight + cell_in.z * cell_in.weight) / (cell.weight + cell_in.weight);
                // cell.weight = cell.weight + cell_in.weight;
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    int slice_num;
    std::vector<std::mutex> mtx_list;
    std::vector<phmap::flat_hash_map<Eigen::Vector2i, M2VoxelCell, Vector2iHash>> hash_map_list;
};

class Cm10VoxelMap
{
public:
    Cm10VoxelMap()
    {
        slice_num = 64;
        std::vector<std::mutex> list(slice_num);
        mtx_list.swap(list);
        hash_map_list.resize(slice_num);
    };

    struct Vector3iHash
    {
        inline std::size_t operator()(const Eigen::Vector3i &p) const
        {
            return (p[0] << 42) + (p[1] << 21) + p.z();
        }
    };

    inline uint64_t slice_id(const Eigen::Vector3i &p)
    {
        return (p[0] + p[1] + p[2]) & 0b111111;
    }

    inline void update(const Eigen::Vector3i &p, const ModelPointType &point_in)
    {
        auto id = slice_id(p);
        std::lock_guard<std::mutex> lck(mtx_list[id]);
        auto &hash_map = hash_map_list[id];
        auto r = hash_map.insert(std::make_pair(p, point_in));
        if (r.second == false)
        {
            ModelPointType &point = r.first->second;
            if (point.cloud_bev_traffic_light == 0)
            {
                point.cloud_bev_traffic_light = point_in.cloud_bev_traffic_light;
            }
            // if ((point_in.intensity - int(point_in.intensity)) < (point.intensity - int(point.intensity)))
            //     point = point_in;
        }
    }

    uint64_t cell_num = 0;
    int slice_num;
    std::vector<std::mutex> mtx_list;
    std::vector<phmap::flat_hash_map<Eigen::Vector3i, ModelPointType, Vector3iHash>> hash_map_list;
};

bool cmp(const Frame &a, const Frame &b)
{
    return a.t.z() < b.t.z();
}

class System
{
public:
    System()
    {
        point_cloud_ground_ptr = pcl::PointCloud<ModelPointType>::Ptr(new pcl::PointCloud<ModelPointType>);
        point_cloud_ground_2_ptr = pcl::PointCloud<ModelPointType>::Ptr(new pcl::PointCloud<ModelPointType>);
        point_cloud_air_ptr = pcl::PointCloud<ModelPointType>::Ptr(new pcl::PointCloud<ModelPointType>);
        std::vector<std::mutex> list(32);
        mtx_list.swap(list);
    }
    ~System() {}

    std::string input_json_path;
    std::string output_folder_path;
    std::string flag;
    std::vector<Frame> frame_list;
    pcl::PointCloud<ModelPointType>::Ptr point_cloud_ground_ptr;
    pcl::PointCloud<ModelPointType>::Ptr point_cloud_ground_2_ptr;
    pcl::PointCloud<ModelPointType>::Ptr point_cloud_air_ptr;
    std::vector<std::mutex> mtx_list;
    std::mutex writePointLock;
    std::mutex mtx;
    std::mutex mtx2;

    std::string task_folder;

    void read_json()
    {
        nlohmann::json j = nlohmann::json::parse(std::ifstream(input_json_path));
        for (auto one : j["data"])
        {
            Frame frame;
            frame.cloud_path = one["cloud_path"];
            frame.t = one["t"].get<Eigen::Vector3d>();
            frame.q = one["q"].get<Eigen::Quaterniond>();
            frame.T = qab_tab_to_Tab(frame.q, frame.t);
            frame_list.push_back(frame);
        }
        std::sort(frame_list.begin(), frame_list.end(), cmp);
        LOG_INFO("完成读入 json, 帧数：{}", frame_list.size());
    }

    inline Eigen::Vector2i position_xy_to_5cm_index_xy(const Eigen::Vector2f &p_xy)
    {
        return Eigen::Vector2i(int(round((p_xy[0] + 50000) * 20)), int(round((p_xy[1] + 50000) * 20)));
    }

    inline Eigen::Vector2i position_xy_to_20cm_index_xy(const Eigen::Vector2f &p_xy)
    {
        return Eigen::Vector2i(int(round((p_xy[0] + 50000) * 5)), int(round((p_xy[1] + 50000) * 5)));
    }

    inline Eigen::Vector3i position_xyz_to_20cm_index_xyz(const Eigen::Vector3f &p_xyz)
    {
        return Eigen::Vector3i(int(round((p_xyz[0] + 50000) * 5)), int(round((p_xyz[1] + 50000) * 5)), int(round((p_xyz.z() + 10000) * 5)));
    }

    inline Eigen::Vector3f cm20_index_xyz_to_position_xyz(const Eigen::Vector3i &index_xyz)
    {
        return Eigen::Vector3f(float(index_xyz[0]) / 5 - 50000, float(index_xyz[1]) / 5 - 50000, float(index_xyz.z()) / 5 - 10000);
    }

    inline Eigen::Vector3i position_xyz_to_10cm_index_xyz(const Eigen::Vector3f &p_xyz)
    {
        return Eigen::Vector3i(int(round((p_xyz[0] + 50000) * 10)), int(round((p_xyz[1] + 50000) * 10)), int(round((p_xyz.z() + 10000) * 10)));
    }

    inline Eigen::Vector3f cm10_index_xyz_to_position_xyz(const Eigen::Vector3i &index_xyz)
    {
        return Eigen::Vector3f(float(index_xyz[0]) / 10 - 50000, float(index_xyz[1]) / 10 - 50000, float(index_xyz.z()) / 10 - 10000);
    }

    inline Eigen::Vector2i position_xy_to_2m_index_xy(const Eigen::Vector2f &p_xy)
    {
        return Eigen::Vector2i(int(round((p_xy[0] + 50000) * 0.5)), int(round((p_xy[1] + 50000) * 0.5)));
    }

    inline double sqrt_dis_to_score(const double &sqrt_dis) // 距离得分，(0~1)，距车心越近，得分越高
    {
        return exp(-sqrt_dis / 450);
    }

    inline bool update_voxel_map_py_point(const RenderPointType &point, Cm5VoxelMap &cm5_voxel_map, M2VoxelMap &m2_voxel_map)
    {
        float disx = float(point.distance_x_cm) / 100.;
        float disy = float(point.distance_y_cm) / 100.;
        float point_weight = sqrt_dis_to_score(disx * disx + disy * disy);
        Eigen::Vector2i map_2m_index_xy = position_xy_to_2m_index_xy({point.x, point.y});
        if (m2_voxel_map.closed_to(map_2m_index_xy, M2VoxelCell(point_weight, point.z)) == false)
        {
            return false;
        }
        else
        {
            if (point.a & 0b01000000) // night
            {
                point_weight *= 0.1;
            }
            Eigen::Vector2i map_5cm_index_xy(int(round((point.x + 50000) * 20)), int(round((point.y + 50000) * 20)));
            if (cm5_voxel_map.update(map_5cm_index_xy, point, point_weight) == false)
            {
                return false;
            }
        }
        return true;
    }

    void update_pointcloud_py_counter(Cm5VoxelMap &cm5_voxel_map, pcl::PointCloud<ModelPointType>::Ptr point_cloud_ground_ptr)
    {
        cm5_voxel_map.add_index();
        point_cloud_ground_ptr->points.resize(cm5_voxel_map.cell_num);
        point_cloud_ground_ptr->height = 1;
        point_cloud_ground_ptr->width = point_cloud_ground_ptr->size();

        uint32_t finish_f = 0;
#pragma omp parallel for num_threads(std::thread::hardware_concurrency())
        for (auto &hash_map : cm5_voxel_map.hash_map_list)
        {
            for (auto &[voxel_index_xy, cell] : hash_map)
            {
                finish_f++;
                if (finish_f % 100000 == 0)
                {
                    LOG_INFO("update_pointcloud_py_counter {}/{}", finish_f, cm5_voxel_map.cell_num);
                }

                Cm5VoxelCell local_cell = cell;
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        if (dx == 0 || dy == 0)
                        {
                            continue;
                        }

                        Cm5VoxelCell neighber_cell;
                        if (cm5_voxel_map.get_with_z(voxel_index_xy + Eigen::Vector2i(dx, dy), cell.z, neighber_cell) == false)
                        {
                            continue;
                        }

                        float neighber_loss = 1.0 / (dx * dx + dy * dy + 1);
                        float neighber_weight = neighber_cell.weight * neighber_loss;
                        float sum_weight = local_cell.weight + neighber_weight;
                        local_cell.z = (local_cell.weight * local_cell.z + neighber_weight * neighber_cell.z) / sum_weight;
                        local_cell.rgbi = (local_cell.weight * local_cell.rgbi + neighber_weight * neighber_cell.rgbi) / sum_weight;
                        local_cell.weight = sum_weight;
                        local_cell.car_pano_seg_counter += neighber_cell.car_pano_seg_counter * neighber_loss;
                        local_cell.cloud_pano_seg_counter += neighber_cell.cloud_pano_seg_counter * neighber_loss;
                        local_cell.cloud_line_seg_counter += neighber_cell.cloud_line_seg_counter * neighber_loss;
                        local_cell.cloud_bev_label_counter += neighber_cell.cloud_bev_label_counter * neighber_loss;
                        local_cell.cloud_bev_label_1_counter += neighber_cell.cloud_bev_label_1_counter * neighber_loss;
                        local_cell.cloud_bev_label_2_counter += neighber_cell.cloud_bev_label_2_counter * neighber_loss;
                        local_cell.cloud_bev_color_counter += neighber_cell.cloud_bev_color_counter * neighber_loss;
                        local_cell.cloud_bev_shape_counter += neighber_cell.cloud_bev_shape_counter * neighber_loss;
                        local_cell.cloud_bev_center_line_counter += neighber_cell.cloud_bev_center_line_counter * neighber_loss;
                        local_cell.cloud_bev_traffic_light_counter += neighber_cell.cloud_bev_traffic_light_counter * neighber_loss;
                    }
                }
                point_cloud_ground_ptr->points[cell.index] = local_cell.to_point(voxel_index_xy[0], voxel_index_xy[1]);
            }
        }
    }

    void run_mapping()
    {
        Cm5VoxelMap cm5_voxel_map_1;
        Cm5VoxelMap cm5_voxel_map_2;
        M2VoxelMap m2_voxel_map_1;
        M2VoxelMap m2_voxel_map_2;
        Cm10VoxelMap air_voxel_map;
        std::random_device rd;
        std::mt19937 rng(rd());

        LOG_INFO("start update_voxel_map_py_point()");
        int finish_f = 0;
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
        for (auto &frame : frame_list)
        {
            finish_f++;
            if (finish_f % 100 == 0)
            {
                LOG_INFO("run_mapping {}/{}", finish_f, frame_list.size());
            }

            pcl::PointCloud<RenderPointType>::Ptr frame_pc_ptr(new pcl::PointCloud<RenderPointType>);
            pcl::io::loadPCDFile(frame.cloud_path, *frame_pc_ptr);
            pcl::transformPointCloud(*frame_pc_ptr, *frame_pc_ptr, frame.T);

            // {
            //     pcl::io::savePCDFileBinaryCompressed(path_join(output_folder_path, "debug", get_file_name_in_path(frame.cloud_path)), *frame_pc_ptr);
            // }

            std::vector<uint64_t> air_index;
            air_index.reserve(frame_pc_ptr->points.size());
            for (auto i = 0; i < frame_pc_ptr->points.size(); i++)
            {
                auto &p = frame_pc_ptr->points[i];
                if ((p.a & 3) == 3)
                {
                    bool has_updated = update_voxel_map_py_point(p, cm5_voxel_map_1, m2_voxel_map_1);
                    if (has_updated == false)
                    {
                        update_voxel_map_py_point(p, cm5_voxel_map_2, m2_voxel_map_2);
                    }
                }
                else
                {
                    Eigen::Vector3i index_xyz = position_xyz_to_10cm_index_xyz(p.getVector3fMap());
                    ModelPointType out_p;
                    out_p.getVector3fMap() = p.getVector3fMap();
                    out_p.rgb = p.rgb;
                    out_p.intensity = p.intensity;
                    out_p.label = p.label;
                    out_p.cloud_pano_seg = p.cloud_pano_seg;
                    out_p.cloud_bev_traffic_light = p.cloud_bev_traffic_light;
                    air_voxel_map.update(index_xyz, out_p);
                }
            }
        }

        {
            for (auto &hash_map : air_voxel_map.hash_map_list)
            {
                for (auto &[voxel_index_xy, point] : hash_map)
                {
                    point_cloud_air_ptr->points.push_back(point);
                }
            };
            point_cloud_air_ptr->height = 1;
            point_cloud_air_ptr->width = point_cloud_air_ptr->size();
        }

        LOG_INFO("finish update_voxel_map_py_point()");
        LOG_INFO("update_pointcloud_py_counter() 1");
        update_pointcloud_py_counter(cm5_voxel_map_1, point_cloud_ground_ptr);
        LOG_INFO("update_pointcloud_py_counter() 2");
        update_pointcloud_py_counter(cm5_voxel_map_2, point_cloud_ground_2_ptr);
        *point_cloud_ground_ptr = *point_cloud_ground_ptr + *point_cloud_ground_2_ptr;
    }

    void cut_update(pcl::PointCloud<ModelPointType>::Ptr &ptr, std::string json_path)
    {
        std::vector<Eigen::Vector2f> polygon;
        nlohmann::json j = nlohmann::json::parse(std::ifstream(json_path));
        for (auto one : j["data"])
        {
            Eigen::Vector2f p(one[0], one[1]);
            polygon.push_back(p);
        }

        pcl::PointCloud<ModelPointType>::Ptr ployin(new pcl::PointCloud<ModelPointType>());
        pcl::PointCloud<ModelPointType>::Ptr ploygon(new pcl::PointCloud<ModelPointType>());
        for (auto point : polygon)
        {
            ployin->push_back({point[0], point[1], 0, 0, 0, 0, 0, 0, 0});
        }
        pcl::ConvexHull<ModelPointType> HULL;
        HULL.setInputCloud(ployin);
        HULL.setDimension(2);
        std::vector<pcl::Vertices> polygons;
        HULL.reconstruct(*ploygon, polygons); // 计算凸包

        pcl::PointCloud<ModelPointType>::Ptr in_ptr(new pcl::PointCloud<ModelPointType>());
        // 多边形滤波
        pcl::CropHull<ModelPointType> SURF_HULL;
        SURF_HULL.setDim(2);
        SURF_HULL.setHullIndices(polygons); // 凸包的索引值
        SURF_HULL.setHullCloud(ploygon);    // 凸包的点云

        SURF_HULL.setInputCloud(ptr);   // 查找的点云
        SURF_HULL.setCropOutside(true); // false剔除 true 提取
        SURF_HULL.filter(*in_ptr);

        *ptr = *in_ptr;
    }

    void output()
    {
        // cut for update
        std::string update_folder = output_folder_path;
        update_folder = update_folder.substr(0, update_folder.find_last_of("\\/"));
        update_folder = update_folder.substr(0, update_folder.find_last_of("\\/"));
        if (file_exists(path_join(update_folder, "update", "patch_info.json")))
        {
            cut_update(point_cloud_ground_ptr, path_join(update_folder, "update", "patch_info.json"));
            cut_update(point_cloud_air_ptr, path_join(update_folder, "update", "patch_info.json"));
        }

        if (!point_cloud_ground_ptr->empty())
        {
            point_cloud_ground_ptr->points.back().intensity = 255.0;
            pcl::io::savePCDFileBinary(path_join(output_folder_path, "ground.pcd"), *point_cloud_ground_ptr);
        }
        else
        {
            LOG_ERROR("point_cloud_ground_ptr empty()");
        }

        if (!point_cloud_air_ptr->empty())
        {
            pcl::io::savePCDFileBinary(path_join(output_folder_path, "air.pcd"), *point_cloud_air_ptr);
        }
        else
        {
            LOG_ERROR("point_cloud_air_ptr empty()");
        }

        pcl::PointCloud<ModelPointType>::Ptr final_map_ptr(new pcl::PointCloud<ModelPointType>);
        *final_map_ptr = *point_cloud_air_ptr + *point_cloud_ground_ptr;
        if (!final_map_ptr->empty())
        {
            pcl::io::savePCDFileBinary(path_join(output_folder_path, "final_map.pcd"), *final_map_ptr);
        }
        else
        {
            LOG_ERROR("final_map_ptr empty()");
        }

        {
            ModelPointType min_p1, max_p1;
            pcl::getMinMax3D(*final_map_ptr, min_p1, max_p1);
            std::vector<std::string> lines;
            lines.push_back(str_format("%f %f %f", min_p1.x, min_p1.y, min_p1.z));
            lines.push_back(str_format("%f %f %f", max_p1.x, max_p1.y, max_p1.z));
            write_lines_to_file_overwrite(path_join(output_folder_path, "final_map_bbox.txt"), lines);
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
    logger->setup(path_join(system.output_folder_path, "log", _time_str_s() + "_full_field_mapper.log"), "full_field_mapper");

    LOG_INFO("============================================");
    LOG_INFO("  Hello full field mapper");
    LOG_INFO("  input:{}", system.input_json_path);
    LOG_INFO("  output:{}", system.output_folder_path);
    LOG_INFO("============================================");

    system.read_json();
    system.run_mapping();
    LOG_INFO("output()");
    system.output();

    LOG_INFO("Finish full field mapper");
    return 0;
}

/*

/home/test/code/fsd_map_road_model_dev/mapping/simple_mapper/bin/full_field_mapper --input=/home/test/lidar_mapping_ws/26363/mapping_output/ff_mapping/ff_mapping_info.json --output=/home/test/lidar_mapping_ws/26363/mapping_output/ff_mapping
*/