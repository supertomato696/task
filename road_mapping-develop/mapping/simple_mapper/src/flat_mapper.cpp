#include <Eigen/Geometry>
#include <array>
#include <random>
#include <vector>
#include <thread>
#include <istream>
#include <unordered_set>
#include <map>
#include <unordered_map>
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


template<typename K, typename V>
using UMAP = std::map<K, V>;
// using UMAP = std::unordered_map<K, V>;


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

struct EIGEN_ALIGN16 PointXYZIRGBD
{
    double x;
    double y;
    double z;
    PCL_ADD_RGB;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
}; 
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGBD,
                                  (double, x, x)(double, y, y)(double, z, z)
                                  (float, rgb, rgb)) 

class Frame
{
public:
    Frame() {}
    ~Frame() {}

    std::string cloud_path;
    std::string bev_json_path = "";
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Eigen::Matrix4d T;
};

#define RGBI_NUM 4

enum ELEMENT_TYPE {
    ELEMENT_NULL,
    ELEMENT_LANE_LINE,
    ELEMENT_LANE_CENTER,
    ELEMENT_ROAD_CENTER,
    ELEMENT_BARRIER,
    ELEMENT_CURB,
    ELEMENT_POSE,
    ELEMENT_OBJECT,
    ELEMENT_JUNCTION,
    ELEMENT_VIRTUAL_LANE_LINE,
    ELEMENT_TRAFFICLIGHT,
};

struct LabelWeight {
    uint32_t value;
    float weight;
};

struct Cm5VoxelCell
{
    Cm5VoxelCell() {}
    uint64_t index = 0;
    float weight = 0;
    float z = 0;
    float center_line_score = 0;
    float max_weight = 0;
    Eigen::Matrix<float, RGBI_NUM, 1> rgbi = Eigen::MatrixXf::Zero(RGBI_NUM, 1);
    std::vector<LabelWeight> cloud_bev_label_counter;
    std::vector<LabelWeight> cloud_bev_label_shape_counter;
    std::vector<LabelWeight> cloud_bev_label_color_counter;
    std::vector<LabelWeight> cloud_bev_label_1_counter;
    std::vector<LabelWeight> cloud_bev_label_1_shape_counter;
    std::vector<LabelWeight> cloud_bev_label_1_color_counter;
    std::vector<LabelWeight> cloud_bev_label_2_counter;
    std::vector<LabelWeight> cloud_bev_label_2_shape_counter;
    std::vector<LabelWeight> cloud_bev_label_2_color_counter;
    std::vector<LabelWeight> cloud_bev_center_line_counter;
    std::vector<LabelWeight> cloud_bev_center_line_shape_counter;
    std::vector<LabelWeight> cloud_bev_center_line_color_counter;

    void updateCounter(std::vector<LabelWeight> &counter, uint32_t value, float weight) {
        bool found = false;
        for (auto &labelWeight : counter) {
            if (labelWeight.value == value) {
                labelWeight.weight += weight;
                found = true;
                break;
            }
        }
        if (!found) {
            counter.push_back(LabelWeight{value, weight});
        }
    }

    uint32_t findMaxWeightValue(const std::vector<LabelWeight>& counter, float& score){
        if (counter.empty()) {
            score = 0;
            return 0;
        }

        std::vector<LabelWeight> modified_counter = counter;
        for (size_t i = 0; i < modified_counter.size(); i++)
        {
            if (modified_counter[i].value == 0){
                modified_counter[i].weight = modified_counter[i].weight/20;
                break;
            }
        }

        uint32_t max_value = modified_counter[0].value;
        float max_weight = modified_counter[0].weight;
        float sum_weight = 0.0;

        for (const auto &labelWeight : modified_counter) {
            sum_weight += labelWeight.weight;
        }

        for (auto &labelWeight : modified_counter) {
            if (labelWeight.weight > max_weight) {
                max_weight = labelWeight.weight;
                max_value = labelWeight.value;
            }
        }

        score = (sum_weight > 0) ? (max_weight / sum_weight) : 0.0f;

        return max_value;
    }

    ModelPointTypeNew to_point(uint64_t index_x, uint64_t index_y)
    {
        ModelPointTypeNew out_p;
        out_p.x = double(index_x) / 20 - 50000;
        out_p.y = double(index_y) / 20 - 50000;
        out_p.z = z;
        out_p.cloud_bev_center_line_score = center_line_score;
        out_p.r = round(rgbi[0]);
        out_p.g = round(rgbi[1]);
        out_p.b = round(rgbi[2]);
        out_p.intensity = rgbi[3];

        // cloud_bev_label
        float label_score, shape_score, color_score;
        out_p.cloud_bev_label = findMaxWeightValue(cloud_bev_label_counter, label_score);
        out_p.cloud_bev_label_score = round(label_score * 100);
        out_p.cloud_bev_label_shape = findMaxWeightValue(cloud_bev_label_shape_counter, shape_score);
        out_p.cloud_bev_label_color = findMaxWeightValue(cloud_bev_label_color_counter, color_score);

        // cloud_bev_label_1
        float label1_score, shape1_score, color1_score;
        out_p.cloud_bev_label_1 = findMaxWeightValue(cloud_bev_label_1_counter, label1_score);
        out_p.cloud_bev_label_1_shape = findMaxWeightValue(cloud_bev_label_1_shape_counter, shape1_score);
        out_p.cloud_bev_label_1_color = findMaxWeightValue(cloud_bev_label_1_color_counter, color1_score);

        // cloud_bev_label_2
        float label2_score, shape2_score, color2_score;
        out_p.cloud_bev_label_2 = findMaxWeightValue(cloud_bev_label_2_counter, label2_score);
        out_p.cloud_bev_label_2_shape = findMaxWeightValue(cloud_bev_label_2_shape_counter, shape2_score);
        out_p.cloud_bev_label_2_color = findMaxWeightValue(cloud_bev_label_2_color_counter, color2_score);
        
        // cloud_bev_center_line
        float center_line_score, center_shape_score, center_color_score;
        out_p.cloud_bev_center_line = findMaxWeightValue(cloud_bev_center_line_counter, center_line_score);
        out_p.cloud_bev_center_line_shape = findMaxWeightValue(cloud_bev_center_line_shape_counter, center_shape_score);
        out_p.cloud_bev_center_line_color = findMaxWeightValue(cloud_bev_center_line_color_counter, center_color_score);

        return out_p;
    }

    void mergeLabelWeights(std::vector<LabelWeight>& local_cell, const std::vector<LabelWeight>& neighber_cell, float neighbor_weight_coefficient) {
        std::unordered_map<int, float> merged_weights;

        for (const auto& lw : local_cell) {
            merged_weights[lw.value] += lw.weight;
        }

        for (const auto& lw : neighber_cell) {
            merged_weights[lw.value] += lw.weight * neighbor_weight_coefficient;
        }

        local_cell.clear();
        for (const auto& entry : merged_weights) {
            local_cell.push_back({entry.first, entry.second});
        }
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

    inline bool update(const Eigen::Vector2i &p, const ModelPointTypeNew &point, const float &point_weight)
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
            cell.center_line_score = (cell.center_line_score * cell.weight + point.cloud_bev_center_line_score * point_weight) / (cell.weight + point_weight);
            cell.weight = cell.weight + point_weight;
            if (point_weight > cell.max_weight)
            {
                cell.rgbi = Eigen::Matrix<float, RGBI_NUM, 1>(point.r, point.g, point.b, point.intensity);
                cell.max_weight = point_weight;
            }

            cell.updateCounter(cell.cloud_bev_label_counter, point.cloud_bev_label, point_weight);
            cell.updateCounter(cell.cloud_bev_label_shape_counter, point.cloud_bev_label_shape, point_weight);
            cell.updateCounter(cell.cloud_bev_label_color_counter, point.cloud_bev_label_color, point_weight);
            cell.updateCounter(cell.cloud_bev_label_1_counter, point.cloud_bev_label_1, point_weight);
            cell.updateCounter(cell.cloud_bev_label_1_shape_counter, point.cloud_bev_label_1_shape, point_weight);
            cell.updateCounter(cell.cloud_bev_label_1_color_counter, point.cloud_bev_label_1_color, point_weight);
            cell.updateCounter(cell.cloud_bev_label_2_counter, point.cloud_bev_label_2, point_weight);
            cell.updateCounter(cell.cloud_bev_label_2_shape_counter, point.cloud_bev_label_2_shape, point_weight);
            cell.updateCounter(cell.cloud_bev_label_2_color_counter, point.cloud_bev_label_2_color, point_weight);
            cell.updateCounter(cell.cloud_bev_center_line_counter, point.cloud_bev_center_line, point_weight);
            cell.updateCounter(cell.cloud_bev_center_line_shape_counter, point.cloud_bev_center_line_shape, point_weight);
            cell.updateCounter(cell.cloud_bev_center_line_color_counter, point.cloud_bev_center_line_color, point_weight);
         }
        else
        {
            Cm5VoxelCell &cell = r.first->second;
            cell.z = point.z;
            cell.center_line_score = point.cloud_bev_center_line_score;
            cell.rgbi = Eigen::Matrix<float, RGBI_NUM, 1>(point.r, point.g, point.b, point.intensity);
            cell.weight = point_weight;
            cell.max_weight = point_weight;

            cell.updateCounter(cell.cloud_bev_label_counter, point.cloud_bev_label, point_weight);
            cell.updateCounter(cell.cloud_bev_label_shape_counter, point.cloud_bev_label_shape, point_weight);
            cell.updateCounter(cell.cloud_bev_label_color_counter, point.cloud_bev_label_color, point_weight);
            cell.updateCounter(cell.cloud_bev_label_1_counter, point.cloud_bev_label_1, point_weight);
            cell.updateCounter(cell.cloud_bev_label_1_shape_counter, point.cloud_bev_label_1_shape, point_weight);
            cell.updateCounter(cell.cloud_bev_label_1_color_counter, point.cloud_bev_label_1_color, point_weight);
            cell.updateCounter(cell.cloud_bev_label_2_counter, point.cloud_bev_label_2, point_weight);
            cell.updateCounter(cell.cloud_bev_label_2_shape_counter, point.cloud_bev_label_2_shape, point_weight);
            cell.updateCounter(cell.cloud_bev_label_2_color_counter, point.cloud_bev_label_2_color, point_weight);
            cell.updateCounter(cell.cloud_bev_center_line_counter, point.cloud_bev_center_line, point_weight);
            cell.updateCounter(cell.cloud_bev_center_line_shape_counter, point.cloud_bev_center_line_shape, point_weight);
            cell.updateCounter(cell.cloud_bev_center_line_color_counter, point.cloud_bev_center_line_color, point_weight);

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

    inline void update(const Eigen::Vector3i &p, const ModelPointTypeNew &point_in)
    {
        auto id = slice_id(p);
        std::lock_guard<std::mutex> lck(mtx_list[id]);
        auto &hash_map = hash_map_list[id];
        auto r = hash_map.insert(std::make_pair(p, point_in));
        if (r.second == false)
        {
            ModelPointTypeNew &point = r.first->second;
            if ((point_in.intensity - int(point_in.intensity)) < (point.intensity - int(point.intensity)))
                point = point_in;
        }
    }

    uint64_t cell_num = 0;
    int slice_num;
    std::vector<std::mutex> mtx_list;
    std::vector<phmap::flat_hash_map<Eigen::Vector3i, ModelPointTypeNew, Vector3iHash>> hash_map_list;
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
        point_cloud_ground_ptr = pcl::PointCloud<ModelPointTypeNew>::Ptr(new pcl::PointCloud<ModelPointTypeNew>);
        point_cloud_ground_2_ptr = pcl::PointCloud<ModelPointTypeNew>::Ptr(new pcl::PointCloud<ModelPointTypeNew>);
        point_cloud_air_ptr = pcl::PointCloud<ModelPointTypeNew>::Ptr(new pcl::PointCloud<ModelPointTypeNew>);
        gcj02_cloud_ptr = pcl::PointCloud<PointXYZIRGBD>::Ptr(new pcl::PointCloud<PointXYZIRGBD>);
        sm_cloud_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        // combined_pc_ptr = pcl::PointCloud<RenderPointTypeNew>::Ptr(new pcl::PointCloud<RenderPointTypeNew>);
        std::vector<std::mutex> list(32);
        mtx_list.swap(list);
    }
    ~System() {}
    // <label layer, label, color, shape>
    // label layer: 0,1,2,3(center line),4(traffic_light)
    // label:
    // color:
    // shape:
    UMAP<std::string, UMAP<std::tuple<uint16_t, uint16_t, uint16_t, uint16_t>, cv::Scalar>> attr_map;

    std::string input_json_path;
    std::string output_folder_path;
    std::string data_type="";
    int debug_lidar_bev_mapping = 0;
    std::string flag;
    std::vector<Frame> frame_list;
    std::vector<Frame> lidar_frame_list;
    pcl::PointCloud<ModelPointTypeNew>::Ptr point_cloud_ground_ptr;
    pcl::PointCloud<ModelPointTypeNew>::Ptr point_cloud_ground_2_ptr;
    pcl::PointCloud<ModelPointTypeNew>::Ptr point_cloud_air_ptr;
    pcl::PointCloud<PointXYZIRGBD>::Ptr gcj02_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sm_cloud_ptr;
    int utm_zone;
    Eigen::Vector3d local_orin_utm;
    std::vector<std::mutex> mtx_list;
    std::mutex writePointLock;
    std::mutex mtx;
    std::mutex mtx2;
    // pcl::PointCloud<RenderPointTypeNew>::Ptr combined_pc_ptr;

#if 0
    void init_attr_map() {
        this->attr_map["mmt_to_std"] = UMAP<std::tuple<uint16_t, uint16_t, uint16_t, uint16_t>, cv::Scalar>();
        auto &attr_map_output = this->attr_map["mmt_to_std"];
        // stop line：粉色
        attr_map_output[std::make_tuple(0,4,0,0)] = {255, 0, 255}; // 停止线，unknown
        attr_map_output[std::make_tuple(0,4,0,1)] = {255, 0, 255}; // 停止线，普通停止线
        attr_map_output[std::make_tuple(0,4,0,2)] = {255, 0, 255}; // 停止线，左转待转区停止线
        attr_map_output[std::make_tuple(0,4,0,3)] = {255, 0, 255}; // 停止线，掉头转待转区停止线
        attr_map_output[std::make_tuple(0,4,0,4)] = {255, 0, 255}; // 停止线，右转待转区停止线
        attr_map_output[std::make_tuple(0,4,0,5)] = {255, 0, 255}; // 停止线，直行待转区停止线

        // lane_boundary: 实线：浅灰色； 虚线：黄色
        attr_map_output[std::make_tuple(1,1,0,0)] = {200, 200, 200}; // 车道线，UNKNOWN_COLOR，UNKNOWN
        attr_map_output[std::make_tuple(1,1,0,1)] = {255, 255, 0}; // 车道线，UNKNOWN_COLOR，dashed
        attr_map_output[std::make_tuple(1,1,0,2)] = {200, 200, 200}; // 车道线，UNKNOWN_COLOR，solid
        attr_map_output[std::make_tuple(1,1,0,3)] = {255, 255, 0}; // 车道线，UNKNOWN，double dashed
        attr_map_output[std::make_tuple(1,1,0,4)] = {200, 200, 200}; // 车道线，UNKNOWN，double solid
        attr_map_output[std::make_tuple(1,1,0,5)] = {255, 255, 180}; // 车道线，UNKNOWN，left_dashed_right_solid
        attr_map_output[std::make_tuple(1,1,0,6)] = {255, 255, 180}; // 车道线，UNKNOWN，left_solid_right_dashed

        attr_map_output[std::make_tuple(1,1,1,0)] = {200, 200, 200}; // 车道线，白色线类，UNKNOWN
        attr_map_output[std::make_tuple(1,1,1,1)] = {255, 255, 0}; // 车道线，白色线类，dashed
        attr_map_output[std::make_tuple(1,1,1,2)] = {200, 200, 200}; // 车道线，白色线类，solid
        attr_map_output[std::make_tuple(1,1,1,3)] = {255, 255, 0}; // 车道线，白色线类，double dashed
        attr_map_output[std::make_tuple(1,1,1,4)] = {200, 200, 200}; // 车道线，白色线类，double solid
        attr_map_output[std::make_tuple(1,1,1,5)] = {255, 255, 180}; // 车道线，白色线类，left_dashed_right_solid
        attr_map_output[std::make_tuple(1,1,1,6)] = {255, 255, 180}; // 车道线，白色线类，left_solid_right_dashed

        attr_map_output[std::make_tuple(1,1,2,0)] = {200, 200, 200}; // 车道线，黄色线类，UNKNOWN
        attr_map_output[std::make_tuple(1,1,2,1)] = {255, 255, 0}; // 车道线，黄色线类，dashed
        attr_map_output[std::make_tuple(1,1,2,2)] = {200, 200, 200}; // 车道线，黄色线类，solid
        attr_map_output[std::make_tuple(1,1,2,3)] = {255, 255, 0}; // 车道线，黄色线类，double dashed
        attr_map_output[std::make_tuple(1,1,2,4)] = {200, 200, 200}; // 车道线，黄色线类，double solid
        attr_map_output[std::make_tuple(1,1,2,5)] = {255, 255, 180}; // 车道线，黄色线类，left_dashed_right_solid
        attr_map_output[std::make_tuple(1,1,2,6)] = {255, 255, 180}; // 车道线，黄色线类，left_solid_right_dashed

        // arrow
        attr_map_output[std::make_tuple(1,4,0,0)] = {0, 0, 255}; // 地面标识，unknown
        attr_map_output[std::make_tuple(1,4,0,1)] = {0, 0, 255}; // 地面标识，直行
        attr_map_output[std::make_tuple(1,4,0,2)] = {0, 0, 255}; // 地面标识，右转
        attr_map_output[std::make_tuple(1,4,0,3)] = {0, 0, 255}; // 地面标识，左转
        attr_map_output[std::make_tuple(1,4,0,4)] = {0, 0, 255}; // 地面标识，左转掉头
        attr_map_output[std::make_tuple(1,4,0,5)] = {0, 0, 255}; // 地面标识，右转掉头
        attr_map_output[std::make_tuple(1,4,0,6)] = {0, 0, 255}; // 向左合流
        attr_map_output[std::make_tuple(1,4,0,7)] = {0, 0, 255}; // 向右合流

        // 导流区间：浅红色
        attr_map_output[std::make_tuple(1,5,0,0)] = {255, 128, 128}; // 导流区间

        // 人行道区域：酒红色
        attr_map_output[std::make_tuple(2,3,0,0)] = {165, 42, 42}; // 人行道区域

        // 禁停区域：橘色
        attr_map_output[std::make_tuple(2,1,0,0)] = {255, 128, 0}; // 禁停区域

        // 中心线：绿色
        attr_map_output[std::make_tuple(3,1,0,0)] = {0, 255, 0}; // 中心线，混合车道，unknown
        attr_map_output[std::make_tuple(3,1,0,1)] = {0, 255, 0}; // 中心线，混合车道，普通车道
        attr_map_output[std::make_tuple(3,1,0,5)] = {0, 255, 0}; // 中心线，混合车道，应急车道
        attr_map_output[std::make_tuple(3,1,0,6)] = {0, 255, 0}; // 中心线，混合车道，公交车道
        attr_map_output[std::make_tuple(3,1,0,15)] = {0, 255, 0}; // 中心线，混合车道，非机动车道
        attr_map_output[std::make_tuple(3,1,0,20)] = {0, 255, 0}; // 中心线，混合车道，收费站
        attr_map_output[std::make_tuple(3,1,0,28)] = {0, 255, 0}; // 中心线，混合车道，可变车道
        attr_map_output[std::make_tuple(3,1,0,32)] = {0, 255, 0}; // 中心线，混合车道，潮汐车道
        attr_map_output[std::make_tuple(3,1,0,71)] = {0, 255, 0}; // 中心线，混合车道，右转或掉头待转区
        attr_map_output[std::make_tuple(3,1,0,73)] = {0, 255, 0}; // 中心线，混合车道，不完整车道
        attr_map_output[std::make_tuple(3,1,0,74)] = {0, 255, 0}; // 中心线，混合车道，blocked (byd)

        // 道路边界线: 蓝绿色
        attr_map_output[std::make_tuple(3,2,0,0)] = {0, 255, 255}; // 道路边界，UNKNOWN
        attr_map_output[std::make_tuple(3,2,0,2)] = {0, 255, 255}; // 道路边界，mmt:普通, byd: ROADEDGE_TYPE_FLAT
        attr_map_output[std::make_tuple(3,2,0,4)] = {0, 255, 255}; // 道路边界，mmt:水马, byd: ROADEDGE_TYPE_HIGH
        attr_map_output[std::make_tuple(3,2,0,5)] = {0, 255, 255}; // 道路边界，mmt:锥桶, byd: ROADEDGE_TYPE_LOW
        attr_map_output[std::make_tuple(3,2,0,20)]= {0, 255, 255}; // 道路边界，mmt: 锥桶水马混合/施工牌/防撞桶, byd: ROADEDGE_TYPE_FENCE

        // 交通灯, 粉红色
        attr_map_output[std::make_tuple(4,1,0,1)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,2)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,3)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,4)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,5)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,6)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,7)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,8)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,9)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,10)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,11)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,12)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,13)] = {255, 0, 128}; // 
        attr_map_output[std::make_tuple(4,1,0,14)] = {255, 0, 128}; // 
    }
#endif
    void init_attr_map() {
        this->attr_map["mmt_to_std"] = UMAP<std::tuple<uint16_t, uint16_t, uint16_t, uint16_t>, cv::Scalar>();
        auto &attr_map_output = this->attr_map["mmt_to_std"];
        // stop line：黄色
        attr_map_output[std::make_tuple(0,4,0,0)] = {255, 255, 255}; // 停止线，unknown
        attr_map_output[std::make_tuple(0,4,0,1)] = {255, 255, 255}; // 停止线，普通停止线
        attr_map_output[std::make_tuple(0,4,0,2)] = {255, 255, 255}; // 停止线，左转待转区停止线
        attr_map_output[std::make_tuple(0,4,0,3)] = {255, 255, 255}; // 停止线，掉头转待转区停止线
        attr_map_output[std::make_tuple(0,4,0,4)] = {255, 255, 255}; // 停止线，右转待转区停止线
        attr_map_output[std::make_tuple(0,4,0,5)] = {255, 255, 255}; // 停止线，直行待转区停止线

        // lane_boundary: 实线：白色； 虚线：灰色，虚拟线：蓝色
        attr_map_output[std::make_tuple(1,1,0,0)] = {255,255,255}; // 车道线，UNKNOWN_COLOR，UNKNOWN
        attr_map_output[std::make_tuple(1,1,0,1)] = {134,134,134}; // 车道线，UNKNOWN_COLOR，dashed
        attr_map_output[std::make_tuple(1,1,0,2)] = {183, 183, 183}; // 车道线，UNKNOWN_COLOR，solid
        attr_map_output[std::make_tuple(1,1,0,3)] = {134,134,134}; // 车道线，UNKNOWN，double dashed
        attr_map_output[std::make_tuple(1,1,0,4)] = {183, 183, 183}; // 车道线，UNKNOWN，double solid
        attr_map_output[std::make_tuple(1,1,0,5)] = {134,134,134}; // 车道线，UNKNOWN，left_dashed_right_solid
        attr_map_output[std::make_tuple(1,1,0,6)] = {134,134,134}; // 车道线，UNKNOWN，left_solid_right_dashed

        attr_map_output[std::make_tuple(1,1,1,0)] = {255,255,255}; // 车道线，白色线类，UNKNOWN
        attr_map_output[std::make_tuple(1,1,1,1)] = {134,134,134}; // 车道线，白色线类，dashed
        attr_map_output[std::make_tuple(1,1,1,2)] = {183, 183, 183}; // 车道线，白色线类，solid
        attr_map_output[std::make_tuple(1,1,1,3)] = {134,134,134}; // 车道线，白色线类，double dashed
        attr_map_output[std::make_tuple(1,1,1,4)] = {183, 183, 183}; // 车道线，白色线类，double solid
        attr_map_output[std::make_tuple(1,1,1,5)] = {134,134,134}; // 车道线，白色线类，left_dashed_right_solid
        attr_map_output[std::make_tuple(1,1,1,6)] = {134,134,134}; // 车道线，白色线类，left_solid_right_dashed

        attr_map_output[std::make_tuple(1,1,2,0)] = {255,255,255}; // 车道线，黄色线类，UNKNOWN
        attr_map_output[std::make_tuple(1,1,2,1)] = {134,134,134}; // 车道线，黄色线类，dashed
        attr_map_output[std::make_tuple(1,1,2,2)] = {183, 183, 183}; // 车道线，黄色线类，solid
        attr_map_output[std::make_tuple(1,1,2,3)] = {134,134,134}; // 车道线，黄色线类，double dashed
        attr_map_output[std::make_tuple(1,1,2,4)] = {183, 183, 183}; // 车道线，黄色线类，double solid
        attr_map_output[std::make_tuple(1,1,2,5)] = {134,134,134}; // 车道线，黄色线类，left_dashed_right_solid
        attr_map_output[std::make_tuple(1,1,2,6)] = {134,134,134}; // 车道线，黄色线类，left_solid_right_dashed

        // arrow：浅绿色
        attr_map_output[std::make_tuple(1,4,0,0)] = {175, 219, 17}; // 地面标识，unknown
        attr_map_output[std::make_tuple(1,4,0,1)] = {175, 219, 17}; // 地面标识，直行
        attr_map_output[std::make_tuple(1,4,0,2)] = {175, 219, 17}; // 地面标识，右转
        attr_map_output[std::make_tuple(1,4,0,3)] = {175, 219, 17}; // 地面标识，左转
        attr_map_output[std::make_tuple(1,4,0,4)] = {175, 219, 17}; // 地面标识，左转掉头
        attr_map_output[std::make_tuple(1,4,0,5)] = {175, 219, 17}; // 地面标识，右转掉头
        attr_map_output[std::make_tuple(1,4,0,6)] = {175, 219, 17}; // 向左合流
        attr_map_output[std::make_tuple(1,4,0,7)] = {175, 219, 17}; // 向右合流

        // 导流区间：浅红色
        attr_map_output[std::make_tuple(1,5,0,0)] = {255, 128, 128}; // 导流区间

        // 人行道区域：浅蓝色
        attr_map_output[std::make_tuple(2,3,0,0)] = {139, 188, 248}; // 人行道区域

        // 禁停区域：橘色
        attr_map_output[std::make_tuple(2,1,0,0)] = {255, 128, 0}; // 禁停区域

        // 中心线：绿色
        attr_map_output[std::make_tuple(3,1,0,0)] = {0, 255, 0}; // 中心线，混合车道，unknown
        attr_map_output[std::make_tuple(3,1,0,1)] = {0, 255, 0}; // 中心线，混合车道，普通车道
        attr_map_output[std::make_tuple(3,1,0,5)] = {0, 255, 0}; // 中心线，混合车道，应急车道
        attr_map_output[std::make_tuple(3,1,0,6)] = {0, 255, 0}; // 中心线，混合车道，公交车道
        attr_map_output[std::make_tuple(3,1,0,15)] = {0, 255, 0}; // 中心线，混合车道，非机动车道
        attr_map_output[std::make_tuple(3,1,0,20)] = {0, 255, 0}; // 中心线，混合车道，收费站
        attr_map_output[std::make_tuple(3,1,0,28)] = {0, 255, 0}; // 中心线，混合车道，可变车道
        attr_map_output[std::make_tuple(3,1,0,32)] = {0, 255, 0}; // 中心线，混合车道，潮汐车道
        attr_map_output[std::make_tuple(3,1,0,71)] = {0, 255, 0}; // 中心线，混合车道，右转或掉头待转区
        attr_map_output[std::make_tuple(3,1,0,73)] = {0, 255, 0}; // 中心线，混合车道，不完整车道
        attr_map_output[std::make_tuple(3,1,0,74)] = {0, 255, 0}; // 中心线，混合车道，blocked (byd)

        // 道路边界线: 浅紫色
        attr_map_output[std::make_tuple(3,2,0,0)] = {161, 107, 205}; // 道路边界，UNKNOWN
        attr_map_output[std::make_tuple(3,2,0,2)] = {161, 107, 205}; // 道路边界，mmt:普通, byd: ROADEDGE_TYPE_FLAT
        attr_map_output[std::make_tuple(3,2,0,4)] = {161, 107, 205}; // 道路边界，mmt:水马, byd: ROADEDGE_TYPE_HIGH
        attr_map_output[std::make_tuple(3,2,0,5)] = {161, 107, 205}; // 道路边界，mmt:锥桶, byd: ROADEDGE_TYPE_LOW
        attr_map_output[std::make_tuple(3,2,0,20)]= {161, 107, 205}; // 道路边界，mmt: 锥桶水马混合/施工牌/防撞桶, byd: ROADEDGE_TYPE_FENCE

        // 交通灯, 粉红色
        attr_map_output[std::make_tuple(4,1,0,1)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,2)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,3)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,4)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,5)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,6)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,7)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,8)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,9)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,10)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,11)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,12)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,13)] = {255, 174, 201}; // 
        attr_map_output[std::make_tuple(4,1,0,14)] = {255, 174, 201}; // 
    }


    cv::Scalar get_std_color(std::string version, uint16_t layer, uint16_t label, uint16_t color, uint16_t shape) {
        auto version_it = attr_map.find(version);
        if (version_it == attr_map.end()) {
            throw std::runtime_error("Version not found");
        }

        const auto &ele_type_map = version_it->second;
        auto attr_tuple = std::make_tuple(layer, label, color, shape);

        cv::Scalar std_value = 0;
        auto attr_tuple_it = ele_type_map.find(attr_tuple);
        if (attr_tuple_it == ele_type_map.end()) {
            // std::cout << "请求的版本: " << version << ", 元素类型: " << ele_type 
            //         << ", 属性名称: " << attr_name << ", 原始值: " << input_value << std::endl;     
            // throw std::runtime_error("Attribute not found");
            std_value = {255,255,255};//unknown
        }
        else {
            std_value = attr_tuple_it->second;
        }

        return std_value;
    }

    void get_point_color(ModelPointTypeNew& p) {
        // 获取当前点的所有层的属性
        // 按照优先级，可视化其中的一个属性: 点 > 线 > 面
        // 交通灯 > 停止线 > 箭头 > 道路边界线 > 车道线 > 中心线 > 人行横道 > 禁停区
        cv::Scalar color;
        if(p.cloud_bev_label == 4){ // 停止线
            uint16_t layer = 0;
            color = get_std_color("mmt_to_std", layer, p.cloud_bev_label, p.cloud_bev_label_color, p.cloud_bev_label_shape);
        } else if(p.cloud_bev_label_1 == 4) { // 箭头
            uint16_t layer = 1;
            color = get_std_color("mmt_to_std", layer, p.cloud_bev_label_1, p.cloud_bev_label_1_color, p.cloud_bev_label_1_shape);
        } else if(p.cloud_bev_center_line == 2) { // 道路边界线
            uint16_t layer = 3;
            color = get_std_color("mmt_to_std", layer, p.cloud_bev_center_line, p.cloud_bev_center_line_color, p.cloud_bev_center_line_shape);
        } else if (p.cloud_bev_label_1 == 1) { // 车道线
            uint16_t layer = 1;
            color = get_std_color("mmt_to_std", layer, p.cloud_bev_label_1, p.cloud_bev_label_1_color, p.cloud_bev_label_1_shape);
        } else if (p.cloud_bev_center_line == 1) { // 中心线
            uint16_t layer = 3;
            color = get_std_color("mmt_to_std", layer, p.cloud_bev_center_line, p.cloud_bev_center_line_color, p.cloud_bev_center_line_shape);
        } else if (p.cloud_bev_label_2 == 3) { // 人行横道
            uint16_t layer = 2;
            color = get_std_color("mmt_to_std", layer, p.cloud_bev_label_2, p.cloud_bev_label_2_color, p.cloud_bev_label_2_shape);
        } else if(p.cloud_bev_label_2 == 1) { // 禁停区
            uint16_t layer = 2;
            color = get_std_color("mmt_to_std", layer, p.cloud_bev_label_2, p.cloud_bev_label_2_color, p.cloud_bev_label_2_shape);
        } else {
            uint16_t layer = 100;
            color = get_std_color("mmt_to_std", layer, p.cloud_bev_center_line, p.cloud_bev_center_line_color, p.cloud_bev_center_line_shape);
        }

        p.r = color[0];
        p.g = color[1];
        p.b = color[2];
        
    }

    void read_json()
    {
        nlohmann::json j = nlohmann::json::parse(std::ifstream(input_json_path));
        // 0. utm_zone 及 地图原点utm坐标
        utm_zone = j["utm_num"].get<int>();
        local_orin_utm = j["t_utm_world"].get<Eigen::Vector3d>();

        // 1. bev 数据
        for (auto one : j["data"])
        {
            Frame frame;
            frame.cloud_path = one["cloud_path"];
            frame.bev_json_path = one["bev_json_path"];
            frame.t = one["t"].get<Eigen::Vector3d>();
            frame.q = one["q"].get<Eigen::Quaterniond>();
            frame.T = qab_tab_to_Tab(frame.q, frame.t);
            frame_list.push_back(frame);
        }
        std::sort(frame_list.begin(), frame_list.end(), cmp);
        LOG_INFO("完成读入 bev json, 帧数：{}", frame_list.size());
        // 2. lidar 数据
        for (auto one : j["lidar_data"])
        {
            Frame frame;
            frame.cloud_path = one["cloud_path"];
            frame.t = one["t"].get<Eigen::Vector3d>();
            frame.q = one["q"].get<Eigen::Quaterniond>();
            frame.T = qab_tab_to_Tab(frame.q, frame.t);
            lidar_frame_list.push_back(frame);
        }
        std::sort(lidar_frame_list.begin(), lidar_frame_list.end(), cmp);
        LOG_INFO("完成读入 lidar json, 帧数：{}", lidar_frame_list.size());
    }

    inline Eigen::Vector3i position_xyz_to_10cm_index_xyz(const Eigen::Vector3f &p_xyz)
    {
        return Eigen::Vector3i(int(round((p_xyz[0] + 50000) * 10)), int(round((p_xyz[1] + 50000) * 10)), int(round((p_xyz.z() + 10000) * 10)));
    }

    inline Eigen::Vector2i position_xy_to_2m_index_xy(const Eigen::Vector2f &p_xy)
    {
        return Eigen::Vector2i(int(round((p_xy[0] + 50000) * 0.5)), int(round((p_xy[1] + 50000) * 0.5)));
    }

    inline double sqrt_dis_to_score(const double &sqrt_dis) // 距离得分，(0~1)，距车心越近，得分越高
    {
        return exp(-sqrt_dis / 450);
    }

    // sensor_type: 0->bev, 1->lidar, 
    inline bool update_voxel_map_py_point(const ModelPointTypeNew &point_origin, const ModelPointTypeNew &point, Cm5VoxelMap &cm5_voxel_map, M2VoxelMap &m2_voxel_map, int sensor_type)
    {
        // uint16_t distance_x_cm = round(point_origin.x * 100);
        // uint16_t distance_y_cm = round(fabs(point_origin.y) * 100);
        // float disx = float(distance_x_cm) / 100.;
        // float disy = float(distance_y_cm) / 100.;

        float disx = fabs(point_origin.x);
        float disy = fabs(point_origin.y);
        // if (data_type == "BYD_LIDAR_BEV_B" && sensor_type == 1) {
        //     disx = 0;
        //     disy = 0;
        // }
        
        float point_weight = sqrt_dis_to_score(disx * disx + disy * disy);
        Eigen::Vector2i map_2m_index_xy = position_xy_to_2m_index_xy({point.x, point.y});
        if (m2_voxel_map.closed_to(map_2m_index_xy, M2VoxelCell(point_weight, point.z)) == false)
        {
            return false;
        }
        else
        {
            if (point.a & 0b01000000 && sensor_type == 0) // night
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

    void update_pointcloud_py_counter(Cm5VoxelMap &cm5_voxel_map, pcl::PointCloud<ModelPointTypeNew>::Ptr point_cloud_ground_ptr)
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
                        local_cell.center_line_score = (local_cell.weight * local_cell.center_line_score + neighber_weight * neighber_cell.center_line_score) / sum_weight;
                        local_cell.rgbi = (local_cell.weight * local_cell.rgbi + neighber_weight * neighber_cell.rgbi) / sum_weight;
                        local_cell.weight = sum_weight;

                        local_cell.mergeLabelWeights(local_cell.cloud_bev_label_counter, neighber_cell.cloud_bev_label_counter, neighber_loss);
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_label_shape_counter, neighber_cell.cloud_bev_label_shape_counter, neighber_loss);                        
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_label_color_counter, neighber_cell.cloud_bev_label_color_counter, neighber_loss);
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_label_1_counter, neighber_cell.cloud_bev_label_1_counter, neighber_loss);
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_label_1_shape_counter, neighber_cell.cloud_bev_label_1_shape_counter, neighber_loss);
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_label_1_color_counter, neighber_cell.cloud_bev_label_1_color_counter, neighber_loss);
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_label_2_counter, neighber_cell.cloud_bev_label_2_counter, neighber_loss);                        
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_label_2_shape_counter, neighber_cell.cloud_bev_label_2_shape_counter, neighber_loss);                        
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_label_2_color_counter, neighber_cell.cloud_bev_label_2_color_counter, neighber_loss);                        
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_center_line_counter, neighber_cell.cloud_bev_center_line_counter, neighber_loss);                        
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_center_line_shape_counter, neighber_cell.cloud_bev_center_line_shape_counter, neighber_loss);                        
                        local_cell.mergeLabelWeights(local_cell.cloud_bev_center_line_color_counter, neighber_cell.cloud_bev_center_line_color_counter, neighber_loss);                                               
                    }
                }
                // point_cloud_ground_ptr->points[cell.index] = local_cell.to_point(voxel_index_xy[0], voxel_index_xy[1]);

                ModelPointTypeNew p_out = local_cell.to_point(voxel_index_xy[0], voxel_index_xy[1]);
                // TODO: 过滤点云过滤点云
                if (p_out.cloud_bev_label == 0 &&
                    p_out.cloud_bev_label_1 == 0 &&
                    p_out.cloud_bev_label_2 == 0 &&
                    p_out.cloud_bev_center_line == 0)//&& flat_scan_ptr->points[i].cloud_bev_traffic_light == 0)
                { 
                    continue; // 跳过该点
                }

                get_point_color(p_out);
                point_cloud_ground_ptr->points[cell.index] = p_out;
            }
        }

        point_cloud_ground_ptr->points.erase(std::remove_if(point_cloud_ground_ptr->points.begin(), point_cloud_ground_ptr->points.end(),
                                            [](const ModelPointTypeNew& p){
                                                return p.cloud_bev_label == 0 &&
                                                        p.cloud_bev_label_1 == 0 &&
                                                        p.cloud_bev_label_2 == 0 &&
                                                        p.cloud_bev_center_line == 0;}), 
                                            point_cloud_ground_ptr->points.end());
        point_cloud_ground_ptr->height = 1;
        point_cloud_ground_ptr->width = point_cloud_ground_ptr->size();
    }

    void run_bev_mapping_debug()
    {
        Cm5VoxelMap cm5_voxel_map_1;
        Cm5VoxelMap cm5_voxel_map_2;
        M2VoxelMap m2_voxel_map_1;
        M2VoxelMap m2_voxel_map_2;
        Cm10VoxelMap air_voxel_map;
        std::random_device rd;
        std::mt19937 rng(rd());

        LOG_INFO("[debug] start update_voxel_map_py_point()");
        int finish_f = 0;

        // pcl::PointCloud<RenderPointTypeNew> temp_combined_pc;
        #pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
        for (auto &frame : frame_list)
        {
            finish_f++;
            if (finish_f % 100 == 0)
            {
                LOG_INFO("[debug]run_mapping:BEV {}/{}", finish_f, frame_list.size());
            }

            pcl::PointCloud<ModelPointTypeNew>::Ptr frame_pc_ptr(new pcl::PointCloud<ModelPointTypeNew>);
            pcl::PointCloud<ModelPointTypeNew>::Ptr frame_pc_ptr_origin(new pcl::PointCloud<ModelPointTypeNew>);
            pcl::io::loadPCDFile(frame.cloud_path, *frame_pc_ptr);
            *frame_pc_ptr_origin = *frame_pc_ptr;
            pcl::transformPointCloud(*frame_pc_ptr, *frame_pc_ptr, frame.T);

            for (auto i = 0; i < frame_pc_ptr->points.size(); i++)
            {   
                auto &p = frame_pc_ptr->points[i];
                auto &p_origin = frame_pc_ptr_origin->points[i];
                if ((p.a & 3) == 3)
                {
                    int sensor_type = 0;
                    bool has_updated = update_voxel_map_py_point(p_origin, p, cm5_voxel_map_1, m2_voxel_map_1, sensor_type);
                    if (has_updated == false)
                    {
                        update_voxel_map_py_point(p_origin, p, cm5_voxel_map_2, m2_voxel_map_2, sensor_type);
                    }
                }
                else
                {
                    Eigen::Vector3i index_xyz = position_xyz_to_10cm_index_xyz(p.getVector3fMap());
                    ModelPointTypeNew out_p;
                    out_p.getVector3fMap() = p.getVector3fMap();
                    out_p.rgb = p.rgb;
                    out_p.intensity = p.intensity;
                    // out_p.label = p.label;
                    // out_p.cloud_pano_seg = p.cloud_pano_seg;
                    // out_p.cloud_bev_traffic_light = p.cloud_bev_traffic_light;
                    air_voxel_map.update(index_xyz, out_p);
                }
            }
        }

        bool debug_save_lidar_pcd = true;
        if(debug_save_lidar_pcd && frame_list.size() > 0) {
            LOG_INFO("[debug] finish bev update_voxel_map_py_point()");
            LOG_INFO("update_pointcloud_py_counter() 1");
            update_pointcloud_py_counter(cm5_voxel_map_1, point_cloud_ground_ptr);
            LOG_INFO("update_pointcloud_py_counter() 2");
            update_pointcloud_py_counter(cm5_voxel_map_2, point_cloud_ground_2_ptr);
            LOG_INFO("[debug] finish bev update_pointcloud_py_counter()");
            *point_cloud_ground_ptr = *point_cloud_ground_ptr + *point_cloud_ground_2_ptr;
            if (!point_cloud_ground_ptr->empty()) {
                pcl::io::savePCDFileBinary(path_join(output_folder_path, "ground_bev.pcd"), *point_cloud_ground_ptr);
                // pcl::io::savePCDFileASCII(path_join(output_folder_path, "ground_bev.pcd"), *point_cloud_ground_ptr);
            }
            point_cloud_ground_ptr->clear();
            point_cloud_ground_2_ptr->clear();
            point_cloud_air_ptr->clear();
        }
    }

    void read_sm(std::string& bev_label_path, Eigen::Matrix4d& T)
    {
        if(bev_label_path == "") {
            return;
        }

        std::ifstream ifs(bev_label_path);
        if (!ifs.is_open()) {
            // throw std::runtime_error("无法打开文件: " + bev_label_path);
            return;
        }

        nlohmann::json j = nlohmann::json::parse(ifs);
        std::string cat_version = j.value("cat_version", "BYD_BEV");

        auto &instance_list = j["detail"]["instance_list"];
        for (int instance_id = 0; instance_id < instance_list.size(); ++instance_id) {
            auto &one_instance = instance_list[instance_id];
            std::vector<int> raw_bev_ids = one_instance["attrs"]["type"];
            // float score = one_instance["attrs"]["score"];
            if(raw_bev_ids.size() > 0 && raw_bev_ids[0] == 150) {
                if (one_instance["data"].contains("points") && one_instance["data"]["points"].is_array() && !one_instance["data"]["points"].empty()) {
                    for (const auto &one_point : one_instance["data"]["points"]) {
                        Eigen::Vector4d p(one_point[0], one_point[1], 0, 1);
                        Eigen::Vector4d p_local = T * p;
                        sm_cloud_ptr->points.push_back(pcl::PointXYZI(p_local.x(), p_local.y(), p_local.z(), 0));
                    }
                } else {
                    // LOG_WARN("缺少有效字段");
                    continue;
                }

        
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

        int finish_f = 0;
        if (data_type == "BYD_LIDAR_BEV_B") {
            LOG_INFO("start update_voxel_map_py_point()");

            #pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
            for (auto &frame : lidar_frame_list)
            {
                finish_f++;
                if (finish_f % 100 == 0)
                {
                    LOG_INFO("run_mapping:LIDAR {}/{}", finish_f, lidar_frame_list.size());
                }

                pcl::PointCloud<ModelPointTypeNew>::Ptr frame_pc_ptr(new pcl::PointCloud<ModelPointTypeNew>);
                pcl::PointCloud<ModelPointTypeNew>::Ptr frame_pc_ptr_origin(new pcl::PointCloud<ModelPointTypeNew>);
                pcl::io::loadPCDFile(frame.cloud_path, *frame_pc_ptr);
                *frame_pc_ptr_origin = *frame_pc_ptr;
                pcl::transformPointCloud(*frame_pc_ptr, *frame_pc_ptr, frame.T);

                for (auto i = 0; i < frame_pc_ptr->points.size(); i++)
                {   
                    auto &p = frame_pc_ptr->points[i];
                    auto &p_origin = frame_pc_ptr_origin->points[i];
                    if ((p.a & 3) == 3)
                    {
                        int sensor_type = 1;
                        bool has_updated = update_voxel_map_py_point(p_origin, p, cm5_voxel_map_1, m2_voxel_map_1, sensor_type);
                        if (has_updated == false)
                        {
                            update_voxel_map_py_point(p_origin, p, cm5_voxel_map_2, m2_voxel_map_2, sensor_type);
                        }
                    }
                    else
                    {
                        Eigen::Vector3i index_xyz = position_xyz_to_10cm_index_xyz(p.getVector3fMap());
                        ModelPointTypeNew out_p;
                        out_p.getVector3fMap() = p.getVector3fMap();
                        out_p.rgb = p.rgb;
                        out_p.intensity = p.intensity;
                        // out_p.label = p.label;
                        // out_p.cloud_pano_seg = p.cloud_pano_seg;
                        // out_p.cloud_bev_traffic_light = p.cloud_bev_traffic_light;
                        air_voxel_map.update(index_xyz, out_p);
                    }
                }
            }

            bool debug_save_lidar_pcd = true;
            if(debug_save_lidar_pcd && lidar_frame_list.size() > 0 && (debug_lidar_bev_mapping == 2 || debug_lidar_bev_mapping == 3)) {
                LOG_INFO("finish lidar update_voxel_map_py_point()");
                LOG_INFO("update_pointcloud_py_counter() 1");
                update_pointcloud_py_counter(cm5_voxel_map_1, point_cloud_ground_ptr);
                LOG_INFO("update_pointcloud_py_counter() 2");
                update_pointcloud_py_counter(cm5_voxel_map_2, point_cloud_ground_2_ptr);
                LOG_INFO("finish lidar update_pointcloud_py_counter()");

                // 输出纯激光点云
                *point_cloud_ground_ptr = *point_cloud_ground_ptr + *point_cloud_ground_2_ptr;
                output("ground_lidar.pcd", "pointcloud_semantic_pure_lidar_gcj02.pcd");

                point_cloud_ground_ptr->clear();
                point_cloud_ground_2_ptr->clear();
                point_cloud_air_ptr->clear();
            }
        }

        finish_f = 0;
        #pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
        for (auto &frame : frame_list)
        {
            finish_f++;
            if (finish_f % 100 == 0)
            {
                LOG_INFO("run_mapping:BEV {}/{}", finish_f, frame_list.size());
            }
            // read_sm(frame.bev_json_path, frame.T);

            pcl::PointCloud<ModelPointTypeNew>::Ptr frame_pc_ptr(new pcl::PointCloud<ModelPointTypeNew>);
            pcl::PointCloud<ModelPointTypeNew>::Ptr frame_pc_ptr_origin(new pcl::PointCloud<ModelPointTypeNew>);
            pcl::io::loadPCDFile(frame.cloud_path, *frame_pc_ptr);
            *frame_pc_ptr_origin = *frame_pc_ptr;
            pcl::transformPointCloud(*frame_pc_ptr, *frame_pc_ptr, frame.T);
            // if (!frame_pc_ptr->empty())
            // {
            //     #pragma omp critical
            //     {
            //         temp_combined_pc += *frame_pc_ptr;
            //     }
            // }


            // std::vector<uint64_t> air_index;
            // air_index.reserve(frame_pc_ptr->points.size());
            for (auto i = 0; i < frame_pc_ptr->points.size(); i++)
            {   
                auto &p = frame_pc_ptr->points[i];
                auto &p_origin = frame_pc_ptr_origin->points[i];
                if ((p.a & 3) == 3)
                {
                    int sensor_type = 0;
                    bool has_updated = update_voxel_map_py_point(p_origin, p, cm5_voxel_map_1, m2_voxel_map_1, sensor_type);
                    if (has_updated == false)
                    {
                        update_voxel_map_py_point(p_origin, p, cm5_voxel_map_2, m2_voxel_map_2, sensor_type);
                    }
                }
                else
                {
                    Eigen::Vector3i index_xyz = position_xyz_to_10cm_index_xyz(p.getVector3fMap());
                    ModelPointTypeNew out_p;
                    out_p.getVector3fMap() = p.getVector3fMap();
                    out_p.rgb = p.rgb;
                    out_p.intensity = p.intensity;
                    // out_p.label = p.label;
                    // out_p.cloud_pano_seg = p.cloud_pano_seg;
                    // out_p.cloud_bev_traffic_light = p.cloud_bev_traffic_light;
                    air_voxel_map.update(index_xyz, out_p);
                }
            }
        }

        // combined_pc_ptr->reserve(temp_combined_pc.size());
        // for (auto &p_out : temp_combined_pc) {
        //     if (p_out.cloud_bev_label == 0 &&
        //         p_out.cloud_bev_label_1 == 0 &&
        //         p_out.cloud_bev_label_2 == 0 &&
        //         p_out.cloud_bev_center_line == 0)
        //     { 
        //         continue;
        //     }

        //     combined_pc_ptr->push_back(p_out);
        // }
        // combined_pc_ptr->height = 1;
        // combined_pc_ptr->width = combined_pc_ptr->size();

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
        LOG_INFO("finish bev update_voxel_map_py_point()");
        LOG_INFO("update_pointcloud_py_counter() 1");
        update_pointcloud_py_counter(cm5_voxel_map_1, point_cloud_ground_ptr);
        LOG_INFO("update_pointcloud_py_counter() 2");
        update_pointcloud_py_counter(cm5_voxel_map_2, point_cloud_ground_2_ptr);
        LOG_INFO("finish bev update_pointcloud_py_counter()");
        *point_cloud_ground_ptr = *point_cloud_ground_ptr + *point_cloud_ground_2_ptr;
    }

    void output(std::string local_map_name, std::string global_map_name)
    {
        // if (!combined_pc_ptr->empty())
        // {
        //     pcl::io::savePCDFileBinary(path_join(output_folder_path, "raw_combined.pcd"), *combined_pc_ptr);
        // }
    
        if (!point_cloud_ground_ptr->empty()) {
            LOG_INFO("output local_map_name:{}, global_map_name:{}", local_map_name, global_map_name);

            pcl::io::savePCDFileBinary(path_join(output_folder_path, local_map_name), *point_cloud_ground_ptr);
            // pcl::io::savePCDFileASCII(path_join(output_folder_path, local_map_name), *point_cloud_ground_ptr);

            // 转换为gcj02，给到平台作为底图。
            localmap2wgs(point_cloud_ground_ptr, gcj02_cloud_ptr);
            pcl::io::savePCDFileBinary(path_join(output_folder_path, global_map_name), *gcj02_cloud_ptr);
            // pcl::io::savePCDFileASCII(path_join(output_folder_path, global_map_name), *gcj02_cloud_ptr);

            if(0) {
                localmap2localmap(point_cloud_ground_ptr, gcj02_cloud_ptr);
                pcl::io::savePCDFileBinary(path_join(output_folder_path, "ground_no_lc.pcd"), *gcj02_cloud_ptr);
            }
        } else {
            LOG_ERROR("point_cloud_ground_ptr empty()");
        }

        if (!sm_cloud_ptr->empty()) {
            sm_cloud_ptr->height = 1;
            sm_cloud_ptr->width = sm_cloud_ptr->size();
             pcl::io::savePCDFileBinary(path_join(output_folder_path, "sm.pcd"), *sm_cloud_ptr);
        }

#if 0
        if (!point_cloud_air_ptr->empty())
        {
            pcl::io::savePCDFileBinary(path_join(output_folder_path, "air.pcd"), *point_cloud_air_ptr);
        }
        else
        {
            LOG_ERROR("point_cloud_air_ptr empty()");
        }

        pcl::PointCloud<ModelPointTypeNew>::Ptr final_map_ptr(new pcl::PointCloud<ModelPointTypeNew>);
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
            ModelPointTypeNew min_p1, max_p1;
            pcl::getMinMax3D(*final_map_ptr, min_p1, max_p1);
            std::vector<std::string> lines;
            lines.push_back(str_format("%f %f %f", min_p1.x, min_p1.y, min_p1.z));
            lines.push_back(str_format("%f %f %f", max_p1.x, max_p1.y, max_p1.z));
            write_lines_to_file_overwrite(path_join(output_folder_path, "final_map_bbox.txt"), lines);
        }
#endif
    }

    void localmap2wgs(const pcl::PointCloud<ModelPointTypeNew>::Ptr & local_map_ptr, pcl::PointCloud<PointXYZIRGBD>::Ptr &wgs_map_ptr)
    {
        wgs_map_ptr->clear();
        PointXYZIRGBD gcj02_p;
        for (auto p : local_map_ptr->points){
            if (p.cloud_bev_center_line == 1 && p.cloud_bev_label_1 != 4) {
                continue;
            }
            
            // 修改颜色
            get_point_color(p);

            Eigen::Vector3d local_p(p.x, p.y, p.z);
            Eigen::Vector3d utm_p = local_p + local_orin_utm;
            Eigen::Vector3d wgs;
            if (utm2wgs(utm_zone, utm_p, wgs) == 0) {
                gcj02_p.x = wgs.x();
                gcj02_p.y = wgs.y();
                gcj02_p.z = wgs.z();
                gcj02_p.r = p.r;
                gcj02_p.g = p.g;
                gcj02_p.b = p.b;
                wgs_map_ptr->points.push_back(gcj02_p);
            }  
        }

        // wgs_map_ptr->points.resize(cm5_voxel_map.cell_num);
        wgs_map_ptr->height = 1;
        wgs_map_ptr->width = wgs_map_ptr->size();
    }

    void localmap2localmap(const pcl::PointCloud<ModelPointTypeNew>::Ptr& local_map_ptr, pcl::PointCloud<PointXYZIRGBD>::Ptr& local_map_ptr2)
    {
        local_map_ptr2->clear();
        PointXYZIRGBD local_p;
        for (auto p : local_map_ptr->points){
            if (p.cloud_bev_center_line == 1 && p.cloud_bev_label_1 != 4) {
                continue;
            }
            
            // 修改颜色
            get_point_color(p);

            local_p.x = p.x;
            local_p.y = p.y;
            local_p.z = p.z;
            local_p.r = p.r;
            local_p.g = p.g;
            local_p.b = p.b;
            local_map_ptr2->points.push_back(local_p);
        }

        // local_map_ptr2->points.resize(cm5_voxel_map.cell_num);
        local_map_ptr2->height = 1;
        local_map_ptr2->width = local_map_ptr2->size();
    }
};

int main(int argc, char **argv)
{
    ArgParser arg_parser;
    arg_parser.add<std::string>("input", '\0', "输入 json path", true, "");
    arg_parser.add<std::string>("output", '\0', "输出文件夹", true, "");
    arg_parser.add<std::string>("data_type", '\0', "数据类型", true, "");
    arg_parser.add<int>("debug_lidar_bev_mapping", '\0', "调试是task_type=3,激光pose+BEV生成bev地图", true, 0);
    arg_parser.parse_check(argc, argv);

    System system;
    system.input_json_path = arg_parser.get<std::string>("input");
    system.output_folder_path = arg_parser.get<std::string>("output");
    system.data_type = arg_parser.get<std::string>("data_type");
    system.debug_lidar_bev_mapping = arg_parser.get<int>("debug_lidar_bev_mapping");
    add_folder_if_not_exist(path_join(system.output_folder_path, "debug"));

    auto logger = Logger::instance();
    logger->setLogLevel(spdlog::level::level_enum::info, spdlog::level::level_enum::trace);
    logger->setup(path_join(system.output_folder_path, "log", _time_str_s() + "_flat_mapper.log"), "flat_mapper");

    LOG_INFO("============================================");
    LOG_INFO("  Hello flat mapper");
    LOG_INFO("  input:{}", system.input_json_path);
    LOG_INFO("  output:{}", system.output_folder_path);
    LOG_INFO("  data_type:{}", system.data_type);
    LOG_INFO("  debug_lidar_bev_mapping:{}", system.debug_lidar_bev_mapping);
    LOG_INFO("============================================");

    system.init_attr_map();
    system.read_json();

    if (system.data_type == "BYD_LIDAR_BEV_B" && (system.debug_lidar_bev_mapping == 1 || system.debug_lidar_bev_mapping == 3)) {
        system.run_bev_mapping_debug();
    } 

    system.run_mapping();
    LOG_INFO("output()");
    system.output("ground.pcd", "pointcloud_semantic_gcj02.pcd");

    LOG_INFO("Finish flat mapper");
    return 0;
}

/*

/home/test/code/fsd_map_road_model_dev/mapping/simple_mapper/bin/flat_mapper --input=/home/test/lidar_mapping_ws/26363/mapping_output/ff_mapping/ff_mapping_info.json --output=/home/test/lidar_mapping_ws/26363/mapping_output/ff_mapping
*/