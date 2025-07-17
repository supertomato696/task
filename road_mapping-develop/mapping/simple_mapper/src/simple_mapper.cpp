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

// clang-format off
struct EIGEN_ALIGN16 MyPointXYZL
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;
    uint16_t label;
    uint8_t geom_type;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointXYZL,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint8_t, geom_type, geom_type))
// clang-format on

class FastVoxelMap
{
public:
    FastVoxelMap()
    {
        std::vector<std::mutex> list(zone_num);
        mtx_list.swap(list);
        hash_map_list.resize(zone_num);
    };

    inline uint64_t vector3f_to_hash_id(const Eigen::Vector3f &p)
    {
        return (uint64_t(round((p[0] + 50000) * 10)) << 42) + (uint64_t(round((p[1] + 50000) * 10)) << 21) + (uint64_t(round((p[0] + 50000) * 10)));
    }

    inline uint32_t vector3f_to_zone_id(const Eigen::Vector3f &p)
    {
        return uint32_t(p[0] + p[1] + p[2]) & 0b111111;
    }

    inline void update(const MyPointXYZL &point_in)
    {
        auto hash_id = vector3f_to_hash_id(point_in.getVector3fMap());
        auto zone_id = vector3f_to_zone_id(point_in.getVector3fMap());
        std::lock_guard<std::mutex> lck(mtx_list[zone_id]);
        auto &hash_map = hash_map_list[zone_id];
        auto r = hash_map.insert(std::make_pair(hash_id, point_in));
        if (r.second == false)
        {
            auto &point = r.first->second;
            if ((point_in.intensity - int(point_in.intensity)) < (point.intensity - int(point.intensity)))
                point = point_in;
        }
    }

    void to_point(pcl::PointCloud<MyPointXYZL>::Ptr output_pc_ptr)
    {
        for (auto &hash_map : hash_map_list)
        {
            for (auto &[voxel_index_xy, point] : hash_map)
            {
                output_pc_ptr->points.push_back(point);
            }
        };
        output_pc_ptr->height = 1;
        output_pc_ptr->width = output_pc_ptr->size();
    }

    int zone_num = 64;
    std::vector<std::mutex> mtx_list;
    std::vector<phmap::flat_hash_map<uint64_t, MyPointXYZL>> hash_map_list;
};

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

class System
{
public:
    System()
    {
        final_map_ptr = pcl::PointCloud<MyPointXYZL>::Ptr(new pcl::PointCloud<MyPointXYZL>);
    }
    ~System() {}

    std::string input_json_path;
    std::string output_folder_path;
    std::string flag;
    std::vector<Frame> frame_list;
    pcl::PointCloud<MyPointXYZL>::Ptr final_map_ptr;
    std::mutex writePointLock;

    void read_json()
    {
        nlohmann::json j;
        std::ifstream ifs(input_json_path);
        ifs >> j;
        for (auto one : j["data"])
        {
            Frame frame;
            frame.cloud_path = one["cloud_path"];
            for (int i = 0; i < 3; i++)
            {
                frame.t[i] = one["t"][i];
            }
            Eigen::Vector4d q;
            for (int i = 0; i < 4; i++)
            {
                q[i] = one["q"][i];
            }
            frame.q = Eigen::Quaterniond(q);
            frame.T = qab_tab_to_Tab(frame.q, frame.t);
            frame_list.push_back(frame);
        }
        LOG_INFO("完成读入 json, 帧数：{}", frame_list.size());
    }

    struct Point2DHash
    {
        std::size_t operator()(const std::tuple<int, int, int> &p) const
        {
            return (std::get<2>(p) << 40) + (std::get<1>(p) << 20) + std::get<0>(p);
        }
    };

    void run_mapping()
    {
        std::string debug_folder = "";
        if (std::string(getenv("DEBUG_LIDAR_MAPPING")) == "YES")
        {
            debug_folder = path_join(output_folder_path, "debug");
            add_folder_if_not_exist(path_join(debug_folder));
        }

        FastVoxelMap fast_voxel_map;

        int finish_f = 0;
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() / 2))
        for (auto &frame : frame_list)
        {
            finish_f++;
            if (finish_f % 100 == 0)
            {
                LOG_INFO("run_mapping {}/{}", finish_f, frame_list.size());
            }

            pcl::PointCloud<MyPointXYZL>::Ptr pc_ptr(new pcl::PointCloud<MyPointXYZL>);
            pcl::io::loadPCDFile(frame.cloud_path, *pc_ptr);
            pcl::transformPointCloud(*pc_ptr, *pc_ptr, frame.T);

            if (std::string(getenv("DEBUG_LIDAR_MAPPING")) == "YES" && (!pc_ptr->empty())) // 开启调试
            {
                pcl::io::savePCDFileBinary(path_join(output_folder_path, "debug", get_file_name_in_path(frame.cloud_path)), *pc_ptr);
            }

            for (MyPointXYZL &p : pc_ptr->points)
            {
                fast_voxel_map.update(p);
            }
        }

        fast_voxel_map.to_point(final_map_ptr);
    }

    void output()
    {
        if (!final_map_ptr->empty())
        {
            pcl::io::savePCDFileBinary(path_join(output_folder_path, "final_map.pcd"), *final_map_ptr);
        }
        else
        {
            LOG_ERROR("final_map_ptr empty()");
        }

        {
            MyPointXYZL min_p1, max_p1;
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

    auto logger = Logger::instance();
    logger->setLogLevel(spdlog::level::level_enum::info, spdlog::level::level_enum::trace);
    logger->setup(system.output_folder_path + "/log/" + _time_str_s() + "_simple_mapper.log", "simple_mapper");

    LOG_INFO("============================================");
    LOG_INFO("  Hello simple mapper");
    LOG_INFO("  input:{}", system.input_json_path);
    LOG_INFO("  output:{}", system.output_folder_path);
    LOG_INFO("============================================");

    system.read_json();
    system.run_mapping();
    system.output();

    LOG_INFO("Finish simple mapper");
    return 0;
}