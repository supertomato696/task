#include <Eigen/Eigenvalues>

#include <cstdint>
#include <random>
#include <ctime>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <malloc.h>
#include <nlohmann/json.hpp>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
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
#include <set>
#include "util.h"
#include "omp.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "include/util.h"
#include <boost/filesystem/path.hpp> 
#include <boost/filesystem/operations.hpp>


#include "util.h"

struct EIGEN_ALIGN16 RenderPointType
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;
    uint16_t label;
    uint8_t cloud_pano_seg;
    uint8_t cloud_line_seg;
    uint8_t cloud_bev_label;
    uint16_t distance_x_cm;
    uint16_t distance_y_cm;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(RenderPointType,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint8_t, cloud_pano_seg, cloud_pano_seg)
                                  (uint8_t, cloud_line_seg, cloud_line_seg)
                                  (uint8_t, cloud_bev_label, cloud_bev_label)
                                  (uint16_t, distance_x_cm, distance_x_cm)
                                  (uint16_t, distance_y_cm, distance_y_cm))

using namespace std;

// void custom_voxel_filter_label(pcl::PointCloud<RenderPointType> &input, float lx, float ly, float lz)
// {
//     pcl::PointCloud<RenderPointType>::Ptr output(new pcl::PointCloud<RenderPointType>);
//     if (input.size() == 0)
//     {
//         log_warning("[custom_voxel_filter] No input dataset given!");
//         output->width = output->height = 0;
//         output->clear();
//         return;
//     }

//     output->height = 1;
//     output->is_dense = true;
//     Eigen::Vector4f min_p, max_p;
//     Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
//     Eigen::Vector4f leaf_size_;
//     Eigen::Array4f inverse_leaf_size_;
//     pcl::getMinMax3D(input, min_p, max_p);
//     leaf_size_ << lx, ly, lz, 1;
//     inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();

//     unsigned int min_points_per_voxel_ = 1;
//     std::vector<unsigned int> indices_;
//     indices_.resize(input.points.size());
//     for (size_t j = 0; j < input.points.size(); ++j)
//     {
//         indices_[j] = static_cast<unsigned int>(j);
//     }

//     min_b_[0] = static_cast<int>(std::floor(min_p[0] * inverse_leaf_size_[0]));
//     max_b_[0] = static_cast<int>(std::floor(max_p[0] * inverse_leaf_size_[0]));
//     min_b_[1] = static_cast<int>(std::floor(min_p[1] * inverse_leaf_size_[1]));
//     max_b_[1] = static_cast<int>(std::floor(max_p[1] * inverse_leaf_size_[1]));
//     min_b_[2] = static_cast<int>(std::floor(min_p[2] * inverse_leaf_size_[2]));
//     max_b_[2] = static_cast<int>(std::floor(max_p[2] * inverse_leaf_size_[2]));
//     div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
//     div_b_[3] = 0;
//     divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

//     struct cloud_point_index_idx
//     {
//         unsigned int idx;               // voxel index
//         unsigned int cloud_point_index; // point index

//         cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}
//         bool operator<(const cloud_point_index_idx &p) const { return (idx < p.idx); }
//     };
//     std::vector<cloud_point_index_idx> index_vector;
//     index_vector.reserve(indices_.size());

//     for (const auto &index : indices_)
//     {
//         if (!input.is_dense)
//             // Check if the point is invalid
//             if (!pcl::isXYZFinite(input.points[index]))
//                 continue;

//         int ijk0 = static_cast<int>(std::floor(input.points[index].x * inverse_leaf_size_[0]) - static_cast<float>(min_b_[0]));
//         int ijk1 = static_cast<int>(std::floor(input.points[index].y * inverse_leaf_size_[1]) - static_cast<float>(min_b_[1]));
//         int ijk2 = static_cast<int>(std::floor(input.points[index].z * inverse_leaf_size_[2]) - static_cast<float>(min_b_[2]));

//         // Compute the centroid leaf index
//         int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
//         index_vector.emplace_back(static_cast<unsigned int>(idx), index);
//     }
//     std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

//     unsigned int total = 0;
//     unsigned int index = 0;
//     std::vector<std::vector<int>> first_and_last_indices_vector;
//     first_and_last_indices_vector.reserve(index_vector.size());
//     while (index < index_vector.size())
//     {
//         std::vector<int> point_index_in_voxel;
//         unsigned int i = index + 1;
//         point_index_in_voxel.push_back(index_vector[index].cloud_point_index);
//         while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
//         {
//             point_index_in_voxel.push_back(index_vector[i].cloud_point_index);
//             ++i;
//         }

//         if (i - index >= min_points_per_voxel_)
//         {
//             ++total;
//             first_and_last_indices_vector.push_back(point_index_in_voxel);
//         }
//         index = i;
//     }
//     output->resize(total);

//     index = 0;
//     for (const auto &cp : first_and_last_indices_vector)
//     {   
//         ////////////////////////////////////////////////////////
//         std::map<uint16_t,int> label_vector;
//         std::map<uint8_t,int> semantic_label_vector;
//         std::map<uint8_t,int> cloud_label_vector;
//         int label_num = 0;
//         int cloud_label_num = 0;
//         int semantic_label_num = 0;
//         for (const auto &per_index : cp)
//         {
//             auto per_cloud = input.points[per_index];
//             if (per_cloud.cloud_pano_seg!= 255 ){
//                 if(semantic_label_vector.find(per_cloud.cloud_pano_seg) == semantic_label_vector.end()){
//                 semantic_label_vector[per_cloud.cloud_pano_seg] =1;
//                 semantic_label_num+=1;
//                 }
//                 else{
//                 semantic_label_vector[per_cloud.cloud_pano_seg] +=1;
//                 semantic_label_num+=1;
//                 }
//             }
//             if (per_cloud.label!= 255 ){
//                 if(label_vector.find(per_cloud.label) == label_vector.end()){
//                 label_vector[per_cloud.label] =1;
//                 label_num+=1;
//                 }
//                 else{
//                 label_vector[per_cloud.label] +=1;
//                 label_num+=1;
//                 }
//             }
//             if (per_cloud.cloud_line_seg!= 255){
//                 if(cloud_label_vector.find(per_cloud.cloud_line_seg) == cloud_label_vector.end()){
//                 cloud_label_vector[per_cloud.cloud_line_seg] =1;
//                 cloud_label_num+=1;
//                 }
//                 else{
//                 cloud_label_vector[per_cloud.cloud_line_seg] +=1;
//                 cloud_label_num+=1;
//                 }
//             }
//         }
//         std::vector<std::pair<uint16_t, int>> temp_label(label_vector.begin(), label_vector.end());
//         std::vector<std::pair<uint8_t, int>> temp_cloud_label(cloud_label_vector.begin(), cloud_label_vector.end());
//         std::vector<std::pair<uint8_t, int>> temp_semantic_label(semantic_label_vector.begin(), semantic_label_vector.end());

//         std::sort(temp_semantic_label.begin(), temp_semantic_label.end(), [](auto& a, auto& b) {return a.second >= b.second;});
//         std::sort(temp_label.begin(), temp_label.end(), [](auto& a, auto& b) {return a.second >= b.second;});
//         std::sort(temp_cloud_label.begin(), temp_cloud_label.end(), [](auto& a, auto& b) {return a.second >= b.second;});
//         if (cloud_label_num>=1) {input.points[cp[0]].cloud_line_seg = temp_cloud_label[0].first;}
//         else{input.points[cp[0]].cloud_line_seg = 255;}
//         if (semantic_label_num>=1) {input.points[cp[0]].cloud_pano_seg = temp_semantic_label[0].first;}
//         else{input.points[cp[0]].cloud_pano_seg = 255;}
//         if (label_num>=1) {input.points[cp[0]].label = temp_label[0].first;}
//         else{input.points[cp[0]].label = 255;}
//         /////////////////////////////////////////////////////

//         output->points[index] = input.points[cp[0]];
//         ++index;
//     }
//     output->width = output->size();
//     input = *output;
// }

int main(int argc, char **argv)
{
    ArgParser arg_parser;
    arg_parser.add<std::string>("task_folder", '\0', "输入 task_folder path", true, "");
    arg_parser.add<std::string>("info_json_path", '\0', "输入 info_json_path path", true, "");
    arg_parser.add<std::string>("output_folder", '\0', "输出 output path", true, "");
    arg_parser.add<std::string>("filter_flag", '\0', "输出 output path", true, "1");
    arg_parser.parse_check(argc, argv);

    string task_folder;
    string info_json_path;
    // string submap_json;
    string output_path;
    string filter_flag;
    vector<long> mono_link;
    task_folder = arg_parser.get<std::string>("task_folder");
    info_json_path = arg_parser.get<std::string>("info_json_path");
    output_path = arg_parser.get<std::string>("output_folder");
    filter_flag = arg_parser.get<std::string>("filter_flag");

    nlohmann::json opt_pose_data;
    std::ifstream ifs_opt_pose(info_json_path);
    opt_pose_data = nlohmann::json::parse(ifs_opt_pose);
    pcl::PointCloud<RenderPointType>::Ptr globalcloud(new pcl::PointCloud<RenderPointType>);

    for(auto per_pose : opt_pose_data["data"]){
        string trail_id = to_string(per_pose["trail_id"]);
        string timestamp_name = to_string(per_pose["stamp_ms"]);
        string pcd_path = task_folder + "/" + trail_id + "/veh_scan/" + timestamp_name + ".pcd";
        pcl::PointCloud<RenderPointType>::Ptr perframecloud(new pcl::PointCloud<RenderPointType>);
        if(!boost::filesystem::exists(pcd_path)){
            continue;
        }
        pcl::io::loadPCDFile<RenderPointType>(pcd_path, *perframecloud);
        
        Eigen::Quaterniond per_pose_Q;
        Eigen::Vector3d per_pose_t;
        Eigen::Matrix4d per_pose_T;
        for (int i = 0; i < 3; i++)
        {
            per_pose_t[i] = per_pose["opt_t_world_veh"][i];
        }
        Eigen::Vector4d per_pose_q;
        for (int i = 0; i < 4; i++)
        {
            per_pose_q[i] = per_pose["opt_q_world_veh"][i];
        }
        std::cout.precision(15);
        per_pose_Q = Eigen::Quaterniond(per_pose_q);
        per_pose_T = qab_tab_to_Tab(per_pose_Q, per_pose_t);     
        pcl::PointCloud<RenderPointType>::Ptr perframecloud_out(new pcl::PointCloud<RenderPointType>); 

        for (size_t i = 0; i < perframecloud->points.size(); ++i)
        {
            // is_road放到了a字段中，bit_0 感知当前车道地面, bit_1 ransac 地面, bit_0 and bit_1 感知当前车道 且 Ransac 地面
            if ((perframecloud->points[i].a & 3) == 3 )
            {
                perframecloud_out->points.push_back(perframecloud->points[i]);
            }
        }  

        pcl::transformPointCloud(*perframecloud_out, *perframecloud_out, per_pose_T);
        std::string pcd_folder = task_folder + "/" + trail_id + "/veh_scan_global";
        if (!boost::filesystem::exists(pcd_folder)) {
            boost::filesystem::create_directory(pcd_folder);
        }
        std::string pcd_path_out = pcd_folder +"/" + timestamp_name + ".pcd";
        if (!perframecloud_out->empty()){
            pcl::io::savePCDFileBinary(pcd_path_out, *perframecloud_out);
        }
    }
    // if (!globalcloud->empty() && filter_flag == "1")
    // {
    //     custom_voxel_filter_label(*globalcloud, 0.05, 0.05, 0.05);
    // }
    // pcl::io::savePCDFileBinary(output_path + "/global_cloud.pcd", *globalcloud);
    
    return 0;
}