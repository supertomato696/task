

#pragma once

#include "road_model/road_model_meta.h"
#include "dao/data_processer.h"
#include "utils/algorithm_util.h"
#include <nlohmann/json.hpp>
#include <liblas/liblas.hpp>
#include <liblas/point.hpp>
#include <liblas/version.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <dirent.h>
#include <string>
#include <iostream>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>

DEFINE_int64(tile_id, 0, "level 15 tile_id");
DEFINE_string(base_dir, "", "base_dir");
DEFINE_string(bev_dir, "detection/bev", "bev_dir");
DEFINE_string(base_log_dir, "./", "base_log_dir");
DEFINE_string(pcd_dir, "lidar", "pcd_dir");
DEFINE_string(key_pose_file, "imgpos.txt", "key_pose_file");
DEFINE_string(bev_label_file, "bev_label/*.json", "bev_label_dir");
DEFINE_string(land_mark_file, "_ddld_landmark.json", "land_mark_dir");
DEFINE_string(client_label_file, "pp_laneline/*.json", "client_label_file");
DEFINE_string(middle_json_path, "middle.json", "middle_json_path");
DEFINE_string(link_json_path, "link.json", "link_json_path");
DEFINE_int32(utm_zone, 50, "utm_zone");
DEFINE_double(utm_center_x, 0, "utm_center_x");
DEFINE_double(utm_center_y, 0, "utm_center_y");
DEFINE_double(utm_center_z, 0, "utm_center_z");
DEFINE_string(ground_pcd_file, "model_pcd/global_cloud.pcd", "ground_pcd_file");
DEFINE_string(ground_las_dir, "mapping_output/ground", "ground_las_dir");
DEFINE_string(mapping_line_dir, "lane_track", "mapping_line_dir");
DEFINE_string(mapping_line_dir_boundary, "lane_track", "mapping_line_dir_boundary");
DEFINE_string(mapping_line_dir_diversion, "fsd_mapbuild_out_cloud_pano_seg", "mapping_line_dir_diversion");
DEFINE_string(mapping_object_dir, "fsd_mapbuild_out", "mapping_object_dir");
DEFINE_string(key_pos_dir, "ins", "key_pos_dir");
DEFINE_string(key_pos_filename, "_mla_egopose.json", "key_pos_filename");
DEFINE_bool(use_middle_enable, false, "use_middle_enable");
DEFINE_bool(custom_utm_enable, true, "custom_utm_enable");
DEFINE_bool(polygon_use_custom, true, "polygon_use_custom");
DEFINE_string(polygon_custom_file, "text.polygon", "polygon_custom_file");
DEFINE_int32(process_thread_concurrency, 10, "process_thread_concurrency");

DEFINE_bool(valid_enable, false, "enable for valid");
DEFINE_double(valid_x, false, "valid_x");
DEFINE_double(valid_y, false, "valid_y");
DEFINE_double(valid_dis, 50, "valid_dis");
DEFINE_double(debug_x, 0, "x for debug");
DEFINE_double(debug_y, 0, "y for debug");
DEFINE_double(debug_z, 0, "z for debug");
DEFINE_double(debug_x2, 0, "x for debug");
DEFINE_double(debug_y2, 0, "y for debug");
DEFINE_double(debug_z2, 0, "z for debug");
DEFINE_double(debug_dis, 10, "dis for debug");
DEFINE_string(debug_frame_id, "", "frame id for debug");
DEFINE_double(display_scope_buff, 30, "display_scope_buff");
DEFINE_double(display_scale_rate, 0.1, "display_scale_rate");
DEFINE_double(bev_lable_min_score, 0.0, "bev_lable_min_score");
DEFINE_double(bev_lable_max_score, 1, "bev_lable_max_score");
DEFINE_int64(bev_trail_id, 0, "bev_trail_id");
DEFINE_bool(scope_use_link, false, "scope_use_link");
DEFINE_bool(polygon_use_task, true, "polygon_use_task");
DEFINE_bool(lane_line_use_cache, false, "lane_line_use_cache");
DEFINE_string(lane_line_cache_file, "../lane_line.pcd", "lane_line_cache_file");

DEFINE_bool(lane_center_use_cache, false, "lane_center_use_cache");
DEFINE_string(lane_center_cache_file, "../lane_center.pcd", "lane_center_cache_file");

DEFINE_bool(boundary_line_use_cache, false, "boundary_line_use_cache");
DEFINE_string(boundary_line_cache_file, "../boundary_line.pcd", "boundary_line_cache_file");
DEFINE_string(site_boundary_line_cache_file, "../site_boundary_line.pcd", "site_boundary_line_cache_file");

DEFINE_bool(bev_data_use_scope, false, "bev_data_use_scope");
DEFINE_double(bev_scope_x_min, 0, "bev_scope_x_min");
DEFINE_double(bev_scope_x_max, 30, "bev_scope_x_max");
DEFINE_double(bev_scope_y_min, -8, "bev_scope_y_min");
DEFINE_double(bev_scope_y_max, 8, "bev_scope_y_max");

DEFINE_bool(client_data_use_scope, false, "client_data_use_scope");
DEFINE_double(client_scope_x_min, 0, "client_scope_x_min");
DEFINE_double(client_scope_x_max, 30, "client_scope_x_max");
DEFINE_double(client_scope_y_min, -8, "client_scope_y_min");
DEFINE_double(client_scope_y_max, 8, "client_scope_y_max");

DEFINE_bool(cloud_data_use_scope, false, "cloud_data_use_scope");
DEFINE_double(cloud_scope_x_min, 0, "cloud_scope_x_min");
DEFINE_double(cloud_scope_x_max, 30, "cloud_scope_x_max");
DEFINE_double(cloud_scope_y_min, -8, "cloud_scope_y_min");
DEFINE_double(cloud_scope_y_max, 8, "cloud_scope_y_max");

#define COLOR_GREEN  "\033[32m"
#define COLOR_RESET "\033[0m"


//filter invalid line pos
DEFINE_double(max_valid_line_dis, 20, "max_valid_line_dis");

namespace fs = boost::filesystem;

namespace fsdmap {
namespace dao {

bool BevDataProcessor::debug_pos(Eigen::Vector3d &pos, std::string frame_id, bool enable) {
    static double global_debug_x = FLAGS_debug_x;
    static double global_debug_y = FLAGS_debug_y;
    static double global_debug_dis = FLAGS_debug_dis;
    if (!enable) {
        return false;
    }
    if (FLAGS_debug_frame_id != "" && frame_id != FLAGS_debug_frame_id) {
        return false;
    }
    utils::DisplayScope box = _scope;
    if (FLAGS_debug_frame_id != "") {
        box = get_display_scope(FLAGS_debug_frame_id);
    }
    Eigen::Vector3d tar_pos = pos;
    // utils::trans_display_local_pos(box, tar_pos);
    // double distance = alg::calc_dis(trans, tar_pos, true);
    Eigen::Vector3d trans = {global_debug_x, global_debug_y, FLAGS_debug_z};
    double distance = alg::calc_dis(trans, tar_pos, true);
    if (distance <= global_debug_dis) {
        return true;
    }
    return false;
}

bool BevDataProcessor::debug_pos2(Eigen::Vector3d &pos1, Eigen::Vector3d &pos2, bool enable) {
    if (!enable) {
        return false;
    }
    Eigen::Vector3d tar_pos1 = pos1;
    Eigen::Vector3d tar_pos2 = pos2;
    utils::trans_display_local_pos(_scope, tar_pos1);
    utils::trans_display_local_pos(_scope, tar_pos2);
    Eigen::Vector3d trans1 = {FLAGS_debug_x, FLAGS_debug_y, FLAGS_debug_z};
    Eigen::Vector3d trans2 = {FLAGS_debug_x2, FLAGS_debug_y2, FLAGS_debug_z2};
    double distance1 = alg::calc_dis(trans1, tar_pos1, true);
    double distance2 = alg::calc_dis(trans2, tar_pos2, true);
    if (distance1 < FLAGS_debug_dis && distance2 < FLAGS_debug_dis) {
        return true;
    }
    return false;
}

int BevDataProcessor::init() {
    std::vector<google::CommandLineFlagInfo> all_flags;
    google::GetAllFlags(&all_flags);
    for (auto &flag : all_flags) {
        LOG_DEBUG("{}={}", flag.name, flag.current_value);
    }
    if (FLAGS_use_middle_enable) {
        std::string file_name = FLAGS_middle_json_path;
        std::ifstream json_file;
        LOG_INFO("fast_road_model json path[{}]", file_name);
        json_file.open(file_name);

        if (!json_file.is_open()) {
            LOG_ERROR("failed to open json file[file={}]", file_name);
            return fsdmap::FAIL;
        }
        nlohmann::json root = nlohmann::json::parse(json_file);
        // 数据类型
        this->_data_type = root["middle"]["data_type"];
        // 中心点位置
        if (root["middle"].contains("site_center") and root["middle"]["site_center"].is_array()) {
            const auto &site_center_obj = root["middle"]["site_center"];
            double site_center_utm_x = site_center_obj[0];
            double site_center_utm_y = site_center_obj[1];
            this->_center_link_pos = Eigen::Vector3d(site_center_utm_x, site_center_utm_y, 0);
            std::cout << "[bev_roaad_model]site_center: " << site_center_utm_x << ", " << site_center_utm_y << std::endl;
        } else {
            std::cout << "[bev_roaad_model]site_center数据结构不正确" << std::endl;
        }

        //frame_id
        if (root["middle"].contains("frame_id") and root["middle"]["frame_id"].is_string()) {
            this->frame_id = root["middle"]["frame_id"];
            std::cout << "[bev_roaad_model]frame_id: " << root["middle"]["frame_id"] << std::endl;
        } else {
            std::cout << "[bev_roaad_model]frame_id数据结构不正确" << std::endl;
        }


        auto &json_link_list = root["middle"]["links"];
        if (json_link_list.size() == 0) {
            LOG_ERROR("failed to get link list from json[file={}]", file_name);
            return fsdmap::FAIL;
        }
        this->_tile_id = json_link_list[0]["tile_ids"][0].get<int64_t>();
        if (FLAGS_custom_utm_enable) {
            _utm_zone = FLAGS_utm_zone;
            _center_pos = {FLAGS_utm_center_x, FLAGS_utm_center_y, FLAGS_utm_center_z};
        } else {
            _utm_zone = root["utm_num"].get<int32_t>();
            auto &utm_list = root["t_utm_world"];
            _center_pos = {utm_list[0].get<double>(), utm_list[1].get<double>(), utm_list[2].get<double>()};
        }
        // _task_id = root["task_id"].get<std::string>();
        if (root["task_id"].is_number()) {
            _task_id = root["task_id"].get<int32_t>();
        } else if (root["task_id"].is_string()) {
            _task_id = root["task_id"].get<std::string>();
        } else {
            _task_id = "unkwoun";
        }
        LOG_INFO("utm_center[{}:{},{},{}], data_type:{}", _utm_zone, _center_pos.x(), _center_pos.y(), _center_pos.z(), this->_data_type);
    } else {
        if (FLAGS_custom_utm_enable) {
            _utm_zone = FLAGS_utm_zone;
            _center_pos = {FLAGS_utm_center_x, FLAGS_utm_center_y, FLAGS_utm_center_z};
        } else {
            return fsdmap::FAIL;
            // if (FLAGS_tile_id != 0) {
            //     this->_tile_id = FLAGS_tile_id;
            // } else {
            //     std::vector<std::string> tile_list;
            //     std::string base_dir = utils::fmt("{}/*", FLAGS_base_dir);
            //     utils::glob_dir(base_dir.c_str(), tile_list);
            //     if (tile_list.size() == 0) {
            //         LOG_ERROR("failed to get tile[path={}]", base_dir);
            //         return fsdmap::FAIL;
            //     }
            //     for (auto &str : tile_list) {
            //         boost::filesystem::path path(str);
            //         this->_tile_id = std::stoi(path.filename().string());
            //         break;
            //     }
            //     if (this->_tile_id == 0) {
            //         LOG_ERROR("failed to get tile[0]");
            //         return fsdmap::FAIL;
            //     }
            // }
            // _task_id = utils::fmt("{}", this->_tile_id);
            // this->_proj15.get_tile_center(_tile_id, _center_pos.x(), _center_pos.y());
        }
    }
    _thread_pool = std::make_shared<utils::ThreadPoolProxy>(FLAGS_process_thread_concurrency);
    init_attr_map();
    return init_ele_type();
}

int BevDataProcessor::download_cross_point(std::vector<BreakInfo*>& split_merge_break_points) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cp(new pcl::PointCloud<pcl::PointXYZI>);
    // std::string base_dir = utils::fmt("{}/*", FLAGS_base_dir);
    std::string cross_point_file = utils::fmt("{}/../fsd_mapbuild_out_cloud_bev_label/crosspoint.pcd", FLAGS_base_dir);
    LOG_INFO("read_cross_point_file:  {}", cross_point_file);
    std::ifstream file(cross_point_file);
    bool fileExist = file.good();
    file.close();
    if (fileExist) {
        pcl::io::loadPCDFile<pcl::PointXYZI>(cross_point_file, *cloud_cp);
        LOG_INFO("read_cross_point: {} pts", cloud_cp->points.size());
    } 

    for (int i = 0; i < cloud_cp->size(); i++) {
        Eigen::Vector3d pt(cloud_cp->points[i].x, cloud_cp->points[i].y, cloud_cp->points[i].z);
        BreakInfo*bk = new BreakInfo(BreakStatus::SPLIT_MERGE_VECTORIZE, pt, true);
        split_merge_break_points.push_back(bk);
    }

    return fsdmap::SUCC;
}

int BevDataProcessor::download_key_pose(std::vector<std::shared_ptr<KeyPose>> &key_poses, 
        bool need_pcd) {
    // std::vector<std::string> trail_list;
    std::vector<std::string> zb_data_list;
    std::string base_dir;
    if (FLAGS_use_middle_enable) {
        base_dir = utils::fmt("{}/*", FLAGS_base_dir);
        utils::glob_dir(base_dir.c_str(), zb_data_list);
    } else {
        
        // base_dir = utils::fmt("{}/{}/*", FLAGS_base_dir, FLAGS_key_pos_dir);
        base_dir = utils::fmt("{}/*", FLAGS_base_dir);
        utils::glob_dir(base_dir.c_str(), zb_data_list);
    }
    this->_key_map.clear();
    UMAP<std::string, std::string> path_map;

    for(auto& zb_dir: zb_data_list){
        LOG_INFO("zb dir:{}", zb_dir);
        std::vector<std::string> trail_list;

        std::string zb_base_dir = utils::fmt("{}/{}/*", zb_dir, FLAGS_key_pos_dir);

        utils::glob_dir(zb_base_dir.c_str(), trail_list);
        for (int i = 0; i < trail_list.size(); ++i) {
            auto &trail_dir = trail_list[i];
            
            if(!boost::filesystem::is_directory(trail_dir)){
                continue;
            }
            // LOG_INFO("trail_dir [path={}]", trail_dir);

            std::vector<std::string> key_pos_list;
            utils::glob_dir_all(trail_dir.c_str(), key_pos_list,true);

            //sort
            SORT(key_pos_list, [](const std::string &l, const std::string  &r) {
                return l < r;
            });

            KeyPose* prev_pose = NULL;
            int64_t line_index = 0;
            int64_t raw_no = 1;
            for(int j = 0; j < key_pos_list.size(); j+=100){ // 100个点取一个pose数据
                std::string file_path = key_pos_list[j];
                if(boost::filesystem::is_directory(file_path)){
                    continue;
                }
                std::ifstream ifs(file_path);
                utils::ResPtr file_res([&ifs]() {
                        ifs.close();
                        });
                if(!ifs.is_open()){
                    LOG_ERROR("failed to load key_pose[path={}]", file_path);
                    continue;
                }
                // LOG_ERROR("key_pos_list [path={}]", file_path);
                nlohmann::json root = nlohmann::json::parse(ifs);

                int64_t time_stamp = int64_t(root["header"]["measurementTimestamp"].get<double>()*1e3);
                // int32_t utm = root["pose"]["utmZone"].get<int32_t>();
                // double json_key_pose_x = root["pose"]["gcjPosition"]["lon"].get<double>();
                // double json_key_pose_y = root["pose"]["gcjPosition"]["lat"].get<double>();
                // double json_key_pose_z = root["pose"]["gcjPosition"]["height"].get<double>();
                // double json_key_pose_qx = root["pose"]["orientation"]["qx"].get<double>();
                // double json_key_pose_qy = root["pose"]["orientation"]["qy"].get<double>();
                // double json_key_pose_qz = root["pose"]["orientation"]["qz"].get<double>();
                // double json_key_pose_qw = root["pose"]["orientation"]["qw"].get<double>();

                double json_key_pose_x = root["positionLlh"]["lon"].get<double>();
                double json_key_pose_y = root["positionLlh"]["lat"].get<double>();
                double json_key_pose_z = root["positionLlh"]["height"].get<double>();
                double json_key_pose_qx = root["orientation"]["qx"].get<double>();
                double json_key_pose_qy = root["orientation"]["qy"].get<double>();
                double json_key_pose_qz = root["orientation"]["qz"].get<double>();
                double json_key_pose_qw = root["orientation"]["qw"].get<double>();

                double json_key_orientation_x = root["orientationStd"]["x"].get<double>();
                double json_key_orientation_y = root["orientationStd"]["y"].get<double>();
                double json_key_orientation_z = root["orientationStd"]["z"].get<double>();

                auto key_pose = std::make_shared<KeyPose>();
                // key_pose->timestamp = time_stamp/1e3;
                key_pose->timestamp = time_stamp;
                key_pose->id = std::to_string(int64_t(time_stamp*1e-3));
                // key_pose->frame_id = utils::fmt("{}_{}_{}", tile_id, trail_id, vec[0]);
                key_pose->trail_id = trail_dir;
                key_pose->frame_id = get_frame_id(trail_dir, key_pose->id);
                key_pose->line_id = trail_dir;
                key_pose->line_index = line_index++;
                key_pose->raw_no = raw_no++;
                key_pose->raw_file_no = i + 1;
                path_map[key_pose->line_id] = trail_dir;
                key_pose->wgs[0] = json_key_pose_x;
                key_pose->wgs[1] = json_key_pose_y;
                key_pose->wgs[2] = json_key_pose_z;

                this->wgs2local(key_pose->wgs, key_pose->pos);
                // key_pose->yaw = 90;  
                key_pose->quat.x() = json_key_pose_qx;
                key_pose->quat.y() = json_key_pose_qy;
                key_pose->quat.z() = json_key_pose_qz;
                key_pose->quat.w() = json_key_pose_qw;
                key_pose->r = key_pose->quat.normalized().toRotationMatrix();
                // key_pose->pos = key_pose->r * key_pose->pos;

                // Eigen::AngleAxisd angle_axis_x(json_key_orientation_x, Eigen::Vector3d::UnitX());
                // key_pose->r *= angle_axis_x.matrix();
                // Eigen::AngleAxisd angle_axis_y(json_key_orientation_y, Eigen::Vector3d::UnitY());
                // key_pose->r *= angle_axis_y.matrix();
                // Eigen::AngleAxisd angle_axis_z(json_key_orientation_z, Eigen::Vector3d::UnitZ());
                // key_pose->r *= angle_axis_z.matrix();

                utils::DisplayScope box = {FLAGS_display_scope_buff, FLAGS_display_scope_buff,
                    key_pose->pos, FLAGS_display_scale_rate};

                // LOG_INFO("key_frame{}", key_pose->frame_id);
                this->_key_map[key_pose->frame_id] = std::make_pair(key_pose, box);
                if (prev_pose != NULL) {
                    key_pose->set_prev(prev_pose);
                    alg::calc_dir(key_pose->pos, prev_pose->pos, prev_pose->dir);
                    key_pose->dir = prev_pose->dir;
                }
                prev_pose = key_pose.get();

                // LOG_ERROR("{} {} {} {} {} {} {}", key_pose->timestamp, 
                //                                     key_pose->frame_id, 
                //                                     key_pose->line_id, 
                //                                     key_pose->id, 
                //                                     key_pose->pos.x(),
                //                                     key_pose->pos.y(),
                //                                     key_pose->pos.z());
            }
            // std::string file_path = utils::fmt("{}/{}", trail_dir, FLAGS_key_pose_file);
            // std::ifstream ifs(file_path);
            // utils::ResPtr file_res([&ifs]() {
            //         ifs.close();
            //         });
            // if(!ifs.is_open()){
            //     LOG_ERROR("failed to load key_pose[path={}]", file_path);
            //     continue;
            // }
            // boost::filesystem::path trail_path(trail_dir);
            // std::string trail_id = trail_path.filename().string();
            // std::string tile_id = trail_path.parent_path().filename().string();
            // std::string delimiter = " ";
            // std::string line;
            // KeyPose* prev_pose = NULL;
            // int64_t line_index = 0;
            // int64_t raw_no = 1;
            // while (getline(ifs, line)) {
            //     std::vector<std::string> vec;
            //     utils::split(line, delimiter, vec);
            //     if (vec.size() < 5){
            //         LOG_ERROR(" failed to parse imgpos[line={}]", line);
            //         continue;
            //     }
            //     auto key_pose = std::make_shared<KeyPose>();
            //     key_pose->timestamp = std::stod(vec[0]) / 1e3;
            //     key_pose->id = vec[0];
            //     // key_pose->frame_id = utils::fmt("{}_{}_{}", tile_id, trail_id, vec[0]);
            //     key_pose->trail_id = trail_id;
            //     key_pose->frame_id = get_frame_id(trail_id, vec[0]);
            //     key_pose->line_id = utils::fmt("{}_{}", tile_id, trail_id);
            //     key_pose->line_index = line_index++;
            //     key_pose->raw_no = raw_no++;
            //     key_pose->raw_file_no = i + 1;
            //     path_map[key_pose->line_id] = trail_dir;
            //     Eigen::Vector3d wgs = {std::stod(vec[1]), std::stod(vec[2]), std::stod(vec[3])};
            //     this->wgs2local(wgs, key_pose->pos);
            //     // key_pose->yaw = std::stod(vec[4]);
            //     key_pose->quat.x() = std::stod(vec[5]);
            //     key_pose->quat.y() = std::stod(vec[6]);
            //     key_pose->quat.z() = std::stod(vec[7]);
            //     key_pose->quat.w() = std::stod(vec[8]);
            //     key_pose->r = key_pose->quat.normalized().toRotationMatrix();

            //     utils::DisplayScope box = {FLAGS_display_scope_buff, FLAGS_display_scope_buff,
            //         key_pose->pos, FLAGS_display_scale_rate};

            //     // LOG_INFO("key_frame{}", key_pose->frame_id);
            //     this->_key_map[key_pose->frame_id] = std::make_pair(key_pose, box);
            //     if (prev_pose != NULL) {
            //         key_pose->set_prev(prev_pose);
            //         alg::calc_dir(key_pose->pos, prev_pose->pos, prev_pose->dir);
            //         key_pose->dir = prev_pose->dir;
            //     }
            //     prev_pose = key_pose.get();
            // }
        }
    }
 
    // if (!FLAGS_use_middle_enable) {
        init_display_scope();
    // }
    LOG_INFO("_key_map size: {} ", this->_key_map.size());
    for (auto &key_pose_it : this->_key_map) {
        auto &key_pose_ptr = key_pose_it.second.first;
        if (!is_in_processe_scope(key_pose_ptr->pos)) {
            continue;
        }
        key_poses.push_back(key_pose_ptr);
        auto key_pose = key_pose_ptr.get();
        if (!need_pcd) {
            continue;
        }
        _thread_pool->schedule([&, key_pose, this](utils::ProcessBar *bar) {
        auto &trail_dir = path_map[key_pose->line_id];
        std::string pcd_file = utils::fmt("{}/{}/{}.pcd", 
                trail_dir, FLAGS_pcd_dir, key_pose->id);
        key_pose->pcd.reset(new pcl::PointCloud<utils::CloudPoint>);
        if (pcl::io::loadPCDFile<utils::CloudPoint>(pcd_file.c_str(), *key_pose->pcd) == -1) {
            LOG_ERROR("failed to read pcd[frame_id={}, file={}]", key_pose->frame_id, pcd_file);
            bar->set_error();
            return;
        }
        });
    }
    if (need_pcd) {
        _thread_pool->wait(0, "download_key_pos pcd");
    }

    LOG_INFO("key_poses size: {} ", key_poses.size());
    
    SORT(key_poses, [](const std::shared_ptr<KeyPose> &l, const std::shared_ptr<KeyPose> &r) {
               return l->timestamp < r->timestamp;
            });
    return fsdmap::SUCC;
}

int BevDataProcessor::download_key_pose_mnt(std::vector<std::shared_ptr<KeyPose>> &key_poses, 
        bool need_pcd) {
    // std::vector<std::string> trail_list;
    std::vector<std::string> zb_data_list;
    std::string base_dir;

    base_dir = utils::fmt("{}/*", FLAGS_base_dir);
    utils::glob_dir(base_dir.c_str(), zb_data_list);

    this->_key_map.clear();
    // this->origin_poss_tree_.RemoveAll();
    this->origin_poss_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    UMAP<std::string, std::string> path_map;

    for(auto& zb_dir: zb_data_list){
        LOG_INFO("zb dir:{}", zb_dir);
        std::vector<std::string> trail_list;

        std::string zb_base_dir = utils::fmt("{}/*", zb_dir);

        utils::glob_dir(zb_base_dir.c_str(), trail_list);
        
        for (int i = 0; i < trail_list.size(); ++i) {
            auto &trail_dir = trail_list[i];
            
            if(!boost::filesystem::is_directory(trail_dir)){
                continue;
            }
            LOG_INFO("trail_dir [raw_file_no:{} path={}]", i+1, trail_dir);

            std::string this_trail_file_name = utils::fmt("{}/{}", trail_dir, FLAGS_key_pos_filename);
            std::ifstream ifs(this_trail_file_name);
            utils::ResPtr file_res([&ifs]() {
                    ifs.close();
                    });
            if(!ifs.is_open()){
                LOG_ERROR("failed to load key_pose[path={}]", this_trail_file_name);
                continue;
            }
            // LOG_ERROR("key_pos_list [path={}]", file_path);
            nlohmann::json root = nlohmann::json::parse(ifs);

            KeyPose* prev_pose = NULL;
            int64_t line_index = 0;
            int64_t raw_no = 1;
            for(auto &frame : root){
                int64_t time_stamp = int64_t(frame["meta"]["timestamp_us"].get<double>()*1e-3);

                double json_key_pose_x = frame["position"]["position_global"]["longitude"].get<double>();
                double json_key_pose_y = frame["position"]["position_global"]["latitude"].get<double>();
                double json_key_pose_z = frame["position"]["position_global"]["altitude"].get<double>();
                double json_key_pose_qx = frame["orientation"]["quaternion_global"]["x"].get<double>();
                double json_key_pose_qy = frame["orientation"]["quaternion_global"]["y"].get<double>();
                double json_key_pose_qz = frame["orientation"]["quaternion_global"]["z"].get<double>();
                double json_key_pose_qw = frame["orientation"]["quaternion_global"]["w"].get<double>();

                // double json_key_orientation_x = frame["orientationStd"]["x"].get<double>();
                // double json_key_orientation_y = frame["orientationStd"]["y"].get<double>();
                // double json_key_orientation_z = frame["orientationStd"]["z"].get<double>();

                auto key_pose = std::make_shared<KeyPose>();
                // key_pose->timestamp = time_stamp/1e3;
                key_pose->timestamp = time_stamp;
                key_pose->id = std::to_string(int64_t(time_stamp));
                // key_pose->frame_id = utils::fmt("{}_{}_{}", tile_id, trail_id, vec[0]);
                key_pose->trail_id = trail_dir;
                key_pose->frame_id = get_frame_id(trail_dir, key_pose->id);
                key_pose->line_id = trail_dir;
                key_pose->line_index = line_index++;
                key_pose->raw_no = raw_no++;
                key_pose->raw_file_no = i + 1;
                path_map[key_pose->line_id] = trail_dir;
                // LOG_INFO("trail_id [{}]", key_pose->trail_id);
                // LOG_INFO("frame_id [{}]", key_pose->frame_id);
                double lng_wgs, lat_wgs;
                // TODO:qzc gcj01
                // alg::gcj2Towgs(json_key_pose_x, json_key_pose_y, lng_wgs, lat_wgs);
                lng_wgs = json_key_pose_x;
                lat_wgs = json_key_pose_y;
                // TODO:qzc gcj01
                key_pose->wgs[0] = lng_wgs;
                key_pose->wgs[1] = lat_wgs;
                key_pose->wgs[2] = json_key_pose_z;

                this->wgs2local(key_pose->wgs, key_pose->pos);
                // key_pose->yaw = 90;  
                key_pose->quat.x() = json_key_pose_qx;
                key_pose->quat.y() = json_key_pose_qy;
                key_pose->quat.z() = json_key_pose_qz;
                key_pose->quat.w() = json_key_pose_qw;
                key_pose->r = key_pose->quat.normalized().toRotationMatrix();

                utils::DisplayScope box = {FLAGS_display_scope_buff, FLAGS_display_scope_buff,
                    key_pose->pos, FLAGS_display_scale_rate}; // 默认 FLAGS_display_scope_buff = 30， FLAGS_display_scale_rate = 0.1

                // LOG_INFO("key_frame{}", key_pose->frame_id);
                this->_key_map[key_pose->frame_id] = std::make_pair(key_pose, box);
                if (prev_pose != NULL) {
                    key_pose->set_prev(prev_pose);
                    alg::calc_dir(key_pose->pos, prev_pose->pos, prev_pose->dir);
                    key_pose->dir = prev_pose->dir;
                }
                prev_pose = key_pose.get();
            }

        }
    }
 
    // if (!FLAGS_use_middle_enable) {
        init_display_scope();
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr pose_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    int pose_cnt = 0;
    LOG_INFO("_key_map size: {} ", this->_key_map.size());
    for (auto &key_pose_it : this->_key_map) {
        auto &key_pose_ptr = key_pose_it.second.first;
        if (!is_in_processe_scope(key_pose_ptr->pos)) {
            continue;
        }
        key_poses.push_back(key_pose_ptr);
        
        id_2_pose_map[pose_cnt] = key_pose_ptr;
        pose_cloud->push_back(pcl::PointXYZI(key_pose_ptr->pos.x(), key_pose_ptr->pos.y(), 0.0/*key_pose_ptr->pos.z*/, pose_cnt));
        pose_cnt++;

        auto key_pose = key_pose_ptr.get();
        if (!need_pcd) {
            continue;
        }
        _thread_pool->schedule([&, key_pose, this](utils::ProcessBar *bar) {
        auto &trail_dir = path_map[key_pose->line_id];
        std::string pcd_file = utils::fmt("{}/{}/{}.pcd", 
                trail_dir, FLAGS_pcd_dir, key_pose->id);
        key_pose->pcd.reset(new pcl::PointCloud<utils::CloudPoint>);
        if (pcl::io::loadPCDFile<utils::CloudPoint>(pcd_file.c_str(), *key_pose->pcd) == -1) {
            LOG_ERROR("failed to read pcd[frame_id={}, file={}]", key_pose->frame_id, pcd_file);
            bar->set_error();
            return;
        }
        });
    }
    if (need_pcd) {
        _thread_pool->wait(0, "download_key_pos pcd");
    }

    origin_poss_tree_->setInputCloud(pose_cloud); // 设置要搜索的点云，建立KDTree

    LOG_INFO("key_poses size: {} ", key_poses.size());
    
    SORT(key_poses, [](const std::shared_ptr<KeyPose> &l, const std::shared_ptr<KeyPose> &r) {
               return l->timestamp < r->timestamp;
            });
    return fsdmap::SUCC;
}

bool BevDataProcessor::search_nearest_pose(pcl::PointXYZI& searchPt, std::vector<float>& squaredDistances)
{
    std::vector<int> indices;
    // std::vector<float> squaredDistances;
    // pcl::PointXYZI searchPt(centroid.x(), centroid.y(), centroid.z(), 0);
    origin_poss_tree_->nearestKSearch(searchPt, 1, indices, squaredDistances);
    // std::cout<<"searched indices size: "<<indices.size()<<std::endl;
    if(indices.empty()){
        return false;
    }

    searchPt.intensity = indices[0];

    return true;
}

int BevDataProcessor::download_lane_line(std::vector<std::shared_ptr<Feature>> &feature_vec) {
    // std::string version = "MMT_RC";
    std::string version = this->_data_type;

    std::string cache_file = utils::fmt("{}/{}", 
            FLAGS_base_dir, FLAGS_lane_line_cache_file);

    utils::ElementCloudPtr global_pcd(new utils::ElementCloud);
    if (!FLAGS_lane_line_use_cache) {
        std::vector<std::string> trail_list;
        std::vector<std::string> lane_list;
        std::string base_dir = utils::fmt("{}/../{}/*", FLAGS_base_dir, FLAGS_mapping_line_dir);
        utils::glob_dir(base_dir.c_str(), trail_list);
        for (auto &trail_str : trail_list) {
            if (!fs::is_directory(trail_str)) {
                continue;
            }

            std::string lane_file = utils::fmt("{}/laneBoundaryFromSeg/*_trjCloud.pcd", trail_str);
            std::vector<std::string> tmp;
            utils::glob_dir(lane_file.c_str(), tmp);
            VEC_PUSH_ALL(lane_list, tmp);
        }
        for (int i = 0; i < lane_list.size(); ++i) {
            auto &lane_str = lane_list[i];
            utils::ElementCloudPtr lane_pcd(new utils::ElementCloud);
            if (pcl::io::loadPCDFile<PointElement>(lane_str.c_str(), *lane_pcd) == -1) {
                LOG_ERROR("failed to read pcd[{}]", lane_str);
                return fsdmap::FAIL;;
            }
            for (int j = 0; j < lane_pcd->points.size(); ++j) {
                auto &pt = lane_pcd->points[j];
                float mask = lane_boundary_id + (float)j / 10000;
                pt.intensity = mask;
            }
            lane_boundary_id++;
            *global_pcd += *lane_pcd;
        }
        if (global_pcd->size() > 0) {
            pcl::io::savePCDFile<PointElement>(cache_file.c_str(), *global_pcd);
            LOG_INFO("cache lane_line[file={}]", cache_file);
            utils::ElementCloudPtr lane_pcd(new utils::ElementCloud);
            *lane_pcd = *global_pcd;
            for (int i = 0; i < lane_pcd->points.size(); ++i) {
                auto &pt = lane_pcd->points[i];
                pt.x += this->_center_pos.x();
                pt.y += this->_center_pos.y();
                pt.z += this->_center_pos.z();
            }
            auto global_cache_file = utils::fmt("{}_global.pcd", cache_file);
            pcl::io::savePCDFile<PointElement>(global_cache_file.c_str(), *lane_pcd);
            LOG_INFO("cache lane_line[file={}]", global_cache_file);
        }
    } else {
        if (pcl::io::loadPCDFile<PointElement>(cache_file.c_str(), *global_pcd) == -1) {
            LOG_ERROR("failed to read pcd[{}]", cache_file);
            return fsdmap::FAIL;;
        }
    }
    for (int j = 0; j < global_pcd->points.size(); ++j) {
        auto &pt = global_pcd->points[j];
        auto feature = std::make_shared<Feature>();
        int64_t line_id = (int)pt.intensity;
        int64_t line_index = (int)(pt.intensity - line_id) * 10000;
        feature->line_id = std::to_string(line_id);
        feature->line_index = line_index;
        feature->frame_id = "";
        feature->pos = {pt.x, pt.y, pt.z};
        feature->raw_pos = feature->pos;

        pcl::PointXYZI searchPt(pt.x, pt.y, 0, -1);
        std::vector<float> squaredDistances;
        bool has_found = search_nearest_pose(searchPt, squaredDistances);
        if(has_found) {
            feature->key_pose = id_2_pose_map[int(searchPt.intensity)].get();
        } else {
            feature->key_pose = NULL;
        }

        feature->score = 0;
        feature->type = get_std_attr("mmt_to_std", "lane_boundary", "type", pt.type1);
        feature->color = get_std_attr("mmt_to_std", "lane_boundary", "color", pt.type2);
        // std::cout << "lane_boundary type:" << feature->type << ", color:" << feature->color << std::endl;
        // LOG_INFO("feature : type:{} color:{}",feature->type , feature->color);
        // TODO:qzc change it !
        int data_type = 0;
        if (this->_data_type == "MMT_RC") {
            data_type = 5;
        } else if (this->_data_type == "BYD_BEV" || this->_data_type == "BYD_LIDAR_B" || this->_data_type == "BYD_LIDAR_BEV_B") {
            data_type = 9;
        }
        
        // feature->type = data_type;
        feature->ele_type = this->get_ele_type(version, data_type);
        if (!is_in_processe_scope(feature->pos)) {
            continue;
        }
        feature_vec.push_back(feature);
    }
    return fsdmap::SUCC;
}

int BevDataProcessor::download_lane_center(std::vector<std::shared_ptr<Feature>> &feature_vec) {
    // std::string version = "MMT_RC";
    std::string version = this->_data_type;

    std::string cache_file = utils::fmt("{}/{}", 
            FLAGS_base_dir, FLAGS_lane_center_cache_file);

    utils::ElementCloudPtr global_pcd(new utils::ElementCloud);
    if (!FLAGS_lane_center_use_cache) {
        std::vector<std::string> trail_list;
        std::vector<std::string> lane_list;
        std::string base_dir = utils::fmt("{}/../{}/*", FLAGS_base_dir, FLAGS_mapping_line_dir);
        utils::glob_dir(base_dir.c_str(), trail_list);
        for (auto &trail_str : trail_list) {
            if (!fs::is_directory(trail_str)) {
                continue;
            }
            std::string lane_file = utils::fmt("{}/laneCenterFromSeg/*_trjCloud.pcd", trail_str);
            std::vector<std::string> tmp;
            utils::glob_dir(lane_file.c_str(), tmp);
            VEC_PUSH_ALL(lane_list, tmp);
        }
        for (int i = 0; i < lane_list.size(); ++i) {
            auto &lane_str = lane_list[i];
            utils::ElementCloudPtr lane_pcd(new utils::ElementCloud);
            if (pcl::io::loadPCDFile<PointElement>(lane_str.c_str(), *lane_pcd) == -1) {
                LOG_ERROR("failed to read pcd[{}]", lane_str);
                return fsdmap::FAIL;;
            }
            for (int j = 0; j < lane_pcd->points.size(); ++j) {
                auto &pt = lane_pcd->points[j];
                float mask = lane_center_id + (float)j / 10000;
                pt.intensity = mask;
            }
            lane_center_id++;
            *global_pcd += *lane_pcd;
        }
        if (global_pcd->size() > 0) {
            pcl::io::savePCDFile<PointElement>(cache_file.c_str(), *global_pcd);
            LOG_INFO("cache lane_line[file={}]", cache_file);
            utils::ElementCloudPtr lane_pcd(new utils::ElementCloud);
            *lane_pcd = *global_pcd;
            for (int i = 0; i < lane_pcd->points.size(); ++i) {
                auto &pt = lane_pcd->points[i];
                pt.x += this->_center_pos.x();
                pt.y += this->_center_pos.y();
                pt.z += this->_center_pos.z();
            }
            auto global_cache_file = utils::fmt("{}_global.pcd", cache_file);
            pcl::io::savePCDFile<PointElement>(global_cache_file.c_str(), *lane_pcd);
            LOG_INFO("cache lane_line[file={}]", global_cache_file);
        }
    } else {
        if (pcl::io::loadPCDFile<PointElement>(cache_file.c_str(), *global_pcd) == -1) {
            LOG_ERROR("failed to read pcd[{}]", cache_file);
            return fsdmap::FAIL;;
        }
    }
    for (int j = 0; j < global_pcd->points.size(); ++j) {
        auto &pt = global_pcd->points[j];
        auto feature = std::make_shared<Feature>();
        int64_t line_id = (int)pt.intensity;
        int64_t line_index = (int)(pt.intensity - line_id) * 10000;
        feature->line_id = std::to_string(line_id);
        feature->line_index = line_index;
        feature->frame_id = "";
        feature->pos = {pt.x, pt.y, pt.z};
        feature->raw_pos = feature->pos;
        pcl::PointXYZI searchPt(pt.x, pt.y, 0, -1);
        std::vector<float> squaredDistances;
        bool has_found = search_nearest_pose(searchPt, squaredDistances);
        if(has_found) {
            feature->key_pose = id_2_pose_map[int(searchPt.intensity)].get();
        } else {
            feature->key_pose = NULL;
        }
        feature->score = 0;

        std::vector<uint8_t> attr_vec = get_std_attrs("mmt_to_std", "lane_center", "type", pt.type1);
        // std::string type_str = "";
        // for (size_t i = 0; i < attr_vec.size(); ++i) {
        //     type_str += std::to_string(attr_vec[i]);
        //     if (i < attr_vec.size() - 1) {
        //         type_str += ",";
        //     }
        // }
        // feature->type = type_str;
        feature->type = attr_vec[0];
        // std::cout << "lane_center type: " << feature->type << std::endl;

        // TODO:qzc change it !
        // TODO:qzc check!!!
        int data_type = 0;
        if (this->_data_type == "MMT_RC") {
            data_type = 18;
        } else if (this->_data_type == "BYD_BEV" || this->_data_type == "BYD_LIDAR_B" || this->_data_type == "BYD_LIDAR_BEV_B") {
            data_type = 42;
        }
        // feature->type = data_type;
        feature->ele_type = this->get_ele_type(version, data_type);
        if (!is_in_processe_scope(feature->pos)) {
            continue;
        }
        feature_vec.push_back(feature);
    }

    return fsdmap::SUCC;

}

int BevDataProcessor::download_site_boundary_line(std::vector<std::shared_ptr<Feature>> &feature_vec) {

    // std::string version = "MMT_RC";
    std::string version = this->_data_type;

    std::string cache_file = utils::fmt("{}/{}", FLAGS_base_dir, FLAGS_site_boundary_line_cache_file);

    utils::ElementCloudPtr local_pcd(new utils::ElementCloud);
    if (!FLAGS_boundary_line_use_cache) {
        std::vector<std::string> trail_list;
        std::vector<std::string> lane_list;
        std::vector<std::string> contour_list;
        std::string base_dir = utils::fmt("{}/../{}/site_boundary", FLAGS_base_dir, FLAGS_mapping_line_dir);
        
        // 1 查找所有的 pcd
        utils::glob_dir(base_dir.c_str(), trail_list);
        for (auto &trail_str : trail_list) {
            if (!fs::is_directory(trail_str)) {
                continue;
            }

            std::string lane_file = utils::fmt("{}/*_trjCloud.pcd", trail_str);
            LOG_INFO("lane_file={}", lane_file);
            std::vector<std::string> tmp;
            utils::glob_dir(lane_file.c_str(), tmp);
            VEC_PUSH_ALL(lane_list, tmp);

            tmp.clear();
            std::string contour_file = utils::fmt("{}/*_trjCloud_contour.pcd", trail_str);
            utils::glob_dir(contour_file.c_str(), tmp);
            VEC_PUSH_ALL(contour_list, tmp);
        }

        LOG_INFO("contour_list size={}", contour_list.size());

        // 2 读取边界线
        // TODO: qzc 多线程
        for (int i = 0; i < contour_list.size(); ++i) {
            auto &contour_str = contour_list[i];
            utils::ElementCloudPtr contour_pcd(new utils::ElementCloud);
            if (pcl::io::loadPCDFile<PointElement>(contour_str.c_str(), *contour_pcd) == -1) {
                LOG_ERROR("failed to read pcd[{}]", contour_str);
                return fsdmap::FAIL;;
            }
            std::vector<Eigen::Vector3d> contour;
            for (int j = 0; j < contour_pcd->points.size(); ++j) {
                auto &pt = contour_pcd->points[j];
                contour.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
            }
            this->side_boundary_polygons.push_back(contour);
            // std::cout << "[download_site_boundary_line] index: " << i << " size: " << contour.size() << " contour_str: " << contour_str << std::endl; 
        }

        // 2 读取所有的pcd
        // TODO: qzc 多线程
        for (int i = 0; i < lane_list.size(); ++i) {
            auto &lane_str = lane_list[i];
            utils::ElementCloudPtr lane_pcd(new utils::ElementCloud);
            if (pcl::io::loadPCDFile<PointElement>(lane_str.c_str(), *lane_pcd) == -1) {
                LOG_ERROR("failed to read pcd[{}]", lane_str);
                return fsdmap::FAIL;;
            }

            double total_length = 0;
            Eigen::Vector3d prev_pt;
            prev_pt.setZero();
            for (int j = 0; j < lane_pcd->points.size(); ++j) {
                auto &pt = lane_pcd->points[j];
                float mask = road_boundary_id + (float)j / 10000;
                pt.intensity = mask;

                Eigen::Vector3d cur_pt = Eigen::Vector3d(pt.x, pt.y, pt.z);
                if (prev_pt.norm() > 1e-6) {
                    total_length += alg::calc_dis(prev_pt, cur_pt);
                }
                prev_pt = cur_pt;
            }

            // 删除较短的边界线
            if(total_length < 3) {
                // std::cout << "download_site_boundary_line, total_length: " << total_length << std::endl;
                continue;
            }

            road_boundary_id++;
            *local_pcd += *lane_pcd;

            // TODO:qzc for debug
            // LOG_INFO("line={}, line_id={}", lane_str.c_str(), i);
        }

        // 3 保存 local and global pcd
        if (local_pcd->size() > 0) {
            pcl::io::savePCDFile<PointElement>(cache_file.c_str(), *local_pcd);
            LOG_INFO("cache lane_line[file={}]", cache_file);
            utils::ElementCloudPtr global_pcd(new utils::ElementCloud);
            *global_pcd = *local_pcd;
            for (int i = 0; i < global_pcd->points.size(); ++i) {
                auto &pt = global_pcd->points[i];
                pt.x += this->_center_pos.x();
                pt.y += this->_center_pos.y();
                pt.z += this->_center_pos.z();
            }
            auto global_cache_file = utils::fmt("{}_global.pcd", cache_file);
            pcl::io::savePCDFile<PointElement>(global_cache_file.c_str(), *global_pcd);
            LOG_INFO("cache lane_line[file={}]", global_cache_file);
        }
    }

    // 4 解析pcd
    for (int j = 0; j < local_pcd->points.size(); ++j) {
        auto &pt = local_pcd->points[j];
        // 质心点
        if (pt.ele_type == 100) {
            continue;
        }

        auto feature = std::make_shared<Feature>();
        int64_t line_id = (int)pt.intensity;
        int64_t line_index = (int)(pt.intensity - line_id) * 10000;
        feature->line_id = std::to_string(line_id);
        feature->line_index = line_index;
        feature->frame_id = "";
        feature->pos = {pt.x, pt.y, pt.z};
        feature->raw_pos = feature->pos;
        feature->boundary_type = LaneType::ISLAND_RB; // 三角岛
        pcl::PointXYZI searchPt(pt.x, pt.y, 0, -1);
        std::vector<float> squaredDistances;
        bool has_found = search_nearest_pose(searchPt, squaredDistances);
        if(has_found) {
            feature->key_pose = id_2_pose_map[int(searchPt.intensity)].get();
        } else {
            feature->key_pose = NULL;
        }
        feature->score = 0;
        feature->type = get_std_attr("mmt_to_std", "road_boundary", "type", pt.type1);
        if (j == 0) {
            LOG_INFO("boundary feature : type:{} color:{}",feature->type , pt.type1);
            // std::cout << "road_boundary type " << feature->type << std::endl;
        }

        // TODO:qzc change it !
        int data_type = 0;
        if (this->_data_type == "MMT_RC") {
            data_type = 10;
        } else if (this->_data_type == "BYD_BEV" || this->_data_type == "BYD_LIDAR_B" || this->_data_type == "BYD_LIDAR_BEV_B") {
            data_type = 29;
        }
        // feature->type = data_type;
        feature->ele_type = this->get_ele_type(version, data_type);
        if (!is_in_processe_scope(feature->pos)) {
            continue;
        }
        feature_vec.push_back(feature);
    }

    // LOG_INFO("succ to download_site_boundary_line[size={}]", feature_vec.size());
    return fsdmap::SUCC;
}

int BevDataProcessor::download_boundary_line(std::vector<std::shared_ptr<Feature>> &feature_vec) {

    // std::string version = "MMT_RC";
    std::string version = this->_data_type;

    std::string cache_file = utils::fmt("{}/{}", 
            FLAGS_base_dir, FLAGS_boundary_line_cache_file);



    utils::ElementCloudPtr global_pcd(new utils::ElementCloud);
    if (!FLAGS_boundary_line_use_cache) {
        std::vector<std::string> trail_list;
        std::vector<std::string> lane_list;
        std::string base_dir = utils::fmt("{}/../{}/*", FLAGS_base_dir, FLAGS_mapping_line_dir);
        utils::glob_dir(base_dir.c_str(), trail_list);
        for (auto &trail_str : trail_list) {
            if (!fs::is_directory(trail_str)) {
                continue;
            }
            std::string lane_file = utils::fmt("{}/roadBoundaryFromSeg/*_trjCloud.pcd", trail_str);
            std::vector<std::string> tmp;
            utils::glob_dir(lane_file.c_str(), tmp);
            VEC_PUSH_ALL(lane_list, tmp);
        }
        for (int i = 0; i < lane_list.size(); ++i) {
            auto &lane_str = lane_list[i];
            utils::ElementCloudPtr lane_pcd(new utils::ElementCloud);
            if (pcl::io::loadPCDFile<PointElement>(lane_str.c_str(), *lane_pcd) == -1) {
                LOG_ERROR("failed to read pcd[{}]", lane_str);
                return fsdmap::FAIL;;
            }

            double total_length = 0;
            Eigen::Vector3d prev_pt;
            prev_pt.setZero();
            std::vector<Eigen::Vector3d> obj_points;
            for (int j = 0; j < lane_pcd->points.size(); ++j) {
                auto &pt = lane_pcd->points[j];
                float mask = road_boundary_id + (float)j / 10000;
                pt.intensity = mask;

                Eigen::Vector3d cur_pt = Eigen::Vector3d(pt.x, pt.y, pt.z);
                obj_points.push_back(cur_pt);

                if (prev_pt.norm() > 1e-6) {
                    total_length += alg::calc_dis(prev_pt, cur_pt);
                }
                prev_pt = cur_pt;

                // if (j == 0 && pt.id == 10) {
                //     std::cout << "path: " << lane_str << " total_length: " << total_length <<  std::endl;
                // }
            }
            // if (lane_str.find("2355356") != std::string::npos) {
            //     std::cout << "lane_str: " << lane_str << " len: " << total_length << std::endl;
            // }

            // 删除较短的边界线
            if(total_length < 3) {
                // std::cout << "download_boundary_line, total_length: " << total_length << std::endl;
                continue;
            }

            // 判断该线段是否为边界线的点
            int poly_index = 0;
            bool intersect_with_polygons = false;
            for(auto& side_boundary_polygon : this->side_boundary_polygons) {
                if(alg::any_point_in_polygon(side_boundary_polygon, obj_points)) {
                    intersect_with_polygons = true;
                    // std::cout << "has intersect with poly: " << poly_index <<  std::endl;
                    break;
                }
                poly_index++;
            }

            if(intersect_with_polygons == false){
                road_boundary_id++;
                *global_pcd += *lane_pcd;
            }

            // TODO:qzc for debug
            // LOG_INFO("line={}, line_id={}", lane_str.c_str(), i);
        }
        if (global_pcd->size() > 0) {
            pcl::io::savePCDFile<PointElement>(cache_file.c_str(), *global_pcd);
            LOG_INFO("cache lane_line[file={}]", cache_file);
            utils::ElementCloudPtr lane_pcd(new utils::ElementCloud);
            *lane_pcd = *global_pcd;
            for (int i = 0; i < lane_pcd->points.size(); ++i) {
                auto &pt = lane_pcd->points[i];
                pt.x += this->_center_pos.x();
                pt.y += this->_center_pos.y();
                pt.z += this->_center_pos.z();
            }
            auto global_cache_file = utils::fmt("{}_global.pcd", cache_file);
            pcl::io::savePCDFile<PointElement>(global_cache_file.c_str(), *lane_pcd);
            LOG_INFO("cache lane_line[file={}]", global_cache_file);
        }
    } else {
        if (pcl::io::loadPCDFile<PointElement>(cache_file.c_str(), *global_pcd) == -1) {
            LOG_ERROR("failed to read pcd[{}]", cache_file);
            return fsdmap::FAIL;;
        }
    }
    for (int j = 0; j < global_pcd->points.size(); ++j) {
        auto &pt = global_pcd->points[j];
        auto feature = std::make_shared<Feature>();
        int64_t line_id = (int)pt.intensity;
        int64_t line_index = (int)(pt.intensity - line_id) * 10000;
        feature->line_id = std::to_string(line_id);
        feature->line_index = line_index;
        feature->frame_id = "";
        feature->pos = {pt.x, pt.y, pt.z};
        feature->raw_pos = feature->pos;
        pcl::PointXYZI searchPt(pt.x, pt.y, 0, -1);
        std::vector<float> squaredDistances;
        bool has_found = search_nearest_pose(searchPt, squaredDistances);
        if(has_found) {
            feature->key_pose = id_2_pose_map[int(searchPt.intensity)].get();
        } else {
            feature->key_pose = NULL;
        }
        feature->score = 0;
        feature->type = get_std_attr("mmt_to_std", "road_boundary", "type", pt.type1);
        // LOG_INFO("boundary feature : type:{} color:{}",feature->type , pt.type1);
        // std::cout << "road_boundary type " << feature->type << std::endl;

        // TODO:qzc change it !
        int data_type = 0;
        if (this->_data_type == "MMT_RC") {
            data_type = 10;
        } else if (this->_data_type == "BYD_BEV" || this->_data_type == "BYD_LIDAR_B" || this->_data_type == "BYD_LIDAR_BEV_B") {
            data_type = 29;
        }
        // feature->type = data_type;
        feature->ele_type = this->get_ele_type(version, data_type);
        if (!is_in_processe_scope(feature->pos)) {
            continue;
        }
        feature_vec.push_back(feature);
    }

    return fsdmap::SUCC;
}


int BevDataProcessor::download_intersection(std::vector<std::shared_ptr<Intersection>> &inter_list) {

    utils::ElementCloud::Ptr global_pcd(new utils::ElementCloud);
    std::vector<std::string> trail_list;
    std::vector<std::string> obj_list;

    //读入路口边界
    std::string junction_bd_dir = utils::fmt("{}/../{}/junction_lukoubd.pcd",
            FLAGS_base_dir, FLAGS_mapping_object_dir);
    if(boost::filesystem::exists(junction_bd_dir)) {
        utils::ElementCloud::Ptr obj_pcd(new utils::ElementCloud);
        if (pcl::io::loadPCDFile<PointElement>(junction_bd_dir, *obj_pcd) == -1) {
            LOG_ERROR("failed to read pcd[{}]", junction_bd_dir);
            return fsdmap::FAIL;
        }
        if (obj_pcd->points.size() == 0) {
            return fsdmap::SUCC;
        }
        UMAP<int64_t, std::shared_ptr<Intersection>> ele_map;
        for (int j = 0; j < obj_pcd->points.size(); ++j) {
            auto &pt = obj_pcd->points[j];
            int64_t line_id = pt.id;
            int64_t line_index = pt.index;
            auto feature = std::make_shared<RoadObjectPointInfo>();
            if (MAP_NOT_FIND(ele_map, line_id)) {
                ele_map[line_id] = std::make_shared<Intersection>();
                auto &obj = ele_map[line_id];
                obj->pos = {0, 0, 0};
                obj->ele_type = pt.ele_type;
            }
            auto &obj = ele_map[line_id];
            obj->point_info.push_back(feature);
            feature->pos = {pt.x, pt.y, pt.z};
            feature->type = pt.type1;
            feature->line_id = std::to_string(line_id);
            feature->line_index = line_index;
            obj->lukou_poly_pts.push_back(feature->pos);
            obj->pos += feature->pos;
        }
        for (auto &it : ele_map) {
            auto &obj = it.second;
            obj->pos /= obj->point_info.size();

            //半径
            obj->radius = 0;
            for(auto &pt : obj->point_info) {
                double dis = alg::calc_dis(pt->pos, obj->pos);
                if(dis > obj->radius) {
                    obj->radius = dis;
                }
            }
            
            int line_id = std::stoi(obj->point_info[0]->line_id);
            obj->id = obj->ele_type * 1e8 + line_id;
            if (!is_in_processe_scope(obj->pos)) {
                continue;
            }
            if (obj->point_info.size() > 1) {
                obj->dir = alg::get_dir(obj->point_info[0]->pos, obj->point_info[1]->pos);
            }

            inter_list.push_back(obj);
        }
    }

    return fsdmap::SUCC;
}


int BevDataProcessor::download_nodes(std::vector<std::shared_ptr<LinkNodes>> &node_list,
                                    std::unordered_map<uint64_t, std::shared_ptr<LinkNodes>>& node_id_map) {
    
    std::string file_name = FLAGS_middle_json_path;
    std::ifstream json_file;
    json_file.open(file_name);

    if (!json_file.is_open()) {
        return fsdmap::FAIL;
    }
    nlohmann::json root = nlohmann::json::parse(json_file);
    auto &json_node_list = root["middle"]["nodes"];

    for (int i = 0; i < json_node_list.size(); ++i) {
        const auto &json_node = json_node_list[i];
        // std::string node_id = utils::fmt("{}", json_node["node_id"].get<int64_t>());
        // auto &bind_trails = json_node["tracks"];
        auto tar_node = std::make_shared<LinkNodes>();
        tar_node->node_id = utils::safe_get<uint64_t>(json_node,"node_id",0);
        tar_node->cross_flag = utils::safe_get<int>(json_node,"cross_flag",0); 
        tar_node->main_node_id = std::stol(utils::safe_get<std::string>(json_node,"main_node_id","")); 
        
        auto &sub_node_ids = json_node["sub_node_id"];
        for (int j = 0; j < sub_node_ids.size(); ++j) {
            uint64_t sub_node_id = std::stol(sub_node_ids[j].get<std::string>());
            if (sub_node_id == 0) {
                continue;
            }
            tar_node->sub_node_ids.push_back(sub_node_id);
        }

        auto &node_link_ids = json_node["node_link_id"];
        for (int j = 0; j < node_link_ids.size(); ++j) {
            auto node_link_id = node_link_ids[j].get<std::string>();
            if (node_link_id == "" || node_link_id == "0") {
                continue;
            }
            tar_node->node_link_ids.push_back(node_link_id);
        }

        auto &cross_link_ids = json_node["cross_link_id"];
        for (int j = 0; j < cross_link_ids.size(); ++j) {
            auto cross_link_id = cross_link_ids[j].get<std::string>();
            if (cross_link_id == "" || cross_link_id == "0") {
                continue;
            }
            tar_node->cross_link_ids.push_back(cross_link_id);
        }

        auto &geoms = json_node["geom"];
        if (geoms.size() == 2) {
            Eigen::Vector3d wgs = {geoms[0].get<double>(), geoms[1].get<double>(), 0};
            this->wgs2local(wgs, tar_node->pos);
        } else {
            LOG_WARN("sdnode geom wrong");
        }

        node_list.push_back(tar_node);
        node_id_map.insert({tar_node->node_id, tar_node});
    }

    return fsdmap::SUCC;
}

int BevDataProcessor::download_link(std::vector<std::shared_ptr<KeyPoseLine>> &link_list, 
        std::vector<std::shared_ptr<KeyPose>> &link_point_list, std::vector<GisPolygon> &polygon_list, std::vector<Eigen::Vector3d> &polygon_vec3d) {

    std::string file_name = FLAGS_middle_json_path;
    std::ifstream json_file;
    json_file.open(file_name);

    if (!json_file.is_open()) {
        return fsdmap::FAIL;
    }
    nlohmann::json root = nlohmann::json::parse(json_file);
    auto &json_link_list = root["middle"]["links"];
    bool has_task_polygon = false;
    
    if (FLAGS_polygon_use_task && root["middle"].contains("task_geom")) {
        //TODO cxf  原来的_valid_polygen 是polygon， task_polygon是muti_是polygon, 统一简化代码
        _valid_polygen.resize(_valid_polygen.size() + 1);
        auto &polygen = _valid_polygen.back();
        boost::geometry::read_wkt(root["middle"]["task_geom"], polygen);
        for (auto &pt : polygen.outer()) {
            Eigen::Vector3d wgs = {boost::geometry::get<0>(pt), boost::geometry::get<1>(pt), 0};
            Eigen::Vector3d tar = wgs;
            this->wgs2local(wgs, tar);
            pt = {tar.x(), tar.y()};
        }
        has_task_polygon = true;

    }
    
    //add
    // if (FLAGS_polygon_use_task && root["middle"].contains("task_geom_local")) {
    if (FLAGS_polygon_use_task) {
        polygon_list.clear();
        GisMultiPolygon multipolygon;
        if(root["middle"]["task_geom_local"] != "") {
            boost::geometry::read_wkt(root["middle"]["task_geom_local"], multipolygon);
            for (auto &polygon : multipolygon) {
                // 遍历多边形的外环点
                for (auto &pt : polygon.outer()) {
                    // WGS 坐标系的转换
                    Eigen::Vector3d wgs = {boost::geometry::get<0>(pt), boost::geometry::get<1>(pt), 0};
                    Eigen::Vector3d tar = wgs;
                    // this->wgs2local(wgs, tar);  
                    pt = {tar.x(), tar.y()}; 
                }
                polygon_list.push_back(polygon);  // 添加多边形到 polygon_list
            }
        }

        //转化成vec3d
        for (const auto& polygon : polygon_list) {
            for (const auto& pt : polygon.outer()) {
                double x = boost::geometry::get<0>(pt);
                double y = boost::geometry::get<1>(pt);
                double z = 0.0;  
                Eigen::Vector3d vec(x, y, z);
                polygon_vec3d.push_back(vec);
            }
        }
    }

    if (FLAGS_polygon_use_custom) {
        bool has_data = false;
        do {
            if (!boost::filesystem::exists(FLAGS_polygon_custom_file)) {
                break;
            }
            std::ifstream json_file;
            json_file.open(FLAGS_polygon_custom_file);
            if (!json_file.is_open()) {
                break;
            }
            nlohmann::json root = nlohmann::json::parse(json_file);
            // boost::iostreams::stream<boost::iostreams::file_source> file(FLAGS_polygon_custom_file);
            // std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            if (root.size() == 0) {
                break;
            }
            _valid_polygen.clear();
            _valid_polygen.resize(_valid_polygen.size() + 1);
            for (auto &content : root) {
                auto &polygen = _valid_polygen.back();
                boost::geometry::read_wkt(content, polygen);
                for (auto &pt : polygen.outer()) {
                    Eigen::Vector3d wgs = {boost::geometry::get<0>(pt), boost::geometry::get<1>(pt), 0};
                    Eigen::Vector3d tar = wgs;
                    this->wgs2local(wgs, tar);
                    pt = {tar.x(), tar.y()};
                }
                has_data = true;
            }
            has_task_polygon = true;
        } while (0);
        if (!has_data) {
            LOG_ERROR("failed to find custom polygon[{}]", FLAGS_polygon_custom_file);
        }
    }
    
    for (int i = 0; i < json_link_list.size(); ++i) {
        const auto &json_link = json_link_list[i];
        std::string link_id = utils::fmt("{}", json_link["link_id"].get<int64_t>());
        auto &bind_trails = json_link["tracks"];
        auto tar_link = std::make_shared<KeyPoseLine>();
        tar_link->id = link_id;
        tar_link->link_direction =utils::safe_get<int>(json_link,"link_direction",0); 
        tar_link->kind =utils::safe_get<int>(json_link,"kind",0); 
        tar_link->lanenum_sum =utils::safe_get<int>(json_link,"lanenum_sum",0); 
        tar_link->start_node_id =utils::safe_get<uint64_t>(json_link,"start_node_id",0); 
        tar_link->end_node_id =utils::safe_get<uint64_t>(json_link,"end_node_id",0); 
        tar_link->width =utils::safe_get<int>(json_link,"width",0);   //暂时没有

        // tar_link->form =utils::safe_get<std::string>(json_link,"form",""); 
        // std::cout << "tar_link->form " << tar_link->form << std::endl;
        auto &forms = json_link["form"];
        for (int j = 0; j < forms.size(); ++j) {
            auto form = std::stol(forms[j].get<std::string>());
            tar_link->forms.insert(form);
        }

        // json_link["link_direction"].get<int>();
        // tar_link->kind = json_link["kind"].get<int>();
        // tar_link->form = json_link["form"].get<std::string>();
        // tar_link->lanenum_sum = json_link["lanenum_sum"].get<int>();
        // tar_link->start_node_id = json_link["start_node_id"].get<std::string>();
        // tar_link->end_node_id = json_link["end_node_id"].get<std::string>();
        // tar_link->width = json_link["end_node_id"].get<int>();
      
        
        for (int j = 0; j < bind_trails.size(); ++j) {
            auto &track = bind_trails[j];
            tar_link->bind_trail_id.push_back(
                    utils::fmt("{}_{}", track["tile_id"].get<int64_t>(), track["trail_id"].get<int64_t>()));
        }
        GisPolyline new_line;
        boost::geometry::read_wkt(json_link["link_geom"], new_line);

        KeyPose* prev_poss = NULL;
        for (auto &pt : new_line) {
            auto tar_pt = std::make_shared<KeyPose>();
            tar_pt->line_id = link_id;
            tar_pt->from_link=tar_link.get();
            tar_pt->from_raw_link=tar_link.get();
            Eigen::Vector3d wgs = {pt.get<0>(), pt.get<1>(), 0};
            this->wgs2local(wgs, tar_pt->pos);
            link_point_list.push_back(tar_pt);
            tar_link->list.push_back(tar_pt.get());
            if (prev_poss != NULL) {
                prev_poss->next = tar_pt.get();
                tar_pt->prev = prev_poss;
                prev_poss->dir = alg::get_dir(tar_pt->pos, prev_poss->pos);
                tar_pt->dir = prev_poss->dir;
            }
            prev_poss = tar_pt.get();
        }
        link_list.push_back(tar_link);
        // 处理范围
        if (!has_task_polygon) {
            _valid_polygen.resize(_valid_polygen.size() + 1);
            auto &polygen = _valid_polygen.back();
            boost::geometry::read_wkt(json_link["task_geom"], polygen);
            for (auto &pt : polygen.outer()) {
                Eigen::Vector3d wgs = {boost::geometry::get<0>(pt), boost::geometry::get<1>(pt), 0};
                Eigen::Vector3d tar = wgs;
                this->wgs2local(wgs, tar);
                pt = {tar.x(), tar.y()};
            }
        }
    }

    return fsdmap::SUCC;
}


int BevDataProcessor::download_feature(std::vector<std::shared_ptr<Feature>> &feature_vec) {
    std::vector<std::string> trail_list;
    std::string base_dir;
    std::string bev_dir = FLAGS_base_dir;
    if (FLAGS_bev_dir != "") {
        bev_dir = FLAGS_bev_dir;
    }
    if (FLAGS_use_middle_enable || FLAGS_tile_id == 0) {
        base_dir = utils::fmt("{}/*/*", bev_dir);
        utils::glob_dir(base_dir.c_str(), trail_list);
    } else {
        base_dir = utils::fmt("{}/{}/*", bev_dir, FLAGS_tile_id);
        utils::glob_dir(base_dir.c_str(), trail_list);
    }

    for (int i = 0; i < trail_list.size(); ++i) {
        auto &trail_dir = trail_list[i];
        std::string file_dir = utils::fmt("{}/{}", trail_dir, FLAGS_bev_label_file);
        boost::filesystem::path trail_path(trail_dir);
        std::string trail_id = trail_path.filename().string();
        std::string tile_id = trail_path.parent_path().filename().string();
        if (FLAGS_bev_trail_id != 0 && std::stol(trail_id) != FLAGS_bev_trail_id) {
            LOG_DEBUG("skip trail[{}]", trail_id);
            continue;
        }

        std::vector<std::string> file_list;
        utils::glob_dir(file_dir.c_str(), file_list);

        for (auto json_file : file_list) {
            _thread_pool->schedule([&feature_vec, json_file, trail_id, this](utils::ProcessBar *bar) {
                boost::filesystem::path path(json_file);
                // std::string frame_id = utils::fmt("{}_{}_{}", tile_id, trail_id, path.stem().string());
                std::string frame_id = get_frame_id(trail_id, path.stem().string());
                // LOG_INFO("feature_key_frame{}", frame_id);
                if (MAP_NOT_FIND(this->_key_map, frame_id)) {
                    LOG_DEBUG("failed to match key frame[file={}]", json_file);
                    return;
                }
                // if (frame_id != "6181675437743_1675437757645000") {
                //     int a = 1;
                //     continue;
                // }
                std::vector<std::shared_ptr<Feature>> frame_feature_vec;
                read_feature_from_json(json_file, trail_id, frame_id, frame_feature_vec);
                if (frame_feature_vec.size() > 0) {
                    std::unique_lock<std::mutex> lk(bar->mutex);
                    VEC_PUSH_ALL(feature_vec, frame_feature_vec);
                }
            });
        }
    }
    _thread_pool->wait(1, "read bev_label");
    LOG_INFO("succ to download_feature[size={}]", feature_vec.size());
    return fsdmap::SUCC;
}

int BevDataProcessor::read_feature_from_json(const std::string &file_name, std::string trail_id, std::string frame_id, 
        std::vector<std::shared_ptr<Feature>> &feature_vec) {
    double x_min = FLAGS_bev_scope_x_min;
    double x_max = FLAGS_bev_scope_x_max;
    double y_min = FLAGS_bev_scope_y_min;
    double y_max = FLAGS_bev_scope_y_max;
    std::ifstream json_file;
    json_file.open(file_name);

    if (!json_file.is_open()) {
        return fsdmap::FAIL;
    }
    nlohmann::json root;
    try {
        root = nlohmann::json::parse(json_file);
    } catch (const std::exception& e) {
        LOG_ERROR("failed to parse[{}, msg={}]", file_name, e.what());
        return fsdmap::FAIL;
    }
    auto &ins_list = root["detail"]["instance_list"];
    std::string version = "0";
    if (root.contains("cat_version")) {
        version = root["cat_version"];
    }

    int64_t skip_num = 0;
    
    for (int i = 0; i < ins_list.size(); ++i) {
        const auto &ins = ins_list[i];
        // if (ins["task"].get<std::string>() != "bev_static") {
        //     continue;
        // }

        std::string ins_id = utils::fmt("{}_{}", frame_id, i);
        int data_type = ins["attrs"]["type"].get<int>();
        double data_score = ins["attrs"]["score"].get<double>();
        auto json_data = ins["data"];
        if (json_data.contains("points")){
            auto data_point = ins["data"]["points"];
            if (data_score < FLAGS_bev_lable_min_score || data_score > FLAGS_bev_lable_max_score) {
                skip_num += data_point.size();
                continue;
            }

            Feature* prev_feature = NULL;
            for (int j = 0; j < data_point.size(); ++j) {
                const auto &point = data_point[j];
                auto feature = std::make_shared<Feature>();
                feature->line_id = ins_id;
                feature->line_index = j;
                feature->frame_id = frame_id;
                feature->trail_id = trail_id;
                feature->raw_pos = {point[0].get<double>(), point[1].get<double>(), -2};
                if (FLAGS_bev_data_use_scope) {
                    if (feature->raw_pos.x() < x_min || feature->raw_pos.x() > x_max) {
                        ++skip_num;
                        continue;
                    }
                    if (feature->raw_pos.y() < y_min || feature->raw_pos.y() > y_max) {
                        ++skip_num;
                        continue;
                    }
                }
                auto key_pose = this->get_key_pose(frame_id);
                // alg::rotate_yaw(feature->raw_pos, feature->pos, key_pose->yaw); 
                // 从自车坐标系转换到第一帧车辆坐标系下的全局坐标
                feature->pos = key_pose->r * feature->raw_pos;
                feature->pos += key_pose->pos;
                if (!is_in_processe_scope(feature->pos)) {
                    ++skip_num;
                    continue;
                }
                feature->key_pose = key_pose;
                feature->score = data_score;
                feature->type = data_type;
                feature->ele_type = this->get_ele_type(version, data_type);

                feature_vec.push_back(feature);
                if (prev_feature != NULL) {
                    feature->set_prev(prev_feature);
                }
                prev_feature = feature.get();
            }
        }
        //红绿灯
        else if(json_data.contains("3dbox")){
            auto data_point = ins["data"]["3dbox"];
            if (data_score < FLAGS_bev_lable_min_score || data_score > FLAGS_bev_lable_max_score) {
                skip_num += data_point.size();
                continue;
            }
            Feature* prev_feature = NULL;
            for (int j = 0; j < data_point.size(); ++j) {
                const auto &point = data_point[j];
                auto feature = std::make_shared<Feature>();
                feature->line_id = ins_id;
                feature->line_index = j;
                feature->frame_id = frame_id;
                feature->trail_id = trail_id;
                feature->raw_pos = {point[0].get<double>(), point[1].get<double>(), point[2].get<double>()};
                if (FLAGS_bev_data_use_scope) {
                    if (feature->raw_pos.x() < x_min || feature->raw_pos.x() > x_max) {
                        ++skip_num;
                        continue;
                    }
                    if (feature->raw_pos.y() < y_min || feature->raw_pos.y() > y_max) {
                        ++skip_num;
                        continue;
                    }
                }
                auto key_pose = this->get_key_pose(frame_id);
                // alg::rotate_yaw(feature->raw_pos, feature->pos, key_pose->yaw);
                feature->pos = key_pose->r * feature->raw_pos;
                feature->pos += key_pose->pos;
                if (!is_in_processe_scope(feature->pos)) {
                    ++skip_num;
                    continue;
                }
                feature->key_pose = key_pose;
                feature->score = data_score;
                feature->type = data_type;
                feature->ele_type = this->get_ele_type(version, data_type);

                feature_vec.push_back(feature);
                if (prev_feature != NULL) {
                    feature->set_prev(prev_feature);
                }
                prev_feature = feature.get();
            }
        }
    }

    LOG_DEBUG("download feature[frame={}, ins_size={}, f_size={}, skip_num={}]", 
            frame_id, ins_list.size(), feature_vec.size(), skip_num);
    return fsdmap::SUCC;
}

int BevDataProcessor::download_feature_from_sem_client(
        std::vector<std::shared_ptr<LaneFeature>> &lane_feature,
        std::vector<std::shared_ptr<BoundaryFeature>> &boundary_feature) {
    std::vector<std::string> trail_list;
    std::string base_dir;
    std::string bev_dir = FLAGS_base_dir;
    base_dir = utils::fmt("{}/../data/*", bev_dir);
    utils::glob_dir(base_dir.c_str(), trail_list);

    for (int i = 0; i < trail_list.size(); ++i) {
        auto &trail_dir = trail_list[i];
        std::string file_dir = utils::fmt("{}/{}", trail_dir, FLAGS_client_label_file);
        boost::filesystem::path trail_path(trail_dir);
        std::string trail_id = trail_path.filename().string();
        std::string tile_id = trail_path.parent_path().filename().string();
        std::vector<std::string> file_list;
        utils::glob_dir(file_dir.c_str(), file_list);

        for (auto &json_file : file_list) {
            boost::filesystem::path path(json_file);
            // std::string frame_id = utils::fmt("{}_{}_{}", tile_id, trail_id, path.stem().string());
            std::string frame_id = get_frame_id(trail_id, path.stem().string());
            // LOG_INFO("feature_key_frame{}", frame_id);
            if (MAP_NOT_FIND(this->_key_map, frame_id)) {
                LOG_DEBUG("failed to match key frame[file={}]", json_file);
                continue;
            }
            read_client_feature_from_json(json_file, trail_id, frame_id, lane_feature, boundary_feature);
        }
    }
    return fsdmap::SUCC;
}

int BevDataProcessor::read_client_feature_from_json(std::string &file_name, 
        std::string trail_id, std::string frame_id, 
        std::vector<std::shared_ptr<LaneFeature>> &lane_feature,
        std::vector<std::shared_ptr<BoundaryFeature>> &boundary_feature) {
    double x_min = FLAGS_client_scope_x_min;
    double x_max = FLAGS_client_scope_x_max;
    double y_min = FLAGS_client_scope_y_min;
    double y_max = FLAGS_client_scope_y_max;
    std::ifstream json_file;
    json_file.open(file_name);

    if (!json_file.is_open()) {
        return fsdmap::FAIL;
    }
    nlohmann::json root = nlohmann::json::parse(json_file);
    int64_t lane_skip_num = 0;
    int64_t boundary_skip_num = 0;
    auto &lane_list = root["LaneBoundary"];

    for (int i = 0; i < lane_list.size(); ++i) {
        const auto &ins = lane_list[i];
        std::string ins_id = utils::fmt("{}_{}", frame_id, i);
        int geo_type = ins["geo_type"].get<int>();
        int color_type = ins["color_type"].get<int>();
        int diversion = ins["isDiversion"].get<int>();
        auto data_point = ins["points"];
        for (int j = 0; j < data_point.size(); ++j) {
            const auto &point = data_point[j];
            auto feature = std::make_shared<LaneFeature>();
            feature->line_id = ins_id;
            feature->line_index = j;
            feature->trail_id = trail_id;
            feature->frame_id = frame_id;
            feature->raw_pos = {std::stod(point[0].get<std::string>()), 
                std::stod(point[1].get<std::string>()), -2};
            if (FLAGS_client_data_use_scope) {
                if (feature->raw_pos.x() < x_min || feature->raw_pos.x() > x_max) {
                    ++lane_skip_num;
                    continue;
                }
                if (feature->raw_pos.y() < y_min || feature->raw_pos.y() > y_max) {
                    ++lane_skip_num;
                    continue;
                }
            }
            auto key_pose = this->get_key_pose(frame_id);
            // alg::rotate_yaw(feature->raw_pos, feature->pos, key_pose->yaw);
            feature->pos = key_pose->r * feature->raw_pos;
            feature->pos += key_pose->pos;
            if (!is_in_processe_scope(feature->pos)) {
                ++lane_skip_num;
                continue;
            }
            feature->key_pose = key_pose;
            feature->attr.geo = geo_type;
            feature->attr.color = color_type;
            feature->attr.diversion = diversion;
            lane_feature.push_back(feature);
        }
    }
    auto &boundary_list = root["RoadBoundary"];

    for (int i = 0; i < boundary_list.size(); ++i) {
        const auto &ins = boundary_list[i];
        std::string ins_id = utils::fmt("{}_{}", frame_id, i);
        int sub_type = 0;
        if (ins.contains("sub_type")) {
            sub_type = ins["sub_type"].get<int>();
        }
        auto data_point = ins["points"];
        for (int j = 0; j < data_point.size(); ++j) {
            const auto &point = data_point[j];
            auto feature = std::make_shared<BoundaryFeature>();
            feature->line_id = ins_id;
            feature->line_index = j;
            feature->frame_id = frame_id;
            // feature->raw_pos = {point[0].get<double>(), point[1].get<double>(), -2};
            feature->raw_pos = {std::stod(point[0].get<std::string>()), 
                std::stod(point[1].get<std::string>()), -2};
            if (FLAGS_client_data_use_scope) {
                if (feature->raw_pos.x() < x_min || feature->raw_pos.x() > x_max) {
                    ++boundary_skip_num;
                    continue;
                }
                if (feature->raw_pos.y() < y_min || feature->raw_pos.y() > y_max) {
                    ++boundary_skip_num;
                    continue;
                }
            }
            auto key_pose = this->get_key_pose(frame_id);
            // alg::rotate_yaw(feature->raw_pos, feature->pos, key_pose->yaw);
            feature->pos = key_pose->r * feature->raw_pos;
            feature->pos += key_pose->pos;
            if (!is_in_processe_scope(feature->pos)) {
                ++boundary_skip_num;
                continue;
            }
            feature->key_pose = key_pose;
            feature->sub_type = sub_type;
            boundary_feature.push_back(feature);
        }
    }

    LOG_DEBUG("download client lane feature[frame={}, ins_size={}, f_size={}, skip_num={}]", 
            frame_id, lane_list.size(), lane_feature.size(), lane_skip_num);
    LOG_DEBUG("download client boundary feature[frame={}, ins_size={}, f_size={}, skip_num={}]", 
            frame_id, boundary_list.size(), boundary_feature.size(), boundary_skip_num);
    return fsdmap::SUCC;
}

int BevDataProcessor::download_object(
        std::vector<std::shared_ptr<RoadObjectInfo>> &raw_object_list) {
    utils::ElementCloud::Ptr global_pcd(new utils::ElementCloud);
    std::vector<std::string> trail_list;
    std::vector<std::string> obj_list;
    std::string obj_dir = utils::fmt("{}/../{}/object_*.pcd",
            FLAGS_base_dir, FLAGS_mapping_object_dir);
    utils::glob_dir(obj_dir.c_str(), obj_list);
    for (int i = 0; i < obj_list.size(); ++i) {
        auto &obj_str = obj_list[i];
        utils::ElementCloud::Ptr obj_pcd(new utils::ElementCloud);
        if (pcl::io::loadPCDFile<PointElement>(obj_str.c_str(), *obj_pcd) == -1) {
            LOG_ERROR("failed to read pcd[{}]", obj_str);
            return fsdmap::FAIL;
        }
        if (obj_pcd->points.size() == 0) {
            continue;
        }
        UMAP<int64_t, std::shared_ptr<RoadObjectInfo>> ele_map;  // <目标id, 目标所包含的点云类型等属性>
        for (int j = 0; j < obj_pcd->points.size(); ++j) {
            auto &pt = obj_pcd->points[j];
            int64_t line_id = pt.id;
            int64_t line_index = pt.index;
            auto feature = std::make_shared<RoadObjectPointInfo>();
            if (MAP_NOT_FIND(ele_map, line_id)) { // 判断该line_id的目标是否已经存在
                ele_map[line_id] = std::make_shared<RoadObjectInfo>();
                auto &obj = ele_map[line_id];
                obj->pos = {0, 0, 0};
                obj->ele_type = pt.ele_type;
                obj->score = pt.score;
                if (obj->ele_type == 3 || obj->ele_type == 4) { // 箭头和其他地面标识
                    std::vector<uint8_t> attr_vec = get_std_attrs("mmt_to_std", "road_mark", "type", pt.type1);
                    std::string type_str = "";
                    for (size_t i = 0; i < attr_vec.size(); ++i) {
                        type_str += std::to_string(attr_vec[i]);
                        if (i < attr_vec.size() - 1) {
                            type_str += ",";
                        }
                    }
                    obj->type = type_str;
                    // std::cout << "road_mark type: " << obj->type << std::endl;
                } else if (obj->ele_type == 5) { // 人行横道
                    obj->type = std::to_string(0);
                } else if (obj->ele_type == 6) { // 停止线
                    obj->type = std::to_string(get_std_attr("mmt_to_std", "stop_line", "type", pt.type1));
                    // std::cout << "stop_line type: " << obj->type << std::endl;
                } else {
                    obj->type = std::to_string(0);
                }
            }
            auto &obj = ele_map[line_id];
            obj->list.push_back(feature);  // 往该目标里面加入对应的点云
            feature->pos = {pt.x, pt.y, pt.z};
            feature->raw_pos = feature->pos;
            // feature->pos.x() += this->_center_pos.x();
            // feature->pos.y() += this->_center_pos.y();
            // feature->pos.z() += this->_center_pos.z();
            feature->type = pt.type1;
            //TODO:cxf 去掉raw_line_
            feature->line_id = std::to_string(line_id);
            feature->line_index = line_index;
            obj->pos += feature->pos;
        }
        for (auto &it : ele_map) {
            auto &obj = it.second;
            obj->pos /= obj->list.size();   //计算目标的平均距离（应该是离自车的距离）

            int line_id = std::stoi(obj->list[0]->line_id);
            obj->obj_id = 5e8 +  obj->ele_type * 1e7 + line_id;

            if(obj->list.size() > 1){
                if(obj->ele_type == 6){
                     //停止线
                    float length =alg::calc_dis(obj->list[0]->pos, obj->list[1]->pos); 
                    obj->dir = alg::get_dir(obj->list[1]->pos, obj->list[0]->pos);
                    obj->length = length;    
                }
                else if(obj->ele_type == 5){
                    //人行横道
                    float length = 0;
                    int max_index = 0;
                    for(int i = 0 ; i < obj->list.size() - 1; i++){
                        float length_tmp =alg::calc_dis(obj->list[i+1]->pos, obj->list[i]->pos);
                        if(length_tmp > length){
                            max_index = i;
                            length = length_tmp;
                        }
                    }
                    obj->dir = alg::get_dir(obj->list[max_index + 1]->pos, obj->list[max_index]->pos);
                    obj->length = length;   
                } 
                else{
                    //箭头
                    obj->dir = alg::get_dir(obj->list[1]->pos, obj->list[0]->pos);
                }

            }
    
            if (!is_in_processe_scope(obj->pos)) {
                continue;
            }

            raw_object_list.push_back(obj);

        }
    }

    return fsdmap::SUCC;
}


int BevDataProcessor::download_junction(std::vector<std::shared_ptr<JunctionInfo>> &raw_junction_list, 
        std::vector<std::shared_ptr<ImpassableAreaInfo>> &raw_area_list) {
    utils::ElementCloud::Ptr global_pcd(new utils::ElementCloud);
    std::vector<std::string> trail_list;
    std::vector<std::string> obj_list;

    //读入路口边界
    std::string junction_bd_dir = utils::fmt("{}/../{}/junction_lukoubd.pcd",
            FLAGS_base_dir, FLAGS_mapping_object_dir);
    if(boost::filesystem::exists(junction_bd_dir))
    {
        utils::ElementCloud::Ptr obj_pcd(new utils::ElementCloud);
        if (pcl::io::loadPCDFile<PointElement>(junction_bd_dir, *obj_pcd) == -1) {
            LOG_ERROR("failed to read pcd[{}]", junction_bd_dir);
            return fsdmap::FAIL;
        }
        if (obj_pcd->points.size() == 0) {
            return fsdmap::SUCC;
        }
        UMAP<int64_t, std::shared_ptr<JunctionInfo>> ele_map;
        for (int j = 0; j < obj_pcd->points.size(); ++j) {
            auto &pt = obj_pcd->points[j];
            int64_t line_id = pt.id;
            int64_t line_index = pt.index;
            auto feature = std::make_shared<RoadObjectPointInfo>();
            if (MAP_NOT_FIND(ele_map, line_id)) {
                ele_map[line_id] = std::make_shared<JunctionInfo>();
                auto &obj = ele_map[line_id];
                obj->pos = {0, 0, 0};
                obj->ele_type = pt.ele_type;
            }
            auto &obj = ele_map[line_id];
            obj->point_info.push_back(feature);
            feature->pos = {pt.x, pt.y, pt.z};
            feature->type = pt.type1;
            feature->line_id = std::to_string(line_id);
            feature->line_index = line_index;
            obj->lukou_poly_pts.push_back(feature->pos);
            obj->pos += feature->pos;
        }
        for (auto &it : ele_map) {
            auto &obj = it.second;
            obj->pos /= obj->point_info.size();
            if (!is_in_processe_scope(obj->pos)) {
                continue;
            }
            raw_junction_list.push_back(obj);
            if (obj->point_info.size() > 1) {
                obj->dir = alg::get_dir(obj->point_info[0]->pos, obj->point_info[1]->pos);
            }
        }
    }

    //读入不可通行区域
    std::string area_dir = utils::fmt("{}/../{}/junction_impassablearea.pcd",
            FLAGS_base_dir, FLAGS_mapping_object_dir);
    if(boost::filesystem::exists(area_dir))
    {
        utils::ElementCloud::Ptr obj_pcd(new utils::ElementCloud);
        if (pcl::io::loadPCDFile<PointElement>(area_dir, *obj_pcd) == -1) {
            LOG_ERROR("failed to read pcd[{}]", area_dir);
            return fsdmap::FAIL;
        }
        if (obj_pcd->points.size() == 0) {
            return fsdmap::SUCC;
        }
        UMAP<int64_t, std::shared_ptr<ImpassableAreaInfo>> ele_map;
        for (int j = 0; j < obj_pcd->points.size(); ++j) {
            auto &pt = obj_pcd->points[j];
            int64_t line_id = pt.id;
            int64_t line_index = pt.index;
            auto feature = std::make_shared<RoadObjectPointInfo>();
            if (MAP_NOT_FIND(ele_map, line_id)) {
                ele_map[line_id] = std::make_shared<ImpassableAreaInfo>();
                auto &obj = ele_map[line_id];
                obj->pos = {0, 0, 0};
                obj->ele_type = pt.ele_type;
                obj->type = pt.type1;
            }
            auto &obj = ele_map[line_id];
            obj->list.push_back(feature);
            feature->pos = {pt.x, pt.y, pt.z};
            feature->raw_pos = feature->pos;
            feature->type = pt.type1;
            feature->line_id = std::to_string(line_id);
            feature->line_index = line_index;
            obj->pos += feature->pos;
        }
        for (auto &it : ele_map) {
            auto &obj = it.second;
            obj->pos /= obj->list.size();
            if (!is_in_processe_scope(obj->pos)) {
                continue;
            }
            raw_area_list.push_back(obj);
            if (obj->list.size() > 1) {
                obj->dir = alg::get_dir(obj->list[0]->pos, obj->list[1]->pos);
            }
        }
    }

    return fsdmap::SUCC;
}

bool BevDataProcessor::wgs2local(Eigen::Vector3d &wgs, Eigen::Vector3d &local_pos, bool need_offset) {
    local_pos = wgs;
    if (FLAGS_use_middle_enable || FLAGS_custom_utm_enable) {
        utils::wgs_2_utm(_utm_zone, wgs, local_pos);
    } else {
        if (!_proj15.WGS_to_tile_local(this->_tile_id, local_pos.x(), local_pos.y(), local_pos.z())) {
            return false;
        }
    }
    if (need_offset) {
        local_pos.x() -= this->_center_pos.x();
        local_pos.y() -= this->_center_pos.y();
    }
    // Eigen::Vector3d new_wgs = {0, 0, 0};
    // this->local2wgs(local_pos, new_wgs);
    // LOG_DEBUG("debug coor raw={},{}  new={},{}", wgs.x(), wgs.y(), new_wgs.x(), new_wgs.y());
    return true;
}

bool BevDataProcessor::wgs2local(int32_t utm_zone, Eigen::Vector3d &wgs, Eigen::Vector3d &local_pos, bool need_offset) {
    local_pos = wgs;
   
    utils::wgs_2_utm(utm_zone, wgs, local_pos);

    if (need_offset) {
        local_pos.x() -= this->_center_pos.x();
        local_pos.y() -= this->_center_pos.y();
    }
    // Eigen::Vector3d new_wgs = {0, 0, 0};
    // this->local2wgs(local_pos, new_wgs);
    // LOG_DEBUG("debug coor raw={},{}  new={},{}", wgs.x(), wgs.y(), new_wgs.x(), new_wgs.y());
    return true;
}

bool BevDataProcessor::local2wgs(Eigen::Vector3d &local_pos, Eigen::Vector3d &wgs, bool need_offset) {
    Eigen::Vector3d raw_pos = local_pos;
    if (need_offset) {
        raw_pos.x() += this->_center_pos.x();
        raw_pos.y() += this->_center_pos.y();
    }

    if (FLAGS_use_middle_enable || FLAGS_custom_utm_enable) {
        utils::utm_2_wgs(_utm_zone, raw_pos, wgs);
    } else {
        wgs = raw_pos;
        if (!this->_proj15.tile_local_to_WGS(this->_tile_id, wgs.x(), wgs.y(), wgs.z())) {
            return false;
        }
    }
    return true;
}

bool BevDataProcessor::is_in_processe_scope(Eigen::Vector3d &pos) {
    // polygen
    if (FLAGS_use_middle_enable && FLAGS_scope_use_link) {
        int valid_num = 0;
        for (auto &poly : _valid_polygen) {
            if (boost::geometry::within(GisPt(pos.x(), pos.y()), poly)) {
                valid_num = 1;
                break;
            }
        }
        if (valid_num == 0) {
            return false;
        }
    }
    if (!FLAGS_valid_enable) {
        return true;
    }
    Eigen::Vector3d trans = {FLAGS_valid_x, FLAGS_valid_y, 0};
    // Eigen::Vector3d local_pos = pos;
    // utils::trans_display_local_pos(_scope, local_pos);
    // double dis = alg::calc_dis(trans, local_pos);
    double dis = alg::calc_dis(trans, pos);
    if (dis < FLAGS_valid_dis) {
        return true;
    }
    return false;
}

utils::DisplayScope& BevDataProcessor::init_display_scope() {
    double scope_buff = FLAGS_display_scope_buff;
    double min_x = 0;
    double min_y = 0;
    double max_x = 0;
    double max_y = 0;
    double total_z = 0;
    int i = 0;
    for (auto &kit : this->_key_map) {
        auto key_pose = kit.second.first.get();
        if (!is_in_processe_scope(key_pose->pos)) {
            continue;
        }
        total_z += key_pose->pos.z();
        if (i == 0) {
            min_x = key_pose->pos.x();
            min_y = key_pose->pos.y();
            max_x = key_pose->pos.x();
            max_y = key_pose->pos.y();
            ++i;
            continue;
        }
        min_x = std::min(min_x, key_pose->pos.x());
        min_y = std::min(min_y, key_pose->pos.y());
        max_x = std::max(max_x, key_pose->pos.x());
        max_y = std::max(max_y, key_pose->pos.y());
    }
    min_x -= scope_buff;
    min_y -= scope_buff;
    max_x += scope_buff;
    max_y += scope_buff;
    total_z /= this->_key_map.size();
    _scope.center_pos = {(min_x + max_x) / 2, (min_y + max_y) / 2, 0};
    _scope.x_radius = (max_x - min_x) / 2;
    _scope.y_radius = (max_y - min_y) / 2;
    _scope.set_resolution(FLAGS_display_scale_rate);
    return _scope;
}

utils::DisplayScope& BevDataProcessor::get_display_scope(std::string frame_id) {
    if (MAP_NOT_FIND(_key_map, frame_id)) {
        return _scope;
    }
    return _key_map[frame_id].second;
};

std::string BevDataProcessor::get_log_dir() {
    boost::filesystem::path path(utils::fmt("{}", FLAGS_base_log_dir));
    return path.string();
}

int BevDataProcessor::init_ele_type() {
    // TODO:qzc delete all
    {
        this->ele_type_map["BYD_BEV"] = UMAP<int, std::tuple<ELEMENT_TYPE, int, int, cv::Scalar>>();
        auto &type_map = ele_type_map["BYD_BEV"];

        // ###################################################################################################
        // !!!!!!!!! 注意，此处在拼图的时候，复用了 mmt_rc 的格式，所以，这里的映射也改为了 和 mmt_rc 一致 ！！！！！！
        // !!!!!!!!! 注意，此处在拼图的时候，复用了 mmt_rc 的格式，所以，这里的映射也改为了 和 mmt_rc 一致 ！！！！！！
        // ###################################################################################################
        // raw_bev_id = {类型，color， shape， 可视化颜色)
        type_map[0] = {ELEMENT_LANE_LINE, 0, 0, {255, 255, 255}}; // 车道线，UNKNOWN_COLOR，UNKNOWN
        type_map[1] = {ELEMENT_LANE_LINE, 0, 2, {255, 255, 255}}; // 车道线，UNKNOWN_COLOR，dashed
        type_map[2] = {ELEMENT_LANE_LINE, 0, 1, {255, 255, 255}}; // 车道线，UNKNOWN_COLOR，solid
        type_map[3] = {ELEMENT_LANE_LINE, 0, 5, {255, 255, 255}}; // 车道线，UNKNOWN，double dashed
        type_map[4] = {ELEMENT_LANE_LINE, 0, 6, {255, 255, 255}}; // 车道线，UNKNOWN，double solid
        type_map[5] = {ELEMENT_LANE_LINE, 0, 3, {255, 255, 255}}; // 车道线，UNKNOWN，left_dashed_right_solid
        type_map[6] = {ELEMENT_LANE_LINE, 0, 4, {255, 255, 255}}; // 车道线，UNKNOWN，left_solid_right_dashed

        type_map[7] = {ELEMENT_LANE_LINE, 1, 0, {255, 255, 255}};  // 车道线，白色线类，UNKNOWN
        type_map[8] = {ELEMENT_LANE_LINE, 1, 2, {255, 255, 255}};  // 车道线，白色线类，dashed
        type_map[9] = {ELEMENT_LANE_LINE, 1, 1, {255, 255, 255}}; // 车道线，白色线类，solid
        type_map[10] = {ELEMENT_LANE_LINE, 1, 5, {255, 255, 255}}; // 车道线，白色线类，double dashed
        type_map[11] = {ELEMENT_LANE_LINE, 1, 6, {255, 255, 255}}; // 车道线，白色线类，double solid
        type_map[12] = {ELEMENT_LANE_LINE, 1, 3, {255, 255, 255}}; // 车道线，白色线类，left_dashed_right_solid
        type_map[13] = {ELEMENT_LANE_LINE, 1, 4, {255, 255, 255}}; // 车道线，白色线类，left_solid_right_dashed

        type_map[14] = {ELEMENT_LANE_LINE, 2, 0,{255, 255, 0}}; // 车道线，黄色线类，UNKNOWN
        type_map[15] = {ELEMENT_LANE_LINE, 2, 2,{255, 255, 0}}; // 车道线，黄色线类，dashed
        type_map[16] = {ELEMENT_LANE_LINE, 2, 1,{255, 255, 0}}; // 车道线，黄色线类，solid
        type_map[17] = {ELEMENT_LANE_LINE, 2, 5,{255, 255, 0}}; // 车道线，黄色线类，double dashed
        type_map[18] = {ELEMENT_LANE_LINE, 2, 6,{255, 255, 0}}; // 车道线，黄色线类，double solid
        type_map[19] = {ELEMENT_LANE_LINE, 2, 3,{255, 255, 0}}; // 车道线，黄色线类，double dashed
        type_map[20] = {ELEMENT_LANE_LINE, 2, 4,{255, 255, 0}}; // 车道线，黄色线类，double solid

        type_map[27]  = {ELEMENT_BARRIER, 0, 0, (152, 99, 60)};   // 道路边界，UNKNOWN
        type_map[28]  = {ELEMENT_BARRIER, 0, 1, (152, 99, 60)};   // 道路边界，mmt:普通, byd: ROADEDGE_TYPE_FLAT
        type_map[29]  = {ELEMENT_BARRIER, 0, 3, (152, 99, 60)};   // 道路边界，mmt:水马, byd: ROADEDGE_TYPE_HIGH
        type_map[30]  = {ELEMENT_BARRIER, 0, 2, (152, 99, 60)};   // 道路边界，mmt:锥桶, byd: ROADEDGE_TYPE_LOW
        type_map[31]  = {ELEMENT_BARRIER, 0, 4, (152, 99, 60)};   // 道路边界，mmt: 锥桶水马混合/施工牌/防撞桶, byd: ROADEDGE_TYPE_FENCE

        type_map[39]  = {ELEMENT_OBJECT, 10, 0, {255, 0, 0}};      // 人行道区域, byd: 

        // 17  = {'ELEMENT_JUNCTION', 1, 0, (100, 255, 100)), // 路口区域

        // 车道中心线
        type_map[40]  = {ELEMENT_LANE_CENTER, 3, 0, {0, 255, 0}};  // 中心线，混合车道，unknown
        type_map[41]  = {ELEMENT_LANE_CENTER, 3, 1, {0, 255, 0}};  // 中心线，混合车道，普通车道
        type_map[42]  = {ELEMENT_LANE_CENTER, 3, 3, {0, 255, 0}};  // 中心线，混合车道，应急车道
        type_map[49]  = {ELEMENT_LANE_CENTER, 3, 4, {0, 255, 0}}; // 中心线，混合车道，不完整车道
        type_map[50]  = {ELEMENT_LANE_CENTER, 3, 5, {0, 255, 0}}; // 中心线，混合车道，不完整车道

        type_map[53]  = {ELEMENT_OBJECT, 7, 0,{255, 255, 0}};   // 地面标识，unknown
        type_map[54]  = {ELEMENT_OBJECT, 7, 11,{255, 255, 0}};  // 地面标识，直行
        type_map[55]  = {ELEMENT_OBJECT, 7, 13,{255, 255, 0}};  // 地面标识，右转
        type_map[56]  = {ELEMENT_OBJECT, 7, 12,{255, 255, 0}};  // 地面标识，左转
        type_map[57]  = {ELEMENT_OBJECT, 7, 14,{255, 255, 0}};  // 地面标识，左转掉头

        type_map[62]  = {ELEMENT_OBJECT, 8, 0, {255, 0, 0}};       // 停止线，unknown
        type_map[63]  = {ELEMENT_OBJECT, 8, 23, {255, 0, 0}};       // 停止线，普通停止线
        
        // other_scripts/msg/perception_msgs/dynamic_common.proto
        // 70  = {ELEMENT_TRAFFICLIGHT, 0, 0,{255, 255, 0}),  // TRAFFIC_LIGHT_GROUP_SHAPE_HORIZONTAL
        // 71  = {ELEMENT_TRAFFICLIGHT, 1, 0,{255, 255, 0}),  // TRAFFIC_LIGHT_GROUP_SHAPE_VERTICAL
        // 72  = {ELEMENT_TRAFFICLIGHT, 2, 0,{255, 255, 0})   // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
    }
    {
        this->ele_type_map["BYD_LIDAR_B"] = UMAP<int, std::tuple<ELEMENT_TYPE, int, int, cv::Scalar>>();
        this->ele_type_map["BYD_LIDAR_B"] = this->ele_type_map["BYD_BEV"];
    }
    {
        this->ele_type_map["BYD_LIDAR_BEV_B"] = UMAP<int, std::tuple<ELEMENT_TYPE, int, int, cv::Scalar>>();
        this->ele_type_map["BYD_LIDAR_BEV_B"] = this->ele_type_map["BYD_BEV"];
    }
    {
        this->ele_type_map["MMT_RC"] = UMAP<int, std::tuple<ELEMENT_TYPE, int, int, cv::Scalar>>();
        auto &type_map = ele_type_map["MMT_RC"];
        type_map[0] = {ELEMENT_LANE_LINE, 0, 0, {255, 255, 255}};//车道线，UNKNOWN_COLOR，UNKNOWN
        type_map[1] = {ELEMENT_LANE_LINE, 0, 1, {255, 255, 255}};//车道线，UNKNOWN_COLOR，dashed
        type_map[2] = {ELEMENT_LANE_LINE, 0, 2, {255, 255, 255}};//车道线，UNKNOWN_COLOR，solid
        type_map[3] = {ELEMENT_LANE_LINE, 1, 0, {255, 255, 255}};//车道线，白色线类，UNKNOWN
        type_map[4] = {ELEMENT_LANE_LINE, 1, 1, {255, 255, 255}};//车道线，白色线类，dashed
        type_map[5] = {ELEMENT_LANE_LINE, 1, 2, {255, 255, 255}};//车道线，白色线类，solid
        type_map[6] = {ELEMENT_LANE_LINE, 2, 0, {255, 255, 0}};//车道线，黄色线类，UNKNOWN
        type_map[7] = {ELEMENT_LANE_LINE, 2, 1, {255, 255, 0}};//车道线，黄色线类，dashed
        type_map[8] = {ELEMENT_LANE_LINE, 2, 2, {255, 255, 0}};//车道线，黄色线类，solid

        type_map[9] = {ELEMENT_BARRIER, 0, 0, {152, 99, 60}};//道路边界，UNKNOWN
        type_map[10] = {ELEMENT_BARRIER, 2, 0, {152, 99, 60}};//道路边界，普通
        type_map[11] = {ELEMENT_BARRIER, 4, 0, {152, 99, 60}};//道路边界，水马
        type_map[12] = {ELEMENT_BARRIER, 5, 0, {152, 99, 60}};//道路边界，锥桶
        type_map[13] = {ELEMENT_BARRIER, 20, 0, {152, 99, 60}};//道路边界，锥桶水马混合/施工牌/防撞桶
        type_map[14] = {ELEMENT_OBJECT,  8, 1, {255, 0, 0}};//停止线
        type_map[15] = {ELEMENT_OBJECT,  10, 0,{255, 0, 0}};// 人行道区域

        type_map[16] = {ELEMENT_JUNCTION, 1, 0, {100, 255, 100}};// 路口区域

        // 车道中心线
        type_map[17] = {ELEMENT_LANE_CENTER, 3, 0, {0, 255, 0}};// lane_center，混合车道，unknown
        type_map[18] = {ELEMENT_LANE_CENTER, 3, 1, {0, 255, 0}};// lane_center，混合车道，普通车道
        type_map[19] = {ELEMENT_LANE_CENTER, 3, 5, {0, 255, 0}};// lane_center，混合车道，应急车道
        type_map[20] = {ELEMENT_LANE_CENTER, 3, 6, {0, 255, 0}};// lane_center，混合车道，公交车道
        type_map[21] = {ELEMENT_LANE_CENTER, 3, 15, {0, 255, 0}};// lane_center，混合车道，非机动车道
        type_map[22] = {ELEMENT_LANE_CENTER, 3, 20, {0, 255, 0}};// lane_center，混合车道，收费站
        type_map[23] = {ELEMENT_LANE_CENTER, 3, 28, {0, 255, 0}};// lane_center，混合车道，可变车道
        type_map[24] = {ELEMENT_LANE_CENTER, 3, 32, {0, 255, 0}};// lane_center，混合车道，潮汐车道
        type_map[25] = {ELEMENT_LANE_CENTER, 3, 71, {0, 255, 0}};// lane_center，混合车道，左右转或者掉头待转区
        type_map[26] = {ELEMENT_LANE_CENTER, 3, 73, {0, 255, 0}};// lane_center，混合车道，不完整车道

        type_map[27] = {ELEMENT_OBJECT, 7, 0, {255, 255, 0}};// 地面标识，unkown
        type_map[28] = {ELEMENT_OBJECT, 7, 1, {255, 255, 0}};// 地面标识，直行
        type_map[29] = {ELEMENT_OBJECT, 7, 2, {255, 255, 0}};// 地面标识，右转
        type_map[30] = {ELEMENT_OBJECT, 7, 3, {255, 255, 0}};// 地面标识，左转
        type_map[31] = {ELEMENT_OBJECT, 7, 4, {255, 255, 0}};// 地面标识，左转掉头
        type_map[32] = {ELEMENT_OBJECT, 7, 5, {255, 255, 0}};// 地面标识，右转掉头
        type_map[33] = {ELEMENT_OBJECT, 7, 6, {255, 255, 0}};// 地面标识，向左合流
        type_map[34] = {ELEMENT_OBJECT, 7, 7, {255, 255, 0}};// 地面标识，向右合流
        type_map[35] = {ELEMENT_OBJECT, 7, 13, {255, 255, 0}};// 地面标识，导流区

        type_map[36] = {ELEMENT_TRAFFICLIGHT, 0, 0, {255, 255, 0}};// 
        type_map[37] = {ELEMENT_TRAFFICLIGHT, 1, 0, {255, 255, 0}};// 
        type_map[38] = {ELEMENT_TRAFFICLIGHT, 2, 0, {255, 255, 0}}; // 
    }
    {
        this->ele_type_map["0"] = UMAP<int, std::tuple<ELEMENT_TYPE, int, int, cv::Scalar>>();
        auto &type_map = ele_type_map["0"];
        type_map[0]  = {ELEMENT_LANE_LINE, 0, 0, {255, 255, 255}};
        type_map[1]  = {ELEMENT_LANE_LINE, 0, 1, {255, 255, 255}};
        type_map[2]  = {ELEMENT_LANE_LINE, 0, 2, {255, 255, 255}};
        type_map[3]  = {ELEMENT_LANE_LINE, 0, 3, {255, 255, 255}};
        type_map[4]  = {ELEMENT_LANE_LINE, 1, 0, {255, 255, 255}};
        type_map[5]  = {ELEMENT_LANE_LINE, 1, 1, {255, 255, 255}};
        type_map[6]  = {ELEMENT_LANE_LINE, 1, 2, {255, 255, 255}};
        type_map[7]  = {ELEMENT_LANE_LINE, 1, 3, {255, 255, 255}};
        type_map[8]  = {ELEMENT_LANE_LINE, 2, 0, {255, 255, 0}};
        type_map[9]  = {ELEMENT_LANE_LINE, 2, 1, {255, 255, 0}};
        type_map[10] = {ELEMENT_LANE_LINE, 2, 2, {255, 255, 0}};
        type_map[11] = {ELEMENT_LANE_LINE, 2, 3, {255, 255, 0}};
        // type, road_boundary, lane_boundary
        type_map[12] = {ELEMENT_BARRIER, 0, 0, {152, 99, 60}};
        type_map[13] = {ELEMENT_CURB,    2, 0, {81, 89, 240}};
        type_map[14] = {ELEMENT_BARRIER, 7, 0, {152, 99, 60}};
        type_map[15] = {ELEMENT_BARRIER, 9, 0, {152, 99, 60}};
        type_map[16] = {ELEMENT_BARRIER, 10, 0, {152, 99, 60}};
        type_map[17] = {ELEMENT_BARRIER, 11, 0, {152, 99, 60}};
        type_map[18] = {ELEMENT_BARRIER, 11, 0, {152, 99, 60}};
        type_map[19] = {ELEMENT_OBJECT,  8, 1, {255, 0, 0}};
        type_map[20] = {ELEMENT_OBJECT,  10, 1, {255, 0, 0}};
        type_map[21] = {ELEMENT_OBJECT,  0, 1, {255, 0, 0}};
        type_map[22] = {ELEMENT_OBJECT,  1, 2, {255, 0, 0}};
        type_map[23] = {ELEMENT_OBJECT,  2, 2, {255, 0, 0}};
        type_map[24] = {ELEMENT_OBJECT,  3, 2, {255, 0, 0}};
        type_map[25] = {ELEMENT_OBJECT,  3, 2, {255, 0, 0}};
        type_map[26] = {ELEMENT_LANE_CENTER, 3, 0, {0, 255, 0}};
    }
    return fsdmap::SUCC;
}

void BevDataProcessor::init_attr_map() {
    {
        this->attr_map["mmt_to_std"] = UMAP<std::tuple<std::string, std::string, uint16_t>, uint16_t>();
        auto &attr_map_out = this->attr_map["mmt_to_std"];
        attr_map_out[std::make_tuple("lane_center", "type", 0)] = 99; // 其他
        attr_map_out[std::make_tuple("lane_center", "type", 1)] = 1; // 常规
        attr_map_out[std::make_tuple("lane_center", "type", 5)] = 99; // 应急车道
        attr_map_out[std::make_tuple("lane_center", "type", 6)] = 99; // 公交车道
        attr_map_out[std::make_tuple("lane_center", "type", 15)] = 99; // 非机动车道
        attr_map_out[std::make_tuple("lane_center", "type", 20)] = 99; // 收费站
        attr_map_out[std::make_tuple("lane_center", "type", 28)] = 99; // 可变车道
        attr_map_out[std::make_tuple("lane_center", "type", 32)] = 99; // 潮汐车道
        attr_map_out[std::make_tuple("lane_center", "type", 71)] = 99; // 左右转或掉头代转区
        attr_map_out[std::make_tuple("lane_center", "type", 73)] = 99; // 不完整车道
        attr_map_out[std::make_tuple("lane_center", "type", 74)] = 99; // block(byd)
        attr_map_out[std::make_tuple("lane_boundary", "type", 0)] = 99; // 其他        
        attr_map_out[std::make_tuple("lane_boundary", "type", 1)] = 3; // 虚线
        attr_map_out[std::make_tuple("lane_boundary", "type", 2)] = 2; // 实线    
        // 新增   
        attr_map_out[std::make_tuple("lane_boundary", "type", 3)] = 4; // 双虚线
        attr_map_out[std::make_tuple("lane_boundary", "type", 4)] = 5; // 双实线
        attr_map_out[std::make_tuple("lane_boundary", "type", 5)] = 7; // 左虚右实线
        attr_map_out[std::make_tuple("lane_boundary", "type", 6)] = 6; // 左实右虚线
        attr_map_out[std::make_tuple("lane_boundary", "type", 7)] = 1; // 虚拟线
        
        attr_map_out[std::make_tuple("lane_boundary", "color", 0)] = 99; // 其他
        attr_map_out[std::make_tuple("lane_boundary", "color", 1)] = 2; // 白色
        attr_map_out[std::make_tuple("lane_boundary", "color", 2)] = 3; // 黄色
        attr_map_out[std::make_tuple("lane_boundary", "color", 3)] = 1; // 虚拟线无颜色

        if (this->_data_type == "MMT_RC") {
            attr_map_out[std::make_tuple("road_boundary", "type", 0)] = 99; // 其他        
            attr_map_out[std::make_tuple("road_boundary", "type", 2)] = 1; // 普通     
            attr_map_out[std::make_tuple("road_boundary", "type", 4)] = 7; // 水马
            attr_map_out[std::make_tuple("road_boundary", "type", 5)] = 8; // 锥桶
            attr_map_out[std::make_tuple("road_boundary", "type", 20)] = 99; // 锥桶水马混合/施工牌/防撞桶
        } else if (this->_data_type == "BYD_BEV" || this->_data_type == "BYD_LIDAR_B" || this->_data_type == "BYD_LIDAR_BEV_B") {
            attr_map_out[std::make_tuple("road_boundary", "type", 0)] = 99; // 其他        
            attr_map_out[std::make_tuple("road_boundary", "type", 2)] = 11; // flat     
            attr_map_out[std::make_tuple("road_boundary", "type", 4)] = 12; // high
            attr_map_out[std::make_tuple("road_boundary", "type", 5)] = 13; // low
            attr_map_out[std::make_tuple("road_boundary", "type", 20)] = 14; // fence
        }

        attr_map_out[std::make_tuple("stop_line", "type", 0)] = 99; // 其他
        attr_map_out[std::make_tuple("stop_line", "type", 1)] = 1; // 普通停止线
        attr_map_out[std::make_tuple("stop_line", "type", 2)] = 1; // 左转待行区停止线
        attr_map_out[std::make_tuple("stop_line", "type", 3)] = 1; // 掉头待行区停止线
        attr_map_out[std::make_tuple("stop_line", "type", 4)] = 1; // 右转待行区停止线
        attr_map_out[std::make_tuple("stop_line", "type", 5)] = 1; // 直行待行区停止线
        attr_map_out[std::make_tuple("road_mark", "type", 0)] = 99; // 其他
        attr_map_out[std::make_tuple("road_mark", "type", 1)] = 1; // 直行
        attr_map_out[std::make_tuple("road_mark", "type", 2)] = 3; // 右转
        attr_map_out[std::make_tuple("road_mark", "type", 3)] = 2; // 左转
        attr_map_out[std::make_tuple("road_mark", "type", 4)] = 4; // 左转掉头
        attr_map_out[std::make_tuple("road_mark", "type", 5)] = 5; // 右转掉头
        attr_map_out[std::make_tuple("road_mark", "type", 6)] = 6; // 向左合流
        attr_map_out[std::make_tuple("road_mark", "type", 7)] = 7; // 向右合流
        attr_map_out[std::make_tuple("road_mark", "type", 13)] = 99; // 导流区
    }
}
    
uint8_t BevDataProcessor::get_std_attr(std::string version, std::string ele_type, std::string attr_name, uint8_t input_value) {
    auto version_it = attr_map.find(version);
    if (version_it == attr_map.end()) {
        throw std::runtime_error("Version not found");
    }

    const auto &ele_type_map = version_it->second;
    auto attr_tuple = std::make_tuple(ele_type, attr_name, input_value);

    uint16_t std_value = 0;
    auto attr_tuple_it = ele_type_map.find(attr_tuple);
    if (attr_tuple_it == ele_type_map.end()) {
        std::cout << "请求的版本: " << version << ", 元素类型: " << ele_type 
                  << ", 属性名称: " << attr_name << ", 原始值: " << input_value << std::endl;     
        // throw std::runtime_error("Attribute not found");
        std_value = 99;//其他
    }
    else {
        std_value = attr_tuple_it->second;
    }

    return std_value;
}

std::vector<uint8_t> BevDataProcessor::get_std_attrs(std::string version, std::string ele_type, std::string attr_name, uint32_t raw_type) {
    std::vector<uint8_t> std_attrs;

    if (raw_type == 0) {
        uint8_t std_attr = get_std_attr(version, ele_type, attr_name, 0);
        std_attrs.push_back(std_attr);
        return std_attrs;
    } 

    for (int i = 0; i < 4; ++i) {
        uint8_t byte = (raw_type >> (8 * i)) & 0xFF; // 有复合属性时, 每个属性占8位，最多支持4个属性组合，属性值有从小到大排列
        if (byte != 0) {
            uint8_t std_attr = get_std_attr(version, ele_type, attr_name, byte);
            std_attrs.push_back(std_attr);
        }       
    }
    std::sort(std_attrs.begin(), std_attrs.end());

    return std_attrs;
}


} // namespace dao
} // namespace fsdmap


