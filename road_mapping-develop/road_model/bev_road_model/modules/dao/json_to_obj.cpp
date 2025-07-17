#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <gflags/gflags.h>
#include "utils/macro_util.h"
#include "utils/log_util.h"
#include "utils/string_util.h"
#include "utils/common_util.h"
#include "utils/visualization_util.h"
#include "dao/data_processer.h"
#include <boost/filesystem.hpp>
#include <thread>

DEFINE_int32(log_level_console, 2, "log_level_console");
DEFINE_int32(log_level_file, 1, "log_level_file");
DEFINE_string(output_path, "/home/test/output/test.obj", "output_path");

std::string get_bin_version()
{
#ifdef VERSION_NUM
    std::string g_version = VERSION_NUM;
#else
    std::string g_version = "N/A";
#endif
#ifdef __COMMIT_ID__
    std::string g_commit = __COMMIT_ID__;
#else
    std::string g_commit = "N/A";
#endif
#ifdef __BRANCH__
    std::string g_branch = __BRANCH__;
#else
    std::string g_branch = "N/A";
#endif
#if defined(__DATE__) && defined(__TIME__)
    std::string g_model_built = __DATE__ " " __TIME__;
#else
    std::string g_model_built = "N/A";
#endif
    std::string algorithm_version;
    algorithm_version += ": " + g_version + ";";
    algorithm_version += "git branch: " + g_branch + ";";
    algorithm_version += "git commit id: " + g_commit + ";";
    algorithm_version += "built date: " + g_model_built + ";";
    return algorithm_version;
}

void FeatureWriteToOBJ(std::vector<std::shared_ptr<fsdmap::Feature>> &key_feature,
                       std::vector<int64_t> &index_list, std::string &path, fsdmap::dao::BevDataProcessor &dp)
{
    std::ofstream ofs(path, std::ofstream::trunc);
    int point_idx = 1;
    std::map<std::string, std::vector<int64_t>> line_map;
    for (auto &index : index_list)
    {
        auto &per_fea = key_feature[index];
        Eigen::Vector3d wgs;
        dp.local2wgs(per_fea->pos, wgs, true);
        ofs.precision(16);
        ofs << "v " << wgs[0] << " " << wgs[1] << " " << wgs[2] << std::endl;
        line_map[per_fea->line_id].push_back(point_idx);
        point_idx += 1;
    }
    std::cout << line_map.size() << std::endl;
    for (auto per_line : line_map)
    {
        if (per_line.second.size() < 2)
        {
            continue;
        }
        ofs << "l";
        for (auto point_idx_l : per_line.second)
        {
            ofs << " " << point_idx_l;
        }
        ofs << std::endl;
    }
    ofs.close();
}

void LaneWriteToOBJ(std::vector<std::shared_ptr<fsdmap::Feature>> &key_feature, std::vector<int64_t> &index_list,
                    std::string objpath, fsdmap::dao::BevDataProcessor &dp)
{
    std::string lane_path = fsdmap::utils::fmt("{}/laneboundarys_bev.obj", objpath);
    std::ofstream ofs(lane_path, std::ofstream::trunc);
    int point_idx = 1;
    std::map<std::string, std::vector<int64_t>> line_map;
    for (auto &index : index_list)
    {
        auto &per_fea = key_feature[index];
        auto &ele_meta = per_fea->ele_type;
        auto ele_type = std::get<0>(ele_meta);
        if (ele_type == fsdmap::ELEMENT_LANE_LINE)
        {
            Eigen::Vector3d wgs;
            dp.local2wgs(per_fea->pos, wgs, true);
            ofs.precision(16);
            ofs << "v " << wgs[0] << " " << wgs[1] << " " << wgs[2] << std::endl;
            line_map[per_fea->line_id].push_back(point_idx);
            point_idx += 1;
        }
    }
    LOG_DEBUG("lane [dir={},ins_size={},feature_size={}]", lane_path, line_map.size(), index_list.size());
    for (auto per_line : line_map)
    {
        if (per_line.second.size() < 2)
        {
            continue;
        }
        ofs << "l";
        for (auto point_idx_l : per_line.second)
        {
            ofs << " " << point_idx_l;
        }
        ofs << std::endl;
    }
    ofs.close();
}

void CenterLineWriteToOBJ(std::vector<std::shared_ptr<fsdmap::Feature>> &key_feature, std::vector<int64_t> &index_list,
                          std::string objpath, fsdmap::dao::BevDataProcessor &dp)
{
    std::string lane_path = fsdmap::utils::fmt("{}/centerlines_bev.obj", objpath);
    std::ofstream ofs(lane_path, std::ofstream::trunc);
    int point_idx = 1;
    std::map<std::string, std::vector<int64_t>> line_map;
    for (auto &index : index_list)
    {
        auto &per_fea = key_feature[index];
        auto &ele_meta = per_fea->ele_type;
        auto ele_type = std::get<0>(ele_meta);
        if (ele_type == fsdmap::ELEMENT_LANE_CENTER)
        {
            Eigen::Vector3d wgs;
            dp.local2wgs(per_fea->pos, wgs, true);
            ofs.precision(16);
            ofs << "v " << wgs[0] << " " << wgs[1] << " " << wgs[2] << std::endl;
            line_map[per_fea->line_id].push_back(point_idx);
            point_idx += 1;
        }
    }
    LOG_DEBUG("centerline [dir={},ins_size={},feature_size={}]", lane_path, line_map.size(), index_list.size());
    for (auto per_line : line_map)
    {
        if (per_line.second.size() < 2)
        {
            continue;
        }
        ofs << "l";
        for (auto point_idx_l : per_line.second)
        {
            ofs << " " << point_idx_l;
        }
        ofs << std::endl;
    }
    ofs.close();
}

void RoadWriteToOBJ(std::vector<std::shared_ptr<fsdmap::Feature>> &key_feature, std::vector<int64_t> &index_list,
                    std::string objpath, fsdmap::dao::BevDataProcessor &dp)
{
    std::string road_path = fsdmap::utils::fmt("{}/roadboundarys_bev.obj", objpath);
    std::ofstream ofs(road_path, std::ofstream::trunc);
    int point_idx = 1;
    std::map<std::string, std::vector<int64_t>> line_map;
    for (auto &index : index_list)
    {
        auto &per_fea = key_feature[index];
        auto &ele_meta = per_fea->ele_type;
        auto ele_type = std::get<0>(ele_meta);
        if (ele_type == fsdmap::ELEMENT_BARRIER || ele_type == fsdmap::ELEMENT_CURB)
        {
            Eigen::Vector3d wgs;
            dp.local2wgs(per_fea->pos, wgs, true);
            ofs.precision(16);
            ofs << "v " << wgs[0] << " " << wgs[1] << " " << wgs[2] << std::endl;
            line_map[per_fea->line_id].push_back(point_idx);
            point_idx += 1;
        }
    }
    LOG_DEBUG("boundary [dir={},ins_size={},feature_size={}]", road_path, line_map.size(), index_list.size());
    for (auto per_line : line_map)
    {
        ofs << "l";
        if (per_line.second.size() < 2)
        {
            continue;
        }
        for (auto point_idx_l : per_line.second)
        {
            ofs << " " << point_idx_l;
        }
        ofs << std::endl;
    }
    ofs.close();
}

void StoplineWriteToOBJ(std::vector<std::shared_ptr<fsdmap::Feature>> &key_feature, std::vector<int64_t> &index_list,
                        std::string objpath, fsdmap::dao::BevDataProcessor &dp)
{
    std::string road_path = objpath + "/stoplines_bev.obj";
    std::ofstream ofs(road_path, std::ofstream::trunc);

    std::string road_path_utm = objpath + "/utm_stoplines_bev.obj";
    std::ofstream ofs_utm(road_path_utm, std::ofstream::trunc);

    int point_idx = 1;
    std::map<std::string, std::vector<int64_t>> line_map;
    for (auto &index : index_list)
    {
        auto &per_fea = key_feature[index];
        auto &ele_meta = per_fea->ele_type;
        auto ele_type = std::get<0>(ele_meta);
        int ele_sub_type = std::get<1>(ele_meta);
        if (ele_type == fsdmap::ELEMENT_OBJECT && ele_sub_type == 8)
        {
            ofs_utm.precision(16);
            ofs_utm << "v " << per_fea->pos[0] << " " << per_fea->pos[1] << " " << per_fea->pos[2] << std::endl;

            Eigen::Vector3d wgs;
            dp.local2wgs(per_fea->pos, wgs, true);
            ofs.precision(16);
            ofs << "v " << wgs[0] << " " << wgs[1] << " " << wgs[2] << std::endl;
            line_map[per_fea->line_id].push_back(point_idx);
            point_idx += 1;
        }
    }
    LOG_DEBUG("stopline [dir={},ins_size={},feature_size={}]", road_path, line_map.size(), index_list.size());
    for (auto per_line : line_map)
    {
        ofs << "l";
        ofs_utm << "l";
        if (per_line.second.size() < 2)
        {
            continue;
        }
        for (auto point_idx_l : per_line.second)
        {
            ofs << " " << point_idx_l;
            ofs_utm << " " << point_idx_l;
        }
        ofs << std::endl;
        ofs_utm << std::endl;
    }
    ofs.close();
    ofs_utm.close();
}

void CrosswalkWriteToOBJ(std::vector<std::shared_ptr<fsdmap::Feature>> &key_feature, std::vector<int64_t> &index_list, std::string objpath, fsdmap::dao::BevDataProcessor &dp)
{
    std::string road_path = objpath + "/crosswalks_bev.obj";
    std::ofstream ofs(road_path, std::ofstream::trunc);

    std::string road_path_utm = objpath + "/utm_crosswalks_bev.obj";
    std::ofstream ofs_utm(road_path_utm, std::ofstream::trunc);

    int point_idx = 1;
    std::map<std::string, std::vector<int64_t>> line_map;
    for (auto index : index_list)
    {
        auto &per_fea = key_feature[index];
        auto &ele_meta = per_fea->ele_type;
        auto ele_type = std::get<0>(ele_meta);
        int ele_sub_type = std::get<1>(ele_meta);
        if (ele_type == fsdmap::ELEMENT_OBJECT && ele_sub_type == 10)
        {
            ofs_utm.precision(16);
            ofs_utm << "v " << per_fea->pos[0] << " " << per_fea->pos[1] << " " << per_fea->pos[2] << std::endl;

            Eigen::Vector3d wgs;
            dp.local2wgs(per_fea->pos, wgs, true);
            ofs.precision(16);
            ofs << "v " << wgs[0] << " " << wgs[1] << " " << wgs[2] << std::endl;
            line_map[per_fea->line_id].push_back(point_idx);
            point_idx += 1;
        }
    }
    LOG_DEBUG("crosswalks [dir={},ins_size={},feature_size={}]", road_path, line_map.size(), index_list.size());
    for (auto per_line : line_map)
    {
        ofs << "l";
        ofs_utm << "l";
        if (per_line.second.size() < 2)
        {
            continue;
        }
        for (auto point_idx_l : per_line.second)
        {
            ofs << " " << point_idx_l;
            ofs_utm << " " << point_idx_l;
        }
        ofs << std::endl;
        ofs_utm << std::endl;
    }
    ofs.close();
    ofs_utm.close();
}

void ArrowWriteToPCD(std::vector<std::shared_ptr<fsdmap::Feature>> &key_feature, std::vector<int64_t> &index_list, std::string pcdpath, fsdmap::dao::BevDataProcessor &dp)
{
    std::string pcd_path = pcdpath + "/arrow_bev.pcd";
    pcl::PointCloud<PointElement>::Ptr pointElement(new pcl::PointCloud<PointElement>);
    std::map<std::string, pcl::PointCloud<PointElement>> line_map;

    for (auto index : index_list)
    {
        auto &per_fea = key_feature[index];
        auto &ele_meta = per_fea->ele_type;
        auto ele_type = std::get<0>(ele_meta);
        int ele_sub_type = std::get<1>(ele_meta);
        int arrow_type = std::get<2>(ele_meta);
        if (ele_type == fsdmap::ELEMENT_OBJECT && ele_sub_type == 7)
        {
            PointElement arrow_point;
            arrow_point.x = per_fea->pos[0];
            arrow_point.y = per_fea->pos[1];
            arrow_point.z = per_fea->pos[2];
            arrow_point.type1 = arrow_type;
            arrow_point.ele_type = 3;
            line_map[per_fea->line_id].points.push_back(arrow_point);
        }
    }
    LOG_DEBUG("arrow [dir={},ins_size={},feature_size={}]", pcd_path, line_map.size(), index_list.size());
    int line_id = 0;
    for (auto per_line : line_map)
    {
        if (per_line.second.points.size() < 2)
        {
            continue;
        }
        for (int i = 0; i < per_line.second.points.size(); i++)
        {
            per_line.second.points[i].index = i;
            per_line.second.points[i].id = line_id;
        }
        *pointElement += per_line.second;
        line_id++;
    }

    if ((!pointElement->empty()) && pointElement->points.size() > 0)
    {
        pcl::io::savePCDFileBinary(pcd_path, *pointElement);
    }
}

void LukouboundaryToPCD(std::vector<std::shared_ptr<fsdmap::Feature>> &key_feature, std::vector<int64_t> &index_list, std::string pcdpath, fsdmap::dao::BevDataProcessor &dp)
{
    std::string road_path = pcdpath + "/lukouboundary_bev.pcd";

    pcl::PointCloud<PointElement>::Ptr pointElement(new pcl::PointCloud<PointElement>);
    std::map<std::string, std::vector<Eigen::Vector3d>> line_map;
    for (auto index : index_list)
    {
        auto &per_fea = key_feature[index];
        auto &ele_meta = per_fea->ele_type;
        auto ele_type = std::get<0>(ele_meta);
        if (ele_type == fsdmap::ELEMENT_JUNCTION)
        {
            line_map[per_fea->line_id].push_back(per_fea->pos);
        }
    }
    LOG_DEBUG("lukouboundary [dir={},ins_size={},feature_size={}]", road_path, line_map.size(), index_list.size());

    int line_id = 0;
    for (auto per_line : line_map)
    {
        if (per_line.second.size() < 2)
        {
            continue;
        }
        for (int i = 0; i < per_line.second.size(); i++)
        {
            PointElement newPclPt;
            newPclPt.x = per_line.second[i][0];
            newPclPt.y = per_line.second[i][1];
            newPclPt.z = per_line.second[i][2];
            newPclPt.ele_type = 7;
            newPclPt.index = i;
            newPclPt.id = line_id;
            pointElement->points.push_back(newPclPt);
        }
        line_id++;
    }
    if ((!pointElement->empty()) && pointElement->points.size() > 0)
    {
        pcl::io::savePCDFileBinary(road_path, *pointElement);
    }
}

void WriteToOBJ(std::vector<std::shared_ptr<fsdmap::Feature>> &key_feature,
                std::string objpath, fsdmap::dao::BevDataProcessor &dp)
{
    // <trail_id, <frame_id, index>>
    UMAP<std::string, UMAP<std::string, std::vector<int64_t>>> trail_map_lane;
    UMAP<std::string, UMAP<std::string, std::vector<int64_t>>> trail_map_boundary;
    UMAP<std::string, UMAP<std::string, std::vector<int64_t>>> trail_map_center_line;
    UMAP<std::string, UMAP<std::string, std::vector<int64_t>>> trail_map_stopline;
    UMAP<std::string, UMAP<std::string, std::vector<int64_t>>> trail_map_crosswalk;
    UMAP<std::string, UMAP<std::string, std::vector<int64_t>>> trail_map_lukouboundary;
    UMAP<std::string, UMAP<std::string, std::vector<int64_t>>> trail_map_arrow;
    std::vector<std::thread> threads;

    for (int64_t i = 0; i < key_feature.size(); ++i)
    {
        auto &per_fea = key_feature[i];
        auto &ele_meta = per_fea->ele_type;
        auto ele_type = std::get<0>(ele_meta);
        int ele_sub_type = std::get<1>(ele_meta);
        std::vector<std::string> tmp;
        fsdmap::utils::split(per_fea->frame_id, "_", tmp);
        auto &frame_id = tmp[1];
        if (ele_type == fsdmap::ELEMENT_LANE_LINE)
        {
            trail_map_lane[per_fea->trail_id][frame_id].push_back(i);
        }
        else if (ele_type == fsdmap::ELEMENT_LANE_CENTER)
        {
            trail_map_center_line[per_fea->trail_id][frame_id].push_back(i);
        }
        else if (ele_type == fsdmap::ELEMENT_BARRIER || ele_type == fsdmap::ELEMENT_CURB)
        {
            trail_map_boundary[per_fea->trail_id][frame_id].push_back(i);
        }
        else if (ele_type == fsdmap::ELEMENT_OBJECT && ele_sub_type == 8)
        {
            trail_map_stopline[per_fea->trail_id][frame_id].push_back(i);
        }
        else if (ele_type == fsdmap::ELEMENT_OBJECT && ele_sub_type == 10)
        {
            trail_map_crosswalk[per_fea->trail_id][frame_id].push_back(i);
        }
        else if (ele_type == fsdmap::ELEMENT_JUNCTION)
        {
            trail_map_lukouboundary[per_fea->trail_id][frame_id].push_back(i);
        }
        else if (ele_type == fsdmap::ELEMENT_OBJECT && ele_sub_type == 7)
        {
            trail_map_arrow[per_fea->trail_id][frame_id].push_back(i);
        }
    }

    // <trail_id, <frame_id, index>>
    for (auto &tit : trail_map_arrow)
    {
        std::vector<int64_t> trail_list;
        for (auto &fit : tit.second)
        {
            VEC_INSERT_ALL(trail_list, fit.second);
            auto fptr = &fit;
            std::string path = fsdmap::utils::fmt("{}/{}/{}", objpath, tit.first, fit.first);
            std::string cmd = "mkdir -p " + path;
            std::system(cmd.c_str());
            dp._thread_pool->schedule([&, path, fptr](fsdmap::utils::ProcessBar *bar)
                                      { ArrowWriteToPCD(key_feature, fptr->second, path, dp); });
        }
        std::string path = fsdmap::utils::fmt("{}/{}", objpath, tit.first);
        ArrowWriteToPCD(key_feature, trail_list, path, dp);
    }

    for (auto &tit : trail_map_lane)
    {
        std::vector<int64_t> trail_list;
        for (auto &fit : tit.second)
        {
            VEC_INSERT_ALL(trail_list, fit.second);
            auto fptr = &fit;
            std::string path = fsdmap::utils::fmt("{}/{}/{}", objpath, tit.first, fit.first);
            std::string cmd = "mkdir -p " + path;
            std::system(cmd.c_str());
            // if (fit.first == "1672018297680000") {
            //     int a = 1;
            // }
            dp._thread_pool->schedule([&, path, fptr](fsdmap::utils::ProcessBar *bar)
                                      { LaneWriteToOBJ(key_feature, fptr->second, path, dp); });
            // LOG_INFO("{}", path);
        }
        std::string path = fsdmap::utils::fmt("{}/{}", objpath, tit.first);
        LaneWriteToOBJ(key_feature, trail_list, path, dp);
    }
    for (auto &tit : trail_map_center_line)
    {
        std::vector<int64_t> trail_list;
        for (auto &fit : tit.second)
        {
            VEC_INSERT_ALL(trail_list, fit.second);
            auto fptr = &fit;
            std::string path = fsdmap::utils::fmt("{}/{}/{}", objpath, tit.first, fit.first);
            std::string cmd = "mkdir -p " + path;
            std::system(cmd.c_str());
            dp._thread_pool->schedule([&, path, fptr](fsdmap::utils::ProcessBar *bar)
                                      { CenterLineWriteToOBJ(key_feature, fptr->second, path, dp); });
        }
        std::string path = fsdmap::utils::fmt("{}/{}", objpath, tit.first);
        CenterLineWriteToOBJ(key_feature, trail_list, path, dp);
    }
    for (auto &tit : trail_map_boundary)
    {
        std::vector<int64_t> trail_list;
        for (auto &fit : tit.second)
        {
            VEC_INSERT_ALL(trail_list, fit.second);
            auto fptr = &fit;
            std::string path = fsdmap::utils::fmt("{}/{}/{}", objpath, tit.first, fit.first);
            std::string cmd = "mkdir -p " + path;
            std::system(cmd.c_str());
            dp._thread_pool->schedule([&, path, fptr](fsdmap::utils::ProcessBar *bar)
                                      { RoadWriteToOBJ(key_feature, fptr->second, path, dp); });
        }
        std::string path = fsdmap::utils::fmt("{}/{}", objpath, tit.first);
        RoadWriteToOBJ(key_feature, trail_list, path, dp);
    }
    for (auto &tit : trail_map_stopline)
    {
        std::vector<int64_t> trail_list;
        for (auto &fit : tit.second)
        {
            VEC_INSERT_ALL(trail_list, fit.second);
            auto fptr = &fit;
            std::string path = fsdmap::utils::fmt("{}/{}/{}", objpath, tit.first, fit.first);
            std::string cmd = "mkdir -p " + path;
            std::system(cmd.c_str());
            dp._thread_pool->schedule([&, path, fptr](fsdmap::utils::ProcessBar *bar)
                                      { StoplineWriteToOBJ(key_feature, fptr->second, path, dp); });
        }
        std::string path = fsdmap::utils::fmt("{}/{}", objpath, tit.first);
        StoplineWriteToOBJ(key_feature, trail_list, path, dp);
    }
    for (auto &tit : trail_map_crosswalk)
    {
        std::vector<int64_t> trail_list;
        for (auto &fit : tit.second)
        {
            VEC_INSERT_ALL(trail_list, fit.second);
            auto fptr = &fit;
            std::string path = fsdmap::utils::fmt("{}/{}/{}", objpath, tit.first, fit.first);
            std::string cmd = "mkdir -p " + path;
            std::system(cmd.c_str());
            dp._thread_pool->schedule([&, path, fptr](fsdmap::utils::ProcessBar *bar)
                                      { CrosswalkWriteToOBJ(key_feature, fptr->second, path, dp); });
        }
        std::string path = fsdmap::utils::fmt("{}/{}", objpath, tit.first);
        CrosswalkWriteToOBJ(key_feature, trail_list, path, dp);
    }
    for (auto &tit : trail_map_lukouboundary)
    {
        std::vector<int64_t> trail_list;
        for (auto &fit : tit.second)
        {
            VEC_INSERT_ALL(trail_list, fit.second);
            auto fptr = &fit;
            std::string path = fsdmap::utils::fmt("{}/{}/{}", objpath, tit.first, fit.first);
            std::string cmd = "mkdir -p " + path;
            std::system(cmd.c_str());
            dp._thread_pool->schedule([&, path, fptr](fsdmap::utils::ProcessBar *bar)
                                      { LukouboundaryToPCD(key_feature, fptr->second, path, dp); });
        }
        std::string path = fsdmap::utils::fmt("{}/{}", objpath, tit.first);
        LukouboundaryToPCD(key_feature, trail_list, path, dp);
    }
    dp._thread_pool->wait(2, "process obj");
}

int main(int argc, char *argv[])
{
    google::SetVersionString(get_bin_version());
    google::ParseCommandLineFlags(&argc, &argv, false);
    auto logger = fsdmap::utils::Logger::instance();
    // logger->setLogLevel(spdlog::level::info, spdlog::level::info);
    logger->setLogLevel(fsdmap::utils::LogLevel(FLAGS_log_level_console),
                        fsdmap::utils::LogLevel(FLAGS_log_level_file));
    logger->get_timer().set_unit_level(4);
    logger->get_timer().start();
    logger->setup("log/road_model.log");

    std::vector<std::shared_ptr<fsdmap::KeyPose>> key_poses;
    fsdmap::dao::BevDataProcessor dp;
    std::vector<std::shared_ptr<fsdmap::Feature>> key_feature;
    if (dp.init() != fsdmap::SUCC)
    {
        CLOG_ERROR("failed to init road_model");
        return -1;
    }
    std::vector<std::shared_ptr<fsdmap::KeyPoseLine>> raw_links;
    std::vector<std::shared_ptr<fsdmap::KeyPose>> raw_links_points;
    std::vector<fsdmap::dao::GisPolygon> polygon_list;
    std::vector<Eigen::Vector3d> polygon_vec3d;

    dp.download_link(raw_links, raw_links_points, polygon_list, polygon_vec3d);
    dp.download_key_pose(key_poses, false);
    // std::cout<<"test key_pose size is "<<key_poses.size()<<std::endl;
    LOG_INFO("key pose size {}", key_poses.size());

    dp.download_feature(key_feature);
    // std::cout<<"test key_feature size is "<<key_feature.size()<<std::endl;
    LOG_INFO("key feature size {}", key_feature.size());

    std::string output_path = FLAGS_output_path;
    std::string cmd = "mkdir -p " + output_path;
    system(cmd.c_str());
    WriteToOBJ(key_feature, output_path, dp);
    // LaneWriteToOBJ(key_feature, output_path,dp);
    // RoadWriteToOBJ(key_feature, output_path,dp);
    // CenterLineWriteToOBJ(key_feature, output_path,dp);
    // StoplineWriteToOBJ(key_feature, output_path,dp);
    // CrosswalkWriteToOBJ(key_feature, output_path,dp);
}
