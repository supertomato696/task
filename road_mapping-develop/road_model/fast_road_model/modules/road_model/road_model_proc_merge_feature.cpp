

#include "road_model_proc_merge_feature.h"
#include "utils/algorithm_util.h"
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include "utils/polyfit.h"
#include <iostream>
#include <sstream>

DECLARE_double(display_scope_buff);

DEFINE_bool(merge_feature_enable, true, "merge_feature_enable");
DEFINE_bool(merge_feature_debug_pos_enable, true, "merge_feature_debug_enable");
DEFINE_bool(merge_feature_save_data_enable, true, "merge_feature_save_data_enable");
DEFINE_bool(merge_feature_save_data_frame_log_enable, false, "merge_feature_save_data_frame_log_enable");
DEFINE_bool(merge_feature_save_data_multi_log_enable, true, "merge_feature_save_data_multi_log_enable");

DEFINE_double(merge_feature_vote_yellow_min_threshold, 1.5, "merge_feature_vote_yellow_min_threshold");
DEFINE_double(merge_feature_vote_yellow_max_threshold, 2.5, "merge_feature_vote_yellow_max_threshold");
DEFINE_double(merge_feature_vote_double_value_threshold, 0.7, "merge_feature_vote_double_value_threshold;");
DEFINE_double(merge_feature_vote_yellow_radius, 5, "merge_feature_vote_yellow_radius");
DEFINE_double(merge_feature_vote_yellow_dis_treshold, 1, "merge_feature_vote_yellow_dis_treshold");
DEFINE_double(merge_feature_vote_yellow_scope, 3, "merge_feature_vote_yellow_scope");
DEFINE_double(merge_feature_vote_yellow_dis_lambda, 0.5, "merge_feature_vote_yellow_dis_lambda");
DEFINE_double(merge_feature_vote_yellow_theta_treshold, 30, "merge_feature_vote_yellow_theta_treshold");
DEFINE_double(merge_feature_match_rate_threshold, 0.3, "merge_feature_match_rate_threshold");
DEFINE_double(merge_feature_scope, 1, "merge_feature_scope");
DEFINE_double(merge_feature_search_radius, 5, "merge_feature_search_radius");
DEFINE_double(merge_feature_theta, 20, "merge_feature_theta");
DEFINE_double(merge_feature_proc_min_size, 10, "merge_feature_proc_min_size");
DEFINE_double(merge_feature_proc_min_match_size, 1, "merge_feature_proc_min_match_size");
DEFINE_double(merge_feature_proc_score_threshold, 0.2, "merge_feature_proc_score_threshold");
DEFINE_double(merge_feature_match_ground_radius, 1, "merge_feature_match_ground_radius");
DEFINE_double(merge_feature_mutil_scope, 1, "merge_feature_mutil_scope");
// DEFINE_double(merge_feature_search_radius, 5, "merge_feature_search_radius");
DEFINE_double(merge_feature_mutil_theta, 20, "merge_feature_mutil_theta");
DEFINE_double(merge_feature_theta_thres, 360, "merge_feature_theta_thres");
DEFINE_double(merge_feature_single_h_dis, 30, "merge_feature_single_h_dis");
DEFINE_double(merge_feature_single_v_dis, 30, "merge_feature_single_v_dis");
// DEFINE_double(merge_feature_match_rate_threshold, 0.8, "merge_feature_match_rate_threshold");
// DEFINE_double(merge_feature_iou_thres_1, 0.85, "merge_feature_iou_thres_1");
// DEFINE_double(merge_feature_iou_thres_2, 0.5, "merge_feature_iou_thres_2");
DEFINE_double(merge_feature_valid_min_length, 20, "merge_feature_valid_min_length");
DEFINE_double(merge_feature_match_line_long_scope, 20, "merge_feature_match_line_long_scope");
DEFINE_double(merge_feature_match_line_long_max_dis_threshold, 2, "merge_feature_match_line_long_max_dis_threshold");
DEFINE_double(merge_feature_single_boundary_length_thres, 5, "merge_feature_single_boundary_length_thres");
DEFINE_double(merge_feature_boundary_theta_thres, 15, "merge_feature_boundary_theta_thres");



namespace fsdmap {
namespace road_model {


fsdmap::process_frame::PROC_STATUS RoadModelProcMergeFeature::proc(
        RoadModelSessionData* session) {
    session->enable_debug_pos = FLAGS_merge_feature_debug_pos_enable;
    if (!FLAGS_merge_feature_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    // 将车道线、车道边界等的采样点和pose绑定，并根与自车的距离计算分数
    CHECK_FATAL_PROC(match_key_pose(session), "match_key_pose");

    // 单轨迹merge：边界线、车道线、车道中心线
    CHECK_FATAL_PROC(merge_single_trail(session), "merge_single_trail_boundary");

    CHECK_FATAL_PROC(remove_short_lines(session),"remove_short_lines");

    CHECK_FATAL_PROC(make_right_turn(session),"make_right_turn");

    CHECK_FATAL_PROC(check_direction(session), "check_direction");

    CHECK_FATAL_PROC(make_tree(session), "make_tree");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info"); // 存储融合结果，用于可视化
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}


int RoadModelProcMergeFeature::match_key_pose(RoadModelSessionData* session) {
    KeyPoseTree poss_tree; // ins数据
    for (auto &it : session->key_pose_map) {
        for (auto &poss : it.second.list) {
            poss_tree.insert(poss->pos, poss);
        }
    }
    for (auto &[frame_id, list] : session->bev_frame_lane_line) { // bev感知车道线的采样结果
        for (auto &line : list) {
            for (auto pt : line->list) {
                session->thread_pool->schedule([&, pt, session, this](
                            utils::ProcessBar *process_bar) {
                        session->debug_pos(pt->pos);
                        auto ret = search_key_pose(session, poss_tree, pt->pos, pt->dir,
                                pt->src->trail_id, pt->src->key_pose);
                        pt->match_frame_num = ret.first;  // 离pt位置较近（轨迹上存在某点pose，pt到pose的方向向量和方向向量垂线距离都小于30m）的frame数量
                        pt->match_trail_num = ret.second; // 离pt位置较近（轨迹上存在某点pose，pt到pose的方向向量距离小于30m）的轨迹条数
                        // pt->src->score = 1e-6;
                        pt->src->score = 1;
                        // pt->src->score = get_feature_score_by_distance(session, pt->src);
                        // if (pt->src->score <= 0) {
                        //    pt->filter_status = 4;
                        // } // modified by ccj, 20241221, pt raw_pos目前是相对于地图起点的pos，不再是相对于自车坐标的pos, score计算不出来
                        });
            }
        }
    }
    for (auto &[frame_id, list] : session->bev_frame_lane_center_line) { // bev感知车道中心线的采样结果
        for (auto &line : list) {
            for (auto pt : line->list) {
                session->thread_pool->schedule([&, pt, session, this](
                            utils::ProcessBar *process_bar) {
                        auto ret = search_key_pose(session, poss_tree, pt->pos, pt->dir,
                                pt->trail_id, pt->key_pose);
                        pt->match_frame_num = ret.first;
                        pt->match_trail_num = ret.second;
                        // pt->score = 1e-6;
                        pt->score = 1;
                        // pt->score = get_feature_score_by_distance(session, pt);
                        // if (pt->score <= 0) {
                        //    pt->filter_status = 4;
                        // } // modified by ccj, 20241221, pt raw_pos目前是相对于地图起点的pos，不再是相对于自车坐标的pos, score计算不出来
                        });
            }
        }
    }
    //bev_frame_boundary_line： 集合BEV感知的栅栏信息(session->barrier_instance_map)、BEV感知的路沿信息(session->curb_instance_map)、
    //BEV感知的boundary信息(session->sem_curb_instance_map)，对曲线做平滑后的结果
    for (auto &[frame_id, list] : session->bev_frame_boundary_line) {
        for (auto &line : list) {
            for (auto pt : line->list) {
                session->thread_pool->schedule([&, pt, session, this](
                            utils::ProcessBar *process_bar) {
                        auto ret = search_key_pose(session, poss_tree, pt->pos, pt->dir,
                                pt->trail_id, pt->key_pose);
                        pt->match_frame_num = ret.first;
                        pt->match_trail_num = ret.second;
                        //根据feature离自车位置远近，计算feature的可靠性，离自车越近，可靠性分数越高
                        // pt->score = 1e-6;
                        pt->score = 1;
                        // pt->score = get_feature_score_by_distance(session, pt);
                        // if (pt->score <= 0) {
                        //     pt->filter_status = 4;
                        // } // modified by ccj, 20241221, pt raw_pos目前是相对于地图起点的pos，不再是相对于自车坐标的pos, score计算不出来 
                        });
            }
        }
    }
    session->thread_pool->wait(5, "match_frame");
    return fsdmap::SUCC;
}


// 计算两点之间的欧几里得距离
float distance(const pcl::PointXYZINormal& p1, const pcl::PointXYZINormal& p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

// 返回向量的点积
double dotProduct(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
    return v1.dot(v2);
}

// 计算方向差的代价（方向越接近代价越小）
double directionCost(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
    double mag1 = v1.norm();
    double mag2 = v2.norm();
    if (mag1 < 1e-6 || mag2 < 1e-6) {
        return 1.0;  // 方向不明，设最大代价
    }
    double cosine = dotProduct(v1, v2) / (mag1 * mag2);
    return 1.0 - cosine;  // 越接近，值越小
}

// 计算两法线之间的夹角
float directionCost(const pcl::PointXYZINormal& p1, const pcl::PointXYZINormal& p2)
{
    float dotProduct = p1.normal_x * p2.normal_x + p1.normal_y * p2.normal_y + p1.normal_z * p2.normal_z;
    float mag1       = std::sqrt(p1.normal_x * p1.normal_x + p1.normal_y * p1.normal_y + p1.normal_z * p1.normal_z);
    float mag2       = std::sqrt(p2.normal_x * p2.normal_x + p2.normal_y * p2.normal_y + p2.normal_z * p2.normal_z);
    if (mag1 < 1e-6 || mag2 < 1e-6) {
        return 1.0;  // 方向不明，设最大代价
    }
    double cosine = dotProduct / (mag1 * mag2);
    // return 1 - acos(dotProduct / (magnitude1 * magnitude2));
    return 1.0 - cosine;  // 越接近，值越小
}

// 计算匹配信息
std::vector<std::pair<int, int>> computeMatchs(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target,
                                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_src,
                                               const Eigen::Vector2d& T, double search_distance)
{
    std::vector<std::pair<int, int>> match_results = {};

    // std::cout << "qzc debug 1 : " << cloud_src->size() << std::endl;
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix(0,3) = T.x();
    transform_matrix(1,3) = T.y();
    // std::cout << "qzc debug 2.0 : " << transform_matrix << std::endl;

    if (cloud_src->size() == 0) {
        return match_results;
    }
    

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src_transformed(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::transformPointCloud(*cloud_src, *cloud_src_transformed, transform_matrix);
    // std::cout << "qzc debug 2 " << std::endl;

    pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
    kdtree.setInputCloud(cloud_src_transformed);
    // std::cout << "qzc debug 3 : " << cloud_src_transformed->size() << std::endl;


    // 遍历每个points1的点,计算点到直线的距离 + 方向 (如果匹配的距离过大,或者没有的话,直接跳过就行)
    // Eigen::VectorXd residuals(2 * points1.size());
    for (size_t i = 0; i < cloud_target->points.size(); ++i) {
        pcl::PointXYZINormal queryPoint = cloud_target->points[i];

        // 查询最近邻
        std::vector<int>   nearestNeighbors;
        std::vector<float> squaredDistances;
        int                K = 5;  // 查找3个最近邻
        if( K > cloud_src->size()) { // 防止点数较少的时候crash
            K = cloud_src->size();
        }

        if (kdtree.nearestKSearch(queryPoint, K, nearestNeighbors, squaredDistances) > 0) {
            double min_direction_cost = 3;
            int    match_j            = 0;
            for (int j = 0; j < K; j++) {
                // TODO： 距离设置一下, 此处为平方
                if (squaredDistances[j] < search_distance*search_distance) {
                    // std::cout << " size: " << cloud_src_transformed->size() 
                    //             << " j: " << j << " n_j: " << nearestNeighbors[j]
                    //             << std::endl;
                    auto hit_pt = cloud_src_transformed->points[nearestNeighbors[j]];
                    // std::cout << " size: " << cloud_src_transformed->size() 
                    //             << " nx: " << queryPoint.normal_x << " ny: " << queryPoint.normal_y<< " nz: " << queryPoint.normal_z
                    //             << " nx: " << hit_pt.normal_x << " ny: " << hit_pt.normal_y<< " nz: " << hit_pt.normal_z
                    //             << std::endl;


                    double direct_cost = directionCost(queryPoint, hit_pt);
                    if (direct_cost < min_direction_cost) {
                        min_direction_cost = direct_cost;
                        match_j            = nearestNeighbors[j];
                    }
                }
            }
            // 0.06030737921409157 = 1 - cos(20*pi/180)
            if (min_direction_cost < 0.06030737921409157) {
                match_results.push_back({i, match_j});
            }
        }
    }

    return match_results;
}


// 计算匹配信息
std::vector<std::pair<int, int>> computeMatchs2(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target,
                                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_src,
                                               const Eigen::Vector2d& T, double search_distance)
{
    std::vector<std::pair<int, int>> match_results = {};

    // std::cout << "qzc debug 1 : " << cloud_src->size() << std::endl;
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix(0,3) = T.x();
    transform_matrix(1,3) = T.y();
    // std::cout << "qzc debug 2.0 : " << transform_matrix << std::endl;

    if (cloud_target->size() == 0 || cloud_src->size() == 0) {
        return match_results;
    }
    

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src_transformed(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::transformPointCloud(*cloud_src, *cloud_src_transformed, transform_matrix);
    // std::cout << "qzc debug 2 " << std::endl;

    pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
    kdtree.setInputCloud(cloud_target);
    // std::cout << "qzc debug 3 : " << cloud_src_transformed->size() << std::endl;


    // 遍历每个points1的点,计算点到直线的距离 + 方向 (如果匹配的距离过大,或者没有的话,直接跳过就行)
    // Eigen::VectorXd residuals(2 * points1.size());
    for (size_t i = 0; i < cloud_src_transformed->points.size(); ++i) {
        pcl::PointXYZINormal queryPoint = cloud_src_transformed->points[i];

        // 查询最近邻
        std::vector<int>   nearestNeighbors;
        std::vector<float> squaredDistances;
        int                K = 5;  // 查找3个最近邻
        if( K > cloud_src->size()) { // 防止点数较少的时候crash
            K = cloud_src->size();
        }

        if (kdtree.nearestKSearch(queryPoint, K, nearestNeighbors, squaredDistances) > 0) {
            double min_direction_cost = 3;
            int    match_j            = 0;
            int searched_num = nearestNeighbors.size();
            for (int j = 0; j < searched_num; j++) {
                // TODO： 距离设置一下, 此处为平方
                if (squaredDistances[j] < search_distance*search_distance) {
                    // std::cout << " size: " << cloud_src_transformed->size() 
                    //             << " j: " << j << " n_j: " << nearestNeighbors[j]
                    //             << std::endl;
                    auto hit_pt = cloud_target->points[nearestNeighbors[j]];
                    // std::cout << " size: " << cloud_src_transformed->size() 
                    //             << " nx: " << queryPoint.normal_x << " ny: " << queryPoint.normal_y<< " nz: " << queryPoint.normal_z
                    //             << " nx: " << hit_pt.normal_x << " ny: " << hit_pt.normal_y<< " nz: " << hit_pt.normal_z
                    //             << std::endl;


                    double direct_cost = directionCost(queryPoint, hit_pt);
                    if (direct_cost < min_direction_cost) {
                        min_direction_cost = direct_cost;
                        match_j            = nearestNeighbors[j];
                    }
                }
            }
            // 0.06030737921409157 = 1 - cos(20*pi/180)
            if (min_direction_cost < 0.06030737921409157) {
                // match_results.push_back({i, match_j});
                match_results.push_back({match_j, i});
            }
        }
    }

    return match_results;
}



// 计算残差向量
Eigen::VectorXd computeResiduals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target,
                                pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_src,
                                 const Eigen::Vector2d& T,
                                 const std::vector<std::pair<int, int>>& matches)
{
    Eigen::Vector3d T_3d(T.x(), T.y(), 0);
    Eigen::VectorXd residuals(2 * matches.size());
    residuals.setZero();
    for (size_t i = 0; i < matches.size(); ++i) {
        int ind1 = matches[i].first;
        int ind2 = matches[i].second;

        Eigen::Vector3d point_target = Eigen::Vector3d(cloud_target->points[ind1].x, cloud_target->points[ind1].y, 0);
        Eigen::Vector3d point_src = Eigen::Vector3d(cloud_src->points[ind2].x, cloud_src->points[ind2].y, 0);
        
        Eigen::Vector3d point_target_dir = Eigen::Vector3d(cloud_target->points[ind1].normal_x, cloud_target->points[ind1].normal_y, 0);
        Eigen::Vector3d point_src_dir = Eigen::Vector3d(cloud_src->points[ind1].normal_x, cloud_src->points[ind1].normal_y, 0);
        
        double weight = 1.0;
        if(alg::judge_left2(point_target, point_src, point_src_dir) == -1) {
            weight *= 1;
        }

        Eigen::Vector3d diff = point_target - (point_src + T_3d);
        residuals[2 * i]     = weight * diff[0];
        residuals[2 * i + 1] = weight * diff[1];
    }
    return residuals;
}

void computeAvgDir(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target,
                        pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_src,
                        const std::vector<std::pair<int, int>>& matches,
                        Eigen::Vector3d& avg_dir)
{
    avg_dir.setZero();
    for (size_t i = 0; i < matches.size(); ++i) {
        int ind1 = matches[i].first;
        int ind2 = matches[i].second;

        Eigen::Vector3d point_target_dir = Eigen::Vector3d(cloud_target->points[ind1].normal_x, cloud_target->points[ind1].normal_y, 0);
        // Eigen::Vector3d point_src = Eigen::Vector3d(cloud_src->points[ind2].normal_x, cloud_src->points[ind2].normal_y, 0);
        
        avg_dir += point_target_dir;
    }

    if (matches.size() > 0) {
        avg_dir /= matches.size();
    }
}

void computeAvgDistance(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target,
                        pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_src,
                        const std::vector<std::pair<int, int>>& matches,
                        const Eigen::Vector2d& init_T,
                        Eigen::Vector2d& optimize_T,
                        double& avg_distance)
{
    Eigen::Vector3d init_T_3d(init_T.x(), init_T.y(), 0);
    avg_distance = FLT_MAX;

    Eigen::Vector3d sum_diff_3d;
    sum_diff_3d.setZero();
    for (size_t i = 0; i < matches.size(); ++i) {
        if (i == 0) {
            avg_distance = 0.f;
        }
        int ind1 = matches[i].first;
        int ind2 = matches[i].second;

        Eigen::Vector3d point_target = Eigen::Vector3d(cloud_target->points[ind1].x, cloud_target->points[ind1].y, 0);
        Eigen::Vector3d point_src = Eigen::Vector3d(cloud_src->points[ind2].x, cloud_src->points[ind2].y, 0);
        
        // Eigen::Vector3d point_target_dir = Eigen::Vector3d(cloud_target->points[ind1].normal_x, cloud_target->points[ind1].normal_y, 0);
        // Eigen::Vector3d point_src_dir = Eigen::Vector3d(cloud_src->points[ind1].normal_x, cloud_src->points[ind1].normal_y, 0);
        // double weight = 1.0;
        // if(alg::judge_left2(point_target, point_src, point_src_dir) == -1) {
        //     weight *= 1;
        // }

        Eigen::Vector3d diff = point_target - (point_src + init_T_3d);
        avg_distance += diff.norm();
        sum_diff_3d += diff;
    }

    if (matches.size() > 0) {
        avg_distance  /= matches.size();
        Eigen::Vector3d avg_diff_3d = sum_diff_3d / matches.size();
        optimize_T = Eigen::Vector2d(avg_diff_3d.x(), avg_diff_3d.y());
    }
}


void computeResidualsAndJacobian(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target,
                                pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_src,
                                 const Eigen::Vector2d& T,
                                 const std::vector<std::pair<int, int>>& matches, 
                                 int mode, double offset, 
                                 Eigen::VectorXd& residuals,
                                 Eigen::MatrixXd& J)
{
    int match_size = matches.size();
    Eigen::VectorXd residuals1(2 * match_size);
    residuals1.setZero();

    Eigen::MatrixXd J1(2 * match_size, 2);
    J1.setZero();

    if (mode == 2) {
        residuals1.resize(match_size);
        residuals1.setZero();

        J1.resize(match_size, 2);
        J1.setZero();
    } else if (mode == 3) {
        residuals1.resize(match_size);
        residuals1.setZero();

        J1.resize(match_size, 2);
        J1.setZero();
    }
    

    Eigen::Vector3d T_3d(T.x(), T.y(), 0);
    for (size_t i = 0; i < match_size; ++i) {
        int ind1 = matches[i].first;
        int ind2 = matches[i].second;

        Eigen::Vector3d point_target = Eigen::Vector3d(cloud_target->points[ind1].x, cloud_target->points[ind1].y, 0);
        Eigen::Vector3d point_src = Eigen::Vector3d(cloud_src->points[ind2].x, cloud_src->points[ind2].y, 0);
        
        Eigen::Vector3d point_target_dir = Eigen::Vector3d(cloud_target->points[ind1].normal_x, cloud_target->points[ind1].normal_y, 0);
        Eigen::Vector3d point_src_dir = Eigen::Vector3d(cloud_src->points[ind2].normal_x, cloud_src->points[ind2].normal_y, 0);
        
        double weight = 1.0;
        if (mode == 1) {
            // 1： 拟合模式，target是中心线、边界线，src 是初始带平移的道路边界线，src往左边的target车道线或中心线拟合
            if(alg::judge_left2(point_target, point_src+T_3d, point_src_dir) == -1) {
                // weight *= 1/2;
                // weight *= 1.5;
            }
        } else if (mode == 2) {
            // 2： 平移模式，target是道路边界线，src 是初始带平移的道路边界线，src往右边的target边界线拟合
            if(alg::judge_left2(point_target, point_src+T_3d, point_src_dir) == 1) {
                // weight *= 1/2;
                weight *= 1.5;
            }
        } else if (mode == 3) {
            // 1： 拟合模式，target是中心线、边界线，src 是初始带平移的道路边界线，src往左边的target车道线或中心线拟合
            if(alg::judge_left2(point_target, point_src+T_3d, point_src_dir) == -1) {
                // weight *= 1/10;
                // weight *= 10;
            }
        } 
        
        if (mode == 1) {
            Eigen::Vector3d point_src_transformed = point_src + T_3d;
            double project_len = alg::project_point_to_line(point_target, point_target_dir, point_src_transformed, true);
            Eigen::Vector3d normalized_dir = point_target_dir.normalized();
            Eigen::Vector3d project_pt = point_target + normalized_dir * project_len;

            Eigen::Vector3d diff = project_pt - point_src_transformed;
            // Eigen::Vector3d diff = point_target - (point_src + T_3d);
            residuals1[2 * i]     = weight * diff[0] + 1e-6;
            residuals1[2 * i + 1] = weight * diff[1] + 1e-6;

            J1(2 * i, 0)     = -1;
            J1(2 * i + 1, 1) = -1;
        } else if (mode == 2) {
            Eigen::Vector3d diff = point_target - (point_src + T_3d);
            double diff_square = diff.squaredNorm() - offset * offset + 1e-6;
            residuals1[i] = weight * diff_square;

            J1(i, 0) = -2 * diff[0] + 1e-6;
            J1(i, 1) = -2 * diff[1] + 1e-6;
        } else if(mode == 3) {
            Eigen::Vector3d lp = point_src + T_3d;
            Eigen::Vector3d nu = (lp - point_target).cross(lp - point_target - point_target_dir);
            Eigen::Vector3d de = point_target_dir;
            Eigen::Matrix3d skew_de;
            skew_de << 0, -de.z(), de.y(),
                    de.z(), 0, -de.x(),
                    -de.y(), de.x(), 0;

            Eigen::Vector3d jacb = - nu.transpose() / (nu.norm() * de.norm()) * skew_de;
            residuals1[i] = weight * nu.norm() / de.norm();
            J1(i, 0)     = jacb[0];
            J1(i, 1) = jacb[1];

            // residuals1[2*i] = weight * nu.x() / de.norm();
            // residuals1[2*i + 1] = weight * nu.y() / de.norm();

            // Eigen::Matrix3d jacb = - skew_de / de.norm();
            // J1.block<2,2>(2*i,0) = jacb.block<2,2>(0,0);
        }
    }

    J = J1;
    residuals = residuals1;
}


Eigen::Vector2d computeInitOffset(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target,
                                pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_src,
                                 const Eigen::Vector2d& T,
                                 const std::vector<std::pair<int, int>>& matches)
{
    Eigen::Vector2d init_offset;
    init_offset.setZero();
    for (size_t i = 0; i < matches.size(); ++i) {
        int ind1 = matches[i].first;
        int ind2 = matches[i].second;

        Eigen::Vector2d point_target = Eigen::Vector2d(cloud_target->points[ind1].x, cloud_target->points[ind1].y);
        Eigen::Vector2d point_src = Eigen::Vector2d(cloud_src->points[ind2].x, cloud_src->points[ind2].y);

        Eigen::Vector2d diff = point_target - (point_src + T);
        init_offset += diff;
    }
    if (matches.size() > 0) {
        init_offset /= matches.size();
    }
    
    return init_offset;
}

// 计算雅可比矩阵
Eigen::MatrixXd computeJacobian(size_t numPoints)
{
    // TODO：qzc，加权重
    // TODO：qzc，限制范围
    Eigen::MatrixXd J(2 * numPoints, 2);
    J.setZero();
    for (size_t i = 0; i < numPoints; ++i) {
        J(2 * i, 0)     = -1;
        J(2 * i + 1, 1) = -1;
    }
    return J;
}

// 高斯-牛顿法优化平移量
Eigen::Vector2d gaussNewtonOptimization(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target,
                                        pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_src,
                                        const Eigen::Vector2d&              initialT,
                                        int mode,
                                        double offset,
                                        double search_distance,
                                        int                                 maxIter = 100,
                                        double                              tol     = 1e-6)
{
    Eigen::Vector2d T = initialT;
    for (int iter = 0; iter < maxIter; ++iter) {
        // std::vector<std::pair<int, int>> matches = computeMatchs(cloud_target, cloud_src, T, search_distance);
        std::vector<std::pair<int, int>> matches = computeMatchs2(cloud_target, cloud_src, T, search_distance);
        if (matches.size() < 3) {
            continue;
        }

        // Eigen::VectorXd residuals = computeResiduals(cloud_target, cloud_src, T, matches);
        // Eigen::MatrixXd J = computeJacobian(matches.size());

        Eigen::VectorXd residuals;
        Eigen::MatrixXd J;
        computeResidualsAndJacobian(cloud_target, cloud_src, T, matches, mode, offset, residuals, J);

        Eigen::MatrixXd JT  = J.transpose();
        Eigen::MatrixXd JTJ = JT * J;
        Eigen::VectorXd JTr = JT * residuals;

        Eigen::Vector2d deltaT = JTJ.llt().solve(JTr);

        T -= deltaT;

        if (iter == 0) {
            LOG_INFO("[make_right_turn: start] iter:{}, matches size:{}, mode:{}, offset:{}, search_distance:{}, tgt:{}, src:{}, T: {} {}, resudiuals:{}",
                    iter, matches.size(), mode, offset, search_distance, cloud_target->size(), cloud_src->size(), T.x(), T.y(), residuals.norm());
        }
                    
        if (deltaT.norm() < tol) {
            LOG_INFO("[make_right_turn: converge sucess] iter:{}, matches size:{}, mode:{}, offset:{}, search_distance:{}, tgt:{}, src:{}, T: {} {}, resudiuals:{}",
                    iter, matches.size(), mode, offset, search_distance, cloud_target->size(), cloud_src->size(), T.x(), T.y(), residuals.norm());
            break;
        }
    }

    return T;
}


void get_point(std::shared_ptr<BoundaryGroupLine> &line ,int index,int size,bool is_forward,double distance,std::deque<Eigen::Vector3d> &points)
{
    if(distance<=0)
    {
        return ;
    }
    int prev_index=is_forward?index-1:index+1;
    if(!line)
    {
        return ;
    }
    if(index>=size||index<0|| prev_index>=size||prev_index<0)
    {
        return ;
    }
    auto cur_pt=line->list[index];
    auto prev_pt=line->list[prev_index];
    if(!cur_pt|| !prev_pt)
    {
        return ;
    }
    if(is_forward)
    {
        points.push_back(cur_pt->pos);
    }
    else
    {
        points.push_front(cur_pt->pos);
    }
    int next_index=is_forward?index+1:index-1;
    double next_distance=distance-alg::calc_dis(cur_pt->pos,prev_pt->pos);
    get_point(line,next_index,size,is_forward,next_distance,points);
}

int RoadModelProcMergeFeature::make_right_turn(RoadModelSessionData *session)
{
    auto lit = session->key_pose_map.begin();
    if (lit != session->key_pose_map.end()) {
        auto trail_ptr = &lit->second;

        RTreeProxy<BoundaryFeature *, float, 2> road_boundary_tree;
        for (auto line : trail_ptr->boundary_line_group_list)
        {
            for (auto rb : line->list)
            {
                road_boundary_tree.insert(rb->pos, rb.get());
            }
        }
        RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
        for (auto line : trail_ptr->lane_center_line_group_list)
        {
            for (auto pt : line->list)
            {
                lane_center_tree.insert(pt->pos, pt.get());
            }
        }
        RTreeProxy<LaneLineSample *, float, 2> laneline_tree;
        for (auto line : trail_ptr->lane_line_group_list)
        {
            for (auto pt : line->list)
            {
                laneline_tree.insert(pt->pos, pt.get());
            }
        }

        // Eigen::Vector3d turnright_link_center;
        // double link_nums = 0;
        // for (auto &link : session->link_sample_list)
        // {   
        //     if(link->list.empty())
        //     {
        //         continue;
        //     }
        //     auto front=link->list.front();
        //     auto from_raw_link = front->from_raw_link;
        //     if (!from_raw_link)
        //     {
        //         continue;
        //     }

        //     if (alg::match_any_except_forms(from_raw_link->forms, {25, 26}))
        //     {
        //         continue;
        //     }

        //     int idx = link->list.size() / 2;
        //     turnright_link_center += link->list[idx]->pos;
        //     LOG_INFO("--------({}, {}), center:({},{})-------", link->list[idx]->pos.x(), link->list[idx]->pos.y(), turnright_link_center.x(), turnright_link_center.y())
        //     link_nums += 1.0;
        // }
        // if(link_nums>0){
        //     turnright_link_center /= link_nums;
        // }


        auto generate_longer_link = [&session](std::vector<KeyPose*> &link_pts, double distance, int &pos)
        {
            std::vector<Eigen::Vector3d> points;
            for(auto pt: link_pts)
            {
                points.push_back({pt->pos.x(), pt->pos.y(), 0});
            }
            
            PolyFit::Problem problem;
            problem.degree=3;
            problem.points=points;
            PolyFit::Options option;
            option.use_calc_direction=true;
            option.use_calc_center=true;
            PolyFit fit = PolyFit(problem, option);
            fit.solver();

            std::vector<KeyPose*> new_link;
            // 加长首端
            std::vector<Eigen::Vector3d> fit_points;
            auto ref_pt = points[0];
            auto ref_dir = alg::get_dir(points[1], ref_pt);
            auto fit_start_pt = ref_pt - ref_dir*distance;

            auto diff = alg::calc_dis(fit_start_pt, ref_pt);
            Eigen::Vector3d last_fit_pnt=fit_start_pt;
            // while (diff > 2.1)
            int count= 5;
            while(count--)
            { // 3.5 * 0.6
                auto predict_fit_pnt = last_fit_pnt + ref_dir*2;
                auto added_fit_pnt = fit.eval(predict_fit_pnt);
                Eigen::Vector3d out_fit_pnt(added_fit_pnt.x(), added_fit_pnt.y(), link_pts.front()->pos.z());
                fit_points.push_back(out_fit_pnt);
                auto tar_pt = session->add_ptr(session->key_pose_ptr);
                tar_pt->pos = out_fit_pnt;
                tar_pt->dir = alg::get_dir(tar_pt->pos, last_fit_pnt);
                // tar_pt->from_raw_link = link_pts.front()->from_raw_link;
                new_link.push_back(tar_pt.get());
                // LOG_INFO("New Added front-point: ({}, {}, {})", out_fit_pnt.x(), out_fit_pnt.y(), out_fit_pnt.z());

                last_fit_pnt = out_fit_pnt;
                diff = alg::calc_dis(ref_pt, last_fit_pnt);
            }

            // 添加已有link
            double max_curvature = 0;
            for(auto pt: link_pts)
            {   
                auto added_fit_pnt = fit.eval({pt->pos.x(), pt->pos.y(), pt->pos.z()});
                fit_points.push_back(added_fit_pnt);

                auto tar_pt = session->add_ptr(session->key_pose_ptr);
                tar_pt->pos = added_fit_pnt;
                tar_pt->dir = alg::get_dir(added_fit_pnt, last_fit_pnt);;
                // tar_pt->from_raw_link = pt->from_raw_link;
                new_link.push_back(tar_pt.get());
                
                double cur_ = fit.curvature(added_fit_pnt);
                if(cur_ >= max_curvature){
                    max_curvature = cur_;
                    pos = new_link.size();
                } 

                last_fit_pnt = added_fit_pnt;
            }

            // 加长末端
            auto size=fit_points.size();
            auto ref_pt2 = fit_points[size-1];
            auto ref_dir2 = alg::get_dir(ref_pt2, fit_points[size-2]);
            auto fit_end_pt = ref_pt2 + ref_dir2*distance;

            auto diff2 = alg::calc_dis(ref_pt2, fit_end_pt);
            Eigen::Vector3d last_fit_pnt2=ref_pt2;
            // while (diff2 > 2.1)
            count= 5;
            while(count--)
            { // 3.5 * 0.6
                auto predict_fit_pnt = last_fit_pnt2 + ref_dir2*2;
                auto added_fit_pnt = fit.eval(predict_fit_pnt);
                Eigen::Vector3d out_fit_pnt(added_fit_pnt.x(), added_fit_pnt.y(), link_pts.back()->pos.z());
                fit_points.push_back(out_fit_pnt);

                auto tar_pt = session->add_ptr(session->key_pose_ptr);
                tar_pt->pos = out_fit_pnt;
                tar_pt->dir = alg::get_dir(tar_pt->pos, last_fit_pnt2);
                // tar_pt->from_raw_link = link_pts.back()->from_raw_link;
                new_link.push_back(tar_pt.get());
                // LOG_INFO("New Added end-point: ({}, {}, {})", out_fit_pnt.x(), out_fit_pnt.y(), out_fit_pnt.z());

                last_fit_pnt2 = out_fit_pnt;
                // diff2 = alg::calc_dis(fit_end_pt, last_fit_pnt2);
            }

            link_pts.clear();
            link_pts.assign(new_link.begin(), new_link.end());
        };

        auto get_right_boundary_rb = [&road_boundary_tree](KeyPose *poss, BoundaryFeature *prev_rb) -> BoundaryFeature *
        {
            auto from_raw_link = poss->from_raw_link;
            if (!from_raw_link)
            {
                return NULL;
            }
            else
            {
                // int from = std::stoi(from_raw_link->form);
                // LOG_INFO("from_raw_link:{} {}",from ,from_raw_link->form)
                // if(from!=25&&from!=26)
                if (alg::match_any_except_forms(from_raw_link->forms, {25, 26}))
                {
                    return NULL;
                }
            }

            std::vector<BoundaryFeature *> bfs;
            road_boundary_tree.search(poss->pos, 10, bfs);
            Eigen::Vector3d p1 = poss->pos;
            Eigen::Vector3d p2 = alg::get_vertical_pos(poss->pos, poss->dir, 10);
            BoundaryFeature *min_dis_bf = NULL;
            double min_dis = 100;
            // LOG_INFO("bfs:{}",bfs.size());
            for (auto bf : bfs)
            {
                if (!bf->next)
                {
                    continue;
                }
                Eigen::Vector3d cross;
                if (alg::findIntersection(p1, p2, bf->pos, bf->next->pos, cross))
                {
                    // TODO CHECK
                    if (alg::judge_left(cross, poss->pos, poss->dir) < 0)
                    {
                        continue;
                    }
                    // 检查是否有同向多条边界线的情况
                    if(prev_rb){
                        Eigen::Vector3d cur_dir=bf->next->pos-bf->pos;
                        Eigen::Vector3d prev_dir=prev_rb->next->pos-prev_rb->pos;
                        auto prev_theta = alg::calc_theta(cur_dir, prev_dir);
                        prev_theta = std::min(prev_theta, 180-prev_theta);
                        auto prev_vertical_dis = alg::calc_vertical_dis(prev_rb->pos, bf->pos, prev_dir);
                        if(prev_theta<20 && prev_vertical_dis>3.5*0.6){
                            continue;
                        }
                    }

                    double dis = alg::calc_dis(cross, poss->pos);
                    if (dis < min_dis)
                    {
                        min_dis = dis;
                        min_dis_bf = bf;
                    }
                }
            }
            return min_dis_bf;
        };

        auto check_fit_points = [](std::vector<std::shared_ptr<CrossPoint<BoundaryGroupLine>>>& new_line) -> bool {
            int GAP_NUM = 1;
            int feature_size = new_line.size();
            for(int i = GAP_NUM; i < feature_size - GAP_NUM; ++i) {
                auto& p0 = new_line[i-1];
                auto& p1 = new_line[i];
                auto& p2 = new_line[i+1];

                //计算曲率
                Eigen::Vector3d v1 = p0->pos - p1->pos;
                Eigen::Vector3d v2 = p2->pos - p1->pos;
                Eigen::Vector3d v3 = p0->pos - p2->pos;
                double numerator = 4 * v1.cross(v2).norm();
                double denominator = (v1.norm() + v2.norm() + v3.norm());

                if(denominator < 1e-6) {
                    // 避免除以零的情况
                    p1->curvature = 0.0;
                } else {
                    p1->curvature = (numerator / denominator) * 100;
                }
            }

            float ABNORMAL_CURVATURE = 25;
            int abnormal_curvature_cnt = 0;
            for(int i = GAP_NUM; i < feature_size - GAP_NUM; ++i) {
                auto& p1 = new_line[i];
                if (p1->curvature > ABNORMAL_CURVATURE) {
                    abnormal_curvature_cnt ++;
                }
            }

            return abnormal_curvature_cnt < 3;
        };

        auto set_calc_curvature = [](std::shared_ptr<BoundaryGroupLine> line, int index, const std::deque<Eigen::Vector3d> &points)
        {
            if (!line)
            {
                return;
            }
            auto pt = line->list[index];
            std::vector<Eigen::Vector3d> temp;
            temp.insert(temp.end(), points.begin(), points.end());
            // 计算曲率
            PolyFit::Problem problem;
            problem.degree=3;
            problem.points=temp;
            problem.center=pt->pos;
            problem.dir=pt->dir;
            PolyFit::Options option;
            option.use_calc_direction=true;
            option.use_calc_center=true;
            PolyFit fit=PolyFit(problem,option); 
            fit.solver();
            double curvature= fit.curvature(pt->pos);
            pt->curvature=curvature;
        };

        auto find_crosspoint=[&](
                std::shared_ptr<BoundaryGroupLine>& group_line, std::vector<KeyPose*> &link,
                int startindex, int endindex, bool forward, int &cp_index)
        {
            const int size = group_line->list.size();
            std::vector<Eigen::Vector3d> dirs;
            std::vector<LaneCenterFeature *> kpfs;

            auto pt1 = !forward? link.front(): link.back();
            lane_center_tree.search(pt1->pos, 5.0, kpfs);
            if (!kpfs.empty()) {
                for (const auto kp : kpfs) {
                    dirs.push_back(kp->dir);
                }
            }
            auto lc_dir = alg::get_direction(dirs).normalized();
            LOG_INFO("[find_crosspoint] -- forward: {}, pos: ({}, {}), dir: ({}, {})", forward, pt1->pos.x(), pt1->pos.y(), lc_dir.x(), lc_dir.y());

            const double theta_thres=FLAGS_merge_feature_boundary_theta_thres;
            std::vector<double> theta_list;
            std::deque<int> indexs;
            for (int i = startindex; i < endindex; i++) {
                int j = forward? i:endindex-1-i;
                /////////////////////////////////////////
                auto pt1 = group_line->list[j];
                auto pt_dir = pt1->dir.normalized();
                double theta = alg::calc_theta(lc_dir, pt_dir);
                // theta = theta>90? 180-theta: theta;
                theta_list.push_back(theta);
                // LOG_INFO("{} th pt -- ({}, {}), ({}, {}), angle_theta: {}", j, pt_dir.x(), pt_dir.y(), pt1->pos.x(), pt1->pos.y(), theta);
                if (theta < theta_thres) {
                    indexs.push_back(i-startindex);
                }
            }
            while (indexs.size() > 0) {
                auto start = indexs.front();
                indexs.pop_front();
                if (start >= theta_list.size()) {
                    break;
                }

                int count=0, bins=10;
                auto end = start+bins;
                size_t validCount = (end <= theta_list.size()) ? bins : theta_list.size() - start;

                for (size_t i = 0; i < validCount; ++i) {
                    if (theta_list[start + i] < theta_thres) {
                        count++;
                    }
                }
                if (count > validCount*0.8){
                    cp_index = forward? start+startindex: endindex-1-(start+startindex);
                    // pt = group_line->list[start+startindex];
                    LOG_INFO("---------[ Find boundary crosspoint ] find cut point: pos-({}, {}), index-{} ... ---------", 
                        group_line->list[cp_index]->pos.x(), group_line->list[cp_index]->pos.y(), cp_index);
                    return true;
                }
            }

            LOG_INFO("---------[ Find boundary crosspoint ] find cut point: fail ... ---------");
            return false;
        };

        auto connect_lane_center = [&lane_center_tree](const fast_road_model::LBPointPtr lb, double &dis) -> LaneCenterFeature *
        {
            std::vector<LaneCenterFeature *> lcfs;
            Eigen::Vector3d p1 = lb->pos;
            Eigen::Vector3d p2 = alg::get_vertical_pos(lb->pos, lb->dir, -3.75*2);
            lane_center_tree.search(lb->pos, 3.75 * 1.414*2, lcfs);
            LaneCenterFeature *ret = NULL;
            double min_dis = 3.75;    // 增加lb-lc的宽度限制，防止找到错误点位
            for (auto lc : lcfs)
            {
                auto neighbors = {lc->next, lc->prev};
                for (auto neighbor : neighbors)
                {
                    if (!neighbor)
                    {
                        continue;
                    }

                    double theta = alg::calc_theta(lb->dir, lc->dir);
                    bool cond2 = theta < 15;
                    Eigen::Vector3d cross;
                    bool cond1 = alg::findIntersection(p1, p2, lc->pos, neighbor->pos, cross);
                    if (cond1 && cond2)
                    {   
                        //  TODO add right check
                        double dis = alg::calc_dis(cross, lb->pos);
                        if (dis < min_dis)
                        {
                            min_dis = dis;
                            ret = lc;
                        }
                    }
                }
            }
            // 跳变
            if(dis>0&&std::abs(dis-min_dis)>3.5/2)
            {
                dis=-1;
                return NULL;
            }

            dis = min_dis;
            return ret;
        };

        auto connect_laneline = [&laneline_tree](const fast_road_model::LBPointPtr lb, bool is_right, double &dis) -> LaneLineSample *
        {
            std::vector<LaneLineSample *> lcfs;
            Eigen::Vector3d p1 = lb->pos;
            Eigen::Vector3d p2 = alg::get_vertical_pos(lb->pos, lb->dir, -3.75*2);
            laneline_tree.search(lb->pos, 3.75 * 1.414*2, lcfs);
            LaneLineSample *ret = NULL;
            double min_dis = is_right?1.0: 3.75*1.5;    // 增加lb-lc的宽度限制，防止找到错误点位
            for (auto lc : lcfs)
            {
                auto neighbors = {lc->next, lc->prev};
                for (auto neighbor : neighbors)
                {
                    if (!neighbor)
                    {
                        continue;
                    }

                    double theta = alg::calc_theta(lb->dir, lc->dir);
                    bool cond2 = theta < 15;
                    Eigen::Vector3d cross;
                    bool cond1 = alg::findIntersection(p1, p2, lc->pos, neighbor->pos, cross);
                    if (cond1 && cond2)
                    {   
                        //  TODO add right check
                        double dis = alg::calc_dis(cross, lb->pos);
                        bool cond = is_right? true: dis > 3.75*0.5;
                        if (dis < min_dis && cond)
                        {
                            min_dis = dis;
                            ret = lc;
                            // if(!is_right){
                            //     LOG_INFO("------------------  right-{}, dis-{} , min_dis-{} ----------------", is_right, dis, min_dis);
                            // }
                        }
                    }
                }
            }
            // 跳变
            if(dis>0&&std::abs(dis-min_dis)>3.5/2)
            {
                dis=-1;
                return NULL;
            }

            dis = min_dis;
            return ret;
        };

        auto convert_boundary_line_to_pcd = [](std::shared_ptr<BoundaryGroupLine>& group_line, Eigen::Vector3d anchor_pos) -> pcl::PointCloud<pcl::PointXYZINormal>::Ptr {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZINormal>);
            for (int j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                if (pt->dir.allFinite() && (pt->pos-anchor_pos).norm() < 50) {
                    
                    cloud_in->points.push_back(pcl::PointXYZINormal(pt->pos.x(), pt->pos.y(), pt->pos.z(), pt->curvature * 100,
                                                                    pt->dir.x(), pt->dir.y(), pt->dir.z()));
                }
            }

            cloud_in->width = cloud_in->size();
            cloud_in->height = 1;
            cloud_in->is_dense = false;

            return cloud_in;
        };

        auto convert_lane_lines_to_pcd = [](std::vector<LaneLineSampleGroupLine*>& group_lines, Eigen::Vector3d anchor_pos) -> pcl::PointCloud<pcl::PointXYZINormal>::Ptr {

            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZINormal>);
            for (int i = 0; i < group_lines.size(); i++) {
                LaneLineSampleGroupLine* group_line = group_lines[i];
                if (group_line == nullptr) {
                    continue;
                }
                
                for (int j = 0; j < group_line->list.size(); ++j) {
                    auto &pt = group_line->list[j];
                    if(pt == nullptr) {
                        continue;
                    }

                    if (pt->dir.allFinite() && (pt->pos-anchor_pos).norm() < 50) {
                        cloud_in->points.push_back(pcl::PointXYZINormal(pt->pos.x(), pt->pos.y(), pt->pos.z(), pt->curvature * 100,
                                                                        pt->dir.x(), pt->dir.y(), pt->dir.z()));
                    }
                }
            }

            cloud_in->width = cloud_in->size();
            cloud_in->height = 1;
            cloud_in->is_dense = false;

            return cloud_in;
        };

        auto convert_lane_centers_to_pcd = [](std::vector<LaneCenterGroupLine*>& group_lines, Eigen::Vector3d anchor_pos) -> pcl::PointCloud<pcl::PointXYZINormal>::Ptr {

            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZINormal>);
            for (int i = 0; i < group_lines.size(); i++) {
                LaneCenterGroupLine* group_line = group_lines[i];
                if (group_line == nullptr) {
                    continue;
                }
                
                for (int j = 0; j < group_line->list.size(); ++j) {
                    auto &pt = group_line->list[j];
                    if(pt == nullptr) {
                        continue;
                    }

                    if(pt->dir.allFinite() && (pt->pos-anchor_pos).norm() < 50){
                        cloud_in->points.push_back(pcl::PointXYZINormal(pt->pos.x(), pt->pos.y(), pt->pos.z(), pt->curvature * 100,
                                                                        pt->dir.x(), pt->dir.y(), pt->dir.z()));
                    }

                }
            }

            cloud_in->width = cloud_in->size();
            cloud_in->height = 1;
            cloud_in->is_dense = false;
            return cloud_in;
        };

        // 重新添加现有的车道线
        session->lane_line_sample_line_ptr_for_merge.clear();
        session->bev_frame_lane_line.clear();
        // int lane_line_size = trail_ptr->lane_line_group_list.size();
        // for (int64_t i = 0; i < lane_line_size; ++i) {
        //     auto &group_line = trail_ptr->lane_line_group_list[i];
        //     auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);
        //     new_line->cur_line_id = group_line->cur_line_id;
        //     new_line->matched_line_ids = group_line->matched_line_ids;

        //     LaneLineSample* prev = NULL;
        //     for (int64_t j = 0; j < group_line->list.size(); ++j) {
        //         int valid_index = j;
        //         auto &pt = group_line->list[j];
        //         if(pt == nullptr) {
        //             continue;
        //         }

        //         if(pt->invalid()) {
        //             continue;
        //         }

        //         if (prev != NULL) {
        //             pt->set_prev(prev);
        //         }
                
        //         auto new_node = session->add_ptr(session->lane_line_sample_ptr);
        //         new_node->init(pt.get());
        //         new_line->list.push_back(new_node.get());

        //         prev = pt.get();
        //     }

        //     if (new_line->list.size() == 0) {
        //         continue;
        //     }

        //     session->bev_frame_lane_line[""].push_back(new_line.get());
        // }

        // 重新添加现有的中心线
        session->lane_center_line_ptr_for_merge.clear();
        session->bev_frame_lane_center_line.clear();
        // int lane_center_size = trail_ptr->lane_center_line_group_list.size();
        // for (int64_t i = 0; i < lane_center_size; ++i) {
        //     auto &group_line = trail_ptr->lane_center_line_group_list[i];

        //     auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);
        //     new_line->boundary_type = group_line->boundary_type;
        //     std::shared_ptr<LaneCenterFeature> prev = nullptr;
        //     for (int64_t j = 0; j < group_line->list.size(); ++j) {
        //         auto &pt = group_line->list[j];
        //         if(pt == nullptr) {
        //             continue;
        //         }
        //         if(pt->invalid()) {
        //             continue;
        //         }

        //         if (prev != NULL) {
        //             pt->set_prev(prev.get());
        //         }

        //         auto new_node = session->add_ptr(session->lane_center_feature_ptr);
        //         new_node->init(pt.get());

        //         new_line->list.push_back(new_node.get());

        //         prev = pt;
        //     }

        //     if (new_line->list.size() == 0) {
        //         continue;
        //     }

        //     session->bev_frame_lane_center_line[""].push_back(new_line.get());
        // }

        // 重新添加现有的边界线
        session->boundary_line_ptr_for_merge.clear();
        session->bev_frame_boundary_line.clear();
        // int boundary_line_size = trail_ptr->boundary_line_group_list.size();
        // for (int64_t i = 0; i < boundary_line_size; ++i) {
        //     auto &group_line = trail_ptr->boundary_line_group_list[i];

        //     auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
        //     new_line->boundary_type = group_line->boundary_type;
        //     std::shared_ptr<BoundaryFeature> prev = nullptr;
        //     for (int64_t j = 0; j < group_line->list.size(); ++j) {
        //         auto &pt = group_line->list[j];
        //         if(pt == nullptr) {
        //             continue;
        //         }
        //         if(pt->invalid()) {
        //             continue;
        //         }

        //         if (prev != NULL) {
        //             pt->set_prev(prev.get());
        //         }

        //         auto new_node = session->add_ptr(session->boundary_feature_ptr);
        //         new_node->init(pt.get());

        //         new_line->list.push_back(new_node.get());

        //         prev = pt;
        //     }

        //     if (new_line->list.size() == 0) {
        //         continue;
        //     }

        //     session->bev_frame_boundary_line[""].push_back(new_line.get());
        // }

        session->set_display_name("turn_right_candicate");
        std::vector<std::shared_ptr<BoundaryGroupLine>> tmp_boundary_line_ptr;
        for (auto &link : session->link_sample_list)
        {   
            // Step 01. 构建右转: 边界线
            // 1. 判断link信息, 并进行密集采样、加长首尾两端
            if(link->list.empty())
            {
                continue;
            }
            auto front=link->list.front();
            auto from_raw_link = front->from_raw_link;
            if (!from_raw_link)
            {
                continue;
            }

            if (alg::match_any_except_forms(from_raw_link->forms, {25, 26}))
            {
                continue;
            }
            // LOG_INFO("-------- make_right_turn  start... ---------");
            LOG_INFO("[  make_right_turn -- 0.start...  ]");
            
            std::vector<KeyPose*> new_single_line_tmp;
            session->sample_line(0.5, link->list, session->key_pose_ptr, new_single_line_tmp);
            
            std::vector<KeyPose*> new_single_line;
            // new_single_line.assign(new_single_line_tmp.begin(), new_single_line_tmp.end());
            // LOG_INFO("--------- process_link: generate_longer_link ({})...", new_single_line.size());
            int max_curvature_pos;
            generate_longer_link(new_single_line_tmp, 10.0, max_curvature_pos);

            Eigen::Vector3d start_pt = new_single_line_tmp.front()->pos;
            Eigen::Vector3d end_pt = new_single_line_tmp.back()->pos;
            Eigen::Vector3d anchor_pt = new_single_line_tmp[max_curvature_pos]->pos;
            if(alg::calc_dis(start_pt, anchor_pt, true)>60 || alg::calc_dis(end_pt, anchor_pt, true)>60){
                LOG_INFO("[  make_right_turn -- 0.abnormal link: too long [{}, {}]...  ]",
                    alg::calc_dis(start_pt, anchor_pt, true), alg::calc_dis(end_pt, anchor_pt, true)
                );
                continue;
            }


            session->sample_line(0.5, new_single_line_tmp, session->key_pose_ptr, new_single_line);
            // LOG_INFO("--------- process_link: success ({})...", new_single_line.size());
            LOG_INFO("[  make_right_turn -- 1.generate longer link ({}): success...  ]", new_single_line.size());
        
            for(auto &p:new_single_line){
                p->from_raw_link=front->from_raw_link;
                p->from_link=front->from_link;
            }

            if(1){
                // session->set_display_name("turn_right_candicate");
                auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "completed_link");
                log_2->color= {0,0,255};
                int idx = 0;
                for(auto pt : new_single_line){
                    // LOG_INFO("Print point: ({}, {}, {})", pt->pos.x(), pt->pos.y(), pt->pos.z());
                    auto &ele=log_2->add(pt->pos);
                    ele.color={255,0,0};
                    if(idx==max_curvature_pos){
                        ele.color={0,0,255};
                    }else if(idx == new_single_line.size()/2){
                        ele.color={0,255,0};
                    }
                    ele.label.score=pt->curvature;
                    ele.label.intensity_opt=idx++;
                }
                // session->save_debug_info("turn_right_candicate");
            }

            // 2. 通过link初步找到边界线候选点
            BoundaryFeature *ref_boundary_feature=NULL;
            LaneCenterFeature *ref_lc_feature=NULL;
            LaneLineSample *ref_rlane_feature=NULL, *ref_llane_feature=NULL;

            std::vector<Eigen::Vector3d> points;
            std::vector<Eigen::Vector3d> points_without_z;
            BoundaryFeature *prev_rb=NULL;
            int idx = 0, left_hits = 0, right_hits = 0, ref_pos = new_single_line.size()/2>max_curvature_pos? new_single_line.size()/2:max_curvature_pos;
            for (auto poss : new_single_line)
            {
                BoundaryFeature *rb = get_right_boundary_rb(poss, prev_rb);
                if (rb)
                {
                    Eigen::Vector3d dir={0,0,0};
                    if(prev_rb)
                    {
                        // 避免太近方向算出来误差太大
                        if(alg::calc_dis(prev_rb->pos,rb->pos)<0.25)
                        {
                            continue;
                        }
                        // 避免弯道交叉
                        if(!alg::judge_front(rb->pos,prev_rb->pos,prev_rb->dir))
                        {
                            continue;
                        }
                        dir=alg::get_dir(rb->pos,prev_rb->pos);
                    }
                    prev_rb=rb;
                    auto &trc=turn_right_candidate[link];
                    TurnRightPoint  new_turn_right_point;
                    new_turn_right_point.lb=std::make_shared<fast_road_model::LBPoint>();
                    // new_turn_right_point.from_road_boundary = rb;
                    new_turn_right_point.lb->init(rb->pos, dir);
                    if(!trc.empty())
                    {
                        auto &back=trc.back();
                        back.lb->set_dir(dir);
                    }
                    trc.push_back(new_turn_right_point);
                    
                    if(idx < ref_pos){
                        left_hits += 1;
                    } else {
                        right_hits += 1;
                    }
                    
                    // 初始化参考boundary_feature
                    if(!ref_boundary_feature){
                        ref_boundary_feature = rb;
                    }

                    // 用于拟合边界线
                    auto pnt = rb->pos;
                    points.push_back(pnt);
                    points_without_z.push_back({pnt.x(), pnt.y(), 0});
                }
                idx += 1;

            }

            if((left_hits >= 10 && right_hits >= 10)){
                // LOG_INFO("--------- find_boundary_candidate: success...");
                LOG_INFO("[  make_right_turn -- 2.find_boundary_candidate [size=({}-{})]: success...  ]", left_hits, right_hits);
            } else {
                // LOG_INFO("--------- !!! find_boundary_candidate: fail [points on both sides is insufficient  -- left:{}, right:{}] ...", left_hits, right_hits);
                link->list.clear();
                LOG_INFO("[  make_right_turn -- 2.find_boundary_candidate: fail [so little points: size=({}-{})]", left_hits, right_hits);
                continue;
            }

            // 3. 拟合边界线曲线
            PolyFit::Problem problem;
            problem.degree=3;
            problem.points=points_without_z;
            PolyFit::Options option;
            option.use_calc_direction=true;
            option.use_calc_center=true;
            PolyFit fit=PolyFit(problem,option); 
            fit.solver();

            // 4. 边界线生成补点(中间区域)
            auto new_boundary_line = session->add_ptr(tmp_boundary_line_ptr);
            std::vector<Eigen::Vector3d> fit_points; 
            Eigen::Vector3d last_fit_pnt;
            for (int j = 0; j < points_without_z.size(); j++)
            {
                auto fit_pnt=fit.eval(points_without_z[j]);                    
                if (j == 0) {
                    Eigen::Vector3d out_fit_pnt(fit_pnt.x(), fit_pnt.y(), points[j].z());
                    fit_points.push_back(out_fit_pnt);
                    last_fit_pnt = fit_pnt;

                    std::shared_ptr<BoundaryFeature> rb = std::make_shared<BoundaryFeature>();
                    rb->pos = out_fit_pnt;
                    new_boundary_line->list.push_back(rb);
                    double curvature= fit.curvature(out_fit_pnt);
                    new_boundary_line->list.back()->curvature=curvature;
                } else {
                    auto diff = alg::calc_dis(fit_pnt, last_fit_pnt);
                    auto diff_bak = diff;
                    Eigen::Vector3d dir = alg::get_dir(fit_pnt, last_fit_pnt, false);
                    while (diff > 0.6) { // 3.5 * 0.6
                        auto predict_fit_pnt = last_fit_pnt + dir*0.5;
                        auto added_fit_pnt = fit.eval(predict_fit_pnt);

                        // 插值 z
                        double factor= diff_bak < 1e-6 ? 0 : diff / diff_bak;
                        double interp_z = factor*points[j-1].z()+(1.0-factor)*points[j].z();
                        Eigen::Vector3d out_fit_pnt(added_fit_pnt.x(), added_fit_pnt.y(), interp_z);
                        fit_points.push_back(out_fit_pnt);

                        std::shared_ptr<BoundaryFeature> rb = std::make_shared<BoundaryFeature>();
                        rb->pos = out_fit_pnt;
                        new_boundary_line->list.push_back(rb);
                        double curvature= fit.curvature(out_fit_pnt);
                        new_boundary_line->list.back()->curvature=curvature;
                        auto &back=new_boundary_line->list.back();
                        back->dir = alg::get_dir(out_fit_pnt, last_fit_pnt);
                    
                        last_fit_pnt = added_fit_pnt;
                        diff = alg::calc_dis(fit_pnt, last_fit_pnt);
                    }

                    // 插值 z
                    double factor= diff_bak < 1e-6 ? 0 : diff / diff_bak;
                    double interp_z = factor*points[j-1].z()+(1.0-factor)*points[j].z();
                    Eigen::Vector3d out_fit_pnt(fit_pnt.x(), fit_pnt.y(), interp_z);
                    fit_points.push_back(out_fit_pnt);

                    std::shared_ptr<BoundaryFeature> rb = std::make_shared<BoundaryFeature>();
                    rb->pos = out_fit_pnt;
                    new_boundary_line->list.push_back(rb);
                    double curvature= fit.curvature(out_fit_pnt);
                    new_boundary_line->list.back()->curvature=curvature;
                    auto &back=new_boundary_line->list.back();
                    back->dir = alg::get_dir(out_fit_pnt, last_fit_pnt);

                    last_fit_pnt = fit_pnt;
                }
            }
            if(new_boundary_line->list.size()>0){
                new_boundary_line->list.pop_back();
            }
            // LOG_INFO("---------- complete_boundary_candidate (medium region): success...");
            LOG_INFO("[  make_right_turn -- 3.complete_boundary_candidate [size={}]: success...  ]", new_boundary_line->list.size());

            // 5. 识别首尾端点 (若两端缺则补线, 加长link后一般不会出现两端缺线的情况)
            int size = new_boundary_line->list.size();
            double max_curvature = -1;
            int max_index = 0;

            for (int j = 0; j < size; j++) {
                std::deque<Eigen::Vector3d> points;
                if (new_boundary_line->list[j]->curvature > max_curvature) {
                    max_curvature = new_boundary_line->list[j]->curvature;
                    max_index = j;
                }
            }

            if(1){
                // session->set_display_name("turn_right_candicate");
                // auto log_1 = session->add_debug_log(utils::DisplayInfo::POINT, "completed_lb");
                // log_1->color= {0,255,0};
                // int idx = 0;
                // for(auto pt : new_boundary_line->list){
                //     auto &ele=log_1->add(pt->pos);
                //     ele.color={0,128,128};
                //     ele.label.score=pt->curvature;
                //     ele.label.intensity_opt=idx++;
                //     if(max_curvature==pt->curvature){
                //         ele.color={0,0,255};
                //     }
                // }
                
                auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "found_lb");
                log_2->color= {0,255,0};
                auto trcs = turn_right_candidate[link];
                for (int i = 0; i < trcs.size(); i++)
                {
                    if(trcs[i].lb)
                    {
                        auto &ele1=log_2->add(trcs[i].lb->pos);
                        ele1.color={0,0,255};
                        ele1.label.label = 2;
                    }
                }
                // session->save_debug_info("turn_right_candicate");
            }

            int cp_idx1, cp_idx2;
            std::shared_ptr<fsdmap::BoundaryFeature> pt1, pt2;
            max_index = new_boundary_line->list.size() / 2;  //直接用中位点
            /* ---------------------------------- 处理边界线首端 ---------------------------------- */
            if (!find_crosspoint(new_boundary_line, new_single_line, 0, max_index, false, cp_idx1)) {
                //补线
                auto link_pt1 = link->list.front();
                std::vector<LaneCenterFeature *> kpfs;
                std::vector<Eigen::Vector3d> dirs;
                lane_center_tree.search(link_pt1->pos, 10, kpfs);
                if (!kpfs.empty()) {
                    for (const auto kp : kpfs) {
                        dirs.push_back(kp->dir);
                    }
                }
                auto lc_dir = alg::get_direction(dirs).normalized();

                auto endpoint = new_boundary_line->list[0];
                bool to_end=true;  // 相当于注释掉端点延长; 延长link后理论上不需要再进行这步, 可能导致死循环, 视为异常情况先跳过吧
                while(!to_end){
                    Eigen::Vector3d dir = -endpoint->dir;
                    Eigen::Vector3d predict_fit_pnt = endpoint->pos + dir*0.5;
                    predict_fit_pnt = fit.eval(predict_fit_pnt);
                    // 插值 z
                    double interp_z = endpoint->pos.z();
                    std::vector<LaneCenterFeature *> kpfs;
                    lane_center_tree.search(predict_fit_pnt, 5, kpfs);
                    if (!kpfs.empty()) {
                        for (const auto kp : kpfs) {
                            interp_z += kp->pos.z();
                        }
                        interp_z /= (kpfs.size()+1);
                    }
                    Eigen::Vector3d out_fit_pnt(predict_fit_pnt.x(), predict_fit_pnt.y(), interp_z);
                    auto &back=new_boundary_line->list.front();
                    std::shared_ptr<BoundaryFeature> rb = std::make_shared<BoundaryFeature>();
                    rb->pos = out_fit_pnt;
                    new_boundary_line->list.insert(new_boundary_line->list.begin(), rb);
                    back->dir = endpoint->pos-out_fit_pnt;
                    endpoint = rb;

                    double theta = alg::calc_theta(lc_dir, back->dir);
                    // theta = theta>90? 180-theta: theta;
                    if(theta<FLAGS_merge_feature_boundary_theta_thres){
                        to_end=true;
                    }
                }
                cp_idx1 = 30;  // 拟定的一个起点, 延长link时的起点
                LOG_INFO("---------[ Find boundary crosspoint ] generate longer line ... ---------");
            }
            pt1 = new_boundary_line->list[cp_idx1];
            LOG_INFO("---------[ Find boundary crosspoint ] start_pt : ({}, {})---------", pt1->pos.x(), pt1->pos.y());
            BreakInfo *bk1 = new BreakInfo(BreakStatus::SPLIT_MERGE_VECTORIZE, pt1->pos, true);
            bk1->dir = pt1->dir;
            bk1->front_or_back = true;
            // new_crosspoint.push_back(bk);
            session->split_merge_break_points.push_back(bk1);
            
            /* ---------------------------------- 处理边界线尾端 ---------------------------------- */
            // max_index = max_index + (new_boundary_line->list.size() - size); //补线后更新中心点索引
            max_index = new_boundary_line->list.size() / 2;  //直接用中位点
            size = new_boundary_line->list.size();
            if (!find_crosspoint(new_boundary_line, new_single_line, max_index, size, true, cp_idx2)) {
                //补线
                auto link_pt1 = link->list.back();
                std::vector<LaneCenterFeature *> kpfs;
                std::vector<Eigen::Vector3d> dirs;
                lane_center_tree.search(link_pt1->pos, 10, kpfs);
                if (!kpfs.empty()) {
                    for (const auto kp : kpfs) {
                        dirs.push_back(kp->dir);
                    }
                }
                auto lc_dir = alg::get_direction(dirs).normalized();

                auto endpoint = new_boundary_line->list[size-1];
                bool to_end=true;  // 相当于注释掉端点延长; 延长link后理论上不需要再进行这步, 可能导致死循环, 视为异常情况先跳过吧
                while(!to_end){
                    // auto &back=new_boundary_line->list.back();
                    Eigen::Vector3d dir = endpoint->dir;
                    Eigen::Vector3d predict_fit_pnt = endpoint->pos + dir*0.5;
                    predict_fit_pnt = fit.eval(predict_fit_pnt);
                    // 插值 z
                    double interp_z = endpoint->pos.z();
                    std::vector<LaneCenterFeature *> kpfs;
                    lane_center_tree.search(predict_fit_pnt, 5, kpfs);
                    if (!kpfs.empty()) {
                        for (const auto kp : kpfs) {
                            interp_z += kp->pos.z();
                        }
                        interp_z /= (kpfs.size()+1);
                    }
                    Eigen::Vector3d out_fit_pnt(predict_fit_pnt.x(), predict_fit_pnt.y(), interp_z);
                    std::shared_ptr<BoundaryFeature> rb = std::make_shared<BoundaryFeature>();
                    auto new_dir = alg::get_dir(out_fit_pnt, endpoint->pos);
                    rb->pos = out_fit_pnt;
                    rb->dir = new_dir;
                    endpoint->dir = new_dir; // refresh dir
                    new_boundary_line->list.push_back(rb);
                    endpoint = rb;

                    double theta = alg::calc_theta(lc_dir, new_dir);
                    // theta = theta>90? 180-theta: theta;
                    if(theta<FLAGS_merge_feature_boundary_theta_thres){
                        to_end=true;
                    }
                }
                new_boundary_line->list.pop_back();
                cp_idx2 = new_boundary_line->list.size()-30;  // 拟定的一个终点, 延长link时的起点
                LOG_INFO("---------[ Find boundary crosspoint ] generate longer line ... ---------");
            }
            pt2 = new_boundary_line->list[cp_idx2];
            LOG_INFO("---------[ Find boundary crosspoint ] end_pt: ({}, {})---------", pt2->pos.x(), pt2->pos.y());
            BreakInfo* bk2 = new BreakInfo(BreakStatus::SPLIT_MERGE_VECTORIZE, pt2->pos, true);
            bk2->dir = pt2->dir;
            bk2->front_or_back = false;
            // new_crosspoint.push_back(bk);
            session->split_merge_break_points.push_back(bk2);

            /* ---------------------------------- 截取首尾切点之间的边界线 ---------------------------------- */
            auto &vec = new_boundary_line->list;

            auto ref_boundary_line = session->add_ptr(tmp_boundary_line_ptr);
            ref_boundary_line->list = std::vector<std::shared_ptr<BoundaryFeature>>(vec.begin(), vec.end());
            vec = std::vector<std::shared_ptr<BoundaryFeature>>(vec.begin()+cp_idx1, vec.begin()+cp_idx2+1);

            // if(1){
            //     auto log_3 = session->add_debug_log(utils::DisplayInfo::POINT, "completed_lb");
            //     log_3->color= {0,0,255};
            //     int idx = 0;
            //     for(auto pt : ref_boundary_line->list){
            //         auto &ele=log_3->add(pt->pos);
            //         ele.color={255,0,0};
            //         ele.label.score=pt->curvature;
            //         ele.label.intensity_opt=idx++;
            //         if(max_curvature==pt->curvature){
            //             ele.color={0,0,255};
            //         }
            //     }
            // }

            // 6. 判断拟合出的边界线是否合格, 并写入到turn_right_candidate[link]中
            if (ref_boundary_line->list.size() > 0) {
                std::vector<TurnRightPoint> new_trcs;
                std::vector<std::shared_ptr<CrossPoint<BoundaryGroupLine>>> tmp_new_boundary_line;
                int num_fit_pts = ref_boundary_line->list.size();
                for (int k = 0; k < num_fit_pts; k++) {
                    std::shared_ptr<CrossPoint<BoundaryGroupLine>> new_point \
                        = std::make_shared<CrossPoint<BoundaryGroupLine>>(PointStatus::FIT, ref_boundary_line->list[k]->pos);
                    tmp_new_boundary_line.push_back(new_point);

                    Eigen::Vector3d dir={0,0,0};
                    TurnRightPoint  new_turn_right_point;
                    new_turn_right_point.lb=std::make_shared<fast_road_model::LBPoint>();
                    new_turn_right_point.lb->init(ref_boundary_line->list[k]->pos, ref_boundary_line->list[k]->dir);
                    new_trcs.push_back(new_turn_right_point);
                }

                if(check_fit_points(tmp_new_boundary_line) == false) {
                    LOG_INFO("[  make_right_turn -- 4.refine_boundary_candidate: [check_fit_points=false, ABNORMAL_CURVATURE]...");
                    // continue;
                }
                
                // write into 
                // LOG_INFO("---------- Refine_boundary: success!!! [before -- ({}), after -- ({})]", turn_right_candidate[link].size(), new_trcs.size());
                LOG_INFO("[  make_right_turn -- 4.refine_boundary_candidate: success... [before -- ({}), after -- ({})]", turn_right_candidate[link].size(), new_trcs.size());
                turn_right_candidate[link] = new_trcs;
            }

            //  Step 02. 构造右转：边界线、右车道线、中心线
            // 1. 根据拟合结果, 构造边界线并写入全局列表中 (for 融合)
            auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
            BoundaryFeature* prev = NULL;
            int line_index = 0;
            for (int64_t j = 0; j < new_boundary_line->list.size(); ++j) {
                auto &pt = new_boundary_line->list[j];
                if(pt == nullptr) {
                    continue;
                }
                
                if(pt->invalid()) {
                    continue;
                }
                
                auto new_node = session->add_ptr(session->boundary_feature_ptr);
                // // 利用参考boundary_feature进行初始化
                // if(ref_boundary_feature){
                //     new_node->init(ref_boundary_feature);
                // } else {
                //     new_node->type = 2;
                // }
                new_node->line_id = std::to_string(session->road_boundary_id);
                new_node->line_index = line_index++;
                new_node->dir = pt->dir;
                new_node->pos = pt->pos;
                new_node->raw_pos = pt->pos;
                new_node->key_pose = pt->key_pose;
                new_node->trail_id = std::to_string(session->road_boundary_id);
                new_node->filter_status = 1;
                new_node->boundary_type = LaneType::RIGHT_TURN_RB_LEFT; // 右转生成的边界线
                int default_type = session->get_default_label_map("turn_right", "type", "boundary", "BYD");
                new_node->type = ref_boundary_feature? ref_boundary_feature->type: default_type;
                // new_node->score = 100; // 注意:在merge feature中必须要加
                
                if (prev != NULL) {
                    prev->dir = alg::get_dir(new_node->pos, prev->pos);
                    new_node->dir = prev->dir;
                    // new_node->src->dir = prev->dir;
                    
                    new_node->set_prev(prev);
                }
                prev = new_node.get();
                
                new_line->list.push_back(new_node.get());
            }
            
            new_line->id = session->road_boundary_id++;
            session->bev_frame_boundary_line[""].push_back(new_line.get());
            // LOG_INFO("-------- generate right_boundary:  success... ---------");
            LOG_INFO("[  make_right_turn -- 5.generate right_boundary [size={}]: success... ---------", new_line->list.size());

            if(1){
                auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "generated_rb");
                log_2->color= {0,255,0};
                int idx = 0;
                for(auto pt : new_line->list){
                    auto &ele=log_2->add(pt->pos);
                    ele.color={128,128,0};
                    ele.label.score=pt->curvature;
                    ele.label.intensity_opt=idx++;
                    if(max_curvature==pt->curvature){
                        ele.color={0,0,255};
                    }
                }

                auto &ele=log_2->add(session->data_processer->_center_link_pos);
                ele.color={255,0,0};
                // session->data_processer->_center_link_pos
            }

            // 2. 遍历turn_right_candidate[link], 初步找到对应的左右车道线、中心线
            std::vector<LaneCenterGroupLine*> matched_lane_centers;
            std::vector<LaneLineSampleGroupLine*> matched_right_lane_lines;
            std::vector<LaneLineSampleGroupLine*> matched_left_lane_lines;
            double dis1=-1, dis2=-1, dis3=-1;
            UMAP<LaneCenterGroupLine*, int> cache_map1;
            UMAP<LaneLineSampleGroupLine*, int> cache_map2;
            UMAP<LaneLineSampleGroupLine*, int> cache_map3;

            auto &trcs = turn_right_candidate[link];
            for (int i = 0; i < trcs.size(); i++)
            {
                auto lc= connect_lane_center(trcs[i].lb, dis1);
                if(lc)
                {
                    trcs[i].lc = lc;
                    trcs[i].lc_distance = dis1;
                    if(!MAP_FIND(cache_map1, lc->group_line)){
                        matched_lane_centers.push_back(lc->group_line);
                    }

                    if(!ref_lc_feature){
                        ref_lc_feature=lc;
                    }
                }else
                {
                    dis1=-1;
                }

                auto right_ll= connect_laneline(trcs[i].lb, true, dis2);
                if(right_ll)
                {
                    trcs[i].right_ll = right_ll;
                    if(!MAP_FIND(cache_map2, right_ll->group_line)){
                        matched_right_lane_lines.push_back(right_ll->group_line);
                    }

                    if(!ref_rlane_feature){
                        ref_rlane_feature=right_ll;
                    }
                }else
                {
                    dis2=-1;
                }

                auto left_ll= connect_laneline(trcs[i].lb, false, dis3);
                if(left_ll)
                {
                    trcs[i].left_ll = left_ll;
                    if(!MAP_FIND(cache_map3, left_ll->group_line)){
                        matched_left_lane_lines.push_back(left_ll->group_line);
                    }

                    if(!ref_llane_feature){
                        ref_llane_feature=left_ll;
                    }
                }else
                {
                    dis3=-1;
                }
            }

            if(0){
                // auto log_1 = session->add_debug_log(utils::DisplayInfo::POINT, "completed_link");
                // log_1->color= {0,0,255};
                // int idx = 0;
                // for(auto pt : new_single_line){
                //     auto &ele=log_1->add(pt->pos);
                //     ele.color={0,0,255};
                //     ele.label.score=pt->curvature;
                //     ele.label.intensity_opt=idx++;
                // }

                auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "found_light_lb");
                log_2->color= {0,255,0};

                LOG_INFO("[print turn_right info] matched_left: {}; matched_right: {}; matched_lane_centers: {}",
                    matched_left_lane_lines.size(), matched_right_lane_lines.size(), matched_lane_centers.size());
                for(auto &line1: matched_left_lane_lines){
                    for(auto &pt1: line1->list){
                        auto &ele1=log_2->add(pt1->pos);
                        ele1.color={0,0,255};
                        ele1.label.label = 1;
                    }
                }

                for (int i = 0; i < trcs.size(); i++)
                {
                    if(trcs[i].left_ll)
                    {
                        auto &ele1=log_2->add(trcs[i].left_ll->pos);
                        ele1.color={0,255,0};
                        ele1.label.label = 2;
                    }
                }
                // for(auto &line2: matched_right_lane_lines){
                //     for(auto &pt2: line2->list){
                //         auto &ele2=log_2->add(pt2->pos);
                //         ele2.color={0,128,0};
                //         ele2.label.label = 2;
                //     }
                // }
                // for(auto &line3: matched_lane_centers){
                //     for(auto &pt3: line3->list){
                //         auto &ele3=log_2->add(pt3->pos);
                //         ele3.color={0,128,128};
                //         ele3.label.label = 3;
                //     }
                // }
            }

            // 3. 构建对应的右车道线

            // 构建新线
            Eigen::Vector3d center = session->data_processer->_center_link_pos;  //check
            // 找到曲率最大的位置，计算初始的平移量
            int anchor_idx = new_boundary_line->list.size() / 2;
            Eigen::Vector3d anchor_pos = new_boundary_line->list[anchor_idx]->pos;
            Eigen::Vector3d anchor_dir = new_boundary_line->list[anchor_idx]->dir;
            Eigen::Vector3d anchor_dir_vertical = -alg::get_vertical_dir(anchor_dir);
            Eigen::Vector3d anchor_2_center = center - anchor_pos;
            anchor_2_center.normalize();
            // if(alg::calc_theta1(anchor_2_center, anchor_dir_vertical, true, true) > 90) {
            //     anchor_dir_vertical = -anchor_dir_vertical;
            // }
            LOG_INFO("anchor info: center-({},{}), pos-({},{}), dir-({},{}), offset_dir-({},{}), center_dir-({},{})",
                center.x(), center.y(), anchor_pos.x(), anchor_pos.y(), anchor_dir.x(), anchor_dir.y(), 
                anchor_dir_vertical.x(), anchor_dir_vertical.y(), anchor_2_center.x(), anchor_2_center.y());

            // 边界线当 src
            auto cloud_src = convert_boundary_line_to_pcd(ref_boundary_line, anchor_pos);
            if (cloud_src->size() < 3) {
                continue;
            }

            // *逻辑: 将找到的右车道线和边界线进行配准，计算得到偏移量，用于生成新右车道线
            // 待拟合的右车道线当作 target
            LOG_INFO("yx debug before right_laneline [size={}]",   session->bev_frame_lane_line[""].size());
            auto cloud_target = convert_lane_lines_to_pcd(matched_right_lane_lines, anchor_pos);
            if (cloud_target->size() > 3) {
                int mode = 1; // 1 ：代表拟合曲线， 2：代表平移曲线，平移距离为 offset，3：点到线
                double offset = std::sqrt(0.3*0.3 + 0.3*0.3); // 模式2下的平移距离
                double search_distance = 1.0;
                Eigen::Vector2d initialT(offset * anchor_dir_vertical.x(), offset * anchor_dir_vertical.y());
                Eigen::Vector2d optimizedT = gaussNewtonOptimization(cloud_target, cloud_src, initialT, mode, offset, search_distance);
                Eigen::Vector3d right_optimizedTs;
                if (fabs(optimizedT.x()) < 0.6 && fabs(optimizedT.y()) < 0.6) {
                    right_optimizedTs = Eigen::Vector3d(optimizedT.x(), optimizedT.y(), 0);
                    LOG_INFO("[make_right_turn] success right_line: cloud_src:{}, cloud_target:{}, T: {} {}", cloud_src->size(), cloud_target->size(), optimizedT.x() , optimizedT.y());
                    if (right_optimizedTs.norm() > 1e-6) {
                        // 创建新线: 添加右车道线
                        auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);
                        LaneLineSample* prev = NULL;
                        int line_index = 0;
                        for (int64_t j = 0; j < new_boundary_line->list.size(); ++j) {
                            auto &pt = new_boundary_line->list[j];
                            if(pt == nullptr) {
                                continue;
                            }
                            
                            if(pt->invalid() || (pt->pos-anchor_pos).norm()>50) {
                                continue;
                            }
                            
                            auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                            
                            auto feature = session->add_ptr(session->raw_client_lane_feature);
                            new_node->src = feature.get();
                            // new_node->src->src = feature.get();
                            new_node->src->line_id = std::to_string(session->lane_boundary_id);
                            line_index++;
                            new_node->src->line_index = line_index;
                            new_node->src->dir = pt->dir;
                            new_node->src->pos = pt->pos + right_optimizedTs;
                            new_node->src->raw_pos = pt->pos + right_optimizedTs;
                            new_node->src->key_pose = pt->key_pose;
                            new_node->src->trail_id = std::to_string(session->lane_boundary_id);
                            
                            new_node->src->score = 1e-6; // 注意：在mergefeature中必须要
                            int default_type = session->get_default_label_map("turn_right", "type", "laneline", "BYD");
                            int default_color = session->get_default_label_map("turn_right", "color", "laneline", "BYD");
                            new_node->src->type = ref_rlane_feature? ref_rlane_feature->attr.type: default_type; // get_std_attr("mmt_to_std", "lane_boundary", "type", 2);
                            new_node->src->color = ref_rlane_feature? ref_rlane_feature->attr.color: default_color; // get_std_attr("mmt_to_std", "lane_boundary", "color", 1);
                            new_node->src->boundary_type = LaneType::RIGHT_TURN_LB_RIGHT; // 右转：右车道线
                            
                            new_node->src->attr.type = new_node->src->type;
                            new_node->src->attr.color = new_node->src->color;
                            
                            new_node->pos = pt->pos + right_optimizedTs;
                            new_node->dir = pt->dir;
                            new_node->filter_status = 1;
                            new_node->attr.type = new_node->src->type;
                            new_node->attr.color = new_node->src->color;
                            
                            if (prev != NULL) {
                                prev->dir = alg::get_dir(new_node->pos, prev->pos);
                                new_node->dir = prev->dir;
                                new_node->src->dir = prev->dir;
                                
                                new_node->set_prev(prev);
                            }
                            prev = new_node.get();
                            
                            new_line->list.push_back(new_node.get());
                        }
                        
                        new_line->id = session->lane_boundary_id++;
                        session->bev_frame_lane_line[""].push_back(new_line.get());

                        if(1){
                            auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "generated_lb");
                            log_2->color= {0,255,0};
                            int idx = 0;
                            for(auto pt : new_line->list){
                                auto &ele=log_2->add(pt->pos);
                                ele.color={0,255,0};
                                ele.label.score=pt->curvature;
                                ele.label.intensity_opt=idx++;
                                if(max_curvature==pt->curvature){
                                    ele.color={0,0,255};
                                }
                            }
                        }
                    }
                } else {
                    LOG_INFO("[make_right_turn] failed right_line: cloud_src:{}, cloud_target:{}, T: {} {}", cloud_src->size(), cloud_target->size(), optimizedT.x() , optimizedT.y());
                }

            }
            // LOG_INFO("--------- generate right_laneline: success!!! [size={}]",   session->bev_frame_lane_line[""].size());
            LOG_INFO("[  make_right_turn -- 6.generate right_laneline:  success... [size={}]",   session->bev_frame_lane_line[""].size());

            // 4. 构建对应的中心线
            // *逻辑: 将找到的中心线和边界线进行配准，计算得到偏移量生成临时的中心线；再拿临时的中心线和找到的中心线点进行拟合，合成最终的中心线
            // 待拟合的车道线、中心线当作 target
            auto cloud_target2 = convert_lane_centers_to_pcd(matched_lane_centers, anchor_pos);
            if (cloud_target2->size() > 3) {
                double offset=0, count=0, dis1=-1.0;
                auto &trcs = turn_right_candidate[link];
                std::vector<TurnRightPoint> tmp_trc1;
                for (int i = 0; i < new_boundary_line->list.size(); i++)
                {   
                    auto &pt = new_boundary_line->list[i];
                    TurnRightPoint  new_turn_right_point;
                    new_turn_right_point.lb=std::make_shared<fast_road_model::LBPoint>();
                    new_turn_right_point.lb->init(pt->pos, pt->dir);
                    auto lc= connect_lane_center(new_turn_right_point.lb, dis1);
                    if(lc)
                    {   
                        // LOG_INFO("----------- {} -------------", dis1);
                        offset+=dis1;
                        count+=1.0;
                    }else
                    {
                        dis1=-1.0;
                    }
                    tmp_trc1.push_back(new_turn_right_point);
                }
                offset = offset/count;
                int mode = 1; // 1 ：代表拟合曲线， 2：代表平移曲线，平移距离为 offset，3：点到线
                offset = offset > 1.5? offset: std::sqrt(2.5*2.5 + 2.5*2.5); // 模式2下的平移距离
                // offset=std::sqrt(2.5*2.5 + 2.5*2.5); // 模式2下的平移距离
                double search_distance = 1.5;
                Eigen::Vector2d init_offset(offset * anchor_dir_vertical.x(), offset * anchor_dir_vertical.y());
                Eigen::Vector3d center_optimizedTs(init_offset.x(), init_offset.y(), 0);
                LOG_INFO("[make_right-turn: center_line] origin offset: {} - ({}, {})", offset, init_offset.x(), init_offset.y());
                bool has_island_intersection = false;
    
                // mode 2:
                // double search_distance = 1.0;
                // 重新找到 anchor pose的位置，其实和max_curvature_info.first一样，但是这个索引是原始的 group_line_src
                int max_curvature_index = 0;
                for (int j = 0; j < cloud_src->points.size(); j++) {
                    Eigen::Vector3d clout_pt(cloud_src->points[j].x, cloud_src->points[j].y, cloud_src->points[j].z);
                    if((clout_pt-anchor_pos).norm() < 1e-6) {
                        max_curvature_index = j;
                        break;
                    }
                }

                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src_stage1(new pcl::PointCloud<pcl::PointXYZINormal>);
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src_stage2(new pcl::PointCloud<pcl::PointXYZINormal>);
                for (int j = 0; j < cloud_src->points.size(); j++) {
                    const auto& pt = cloud_src->points[j];
                    if (pt.intensity < 3 && j < max_curvature_index) {
                        cloud_src_stage1->points.push_back(pt);
                    }
    
                    if (pt.intensity < 3 && j > max_curvature_index) {
                        cloud_src_stage2->points.push_back(pt);
                    }
                }
                cloud_src_stage1->width = cloud_src_stage1->size();
                cloud_src_stage1->height = 1;
                cloud_src_stage1->is_dense = false;
                cloud_src_stage2->width = cloud_src_stage2->size();
                cloud_src_stage2->height = 1;
                cloud_src_stage2->is_dense = false;
    
                std::vector<std::pair<int, int>> matches_back = {};
                if(cloud_src_stage1->size() >= 2) {
                    matches_back = computeMatchs2(cloud_target2, cloud_src_stage1, init_offset, search_distance);
                    LOG_INFO("[make_right-turn: center_line] boundary size1: {}, matched back: {}", cloud_src_stage1->size(), matches_back.size());
                }
    
                std::vector<std::pair<int, int>> matches_front = {};
                if(cloud_src_stage2->size() >= 2) {
                    matches_front = computeMatchs2(cloud_target2, cloud_src_stage2, init_offset, search_distance);
                    LOG_INFO("[make_right-turn: center_line] boundary size1: {}, matched front: {}", cloud_src_stage2->size(), matches_front.size());
                }
                int min_size = std::min(matches_back.size(), matches_front.size());
    
                Eigen::Vector2d optimizedT;optimizedT.setZero();
                if (min_size >= 8) { // 如果有匹配点，则可以走匹配流程, 取相同的个数进行匹配
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src_stage_1_2(new pcl::PointCloud<pcl::PointXYZINormal>);
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_stage_1_2(new pcl::PointCloud<pcl::PointXYZINormal>);
                    // for (int k = 0; k < min_size; k++) { // 取开头的一段，不取中间的
                    for (int k = matches_back.size()-min_size; k < matches_back.size(); k++) { // 取靠近中间的
                        cloud_src_stage_1_2->points.push_back(cloud_src_stage1->points[matches_back[k].second]);
                        cloud_target_stage_1_2->points.push_back(cloud_target2->points[matches_back[k].first]);
    
                    }
                    // for (int k = matches_front.size()-min_size; k < matches_front.size(); k++) { // 取最后的一段，不取中间的
                    for (int k = 0; k < min_size; k++) { // 取靠近中间的
                        cloud_src_stage_1_2->points.push_back(cloud_src_stage2->points[matches_front[k].second]);
                        cloud_target_stage_1_2->points.push_back(cloud_target2->points[matches_front[k].first]);
                    }
                    cloud_src_stage_1_2->width = cloud_src_stage_1_2->size();
                    cloud_src_stage_1_2->height = 1;
                    cloud_src_stage_1_2->is_dense = false;
    
                    cloud_target_stage_1_2->width = cloud_target_stage_1_2->size();
                    cloud_target_stage_1_2->height = 1;
                    cloud_target_stage_1_2->is_dense = false;
                    Eigen::Vector2d optimizedT_stage_1_2 = gaussNewtonOptimization(cloud_target2, cloud_src_stage_1_2, init_offset, mode, offset, search_distance);
                    
                    optimizedT = optimizedT_stage_1_2;
                    if (fabs(optimizedT.x()) < 3 && fabs(optimizedT.y()) < 3) {
                        center_optimizedTs = Eigen::Vector3d(optimizedT.x(), optimizedT.y(), 0);
                        LOG_INFO("[make_right_turn: center_line] 1 success cloud_src:{}, cloud_target:{}, T: {} {}", cloud_src->size(), cloud_target2->size(), optimizedT.x(), optimizedT.y());
                    } else {
                        LOG_INFO("[make_right_turn: center_line] 1 failed cloud_src:{}, cloud_target:{}, T: {} {}", cloud_src->size(), cloud_target2->size(), optimizedT.x(), optimizedT.y());
                    }
                } else if (matches_back.size() <= 5 && matches_front.size() >= 20) {
                    double avg_distance;
                    computeAvgDistance(cloud_target2, cloud_src_stage2, matches_front, Eigen::Vector2d::Zero(), optimizedT, avg_distance);
                    if(avg_distance < 3.3 || (/*has_island_intersection &&*/ avg_distance < 6)) {
                        center_optimizedTs = Eigen::Vector3d(optimizedT.x(), optimizedT.y(), 0);
                    }
                    LOG_INFO("[make_right_turn: center_line] 2 cloud_src:{}, cloud_target:{}, T: {} {}, d: {}, has_island_intersection:{}",
                            cloud_src_stage2->size(), cloud_target2->size(), optimizedT.x(), optimizedT.y(), avg_distance, has_island_intersection);
                } else if (matches_back.size() >= 20 && matches_front.size() <= 5) {
                    double avg_distance;
                    computeAvgDistance(cloud_target2, cloud_src_stage1, matches_back, Eigen::Vector2d::Zero(), optimizedT, avg_distance);
                    if(avg_distance < 3.3 || (/*has_island_intersection &&*/ avg_distance < 6)) {
                        center_optimizedTs = Eigen::Vector3d(optimizedT.x(), optimizedT.y(), 0);
                    }
                    LOG_INFO("[make_right_turn: center_line] 3 cloud_src:{}, cloud_target:{}, T: {} {}, d: {}, has_island_intersection:{}",
                            cloud_src_stage1->size(), cloud_target2->size(), optimizedT.x(), optimizedT.y(), avg_distance, has_island_intersection);
                } else {
                    // TODO: i don't konw how to impelement
                    LOG_INFO("[make_right_turn: center_line] not implement cloud_src:{}, cloud_target:{}, T: {} {}",
                            cloud_src_stage1->size(), cloud_target2->size(), optimizedT.x(), optimizedT.y());
                }

                // add new_centerline
                convert_lane_center_to_bg(trail_ptr);
                if (center_optimizedTs.norm() > 1e-6) {
                    // Eigen::Vector3d anchor_pos = max_curvature_infos_anchor_pose[i][i2];
                    LOG_INFO("[make_right_turn: 7.generate center_line offset - success !!!] T: {} {}", center_optimizedTs.x(), center_optimizedTs.y());
                    
                    // 1. 添加匹配、构建到的中心线
                    std::vector<Eigen::Vector3d> points_lc;
                    std::vector<Eigen::Vector3d> points_without_z_lc;
                    double dis1=-1;
                    std::vector<TurnRightPoint> tmp_trc;
                    for (int i = 0; i < new_boundary_line->list.size(); i++)
                    {   
                        auto &pt = new_boundary_line->list[i];
                        TurnRightPoint  new_turn_right_point;
                        new_turn_right_point.lb=std::make_shared<fast_road_model::LBPoint>();
                        new_turn_right_point.lb->init(pt->pos, pt->dir);
                        auto lc= connect_lane_center(new_turn_right_point.lb, dis1);
                        if(lc)
                        {   
                            points_lc.push_back(lc->pos);
                            points_without_z_lc.push_back({lc->pos.x(), lc->pos.y(), 0});
                            new_turn_right_point.lc_distance=dis1;
                        }else
                        {
                            dis1=-1;
                        }
                        tmp_trc.push_back(new_turn_right_point);
                    }
                    // for (int i = 0; i < new_boundary_line->list.size(); i++){
                    //     if(trcs[i].lc){
                    //         points_lc.push_back(trcs[i].lc->pos);
                    //         points_without_z_lc.push_back({trcs[i].lc->pos.x(), trcs[i].lc->pos.y(), 0});
                    //     }
                    // }
                    for (int64_t j = 0; j < new_boundary_line->list.size(); ++j) {
                        auto &pt = new_boundary_line->list[j];
                        auto tpos = pt->pos + center_optimizedTs;
                        points_lc.push_back(tpos);
                        points_without_z_lc.push_back({tpos.x(), tpos.y(), 0});
                    }

                    if(0){
                        auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "found_lc_pt");
                        log_2->color= {255,0,0};
                        int idx = 0;
                        for(auto pt : points_lc){
                            auto &ele=log_2->add(pt);
                            ele.color={255,0,0};
                            ele.label.intensity_opt=idx++;
                        }
                    }

                    // 2. 拟合中心线曲线
                    PolyFit::Problem problem;
                    problem.degree=3;
                    problem.points=points_without_z_lc;
                    PolyFit::Options option;
                    option.use_calc_direction=true;
                    option.use_calc_center=true;
                    PolyFit fit=PolyFit(problem,option); 
                    fit.solver();

                    auto new_center_line = session->add_ptr(tmp_boundary_line_ptr);
                    std::vector<Eigen::Vector3d> fit_points_lc;
                    Eigen::Vector3d last_fit_pnt;
                    double pre_distance=2.5;
                    for (int i = 0; i < tmp_trc.size(); i++){
                        Eigen::Vector3d pre_fit_pnt, fit_pnt_without_z;
                        if(tmp_trc[i].lc){
                            pre_fit_pnt = tmp_trc[i].lc->pos;
                            fit_pnt_without_z=fit.eval({pre_fit_pnt.x(), pre_fit_pnt.y(), 0});
                            pre_distance = tmp_trc[i].lc_distance;
                        }else{
                            pre_fit_pnt = alg::get_vertical_pos(tmp_trc[i].lb->pos, tmp_trc[i].lb->dir, -pre_distance );
                            fit_pnt_without_z=fit.eval({pre_fit_pnt.x(), pre_fit_pnt.y(), 0});
                        }

                        Eigen::Vector3d out_fit_pnt(fit_pnt_without_z.x(), fit_pnt_without_z.y(), pre_fit_pnt.z());
                        std::shared_ptr<BoundaryFeature> rb = std::make_shared<BoundaryFeature>();
                        rb->pos = out_fit_pnt;
                        new_center_line->list.push_back(rb);
                        double curvature= fit.curvature(out_fit_pnt);
                        if(new_center_line->list.size()>0){
                            new_center_line->list.back()->curvature=curvature;
                            auto &back=new_center_line->list.back();
                            back->dir = alg::get_dir(out_fit_pnt, last_fit_pnt);
                        }

                        last_fit_pnt = out_fit_pnt;
                    }

                    // for (int i = 0; i < new_boundary_line->list.size(); i++){   
                    //     auto &pt = new_boundary_line->list[i];
                    //     Eigen::Vector3d pre_fit_pnt, fit_pnt_without_z;
                    //     pre_fit_pnt = pt->pos;
                    //     fit_pnt_without_z=fit.eval({pre_fit_pnt.x(), pre_fit_pnt.y(), 0});

                    //     Eigen::Vector3d out_fit_pnt(fit_pnt_without_z.x(), fit_pnt_without_z.y(), pre_fit_pnt.z());
                    //     std::shared_ptr<BoundaryFeature> rb = std::make_shared<BoundaryFeature>();
                    //     rb->pos = out_fit_pnt;
                    //     rb->dir = alg::get_dir(out_fit_pnt, last_fit_pnt);
                    //     if(new_center_line->list.size()>0){
                    //         auto &back=new_center_line->list.back();
                    //         back->dir = alg::get_dir(out_fit_pnt, last_fit_pnt);
                    //     }
                    //     new_center_line->list.push_back(rb);
                    //     double curvature= fit.curvature(out_fit_pnt);
                    //     new_center_line->list.back()->curvature=curvature;

                    //     last_fit_pnt = out_fit_pnt;
                    // }

                    // if(new_center_line->list.size()>0){
                    //     new_center_line->list.pop_back();
                    // }
    
                    // 3. 构建新的中心线
                    auto new_line_lc = session->add_ptr(session->lane_center_line_ptr_for_merge);
                    std::shared_ptr<LaneCenterFeature> prev = nullptr;
                    int line_index = 0;
                    for (int64_t j = 0; j < new_center_line->list.size(); ++j) {
                        auto &pt = new_center_line->list[j];
                        if(pt == nullptr) {
                            continue;
                        }
    
                        if(pt->invalid() || (pt->pos-anchor_pos).norm()>50) {
                            continue;
                        }
                        
                        auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                        new_node->line_id = std::to_string(session->lane_center_id);
                        new_node->line_index = line_index++;
                        new_node->dir = pt->dir;
                        new_node->pos = pt->pos;
                        new_node->raw_pos = pt->pos;
                        new_node->key_pose = pt->key_pose;
                        new_node->trail_id = std::to_string(session->lane_center_id);
                        new_node->filter_status = 1;
                        new_node->boundary_type = LaneType::RIGHT_TURN_LC; // 右转生成的中心线
                        int default_type = session->get_default_label_map("turn_right", "type", "centerline", "BYD");
                        new_node->type = ref_lc_feature? ref_lc_feature->type: default_type; 
                        new_node->score = 100; // 注意:在merge feature中必须要加
    
                        if (prev != NULL) {
                            prev->dir = alg::get_dir(new_node->pos, prev->pos);
                            new_node->dir = prev->dir;
    
                            new_node->set_prev(prev.get());
                        }
                        prev = new_node;
    
                        new_line_lc->list.push_back(new_node.get());
                        new_line_lc->boundary_type = new_node->boundary_type;
                    }
    
                    new_line_lc->id = session->lane_center_id++;
                    // 检查是否和其他的直行线有重叠点, 对线段进行打断
                    merge_lane_center_split_line(session, trail_ptr, new_line_lc);
                    session->bev_frame_lane_center_line[""].push_back(new_line_lc.get());
                    LOG_INFO("[  make_right_turn -- 7.generate centerline:  success... ");

                    if(1){
                        auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "generated_lc");
                        log_2->color= {0,0,255};
                        int idx = 0;
                        for(auto pt : new_line_lc->list){
                            auto &ele=log_2->add(pt->pos);
                            ele.color={0,0,255};
                            ele.label.score=pt->curvature;
                            ele.label.intensity_opt=idx++;
                            if(max_curvature==pt->curvature){
                                ele.color={0,255,0};
                            }
                        }
                    }

                    // 5. 构建新的link
                    // *逻辑: 生成一条和中心线一样的新link
                    std::shared_ptr<KeyPose> prev_pose = nullptr;
                    auto front=link->list.front();
                    std::vector<KeyPose *> new_link;
                    for (int64_t j = 0; j < new_center_line->list.size(); ++j) {
                        auto &pt = new_center_line->list[j];
                        if(pt == nullptr) {
                            LOG_INFO("exist bug ....");
                            continue;
                        }
    
                        // auto pre_fit_pnt = alg::get_vertical_pos(pt->pos, pt->dir, -2.5);
                        // auto fit_pnt_without_z=fit.eval({pre_fit_pnt.x(), pre_fit_pnt.y(), 0});
                        // Eigen::Vector3d out_fit_pnt(fit_pnt_without_z.x(), fit_pnt_without_z.y(), pt->pos.z());
                        
                        auto new_node = session->add_ptr(session->key_pose_ptr);
                        new_node->dir = pt->dir;
                        new_node->pos = pt->pos; //out_fit_pnt;
                        new_node->from_raw_link=front->from_raw_link;
                        new_node->from_link=front->from_link;
    
                        if (prev_pose != NULL) {
                            prev_pose->dir = alg::get_dir(new_node->pos, prev_pose->pos);
                            new_node->dir = prev_pose->dir;
    
                            new_node->set_prev(prev_pose.get());
                        }
                        prev_pose = new_node;
    
                        new_link.push_back(new_node.get());
                    }
                    if(new_link.size()>0){
                        link->list.clear();
                        link->list.assign(new_link.begin(), new_link.end());
                        // LOG_INFO("[make_right_turn: sd_link success !!!] [size: {}]", new_link.size());
                        LOG_INFO("[  make_right_turn -- 8.generate sd_link:  success... [size: {}]", new_link.size());
                        
                        if(0){
                            // session->set_display_name("turn_right_candicate");
        
                            // auto log_1 = session->add_debug_log(utils::DisplayInfo::POINT, "new_link");
                            // log_1->color= {0,255,0};
                            // int idx = 0;
                            // for(auto pt : link->list){
                            //     auto &ele=log_1->add(pt->pos);
                            //     ele.color={0,0,255};
                            //     ele.label.intensity_opt=idx++;
                            // }

                            auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "new_lc");
                            log_2->color= {0,255,0};
                            int idx111 = 0;
                            for(auto pt : new_line_lc->list){
                                auto &ele=log_2->add(pt->pos);
                                ele.color={0,128,128};
                                ele.label.intensity_opt=idx111++;
                            }
                            // session->save_debug_info("turn_right_candicate");

                            LOG_INFO("[make_right_turn: sd_link success !!!] testing [{}] ......", link->list.size());
                        }
                    }
                
                    // 6. 构建新的left_boundary
                    // *逻辑: 利用2倍的中心线偏移量，生成一条左边界线; for 后续成组时使用
                    auto new_line2 = session->add_ptr(session->boundary_line_ptr_for_merge);
                    BoundaryFeature* prev2 = NULL;
                    int line_index2 = 0;
                    int start_idx = int(new_boundary_line->list.size()*0.3), end_idx = int(new_boundary_line->list.size()*0.7);
                    for (int64_t j = start_idx; j < end_idx; ++j) {
                        auto &pt = new_boundary_line->list[j];
                        if(pt == nullptr) {
                            continue;
                        }
                        
                        if(pt->invalid()) {
                            continue;
                        }
                        
                        auto new_node = session->add_ptr(session->boundary_feature_ptr);
                        // // 利用参考boundary_feature进行初始化
                        // if(ref_boundary_feature){
                        //     new_node->init(ref_boundary_feature);
                        // } else {
                        //     new_node->type = 2;
                        // }
                        new_node->line_id = std::to_string(session->road_boundary_id);
                        new_node->line_index = line_index2++;
                        new_node->dir = pt->dir;
                        new_node->pos = pt->pos+center_optimizedTs*2.0;
                        new_node->raw_pos = pt->pos+center_optimizedTs*2.0;
                        new_node->key_pose = pt->key_pose;
                        new_node->trail_id = std::to_string(session->road_boundary_id);
                        new_node->filter_status = 1;
                        new_node->boundary_type = LaneType::RIGHT_TURN_RB_LEFT; // 右转生成的边界线
                        int default_type = session->get_default_label_map("turn_right", "type", "boundary", "BYD");
                        new_node->type = ref_boundary_feature? ref_boundary_feature->type: default_type;
                        // new_node->score = 100; // 注意:在merge feature中必须要加
                        
                        if (prev2 != NULL) {
                            prev2->dir = alg::get_dir(new_node->pos, prev2->pos);
                            new_node->dir = prev2->dir;
                            // new_node->src->dir = prev->dir;
                            
                            new_node->set_prev(prev2);
                        }
                        prev2 = new_node.get();
                        
                        new_line2->list.push_back(new_node.get());
                    }
                    
                    new_line2->id = session->road_boundary_id++;
                    session->bev_frame_boundary_line[""].push_back(new_line2.get());
                    // LOG_INFO("[make_right_turn: left_boundary success !!!] [size: {}] ......", link->list.size());
                    LOG_INFO("[  make_right_turn -- 9.generate left_boundary:  success... [size: {}]", link->list.size());
                    
                    if(1){
                        auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "found_light_lb");
                        log_2->color= {0,255,0};

                        for (int64_t j = start_idx; j < end_idx; ++j)
                        {
                            auto &pt = new_boundary_line->list[j];
                            Eigen::Vector3d p = pt->pos+center_optimizedTs*2.0;
                            auto &ele1=log_2->add(p);
                            ele1.color={0,255,0};
                            ele1.label.label = 3;
                        }
                    }
                }

            }
            
            // LOG_INFO("[make_right_turn: left_boundary success !!!] [size: {}] ......", link->list.size());
        }

        // 重新添加现有的车道线
        // session->lane_line_sample_line_ptr_for_merge.clear();
        // session->bev_frame_lane_line.clear();
        int lane_line_size = trail_ptr->lane_line_group_list.size();
        for (int64_t i = 0; i < lane_line_size; ++i) {
            auto &group_line = trail_ptr->lane_line_group_list[i];
            auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            LaneLineSample* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                if(pt->invalid()) {
                    continue;
                }

                if (prev != NULL) {
                    pt->set_prev(prev);
                }
                
                auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                new_node->init(pt.get());
                new_line->list.push_back(new_node.get());

                prev = pt.get();
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_line[""].push_back(new_line.get());
        }

        // 重新添加现有的中心线
        // session->lane_center_line_ptr_for_merge.clear();
        // session->bev_frame_lane_center_line.clear();
        int lane_center_size = trail_ptr->lane_center_line_group_list.size();
        for (int64_t i = 0; i < lane_center_size; ++i) {
            auto &group_line = trail_ptr->lane_center_line_group_list[i];

            auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);
            new_line->boundary_type = group_line->boundary_type;
            std::shared_ptr<LaneCenterFeature> prev = nullptr;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }
                if(pt->invalid()) {
                    continue;
                }

                if (prev != NULL) {
                    pt->set_prev(prev.get());
                }

                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                new_node->init(pt.get());

                new_line->list.push_back(new_node.get());

                prev = pt;
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_center_line[""].push_back(new_line.get());
        }

        // 重新添加现有的边界线
        // session->boundary_line_ptr_for_merge.clear();
        // session->bev_frame_boundary_line.clear();
        int boundary_line_size = trail_ptr->boundary_line_group_list.size();
        for (int64_t i = 0; i < boundary_line_size; ++i) {
            auto &group_line = trail_ptr->boundary_line_group_list[i];

            auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
            new_line->boundary_type = group_line->boundary_type;
            std::shared_ptr<BoundaryFeature> prev = nullptr;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }
                if(pt->invalid()) {
                    continue;
                }

                if (prev != NULL) {
                    pt->set_prev(prev.get());
                }

                auto new_node = session->add_ptr(session->boundary_feature_ptr);
                new_node->init(pt.get());

                new_line->list.push_back(new_node.get());

                prev = pt;
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_boundary_line[""].push_back(new_line.get());
        }
        
        // 7. 重新调用 merge single 操作
        merge_single_lane_by_trail(session, trail_ptr);
        merge_lane_duplicate(session, trail_ptr, false, 0);
        merge_lane_duplicate(session, trail_ptr, true, 0);

        merge_single_lane_center_by_trail(session, trail_ptr);
        merge_lane_center_duplicate(session, trail_ptr, false, 0);
        merge_lane_center_duplicate(session, trail_ptr, true, 0);

        merge_single_boundary_by_trail(session, trail_ptr);
        merge_boundary_duplicate(session, trail_ptr, false);
        merge_boundary_duplicate(session, trail_ptr, true);
    }

    session->save_debug_info("turn_right_candicate");
    // LOG_INFO("-------- make_right_turn  sucessfully --------");
    // 
    return fsdmap::SUCC;
}

void RoadModelProcMergeFeature::convert_lane_center_to_bg(KeyPoseLine* trail_ptr)
{
    int line_size = trail_ptr->lane_center_line_group_list.size();
    all_lc.clear();
    all_lc_pnt_dir.clear();

    all_lc.resize(line_size);
    all_lc_pnt_dir.resize(line_size);
    for (int64_t i = 0; i < line_size; ++i) {
        auto &group_line = trail_ptr->lane_center_line_group_list[i];
        group_line->cur_line_id = i;
        
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if(pt == nullptr) {
                LOG_ERROR("!!!!!!!!!!!!not normal point!!!!!!!");
                continue;
            }
            all_lc[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
            all_lc_pnt_dir[i].push_back(pt->dir);
        }
    }

}

void RoadModelProcMergeFeature::merge_lane_center_split_line(RoadModelSessionData* session, KeyPoseLine* trail_ptr, std::shared_ptr<LaneCenterLine> new_line)
{   
    // bool debug_log = false;
    bool debug_log = true;
    alg::linestring_t new_lc;
    std::vector<Eigen::Vector3d> new_lc_pnt_dir;
    int new_pnt_size = new_line->list.size();

    for (int64_t j = 0; j < new_pnt_size; ++j) {
        auto &pt = new_line->list[j];
        if(pt == nullptr) {
            LOG_ERROR("!!!!!!!!!!!!not normal point!!!!!!!");
            continue;
        }
        new_lc.push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
        new_lc_pnt_dir.push_back(pt->dir);
    }

    int line_size = trail_ptr->lane_center_line_group_list.size();
    for(int i = 0; i < line_size; i++) {
        alg::OverlapInfoLineString curve_overlap;
        curve_overlap.l1_id = i;
        curve_overlap.l2_id = line_size;
        // lines_overlap_info(all_lc[i], new_lc, all_lc_pnt_dir[i], new_lc_pnt_dir, curve_overlap, 0.1);
        lines_overlap_info(all_lc[i], new_lc, all_lc_pnt_dir[i], new_lc_pnt_dir, curve_overlap, 0.2);
        if (curve_overlap.is_overlap) {

        } else if(curve_overlap.is_nearby) {

        } else if (curve_overlap.is_overlap_num > 5) {
            if(curve_overlap.l1_index.front() > 5 && curve_overlap.l2_index.front() > 5) {
                new_line->list[curve_overlap.l2_index.front()]->filter_status = 2;
                LOG_INFO("[lane_center-s] split-> is_overlap_num: {}, i:{}, l1.size:{}, l2.size:{}, l1.front:{}, l1.back:{}, l2.front:{}, l2.back:{}, xy: {} {}",
                            curve_overlap.is_overlap_num, 
                            i, all_lc[i].size(), new_pnt_size, 
                            curve_overlap.l1_index.front(), curve_overlap.l1_index.back(), 
                            curve_overlap.l2_index.front(), curve_overlap.l2_index.back(),
                            new_line->list[curve_overlap.l2_index.front()]->pos.x(), new_line->list[curve_overlap.l2_index.front()]->pos.y());
            }

            if(curve_overlap.l1_index.back() < all_lc[i].size() - 5 && curve_overlap.l2_index.back() < new_pnt_size - 5) {
                new_line->list[curve_overlap.l2_index.back()]->filter_status = 2;
                LOG_INFO("[lane_center-e] split-> is_overlap_num: {}, i:{}, l1.size:{}, l2.size:{}, l1.front:{}, l1.back:{}, l2.front:{}, l2.back:{}, xy: {} {}",
                            curve_overlap.is_overlap_num, 
                            i, all_lc[i].size(), new_pnt_size, 
                            curve_overlap.l1_index.front(), curve_overlap.l1_index.back(), 
                            curve_overlap.l2_index.front(), curve_overlap.l2_index.back(),
                            new_line->list[curve_overlap.l2_index.back()]->pos.x(), new_line->list[curve_overlap.l2_index.back()]->pos.y());
            }
            if (debug_log) {
                LOG_INFO("[lane_center-a] split-> is_overlap_num: {}, i:{}, l1.size:{}, l2.size:{}, l1.front:{}, l1.back:{}, l2.front:{}, l2.back:{}, xy2-front: {} {}, xy2-back: {} {}",
                            curve_overlap.is_overlap_num, 
                            i, all_lc[i].size(), new_pnt_size, 
                            curve_overlap.l1_index.front(), curve_overlap.l1_index.back(), 
                            curve_overlap.l2_index.front(), curve_overlap.l2_index.back(),
                            new_line->list[curve_overlap.l2_index.front()]->pos.x(), new_line->list[curve_overlap.l2_index.front()]->pos.y(),
                            new_line->list[curve_overlap.l2_index.back()]->pos.x(), new_line->list[curve_overlap.l2_index.back()]->pos.y());
            }
        }
    }

    all_lc.clear();
    all_lc_pnt_dir.clear();
}


int RoadModelProcMergeFeature::remove_short_lines(RoadModelSessionData* session)
{
    // 1. 去除不合理的的短中心线
    auto lit = session->key_pose_map.begin();
    if (lit != session->key_pose_map.end()) {
    // for (auto &lit : session->key_pose_map) {
    //     auto trail_ptr = &lit.second;
        auto trail_ptr = &lit->second;
        int line_size = trail_ptr->lane_center_line_group_list.size();
        std::vector<alg::linestring_t> all_curve(line_size);
        std::vector<std::vector<Eigen::Vector3d>> all_curve_points_dir(line_size);
        for (int64_t i = 0; i < line_size; ++i) {
            auto &group_line = trail_ptr->lane_center_line_group_list[i];
            // group_line->cur_line_id = i;
            
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    LOG_ERROR("!!!!!!!!!!!!not normal point!!!!!!!");
                    continue;
                }
                all_curve[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
                all_curve_points_dir[i].push_back(pt->dir);
            }
        }

        std::unordered_set<int> short_center_lines;
        // std::unordered_set<int> used_map;
        for(int i = 0; i < line_size; i++) {
            auto& l1 = all_curve[i];
            double l1_len = alg::bg::length(l1);
            if (l1_len > 8) {
                continue;
            }
            if (short_center_lines.count(i) > 0) {
                continue;
            }
            
            // for (int j = i+1; j < line_size; j++) {
            for (int j = 0; j < line_size; j++) {
                if (short_center_lines.count(j) > 0 || i == j) {
                    continue;
                }

                auto& l2 =  all_curve[j];
                double l2_len = alg::bg::length(l2);
                if (l2_len < l1_len) {
                    continue;
                }

                int l1_size = l1.size();
                double d_start = alg::bg::distance(l1[0], l2);
                double d_end = alg::bg::distance(l1[l1_size-1], l2);
                if (d_start < 3 && d_end < 3) {
                    short_center_lines.insert(i);

                    // 用于可视化
                    auto &group_line = trail_ptr->lane_center_line_group_list[i];
                    group_line->boundary_type = LaneType::DEBUG;

                    // 仅可视化使用
                    removed_lane_center_line_group_list.push_back(trail_ptr->lane_center_line_group_list[i]);

                    break;
                }
            }
        }

        std::vector<std::shared_ptr<LaneCenterGroupLine>> lane_center_line_group_list_tmp;
        lane_center_line_group_list_tmp.clear();
        for(int i = 0; i < line_size; i++) {
            if(short_center_lines.count(i) == 0) {
                lane_center_line_group_list_tmp.push_back(trail_ptr->lane_center_line_group_list[i]);
            }
        }
        LOG_INFO("!!!!! remove_short_lines origin center line: {}, after {} ", line_size, lane_center_line_group_list_tmp.size());

        trail_ptr->lane_center_line_group_list.clear();
        trail_ptr->lane_center_line_group_list.swap(lane_center_line_group_list_tmp);
    }

    // 2. 去除不合理的短道路边界线
    KeyPoseTree poss_tree; // ins数据
    for (auto &it : session->key_pose_map) {
        for (auto &poss : it.second.list) {
            poss_tree.insert(poss->pos, poss);
        }
    }

    auto lit2 = session->key_pose_map.begin();
    if (lit2 != session->key_pose_map.end()) {
        auto trail_ptr = &lit->second;
        int line_size = trail_ptr->boundary_line_group_list.size();

        std::vector<alg::linestring_t> all_curve(line_size);
        // std::vector<std::vector<Eigen::Vector3d>> all_curve_points_dir(line_size);
        for (int64_t i = 0; i < line_size; ++i) {
            auto &group_line = trail_ptr->boundary_line_group_list[i];
            // group_line->cur_line_id = i;
            
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    LOG_ERROR("!!!!!!!!!!!! boundary not normal point!!!!!!!");
                    continue;
                }
                all_curve[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
                // all_curve_points_dir[i].push_back(pt->dir);
            }
        }

        std::unordered_set<int> invalid_lines;
        for(int i = 0; i < line_size; i++) {
            auto& l1 = all_curve[i];
            double l1_len = alg::bg::length(l1);
            if (l1_len > 20) {
                continue;
            }

            int invalid_cnt = 0;
            int l1_size = l1.size();
            for (int j = 0; j < l1_size; j++) {
                pcl::PointXYZI searchPt(l1[j].x(), l1[j].y(), 0, -1);
                std::vector<float> squaredDistances;
                bool has_found = session->data_processer->search_nearest_pose(searchPt, squaredDistances);
                if (has_found && 
                    ((l1_len < 8 && squaredDistances[0] < 1.75*1.75) || (l1_len > 8 && squaredDistances[0] < 1.2*1.2))) {
                    invalid_cnt++;
                }
            }

            if (invalid_cnt > l1_size * 0.8) {
                invalid_lines.insert(i);

                // 仅可视化使用
                removed_boundary_line_group_list.push_back(trail_ptr->boundary_line_group_list[i]);
            }
        }


        std::vector<std::shared_ptr<BoundaryGroupLine>> boundary_line_group_list_tmp;
        boundary_line_group_list_tmp.clear();
        for(int i = 0; i < line_size; i++) {
            if(invalid_lines.count(i) == 0) {
                boundary_line_group_list_tmp.push_back(trail_ptr->boundary_line_group_list[i]);
            }
        }
        LOG_INFO("!!!!! remove_short_lines origin boundary line: {}, after {} ", line_size, boundary_line_group_list_tmp.size());

        trail_ptr->boundary_line_group_list.clear();
        trail_ptr->boundary_line_group_list.swap(boundary_line_group_list_tmp);
    }

    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::check_direction(RoadModelSessionData* session)
{
    bool open_debug = false;

    // 1. 检查车道线是否反向
    RTreeProxy<BoundaryFeature *, float, 2> road_boundary_tree;
    auto lit3 = session->key_pose_map.begin();
    if (lit3 != session->key_pose_map.end()) {
        auto trail_ptr = &lit3->second;
        for (auto line : trail_ptr->boundary_line_group_list) {
            for (auto rb : line->list) {
                road_boundary_tree.insert(rb->pos, rb.get());
            }
        }
    }

    auto is_reverse_line = [&road_boundary_tree, &open_debug](std::shared_ptr<LaneLineSample> pt) -> bool
    {
        std::vector<BoundaryFeature *> bfs;
        road_boundary_tree.search(pt->pos, 50, bfs);
        auto v_dir = alg::get_vertical_dir(pt->pos, true);
        Eigen::Vector3d p1 = pt->pos + v_dir * 50;
        Eigen::Vector3d p2 = pt->pos - v_dir * 50;

        BoundaryFeature *min_dis_bf_left = NULL;
        BoundaryFeature *min_dis_bf_right = NULL;
        double min_dis_left = 100;
        double min_dis_right = 100;
        for (auto bf : bfs) {
            if (!bf->next) {
                continue;
            }
            Eigen::Vector3d cross;
            if (alg::findIntersection(p1, p2, bf->pos, bf->next->pos, cross))  {
                // TODO CHECK
                if (alg::judge_left2(cross, pt->pos, pt->dir) > 0) { // cross 在 pt 的左侧
                    double dis = alg::calc_dis(cross, pt->pos);
                    if (dis < min_dis_left) {
                        min_dis_left = dis;
                        min_dis_bf_left = bf;
                    }
                } else {
                    double dis = alg::calc_dis(cross, pt->pos);
                    if (dis < min_dis_right) {
                        min_dis_right = dis;
                        min_dis_bf_right = bf;
                    }
                }
            }
        }

        if(open_debug) {
            LOG_ERROR("open_debug qzc 1.1: min_dis_left {} min_dis_right {}",  min_dis_left, min_dis_right);
        }
        if (min_dis_bf_left && min_dis_bf_right) {
            auto theta_left = alg::calc_theta2(pt->dir, min_dis_bf_left->dir, true);
            auto theta_right = alg::calc_theta2(pt->dir, min_dis_bf_right->dir, true);
            if(open_debug) {
                LOG_ERROR("open_debug qzc 1.2: theta_left {} theta_right {}",  theta_left, theta_right);
            }

            if(theta_left > 90 && theta_right > 90) {
                return true;
            }
            if(open_debug) {
                LOG_ERROR("open_debug qzc 1.3: min_dis_left {} min_dis_right {}",  min_dis_left, min_dis_right);
            }
        }
        return false;
    };

    auto reverse_line = [](std::shared_ptr<LaneLineSampleGroupLine>& group_line) {
        reverse(group_line->list.begin(), group_line->list.end());
        std::shared_ptr<LaneLineSample> prev = NULL;
        for (auto &fls : group_line->list) {
            if (prev == NULL) {
                prev = fls;
                continue;
            }
            fls->dir = alg::get_dir(fls->pos, prev->pos);
            prev->dir = fls->dir;
            fls->set_prev(prev.get());
            prev = fls;
        }
    };

    if (lit3 != session->key_pose_map.end()) {
        auto trail_ptr = &lit3->second;

        int line_size = trail_ptr->lane_line_group_list.size();
        for (int64_t i = 0; i < line_size; ++i) {
            auto &group_line = trail_ptr->lane_line_group_list[i];
            if(group_line == nullptr) {
                continue;
            }

            // 为了节省耗时，先从中间位置选3个点检查一下，是否和两侧的道路边界线反向，如果都反向，那么需要全部遍历一遍，判断反向个数，如果大于0.8，那么反向；否则不需要处理
            int pnt_size = group_line->list.size();
            int cnt = 0;
            int reverse_line_cnt = 0;
            for (int64_t j = pnt_size/2; cnt < 3 && j < pnt_size; j+=2,++cnt) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                if(is_reverse_line(pt)) {
                    reverse_line_cnt++;
                }
            }

            if(open_debug && i == 5) {
                LOG_ERROR("open_debug qzc 1: {} pnt_size {} : reverse_line_cnt:{}",  i, pnt_size, reverse_line_cnt);
            }

            if(reverse_line_cnt == 3) {
                reverse_line_cnt = 0;
                for (int j = 0; j < pnt_size; ++j) {
                    auto &pt = group_line->list[j];
                    if(pt == nullptr) {
                        continue;
                    }

                    if(is_reverse_line(pt)) {
                        reverse_line_cnt++;
                    }
                }
            } else {
                continue;
            }

            if(open_debug && i == 5) {
                LOG_ERROR("open_debug qzc 3: {} pnt_size {} : reverse_line_cnt:{}, ratio:{}",  i, pnt_size, reverse_line_cnt, reverse_line_cnt / double(pnt_size));
            }
            if(reverse_line_cnt / double(pnt_size) > 0.6) {
                reverse_line(group_line);
                LOG_INFO("!!!!! [check_direction] reverse lane line: {} cur_line_id {}, ratio:{}", i, group_line->cur_line_id, reverse_line_cnt / double(pnt_size));
            }
        }
    }

    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::merge_single_trail(RoadModelSessionData* session) {
    // 遍历每条轨迹，单独融合每条轨迹里面的boundary信息
    // for (auto &lit : session->key_pose_map) {
    //     auto trail_ptr = &lit.second;
    //     session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
    //             merge_single_boundary_by_trail(session, trail_ptr);
    //             LOG_INFO("trail_boundary_merge[size={}]", 
    //                     trail_ptr->boundary_line_group_list.size());
    //     });
    // }

    auto lit1 = session->key_pose_map.begin();
    if (lit1 != session->key_pose_map.end()) {
        auto trail_ptr = &lit1->second;
        session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
            merge_single_boundary_by_trail(session, trail_ptr);
            LOG_INFO("trail_boundary_merge[size={}]",  trail_ptr->boundary_line_group_list.size());

            // 正序
            if(1) {
                // merge multi group line
                // std::vector<std::shared_ptr<BoundaryLine>> tmp_boundary_line_ptr;
                // UMAP<std::string, std::vector<BoundaryLine*>> tmp_bev_frame_boundary_line; 
                session->boundary_line_ptr_for_merge.clear();
                session->bev_frame_boundary_line.clear();
                
                for (int64_t i = 0; i < trail_ptr->boundary_line_group_list.size(); ++i) {
                    auto &group_line = trail_ptr->boundary_line_group_list[i];

                    // 1 构造单趟合并的 boundary
                    auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
                    new_line->boundary_type = group_line->boundary_type;

                    // std::shared_ptr<BoundaryGroupLine> group_line;
                    //      std::vector<std::shared_ptr<BoundaryFeature>> list;  // 该曲线上所有的点
                    BoundaryFeature* prev = NULL;
                    for (int64_t j = 0; j < group_line->list.size(); ++j) {
                        auto &pt = group_line->list[j];
                        if(pt == nullptr) {
                            continue;
                        }
                        if(pt->invalid()) {
                            continue;
                        }


                        auto new_node = session->add_ptr(session->boundary_feature_ptr);
                        // new_node = pt;
                        // new_node->pose = pt->pos;
                        new_node->init(pt.get());


                        if (prev != NULL) {
                            new_node->set_prev(prev);
                        }
                        prev = new_node.get();

                        new_line->list.push_back(new_node.get());
                    }

                    if (new_line->list.size() <= 2) {
                        continue;
                    }

                    session->bev_frame_boundary_line[""].push_back(new_line.get());


                }
                // session->bev_frame_boundary_line = tmp_bev_frame_boundary_line;

                // 2. 重新调用 merge single 操作
                merge_single_boundary_by_trail(session, trail_ptr);
            }

            // 反序
            if(1) {
                // merge multi group line
                // std::vector<std::shared_ptr<BoundaryLine>> tmp_boundary_line_ptr;
                // UMAP<std::string, std::vector<BoundaryLine*>> tmp_bev_frame_boundary_line; 
                session->boundary_line_ptr_for_merge.clear();
                session->bev_frame_boundary_line.clear();
                
                int lane_boundary_size = trail_ptr->boundary_line_group_list.size();
                for (int64_t i = lane_boundary_size-1; i >= 0; --i) {
                    auto &group_line = trail_ptr->boundary_line_group_list[i];

                    // 1 构造单趟合并的 boundary
                    auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
                    new_line->boundary_type = group_line->boundary_type;

                    // std::shared_ptr<BoundaryGroupLine> group_line;
                    //      std::vector<std::shared_ptr<BoundaryFeature>> list;  // 该曲线上所有的点
                    BoundaryFeature* prev = NULL;
                    for (int64_t j = 0; j < group_line->list.size(); ++j) {
                        auto &pt = group_line->list[j];
                        if(pt == nullptr) {
                            continue;
                        }
                        if(pt->invalid()) {
                            continue;
                        }

                        auto new_node = session->add_ptr(session->boundary_feature_ptr);
                        // new_node = pt;
                        // new_node->pose = pt->pos;
                        new_node->init(pt.get());

                        if (prev != NULL) {
                            new_node->set_prev(prev);
                        }
                        prev = new_node.get();
                        
                        new_line->list.push_back(new_node.get());
                    }

                    if (new_line->list.size() <= 2) {
                        continue;
                    }

                    session->bev_frame_boundary_line[""].push_back(new_line.get());


                }
                // session->bev_frame_boundary_line = tmp_bev_frame_boundary_line;

                // 2. 重新调用 merge single 操作
                merge_single_boundary_by_trail(session, trail_ptr);
            }

            // save_debug_info_only_boundary(session, 1);
            merge_boundary_duplicate(session, trail_ptr, false);
            merge_boundary_duplicate(session, trail_ptr, true);
        });
        // session->road_boundary_id = trail_ptr->boundary_line_group_list.size() + 1;
    }    
    // for (auto &lit : session->key_pose_map) {
    //     auto trail_ptr = &lit.second;
    //     session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
    //             merge_single_lane_by_trail(session, trail_ptr);
    //             LOG_INFO("trail_lane_merge[size={}]", 
    //                     trail_ptr->lane_line_group_list.size());
    //     });
    // }

    auto lit2 = session->key_pose_map.begin();
    if (lit2 != session->key_pose_map.end()) {
        auto trail_ptr = &lit2->second;        
        session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
                merge_single_lane_by_trail(session, trail_ptr);
                LOG_INFO("trail_lane_merge[size={}]", 
                        trail_ptr->lane_line_group_list.size());
                
                // 正序
                if(1) {
                    // merge multi group line
                    session->lane_line_sample_line_ptr_for_merge.clear();
                    session->bev_frame_lane_line.clear();
                    
                    for (int64_t i = 0; i < trail_ptr->lane_line_group_list.size(); ++i) {
                        auto &group_line = trail_ptr->lane_line_group_list[i];

                        // 1 构造单趟合并的 boundary
                        auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);

                        // std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_list; // 该条轨迹里所有lane line，已经做了融合
                        //      std::vector<std::shared_ptr<LaneLineSample>> list;  // 该曲线上所有的点
                        for (int64_t j = 0; j < group_line->list.size(); ++j) {
                            int valid_index = j;
                            // if (j == group_line->list.size() - 1) {
                            //     valid_index = j - 1;
                            // }
                            auto &pt = group_line->list[j];
                            if(pt == nullptr) {
                                continue;
                            }

                            auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                            new_node->init(pt.get());

                            new_line->list.push_back(new_node.get());
                        }

                        if (new_line->list.size() == 0) {
                            continue;
                        }

                        session->bev_frame_lane_line[""].push_back(new_line.get());
                    }

                    // 2. 重新调用 merge single 操作
                    merge_single_lane_by_trail(session, trail_ptr);
                }

                // 反序
                if(1) {
                    // merge multi group line
                    session->lane_line_sample_line_ptr_for_merge.clear();
                    session->bev_frame_lane_line.clear();
                    
                    int lane_line_size = trail_ptr->lane_line_group_list.size();
                    for (int64_t i = lane_line_size-1; i >= 0; --i) {
                        auto &group_line = trail_ptr->lane_line_group_list[i];

                        // 1 构造单趟合并的 boundary
                        auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);

                        // std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_list; // 该条轨迹里所有lane line，已经做了融合
                        //      std::vector<std::shared_ptr<LaneLineSample>> list;  // 该曲线上所有的点
                        for (int64_t j = 0; j < group_line->list.size(); ++j) {
                            int valid_index = j;
                            // if (j == group_line->list.size() - 1) {
                            //     valid_index = j - 1;
                            // }
                            auto &pt = group_line->list[j];
                            if(pt == nullptr) {
                                continue;
                            }

                            auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                            new_node->init(pt.get());

                            new_line->list.push_back(new_node.get());
                        }

                        if (new_line->list.size() == 0) {
                            continue;
                        }

                        session->bev_frame_lane_line[""].push_back(new_line.get());
                    }

                    // 2. 重新调用 merge single 操作
                    merge_single_lane_by_trail(session, trail_ptr);
                }

                // save_debug_info_only_lane(session, 1);
                merge_lane_duplicate(session, trail_ptr, false, 0);
                merge_lane_duplicate(session, trail_ptr, true, 0);
        });
        // session->lane_boundary_id = trail_ptr->lane_line_group_list.size() + 1;
    }

    // for (auto &lit : session->key_pose_map) {
    //     auto trail_ptr = &lit.second;
    //     session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
    //             merge_single_lane_center_by_trail(session, trail_ptr);
    //             LOG_INFO("trail_lane_center_merge[size={}]", 
    //                     trail_ptr->lane_center_line_group_list.size());
    //     });
    // }

    auto lit3 = session->key_pose_map.begin();
    if (lit3 != session->key_pose_map.end()) {
        auto trail_ptr = &lit3->second;
        session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
                merge_single_lane_center_by_trail(session, trail_ptr);
                LOG_INFO("trail_lane_center_merge[size={}]", 
                        trail_ptr->lane_center_line_group_list.size());

                // 正序
                if(1) {
                    // merge multi group line
                    session->lane_center_line_ptr_for_merge.clear();
                    session->bev_frame_lane_center_line.clear();
                    
                    for (int64_t i = 0; i < trail_ptr->lane_center_line_group_list.size(); ++i) {
                        auto &group_line = trail_ptr->lane_center_line_group_list[i];

                        // 1 构造单趟合并的 boundary
                        auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);
                        new_line->boundary_type = group_line->boundary_type;

                        // std::shared_ptr<LaneCenterGroupLine> group_line;
                        //      sstd::vector<std::shared_ptr<LaneCenterFeature>> list;;  // 该曲线上所有的点
                        for (int64_t j = 0; j < group_line->list.size(); ++j) {
                            int valid_index = j;
                            // if (j == group_line->list.size() - 1) {
                            //     valid_index = j - 1;
                            // }
                            auto &pt = group_line->list[j];
                            if(pt == nullptr) {
                                continue;
                            }

                            auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                            // new_node = pt;
                            // new_node->pose = pt->pos;
                            new_node->init(pt.get());

                            new_line->list.push_back(new_node.get());
                        }

                        if (new_line->list.size() == 0) {
                            continue;
                        }

                        session->bev_frame_lane_center_line[""].push_back(new_line.get());
                    }

                    // 2. 重新调用 merge single 操作
                    merge_single_lane_center_by_trail(session, trail_ptr);
                }

                // 反序
                if(1) {
                    // merge multi group line
                    session->lane_center_line_ptr_for_merge.clear();
                    session->bev_frame_lane_center_line.clear();
                    
                    int lane_center_size = trail_ptr->lane_center_line_group_list.size();
                    for (int64_t i = lane_center_size-1; i >= 0; --i) {
                        auto &group_line = trail_ptr->lane_center_line_group_list[i];

                        // 1 构造单趟合并的 boundary
                        auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);
                        new_line->boundary_type = group_line->boundary_type;

                        // std::shared_ptr<LaneCenterGroupLine> group_line;
                        //      sstd::vector<std::shared_ptr<LaneCenterFeature>> list;;  // 该曲线上所有的点
                        for (int64_t j = 0; j < group_line->list.size(); ++j) {
                            int valid_index = j;
                            // if (j == group_line->list.size() - 1) {
                            //     valid_index = j - 1;
                            // }
                            auto &pt = group_line->list[j];
                            if(pt == nullptr) {
                                continue;
                            }

                            auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                            // new_node = pt;
                            // new_node->pose = pt->pos;
                            new_node->init(pt.get());

                            new_line->list.push_back(new_node.get());
                        }

                        if (new_line->list.size() == 0) {
                            continue;
                        }

                        session->bev_frame_lane_center_line[""].push_back(new_line.get());
                    }

                    // 2. 重新调用 merge single 操作
                    merge_single_lane_center_by_trail(session, trail_ptr);
                }

                // save_debug_info_only_lane_center(session, 1);
                merge_lane_center_duplicate(session, trail_ptr, false, 0);
                merge_lane_center_duplicate(session, trail_ptr, true, 0);
        });
        // session->lane_center_id = trail_ptr->lane_center_line_group_list.size() + 1;
    }
    session->thread_pool->wait(5, "merge_single");
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::make_tree(RoadModelSessionData* session) {
    // LOG_INFO("qzc debug after4.0[size={}]", trail->lane_line_group_list.size());
    // for (auto &lit : session->key_pose_map) {
        // auto trail = &lit.second;

    session->link_pos_tree.RemoveAll();
    for (auto& line: session->link_sample_list) {
        for (auto &poss : line->list) {
            session->link_pos_tree.insert(poss->pos, poss); //切断后link的pose
        }
    }

    if(1){
        session->set_display_name("new_link_info");

        auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "new_link");
        log_2->color= {0,255,0};
        int idx111 = 0;
        for (auto& line: session->link_sample_list) {
            for (auto &poss : line->list) {
                auto &ele=log_2->add(poss->pos);
                ele.color={0,255,0};
                ele.label.label = *(poss->from_raw_link->forms.begin());
                ele.label.score = idx111;
            }
            idx111++;
        }
        session->save_debug_info("new_link_info");

    }

    auto lit = session->key_pose_map.begin();
    if (lit != session->key_pose_map.end()) {
        auto trail = &lit->second;
        for (int64_t i = 0; i < trail->boundary_line_group_list.size(); ++i) {
            auto &group_line = trail->boundary_line_group_list[i];
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if (pt->invalid()) {
                    prev = NULL;
                    continue;
                }
                pt->group_line = group_line.get();
                // pt->src_status = 2;
                if (prev != NULL) {
                    pt->set_prev(prev);
                }
                session->boundary_line_sample_tree.insert(pt->pos, pt.get());
                prev = pt.get();
            }
            session->merge_boundary_line_list.push_back(group_line.get()); // 放入每条轨迹融合后的所有boundary曲线
        }
        LOG_INFO("qzc debug after4[size={}]", trail->lane_line_group_list.size());
        for (int64_t i = 0; i < trail->lane_line_group_list.size(); ++i) {
            auto &group_line = trail->lane_line_group_list[i];
            LaneLineSample* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                // if (pt->invalid()) {
                //     prev = NULL;
                //     continue;
                // }
                pt->group_line = group_line.get();
                // pt->src->src_status = 2;
                if (prev != NULL) {
                    pt->set_prev(prev);
                }
                session->lane_line_sample_tree.insert(pt->pos, pt.get());
                prev = pt.get();
                // LOG_INFO("merge_lane_line_list:[{} {}]",pt->attr.color,pt->attr.type);
            }
            session->merge_lane_line_list.push_back(group_line.get());
        }
        LOG_INFO("qzc debug after5[size={}]", trail->lane_line_group_list.size());
        for (int64_t i = 0; i < trail->lane_center_line_group_list.size(); ++i) {
            auto &group_line = trail->lane_center_line_group_list[i];
            LaneCenterFeature* prev = NULL;
            double length=0;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                // if (pt->invalid()) {
                //     prev = NULL;
                //     continue;
                // }
                pt->group_line = group_line.get();
                // pt->src->src_status = 2;
                if (prev != NULL) {
                    pt->set_prev(prev);
                    length+=alg::calc_dis(prev->pos,pt->pos);
                }
                session->merge_lane_center_sample_tree.insert(pt->pos, pt.get());
                prev = pt.get();
            }
            group_line->length=length;
            session->merge_lane_center_list.push_back(group_line.get());
        }
    }
    return fsdmap::SUCC;
}

void RoadModelProcMergeFeature::merge_lane_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process, int stage) {
    bool debug_log = false;
    // bool debug_log = true;
    int line_size = trail_ptr->lane_line_group_list.size();
    std::vector<alg::linestring_t> all_curve(line_size);
    std::vector<std::vector<Eigen::Vector3d>> all_curve_points_dir(line_size);
    for (int64_t i = 0; i < line_size; ++i) {
        auto &group_line = trail_ptr->lane_line_group_list[i];
        group_line->cur_line_id = i;
        
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if(pt == nullptr) {
                LOG_ERROR("!!!!!!!!!!!!not normal point!!!!!!!");
                continue;
            }
            all_curve[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
            all_curve_points_dir[i].push_back(pt->dir);
        }
    }

    std::vector<std::vector<alg::OverlapInfoLineString>> matched_list(line_size); // 每条线段与其他线段的匹配信息
    std::map<int, std::unordered_set<int>> common_prcessed_line; // 当前id，匹配上的line
    for(int i = 0; i < line_size; i++) {
        for (int j = i+1; j < line_size; j++) {
            alg::OverlapInfoLineString curve_overlap;
            curve_overlap.l1_id = i;
            curve_overlap.l2_id = j;
            // lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.05);
            lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.1);
            if (curve_overlap.is_overlap) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                auto &group_line_i = trail_ptr->lane_line_group_list[i];
                group_line_i->matched_line_ids.insert(j);
                auto &group_line_j = trail_ptr->lane_line_group_list[j];
                group_line_j->matched_line_ids.insert(i);

                 if (debug_log && reverse_process == false) {
                    std::cout << "[lane] 1 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 
                            << " l1 len: " << curve_overlap.l1_length << " l2 len: " << curve_overlap.l2_length 
                            << std::endl;
                }
                

                // case1: 有重叠 & 方式一致, 进行合并
            } else if(curve_overlap.is_nearby) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                // auto &group_line_i = trail_ptr->lane_line_group_list[i];
                // group_line_i->matched_line_ids.insert(j);
                // auto &group_line_j = trail_ptr->lane_line_group_list[j];
                // group_line_j->matched_line_ids.insert(i);

                 if (debug_log && reverse_process == false) {
                    std::cout << "[lane] 2 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 << std::endl;
                }
            }
            // case4: 无重叠 & 方式不一致，不处理
        }
    }

    for (auto& matched_info : matched_list) {
        std::sort(matched_info.begin(), matched_info.end(), [](const alg::OverlapInfoLineString& a, const alg::OverlapInfoLineString& b) {
            return a.l2_length > b.l2_length;
        });
    }

    std::sort(matched_list.begin(), matched_list.end(), [](const std::vector<alg::OverlapInfoLineString>& a, const std::vector<alg::OverlapInfoLineString>& b) {
        if (a.size() != b.size()) {
            return a.size() > b.size();
        } else if(a.size() == 0 && b.size() == 0) {
            return false;
        }
        
        return a[0].l1_length > b[0].l1_length;
    });

    std::vector<bool> has_processed(line_size, false);
    // 注意：matched_list 已经不是按照，line_size 的顺序了
    for (const auto& matched_info_i : matched_list) {
        int matched_num = matched_info_i.size();
        if (matched_num == 0) {
            continue;
        }

        // 当自己长度比较短，且需要反转，那么除了0以外的其他line，如果需要反转，则需要和以 need_reverse 为准
        auto const& curve_overlap_0 = matched_info_i[0];
        int id1 = curve_overlap_0.l1_id;
        bool need_reverse = curve_overlap_0.is_need_reverse_l1;

        // 必须要是还有未处理过的match点，再进行融合， TODO：qzc 可能还存在一种情况是，上次已经反转了一次，当前还有，那么可能会导致错误的反转，可能需要同步改掉matched_list中的状态才行
        int valid_match_size = 0;
        std::unordered_set<int> other_line_prcessed;
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            int id2 = curve_overlap_j.l2_id;
            if (has_processed[id2] == true) {
                // 如果已经处理过的id2，对应的匹配信息中，有id1，那么就不能重复处理，需要过滤掉
                if (common_prcessed_line.find(id2) != common_prcessed_line.end() && common_prcessed_line[id2].find(id1) != common_prcessed_line[id2].end()) {
                    other_line_prcessed.insert(common_prcessed_line[id2].begin(), common_prcessed_line[id2].end());
                }
                
                continue;
            }
            // 如果之前处理过，那么直接跳过
            if (other_line_prcessed.find(id2) != other_line_prcessed.end()) {
                continue;
            }

            valid_match_size++;
        }

        if (need_reverse && valid_match_size > 0) {
            reverse(trail_ptr->lane_line_group_list[id1]->list.begin(), trail_ptr->lane_line_group_list[id1]->list.end());
            std::shared_ptr<LaneLineSample> prev = NULL;
            for (auto &fls : trail_ptr->lane_line_group_list[id1]->list) {
                if (prev == NULL) {
                    prev = fls;
                    continue;
                }
                fls->dir = alg::get_dir(fls->pos, prev->pos);
                prev->dir = fls->dir;
                fls->set_prev(prev.get());
                prev = fls;
            }
             if (debug_log && reverse_process == false) {
                std::cout << "[lane] 0#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << curve_overlap_0.l2_id << std::endl;
            }
        }

        
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            
            int id2 = curve_overlap_j.l2_id;
             if (debug_log && reverse_process == false) {
                std::cout << "[lane] 1#need_reverse: " << need_reverse << " id1 : " << id1 << " id2: " << id2  << " matched_num: " << matched_num  << " has_processed[id2]:" << has_processed[id2] << std::endl; 
            }
            if (has_processed[id2] == true) {
                continue;
            }

            if (need_reverse == false) {
                // 如果自己不需要反转，其他和自己不相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 != curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->lane_line_group_list[id2]->list.begin(), trail_ptr->lane_line_group_list[id2]->list.end());
                    std::shared_ptr<LaneLineSample> prev = NULL;
                    for (auto &fls : trail_ptr->lane_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                        fls->set_prev(prev.get());
                        prev = fls;
                    }
                     if (debug_log && reverse_process == false) {
                        std::cout << "[lane] 2#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            } else {
                // 如果自己需要反转，其他和自己相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 == curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->lane_line_group_list[id2]->list.begin(), trail_ptr->lane_line_group_list[id2]->list.end());
                    std::shared_ptr<LaneLineSample> prev = NULL;
                    for (auto &fls : trail_ptr->lane_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                        fls->set_prev(prev.get());
                        prev = fls;
                    }
                     if (debug_log && reverse_process == false) {
                        std::cout << "[lane] 3#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            }

            // // case1: 有重叠 & 方向一致, 进行合并
            // // case2: 有重叠 & 方向不一致，但是是180度, 则先进行反转再进行合并
            // if (curve_overlap_j.is_overlap) {
            // } 
            // // case3: 无重叠 & 方向一致（首末尾）, 进行合并
            // if (curve_overlap_j.is_nearby) {
            // }
            // case4: 无重叠 & 方向不一致，不处理
        }

        has_processed[id1] = true;
    }
    

    // merge multi group line
    session->lane_line_sample_line_ptr_for_merge.clear();
    session->bev_frame_lane_line.clear();
    
    int lane_size = trail_ptr->lane_line_group_list.size();
    if (reverse_process) {
        for (int64_t i = lane_size-1; i >= 0; --i) {
            auto &group_line = trail_ptr->lane_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_list; // 该条轨迹里所有lane line，已经做了融合
            //      std::vector<std::shared_ptr<LaneLineSample>> list;  // 该曲线上所有的点
            LaneLineSample* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                // if (j == group_line->list.size() - 1) {
                //     valid_index = j - 1;
                // }
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                if(pt->invalid()) {
                    continue;
                }

                auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                new_node->init(pt.get());

                if (prev != NULL) {
                    new_node->set_prev(prev);
                }
                prev = new_node.get();

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_line[""].push_back(new_line.get());
        }
    } else {
        for (int64_t i = 0; i < lane_size; ++i) {
            auto &group_line = trail_ptr->lane_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_list; // 该条轨迹里所有lane line，已经做了融合
            //      std::vector<std::shared_ptr<LaneLineSample>> list;  // 该曲线上所有的点
            LaneLineSample* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                // if (j == group_line->list.size() - 1) {
                //     valid_index = j - 1;
                // }
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                if(pt->invalid()) {
                    continue;
                }

                auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                new_node->init(pt.get());

                if (prev != NULL) {
                    new_node->set_prev(prev);
                }
                prev = new_node.get();

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_line[""].push_back(new_line.get());
        }

    }

    // 2. 重新调用 merge single 操作
    merge_single_lane_by_trail(session, trail_ptr);
}

int RoadModelProcMergeFeature::merge_single_lane_by_trail(RoadModelSessionData* session,
        KeyPoseLine* trail) {
    int min_size = FLAGS_merge_feature_proc_min_size;
    int min_match_size = FLAGS_merge_feature_proc_min_match_size;
    double score_thres = FLAGS_merge_feature_proc_score_threshold;
    if (trail->list.size() < min_size) {
        return fsdmap::FAIL;
    }
    trail->lane_line_group_list.reserve(trail->list.size());

    int times = 0;
    while (times < 2) {
        ++times;
        LaneLineSampleTree trail_tree;
        trail->lane_line_group_list.clear();
        int64_t raw_line_size = 0;
        // for (int64_t i = 0; i < trail->list.size(); ++i) {
        //     auto &poss = trail->list[i];
        //     // if (poss->frame_id == "6231672047094_1672018298180000") {
        //     //     int a = 1;
        //     // }
        //     if (MAP_NOT_FIND(session->bev_frame_lane_line, poss->frame_id)) {
        //         continue;
        //     }
        //     auto &ins_vec = session->bev_frame_lane_line[poss->frame_id];
        LOG_INFO("session->bev_frame_lane_line size: {}", session->bev_frame_lane_line.size());
        for (const auto &ins_pair : session->bev_frame_lane_line) {
            auto &ins_vec = ins_pair.second;
            if (ins_vec.empty()) {
                continue;
            }
            for (auto &ins : ins_vec) {
                ins->sub_line.clear();
                LaneLineSample* prev = NULL;
                auto new_line = session->add_ptr(ins->sub_line);
                new_line->list.reserve(ins->list.size());
                new_line->src = ins; // src表示sub_line的每段都是来自于父曲线ins
                int64_t sub_line_index = 0;
                new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);

                // add by qzc
                new_line->cur_line_id = ins->cur_line_id;
                new_line->matched_line_ids = ins->matched_line_ids;

                for (auto &pt : ins->list) {
                    session->debug_pos(pt->pos);
                    if (pt->invalid()) {
                        if (new_line->list.size() > 1) {
                            new_line = session->add_ptr(ins->sub_line);
                            new_line->src = ins;
                            new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);
                        } else {
                            new_line->list.clear();
                        }
                        prev = NULL;
                        continue;
                    }
                    new_line->list.push_back(pt);
                    pt->merge_match_map.clear();
                    if (prev != NULL) {
                        pt->set_prev(prev);
                    }
                    prev = pt;
                }
                for (auto &sub_line : ins->sub_line) {
                    if (sub_line->list.size() <= 1) {
                        continue;
                    }
                    match_frame_line_lane(session, trail->lane_line_group_list, trail_tree, sub_line.get());
                }
            }
            // save_frame_log_lane(session, trail, poss, times);
            raw_line_size += ins_vec.size();

        }
        LOG_INFO("merge_lane_final[id={}, p_size={}, times={}, size={}, raw_size={}]", 
                (int64_t)trail, trail->list.size(), times, 
                trail->lane_line_group_list.size(), raw_line_size);
        // 过滤
        #if 0
        std::vector<LaneLineSample*> match_fls_list;
        for (int64_t i = 0; i < trail->lane_line_group_list.size(); ++i) {
            auto &group_line = trail->lane_line_group_list[i];
            LaneLineSample* prev_fls = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                if (j == group_line->list.size() - 1) {
                    valid_index = j - 1;
                }
                auto &pt = group_line->list[j];
                // session->debug_pos(pt->pos);
                // int match_num = group_line->match_list[j].size();
                int match_num = group_line->match_list_point[valid_index].size();
                // UMAP<std::string, int> match_frame_map;
                // for (auto &raw_pt : group_line->match_list_point[valid_index]) {
                //     match_frame_map[raw_pt->src->frame_id]++;
                // }
                match_fls_list.clear();
                VEC_PUSH_ALL(match_fls_list, group_line->match_list_point[valid_index]);
                int other_match_num = 0;
                for (auto &raw_pt : group_line->match_list[valid_index]) {
                    auto &match_map = raw_pt->merge_match_map;
                    // match_frame_map[raw_pt->src->frame_id]++;
                    for (auto &it : match_map) {
                        if (it.first == group_line.get()) {
                            continue;
                        }
                        auto &line_match_list = it.first->match_list_point;
                        other_match_num += line_match_list[it.second->line_index].size();
                        // match_frame_map[it.second->src->frame_id]++;
                        VEC_PUSH_ALL(match_fls_list, line_match_list[it.second->line_index]);
                    }
                }
                double total_num = match_num + other_match_num;
                pt->score = match_num / (total_num + 1e-6);//1e-6防止除数为0

                // double match_frame_num = match_frame_map.size() > pt->match_frame_num ? 
                //     pt->match_frame_num : match_frame_map.size();
                // pt->score = match_frame_num / pt->match_frame_num;
                // if (total_num < min_match_size) {
                //     pt->filter_status = 2;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 2;
                //     }
                // }
                // if (pt->score < score_thres) {
                //     pt->filter_status = 3;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 3;
                //     }
                // }
                if (pt->invalid()) {
                    prev_fls = NULL;
                    continue;
                }
                if (prev_fls != NULL) {
                    prev_fls->next = pt.get();
                    pt->prev = prev_fls;
                    pt->dir = alg::get_dir(pt->pos, prev_fls->pos);
                    prev_fls->dir = pt->dir;
                    vote_yellow(session, prev_fls, match_fls_list);  // 投票计算prev_fls的颜色属性
                    vote_geo(session, prev_fls, match_fls_list); // 投票计算几何属性
                    vote_double(session, prev_fls, match_fls_list); // 投票计算是否是双线
                }
                prev_fls = pt.get();
                // DLOG_POINT(pt->pos, "merge_filter_lane[s={}, m1={}, m2={}, f={}]",
                //         pt->score, match_num, other_match_num, pt->filter_status);
            }
        }
        #endif
    }
    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::match_frame_line_lane(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneLineSampleGroupLine> > &trail_group, 
        LaneLineSampleTree &trail_tree,
        LaneLineSampleLine* tar_line) {
    double radius = FLAGS_merge_feature_search_radius; // 5
    double scope = FLAGS_merge_feature_scope; // 1
    double theta_thres = FLAGS_merge_feature_theta; // 20

    std::vector<LaneLineSample*> secs;
    UMAP<LaneLineSampleGroupLine*, int> cache_map;
    UMAP<LaneLineSample*, int> used_map;
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        auto &pt = tar_line->list[j];
        // session->debug_pos(pt->pos);
        secs.clear();
        used_map.clear();
        auto &match_map = pt->merge_match_map;
        trail_tree.search(pt->pos, radius, secs);
        Eigen::Vector3d v_pt = alg::get_vertical_pos(pt->pos, pt->dir, 50);
        Eigen::Vector3d cross_point = {0, 0, 0};
        for (int64_t k = 0; k < secs.size(); ++k) {
            auto &src_pt = secs[k];
            
            if (MAP_FIND(used_map, src_pt)) {
                continue;
            }
            used_map[src_pt] = 1;
            double dis = 0;
            if (src_pt->next == NULL) {
                continue;
            } else {
                if (!alg::get_cross_point_by_point(src_pt->pos, src_pt->next->pos,
                            pt->pos, v_pt, cross_point, false, 2, 1000)) {
                    continue;
                }
                dis = alg::calc_dis(pt->pos, cross_point);
            }
            if (dis > scope) { // tar的垂直方向向量 与 src方向向量的交点 与 当前 tar 点 在 1m范围内
                continue;
            }
            double theta = alg::calc_theta(src_pt->dir, pt->dir);
            if (theta > theta_thres) { // 两条线上的匹配点，对应的朝向 在 20 度范围内
                continue;
            }
            double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            if (s_dis > scope) { // tar 到 src 垂足距离 在 1m范围内
                continue;
            }
            match_map[src_pt->group_line] = src_pt;
            src_pt->group_line->match_list_point[src_pt->line_index].push_back(pt);
            if (MAP_FIND(cache_map, src_pt->group_line)) {
                continue;
            }
            cache_map[src_pt->group_line] = 1;
        }
    }
    using line_score = std::pair<LaneLineSampleGroupLine*, double>;
    std::vector<line_score> match_list;
    for (auto &it : cache_map) {
        double score = 0;
        if (!is_match_line_lane(session, it.first, tar_line, score)) {
            continue;
        }
        match_list.emplace_back(std::make_pair(it.first, score));
    }
    if (match_list.size() == 0) {
        gen_new_lane_group(session, trail_group, tar_line, trail_tree);
    } else {
        SORT(match_list, [](const line_score &l, const line_score &r) {
                return l.second > r.second;
                });
        auto &top_line = match_list[0].first;
        update_lane_group_line(session, top_line, tar_line, trail_tree);
    }
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::gen_new_lane_group(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneLineSampleGroupLine> > &trail_group, 
        LaneLineSampleLine* tar_line,
        LaneLineSampleTree &trail_tree) {
    auto new_line = std::make_shared<LaneLineSampleGroupLine>();
    new_line->id = tar_line->id;
    LaneLineSample* prev_pt = NULL;
    for (auto &pt : tar_line->list) {
        auto new_pt = std::make_shared<LaneLineSample>();
        new_pt->init(pt);
        new_pt->group_line = new_line.get();
        new_pt->line_index = new_line->list.size();
        new_pt->line_id = new_line->id;
        // new_pt->boundary_type = tar_line->boundary_type; // TODO：qzc，暂时没用到，所以没有添加
        new_line->list.push_back(new_pt);
        new_line->match_list.push_back({pt});
        new_line->match_list_point.resize(new_line->match_list.size());
        new_line->match_score.push_back(pt->src->score);
        if (prev_pt != NULL) {
            prev_pt->next = new_pt.get();
            new_pt->prev = prev_pt;
        }
        trail_tree.insert(new_pt->pos, new_pt.get());
        prev_pt = new_pt.get();
    }
    new_line->merge_lines.push_back(tar_line->src);
    new_line->cur_line_id = tar_line->cur_line_id;
    new_line->boundary_type = tar_line->boundary_type;
    new_line->matched_line_ids.insert(tar_line->matched_line_ids.begin(), tar_line->matched_line_ids.end());
    trail_group.push_back(new_line);
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::update_lane_group_line(RoadModelSessionData* session,
        LaneLineSampleGroupLine* src_line, 
        LaneLineSampleLine* tar_line,
        LaneLineSampleTree &trail_tree) {
    double scope = FLAGS_merge_feature_scope;
    int64_t max_tar_valid = 0;
    int64_t max_src_valid = 0;
    int64_t min_tar_valid = INT_MAX;
    int64_t min_src_valid = INT_MAX;
    for (int64_t i = 0; i < tar_line->list.size(); ++i) {
        auto &pt = tar_line->list[i];
        auto &map = pt->merge_match_map;
        if (MAP_NOT_FIND(map, src_line)) {
            continue;
        }
        auto &src_pt = map[src_line];
        int64_t src_index = src_pt->line_index;
        double total_score = src_line->match_score[src_index];
        double v_dis_1 = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir, true);
        // double v_dis_2 = -alg::calc_vertical_dis(src_pt->pos, pt->pos, pt->dir, true);
        double v_dis = v_dis_1;
        // if (fabs(v_dis_1) > fabs(v_dis_2)) {
        //     v_dis = v_dis_2;
        // }
        if (v_dis > scope) {
            continue;
        }
        total_score  += pt->src->score;
        if(total_score < 1e-6) {
             LOG_ERROR("lane line total_score: {}, pt->src->score:{}]", total_score, pt->src->score);
             total_score += 1e-6;
        }
        double opt_dis = pt->src->score / total_score * v_dis;// 1e-6 防止分母为0
        Eigen::Vector3d opt_pos = alg::get_vertical_pos(src_pt->pos, src_pt->dir, opt_dis);
        session->debug_pos(opt_pos);
        src_pt->pos = opt_pos;
        trail_tree.insert(src_pt->pos, src_pt);
        src_line->match_list[src_index].push_back(pt);
        src_line->match_score[src_index] += pt->src->score;
        max_tar_valid = std::max(max_tar_valid, i);
        min_tar_valid = std::min(min_tar_valid, i);
        max_src_valid = std::max(max_src_valid, src_index);
        min_src_valid = std::min(min_src_valid, src_index);
    }
    // 深坑，size 返回的是Uint 不能减
    int64_t end_index = (tar_line->list.size() >= 3) ? (tar_line->list.size() - 3) : 0;
    if (max_tar_valid <= end_index) {
        auto &next_tar_pt = tar_line->list[max_tar_valid + 1];
        auto prev_pt = src_line->list.back().get();
        if (alg::judge_front(next_tar_pt->pos, prev_pt->pos, prev_pt->dir)) {
            for (int i = max_tar_valid + 2; i < tar_line->list.size(); ++i) {
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<LaneLineSample>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_index = src_line->list.size();
                new_pt->line_id = src_line->id;
                src_line->list.push_back(new_pt);
                src_line->match_list.push_back({pt});
                src_line->match_score.push_back(pt->src->score);
                src_line->match_list_point.resize(src_line->match_list.size());
                if (prev_pt != NULL) {
                    prev_pt->next = new_pt.get();
                    new_pt->prev = prev_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                prev_pt = new_pt.get();
            }
        }
    }
    if (min_tar_valid >= 2) {
        auto &prev_tar_pt = tar_line->list[min_tar_valid - 1];
        auto next_pt = src_line->list.front().get();
        if (!alg::judge_front(prev_tar_pt->pos, next_pt->pos, next_pt->dir)) {
            for (int i = min_tar_valid - 2; i >= 0; --i) {
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<LaneLineSample>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                VEC_INSERT(src_line->list, new_pt);
                VEC_INSERT(src_line->match_list, {pt});
                VEC_INSERT(src_line->match_score, pt->src->score);
                src_line->match_list_point.insert(
                        src_line->match_list_point.begin(), std::vector<LaneLineSample*>());
                if (next_pt != NULL) {
                    next_pt->prev = new_pt.get();
                    new_pt->next = next_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                next_pt = new_pt.get();
            }
            for (int64_t i = 0; i < src_line->list.size(); ++i) {
                auto &src_pt = src_line->list[i];
                src_pt->line_index = i;
            }
        }
    }
    src_line->merge_lines.push_back(tar_line->src);
    return fsdmap::SUCC;
}

bool RoadModelProcMergeFeature::is_match_line_lane(RoadModelSessionData* session,
        LaneLineSampleGroupLine* src, LaneLineSampleLine* tar, double &score) {
    if(src->matched_line_ids.count(tar->cur_line_id) > 0) {
        score = 1;
        return true;
    }

    double rate_thres = FLAGS_merge_feature_match_rate_threshold;

    int total_num = tar->list.size();
    int total_num_origin = tar->list.size();
    int match_num = 0;
    int64_t tar_max_index = 0;
    int64_t src_max_index = 0;
    int64_t tar_min_index = INT_MAX;
    int64_t src_min_index = INT_MAX;
    for (int64_t i = 0; i < total_num; ++i) {
        auto &pt = tar->list[i];
        auto &map = pt->merge_match_map;
        if (map.find(src) != map.end()) {
            if (map[src] != NULL) {
               ++match_num;
               auto &src_pt = map[src];
               src_max_index = src_pt->line_index;
               tar_max_index = i;
               tar_min_index = std::min(tar_min_index, i);
               src_min_index = std::min(src_min_index, src_pt->line_index);
            }
        }
    }
    if (src_min_index == 0) {
        total_num -= tar_min_index;
    }
    if (src_max_index == src->list.size() - 2) {
        total_num -= (tar->list.size() - tar_max_index - 1);
    }
    score = (double) match_num / total_num + 1e-6;
    if (match_num == 1 && total_num_origin > 1) {
        score = (double) match_num / total_num_origin + 1e-6;
    }
    // if (score > rate_thres) {
    //     int a = 1;
    // }
    // LOG_DEBUG("merge_lane[score={:.2f}, match={}, total={}, "
    //         "tar_max={}, tar_min={}, src_max={}, src_min={}]",
    //         score, match_num, total_num, 
    //         tar_max_index, tar_min_index, src_max_index, src_min_index);
    if (score > rate_thres) {
        return true;
    }
    return false;
}

void RoadModelProcMergeFeature::merge_boundary_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process) {
    bool debug_log = false;
    // bool debug_log = true;
    int line_size = trail_ptr->boundary_line_group_list.size();
    std::vector<alg::linestring_t> all_curve(line_size);
    std::vector<std::vector<Eigen::Vector3d>> all_curve_points_dir(line_size);
    for (int64_t i = 0; i < line_size; ++i) {
        auto &group_line = trail_ptr->boundary_line_group_list[i];
        group_line->cur_line_id = i;
        
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if(pt == nullptr) {
                LOG_ERROR("!!!!!!!!!!!!not normal point!!!!!!!");
                continue;
            }
            all_curve[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
            all_curve_points_dir[i].push_back(pt->dir);
        }
    }

    std::vector<std::vector<alg::OverlapInfoLineString>> matched_list(line_size); // 每条线段与其他线段的匹配信息
    std::map<int, std::unordered_set<int>> common_prcessed_line; // 当前id，匹配上的line
    for(int i = 0; i < line_size; i++) {
        for (int j = i+1; j < line_size; j++) {
            alg::OverlapInfoLineString curve_overlap;
            curve_overlap.l1_id = i;
            curve_overlap.l2_id = j;
            // lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.05);
            lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.1);
            if (curve_overlap.is_overlap) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                auto &group_line_i = trail_ptr->boundary_line_group_list[i];
                group_line_i->matched_line_ids.insert(j);
                auto &group_line_j = trail_ptr->boundary_line_group_list[j];
                group_line_j->matched_line_ids.insert(i);

                 if (debug_log && reverse_process == false) {
                    std::cout << "[boundary] 1 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 << std::endl;
                }
                #if 0
                // case2: 有重叠 & 方式不一致，但是是180度, 则先进行反转再进行合并
                // TODO: qzc 需要考虑有多条线重叠的时候，应该以最长的方向为准
                if (curve_overlap.is_need_reverse_l1) {

                    // // for debug
                    // auto first_before = trail_ptr->boundary_line_group_list[i]->list[0]->pos;
                    // int p_size = trail_ptr->boundary_line_group_list[i]->list.size();
                    // auto last_before = trail_ptr->boundary_line_group_list[i]->list[p_size-1]->pos;

                    reverse(trail_ptr->boundary_line_group_list[i]->list.begin(), trail_ptr->boundary_line_group_list[i]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[i]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;

                        fls->set_prev(prev.get());
                        prev = fls;
                    }

                    // // for debug
                    // auto frist_after = trail_ptr->boundary_line_group_list[i]->list[0]->pos;
                    // auto last_after = trail_ptr->boundary_line_group_list[i]->list[p_size-1]->pos;
                    // std::cout << "l1: first_before: " <<  first_before.transpose() << " last_before: " << last_before.transpose() << std::endl;
                    // std::cout << "l1: frist_after: " <<  frist_after.transpose() << " last_after " << last_after.transpose() << std::endl;
                } else if (curve_overlap.is_need_reverse_l2) {

                    // // for debug
                    // auto first_before = trail_ptr->boundary_line_group_list[j]->list[0]->pos;
                    // int p_size = trail_ptr->boundary_line_group_list[j]->list.size();
                    // auto last_before = trail_ptr->boundary_line_group_list[j]->list[p_size-1]->pos;

                    reverse(trail_ptr->boundary_line_group_list[j]->list.begin(), trail_ptr->boundary_line_group_list[j]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[j]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                        fls->set_prev(prev.get());
                        prev = fls;
                    }

                    // // for debug
                    // auto frist_after = trail_ptr->boundary_line_group_list[j]->list[0]->pos;
                    // auto last_after = trail_ptr->boundary_line_group_list[j]->list[p_size-1]->pos;
                    // std::cout << "l2: first_before: " <<  first_before.transpose() << " last_before: " << last_before.transpose() << std::endl;
                    // std::cout << "l2: frist_after: " <<  frist_after.transpose() << " last_after " << last_after.transpose() << std::endl;
                }
                #endif
                // case1: 有重叠 & 方式一致, 进行合并
            } else if(curve_overlap.is_nearby) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                // auto &group_line_i = trail_ptr->boundary_line_group_list[i];
                // group_line_i->matched_line_ids.insert(j);
                // auto &group_line_j = trail_ptr->boundary_line_group_list[j];
                // group_line_j->matched_line_ids.insert(i);
                
                 if (debug_log && reverse_process == false) {
                    std::cout << "[boundary] 2 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 << std::endl;
                }

                #if 0
                // case3: 无重叠 & 方式一致（首末尾）, 进行合并
                if (curve_overlap.is_need_reverse_l1) {
                    reverse(trail_ptr->boundary_line_group_list[i]->list.begin(), trail_ptr->boundary_line_group_list[i]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[i]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                        fls->set_prev(prev.get());
                        prev = fls;
                    }
                } else if (curve_overlap.is_need_reverse_l2) {
                    reverse(trail_ptr->boundary_line_group_list[j]->list.begin(), trail_ptr->boundary_line_group_list[j]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[j]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                        fls->set_prev(prev.get());
                        prev = fls;
                    }
                }
                #endif

            }
            // case4: 无重叠 & 方式不一致，不处理
        }
    }

    for (auto& matched_info : matched_list) {
        std::sort(matched_info.begin(), matched_info.end(), [](const alg::OverlapInfoLineString& a, const alg::OverlapInfoLineString& b) {
            return a.l2_length > b.l2_length;
        });
    }

    std::sort(matched_list.begin(), matched_list.end(), [](const std::vector<alg::OverlapInfoLineString>& a, const std::vector<alg::OverlapInfoLineString>& b) {
        if (a.size() != b.size()) {
            return a.size() > b.size();
        } else if(a.size() == 0 && b.size() == 0) {
            return false;
        }
        
        return a[0].l1_length > b[0].l1_length;
    });

    std::vector<bool> has_processed(line_size, false);
    // 注意：matched_list 已经不是按照，line_size 的顺序了
    for (const auto& matched_info_i : matched_list) {
        int matched_num = matched_info_i.size();
        if (matched_num == 0) {
            continue;
        }

        // 当自己长度比较短，且需要反转，那么除了0以外的其他line，如果需要反转，则需要和以 need_reverse 为准
        auto const& curve_overlap_0 = matched_info_i[0];
        int id1 = curve_overlap_0.l1_id;
        bool need_reverse = curve_overlap_0.is_need_reverse_l1;

        // 必须要是还有未处理过的match点，再进行融合， TODO：qzc 可能还存在一种情况是，上次已经反转了一次，当前还有，那么可能会导致错误的反转，可能需要同步改掉matched_list中的状态才行
        int valid_match_size = 0;
        std::unordered_set<int> other_line_prcessed;
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            int id2 = curve_overlap_j.l2_id;
            if (has_processed[id2] == true) {
                // 如果已经处理过的id2，对应的匹配信息中，有id1，那么就不能重复处理，需要过滤掉
                if (common_prcessed_line.find(id2) != common_prcessed_line.end() && common_prcessed_line[id2].find(id1) != common_prcessed_line[id2].end()) {
                    other_line_prcessed.insert(common_prcessed_line[id2].begin(), common_prcessed_line[id2].end());
                }
                
                continue;
            }
            // 如果之前处理过，那么直接跳过
            if (other_line_prcessed.find(id2) != other_line_prcessed.end()) {
                continue;
            }

            valid_match_size++;
        }

        if (need_reverse && valid_match_size > 0) {
            reverse(trail_ptr->boundary_line_group_list[id1]->list.begin(), trail_ptr->boundary_line_group_list[id1]->list.end());
            std::shared_ptr<BoundaryFeature> prev = NULL;
            for (auto &fls : trail_ptr->boundary_line_group_list[id1]->list) {
                if (prev == NULL) {
                    prev = fls;
                    continue;
                }
                fls->dir = alg::get_dir(fls->pos, prev->pos);
                prev->dir = fls->dir;
                fls->set_prev(prev.get());
                prev = fls;
            }
             if (debug_log && reverse_process == false) {
                std::cout << "[boundary] 0#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << curve_overlap_0.l2_id << std::endl;
            }
        }

        
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            
            int id2 = curve_overlap_j.l2_id;
             if (debug_log && reverse_process == false) {
                std::cout << "[boundary] 1#need_reverse: " << need_reverse << " id1 : " << id1 << " id2: " << id2  << " matched_num: " << matched_num  << " has_processed[id2]:" << has_processed[id2] << std::endl; 
            }
            if (has_processed[id2] == true) {
                continue;
            }

            if (need_reverse == false) {
                // 如果自己不需要反转，其他和自己不相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 != curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->boundary_line_group_list[id2]->list.begin(), trail_ptr->boundary_line_group_list[id2]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                        fls->set_prev(prev.get());
                        prev = fls;
                    }
                     if (debug_log && reverse_process == false) {
                        std::cout << "[boundary] 2#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            } else {
                // 如果自己需要反转，其他和自己相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 == curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->boundary_line_group_list[id2]->list.begin(), trail_ptr->boundary_line_group_list[id2]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                        fls->set_prev(prev.get());
                        prev = fls;
                    }
                     if (debug_log && reverse_process == false) {
                        std::cout << "[boundary] 3#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            }

            // // case1: 有重叠 & 方向一致, 进行合并
            // // case2: 有重叠 & 方向不一致，但是是180度, 则先进行反转再进行合并
            // if (curve_overlap_j.is_overlap) {
            // } 
            // // case3: 无重叠 & 方向一致（首末尾）, 进行合并
            // if (curve_overlap_j.is_nearby) {
            // }
            // case4: 无重叠 & 方向不一致，不处理
        }

        has_processed[id1] = true;
    }
    

    // merge multi group line
    // std::vector<std::shared_ptr<BoundaryLine>> tmp_boundary_line_ptr;
    // UMAP<std::string, std::vector<BoundaryLine*>> tmp_bev_frame_boundary_line; 
    session->boundary_line_ptr_for_merge.clear();
    session->bev_frame_boundary_line.clear();
    
    int lane_size = trail_ptr->boundary_line_group_list.size();
    if (reverse_process) {
        for (int64_t i = lane_size-1; i >= 0; --i) {
            auto &group_line = trail_ptr->boundary_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
            new_line->boundary_type = group_line->boundary_type;
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::shared_ptr<BoundaryGroupLine> group_line;
            //      std::vector<std::shared_ptr<BoundaryFeature>> list;  // 该曲线上所有的点
            // prev = NULL;
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                if(pt->invalid()) {
                    continue;
                }

                auto new_node = session->add_ptr(session->boundary_feature_ptr);
                // new_node = pt;
                // new_node->pose = pt->pos;
                new_node->init(pt.get());

                if (prev != NULL) {
                    new_node->set_prev(prev);
                }
                prev = new_node.get();

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() <= 2) {
                continue;
            }

            session->bev_frame_boundary_line[""].push_back(new_line.get());
        }
    } else {
        for (int64_t i = 0; i < lane_size; ++i) {
            auto &group_line = trail_ptr->boundary_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
            new_line->boundary_type = group_line->boundary_type;
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::shared_ptr<BoundaryGroupLine> group_line;
            //      std::vector<std::shared_ptr<BoundaryFeature>> list;  // 该曲线上所有的点
            // prev = NULL;
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }
                if(pt->invalid()) {
                    continue;
                }

                auto new_node = session->add_ptr(session->boundary_feature_ptr);
                // new_node = pt;
                // new_node->pose = pt->pos;
                new_node->init(pt.get());

                if (prev != NULL) {
                    new_node->set_prev(prev);
                }
                prev = new_node.get();

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() <= 2) {
                continue;
            }

            session->bev_frame_boundary_line[""].push_back(new_line.get());
        }
    }
    // session->bev_frame_boundary_line = tmp_bev_frame_boundary_line;

    // 2. 重新调用 merge single 操作
    merge_single_boundary_by_trail(session, trail_ptr);
}

int RoadModelProcMergeFeature::merge_single_boundary_by_trail(
        RoadModelSessionData* session, KeyPoseLine* trail) {
    int min_size = FLAGS_merge_feature_proc_min_size;
    int min_match_size = FLAGS_merge_feature_proc_min_match_size;
    double score_thres = FLAGS_merge_feature_proc_score_threshold;
    double scope = FLAGS_merge_feature_scope;

    if (trail->list.size() < min_size) {
        return fsdmap::FAIL;
    }

    trail->boundary_line_group_list.reserve(trail->list.size());

    int times = 0;
    while (times < 2) {
        ++times;
        BoundaryFeatureTree trail_tree;
        trail->boundary_line_group_list.clear();
        int64_t raw_line_size = 0;
        // 遍历这整条轨迹上的轨迹点
        // for (int64_t i = 0; i < trail->list.size(); ++i) {
        //     auto &poss = trail->list[i];
        //     // if (poss->frame_id == "6231672047094_1672018299880000") {
        //     //     int a = 1;
        //     // }
        //     LOG_ERROR("session->bev_frame_boundary_line size:{} poss->frame_id :{}", session->bev_frame_boundary_line.size(),
        //      poss->frame_id);
        //     if (MAP_NOT_FIND(session->bev_frame_boundary_line, poss->frame_id)) {          
        //         continue;
        //     }
        
        int debug_index = 0;

        //     auto &ins_vec = session->bev_frame_boundary_line[poss->frame_id]; // 被当前帧pose看到的boundary
        for (const auto& ins_pair : session->bev_frame_boundary_line) { // 每一条边界线，有很多重复
            const auto& ins_vec = ins_pair.second;
            if (ins_vec.empty()) {
                continue;
            }
            for (auto &ins : ins_vec) { // 每一条线
                // if (ins->boundary_type != LaneType::ISLAND_RB) {
                //     continue;
                // }
                
                ins->sub_line.clear();
                BoundaryFeature* prev = NULL;
                auto new_line = session->add_ptr(ins->sub_line);
                new_line->list.reserve(ins->list.size());
                new_line->src = ins;
                int64_t sub_line_index = 0;
                new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);

                // add by qzc
                new_line->boundary_type = ins->boundary_type;
                new_line->cur_line_id = ins->cur_line_id;
                new_line->matched_line_ids = ins->matched_line_ids;

                for (auto &pt : ins->list) {
                    session->debug_pos(pt->pos);
                    // 三角岛不需要打断
                    if (new_line->boundary_type != LaneType::ISLAND_RB && pt->invalid()) { // 该点是异常点（比如是一个突变点，方向想来那个发生了突变），在前面步骤的sample_line()函数里把无效点filter_status置为2
                        if (new_line->list.size() > 1) { // pt这个点出现在了曲线的中间，那么将ins这条曲线从pt点这个位置切开，重启另外一条line放入ins->sub_line
                            new_line = session->add_ptr(ins->sub_line);
                            new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);
                            new_line->src = ins;
                        } else { // pt突变点出现在曲线开头，只需要清空下new_line，后面继续往这里面填
                            new_line->list.clear();
                        }
                        prev = NULL;
                        continue;
                    }
                    new_line->list.push_back(pt);
                    pt->merge_match_map.clear();
                    if (prev != NULL) {
                        pt->set_prev(prev);
                    }
                    prev = pt;
                }
                for (auto &sub_line : ins->sub_line) {
                    if (sub_line->list.size() <= 1) {
                        continue;
                    }
                    // 找到 sub_line 的匹配曲线，把他连接到这条曲线上
                    match_frame_line_boundary(session, trail->boundary_line_group_list, trail_tree, sub_line.get());
                }

                auto &poss = trail->list[0];
                debug_index += 1;
                std::string id_index(std::to_string(debug_index)+"_"+ins->id);
                save_frame_log_boundary(session, trail, poss, times, id_index);
            }
            raw_line_size += ins_vec.size();

        }
        LOG_INFO("merge_boundary_final[id={}, p_size={}, times={}, size={}, raw_size={}]", 
                (int64_t)trail, trail->list.size(), times, 
                trail->boundary_line_group_list.size(), raw_line_size);
        // 过滤
        #if 1
        double length_thres = FLAGS_merge_feature_single_boundary_length_thres;
        for (int64_t i = 0; i < trail->boundary_line_group_list.size(); ++i) {
            auto &group_line = trail->boundary_line_group_list[i];
            // add by qzc: 三角岛不处理
            if (group_line->boundary_type == LaneType::ISLAND_RB) {
                continue;
            }
            
            double total_length = 0;
            BoundaryFeature* prev_pt = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                if (j == group_line->list.size() - 1) {
                    valid_index = j - 1;
                }
                auto &pt = group_line->list[j];
                // session->debug_pos(pt->pos);
                // int match_num = group_line->match_list[j].size();
                int match_num = group_line->match_list_point[valid_index].size();
                // UMAP<std::string, int> match_frame_map;

                double total_score = group_line->match_score[j];
                for (auto &raw_pt : group_line->match_list_point[valid_index]) {
                    // match_frame_map[raw_pt->frame_id]++;
                    // 合并其他点
                    double v_dis = alg::calc_vertical_dis(raw_pt->pos, pt->pos, pt->dir, true); // 计算点raw_pt->pos到直线pt->dir的距离
                    if (v_dis > scope) { // 小于 scope 的匹配点才进行合并
                        continue;
                    }
                    double opt_dis = raw_pt->score / (total_score + raw_pt->score /*+ 1e-6*/) * v_dis;// 1e-6 防止分母为0
                    Eigen::Vector3d opt_pos = alg::get_vertical_pos(pt->pos, pt->dir, opt_dis);
                    pt->pos = opt_pos;
                    total_score += raw_pt->score;
                }
                int other_match_num = 0;
                for (auto &raw_pt : group_line->match_list[valid_index]) {
                    auto &match_map = raw_pt->merge_match_map; // 与pt点匹配的点的对应其他的匹配关系
                    // match_frame_map[raw_pt->frame_id]++;
                    for (auto &it : match_map) {
                        if (it.first == group_line.get()) {
                            continue;
                        }
                        auto &line_match_list = it.first->match_list_point;
                        other_match_num += line_match_list[it.second->line_index].size();
                        // match_frame_map[raw_pt->frame_id]++;
                    }
                }
                double total_num = match_num + other_match_num;
                // pt->score = match_num / (total_num + 1e-6); // 1e-6 防止分母为0

                // pt->score = (double)match_frame_map.size() / pt->match_frame_num;

                // if (total_num < min_match_size) { // 匹配点少于1个，滤除，与该点匹配上的其他曲线上的点也删除
                //     pt->filter_status = 2;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 2;
                //     }
                // }

                // if (pt->score < score_thres) { //可理解为共视率
                //     pt->filter_status = 3;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 3;
                //     }
                // }
                if (pt->invalid()) {
                    continue;
                }
                if (prev_pt != NULL) {
                    total_length += alg::calc_dis(prev_pt->pos, pt->pos);
                }
                // 用group_line->match_list_point[valid_index]中出现频率最高的类别作为pt的类别

                vote_type(session, pt.get(), group_line->match_list_point[valid_index]);
                // DLOG_POINT(pt->pos, "merge_filter_boundary[s={}, m1={}, m2={}, f={}]",
                //         pt->score, match_num, other_match_num, pt->filter_status);
                prev_pt = pt.get();
            }
            // 长度小于5m的也滤除
            if (total_length < length_thres) {
                for (auto &pt : group_line->list) {
                    pt->filter_status = 4;
                }
            }
        }
        #endif

    }
    
    return fsdmap::SUCC;
}

// trail_tree: 存储了之前遍历到的单条轨迹上的轨迹点看到的每段曲线信息
// 在该趟路线中寻找与tar_line有80%的点都能匹配上的去遍历过的曲线，选出匹配度最高的那条曲线，
// 把tar_line上的点扩展到这条曲线上
int RoadModelProcMergeFeature::match_frame_line_boundary(RoadModelSessionData* session,
        std::vector<std::shared_ptr<BoundaryGroupLine> > &trail_group, 
        BoundaryFeatureTree &trail_tree,
        BoundaryLine* tar_line) {
    double radius = FLAGS_merge_feature_search_radius; // 5
    double scope = FLAGS_merge_feature_scope; // 1
    double theta_thres = FLAGS_merge_feature_theta; // 20

    std::vector<BoundaryFeature*> secs;
    UMAP<BoundaryGroupLine*, int> cache_map; // 这趟路线中与tar_line有点匹配上的历史曲线
    UMAP<BoundaryFeature*, int> used_map;
    // 遍历tar_line上的每个点pt，在历史曲线树trail_tree中搜索最近邻的点，
    // 遍历这些最近邻点，判断是否与pt点满足距离要求、以及方向一致的匹配点，
    // 匹配上的话就更新pt点以及最近邻点的匹配关系
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        // add by qzc: 三角岛不处理
        if (tar_line->boundary_type == LaneType::ISLAND_RB) {
            continue;
        }
        
        auto &pt = tar_line->list[j];
        // session->debug_pos(pt->pos);
        secs.clear();
        used_map.clear();
        // 在单轨迹车道线等特征融合过程中，与该点满足距离要求，以及方向一致的匹配点，以及该匹配点对应所属的曲线 BoundaryGroupLine，
        // <该匹配点对应所属的曲线，与该点满足距离要求、以及方向一致的匹配点>
        auto &match_map = pt->merge_match_map;
        trail_tree.search(pt->pos, radius, secs); // 搜索 5m 内的点
        Eigen::Vector3d v_pt = alg::get_vertical_pos(pt->pos, pt->dir, 50, true); //计算dir垂线方向上离pos距离是50m的点
        Eigen::Vector3d cross_point = {0, 0, 0};
        for (int64_t k = 0; k < secs.size(); ++k) {
            auto &src_pt = secs[k];
            // add by qzc: 三角岛不处理
            if (src_pt->boundary_type == LaneType::ISLAND_RB) {
                continue;
            }
            
            if (MAP_FIND(used_map, src_pt)) {
                continue;
            }
            used_map[src_pt] = 1;
            double dis = 0;
            if (src_pt->next == NULL) {
                continue;
                // double h_dis = alg::calc_hori_dis(pt->pos, src_pt->pos, src_pt->dir, true);
                // if (h_dis < 0 && src_pt->next == NULL) {
                //     continue;
                // } 
                // if (h_dis > 0 && src_pt->prev == NULL) {
                //     continue;
                // }
                // dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            } else {
                if (!alg::get_cross_point_by_point(src_pt->pos, src_pt->next->pos,
                            pt->pos, v_pt, cross_point, false, 2, 1000)) { // 计算交点【通过pt的垂线与（src_pt, src_pt->next）线段的交点】
                    continue;
                }
                dis = alg::calc_dis(pt->pos, cross_point);
            }
            if (dis > scope) { // 1. 交点在 1 m 内 
                // match_map[src_pt->group_line] = NULL;
                continue;
            }
            double theta = alg::calc_theta(src_pt->dir, pt->dir); //2.计算方向一致性
            if (theta > theta_thres) { // 20
                continue;
            }
            // 3.pt->pos 到直线 src_pt->dir 的垂直距离
            double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            if (s_dis > scope) {
                continue;
            }
            match_map[src_pt->group_line] = src_pt;
            // src_pt->group_line: src_pt点所属的曲线
            // src_pt->line_index 在所属曲线上的index
            // src_pt->group_line->match_list_point ：<src_pt在所属曲线上的Index，与src距离方向都匹配的其他曲线上的点（应该用于合并曲线）>
            src_pt->group_line->match_list_point[src_pt->line_index].push_back(pt);
            if (MAP_FIND(cache_map, src_pt->group_line)) {
                continue;
            }
            cache_map[src_pt->group_line] = 1; // 记录 该src_pt 所属的 group_line 已经 cache ， 后面会用来和 tar_line 匹配，获取匹配度最高的线进行融合
        }
    }
    using line_score = std::pair<BoundaryGroupLine*, double>;
    std::vector<line_score> match_list; // 记录cache_map中与tar_line有80%的点都能匹配上的曲线
    for (auto &it : cache_map) {
        double score = 0;
        // 判断it.first与tar_line是否是同一条曲线
        if (!is_match_line_boundary(session, it.first, tar_line, score)) {
            continue;
        }
        // is_match_line_boundary(session, it.first, tar_line, score);

        match_list.emplace_back(std::make_pair(it.first, score));
    }
    if (match_list.size() == 0) { // 这趟路线中没有与tar_line有点匹配上的历史曲线
        // 通过tar_line产生新的线放入trail_group，并把tar_line中的点放入trail_tree
        gen_new_boundary_group(session, trail_group, tar_line, trail_tree);
    } else {
        SORT(match_list, [](const line_score &l, const line_score &r) {
                return l.second > r.second;
                });
        auto &top_line = match_list[0].first;
        //选出匹配度最高的一条曲线，把tar_line上的点扩展到这条曲线上，并更新top_line上匹配点的位置 
        update_boundary_group_line(session, top_line, tar_line, trail_tree);
    }
    return fsdmap::SUCC;
}

// 通过tar_line产生新的线放入trail_group，并把tar_line中的点放入trail_tree
int RoadModelProcMergeFeature::gen_new_boundary_group(RoadModelSessionData* session,
        std::vector<std::shared_ptr<BoundaryGroupLine> > &trail_group, BoundaryLine* tar_line,
        BoundaryFeatureTree &trail_tree) {
    auto new_line = std::make_shared<BoundaryGroupLine>();
    new_line->id = tar_line->id;
    BoundaryFeature* prev_pt = NULL;
    for (auto &pt : tar_line->list) {
        auto new_pt = std::make_shared<BoundaryFeature>();
        new_pt->boundary_type = tar_line->boundary_type;
        new_pt->init(pt);
        new_pt->group_line = new_line.get();
        new_pt->line_index = new_line->list.size();
        new_pt->line_id = new_line->id;
        new_line->list.push_back(new_pt);
        new_line->match_list.push_back({pt});
        new_line->match_score.push_back(pt->score);
        new_line->match_list_point.resize(new_line->match_list.size());
        if (prev_pt != NULL) {
            prev_pt->next = new_pt.get();
            new_pt->prev = prev_pt;
        }
        trail_tree.insert(new_pt->pos, new_pt.get());
        prev_pt = new_pt.get();
    }     
    new_line->merge_lines.push_back(tar_line->src); // tar_line 在这里其实是 sub_line, src 其实就是 bev_frame_boundary_line 中的原始线段
    new_line->cur_line_id = tar_line->cur_line_id;
    new_line->boundary_type = tar_line->boundary_type;
    new_line->matched_line_ids.insert(tar_line->matched_line_ids.begin(), tar_line->matched_line_ids.end()); 
    trail_group.push_back(new_line);
    return fsdmap::SUCC;
}

// 遍历tar_line中的每个点pt，在src_line寻找与他匹配上的点src_pt，如果计算点pt到直线src_pt->dir的距离dist小于1m，
// 那么更新src_pt的位置，更新原则按照pt->score（点的位置可信度）的占比，以及dist重新计算。
// 如果tar_line中尾部还剩下大于3个点没有在src_line找到匹配点，
// 同时满足tar_line中max_tar_valid + 1出的点在src_line最后一个点的前方，那么把
// tar_line索引从max_tar_valid + 2开始往后的点都归入src_line，拼接起来。
// 如果tar_line中头部还有多于2个点没有在src_line找到匹配点，
// 同时满足src_line第一个点在tar_line中min_tar_valid - 1处的点的前方，那么把
// tar_line索引从0开始到min_tar_valid - 2的点都归入src_line，拼接起来
int RoadModelProcMergeFeature::update_boundary_group_line(RoadModelSessionData* session,
        BoundaryGroupLine* src_line, BoundaryLine* tar_line,
        BoundaryFeatureTree &trail_tree) {
    double scope = FLAGS_merge_feature_scope; // 1m
    int64_t max_tar_valid = 0;
    int64_t max_src_valid = 0;
    int64_t min_tar_valid = INT_MAX;
    int64_t min_src_valid = INT_MAX;
    for (int64_t i = 0; i < tar_line->list.size(); ++i) {
        auto &pt = tar_line->list[i];
        auto &map = pt->merge_match_map;
        if (MAP_NOT_FIND(map, src_line)) { // tar_line上的该点没有与src_line匹配上
            continue;
        }
        auto &src_pt = map[src_line];
        int64_t src_index = src_pt->line_index;
        double total_score = src_line->match_score[src_index];
        // 计算点pt->pos到直线src_pt->dir的距离
        double v_dis_1 = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir, true);
        // double v_dis_2 = -alg::calc_vertical_dis(src_pt->pos, pt->pos, pt->dir, true);
        double v_dis = v_dis_1;
        // if (fabs(v_dis_1) > fabs(v_dis_2)) {
        //     v_dis = v_dis_2;
        // }
        if (v_dis > scope) {
            continue;
        }
        // 按照pt->score（点的位置可信度）的占比，重新计算src_pt->pos的位置
        total_score  += pt->score;
        if(total_score < 1e-6) {
             LOG_ERROR("lane boundary total_score: {}, pt->src->score:{}]", total_score, pt->score);
             total_score += 1e-6;
        }
        double opt_dis = pt->score / total_score * v_dis;// 1e-6 防止分母为0

        Eigen::Vector3d opt_pos = alg::get_vertical_pos(src_pt->pos, src_pt->dir, opt_dis);
        session->debug_pos(opt_pos);
        src_pt->pos = opt_pos;
        trail_tree.insert(src_pt->pos, src_pt); //更新完位置的点放入tree
        src_line->match_list[src_index].push_back(pt);
        src_line->match_score[src_index] += pt->score; // 又新增了一个匹配点，加上匹配点的score
        max_tar_valid = std::max(max_tar_valid, i);
        min_tar_valid = std::min(min_tar_valid, i);
        max_src_valid = std::max(max_src_valid, src_index);
        min_src_valid = std::min(min_src_valid, src_index);
    }
    // 深坑，size 返回的是Uint 不能减
    int64_t end_index = (tar_line->list.size() >= 3) ? (tar_line->list.size() - 3) : 0;
    // 如果tar_line中尾部还剩下大于3个点没有在src_line找到匹配点，
    // 同时满足tar_line中max_tar_valid + 1出的点在src_line最后一个点的前方，那么把
    // tar_line索引从max_tar_valid + 2开始往后的点都归入src_line，拼接起来
    if (max_tar_valid <= end_index) {
        auto &next_tar_pt = tar_line->list[max_tar_valid + 1];
        auto prev_pt = src_line->list.back().get();
        // 判断next_tar_pt->pos是否在prev_pt->pos的前方
        if (alg::judge_front(next_tar_pt->pos, prev_pt->pos, prev_pt->dir)) {
            // for (int i = max_tar_valid + 1; i < tar_line->list.size(); ++i) { // TODO:qzc 为什么不是+1，而是+2？
            for (int i = max_tar_valid + 2; i < tar_line->list.size(); ++i) { // TODO:qzc 为什么不是+1，而是+2？
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<BoundaryFeature>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                new_pt->line_index = src_line->list.size();
                src_line->list.push_back(new_pt);
                src_line->match_list.push_back({pt});
                src_line->match_score.push_back(pt->score);
                src_line->match_list_point.resize(src_line->match_list.size());
                if (prev_pt != NULL) {
                    prev_pt->next = new_pt.get();
                    new_pt->prev = prev_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                prev_pt = new_pt.get();
            }
        }
    }

    // 如果tar_line中头部还有多于2个点没有在src_line找到匹配点，
    // 同时满足src_line第一个点在tar_line中min_tar_valid - 1处的点的前方，那么把
    // tar_line索引从0开始到min_tar_valid - 2的点都归入src_line，拼接起来
    if (min_tar_valid >= 2) {
        auto &prev_tar_pt = tar_line->list[min_tar_valid - 1];
        auto next_pt = src_line->list.front().get();
        // 判断next_pt->pos是否在prev_tar_pt->pos的前方
        if (!alg::judge_front(prev_tar_pt->pos, next_pt->pos, next_pt->dir)) {
            // for (int i = min_tar_valid - 1; i >= 0; --i) { // TODO:qzc 为什么不是-1，而是-2？
            for (int i = min_tar_valid - 2; i >= 0; --i) { // TODO:qzc 为什么不是-1，而是-2？
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<BoundaryFeature>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                VEC_INSERT(src_line->list, new_pt);
                VEC_INSERT(src_line->match_list, {pt});
                VEC_INSERT(src_line->match_score, pt->score);
                src_line->match_list_point.insert(
                        src_line->match_list_point.begin(), std::vector<BoundaryFeature*>());
                if (next_pt != NULL) {
                    next_pt->prev = new_pt.get();
                    new_pt->next = next_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                next_pt = new_pt.get();
            }
            for (int64_t i = 0; i < src_line->list.size(); ++i) {
                auto &src_pt = src_line->list[i];
                src_pt->line_index = i;
            }
        }
    }
    src_line->merge_lines.push_back(tar_line->src);
    return fsdmap::SUCC;
}

// 根据曲线src与tar匹配上的点的个数，计算匹配度，如果tar中有80%的点都能在src上匹配到，那么认为他们是同一条曲线，可以合并
bool RoadModelProcMergeFeature::is_match_line_boundary(RoadModelSessionData* session,
        BoundaryGroupLine* src, BoundaryLine* tar, double &score) {
    // add by qzc: 如果在外面已经匹配好了，那么直接进行合并
    if(src->matched_line_ids.count(tar->cur_line_id) > 0) {
        score = 1;
        return true;
    }

    double rate_thres = FLAGS_merge_feature_match_rate_threshold; // 0.8

    int total_num = tar->list.size();
    int total_num_origin = tar->list.size();
    int match_num = 0;
    int64_t tar_max_index = 0;
    int64_t src_max_index = 0;
    int64_t tar_min_index = INT_MAX;
    int64_t src_min_index = INT_MAX;
    for (int64_t i = 0; i < total_num; ++i) {
        auto &pt = tar->list[i];
        auto &map = pt->merge_match_map;
        if (map.find(src) != map.end()) { // pt点与曲线src有匹配点
            if (map[src] != NULL) {
               ++match_num;
               auto &src_pt = map[src];
               src_max_index = src_pt->line_index;
               tar_max_index = i;
               tar_min_index = std::min(tar_min_index, i);
               src_min_index = std::min(src_min_index, src_pt->line_index);
            }
        }
    }
    if (src_min_index == 0) {
        total_num -= tar_min_index;
    }
    if (src_max_index == src->list.size() - 2) {
        total_num -= (tar->list.size() - tar_max_index - 1);
    }
    score = (double) match_num / total_num + 1e-6;
    if (match_num == 1 && total_num_origin > 1) {
        score = (double) match_num / total_num_origin + 1e-6;
    }
    // if (score > rate_thres) {
    //     int a = 1;
    // }
    // LOG_DEBUG("merge_boundary[score={:.2f}, match={}, total={}, "
    //         "tar_max={}, tar_min={}, src_max={}, src_min={}]",
    //         score, match_num, total_num, 
    //         tar_max_index, tar_min_index, src_max_index, src_min_index);
    if (score > rate_thres) {
        return true;
    }
    return false;
}

void RoadModelProcMergeFeature::merge_lane_center_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process, int stage) {
    bool debug_log = false;
    // bool debug_log = true;
    int line_size = trail_ptr->lane_center_line_group_list.size();
    std::vector<alg::linestring_t> all_curve(line_size);
    std::vector<std::vector<Eigen::Vector3d>> all_curve_points_dir(line_size);
    for (int64_t i = 0; i < line_size; ++i) {
        auto &group_line = trail_ptr->lane_center_line_group_list[i];
        group_line->cur_line_id = i;
        group_line->matched_line_ids.clear();
        
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if(pt == nullptr) {
                LOG_ERROR("!!!!!!!!!!!!not normal point!!!!!!!");
                continue;
            }
            all_curve[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
            all_curve_points_dir[i].push_back(pt->dir);
        }
    }

    static int g_test_cnt = 10;
    if (debug_log) {
        save_debug_info_only_lane_center_qzc(session, g_test_cnt++);
    }

    std::vector<std::vector<alg::OverlapInfoLineString>> matched_list(line_size); // 每条线段与其他线段的匹配信息
    std::map<int, std::unordered_set<int>> common_prcessed_line; // 当前id，匹配上的line
    for(int i = 0; i < line_size; i++) {
        for (int j = i+1; j < line_size; j++) {
            alg::OverlapInfoLineString curve_overlap;
            curve_overlap.l1_id = i;
            curve_overlap.l2_id = j;
            // lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.05);
            lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.2);
            if (curve_overlap.is_overlap) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                auto &group_line_i = trail_ptr->lane_center_line_group_list[i];
                group_line_i->matched_line_ids.insert(j);
                auto &group_line_j = trail_ptr->lane_center_line_group_list[j];
                group_line_j->matched_line_ids.insert(i);

                if (debug_log && reverse_process == false) {
                    LOG_INFO("[lane_center] 1 is_overlap : {}, is_nearby: {}, i: {}, j:{}, need reverse l1:{}, l2:{}",
                            curve_overlap.is_overlap, curve_overlap.is_nearby, i, j, curve_overlap.is_need_reverse_l1, curve_overlap.is_need_reverse_l2);
                }
                // case1: 有重叠 & 方式一致, 进行合并
            } else if(curve_overlap.is_nearby) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                // auto &group_line_i = trail_ptr->lane_center_line_group_list[i];
                // group_line_i->matched_line_ids.insert(j);
                // auto &group_line_j = trail_ptr->lane_center_line_group_list[j];
                // group_line_j->matched_line_ids.insert(i);

                 if (debug_log && reverse_process == false) {
                    LOG_INFO("[lane_center] 2 is_overlap : {}, is_nearby: {}, i: {}, j:{}, need reverse l1:{}, l2:{}",
                        curve_overlap.is_overlap, curve_overlap.is_nearby, i, j, curve_overlap.is_need_reverse_l1, curve_overlap.is_need_reverse_l2);
                }
            } else {
                if(stage == 0) {
                    auto &group_line_i = trail_ptr->lane_center_line_group_list[i];
                    auto &group_line_j = trail_ptr->lane_center_line_group_list[j];
                    if (curve_overlap.is_overlap_num > 5 && 
                        group_line_i->boundary_type != LaneType::RIGHT_TURN_LC && 
                        group_line_j->boundary_type != LaneType::RIGHT_TURN_LC) {
                        // 如果还存在重复 或者 交叉，就需要打断两条线
                        auto group_line1 = trail_ptr->lane_center_line_group_list[i];
                        int index1 = -1;
                        index1 = curve_overlap.l1_index.front();
                        if(index1 > 0) {
                            group_line1->list[index1]->filter_status = 2;
                        }
                        index1 = curve_overlap.l1_index.back();
                        if(index1 < group_line1->list.size() -1) {
                            group_line1->list[index1]->filter_status = 2;
                        }
    
                        auto group_line2 = trail_ptr->lane_center_line_group_list[j];
                        int index2 = -1;
                        index2 = curve_overlap.l2_index.front();
                        if(index2 > 0) {
                            group_line2->list[index2]->filter_status = 2;
                        }
                        index2 = curve_overlap.l2_index.back();
                        if(index2 < group_line2->list.size() -1) {
                            group_line2->list[index2]->filter_status = 2;
                        }
                        
                        // g_test_cnt >= 10
                        if (debug_log && curve_overlap.is_overlap_num > 0) {
                            LOG_INFO("[lane_center] 3 is_overlap : {}, is_nearby: {}, i: {}, j:{}, l1.size:{}, l2.size:{},    is_overlap_num: {}, l1.front:{}, l1.back:{}, l2.front:{}, l2.back:{}, g_test_cnt:{}",
                                    curve_overlap.is_overlap, curve_overlap.is_nearby, i, j, 
                                    all_curve[i].size(), all_curve[j].size(), curve_overlap.is_overlap_num,
                                    curve_overlap.l1_index.front(), curve_overlap.l1_index.back(), curve_overlap.l2_index.front(), curve_overlap.l2_index.back(), g_test_cnt);
                        }
                    }
                }
            }

            // case4: 无重叠 & 方式不一致，不处理
        }
    }

    for (auto& matched_info : matched_list) {
        std::sort(matched_info.begin(), matched_info.end(), [](const alg::OverlapInfoLineString& a, const alg::OverlapInfoLineString& b) {
            return a.l2_length > b.l2_length;
        });
    }

    // 排序：匹配个数》长度
    std::sort(matched_list.begin(), matched_list.end(), [](const std::vector<alg::OverlapInfoLineString>& a, const std::vector<alg::OverlapInfoLineString>& b) {
        if (a.size() != b.size()) {
            return a.size() > b.size();
        } else if(a.size() == 0 && b.size() == 0) {
            return false;
        }
        
        return a[0].l1_length > b[0].l1_length;
    });

    std::vector<bool> has_processed(line_size, false);
    // 注意：matched_list 已经不是按照，line_size 的顺序了
    for (const auto& matched_info_i : matched_list) {
        int matched_num = matched_info_i.size();
        if (matched_num == 0) {
            continue;
        }

        // 当自己长度比较短，且需要反转，那么除了0以外的其他line，如果需要反转，则需要和以 need_reverse 为准
        auto const& curve_overlap_0 = matched_info_i[0];
        int id1 = curve_overlap_0.l1_id;
        bool need_reverse = curve_overlap_0.is_need_reverse_l1;

        // 必须要是还有未处理过的match点，再进行融合， TODO：qzc 可能还存在一种情况是，上次已经反转了一次，当前还有，那么可能会导致错误的反转，可能需要同步改掉matched_list中的状态才行
        int valid_match_size = 0;
        std::unordered_set<int> other_line_prcessed;
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            int id2 = curve_overlap_j.l2_id;
            if (has_processed[id2] == true) {
                // 如果已经处理过的id2，对应的匹配信息中，有id1，那么就不能重复处理，需要过滤掉
                if (common_prcessed_line.find(id2) != common_prcessed_line.end() && common_prcessed_line[id2].find(id1) != common_prcessed_line[id2].end()) {
                    other_line_prcessed.insert(common_prcessed_line[id2].begin(), common_prcessed_line[id2].end());
                }
                
                continue;
            }
            // 如果之前处理过，那么直接跳过
            if (other_line_prcessed.find(id2) != other_line_prcessed.end()) {
                continue;
            }

            valid_match_size++;
        }

        if (need_reverse && valid_match_size > 0) {
            reverse(trail_ptr->lane_center_line_group_list[id1]->list.begin(), trail_ptr->lane_center_line_group_list[id1]->list.end());
            std::shared_ptr<LaneCenterFeature> prev = NULL;
            for (auto &fls : trail_ptr->lane_center_line_group_list[id1]->list) {
                if (prev == NULL) {
                    prev = fls;
                    continue;
                }
                fls->dir = alg::get_dir(fls->pos, prev->pos);
                prev->dir = fls->dir;
                fls->set_prev(prev.get());
                prev = fls;
            }
             if (debug_log && reverse_process == false) {
                LOG_INFO("[lane_center] 0# need_reverse:{}, matched_num:{}, l1:{}, l2:{}", need_reverse, matched_num, id1, curve_overlap_0.l2_id);
            }
        }

        
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            
            int id2 = curve_overlap_j.l2_id;
             if (debug_log && reverse_process == false) {
                LOG_INFO("[lane_center] 1# need_reverse:{}, matched_num:{}, l1:{}, l2:{}, has_processed[id2]:{}", need_reverse, matched_num, id1, id2, has_processed[id2]);
            }
            if (has_processed[id2] == true) {
                continue;
            }

            if (need_reverse == false) {
                // 如果自己不需要反转，其他和自己不相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 != curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->lane_center_line_group_list[id2]->list.begin(), trail_ptr->lane_center_line_group_list[id2]->list.end());
                    std::shared_ptr<LaneCenterFeature> prev = NULL;
                    for (auto &fls : trail_ptr->lane_center_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                        fls->set_prev(prev.get());
                        prev = fls;
                    }

                     if (debug_log && reverse_process == false) {
                        LOG_INFO("[lane_center] 2# need_reverse:{}, matched_num:{}, l1:{}, l2:{}", need_reverse, matched_num, id1, id2);
                    }
                }
            } else {
                // 如果自己需要反转，其他和自己相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 == curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->lane_center_line_group_list[id2]->list.begin(), trail_ptr->lane_center_line_group_list[id2]->list.end());
                    std::shared_ptr<LaneCenterFeature> prev = NULL;
                    for (auto &fls : trail_ptr->lane_center_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                        fls->set_prev(prev.get());
                        prev = fls;
                    }
                     if (debug_log && reverse_process == false) {
                        LOG_INFO("[lane_center] 3# need_reverse:{}, matched_num:{}, l1:{}, l2:{}", need_reverse, matched_num, id1, id2);
                    }
                }
            }

            // // case1: 有重叠 & 方向一致, 进行合并
            // // case2: 有重叠 & 方向不一致，但是是180度, 则先进行反转再进行合并
            // if (curve_overlap_j.is_overlap) {
            // } 
            // // case3: 无重叠 & 方向一致（首末尾）, 进行合并
            // if (curve_overlap_j.is_nearby) {
            // }
            // case4: 无重叠 & 方向不一致，不处理
        }

        has_processed[id1] = true;
    }

    // merge multi group line
    session->lane_center_line_ptr_for_merge.clear();
    session->bev_frame_lane_center_line.clear();
    
    int lane_size = trail_ptr->lane_center_line_group_list.size();
    if (reverse_process) {
        for (int64_t i = lane_size-1; i >= 0; --i) {
            auto &group_line = trail_ptr->lane_center_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);
            new_line->boundary_type = group_line->boundary_type;
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            if (new_line->boundary_type == LaneType::RIGHT_TURN_LC) { // 人造的中心线直接添加进去
                LOG_ERROR("qzc 0 reverse new_line id: {}", new_line->cur_line_id);
            }
            // std::shared_ptr<LaneCenterGroupLine> group_line;
            //      sstd::vector<std::shared_ptr<LaneCenterFeature>> list;;  // 该曲线上所有的点
            LaneCenterFeature* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                // if (j == group_line->list.size() - 1) {
                //     valid_index = j - 1;
                // }
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                if(pt->filter_status > 2 && pt->invalid()) {
                    continue;
                }
                
                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                new_node->init(pt.get());
                if(pt->filter_status == 2) {
                    new_node->filter_status = 2;
                }
                new_node->boundary_type = group_line->boundary_type;

                if (prev != NULL) {
                    new_node->set_prev(prev);
                }
                prev = new_node.get();

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_center_line[""].push_back(new_line.get());
        }
    } else {
        for (int64_t i = 0; i < lane_size; ++i) {
            auto &group_line = trail_ptr->lane_center_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);
            new_line->boundary_type = group_line->boundary_type;
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;
            if (new_line->boundary_type == LaneType::RIGHT_TURN_LC) { // 人造的中心线直接添加进去
                LOG_ERROR("qzc 0 new_line id: {}", new_line->cur_line_id);
            }

            // std::shared_ptr<LaneCenterGroupLine> group_line;
            //      sstd::vector<std::shared_ptr<LaneCenterFeature>> list;;  // 该曲线上所有的点
            LaneCenterFeature* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                // if (j == group_line->list.size() - 1) {
                //     valid_index = j - 1;
                // }
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                if(pt->filter_status > 2 && pt->invalid()) {
                    continue;
                }

                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                new_node->init(pt.get());
                if(pt->filter_status == 2) {
                    new_node->filter_status = 2;
                }
                new_node->boundary_type = group_line->boundary_type;

                if (prev != NULL) {
                    new_node->set_prev(prev);
                }
                prev = new_node.get();

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_center_line[""].push_back(new_line.get());
        }
    }

    if (debug_log) {
        save_debug_info_only_lane_center_qzc(session, g_test_cnt++);
    }

    // 2. 重新调用 merge single 操作
    merge_single_lane_center_by_trail(session, trail_ptr);

    if (debug_log) {
        save_debug_info_only_lane_center_qzc(session, g_test_cnt++);
    }

}


int RoadModelProcMergeFeature::merge_single_lane_center_by_trail(
        RoadModelSessionData* session, KeyPoseLine* trail) {
    int min_size = FLAGS_merge_feature_proc_min_size; // 10
    int min_match_size = FLAGS_merge_feature_proc_min_match_size; // 1
    double score_thres = FLAGS_merge_feature_proc_score_threshold; // 0.2
    if (trail->list.size() < min_size) { // 轨迹太短
        return fsdmap::FAIL;
    }
    trail->lane_center_line_group_list.reserve(trail->list.size());

    int times = 0;
    while (times < 2) {
        ++times;
        LaneCenterFeatureTree trail_tree;
        trail->lane_center_line_group_list.clear(); // 每次循环前清除
        int64_t raw_line_size = 0;
        // for (int64_t i = 0; i < trail->list.size(); ++i) {
        //     auto &poss = trail->list[i];
        //     if (poss->frame_id == "6181672392159_1672363358942000") {
        //         int a = 1;
        //     }
        //     if (MAP_NOT_FIND(session->bev_frame_lane_center_line, poss->frame_id)) { 
        //         continue;
        //     }
        //     auto &ins_vec = session->bev_frame_lane_center_line[poss->frame_id]; // 当前帧看到的所有车道中心线
        for (const auto &ins_pair : session->bev_frame_lane_center_line) {
            const auto &ins_vec = ins_pair.second;
            if (ins_vec.empty()) {
                continue;
            }
            for (auto &ins : ins_vec) {
                ins->sub_line.clear();
                LaneCenterFeature* prev = NULL;
                auto new_line = session->add_ptr(ins->sub_line);
                new_line->list.reserve(ins->list.size());
                new_line->src = ins;
                int64_t sub_line_index = 0;
                new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);

                // add by qzc
                new_line->boundary_type = ins->boundary_type;
                new_line->cur_line_id = ins->cur_line_id;
                new_line->matched_line_ids = ins->matched_line_ids;

                for (auto &pt : ins->list) {
                    session->debug_pos(pt->pos);
                    if (pt->invalid()) {
                        if (new_line->list.size() > 1) {
                            new_line = session->add_ptr(ins->sub_line);
                            new_line->src = ins;
                            new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);
                            new_line->boundary_type = ins->boundary_type;
                            // TODO:qzc  此处可能不正确了
                            // new_line->cur_line_id = ins->cur_line_id;
                            // new_line->matched_line_ids = ins->matched_line_ids;

                        } else {
                            new_line->list.clear();
                        }
                        prev = NULL;
                        continue;
                    }
                    new_line->list.push_back(pt);
                    pt->merge_match_map.clear();
                    if (prev != NULL) {
                        pt->set_prev(prev);
                    }
                    prev = pt;
                }
                for (auto &sub_line : ins->sub_line) {
                    if (sub_line->list.size() <= 1) {
                        continue;
                    }
                    match_frame_line_lane_center(session, trail->lane_center_line_group_list, 
                            trail_tree, sub_line.get());
                }
            }
            // save_frame_log_lane_center(session, trail, poss, times);
            raw_line_size += ins_vec.size();

        }
        LOG_INFO("merge_lane_center_final[id={}, p_size={}, times={}, size={}, raw_size={}]", 
                (int64_t)trail, trail->list.size(), times, 
                trail->lane_center_line_group_list.size(), raw_line_size);
        // 过滤
        for (int64_t i = 0; i < trail->lane_center_line_group_list.size(); ++i) {
            auto &group_line = trail->lane_center_line_group_list[i];
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                if (j == group_line->list.size() - 1) {
                    valid_index = j - 1;
                }
                auto &pt = group_line->list[j];
                session->debug_pos(pt->pos);
                int match_num = group_line->match_list[j].size();
                // int match_num = group_line->match_list_point[valid_index].size();
                UMAP<std::string, int> match_frame_map;
                for (auto &raw_pt : group_line->match_list_point[valid_index]) {
                    match_frame_map[raw_pt->frame_id]++;
                }
                int other_match_num = 0;
                for (auto &raw_pt : group_line->match_list[valid_index]) {
                    auto &match_map = raw_pt->merge_match_map;
                    match_frame_map[raw_pt->frame_id]++;
                    for (auto &it : match_map) {
                        if (it.first == group_line.get()) {
                            continue;
                        }
                        auto &line_match_list = it.first->match_list_point;
                        other_match_num += line_match_list[it.second->line_index].size();
                        match_frame_map[it.second->frame_id]++;
                    }
                }
                double total_num = match_num + other_match_num;
                pt->score = match_num / (total_num + 1e-6);
                // pt->score = (double)match_frame_map.size() / pt->match_frame_num;
                if (total_num < min_match_size) {
                    pt->filter_status = 2;
                    for (auto &raw_pt : group_line->match_list[valid_index]) {
                        // session->debug_pos(raw_pt->pos);
                        raw_pt->filter_status = 2;
                    }
                }
                // if (pt->score < score_thres) {
                //     pt->filter_status = 3;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 3;
                //     }
                // }
                // DLOG_POINT(pt->pos, "merge_filter_lane_center[s={}, m1={}, m2={}, f={}]",
                //         pt->score, match_num, other_match_num, pt->filter_status);
            }
        }
    }
    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::match_frame_line_lane_center(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, 
        LaneCenterFeatureTree &trail_tree,
        LaneCenterLine* tar_line) {
    double radius = FLAGS_merge_feature_search_radius; // 5
    double scope = FLAGS_merge_feature_scope; // 1
    double theta_thres = FLAGS_merge_feature_theta; // 20

    bool debug_log = false;
    // bool debug_log = true;

    std::vector<LaneCenterFeature*> secs;
    UMAP<LaneCenterGroupLine*, int> cache_map; // 存储与tar_line的车道中心点存在同一车道的中心点的车道中心线
    UMAP<LaneCenterFeature*, int> used_map;
    // 遍历某条车道中心线里的每个点，选出附近5m半径内的车道中心点，在这些点中选出距离在一个车道范围内的点
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        // if (tar_line->boundary_type == LaneType::RIGHT_TURN_LC) { // 人造的中心线直接添加进去
        //     continue;
        // }

        auto &pt = tar_line->list[j];
        session->debug_pos(pt->pos);
        secs.clear();
        used_map.clear();
        auto &match_map = pt->merge_match_map;  // 该点附近5m半径内的车道中心点，，在这些点中选出距离在一个车道范围内的车道中心点，以及这个车道中心点所属的车道中心线上
        trail_tree.search(pt->pos, radius, secs);  // 筛选出pt点附近5m半径内的车道中心点
        Eigen::Vector3d v_pt = alg::get_vertical_pos(pt->pos, pt->dir, 50, true);
        Eigen::Vector3d cross_point = {0, 0, 0};
        for (int64_t k = 0; k < secs.size(); ++k) {
            auto &src_pt = secs[k];
            if (MAP_FIND(used_map, src_pt)) {
                continue;
            }
            used_map[src_pt] = 1;
            session->debug_pos2(pt->pos, src_pt->pos);
            double dis = 0;
            if (src_pt->next == NULL) {
                continue;
                // double h_dis = alg::calc_hori_dis(pt->pos, src_pt->pos, src_pt->dir, true);
                // if (h_dis < 0 && src_pt->next == NULL) {
                //     continue;
                // } 
                // if (h_dis > 0 && src_pt->prev == NULL) {
                //     continue;
                // }
                // dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            } else {
                if (!alg::get_cross_point_by_point(src_pt->pos, src_pt->next->pos,
                            pt->pos, v_pt, cross_point, false, 2, 1000)) {
                    continue;
                }
                dis = alg::calc_dis(pt->pos, cross_point);
            }
            if (dis > scope) { // 两个车道中心点不能是在不同车道里
                // match_map[src_pt->group_line] = NULL;
                continue;
            }
            double theta = alg::calc_theta(src_pt->dir, pt->dir);
            if (theta > theta_thres) {
                continue;
            }
            double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            if (s_dis > scope) { // 两个车道中心点不能是在不同车道里
                continue;
            }
            match_map[src_pt->group_line] = src_pt;
            src_pt->group_line->match_list_point[src_pt->line_index].push_back(pt);
            if (MAP_FIND(cache_map, src_pt->group_line)) {
                continue;
            }
            cache_map[src_pt->group_line] = 1;
        }
    }
    using line_score = std::pair<LaneCenterGroupLine*, double>;
    std::vector<line_score> match_list; // <与tar_line有匹配点的车道中心线并且匹配率达到80%, 匹配度得分>
    for (auto &it : cache_map) {
        double score = 0;
        if (!is_match_line_lane_center(session, it.first, tar_line, score)) {
            continue;
        }
        
        if (debug_log &&(tar_line->boundary_type == LaneType::RIGHT_TURN_LC || (it.first->cur_line_id>=27 && it.first->cur_line_id<=30))) { // 人造的中心线直接添加进去
            LOG_WARN("qzc 3 id: {} {}", it.first->cur_line_id, tar_line->cur_line_id);
        }

        match_list.emplace_back(std::make_pair(it.first, score));
    }

    if (match_list.size() == 0) { // 没有与tar_line匹配的车道中心线，把与tar_line当成一条新的车道中心线处理
        gen_new_lane_center_group(session, trail_group, tar_line, trail_tree);
        if (debug_log && (tar_line->boundary_type == LaneType::RIGHT_TURN_LC)) { // 人造的中心线直接添加进去
            LOG_WARN("qzc 4 id: {}", tar_line->cur_line_id);
        }
    } else {
        SORT(match_list, [](const line_score &l, const line_score &r) {
                return l.second > r.second;
                });
        auto &top_line = match_list[0].first;
        update_lane_center_group_line(session, top_line, tar_line, trail_tree);
        if (debug_log && (tar_line->boundary_type == LaneType::RIGHT_TURN_LC || (top_line->cur_line_id>=27 && top_line->cur_line_id<=30))) { // 人造的中心线直接添加进去
            LOG_WARN("qzc 5 id: {} {}", top_line->cur_line_id, tar_line->cur_line_id);
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::gen_new_lane_center_group(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, LaneCenterLine* tar_line,
        LaneCenterFeatureTree &trail_tree) {
    auto new_line = std::make_shared<LaneCenterGroupLine>();
    new_line->id = tar_line->id;
    LaneCenterFeature* prev_pt = NULL;
    for (auto &pt : tar_line->list) {
        auto new_pt = std::make_shared<LaneCenterFeature>();
        new_pt->boundary_type = tar_line->boundary_type;
        new_pt->init(pt);
        new_pt->group_line = new_line.get();
        new_pt->line_index = new_line->list.size();
        new_pt->line_id = new_line->id;
        new_line->list.push_back(new_pt);
        new_line->match_list.push_back({pt});
        new_line->match_score.push_back(pt->score);
        new_line->match_list_point.resize(new_line->match_list.size());
        if (prev_pt != NULL) {
            prev_pt->next = new_pt.get();
            new_pt->prev = prev_pt;
        }
        trail_tree.insert(new_pt->pos, new_pt.get());
        prev_pt = new_pt.get();
    }
    new_line->merge_lines.push_back(tar_line->src);
    new_line->cur_line_id = tar_line->cur_line_id;
    new_line->boundary_type = tar_line->boundary_type;
    new_line->matched_line_ids.insert(tar_line->matched_line_ids.begin(), tar_line->matched_line_ids.end());
    trail_group.push_back(new_line);
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::update_lane_center_group_line(RoadModelSessionData* session,
        LaneCenterGroupLine* src_line, LaneCenterLine* tar_line,
        LaneCenterFeatureTree &trail_tree) {
    double scope = FLAGS_merge_feature_scope; // 1m
    int64_t max_tar_valid = 0;  // tar_line与src_line匹配的点中，最大的index
    int64_t max_src_valid = 0; // src_line与tar_line匹配的点中，最大的index
    int64_t min_tar_valid = INT_MAX;  // tar_line与src_line匹配的点中，最小的index
    int64_t min_src_valid = INT_MAX;  // src_line与tar_line匹配的点中，最小的index
    for (int64_t i = 0; i < tar_line->list.size(); ++i) {
        auto &pt = tar_line->list[i];
        auto &map = pt->merge_match_map;
        if (MAP_NOT_FIND(map, src_line)) {
            continue;
        }
        auto &src_pt = map[src_line];
        int64_t src_index = src_pt->line_index;
        double total_score = src_line->match_score[src_index];
        double v_dis_1 = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir, true); // pt与src_pt之间的垂向距离
        // double v_dis_2 = -alg::calc_vertical_dis(src_pt->pos, pt->pos, pt->dir, true);
        double v_dis = v_dis_1;
        // if (fabs(v_dis_1) > fabs(v_dis_2)) {
        //     v_dis = v_dis_2;
        // }
        if (v_dis > scope) {
            continue;
        }
        total_score  += pt->score;
        if(total_score < 1e-6) {
             LOG_ERROR("lane center total_score: {}, pt->src->score:{}]", total_score, pt->score);
             total_score += 1e-6;
        }
        double opt_dis = pt->score / total_score * v_dis;// 1e-6 防止分母为0
        Eigen::Vector3d opt_pos = alg::get_vertical_pos(src_pt->pos, src_pt->dir, opt_dis);
        session->debug_pos(opt_pos);
        src_pt->pos = opt_pos;
        trail_tree.insert(src_pt->pos, src_pt);
        src_line->match_list[src_index].push_back(pt);
        src_line->match_score[src_index] += pt->score;
        max_tar_valid = std::max(max_tar_valid, i);
        min_tar_valid = std::min(min_tar_valid, i);
        max_src_valid = std::max(max_src_valid, src_index);
        min_src_valid = std::min(min_src_valid, src_index);
    }
    // 深坑，size 返回的是Uint 不能减
    int64_t end_index = (tar_line->list.size() >= 3) ? (tar_line->list.size() - 3) : 0;
    if (max_tar_valid <= end_index) {
        auto &next_tar_pt = tar_line->list[max_tar_valid + 1];
        auto prev_pt = src_line->list.back().get();
        if (alg::judge_front(next_tar_pt->pos, prev_pt->pos, prev_pt->dir)) {
            for (int i = max_tar_valid + 2; i < tar_line->list.size(); ++i) {
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<LaneCenterFeature>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                new_pt->line_index = src_line->list.size();
                src_line->list.push_back(new_pt);
                src_line->match_list.push_back({pt});
                src_line->match_score.push_back(pt->score);
                src_line->match_list_point.resize(src_line->match_list.size());
                if (prev_pt != NULL) {
                    prev_pt->next = new_pt.get();
                    new_pt->prev = prev_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                prev_pt = new_pt.get();
            }
        }
    }

    if (min_tar_valid >= 2) {
        auto &prev_tar_pt = tar_line->list[min_tar_valid - 1];
        auto next_pt = src_line->list.front().get();
        if (!alg::judge_front(prev_tar_pt->pos, next_pt->pos, next_pt->dir)) {
            for (int i = min_tar_valid - 2; i >= 0; --i) {
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<LaneCenterFeature>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                VEC_INSERT(src_line->list, new_pt);
                VEC_INSERT(src_line->match_list, {pt});
                VEC_INSERT(src_line->match_score, pt->score);
                src_line->match_list_point.insert(
                        src_line->match_list_point.begin(), std::vector<LaneCenterFeature*>());
                if (next_pt != NULL) {
                    next_pt->prev = new_pt.get();
                    new_pt->next = next_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                next_pt = new_pt.get();
            }
            for (int64_t i = 0; i < src_line->list.size(); ++i) {
                auto &src_pt = src_line->list[i];
                src_pt->line_index = i;
            }
        }
    }
    src_line->merge_lines.push_back(tar_line->src);
    return fsdmap::SUCC;
}

bool RoadModelProcMergeFeature::is_match_line_lane_center(RoadModelSessionData* session,
        LaneCenterGroupLine* src, LaneCenterLine* tar, double &score) {
    bool debug_log = false;
    // bool debug_log = true;
    if(src->matched_line_ids.count(tar->cur_line_id) > 0) {
        if (debug_log && tar->boundary_type == LaneType::RIGHT_TURN_LC) { // 人造的中心线直接添加进去
            LOG_WARN("qzc 1 src id: {}, tar id: {}", src->cur_line_id, tar->cur_line_id);
        }

        score = 1;
        return true;
    }

    // double rate_thres = 0.3;
    double rate_thres = FLAGS_merge_feature_match_rate_threshold;

    int total_num = tar->list.size();
    int total_num_origin = tar->list.size();
    int match_num = 0;
    int64_t tar_max_index = 0;
    int64_t src_max_index = 0;
    int64_t tar_min_index = INT_MAX;
    int64_t src_min_index = INT_MAX;
    for (int64_t i = 0; i < total_num; ++i) {
        auto &pt = tar->list[i];
        auto &map = pt->merge_match_map;
        if (map.find(src) != map.end()) {
            if (map[src] != NULL) {
               ++match_num;
               auto &src_pt = map[src];
               src_max_index = src_pt->line_index;
               tar_max_index = i;
               tar_min_index = std::min(tar_min_index, i);
               src_min_index = std::min(src_min_index, src_pt->line_index);
            }
        }
    }
    if (src_min_index == 0) {
        total_num -= tar_min_index;
    }

    // tar_min_index < 6, 说明 tar 起始点+5的位置也落在 src 中， 如果tar_min_index太大，可能是tar中间某部分和src的部分在一起，导致tar误匹配，
    // 【0，tar_min_index】会被切割掉
    // if (src_max_index == src->list.size() - 2 && tar_min_index < 6) { 
    if (src_max_index == src->list.size() - 2) { 
        total_num -= (tar->list.size() - tar_max_index - 1);
    }
    score = (double) match_num / total_num + 1e-6;    
    if (match_num == 1 && total_num_origin > 1) {
        score = (double) match_num / total_num_origin + 1e-6;
    }

    // if (score > rate_thres) {
    //     int a = 1;
    // }
    if (debug_log && (tar->boundary_type == LaneType::RIGHT_TURN_LC || (src->cur_line_id>=27 && src->cur_line_id<=30))) { // 人造的中心线直接添加进去
        LOG_WARN("qzc 2 src id: {}, tar id: {}", src->cur_line_id, tar->cur_line_id);
        LOG_WARN("merge_lane_center[score={:.2f}, match={}, total={}, "
            "tar_max={}, tar_min={}, src_max={}, src_min={}, src size={}, tar size={}]",
            score, match_num, total_num, 
            tar_max_index, tar_min_index, src_max_index, src_min_index, src->list.size(), tar->list.size());
    }

    if (score > rate_thres) {
        return true;
    }
    return false;
}


std::pair<int, int> RoadModelProcMergeFeature::search_key_pose(RoadModelSessionData* session,
        KeyPoseTree &tree, Eigen::Vector3d pos, Eigen::Vector3d dir, std::string &trail_id,
        KeyPose* curr_poss) {
    double h_dis_thres = FLAGS_merge_feature_single_h_dis; // 30m
    double v_dis_thres = FLAGS_merge_feature_single_v_dis; // 30m
    double theta_thres = FLAGS_merge_feature_theta_thres;
    std::vector<KeyPose*> search_sec;
    tree.search(pos, h_dis_thres, search_sec);
    int poss_num = 1;
    // 在pos点30m范围内搜索所有最近邻ins位置，在这些位置中筛选出pos到这些位置方向向量距离小于30m的位置，
    // 对该位置点对应的轨迹id累加，表示该轨迹id离pos位置较近的轨迹点个数
    UMAP<std::string, int> trail_map; // <轨迹id，该轨迹id中离pos位置较近的轨迹点个数>
    for (auto &poss : search_sec) {
        // double theta =alg::calc_theta(poss->dir, dir, true);
        // if (theta > theta_thres) {
        //     continue;
        // }
        
        double v_dis = alg::calc_vertical_dis(pos, poss->pos, poss->dir); // 计算pos点到直线poss->dir的距离
        if (v_dis > v_dis_thres) {
            continue;
        }
        trail_map[poss->trail_id]++;
        // if (poss->trail_id != trail_id) {
        //     continue;
        // } // modified by ccj, 20241219, 多趟轨迹在semantic阶段已经被融合到一起了，不需要再判断
        if (curr_poss == poss) {
            continue;
        }
        double h_dis = alg::calc_hori_dis(pos, poss->pos, poss->dir);
        if (h_dis > h_dis_thres) {
            continue;
        }
        
        ++poss_num;
    }
    if (poss_num == 1) {
        int a = 1;
    }
    return std::move(std::make_pair(poss_num, trail_map.size()));
}

int64_t RoadModelProcMergeFeature::vote_attr_by_fls(RoadModelSessionData* session,
        LaneLineSample *base_fls) {
    double radius = FLAGS_merge_feature_vote_yellow_radius;
    double scope = FLAGS_merge_feature_vote_yellow_scope;
    double theta_threshold = FLAGS_merge_feature_vote_yellow_theta_treshold;
    double dis_threshold = FLAGS_merge_feature_vote_yellow_dis_treshold;
    auto &pos = base_fls->pos;
    std::vector<LaneLineSample*> search_sec;
    session->lane_line_sample_tree.search(pos, radius, search_sec);
    std::vector<LaneLineSample*> attr_data_list;
    int64_t base_num = 0;
    UMAP<std::string, std::pair<LaneLineSample*, double>> data_map;
    for (auto &fls : search_sec) {
        if (fls->src->src_status != 1 && fls->src->src_status != 4 && fls->src->src_status != 5) {
            continue;
        }
        double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir);
        if (v_dis > dis_threshold) {
            continue;
        }
        double h_dis = alg::calc_hori_dis(fls->pos, base_fls->pos, base_fls->dir);
        if (h_dis > scope) {
            continue;
        }
        double theta = alg::calc_theta(fls->dir, base_fls->dir, true);
        if (theta > theta_threshold) {
            continue;
        }
        ++base_num;
        if (MAP_NOT_FIND(data_map, fls->src->line_id)) {
            data_map[fls->src->line_id] = std::make_pair(fls, h_dis);
        } else if (data_map[fls->src->line_id].second > h_dis) {
            data_map[fls->src->line_id] = std::make_pair(fls, h_dis);
        }
    }
    for (auto &tit : data_map) {
        attr_data_list.push_back(tit.second.first);
    }
    int64_t num_yellow = vote_yellow(session, base_fls, attr_data_list);
    int64_t num_double = vote_double(session, base_fls, attr_data_list);
    int64_t num_geo = vote_geo(session, base_fls, attr_data_list);
    return base_num;
}

int64_t RoadModelProcMergeFeature::vote_yellow(RoadModelSessionData* session,
        LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list) {
    double dis_lambda = FLAGS_merge_feature_vote_yellow_dis_lambda;
    double min_thres = FLAGS_merge_feature_vote_yellow_min_threshold;
    double max_thres = FLAGS_merge_feature_vote_yellow_max_threshold;
    std::vector<std::pair<double, int>> data_list;

    double total_score = 0;
    double total_value = 0;
    int64_t valid_num = 0;
    for (auto &fls : attr_data_list) {
        if (fls->src->src_status != 1) {
            continue;
        }
        ++valid_num;
        double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir);
        double score = dis_lambda / (dis_lambda + v_dis); // v_dis越小，score得分越高
        total_value += score * fls->src->attr.color;
        total_score += score;
    }
    if (valid_num <= 0) {
        return valid_num;
    }
    double value = total_value / total_score;
    if (value > min_thres && value < max_thres) {
        // base_fls->attr.color = 2;
        base_fls->attr.color = 3;
    } else if (value <= min_thres){
        // base_fls->attr.color = 1;
        base_fls->attr.color = 2;
    } else {
        base_fls->attr.color = (int) (value + 0.5);
    }
    LOG_INFO("vote:{} :{}",value, base_fls->attr.color);
    return valid_num;
}

// 投票计算是否是双线
int64_t RoadModelProcMergeFeature::vote_double(RoadModelSessionData* session,
        LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list) {
    double dis_lambda = FLAGS_merge_feature_vote_yellow_dis_lambda;
    double value_threshold = FLAGS_merge_feature_vote_double_value_threshold;
    std::vector<std::pair<double, int>> data_list;

    double total_score = 0;
    double total_value = 0;
    int64_t valid_num = 0;
    for (auto &fls : attr_data_list) {
        if (fls->src->src_status != 4 && fls->src->src_status != 5) {
            continue;
        }
        ++valid_num;
        double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir);
        double score = dis_lambda / (dis_lambda + v_dis); // v_dis越小，score得分越高
        float is_double = fls->src->attr.is_double_line == 1 ? 1 : 0;
        total_value += score * is_double;
        total_score += score;
    }
    if (valid_num <= 0) {
        return valid_num;
    }
    double value = total_value / total_score;
    if (value > value_threshold) {
        base_fls->attr.is_double_line = 1;
    } else {
        base_fls->attr.is_double_line = 0;
    }
    return valid_num;
}

// 投票计算几何属性
int64_t RoadModelProcMergeFeature::vote_geo(RoadModelSessionData* session,
        LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list) {
    double dis_lambda = FLAGS_merge_feature_vote_yellow_dis_lambda;
    // double value_threshold = FLAGS_merge_feature_vote_geo_value_threshold;
    std::vector<std::pair<double, int>> data_list;

    double total_score = 0;
    double total_value = 0;
    int64_t valid_num = 0;
    for (auto &fls : attr_data_list) {
        if (fls->src->src_status != 5 && fls->src->src_status != 4 && fls->src->src_status != 1) {
            continue;
        }
        ++valid_num;
        double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir);
        double score = dis_lambda / (dis_lambda + v_dis); // v_dis越小，score得分越高
        total_value += score * fls->src->attr.geo;
        total_score += score;
    }
    if (valid_num <= 0) {
        return valid_num;
    }
    double value = total_value / total_score;
    base_fls->attr.geo = (int)(value + 0.5);
    return valid_num;
}

// 用与base_fls匹配上的点集attr_data_list来统计base_fls属于的类别，
// 用attr_data_list中出现频率最高的类别作为base_fls的类别
int64_t RoadModelProcMergeFeature::vote_type(RoadModelSessionData* session,
        BoundaryFeature *base_fls, std::vector<BoundaryFeature*> &attr_data_list) {
    std::vector<std::pair<double, int>> data_list;

    double total_score = 0;
    double total_value = 0;
    int64_t valid_num = 0;
    UMAP<int, int> type_map;
    for (auto &fls : attr_data_list) {
        if (fls->src_status != 1) {
            continue;
        }
        ++valid_num;
        if(MAP_NOT_FIND(type_map,fls->sub_type)){
            type_map[fls->sub_type] = 0;
        }
        else{
            type_map[fls->sub_type]++;
        }
    }
    if (valid_num <= 0) {
        return valid_num;
    }
    std::vector<std::pair<int, int>> temp_type(type_map.begin(), type_map.end());
    std::sort(temp_type.begin(), temp_type.end(), [](auto& a, auto& b) {return a.second >= b.second;});

    base_fls->sub_type = temp_type[0].first;
    return valid_num;
}


int RoadModelProcMergeFeature::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    std::string total_log_name = utils::fmt("merge_feature");
    session->set_display_name(total_log_name.c_str());

//    for (auto &trail : session->key_pose_map) {
//         auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
//         auto trail_ptr = &trail;
//         // session->thread_pool->schedule(
//         //         [log, trail_ptr, session, this](utils::ProcessBar *process_bar) {
//         log->color = {223, 130, 154};
//         auto &first_poss = trail_ptr->second.list.front();
//         for (auto &key_pose : trail_ptr->second.list) {
//             auto &ele = log->add(key_pose->pos, 1, key_pose->raw_no);
//             ele.label.opt_label = key_pose->raw_file_no;
//             // ele.label.time = key_pose->timestamp;
//         }
//     }


    // UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;
    // for (auto &it : session->key_pose_map) {
    // auto lit1 = session->key_pose_map.begin();
    // if (lit1 != session->key_pose_map.end()) {
    //     auto trail_ptr = &lit1->second;

        #if 1
        for (int64_t i = 0; i < session->merge_boundary_line_list.size(); ++i) {
            auto &group_list = session->merge_boundary_line_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_boundary");
            // log->color = {81, 89, 240};  // 天蓝色
            // log->color={180, 255, 255}; // 浅蓝绿色
            log->color={0, 255, 255}; // 蓝绿色
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);
                } else {
                    prev=pt.get();
                }
                if(j < 2 || d>=4 || j==group_list->list.size()-1 || group_list->boundary_type == LaneType::ISLAND_RB) {
                    // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                    auto &ele = log->add(pt->pos, 1);
                    ele.label.opt_label = j;
                    ele.label.intensity_opt = i;
                    ele.label.score = pt->color;
                    if (pt->invalid()) {
                        ele.color={255, 0, 255}; // 深紫色
                    }
                    ele.label.label = pt->type; // 临时弄的
                    prev=pt.get();
                }
            }

            // trail_map[lit1->first].push_back(log);
        }

        for (int64_t i = 0; i < removed_boundary_line_group_list.size(); ++i) {
            auto &group_list = removed_boundary_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "removed_boundary");
            log->color={255, 0, 0}; // 红色
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);
                } else {
                    prev=pt.get();
                }

                if(j < 2 || d>=1 || j==group_list->list.size()-1 || group_list->boundary_type == LaneType::ISLAND_RB) {
                    // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                    auto &ele = log->add(pt->pos, 1);
                    ele.label.opt_label = j;
                    ele.label.intensity_opt = i;
                    ele.label.score = pt->score;
                    ele.label.label = 11; // 临时弄的
                    prev=pt.get();
                }
            }
        }
        removed_boundary_line_group_list.clear();
        #endif

        #if 1
        for (int64_t i = 0; i < session->merge_lane_line_list.size(); ++i) {
            auto &group_list = session->merge_lane_line_list[i];
        
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane");
            log->color = {255, 255, 255}; //白色
            LaneLineSample*prev=NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);
                } else {
                    prev=pt.get();
                }

                if(j<2 || d >=4 || j==group_list->list.size()-1) {
                    // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                    auto &ele = log->add(pt->pos, 1);
                    // auto &ele = log->add(pt->pos);
                    ele.label.opt_label = j;
                    ele.label.intensity_opt = i;
                    ele.label.label = pt->attr.type; // 临时弄的
                    ele.label.score = pt->attr.color;
                    if (pt->invalid()) {
                        ele.color={255, 128, 255}; // 浅紫色
                    }

                    switch(pt->attr.type)
                    {
                        case 2:
                        {
                            ele.color={255, 255, 0}; //黄色
                            break;
                        }
                        case 3:
                        {
                            ele.color={255, 255, 255}; //白色
                        
                            break;
                        }
                        default:
                            ele.color={255, 255, 180}; // 浅黄色
                    }

                    ele.label.cloud_pano_seg=pt->attr.type;
                    ele.label.cloud_line_seg=pt->attr.color;
                    // 
                    // ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                    // ele.label.intensity_opt=i;
                    // LOG_INFO("merge color:{} type:{}",pt->attr.color,pt->attr.type)
                    prev=pt.get();
                }
            }

            // trail_map[lit1->first].push_back(log);
        }

        for (int64_t i = 0; i < session->merge_lane_center_list.size(); ++i) {
            int oppo = 0;
            auto &group_list = session->merge_lane_center_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane_center");
            log->color = {0, 255, 0};
            LaneCenterFeature*prev=NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                // ele.label.opt_label = i;
                // ele.label.score = pt->score;
                // if (pt->invalid()) {
                //     ele.color = {100, 100, 100};
                // }
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);

                    auto dir = alg::get_dir(pt->pos,prev->pos);
                    if(alg::calc_theta1(dir,prev->dir,true)>120) {
                        oppo++;
                    } 
                } else {
                    prev=pt.get();
                }

                if(j<2 || d>=4 || j==group_list->list.size()-1) {
                // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                    // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                    auto &ele = log->add(pt->pos, 1);
                    // ele.label.opt_label = i;
                    ele.label.opt_label = j;
                    ele.label.intensity_opt=group_list->cur_line_id;
                    ele.label.score = pt->score;
                    ele.label.label = 3; // 临时弄的
                    if (pt->invalid()) {
                        ele.color={255, 200, 255}; // 浅浅紫色
                    }

                    if (group_list->boundary_type == LaneType::DEBUG) {
                        ele.label.label = 10; // 临时弄的
                    }
                    // ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                    prev=pt.get();
                } 
            }
            if (oppo > 0) {
                LOG_ERROR("line {} has oppo dir, cnt : {}", i, oppo);
            }
            // trail_map[lit1->first].push_back(log);
        }


        for (int64_t i = 0; i < removed_lane_center_line_group_list.size(); ++i) {
            auto &group_list = removed_lane_center_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "removed_lane_center");
            log->color={255, 100, 0}; // 橘红色
            LaneCenterFeature* prev = NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);
                } else {
                    prev=pt.get();
                }

                if(j < 2 || d>=1 || j==group_list->list.size()-1) {
                    // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                    auto &ele = log->add(pt->pos, 1);
                    ele.label.opt_label = j;
                    ele.label.intensity_opt = i;
                    ele.label.score = pt->score;
                    ele.label.label = 13; // 临时弄的
                    prev=pt.get();
                }
            }
        }
        removed_lane_center_line_group_list.clear();
        #endif

        // continue;
    // }

    // for (auto &it : trail_map) {
    //     auto log_name = session->get_debug_dir("merge_detail/1_{}_{}.png", 
    //             "merge_single_trail", lit1->first);
    //     utils::save_display_image(log_name.c_str(), session->_scope, it.second);

    //     auto pcd_name = session->get_debug_dir("merge_detail/1_{}_{}.pcd", 
    //             "merge_single_trail", lit1->first);
    //     utils::save_display_pcd(pcd_name.c_str(), session->_scope, it.second);
    // }
    session->save_debug_info(total_log_name.c_str());

    save_debug_info_only_lane_center(session, 2);

    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::save_debug_info_only_lane(RoadModelSessionData* session, int index) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    std::string total_log_name = utils::fmt("merge_feature_lane_" + std::to_string(index));
    session->set_display_name(total_log_name.c_str());

    // UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;

    // for (auto &it : session->key_pose_map) {
    auto lit1 = session->key_pose_map.begin();
    if (lit1 != session->key_pose_map.end()) {
        auto trail_ptr = &lit1->second;

        for (int64_t i = 0; i < trail_ptr->lane_line_group_list.size(); ++i) {
            auto &group_list = trail_ptr->lane_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane");
            log->color = {255, 255, 255};
            LaneLineSample*prev=NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);
                } else {
                    prev=pt.get();
                }
                if(j<2 || d>=4 || j==group_list->list.size()-1) {
                    auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                    // auto &ele = log->add(pt->pos);
                    ele.label.opt_label = i;
                    ele.label.intensity_opt = j;
                    ele.label.label = 2; // 临时弄的
                    ele.label.score = pt->score;
                    if (pt->invalid()) {
                        ele.color = {255, 0, 0};
                    }
                    ele.label.cloud_pano_seg=pt->attr.type;
                    ele.label.cloud_line_seg=pt->attr.color;
                    // 
                    ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                    // LOG_INFO("merge color:{} type:{}",pt->attr.color,pt->attr.type)

                    prev=pt.get();
                }
            }
            // trail_map[lit1->first].push_back(log);
        }
    }

    session->save_debug_info(total_log_name.c_str());

    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::save_debug_info_only_lane_center(RoadModelSessionData* session, int index) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    std::string total_log_name = utils::fmt("merge_feature_lane_center_" + std::to_string(index));
    session->set_display_name(total_log_name.c_str());

    // UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;
    // for (auto &it : session->key_pose_map) {
    auto lit1 = session->key_pose_map.begin();
    if (lit1 != session->key_pose_map.end()) {
        auto trail_ptr = &lit1->second;

        for (int64_t i = 0; i < trail_ptr->lane_center_line_group_list.size(); ++i) {
            auto &group_list = trail_ptr->lane_center_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane_center");
            log->color = {0, 255, 0};
            LaneCenterFeature*prev=NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                // ele.label.opt_label = i;
                // ele.label.score = pt->score;
                // if (pt->invalid()) {
                //     ele.color = {100, 100, 100};
                // }
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);
                } else {
                    prev=pt.get();
                }
                if(j<2 || d>=0.1 || j==group_list->list.size()-1) {
                // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                    auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                    ele.label.opt_label = j;
                    ele.label.score = pt->score;
                    ele.label.label = 3; // 临时弄的
                    if (pt->invalid()) {
                        ele.color = {100, 100, 100};
                    }
                    // ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                    ele.label.intensity_opt=i;
                    prev=pt.get();
                } 
            }
            // trail_map[lit1->first].push_back(log);
        }
    }

    session->save_debug_info(total_log_name.c_str());

    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::save_debug_info_only_lane_center_qzc(RoadModelSessionData* session, int index, int mode) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    std::string total_log_name = utils::fmt("merge_feature_lane_center_" + std::to_string(index));
    session->set_display_name(total_log_name.c_str());
    if (mode == 1) {
        for (const auto &ins_pair : session->bev_frame_lane_center_line) {
            const auto &ins_vec = ins_pair.second;
            if (ins_vec.empty()) {
                continue;
            }
            int cnt = 0;
            for (auto &group_list : ins_vec) {
                auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane_center");
                log->color = {0, 255, 0};
                LaneCenterFeature*prev=NULL;
                for (int64_t j = 0; j < group_list->list.size(); ++j) {
                    auto &pt = group_list->list[j];
                    double d = 5;
                    if (prev) {
                        d = alg::calc_dis(pt->pos,prev->pos);
                    } else {
                        prev=pt;
                    }
                    if(j<2 || d>=4 || j==group_list->list.size()-1) {
                    // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                        auto &ele = log->add(pt->pos);
                        // ele.label.opt_label = i;
                        ele.label.opt_label = group_list->cur_line_id;
                        // ele.label.score = pt->score;
                        ele.label.score = cnt;
                        ele.label.label = group_list->boundary_type; // 临时弄的
                        if (pt->invalid()) {
                            ele.color = {100, 100, 100};
                        }
                        // ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                        ele.label.intensity_opt=j;
                        prev=pt;
                    }
                }
                cnt++;
            }
        }
    } else {
        auto lit1 = session->key_pose_map.begin();
        if (lit1 != session->key_pose_map.end()) {
            auto trail_ptr = &lit1->second;

            int cnt = 0;
            for (int64_t i = 0; i < trail_ptr->lane_center_line_group_list.size(); ++i) {
                auto &group_list = trail_ptr->lane_center_line_group_list[i];
                auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane_center");
                log->color = {0, 255, 0};
                LaneCenterFeature*prev=NULL;
                for (int64_t j = 0; j < group_list->list.size(); ++j) {
                    auto &pt = group_list->list[j];
                    double d = 5;
                    if (prev) {
                        d = alg::calc_dis(pt->pos,prev->pos);
                    } else {
                        prev=pt.get();
                    }
                    if(j<2 || d>=4 || j==group_list->list.size()-1) {
                    // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                        auto &ele = log->add(pt->pos);
                        // ele.label.opt_label = i;
                        ele.label.opt_label = j;
                        // ele.label.score = pt->score;
                        ele.label.score = cnt;
                        ele.label.label = group_list->boundary_type; // 临时弄的
                        if (pt->invalid()) {
                            ele.color = {100, 100, 100};
                        }
                        // ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                        ele.label.intensity_opt= group_list->cur_line_id;
                        prev=pt.get();
                    }
                }
                // trail_map[lit1->first].push_back(log);
                cnt++;
            }
        }
    }

    session->save_debug_info(total_log_name.c_str());

    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::save_debug_info_only_boundary(RoadModelSessionData* session, int index) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    std::string total_log_name = utils::fmt("merge_feature_boundary_" + std::to_string(index));
    session->set_display_name(total_log_name.c_str());

    // UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;
    // for (auto &it : session->key_pose_map) {
    auto lit1 = session->key_pose_map.begin();
    if (lit1 != session->key_pose_map.end()) {
        auto trail_ptr = &lit1->second;

        for (int64_t i = 0; i < trail_ptr->boundary_line_group_list.size(); ++i) {
            auto &group_list = trail_ptr->boundary_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_boundary");
            log->color = {81, 89, 240};
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);
                } else {
                    prev=pt.get();
                }

                if(j < 2 || d>=4 || j==group_list->list.size()-1 || group_list->boundary_type == LaneType::ISLAND_RB) {
                    auto &ele = log->add(pt->pos, 1);
                    ele.label.opt_label = i;
                    ele.label.intensity_opt = j;
                    ele.label.score = pt->score;
                    if (pt->invalid()) {
                        ele.color = {255, 255, 100};
                    }
                    ele.label.label = 1; // 临时弄的
                    prev=pt.get();
                }  
            }

            // trail_map[lit1->first].push_back(log);
        }
    }

    session->save_debug_info(total_log_name.c_str());

    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::save_frame_log_lane(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }

    if (!FLAGS_merge_feature_save_data_frame_log_enable) {
        return fsdmap::SUCC;
    }
    std::vector<std::shared_ptr<utils::DisplayInfo>> ptr_vec;
    std::vector<utils::DisplayInfo*> log_vec;
    for (int64_t i = 0; i < line->lane_line_group_list.size(); ++i) {
        auto &group_list = line->lane_line_group_list[i];
        auto log = session->add_ptr(ptr_vec);
        log->type = utils::DisplayInfo::LINE;
        log->color = {0, 255, 0};
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            // session->debug_pos(pt->pos);
            auto &ele = log->add(pt->pos, 1, group_list->match_list[i].size());
            ele.label.opt_label = i;
            if (pt->invalid()) {
                ele.color = {255, 255, 0};
            }
          
        }
        log_vec.push_back(log.get());
    }
    auto &ins_vec = session->bev_frame_lane_line[poss->frame_id];
    for (auto &ins : ins_vec) {
        auto mlog = session->add_ptr(ptr_vec);
        mlog->type = utils::DisplayInfo::LINE;
        mlog->color = {100, 100, 125};
        for (auto &pt : ins->list) {
            // session->debug_pos(pt->pos);
            auto &ele = mlog->add(pt->pos);
            if (pt->invalid()) {
                ele.color = {255, 0, 0};
            }
        }
        log_vec.push_back(mlog.get());
    }
    
    auto log_name = session->get_debug_dir("merge_detail/{}_{}_{}.png", "merge_frame_lane", 
            times, poss->frame_id);

    auto pcd_name = session->get_debug_dir("merge_detail/{}_{}_{}.pcd", "merge_frame_lane", 
            times, poss->frame_id);

    if (poss == line->list.back()) {
        log_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.png", "merge_frame_lane", 
                times, poss->frame_id);
        pcd_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.pcd", "merge_frame_lane", 
                times, poss->frame_id);
    }
    utils::save_display_image(log_name.c_str(), session->_scope, log_vec);
    utils::save_display_pcd(pcd_name.c_str(), session->_scope, log_vec);

    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::save_frame_log_boundary(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times, std::string line_id) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    if (!FLAGS_merge_feature_save_data_frame_log_enable) {
        return fsdmap::SUCC;
    }
    std::vector<std::shared_ptr<utils::DisplayInfo>> ptr_vec;
    std::vector<utils::DisplayInfo*> log_vec;
    for (int64_t i = 0; i < line->boundary_line_group_list.size(); ++i) {
        auto &group_list = line->boundary_line_group_list[i];
        auto log = session->add_ptr(ptr_vec);
        log->type = utils::DisplayInfo::LINE;
        if (i == line->boundary_line_group_list.size()-1) {
            log->color = {255, 0, 255};
        } else {
            log->color = {0, 255, 0};
        }
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            // session->debug_pos(pt->pos);
            auto &ele = log->add(pt->pos, 1, group_list->match_list[i].size());
            ele.label.opt_label = i;
            if (pt->invalid()) {
                ele.color = {255, 255, 0};
            }
            if(j == 0) {
                // auto line_index_log = session->add_ptr(ptr_vec);
                // log->type = utils::DisplayInfo::LINE_INDEX;
                // log_vec.push_back(line_index_log.get());
                // 添加车道组的索引日志，并设置为红色
                // auto line_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_id:{}_{}", i, group_list->id);
                auto line_index_log = session->add_debug_log(utils::DisplayInfo::TEXT, "line_id:{}", group_list->id);
                line_index_log->color = {255, 0, 0}; // 红色
                line_index_log->add(pt->pos); // 对第一个点添加到车道组索引日志
                log_vec.push_back(line_index_log);
            }
        }
        log_vec.push_back(log.get());

        // for (int64_t j = 0; j < group_list->list.size(); ++j) {
        // {
        //     if(j == 0) {
        //         // auto line_index_log = session->add_ptr(ptr_vec);
        //         // log->type = utils::DisplayInfo::LINE_INDEX;
        //         // log_vec.push_back(line_index_log.get());
        //         // 添加车道组的索引日志，并设置为红色
        //         // auto line_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_id:{}_{}", i, group_list->id);
        //         auto line_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_id:{}", group_list->id);
        //         line_index_log->color = {255, 0, 0}; // 红色
        //         line_index_log->add(pt->pos); // 对第一个点添加到车道组索引日志
        //         line_index_log->add(pt->pos); // 对第一个点添加到车道组索引日志
        //         // log_vec.push_back(line_index_log);
        //     }
        // }
    }

    // auto &ins_vec = session->bev_frame_boundary_line[poss->frame_id];
    auto &ins_vec = session->bev_frame_boundary_line[""];
    for (auto &ins : ins_vec) {
        auto mlog = session->add_ptr(ptr_vec);
        mlog->color = {100, 100, 125};
        mlog->type = utils::DisplayInfo::LINE;
        for (auto &pt : ins->list) {
            // session->debug_pos(pt->pos);
            auto &ele = mlog->add(pt->pos);
            if (pt->invalid()) {
                ele.color = {255, 0, 0};
            }
        }
        log_vec.push_back(mlog.get());
    }
    auto log_name = session->get_debug_dir("merge_detail/{}_{}_{}.png", "merge_frame_boundary", 
            times, line_id);
    auto pcd_name = session->get_debug_dir("merge_detail/{}_{}_{}.pcd", "merge_frame_boundary", 
            times, line_id);

    // auto log_name = session->get_debug_dir("merge_detail/{}_{}_{}.png", "merge_frame_boundary", 
    //         times, poss->frame_id);
    // auto pcd_name = session->get_debug_dir("merge_detail/{}_{}_{}.pcd", "merge_frame_boundary", 
    //         times, poss->frame_id);
    // if (poss == line->list.back()) {
    //     log_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.png", "merge_frame_boundary", 
    //             times, poss->frame_id);
    //     pcd_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.pcd", "merge_frame_boundary", 
    //             times, poss->frame_id);
    // }
    utils::save_display_image(log_name.c_str(), session->_scope, log_vec);
    utils::save_display_pcd(pcd_name.c_str(), session->_scope, log_vec);

    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::save_frame_log_lane_center(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    if (!FLAGS_merge_feature_save_data_frame_log_enable) {
        return fsdmap::SUCC;
    }
    std::vector<std::shared_ptr<utils::DisplayInfo>> ptr_vec;
    std::vector<utils::DisplayInfo*> log_vec;
    for (int64_t i = 0; i < line->lane_center_line_group_list.size(); ++i) {
        auto &group_list = line->lane_center_line_group_list[i];
        auto log = session->add_ptr(ptr_vec);
        log->type = utils::DisplayInfo::LINE;
        log->color = {0, 255, 0};
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            // session->debug_pos(pt->pos);
            auto &ele = log->add(pt->pos, 1, group_list->match_list[i].size());
            ele.label.opt_label = i;
            if (pt->invalid()) {
                ele.color = {255, 255, 0};
            }
        }
        log_vec.push_back(log.get());
    }
    auto &ins_vec = session->bev_frame_lane_center_line[poss->frame_id];
    for (auto &ins : ins_vec) {
        auto mlog = session->add_ptr(ptr_vec);
        mlog->color = {100, 100, 125};
        mlog->type = utils::DisplayInfo::LINE;
        for (auto &pt : ins->list) {
            // session->debug_pos(pt->pos);
            auto &ele = mlog->add(pt->pos);
            if (pt->invalid()) {
                ele.color = {255, 0, 0};
            }
        }
        log_vec.push_back(mlog.get());
    }

    auto log_name = session->get_debug_dir("merge_detail/{}_{}_{}.png", "merge_frame_lane_center", 
            times, poss->frame_id);
    auto pcd_name = session->get_debug_dir("merge_detail/{}_{}_{}.pcd", "merge_frame_lane_center", 
            times, poss->frame_id);
    if (poss == line->list.back()) {
        log_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.png", "merge_frame_lane_center", 
                poss->frame_id, times);
        pcd_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.pcd", "merge_frame_lane_center", 
                times, poss->frame_id, times);
    }
    utils::save_display_image(log_name.c_str(), session->_scope, log_vec);
    utils::save_display_pcd(pcd_name.c_str(), session->_scope, log_vec);

    return fsdmap::SUCC;
}

}
}

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
