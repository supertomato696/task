

#include "utils/algorithm_util.h"
#include "road_model_proc_build_boundary.h"
namespace fsdmap {
namespace road_model {

DEFINE_bool(build_boundary_enable, true, "build_boundary_enable");
DEFINE_bool(build_boundary_debug_pos_enable, true, "build_boundary_debug_enable");
DEFINE_bool(build_boundary_save_data_enable, true, "build_boundary_save_data_enable");

DEFINE_bool(build_boundary_use_sem_enable, false, "build_boundary_use_sem_enable");
DEFINE_bool(build_boundary_use_trail_enable, true, "build_boundary_use_trail_enable");
DEFINE_double(build_boundary_max_same_dir_feature_radius, 50, "build_boundary_max_same_dir_feature_radius");
DEFINE_double(build_boundary_opposite_pos_same_dir_theta, 30, "build_boundary_opposite_pos_same_dir_theta");
DEFINE_double(build_boundary_opposite_pos_scope_threshold, 3, "build_boundary_opposite_pos_scope_threshold");
DEFINE_double(build_boundary_max_same_pos_cb_theta, 60, "build_boundary_max_same_dir_theta");
DEFINE_double(build_boundary_mark_yellow_dis_threshold, 1, "build_boundary_mark_yellow_dis_threshold");
DEFINE_double(build_boudnary_yellow_spread_length, 20, "build_boudnary_yellow_spread_length");
DEFINE_double(build_boudnary_yellow_spread_gap_threshold, 1, "build_boudnary_yellow_spread_gap_threshold");
DEFINE_double(build_boundary_rc_scope_dis_threshold, 1, "build_boundary_rc_scope_dis_threshold");
DEFINE_double(build_boundary_smooth_variance_lambda, 0.3, "build_boundary_smooth_variance_lambda");
DEFINE_double(build_boundary_road_center_min_width, 2.7, "build_boundary_road_center_min_width");
DEFINE_double(build_boundary_smooth_road_center_scope, 25, "build_boundary_smooth_road_center_scope");
DEFINE_double(build_boundary_link_dir_rate_threshold, 0.7, "build_boundary_link_dir_rate_threshold");
DEFINE_double(build_boundary_match_lidar_rate_2, 0.3, "build_boundary_match_lidar_rate_2");
DEFINE_double(build_boundary_match_lidar_rate_1, 0.7, "build_boundary_match_lidar_rate_1");
DEFINE_double(build_boundary_search_poss_search_radius, 5, "build_boundary_search_poss_search_radius");
DEFINE_double(build_boundary_search_poss_search_scope_h_1, 5, "build_boundary_search_poss_search_scope_h_1");
DEFINE_double(build_boundary_search_poss_search_scope_v_1, 0.3, "build_boundary_search_poss_search_scope_v_1");
DEFINE_double(build_boundary_search_poss_search_scope_h_2, 5, "build_boundary_search_poss_search_scope_h_2");
DEFINE_double(build_boundary_search_poss_search_scope_v_2, 1.5, "build_boundary_search_poss_search_scope_v_2");
DEFINE_double(build_boundary_tracking_dis_threshold, 2, "build_boundary_tracking_dis_threshold");
DEFINE_double(build_boundary_tracking_max_theta, 15, "build_boundary_tracking_max_theta");

DEFINE_double(build_boundary_mark_yellow_road_width_threshold, 7, "build_boundary_mark_yellow_road_width_threshold");
DEFINE_double(build_boundary_mark_yellow_min_width_threshold, 3, "build_boundary_mark_yellow_min_width_threshold");
DEFINE_double(build_boundary_rebuild_min_dis_threshold, 4, "build_boundary_rebuild_min_dis_threshold");
DEFINE_double(build_boundary_pos_match_lane_radius, 30, "bind_trail_pos_match_lane_radius");
DEFINE_double(build_boundary_pos_match_lane_z_max, 6, "bind_trail_pos_match_lane_z_max");
DEFINE_double(build_boundary_pos_match_lane_z_min, -2, "bind_trail_pos_match_lane_z_min");
DEFINE_double(build_boundary_pos_match_lane_theta, 30, "bind_trail_pos_match_lane_theta");
DEFINE_string(road_branch, "BYD", "road_branch");


fsdmap::process_frame::PROC_STATUS RoadModelProcBuildBoundary::proc(
        RoadModelSessionData* session) {
     //
    CHECK_FATAL_PROC(search_boundary(session), "search_boundary");
     // 
    CHECK_FATAL_PROC(build_road_center(session), "build_road_center");
    // 结果可视化
    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcBuildBoundary::search_boundary(RoadModelSessionData *session)
{
    for (auto link : session->link_sample_list)
    {
        for (auto poss : link->list)
        {
            session->thread_pool->schedule([poss, session, this](utils::ProcessBar *process_bar)
                                           {
                session->debug_pos(poss->pos);
                search_nearest_boundary(session, poss); });
        }
    }
    session->thread_pool->wait();
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::build_road_center(RoadModelSessionData *session)
{
    for (auto link : session->link_sample_list)
    {
        for (auto poss : link->list)
        {
            // session->thread_pool->schedule([poss, session, this](utils::ProcessBar *process_bar)
            // {
            // session->debug_pos(poss->pos);
            build_road_center_by_poss(session, poss);
            // });
        }
    }
    // session->thread_pool->wait();
    return fsdmap::SUCC;
}

bool link_pruning(RoadModelSessionData* session,RoadCenterFeature* center ,KeyPose* poss,double radius)
{
    const auto pl = center->left->pos;
    const auto pr = center->right->pos;
    std::vector<KeyPose *> secs;
    session->link_pos_tree.search(poss->pos, radius, secs);
    for (auto sec : secs)
    {
        if(sec->from_link==poss->from_link)
        {
            continue;
        }
        auto neightbors = {sec->prev, sec->next}; 
        for (auto neightbor : neightbors)
        {
            if (!neightbor)
            {
                continue;
            }
            // 排除自己
            Eigen::Vector3d cross_point;
            Eigen::Vector3d dir=neightbor->pos-sec->pos;
            if(dir.norm()<0.1)
            {
                LOG_WARN("link_to_short");
                continue;
            }
            auto p1=sec->pos-dir*0.5;
            auto p2= neightbor->pos+dir*0.5;
            // if (alg::findIntersection(sec->pos, neightbor->pos, pl, pr, cross_point))
             if (alg::findIntersection(p1,p2, pl, pr, cross_point))
            {
                // LOG_INFO("link_pruning");
                return true;
            }
        }
    }
    return false;
}
// 根据轨迹左右两侧的Boundary位置来推算车道中心线，
// 当poss轨迹点左右两侧都有boundary时，车道中心点在当前轨迹点poss两侧左右围栏上两个点的平均值，
// 当只有左侧围栏点时，左侧围栏点在poss->dir垂线方向上向右延伸2m就是车道中心点，
// 当只有有的围栏点时，右侧围栏点在poss->dir垂线方向上向左延伸2m就是车道中心点
int RoadModelProcBuildBoundary::build_road_center_by_poss(RoadModelSessionData* session, 
        KeyPose* poss) {
    double min_width = FLAGS_build_boundary_road_center_min_width; // 2.7，最窄的车道宽度
    BoundaryFeature* prev_fls = NULL;
    auto &boundary = poss->boundary;
    std::vector<BoundaryFeature*> valid_list;
    for (int i = 0; i < boundary.boundary_feature_list.size(); ++i) {
        auto &fls = boundary.boundary_feature_list[i];
        // if (FLAGS_build_boundary_use_sem_enable 
        //         && fls->src_status != 2 
        //         && fls->src_status != 4) {
        //     continue;
        // }
        double width = 0;
        if (prev_fls != NULL) {
            width = fls->bind_dis - prev_fls->bind_dis;
            if (width < min_width) {
                prev_fls = fls;
                // if(i == boundary.boundary_feature_list.size() - 1) {
                //     valid_list.push_back(fls);
                // }
                continue;
            }
        }else
        {
            prev_fls=fls;
            continue;
        }
        // valid_list.push_back(fls);
        auto rc = session->add_ptr(session->road_center_ptr);
        rc->left = prev_fls; // 车道中心点的左侧围栏点
        rc->right = fls;
        rc->width = width;
        double diff=100;
        double pruning_radius=0;
        if(rc->left!=NULL&&rc->right!=NULL )
        {
            diff=alg::calc_dis(rc->left->pos,rc->right->pos);
            pruning_radius=diff*1.414;
            diff-=alg::calc_dis(poss->pos,rc->left->pos);
            diff-=alg::calc_dis(poss->pos,rc->right->pos);
        }
        if(fabs(diff)<3)
        {
            // 剪枝
             if(!link_pruning(session,rc.get(),poss,pruning_radius))
             {
                make_road_center(session, poss, rc.get()); // 计算车道中心点位置
                boundary.curr=rc.get();
             }
        }
        prev_fls = fls;
    }
    // only for not crash
    if(!boundary.curr)
    {
        auto rc = session->add_ptr(session->road_center_ptr);
        boundary.curr=rc.get();
    }
    // 
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::make_road_center(RoadModelSessionData* session, 
        KeyPose* poss, RoadCenterFeature* rc) {
    auto &boundary = poss->boundary;
    if (rc != boundary.curr) {
        boundary.road_center_list.push_back(rc);  // rc本身在存储过程中已经是从左到右的排序，所以road_center_list的存储顺序也是从左到右的
    }
    rc->key_pose = poss;
    if (rc->left != NULL && rc->right != NULL) {
        // rc->width = fabs(fls->bind_dis - prev_fls->bind_fls);
        rc->theta = alg::calc_theta(rc->left->dir, rc->right->dir);
        rc->pos = (rc->left->pos + rc->right->pos) / 2; // 当前轨迹点poss两侧左右围栏上两个点的平均值
        rc->dir = poss->dir;
    } else if (rc->left != NULL) { // 只有左侧围栏点
        rc->pos = alg::get_vertical_pos(rc->left->pos, poss->dir, 2, false); // 左侧围栏点在poss->dir垂线方向上向右延伸2m就是车道中心点
        rc->dir = poss->dir;
    } else if (rc->right != NULL) { // 只有右侧围栏点
        rc->pos = alg::get_vertical_pos(rc->right->pos, poss->dir, -2, false); // 右侧围栏点在poss->dir垂线方向上向左延伸2m就是车道中心点
        rc->dir = poss->dir;
    } else { // 左右两边都没有围栏点，就用当前poss的位置作为车道中心点
        rc->pos = poss->pos;
        rc->dir = poss->dir;
    }
    rc->bind_dis = alg::calc_vertical_dis(rc->pos, poss->pos, poss->dir, true);
    return fsdmap::SUCC;
}

// 在遍历road_segment_list里面每段road_segment的每个位置poss时，搜索离它50范围内的，朝向角度差在60度范围内的boundary点，
// 遍历这些筛选出的boundary点，对于每个点fls，计算poss(轨迹点)处方向向量的垂线与fls（车道线上的点）的方向向量的交点cross point，
// 这些cross point会放入poss->boundary
int RoadModelProcBuildBoundary::search_nearest_boundary(RoadModelSessionData* session, KeyPose* poss) {
    double max_same_dir_theta = FLAGS_build_boundary_max_same_pos_cb_theta; // 60
    float radius = FLAGS_build_boundary_max_same_dir_feature_radius; // 50 m 
    auto pos = poss->pos;
    std::vector<BoundaryFeature*> search_sec;
    session->boundary_line_sample_tree.search(pos, radius, search_sec);
    Eigen::Vector3d dir_get;
    auto &cb = poss->boundary;
    DisDirPoint cross_point;
    Eigen::Vector3d poss_v_pt = alg::get_vertical_pos(poss->pos, poss->dir, +radius, false);
    for (auto& fls : search_sec) {
        // if (FLAGS_build_boundary_use_sem_enable ^ (fls->src_status == 2)) {
        //     continue;
        // }
        session->debug_pos2(poss->pos, fls->pos);
        if (fls->next == NULL) {
            continue;
        }
        double dis = alg::calc_dis(pos, fls->pos);
        if (dis > radius) {
            continue;
        }

        // 无论角度如何(60度以内，如果是反向，那就是大于120度)，都是护栏
        double theta = alg::calc_theta(fls->dir, poss->dir);
        if (theta > max_same_dir_theta && 180 - theta > max_same_dir_theta) {
            continue;
        }

        if (!alg::get_cross_point_by_point(fls->pos, fls->next->pos,
                    poss->pos, poss_v_pt, cross_point.pos, false, 2, 1000)) {
            continue;
        }
        double c_dis = alg::calc_dis(poss->pos, cross_point.pos);
        if (alg::judge_left(cross_point.pos, pos, poss->dir) < 0) { // 判断cross_point.pos是否在pos的左侧
            c_dis = -c_dis;
        }
        auto flsr = session->add_ptr(session->boundary_feature_ptr, true);
        flsr->init(fls);
        flsr->bind_dis = c_dis;
        flsr->pos = cross_point.pos;
        flsr->key_pose = poss;
        flsr->attr_feature = fls;
        if (theta > max_same_dir_theta) {
            flsr->dir = -flsr->dir;
        }
        cb.boundary_feature_list.push_back(flsr.get());
    }
    SORT(cb.boundary_feature_list, 
            [](const BoundaryFeature* l, const BoundaryFeature* r)->bool{
                return l->bind_dis < r->bind_dis;
            });
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::save_debug_info(RoadModelSessionData *session)
{
    if (!FLAGS_build_boundary_save_data_enable)
    {
        return fsdmap::SUCC;
    }
    session->set_display_name("build_boundary");
    for (auto link : session->link_sample_list)
    {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {223, 130, 154};
        for (auto poss : link->list)
        {
            session->debug_pos(poss->pos);
            auto &ele = log->add(poss->pos);
            ele.label.label = poss->filter_status;

            // if(alg::match_any_with_forms(poss->from_raw_link->forms, {25, 26})){
            //     LOG_INFO("-----------BUILD_BOUNDARY: [ valid - {}]-----------", poss->invalid());
            // }

            if (poss->invalid())
            {
                ele.color = {255, 0, 0};
                continue;
            }
            auto &boundary = poss->boundary;
            
            // if(alg::match_any_with_forms(poss->from_raw_link->forms, {25, 26})){
            //     LOG_INFO("-----------BUILD_BOUNDARY: [ size - {}]-----------", boundary.road_center_list.size());
            // }

            for (auto &rc : boundary.road_center_list)
            {
                cv::Scalar color = {100, 100, 100};
                if (rc->invalid() || poss->invalid())
                {
                    color = {255, 0, 0};
                }
                else if (rc == boundary.curr)
                {
                    color = {255, 255, 255};
                }
                if (rc->left != NULL)
                {
                    auto slog = session->add_debug_log(
                        utils::DisplayInfo::LINE_INDEX, "road_center_left");
                    slog->color = color;
                    slog->color[2] = 0;
                    if (rc->left->src_status == 1 && rc == boundary.curr)
                    {
                        slog->color = {0, 255, 0}; //绿色
                    }
                    slog->add(rc->pos);
                    slog->add(rc->left->pos);
                }

                // if(alg::match_any_with_forms(poss->from_raw_link->forms, {25, 26})){
                //     bool cond1 = rc->left != NULL;
                //     bool cond2 = rc->right != NULL;

                //     LOG_INFO("-----------BUILD_BOUNDARY: [ left - {},   right - {}]-----------", cond1, cond2);
                // }

                if (rc->right != NULL)
                {
                    auto rlog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                                                       "road_center_right");
                    rlog->color = color;
                    rlog->color[2] = 125;
                    if (rc->right->src_status == 1 && rc == boundary.curr)
                    {
                        rlog->color = {0, 255, 255};//青绿色
                    }
                    rlog->add(rc->pos);
                    rlog->add(rc->right->pos);
                }
                if (rc->yellow_boundary.src != NULL)
                {
                    auto ylog = session->add_debug_log(utils::DisplayInfo::POINT,
                                                       "road_center_yellow");
                    ylog->color = {0, 255, 0}; //绿色
                    ylog->add(rc->yellow_boundary.src->pos);
                    auto next_pos = alg::get_hori_pos(rc->yellow_boundary.src->pos, rc->yellow_boundary.src->dir, 1);
                    ylog->add(next_pos);
                }
            }
        }
    }
    //
    int count = 0;
    for (auto line : session->merge_boundary_line_list)
    {
        BoundaryFeature *prev = NULL;
        auto llog = session->add_debug_log(utils::DisplayInfo::LINE, "road_boundary_{}", count++);
        llog->color = {81, 89, 240};
        for (auto pt : line->list)
        {

            auto &ele = llog->add(pt->pos);
        }
    }
    //
    session->save_debug_info("build_boundary");
    return fsdmap::SUCC;
}
}
}


/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
