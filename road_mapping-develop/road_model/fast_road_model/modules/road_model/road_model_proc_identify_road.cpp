

#include "road_model_proc_identify_road.h"
#include "utils/algorithm_util.h"

namespace fsdmap {
namespace road_model {

DEFINE_bool(identify_road_enable, true, "identify_road_enable");
DEFINE_bool(identify_road_debug_pos_enable, true, "identify_road_debug_enable");
DEFINE_bool(identify_road_save_data_enable, true, "identify_road_save_data_enable");

DEFINE_double(identify_road_same_lane_dis_threshold, 4, "identify_road_same_lane_dis_threshold");
DEFINE_double(identify_road_same_lane_theta_threshold, 10, "identify_road_same_lane_theta_threshold");
DEFINE_double(identify_road_same_lane_max_width_threshold, 1, "identify_road_same_lane_max_width_threshold");
DEFINE_double(identify_road_same_lane_min_width_threshold, 0.3, "identify_road_same_lane_min_width_threshold");
DEFINE_double(identify_road_identify_null_lane_postion, 0.5, "identify_road_identify_null_lane_postion");
DEFINE_double(identify_road_identify_normal_lane_variance_threshold, 30, "identify_road_identify_normal_lane_variance_threshold");
DEFINE_double(identify_road_identify_triangle_lane_variance_threshold, 40, "identify_road_identify_triangle_lane_variance_threshold");
DEFINE_double(identify_road_identify_triangle_lane_a_threshold, 0.1, "identify_road_identify_triangle_lane_a_threshold");
DEFINE_double(identify_road_identify_pos_length, 20, "identify_road_identify_pos_length");
DEFINE_double(identify_road_identify_trangle_length, 50, "identify_road_identify_trangle_length");
DEFINE_double(identify_road_identify_inner_merge_barrier_max_length, 100, "identify_road_identify_inner_merge_barrier_max_length");
DEFINE_double(identify_road_identify_merge_width_threshold, 2, "identify_road_identify_merge_width_threshold");
DEFINE_double(identify_road_identify_merge_width_delta_factor, 1.5, "identify_road_identify_merge_width_delta_factor");
DEFINE_double(identify_road_poss_gap_threshold, 8, "identify_road_poss_gap_threshold");
DEFINE_double(identify_road_get_segment_max_distance, 50, "identify_road_get_segment_max_distance");
DEFINE_double(identify_road_identify_triangle_lane_expect_max_dis, 2, "identify_road_identify_triangle_lane_expect_max_dis");
DEFINE_double(identify_road_identify_triangle_lane_expect_max_value, 10, "identify_road_identify_triangle_lane_expect_max_value");
DEFINE_double(identify_road_get_break_point_max_length, 30, "identify_road_get_break_point_max_length");
DEFINE_double(identify_road_identify_poss_buff_length, 10, "identify_road_identify_poss_buff_length");
DEFINE_double(identify_road_identify_attr_max_length, 30, "identify_road_identify_attr_max_length");
DEFINE_double(identify_road_identify_attr_min_length, 10, "identify_road_identify_attr_min_length");
DEFINE_double(identify_road_cross_walk_gap, 2, "identify_road_cross_walk_gap");
DEFINE_double(identify_road_get_stop_poss_scope, 10, "identify_road_get_stop_poss_scope");
DEFINE_double(identify_road_stop_line_lc_gap, 4, "identify_road_stop_line_lc_gap");
DEFINE_double(identify_road_stop_line_inner_pos, true, "identify_road_stop_line_inner_pos");
DEFINE_double(identify_road_stop_line_last_gap, 20, "identify_road_stop_line_last_gap");
DEFINE_double(identify_road_nearest_fork_radius, 20, "identify_road_nearest_fork_radius");
DEFINE_double(identify_road_nearest_fork_v_radius, 4, "identify_road_nearest_fork_v_radius");
DEFINE_double(identify_road_nearest_fork_max_theta, 20, "identify_road_nearest_fork_max_theta");
DEFINE_double(identify_road_lc_rate_rate_threshold, 0.5, "identify_road_lc_rate_rate_threshold");
// DECLARE_double(filter_feature_cb_overflow_lambda);
DEFINE_double(priority_range, 0.5, "priority_range");

fsdmap::process_frame::PROC_STATUS RoadModelProcIdentifyRoad::proc(
        RoadModelSessionData* session) {
    
    // CHECK_FATAL_PROC(gen_lane_directin(session),"gen_lane_directin");
    
    // 
    // CHECK_FATAL_PROC(gen_split_merge_candidate(session),"gen_split_merge_candidate");

    // 
    CHECK_FATAL_PROC(gen_lane_break_candidate(session),"gen_lane_break_candidate");
    // 结果可视化
    CHECK_FATAL_PROC(gen_stop_line_break_candidate(session),"gen_stop_line_break_candidate");
    // 
    CHECK_FATAL_PROC(merge_break_candidate(session),"merge_break_candidate");
    CHECK_FATAL_PROC(merge_break_candidate_type(session),"merge_break_candidate_type");
    
    // 
    CHECK_FATAL_PROC(update_break_staus_to_pose(session),"update_break_staus_to_pose");
    // 
    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    // 
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

//  TODO insert new break pose to link 
int RoadModelProcIdentifyRoad::update_break_staus_to_pose(RoadModelSessionData *session)
{
    std::map<int, std::vector<BreakCandiate>> hash_tabel;
    for (auto bk : break_candidates)
    {
        hash_tabel[bk.same_id].push_back(bk);
    }
    for (auto p : hash_tabel)
    {
        int index = -1;
        double min_dis = 1E3;
        int size = p.second.size();
        for (int i = 0; i < size; i++)
        {
            auto bc = p.second[i];
            double dis = alg::calc_dis(bc.pos, bc.raw_cross_point);
            if (dis < min_dis)
            {
                min_dis = dis;
                index = i;
            }
            // 强制使用停止线
            if(bc.type==6)
            {
                min_dis = dis;
                index = i;
                break;
            }
            // LOG_INFO("dis:{}  pos:[{}   {}] raw_cross:[{} {} ] index:{}",dis,bc.pos.x(),bc.pos.y(),bc.raw_cross_point.x(),bc.raw_cross_point.y(),index)
        }
        if (index < 0)
        {
            LOG_WARN("update_break_staus_to_pose index <0")
            continue;
        }
        // LOG_INFO("min_dis:{}  index:{}",min_dis,index)
        auto & break_candidate= p.second[index];
        auto &caidate_pose =break_candidate.raw_cross_pose;
        if (!caidate_pose)
        {
            LOG_WARN("caidate_pose NULL")
        }
        std::shared_ptr<RoadBreak> new_road_break = std::make_shared<RoadBreak>();
        new_road_break->from_pose = caidate_pose;
        new_road_break->objs=break_candidate.objs;
        new_road_break->pos= break_candidate.pos;
        switch(break_candidate.type)
        {
            case 1:
            {
                new_road_break->reasons="lane_end";
                break;
            }
            case 3:
            {
                new_road_break->reasons="split_merge";
                break; 
            }
            case 6:
            {
                new_road_break->reasons="stop_line";
                break; 
            }
        }
        caidate_pose->road_break = new_road_break.get();
        session->road_break_ptr.push_back(new_road_break);
        // LOG_INFO("caidate_pose:[{} {}]",caidate_pose->pos.x(),caidate_pose->pos.y());
    }
    return fsdmap::SUCC;
}

int RoadModelProcIdentifyRoad::gen_stop_line_break_candidate(RoadModelSessionData *session)
{
    auto get_stop_line = [](KeyPose *poss) -> RoadObjectInfo *
    {
        for (auto obj : poss->object_list)
        {
            if (obj->ele_type == 6)
            {
                return obj;
            }
        }
        return NULL;
    };

    auto refine_new_pos = [session](KeyPose *kp, Eigen::Vector3d pos, Eigen::Vector3d dir)->KeyPose *
    {
        kp->pos=pos;
        kp->road_vertical_dir = dir;
        // KeyPose *ret=NULL;
        // auto &from_link = prev->from_link;
        // auto &from_raw_link = prev->from_raw_link;
        // std::vector<KeyPose *> temp;
        // for (auto poss : from_link->list)
        // {
        //     temp.push_back(poss);
        //     if (poss == prev)
        //     {
        //         //
        //         auto new_kp = session->add_ptr(session->key_pose_ptr);
        //         new_kp->pos = pos;
        //         new_kp->dir = prev->dir;
        //         new_kp->init(prev);
        //         new_kp->road_vertical_dir = dir;
        //         new_kp->from_link = from_link;
        //         new_kp->from_raw_link = from_raw_link;
        //         new_kp->centers = prev->centers;
        //         temp.push_back(new_kp.get());
        //         LOG_INFO("insert stop line poss[{} {}]", pos.x(), pos.y());
        //         //
        //         ret=new_kp.get();
        //     }
        // }
        // std::swap(temp, from_link->list);
       return  kp;
    };
    std::vector<KeyPose *> vkp;
    for (auto link : session->link_sample_list)
    {
        if (!link)
        {
            continue;
        }
        for (auto poss : link->list)
        {
            if (!poss)
            {
                continue;
            }
            auto stop_line = get_stop_line(poss);
            if (stop_line)
            {
                vkp.push_back(poss);
            }
        }
    }
    LOG_INFO("vkp_size:{}", vkp.size());
    for (auto kp : vkp)
    {
        auto stop_line = get_stop_line(kp);
        if (!stop_line)
        {
            continue;
        }
        if (stop_line->list.size() < 2)
        {
            continue;
        }
        double theta = alg::calc_theta1(stop_line->dir, kp->road_vertical_dir, true);
        theta = std::min(180 - theta, theta);
        if (theta > 45)
        {
            continue;
        }
        auto ps = stop_line->list[0]->pos;
        auto pe = stop_line->list[1]->pos;
        KeyPose *stop_pose = NULL;
        Eigen::Vector3d cross;
        auto neigbors = {kp->prev, kp->next};
        for (auto next : neigbors)
        {
            if (!next)
            {
                continue;
            }
            if (alg::findIntersection(ps, pe, next->pos, kp->pos, cross))
            {
                double d1 = alg::calc_dis(next->pos, cross);
                double d2 = alg::calc_dis(kp->pos, cross);
                if (d1 < d2)
                {
                    stop_pose = next;
                }
                else
                {
                    stop_pose = kp;
                }
            }
        }
        if (!stop_pose)
        {
            continue;
        }
        // LOG_INFO("satrt  refine_new_pos ");
        refine_new_pos(stop_pose, cross, stop_line->dir);
        BreakCandiate new_candidate;
        new_candidate.type = 6; // 1、车道线末端点
        new_candidate.raw_dir = stop_line->dir;
        new_candidate.raw_pos = stop_line->pos;
        new_candidate.raw_ps = ps;
        new_candidate.raw_pe = pe;
        new_candidate.raw_cross_point = cross;
        new_candidate.raw_cross_pose = stop_pose; // 原始最近pose
        new_candidate.hit_link = stop_pose->from_link;
        new_candidate.objs.push_back(stop_line);
        // Eigen::Vector3d pos;  //最终合并后坐标
        // Eigen::Vector3d ps;  //方向起点
        // Eigen::Vector3d pe;  //方向终点
        // Eigen::Vector3d dir; //方向
        break_candidates.push_back(new_candidate);
    }
    return fsdmap::SUCC;
}


int RoadModelProcIdentifyRoad::gen_lane_break_candidate(RoadModelSessionData* session)
{
    auto set_cross_point=[session]( BreakCandiate& candidate)->bool 
    {
        const auto pl = alg::get_hori_pos(candidate.raw_pos, candidate.raw_dir, +15);
        const auto pr = alg::get_hori_pos(candidate.raw_pos, candidate.raw_dir, -15);
        std::vector<KeyPose *> secs;
        Eigen::Vector3d pos=candidate.raw_pos;
        session->link_pos_tree.search(pos, 30, secs);
        for (auto sec : secs)
        {
            if (sec->from_link != candidate.hit_link)
            {  
                // LOG_INFO("findIntersection from:{}  hit:{}",uint64_t(sec->from_link),uint64_t(candidate.hit_link));
                continue;
            }
            auto neightbors = {sec->prev, sec->next};
            for (auto neightbor : neightbors)
            {
                if (!neightbor)
                {
                    continue;
                }
                Eigen::Vector3d temp_cross_point;
                Eigen::Vector3d dir = neightbor->pos - sec->pos;
                if (dir.norm() < 0.1)
                {
                    LOG_WARN("link_to_short");
                    continue;
                }
                auto p1 = sec->pos;
                auto p2 = neightbor->pos;
                if (alg::findIntersection(p1, p2, pl, pr, temp_cross_point))
                {
                    // select close
                    if(alg::calc_dis(p1,temp_cross_point)<alg::calc_dis(p2,temp_cross_point))
                    {
                        candidate.raw_cross_pose=sec;
                    }else
                    {
                        candidate.raw_cross_pose=neightbor;
                    }
                    candidate.raw_cross_point=temp_cross_point;
                    return true;
                }
            }
        }
        return false;
    };
    auto creat_candiate = [set_cross_point](LaneCenterFeature *pt, BreakCandiate &new_candidate, int type)->bool 
    {
        new_candidate.type = type;
        new_candidate.lane_dir = pt->dir;
        new_candidate.raw_dir = alg::get_vertical_dir(pt->dir);
        new_candidate.raw_ps = alg::get_vertical_pos(pt->pos, pt->dir, +3.17/4);
        new_candidate.raw_pe = alg::get_vertical_pos(pt->pos, pt->dir, -3.17/4);
        new_candidate.raw_pos = pt->pos;
        new_candidate.hit_link = pt->hit_link;
        if (set_cross_point(new_candidate))
        {
            return true;
        }else
        {
            // TODO 
            LOG_INFO("loss_cross_point:[{} {} ] id:{}", pt->pos.x(), pt->pos.y(),u_int64_t( pt->hit_link));
            return false;
        }
        return false;
       
    };

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_cp(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cross_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    int split_merge_size = session->split_merge_break_points.size();
    if (split_merge_size > 0){
        for (int i = 0; i < split_merge_size; i++) {
            auto bkp = session->split_merge_break_points[i];
            if (bkp->valid == false) {
                continue;
            }
            
            cross_point_cloud->push_back(pcl::PointXYZI(bkp->pos.x(), bkp->pos.y(), bkp->pos.z(), 0));
        }

        if(cross_point_cloud->points.size() > 0) {
            kdtree_cp->setInputCloud(cross_point_cloud);
        }
    }
    
    auto get_cross_point = [kdtree_cp, session, cross_point_cloud](LaneCenterFeature *pt) -> Eigen::Vector3d
    {
        if (cross_point_cloud->empty()){
            return Eigen::Vector3d::Zero();
        }

        std::vector<int> neighbors;
        std::vector<float> dist;

        pcl::PointXYZI tmppoint(pt->pos[0], pt->pos[1], pt->pos[2], 0);
        kdtree_cp->nearestKSearch(tmppoint, 1, neighbors, dist);
        
        if (dist[0] < 10.0) {
            pcl::PointXYZI pt = cross_point_cloud->points[neighbors[0]];
            Eigen::Vector3d vec_pt(pt.x, pt.y, pt.z);
            return vec_pt;
        } else {
            return Eigen::Vector3d::Zero();
        }
    };

    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list) {
        for (auto pt : line->list) {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }

    for(auto line:session->merge_lane_center_list)
    {
        LaneCenterFeature *prev_pt = NULL;
        for (auto pt : line->list)
        {
            if (!pt->hit_link)
            {
                continue;
            }
            BreakCandiate new_candidate;
            bool is_creat = false;
            // case 1: 无前驱后继
            if (!pt->prev && pt->hit_link)
            {
                is_creat = creat_candiate(pt.get(), new_candidate, 1);
            }
            if (!pt->next && pt->hit_link)
            {
                is_creat = creat_candiate(pt.get(), new_candidate, 1);
            }

            auto CPoint = get_cross_point(pt.get());
            if (prev_pt && CPoint.isZero())
            {
                // case 2: link 切换
                if (prev_pt->hit_link != pt->hit_link)
                {

                    bool b1 = false;
                    bool b2 = false;
                    b1 = creat_candiate(prev_pt, new_candidate, 3);
                    if (!b1 || alg::match_any_with_forms(new_candidate.raw_cross_pose->from_raw_link->forms, {25}))
                    {
                        b2 = creat_candiate(pt.get(), new_candidate, 3);
                    }
                    is_creat = is_creat || b1 || b2;
                } else {
                    // case 3: 线性属性变化
                    if ((prev_pt->left_lb.type != pt->left_lb.type && prev_pt->left_lb.type != 0 && pt->left_lb.type != 0) 
                    || (prev_pt->right_lb.type != pt->right_lb.type && prev_pt->right_lb.type != 0 && pt->right_lb.type != 0))
                    {
                        bool b1 = false, b2 = false;
                        b1 = creat_candiate(pt.get(), new_candidate, 2);
                        if (!b1)
                        {
                            b2 = creat_candiate(prev_pt, new_candidate, 2);
                        }
                        is_creat = is_creat || b1 || b2;
                    }
                }
            }

            // case 4: 路口分合流分叉点
            // if (!CPoint.isZero() && pt->hit_link->form != "16" && !(prev_pt && prev_pt->hit_link != pt->hit_link)) {
            if (!CPoint.isZero() 
                && alg::match_any_except_forms(pt->hit_link->forms, {16, 25})
                // && !(prev_pt && prev_pt->hit_link != pt->hit_link)
            ) {

                bool b1 = false;
                BreakCandiate tmp_new_candidate;  // 创建临时变量, 防止生成失败覆盖上述结果
                b1 = creat_candiate(pt.get(), tmp_new_candidate, 3);
                if (b1) {
                    // 0. 使用交叉点位置
                    tmp_new_candidate.raw_pos = CPoint;
                    // 1. 先初步用附近中心线，追踪到道路方向 v1
                    std::vector<Eigen::Vector3d> dirs;
                    std::vector<LaneCenterFeature *> lcfs;
                    lane_center_tree.search(CPoint, 10, lcfs);
                    if (!lcfs.empty()) {
                        for (const auto lc : lcfs) {
                            auto vdir = alg::get_vertical_dir(lc->dir);
                            dirs.push_back(vdir);
                        }
                    }
                    // Eigen::Vector3d road_vertical_dir = alg::get_direction(dirs);
                    tmp_new_candidate.raw_dir = alg::get_direction(dirs);
                    b1 = set_cross_point(tmp_new_candidate);
                    // 2. 初步找到keypose后，利用其road_vertical_dir重新计算更为准确的keypose
                    tmp_new_candidate.raw_dir = tmp_new_candidate.raw_cross_pose->road_vertical_dir;
                    b1 = set_cross_point(tmp_new_candidate);
                    // tmp_new_candidate.raw_cross_point = tmp_new_candidate.raw_cross_pose;
                    // if (new_candidate.raw_cross_pose->from_raw_link->form == "25"){
                    if(alg::match_any_with_forms(tmp_new_candidate.raw_cross_pose->from_raw_link->forms, {25})) {
                        b1 = false;
                    }
                    if (b1) {
                        new_candidate = tmp_new_candidate;
                    }
                }

                is_creat = is_creat || b1;
            }

            // case 5: 处理右转侵入到直行道路的情况; 
            // 用右转link的末端pos点在直行车道线上找到一个打断点
            // LOG_INFO("Previous step  ... ");
            if (1) {
                auto pos = pt->pos;
                float radius = 10.0;
                std::vector<KeyPose*> secs;
                // LOG_INFO("Starting  [00] ");
                // auto it = pt->hit_poss->from_raw_link->forms.size();
                // LOG_INFO("Starting  [01]  hit_link: {} ...", it);
                session->link_pos_tree.search(pos, radius, secs);
                // LOG_INFO("Starting  [02]  {} poss...", secs.size());
                bool b1 = false;
                for (auto n_pos : secs){
                    // LOG_INFO("Starting  [03]  poss_type: {};  is_end: {}...", n_pos->from_raw_link->forms.size(), !n_pos->next);
                    double theta = alg::calc_theta(pt->dir, n_pos->dir, true);
                    double dist = alg::calc_dis(pt->pos, n_pos->pos, true);
                    if (alg::match_any_with_forms(n_pos->from_raw_link->forms, {25}) && (!n_pos->next || !n_pos->prev) && (theta<60 || 360-theta<60)) {
                        // if (!n_pos->next){
                        //     LOG_INFO("Starting  [04-1]  success: {} - {} ...", theta, dist);
                        // } else if (!n_pos->prev) {
                        //     LOG_INFO("Starting  [04-2]  success: {} - {} ...", theta, dist);
                        // }
                        b1 = creat_candiate(pt.get(), new_candidate, 3);
                        if (b1) {
                            new_candidate.raw_pos = n_pos->pos;
                            b1 = set_cross_point(new_candidate);
                            if(alg::match_any_with_forms(new_candidate.raw_cross_pose->from_raw_link->forms, {16, 25})) {
                                b1 = false;
                            }
                            if (b1) {
                                break;
                            }
                        }
                        // LOG_INFO("Starting  [04]  success: {} -- {}  ...", is_creat, b1);
                    }
                }
                is_creat = is_creat || b1;
            }

            if(is_creat && alg::match_any_with_forms(new_candidate.raw_cross_pose->from_raw_link->forms, {25}) && new_candidate.type != 6){
                is_creat = false;
            }

            if (is_creat)
            {
                if(new_candidate.type == 2){
                    break_candidates_type.push_back(new_candidate);
                } else {
                    break_candidates.push_back(new_candidate);
                }
            }

            prev_pt = pt.get();
        }
    }
    //
    return fsdmap::SUCC;
}

bool RoadModelProcIdentifyRoad::get_refined_cross_link(RoadModelSessionData* session, 
                                                    RTreeProxy<KeyPose*, float, 2>& link_tree,
                                                    RTreeProxy<LaneCenterFeature *, float, 2>& lane_center_tree,
                                                    Eigen::Vector3d bk_pos)
{
    auto find_link_cross = [&link_tree](Eigen::Vector3d bk_pos,
                                        Eigen::Vector3d road_vertical_dir){
        bool change_keypose = false;
        int i = 2;
        while(i>0) {
            i--;
            if(i == 0) {
                change_keypose = true;
            }

            auto p1 = alg::get_hori_pos(bk_pos, road_vertical_dir, +20);
            auto p2 = alg::get_hori_pos(bk_pos, road_vertical_dir, -20);
            std::vector<KeyPose*> poses;
            link_tree.search(bk_pos,30,poses);
            KeyPose* nearst_pose = NULL;
            double min_dis = DBL_MAX;
            for (auto& pose : poses) {
                if(!pose) {
                    continue;
                }
                double dis_tmp = alg::calc_dis(pose->pos, bk_pos);
                if(dis_tmp < min_dis) {
                    min_dis = dis_tmp;
                    nearst_pose = pose;
                }
            }
            
            if(!nearst_pose || !nearst_pose->next) {
                // std::cout << "2  road_vertical_dir " << road_vertical_dir.transpose() << std::endl;
                return false;
            }

            Eigen::Vector3d cross;
            bool find = alg::get_cross_point_by_point(p1, p2, nearst_pose->pos,nearst_pose->next->pos,cross, false, 3); // TODO:qzc 3 改为 2？
            if(find){
                if(change_keypose) {
                    // 4. 再走一遍 2，获取准确的 link 交叉点 c2，并设置对应的方向为 v2
                    nearst_pose->pos = cross;
                    nearst_pose->road_vertical_dir = road_vertical_dir;
                    // nearst_pose->dir = alg::get_dir(cross, nearst_pose->pos);
                    // std::cout << "change keypose:" << change_keypose 
                    //     << " origin bk_pose " << bk_pos.transpose()
                    //     << " cross: " << cross.transpose() << std::endl;
                    return true;
                } else {
                    // 3. 用 link交叉点的方向，重新设置为道路方向 v2
                    auto poss1 = nearst_pose->pos;
                    auto poss2 = nearst_pose->next->pos;
                    if(alg::calc_dis(poss1, cross) < alg::calc_dis(poss2, cross)) {
                        road_vertical_dir = nearst_pose->road_vertical_dir;
                    } else {
                        road_vertical_dir = nearst_pose->next->road_vertical_dir;
                    }
                    std::cout << "change keypose:" << change_keypose 
                                << " origin road_vertical_dir " << road_vertical_dir.transpose()
                                << " origin bk_pose " << bk_pos.transpose()
                                << " origin poss1 " << poss1.transpose()
                                << " cross: " << cross.transpose() << std::endl;
                }
            }
        }

        return false;
    };

    // 1. 先初步用附近则中心线，追踪到道路方向 v1
    std::vector<Eigen::Vector3d> dirs;
    std::vector<LaneCenterFeature *> lcfs;
    lane_center_tree.search(bk_pos, 10, lcfs);
    if (!lcfs.empty()) {
        for (const auto lc : lcfs) {
            auto vdir = alg::get_vertical_dir(lc->dir);
            dirs.push_back(vdir);
        }
    }
    Eigen::Vector3d road_vertical_dir = alg::get_direction(dirs);

    // 2. 用该道路方向找到 link交叉点 c1
    bool ret = find_link_cross(bk_pos, road_vertical_dir);
    if (ret == false) {
        return false;
    }

    return true;
}


int RoadModelProcIdentifyRoad::gen_split_merge_candidate(RoadModelSessionData* session)
{
    // 3. 修改最近link pose的位置
    RTreeProxy<KeyPose*, float, 2> link_tree; 
    for(auto link: session->link_sample_list) {
         for(auto poss:link->list) {  
            link_tree.insert(poss->pos,poss);
        }
    }

    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list) {
        for (auto pt : line->list) {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }

    for (int i = 0; i < session->split_merge_break_points.size(); i++) {
        auto bk = session->split_merge_break_points[i]; 
        if(!bk || !bk->valid) {
            continue;
        }

        get_refined_cross_link(session, link_tree, lane_center_tree, bk->pos);
    }
    
    return fsdmap::SUCC;
}

bool RoadModelProcIdentifyRoad::check_bc_in_sm_range(RoadModelSessionData* session, BreakCandiate& break_candidate)
{    
    if (break_candidate.type == 3) { // 分合流和交叉点不处理
        return false;
    }
    
    for (auto& smr : session->split_merge_ranges) {
        auto& bk_from = smr.start;
        auto& bk_end = smr.end;

        auto& pose= break_candidate.raw_cross_pose;
        auto p1 = alg::get_hori_pos(pose->pos, pose->road_vertical_dir, +20);
        auto p2 = alg::get_hori_pos(pose->pos, pose->road_vertical_dir, -20);

        auto dir = alg::get_dir(bk_from->pos, bk_end->pos);
        double theta = alg::calc_theta(dir, pose->dir, true); //2.计算方向一致性

        Eigen::Vector3d cross;
        bool find = alg::get_cross_point_by_point(p1, p2, bk_from->pos, bk_end->pos, cross, false, 0);
        if(theta < 90 && find) { // 同方向 且 和分合流区间有交点，则直接过滤掉该断点
            return true;
        }
    }

    return false;
}

int RoadModelProcIdentifyRoad::merge_break_candidate(RoadModelSessionData* session)
{
    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto pt : line->list)
        {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }

    auto merge_candidate=[&lane_center_tree]( std::vector<BreakCandiate*> &one_type)
    {
        Eigen::Vector3d pos={0,0,0};
        Eigen::Vector3d dir={0,0,0};
        int n=one_type.size();
        std::vector<Eigen::Vector3d> dirs;
        // KeyPoseLine *hit_link=NULL;
        BreakCandiate *stop=NULL, *CPoint=NULL;
        int nums_CPoint = 0;
        for(int i=0;i<n;i++)
        {
            if (one_type[i]->type==3) {
                CPoint=one_type[i];
                nums_CPoint = nums_CPoint + 1;
                if (nums_CPoint==1) {
                    pos={0,0,0};
                    dir={0,0,0};
                }
            }
            if(one_type[i]->type==6)
            {
                stop=one_type[i];
                break;
            } 
            if (nums_CPoint > 0 && one_type[i]->type!=3) {
                continue;
            }
            pos=pos.eval()+one_type[i]->raw_cross_point;
            dirs.push_back(one_type[i]->raw_dir);
            // hit_link
        }
        double max_dis=5;
        if(!stop){
            if (nums_CPoint > 0) {
                pos=pos.eval()/static_cast<double>(nums_CPoint);
            } else {
                pos=pos.eval()/static_cast<double>(n);
            }
            // refine dir 
            std::vector<LaneCenterFeature*> lcfs;
            lane_center_tree.search(pos, 10, lcfs);
            if(!lcfs.empty())
            {
                dirs.clear();
                for(const auto lc:lcfs)
                {
                    if (nums_CPoint > 0 && lc->hit_link != CPoint->hit_link){
                        continue;
                    }
                    auto vdir=alg::get_vertical_dir(lc->dir);
                    dirs.push_back(vdir);
                }
            }
            dir=alg::get_direction(dirs);
            for(int i=0;i<n;i++)
            {
                max_dis=std::max(max_dis,alg::calc_dis(one_type[i]->raw_pos,one_type[i]->raw_cross_point));
            }
            if (nums_CPoint > 0) {
                max_dis=20;
            }
        }else
        {
            // LOG_INFO("find stop line ")
            pos=stop->raw_cross_pose->pos;
            dir=stop->raw_dir;
            stop->raw_cross_pose->road_vertical_dir=dir;
            max_dis=20;
        }
        max_dis=max_dis+3.75/2;
        for(int i=0;i<n;i++)
        {
            one_type[i]->pos=pos;
            one_type[i]->ps= alg::get_hori_pos(pos, dir, +max_dis);
            one_type[i]->pe= alg::get_hori_pos(pos, dir, -max_dis);
            if (CPoint) {
                one_type[i]->type=3;
            }
        }

    };
    const double max_align_length = 4 * 1.414;
    RTreeProxy<BreakCandiate *, float, 2> break_candidate_tree;
    for (auto &bc : break_candidates)
    {
        if (bc.raw_cross_pose)
        {
            break_candidate_tree.insert(bc.raw_cross_point, &bc);
        }
    }
    int same_id=0;
    std::set<BreakCandiate *> hash_table;
    for (auto &bc : break_candidates)
    {
        if(check_bc_in_sm_range(session, bc)) {
            continue;
        }

        if (hash_table.count(&bc))
        {
            continue;
        }
        std::vector<BreakCandiate*> one_type;
        std::queue<BreakCandiate *> seed;
        bool is_cpoint=false, is_stopline=false, is_other=false;
        seed.push(&bc);
        hash_table.insert(&bc);
        while (!seed.empty())
        {
            auto front = seed.front();
            seed.pop();
            if (front->type==6) {
                is_stopline=true;
            } else if (front->type==3) {
                is_cpoint=true;
            } else {
                is_other=true;
            }
            one_type.push_back(front);
            //
            Eigen::Vector3d seed_pos = front->raw_cross_point;
            std::vector<BreakCandiate *> nerighbor;
            break_candidate_tree.search(seed_pos, max_align_length, nerighbor);
            for (auto nb : nerighbor)
            {
                if (hash_table.count(nb))
                {
                    continue;
                }
                if(front->hit_link!=nb->hit_link && !(front->type==1 && nb->type==1))
                {
                    continue;
                }
                if(alg::calc_dis(seed_pos,nb->raw_cross_point)>4)
                {
                    continue;
                }
                if ((is_stopline && nb->type==3) || (is_cpoint && (nb->type==6 || nb->type==1)) || (is_other && nb->type==3)) {
                    continue;
                }
                if (front->type==6) {
                    is_stopline=true;
                } else if (front->type==3) {
                    is_cpoint=true;
                } else {
                    is_other=true;
                }
                seed.push(nb);
                hash_table.insert(nb);
            }
        }
        // 
        if(!one_type.empty())
        {
            // LOG_INFO("same_id:{}",same_id);
            same_id++;
            for(auto one:one_type)
            {
                max_id = same_id;
                one->same_id=same_id;
            }
            merge_candidate(one_type);
        }
        // 
    }
    return fsdmap::SUCC;
}

int RoadModelProcIdentifyRoad::merge_break_candidate_type(RoadModelSessionData* session)
{
    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto pt : line->list)
        {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }

    RTreeProxy<BreakCandiate *, float, 2> break_candiate_tree;
    for (auto &bc : break_candidates)
    {
        break_candiate_tree.insert((&bc)->pos, (&bc));
    }

    auto merge_candidate=[&lane_center_tree, &break_candiate_tree]( std::vector<BreakCandiate*> &one_type, std::vector<BreakCandiate> &total_break_candidates)
    {
        Eigen::Vector3d pos={0,0,0};
        Eigen::Vector3d dir={0,0,0};
        int n=one_type.size();
        std::vector<Eigen::Vector3d> dirs;
        // KeyPoseLine *hit_link=NULL;
        float maxDis=-100.0, minDis=100.0;
        for(int i=0;i<n;i++)
        {
            pos=pos.eval()+one_type[i]->raw_cross_point;
            dirs.push_back(one_type[i]->raw_dir);
            // hit_link
            auto dis = alg::calc_vertical_dis(one_type[i]->raw_pos, one_type[i]->raw_cross_point, one_type[i]->lane_dir, true, true);
            if(dis < minDis){
                minDis = dis;
            } 
            if(dis > maxDis){
                maxDis = dis;
            }
        }

        pos=pos.eval()/static_cast<double>(n);
        // refine dir 
        std::vector<LaneCenterFeature*> lcfs;
        lane_center_tree.search(pos, 20, lcfs);
        float lc_maxDis=-100.0, lc_minDis=100.0;
        if(!lcfs.empty())
        {
            dirs.clear();
            for(const auto lc:lcfs)
            {
                auto vdir=alg::get_vertical_dir(lc->dir);
                dirs.push_back(vdir);

                auto theta = alg::calc_theta(one_type[0]->lane_dir, lc->dir, false, true);
                if(theta > 60){
                    continue;
                }

                auto dis = alg::calc_vertical_dis(lc->pos, pos, lc->dir, true, true);
                if(dis < lc_minDis){
                    lc_minDis = dis;
                } 
                if(dis > lc_maxDis){
                    lc_maxDis = dis;
                }
            }
        }
        // std::cout << "----- break_point pos: " << one_type[0]->raw_pos << std::endl;
        // std::cout << "----- center_lane max-min dist: " << (lc_maxDis-lc_minDis) << " (" << lc_maxDis << ", " << lc_minDis << ")" << std::endl;
        // std::cout << "----- break_point max-min dist: " << (maxDis-minDis) << " (" << maxDis << ", " << minDis << ")" << std::endl;

        bool isvalid = true;
        if(std::abs(lc_maxDis-lc_minDis) > 3.75 && std::abs(maxDis-minDis)+3.75 < std::abs(lc_maxDis-lc_minDis)*0.7 && std::abs(maxDis-minDis) < 5.0){
            isvalid = false;
        } 
        // else {
        //     if(std::abs(maxDis-minDis) > std::abs(lc_maxDis-lc_minDis)*0.75){
        //         isvalid = true;
        //     }
        // }

        std::vector<BreakCandiate*> bcfs;
        break_candiate_tree.search(pos, 10, bcfs);
        if(!bcfs.empty())
        {
            for(const auto bc:bcfs)
            {
                auto theta = alg::calc_theta(one_type[0]->lane_dir, bc->lane_dir, false, true);
                if(theta < 30){
                    isvalid = false;
                    break;
                }
            }
        }


        if(isvalid){
            dir=alg::get_direction(dirs);
            double max_dis=-100;
            for(int i=0;i<n;i++)
            {
                max_dis=std::max(max_dis,alg::calc_dis(one_type[i]->raw_pos,one_type[i]->raw_cross_point));
            }
            max_dis=max_dis+3.75/2;
            for(int i=0;i<n;i++)
            {
                one_type[i]->pos=pos;
                one_type[i]->ps= alg::get_hori_pos(pos, dir, +max_dis);
                one_type[i]->pe= alg::get_hori_pos(pos, dir, -max_dis);
                total_break_candidates.push_back(*one_type[i]);
            }
        }
    };
    const double max_align_length = 15 * 1.414;
    RTreeProxy<BreakCandiate *, float, 2> break_candidate_tree;
    for (auto &bc : break_candidates_type)
    {
        if (bc.raw_cross_pose)
        {
            break_candidate_tree.insert(bc.raw_cross_point, &bc);
        }
    }
    int same_id=max_id;
    std::set<BreakCandiate *> hash_table;
    for (auto &bc : break_candidates_type)
    {
        if(check_bc_in_sm_range(session, bc)) {
            continue;
        }

        if (hash_table.count(&bc))
        {
            continue;
        }
        std::vector<BreakCandiate*> one_type;
        std::queue<BreakCandiate *> seed;
        seed.push(&bc);
        hash_table.insert(&bc);
        while (!seed.empty())
        {
            auto front = seed.front();
            seed.pop();
            one_type.push_back(front);
            //
            Eigen::Vector3d seed_pos = front->raw_cross_point;
            std::vector<BreakCandiate *> nerighbor;
            break_candidate_tree.search(seed_pos, max_align_length, nerighbor);
            for (auto nb : nerighbor)
            {
                if (hash_table.count(nb))
                {
                    continue;
                }
                auto theta = alg::calc_theta(front->lane_dir, nb->lane_dir, false, true);
                if(theta > 60)
                {
                    continue;
                }
                seed.push(nb);
                hash_table.insert(nb);
            }
        }
        // 
        if(!one_type.empty())
        {
            // LOG_INFO("same_id:{}",same_id);
            same_id++;
            for(auto one:one_type)
            {
                one->same_id=same_id;
            }
            merge_candidate(one_type, break_candidates);
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcIdentifyRoad::gen_lane_directin(RoadModelSessionData *session)
{
    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto pt : line->list)
        {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }
    for (auto link : session->link_sample_list)
    {
        for (auto poss : link->list)
        {
            std::vector<Eigen::Vector3d> dirs;
            std::vector<LaneCenterFeature *> lcfs;
            dirs.push_back(alg::get_vertical_dir(poss->dir));
            auto pos=poss->pos;
            lane_center_tree.search(pos, 5, lcfs);
            if (!lcfs.empty())
            {
                for (const auto lc : lcfs)
                {
                    auto vdir = alg::get_vertical_dir(lc->dir);
                    dirs.push_back(vdir);
                }
            }
            // 垂向
            poss->road_vertical_dir = alg::get_direction(dirs);
            if(alg::calc_theta1(  poss->road_vertical_dir,poss->dir,true)>90)
            {
                poss->road_vertical_dir=- poss->road_vertical_dir;
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcIdentifyRoad::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_identify_road_save_data_enable) {
        return fsdmap::SUCC;
    }
    session->set_display_name("identify");
    //
    int count = 0;
    auto point_log = session->add_debug_log(utils::DisplayInfo::POINT, "break_{}", count++);
    for (auto candidate : break_candidates)
    {
        // auto log = session->add_debug_log(utils::DisplayInfo::LINE, "break_{}", count++);
        // log->color = {255, 255, 255};
        srand48(candidate.same_id);
        int type_id = drand48() * 1E3;
        if (candidate.raw_cross_pose)
        {
            // LOG_INFO("add_cross_pose");
            point_log->color = {128, 128, 0};
            auto &ele = point_log->add(candidate.raw_cross_point);
            ele.label.label = type_id;
            ele.label.opt_label = candidate.type;
            ele.label.score=0;

            auto mline = session->add_debug_log(utils::DisplayInfo::LINE, "maerge_{}", count++);
            mline->color={255,0,0};
            auto &ps = mline->add(candidate.ps);
            auto &pe = mline->add(candidate.pe);
            auto &point = point_log->add(candidate.pos);
            if(candidate.type==1)
            {
                ps.color={255,0,0};
                pe.color={255,0,0};
                point.color={255,0,0};
            }else if (candidate.type==3)
            {
                ps.color={0,255,0};
                pe.color={0,255,0};
                point.color={0,255,0};
            }else if (candidate.type==6)
            {
                ps.color={0,255,255};
                pe.color={0,255,255};
                point.color={0,255,255};
            }else
            {
                ps.color={0,0,255};
                pe.color={0,0,255};
                point.color={0,0,255};
            }
            ps.label.score=0;
            pe.label.score=0;
            point.label.score=0;
            ps.label.label = type_id;
            pe.label.label = type_id;
            point.label.label = type_id;
            ps.label.opt_label=pe.label.opt_label=candidate.type;
        }
        else
        {
            LOG_WARN("break_candidates cross_pose null!!! ")
        }

    }
    for (auto link : session->link_sample_list)
    {

        for (auto poss : link->list)
        {
            auto mline = session->add_debug_log(utils::DisplayInfo::LINE, "maerge_{}", count++);
            mline->color = {255, 255, 255};
            auto p1 = alg::get_hori_pos(poss->pos, poss->road_vertical_dir, +3.75 / 2);
            auto p2 = alg::get_hori_pos(poss->pos, poss->road_vertical_dir, -3.75 / 2);
            auto &ele1 = mline->add(p1);
            auto &ele2 = mline->add(p2);
            ele1.label.score = alg::calc_theta(poss->road_vertical_dir) * 57.3;
            ele2.label.score = alg::calc_theta(poss->road_vertical_dir) * 57.3;
            ele1.label.label=0;
            ele2.label.label=0;
       
        }
    }
    //
    session->save_debug_info("identify");

    return fsdmap::SUCC;
}

}
}

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
