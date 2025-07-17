#include "road_model_proc_batch_process.h"
#include "utils/algorithm_util.h"
#include "road_model_proc_split_merge.h"
#include <bitset>

DECLARE_double(sample_line_sample_pose_gap);

DEFINE_bool(sm_save_data_enable, true, "sm_save_data_enable");
DEFINE_bool(sm_detial_img_enable, true, "sm_detial_img_enable");
DECLARE_bool(coverage_detial_img_enable);

namespace fsdmap {
    namespace road_model {
fsdmap::process_frame::PROC_STATUS RoadModelProcSplitMerge::proc(RoadModelSessionData* session) {
        
    CHECK_FATAL_PROC(stage_3_process_split_merge_scene(session), "stage_3_process_split_merge_scene");
    
    save_debug_info(session, 7);//画分合流

    CHECK_FATAL_PROC(connect_split_merge_center_line(session),"connect_split_merge_center_line");

    CHECK_FATAL_PROC(connect_split_merge_lane_line(session),"connect_split_merge_lane_line");

    CHECK_FATAL_PROC(merge_sm_break_candidate(session),"merge_split_merge_break_candidate");

    save_debug_info_connect(session, 1);//画连线

    save_debug_info_all(session);//调试图

    return fsdmap::process_frame::PROC_STATUS_SUCC;
}




int RoadModelProcSplitMerge::stage_3_process_split_merge_scene(RoadModelSessionData* session)
{
    // 阶段1：SM1：矢量化识别的交叉点 （SM：split and merge）和 link 识别的分叉路口位置（非分合流位置）
    // find_link_fork_road(session);
    find_link_fork_road_from_nodes(session);

    // 阶段2：SM2：计算无前驱或后继的位置 （SM：split and merge）
    // 阶段3：SM3：通过 道路宽 确定分合流的位置 
    // a 如果该分合流左右都是连起来的，就不需要做分合流？
    // b 可以用 SM2 的结果互相做校验，
    //      b1：如果 SM2 和 SM3 都有识别，SM3 成功后，则将 SM3 附近的前驱后继节点删除？； 
    //      b2：如果 SM3 有识别，但是 SM2 没有，则认为是有； 
    //      b3：如果 SM3 没识别，但是 SM2 识别了，需要在 SM2 的位置再识别一次 SM2 

    bind_sm_to_keypose(session);
    mask_turn_right_link(session);//标记右转link的起始到keypose上
    
    // 阶段3： 
    //1. 计算曲率
    calc_curvature_3pt(session);

    for(auto &info : session->hor_corss_line_m){
        // auto poss = info.first;
        // auto feature_list = info.second->cross_feature_list;

        //2.1 通过N计算初始的曲率区间
        std::vector<InitSplitMergeIndex> init_sm_cs_indexs; // 候选的分合流点
        init_search_region(session, info.second, init_sm_cs_indexs);
        // std::cout << "qzc 1 " << init_sm_cs_indexs.size() << std::endl;
        
        //2.2 检测曲率变化的分合流点位置，并确定N区间是否正确
        std::vector<SearchBoundaryInfo> prev_search_bd_infos; // 往前搜索
        std::vector<SearchBoundaryInfo> next_search_bd_infos; // 往后搜索
        refine_search_region2(session, info.second, init_sm_cs_indexs,
                            prev_search_bd_infos, next_search_bd_infos);
        // std::cout << "qzc 2 " << prev_search_bd_infos.size() << " " << next_search_bd_infos.size() << std::endl;

        //3.补点
        recovery_point(session, info.first, info.second, prev_search_bd_infos, next_search_bd_infos);
    }

    // 阶段3：根据分合流区间补线



    return fsdmap::SUCC;
}

int RoadModelProcSplitMerge::bind_sm_to_keypose(RoadModelSessionData* session)
{
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

    auto find_closest_link = [&link_tree, &lane_center_tree](Eigen::Vector3d pos_in,
                                        Eigen::Vector3d road_vertical_dir) -> KeyPose* {
        auto p1 = alg::get_hori_pos(pos_in, road_vertical_dir, +20);
        auto p2 = alg::get_hori_pos(pos_in, road_vertical_dir, -20);
        std::vector<KeyPose*> poses;
        link_tree.search(pos_in,30,poses);
        KeyPose* nearst_pose = NULL;
        double min_dis = DBL_MAX;
        for (auto& pose : poses) {
            if(!pose) {
                continue;
            }
            double dis_tmp = alg::calc_dis(pose->pos, pos_in);
            if(dis_tmp < min_dis) {
                min_dis = dis_tmp;
                nearst_pose = pose;
            }
        }
        
        if(!nearst_pose || !nearst_pose->next) {
            return NULL;
        }

        return nearst_pose;
    };

    for (int i = 0; i < session->classified_lc_points.size(); i++) {
        Eigen::Vector3d pos = session->classified_lc_points[i]->pos;
        Eigen::Vector3d v_road_dir = alg::get_vertical_dir(session->classified_lc_points[i]->dir);
        auto target_keypose = find_closest_link(pos, v_road_dir);
        if(!target_keypose) {
            continue;
        }

        if(m_keypose_2_sm_lc.count(target_keypose) > 0) {
            continue;
        } 

        m_keypose_2_sm_lc[target_keypose] = session->classified_lc_points[i];
    }

    return fsdmap::SUCC;
}

void RoadModelProcSplitMerge::mask_turn_right_link(RoadModelSessionData* session)
{
    auto search_straight_kp = [&session](KeyPose* cur, KeyPoseLine* link)
    {
        std::vector<KeyPose *> secs;
        session->link_pos_tree.search(cur->pos, 20, secs);
        
        KeyPose* targe_kp = nullptr;
        double min_dis = DBL_MAX;
        for(auto& kp : secs) {
            //剔除自身
            if(kp->from_link == link){
                continue;
            }

            if(!kp->from_raw_link){
                continue;
            }

            if(alg::match_any_with_forms(kp->from_raw_link->forms, {16,25,26,27})) {
                continue;
            }

            double dis = alg::calc_dis(cur->pos, kp->pos);
            if(dis < min_dis){
                min_dis = dis;
                targe_kp = kp;
            }
        }
        return targe_kp;
    };

    auto save_keypose = [&](std::unordered_map<KeyPoseLine*, std::vector<KeyPose*>>& map, KeyPose* kp) 
    {
        if(!kp)  return;

        auto it = map.find(kp->from_link);
        if (it != map.end()) {
            it->second.push_back(kp);
        } else {
            map.insert({kp->from_link, {kp}});  
        }
    };

    //1.拿右转link
    std::vector<KeyPoseLine*> turn_right_links;
    for(auto& link: session->link_sample_list) {
        if(link->list.empty()){
            continue;
        }

        KeyPose* start_kp = link->list[0];
        if(!start_kp->from_raw_link){
            continue;
        }
        
        if(alg::match_any_with_forms(start_kp->from_raw_link->forms, {25,26})) {
            turn_right_links.push_back(link);
        }
    }


    //2.存起始,结束点
    for(auto& link: turn_right_links) {
        KeyPose* start_kp = search_straight_kp(link->list.front(), link);
        KeyPose* end_kp = search_straight_kp(link->list.back(), link); //不同线的
        save_keypose(turn_right_endpoint_m, start_kp);
        save_keypose(turn_right_endpoint_m, end_kp);
    }
}

PolyFit RoadModelProcSplitMerge::init_polyfit(const std::vector<Eigen::Vector3d>& points, LaneCenterFeature* center){
    PolyFit::Problem problem;
    PolyFit::Options option;
    problem.degree = 3;
    problem.points = points;
    if(center){
        problem.center = center->pos;
        problem.dir = center->dir;
        option.use_calc_direction = false;
        option.use_calc_center = false;
    }else{
        option.use_calc_direction = true;
        option.use_calc_center = true;
    }

    PolyFit fit(problem, option);
    fit.solver();
    
    return fit;
}

std::vector<Eigen::Vector3d> RoadModelProcSplitMerge::polyfit_points(const std::vector<Eigen::Vector3d>& points,
        const std::vector<Eigen::Vector3d>& points_without_z,  PolyFit& fit)
{
    std::vector<Eigen::Vector3d> fit_points;  // 用于存储结果
    Eigen::Vector3d last_fit_pnt;

    for (int j = 0; j < points_without_z.size(); j++) {
        auto fit_pnt = fit.eval(points_without_z[j]);
        
        if (j == 0) {
            // 第一条线段，直接插值z并添加到fit_points
            Eigen::Vector3d out_fit_pnt(fit_pnt.x(), fit_pnt.y(), points[j].z());
            fit_points.push_back(out_fit_pnt);
            last_fit_pnt = fit_pnt;
        } else {
            auto diff = alg::calc_dis(fit_pnt, last_fit_pnt);
            auto diff_bak = diff;
            Eigen::Vector3d dir = alg::get_dir(fit_pnt, last_fit_pnt, false);
            
            while (diff > 2.1) {  // 当差值较大时进行插值
                auto predict_fit_pnt = last_fit_pnt + dir * 2;
                auto added_fit_pnt = fit.eval(predict_fit_pnt);
                
                // 插值z
                double factor = diff_bak < 1e-6 ? 0 : diff / diff_bak;
                double interp_z = factor * points[j - 1].z() + (1.0 - factor) * points[j].z();
                Eigen::Vector3d out_fit_pnt(added_fit_pnt.x(), added_fit_pnt.y(), interp_z);
                fit_points.push_back(out_fit_pnt);

                last_fit_pnt = added_fit_pnt;
                diff = alg::calc_dis(fit_pnt, last_fit_pnt);
            }

            // 最后插值z
            double factor = diff_bak < 1e-6 ? 0 : diff / diff_bak;
            double interp_z = factor * points[j - 1].z() + (1.0 - factor) * points[j].z();
            Eigen::Vector3d out_fit_pnt(fit_pnt.x(), fit_pnt.y(), interp_z);
            fit_points.push_back(out_fit_pnt);

            last_fit_pnt = fit_pnt;
        }
    }

    return fit_points;

}

void RoadModelProcSplitMerge::find_link_fork_road(RoadModelSessionData* session) {
    RTreeProxy<KeyPoseLine*, float, 2> raw_link_line_pos_tree;
    for(const auto& link : session->raw_links)
    {
        if(link->list.size() >= 2) {
            auto &front=link->list.front();
            auto &back=link->list.back();
            raw_link_line_pos_tree.insert(front->pos,link.get());
            raw_link_line_pos_tree.insert(back->pos,link.get());
        }
    }
    // 统计每个节点的的度（只包含 form == 10 和 form == 32)
    for (const auto& link : session->raw_links) {
        if (link->list.size() < 2) {
            continue;
        }
        
        for (const auto& p : link->list) {
            std::vector<KeyPoseLine*> tmp_secs;
            raw_link_line_pos_tree.search(p->pos, 0.5, tmp_secs); // 0.5 m
            int fork_road_num = 0;
            for(const auto& link_line : tmp_secs) {
                // if(link_line->form == "10" || link_line->form == "32") {
                if(alg::match_any_with_forms(link_line->forms, {10, 32})) {
                    fork_road_num++;
                }
            }
            // 说明此处是非主路口的叉路口

            if(fork_road_num > 2) {
                std::vector<KeyPose*> tmp_secs2;
                if (raw_link_fork_points.search(p->pos, 0.5, tmp_secs2) == 0) {
                    raw_link_fork_points.insert(p->pos, p);
                    raw_link_fork_points_for_visual.push_back(p->pos);
                }
            }
        }
    }

    std::cout << "raw_link_fork_points_for_visual : " << raw_link_fork_points_for_visual.size() << std::endl;
}

void RoadModelProcSplitMerge::find_link_fork_road_from_nodes(RoadModelSessionData* session) {
    
    int i = 0;
    std::unordered_map<std::string, std::shared_ptr<KeyPoseLine>> link_id_map;
    for(const auto& link : session->raw_links) {
        if(link->list.size() >= 2) {
            link_id_map.insert({link->id, link});
        }
    }

    auto fork_road_count = [&, session](uint64_t node_id, std::unordered_set<uint64_t>& has_exist, Eigen::Vector3d& node_pos) {
        int fork_road_num = 0;
        if (has_exist.count(node_id) == 0 && session->node_id_map.count(node_id) > 0) {
            has_exist.insert(node_id);
            auto& node_link_ids = session->node_id_map[node_id]->node_link_ids;
            node_pos = session->node_id_map[node_id]->pos;

            // 如果为主、副点，说明有十字路口 或 掉头，在这我们只筛选T字路口
            if (session->node_id_map[node_id]->cross_flag == 2 ||
                session->node_id_map[node_id]->cross_flag == 3) {
                return fork_road_num;
            }
            
            // 方式1： 通过统计 node 关联的 link 数，问题：session->raw_links 缺数据（轨迹匹配丢弃了一些）
            // for(const auto& link_id : node_link_ids) {
            //     if(link_id_map.count(link_id) == 0) {
            //         continue;
            //     }
            //     // if(link_id_map[link_id]->form == "10" || link_id_map[link_id]->form == "32") {
            //     if(alg::match_any_with_forms(link_line->forms, {10, 32})) {
            //         fork_road_num++;
            //     }
            // }
            // 方式2：直接通过 node 关联的 link 数，减去 左转的、右转 、 掉头的、路口内的
            fork_road_num = node_link_ids.size();
            for(const auto& link_id : node_link_ids) {
                if(link_id_map.count(link_id) == 0) {
                    continue;
                }

                // if(link_id_map[link_id]->form == "16"  // 交叉点内link
                //     || link_id_map[link_id]->form == "24" // 掉头
                //     || link_id_map[link_id]->form == "25" // 右转专用道
                //     || link_id_map[link_id]->form == "26") // 左转专用道
                // {
                // if(alg::match_any_except_forms(link_id_map[link_id]->forms, {16, 24, 25, 26})) {
                if(alg::match_any_with_forms(link_id_map[link_id]->forms, {16, 24, 25, 26})) {
                    fork_road_num--;
                }
            }
        }

        return fork_road_num;
    };

    // std::unordered_set<std::string> check_link_nums;
    for(auto &info : session->hor_corss_line_m){
        auto feature_list = info.second->cross_feature_list;

        // 查找多个 分合流点
        int feature_size = feature_list.size();
        std::unordered_set<uint64_t> has_exist;
        for (int i = 0; i < feature_size; i++) {
            auto keypose = feature_list[i]->keypose;
            KeyPoseLine* link_line = keypose->from_raw_link;
            
            auto start_node_id = link_line->start_node_id;
            Eigen::Vector3d node_pos;
            int start_fork_road_num = fork_road_count(start_node_id, has_exist, node_pos);
            if(start_fork_road_num >= 3) {
                // 说明此处是非主路口的叉路口:T字路口
                info.second->link_fork_road.insert(node_pos, link_line);
                raw_link_fork_points_for_visual.push_back(node_pos);
            }

            // if(check_link_nums.count(link_line->id) == 0) {
            //     check_link_nums.insert(link_line->id);
            // }

            auto end_node_id = link_line->end_node_id;
            node_pos.setZero();
            int end_fork_road_num = fork_road_count(end_node_id, has_exist, node_pos);
            if(end_fork_road_num >= 3) {
                // 说明此处是非主路口的叉路口:T字路口
                info.second->link_fork_road.insert(node_pos, link_line);
                raw_link_fork_points_for_visual.push_back(node_pos);
            }
        }
    }
    LOG_INFO("raw_link_fork_points_for_visual: {}", raw_link_fork_points_for_visual.size());
}

void RoadModelProcSplitMerge::convert_boundary_to_pcd(RoadModelSessionData* session){
    int cnt = 0;
    for(auto &info : session->hor_corss_line_m){
        //计算左右边界曲率
        for(int j = 0; j < 2; j++){
            pcl::PointCloud<pcl::PointXYZI>::Ptr bd_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            auto feature_list = info.second->cross_feature_list;
            for(int i = 0; i < feature_list.size(); ++i) {
                Eigen::Vector3d p0 = feature_list[i]->road_boundary_pts[j]->pos;
                bd_cloud->points.push_back(pcl::PointXYZI(p0.x(), p0.y(), p0.z(), feature_list[i]->road_boundary_pts[j]->curvature * 100000));

            }
            bd_cloud->width = bd_cloud->size();
            bd_cloud->height = 1;
            bd_cloud->is_dense = false;

            std::string save_bd_path(FLAGS_debug_file_dir + "/cxf_bd_" + std::to_string(cnt) + ".pcd");
            pcl::io::savePCDFile<pcl::PointXYZI>(save_bd_path.c_str(), *bd_cloud);
            cnt ++;
        }
    }
}

// 因为这里需要计算keypose采样点的曲率，方便处理 CrossPointFeature，所以在这针对 session->hor_corss_line_m ，进行曲率计算
void RoadModelProcSplitMerge::calc_curvature_3pt(RoadModelSessionData* session){
    int n = 1; //取前后第n个
    for(auto &info : session->hor_corss_line_m){
        //计算左右边界曲率
        for(int j = 0; j < 2; j++){
            auto feature_list = info.second->cross_feature_list;
        
            int feature_size = feature_list.size();
            if (feature_size < n+2) {
                continue;
            }
            
            for(int i = n; i < feature_size - n; ++i) {

                if(feature_list[i+n]->keypose->is_mask == true || feature_list[i-n]->keypose->is_mask == true 
                        || feature_list[i+n]->keypose->is_mask == true)
                {
                    continue;
                }


                //只处理状态为RAW的点
                std::shared_ptr<CrossPoint<BoundaryGroupLine>> p0 = feature_list[i-n]->road_boundary_pts[j];
                std::shared_ptr<CrossPoint<BoundaryGroupLine>> p1 = feature_list[i]->road_boundary_pts[j];
                std::shared_ptr<CrossPoint<BoundaryGroupLine>> p2 = feature_list[i+n]->road_boundary_pts[j];

                //检查是否可以获取前第n个点
                if(p0->status != PointStatus::RAW || p1->status != PointStatus::RAW || p2->status != PointStatus::RAW){
                    continue;
                }

                //计算曲率
                Eigen::Vector3d v1 = p0->pos - p1->pos;
                Eigen::Vector3d v2 = p2->pos - p1->pos;
                Eigen::Vector3d v3 = p0->pos - p2->pos;
                double numerator = 4 * v1.cross(v2).norm();
                double denominator = (v1.norm() + v2.norm() + v3.norm());

                // Eigen::Vector2d v1 = (p0->pos - p1->pos).head(2);
                // Eigen::Vector2d v2 = (p2->pos - p1->pos).head(2);
                // double numerator = 4 * fabs(v1.x() * v2.y() - v1.y() * v2.x());
                // double denominator = (v1.norm() + v2.norm());
                if(denominator < 1e-6) {
                    // 避免除以零的情况
                    p1->curvature = 0.0;
                } else {
                    p1->curvature = (numerator / denominator) * 100;
                }
            }
        }
    }
    if(0){
        convert_boundary_to_pcd(session);
    }

}

void RoadModelProcSplitMerge::calc_curvature_5pt(RoadModelSessionData* session){
    for(auto &info : session->hor_corss_line_m){
        //计算左右边界曲率
        for(int j = 0; j < 2; j++){
            auto feature_list = info.second->cross_feature_list;
            if(feature_list.size() < 5){
                continue;
            }
            //从第2个点到倒数第3个点进行曲率计算
            for(int i = 2; i < feature_list.size() - 2; ++i) {
                if(i > 2 && 
                   feature_list[i-2]->road_boundary_pts[j]->status == PointStatus::RAW &&
                   feature_list[i-1]->road_boundary_pts[j]->status == PointStatus::RAW &&
                   feature_list[i]->road_boundary_pts[j]->status == PointStatus::RAW &&
                   feature_list[i+1]->road_boundary_pts[j]->status == PointStatus::RAW &&
                   feature_list[i+2]->road_boundary_pts[j]->status == PointStatus::RAW) 
                {
                    
                    Eigen::Vector3d p0 = feature_list[i-2]->road_boundary_pts[j]->pos;
                    Eigen::Vector3d p1 = feature_list[i-1]->road_boundary_pts[j]->pos;
                    Eigen::Vector3d p2 = feature_list[i]->road_boundary_pts[j]->pos;
                    Eigen::Vector3d p3 = feature_list[i+1]->road_boundary_pts[j]->pos;
                    Eigen::Vector3d p4 = feature_list[i+2]->road_boundary_pts[j]->pos;

                    double x0 = p0.x(), y0 = p0.y();
                    double x1 = p1.x(), y1 = p1.y();
                    double x2 = p2.x(), y2 = p2.y();
                    double x3 = p3.x(), y3 = p3.y();
                    double x4 = p4.x(), y4 = p4.y();

                    double numerator = (x3 - x1) * (y4 - y2) - (x4 - x2) * (y3 - y1);
                    double denominator = pow((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1), 3.0 / 2.0);

                    if(!denominator) {
                        // 避免除以零的情况
                        feature_list[i]->road_boundary_pts[j]->curvature = 0.0;
                    } else {
                        feature_list[i]->road_boundary_pts[j]->curvature = (std::abs(numerator) / denominator) * 1000;
                    }
                }
                else {
                    continue;
                }
            }
        }
    }
    if(1){
        convert_boundary_to_pcd(session);
    }
}


std::vector<std::pair<int,int>> RoadModelProcSplitMerge::calc_skip_ranges(RoadModelSessionData* session, HorizonCrossLine* hor_cross_line, int skip_num)
{  
    auto feature_list = hor_cross_line->cross_feature_list;
    int feature_size = feature_list.size();
    auto link = hor_cross_line->cross_feature_list[0]->keypose->from_link;
    std::vector<std::pair<int, int>> skip_ranges;
    
    if(session->link_inter_info.find(link) == session->link_inter_info.end()){
        return skip_ranges;
    }

    for (auto& inter_info : session->link_inter_info[link]) {
        KeyPose* in_keypose = inter_info.second->in_keypose;
        KeyPose* out_keypose = inter_info.second->out_keypose;
        int in_index = -1, out_index = -1;
        for(int i =0; i < feature_size; i++){
            if (feature_list[i]->keypose == in_keypose) {
                in_index = i;
            }
            
            if (feature_list[i]->keypose == out_keypose) {
                out_index = i;
            }
            
            if (in_index != -1 && out_index != -1) {
                break;
            }
        }

        //找到有效的点对位
        if (in_index != -1 && out_index != -1) {
            int start_skip = std::max(0, in_index- skip_num);
            int end_skip = std::min(feature_size - 1, out_index + skip_num);
            skip_ranges.push_back({start_skip, end_skip});
        }
    }
    return skip_ranges;
}


std::vector<std::pair<int, int>> RoadModelProcSplitMerge::calc_skip_ranges_for_turn_right(
    RoadModelSessionData* session, HorizonCrossLine* hor_cross_line, int skip_num)
{
    auto feature_list = hor_cross_line->cross_feature_list;
    int feature_size = feature_list.size();
    auto link = hor_cross_line->cross_feature_list[0]->keypose->from_link;
    std::vector<std::pair<int, int>> skip_ranges;
     
    if(turn_right_endpoint_m.find(link) == turn_right_endpoint_m.end()){
        return skip_ranges;
    }

    for (auto& kp : turn_right_endpoint_m[link]) {
        for(int i =0; i < feature_size; i++)
        {
            if (feature_list[i]->keypose == kp) {
                int start_skip = std::max(0, i - skip_num);  // 保证不越界
                int end_skip = std::min(feature_size - 1, i + skip_num);  // 保证不越界
                skip_ranges.push_back({start_skip, end_skip});
            }
        }
    }
    return skip_ranges;
}

bool RoadModelProcSplitMerge::should_skip(int i, const std::vector<std::pair<int, int>>& skip_ranges) {
    for (auto& range : skip_ranges) {
        if (i >= range.first && i <= range.second) {
            return true;  
        }
    }
    return false;  
}


void RoadModelProcSplitMerge::move_candidate_to_lanes_change(HorizonCrossLine* hor_cross_line, const std::vector<std::pair<int,int>>& skip_ranges, \
        std::vector<InitSplitMergeIndex>& init_sm_cs_indexs, const std::vector<std::pair<int,int>>& skip_ranges2)
{
    

    std::vector<InitSplitMergeIndex> lane_centers_change_index; //车道中心线数变化处
    auto find_closest_index = [&lane_centers_change_index](int init_index, int search_range){
         InitSplitMergeIndex closest_init_sm;
         closest_init_sm.index =  init_index; // 默认使用原始值

         int min_diff = std::numeric_limits<int>::max();
         for(int j = 0; j < lane_centers_change_index.size(); j++){
            int diff = std::abs(lane_centers_change_index[j].index - init_index);
            if(diff < min_diff && diff <= search_range){
                min_diff = diff;
                closest_init_sm.index = lane_centers_change_index[j].index;
                closest_init_sm.status = lane_centers_change_index[j].status;
            }
         }

         return closest_init_sm;
    };

    int move_max_range = 15;
    bool close_min_lane = true;
    bool close_min_num = 2;
    
    auto feature_list = hor_cross_line->cross_feature_list;
    int feature_size = feature_list.size();

    for (int i = 1; i < feature_size - 1; i++) {
        // 如果当前点位在跳过的范围内，跳过处理
        bool skip = should_skip(i, skip_ranges);
        if(skip)  continue;  

        int cur_n = feature_list[i]->N_lane_center; 
        int pre_n = feature_list[i-1]->N_lane_center; 
        int next_n = feature_list[i+1]->N_lane_center; 

        if(pre_n == -1 || next_n == -1 || cur_n == -1){
            continue;
        }

        if(cur_n != pre_n){
            if(cur_n > pre_n){
                int index = close_min_lane ? std::max(0, i - close_min_num -1) : i -1 ;

                InitSplitMergeIndex lane_change_sm;
                lane_change_sm.index = index;
                lane_change_sm.status = 1;
                lane_centers_change_index.push_back(lane_change_sm);
            }else{
                int index = close_min_lane ? std::min(feature_size - 1, i + close_min_num) : i ;

                InitSplitMergeIndex lane_change_sm;
                lane_change_sm.index = index;
                lane_change_sm.status = 2;
                lane_centers_change_index.push_back(lane_change_sm);
            }
        }
    }

    //移动位置
    std::vector<InitSplitMergeIndex> temp_sm_indexs;
    for(int i = 0; i < init_sm_cs_indexs.size(); i++){
         int init_index = init_sm_cs_indexs[i].index;
         InitSplitMergeIndex closest_init_sm = find_closest_index(init_index, move_max_range);
         temp_sm_indexs.push_back(closest_init_sm);
    }


    //去重并保持原来从小到大顺序,   同时去掉在右转附近的
    std::map<int, InitSplitMergeIndex> unique_indexs;
    for(auto sm_init : temp_sm_indexs){
        bool skip = should_skip(sm_init.index, skip_ranges2);
        if(skip) continue;  

        unique_indexs[sm_init.index] = sm_init;
    }

    //重新赋值
    init_sm_cs_indexs.clear();
    for(auto& entry : unique_indexs){
        init_sm_cs_indexs.push_back(entry.second);
    }
        
    //调试使用
    // KeyPose* start = hor_cross_line->start_keypose;
    // for(auto& info : lane_centers_change_index){
    //     auto it = lane_change_indexs_plt.find(start);
    //     if(it != lane_change_indexs_plt.end()){
    //         lane_change_indexs_plt[start].push_back(info.index);
    //     }else{
    //         lane_change_indexs_plt.insert({start, {info.index}});
    //     }
    // }
}

void RoadModelProcSplitMerge::init_search_region(RoadModelSessionData* session, HorizonCrossLine* hor_cross_line,
                                                std::vector<InitSplitMergeIndex>& init_sm_cs_indexs)
{
    // int SKIP_KEYPOSE_NUM = 25; // 25 * 2 = 50m, 前面的50个keypose不用，因为右转处分合流和路中段的不一样
    int SKIP_KEYPOSE_NUM = 10; 
    auto feature_list = hor_cross_line->cross_feature_list;
    std::vector<std::pair<int,int>> skip_ranges = calc_skip_ranges(session, hor_cross_line, SKIP_KEYPOSE_NUM);

    // 查找多个 分合流点
    int feature_size = feature_list.size();
    std::vector<int> pre_next_diff(feature_size, 0);
    for (int i = 1; i < feature_size; i++) {
        // // 前面 classify_prev_and_next_lc 识别的分合流，直接添加进去
        // if (m_keypose_2_sm_lc.count(feature_list[i]->keypose) > 0) {
        //     if(feature_list[i]->keypose) {
        //         auto lc = m_keypose_2_sm_lc[feature_list[i]->keypose];
        //         InitSplitMergeIndex init_sm;
        //         init_sm.index = i;
        //         init_sm_cs_indexs.push_back(init_sm);
        //         hor_cross_line->sm_road_tree.insert(lc->pos, lc);
        //         std::cout << "sm push " << i << std::endl;
        //         continue;
        //     }
        // }

        // 如果当前点位在跳过的范围内，跳过处理
        bool skip = should_skip(i, skip_ranges);
        if(skip)  continue;  

        if(feature_list[i]->keypose->is_mask == true || feature_list[i-1]->keypose->is_mask == true){
            pre_next_diff[i] = 100;
            continue;
        }

        // TODO:qzc use N_hope ?
        pre_next_diff[i] = feature_list[i]->N_road - feature_list[i-1]->N_road;
        // 没有边界的位置，不处理
        if (feature_list[i]->N_road == -1 || feature_list[i-1]->N_road == -1) {
            pre_next_diff[i] = 100;
        }

        double curvature = -1;
        for (auto& bd : feature_list[i]->road_boundary_pts) {
            curvature = std::max(curvature, bd->curvature);
        }
        if (curvature > 100) {
            pre_next_diff[i] = 100;
        }
    }

    // std::vector<int> init_sm_cs_indexs; // 候选的分合流点
    int window_size = 5;
    // for (int i = SKIP_KEYPOSE_NUM; i < feature_size - window_size/2; i++) {
    // for (int i = SKIP_KEYPOSE_NUM; i < feature_size - window_size; i++) {
    for (int i = window_size; i < feature_size - window_size; i++) {
        // 前面 classify_prev_and_next_lc 识别的分合流，直接添加进去

        if(feature_list[i]->keypose->is_mask == true){
            continue;
        }

        if (m_keypose_2_sm_lc.count(feature_list[i]->keypose) > 0) {
            if(feature_list[i]->keypose) {
                auto lc = m_keypose_2_sm_lc[feature_list[i]->keypose];
                InitSplitMergeIndex init_sm;
                init_sm.index = i;
                init_sm_cs_indexs.push_back(init_sm);
                hor_cross_line->sm_road_tree.insert(lc->pos, lc);
                std::cout << "sm push " << i << std::endl;
                continue;
            }
        }

        // 没变化 或者 没有边界的地方都不处理
        if (abs(pre_next_diff[i]) == 0 || pre_next_diff[i] == 100) {
            continue;
        }

        // int pre_half_diff = 0;
        // int next_half_diff = 0;
        int full_diff = 0;
        int single_diff = 0;
        bool no_boundary_pose = false;
        for (int j = 0; j < window_size; j++) {
            // if (pre_next_diff[i-j] == 100 || pre_next_diff[i+j] == 100) {
            //     no_boundary_pose = true;
            //     break;
            // }
            
            if(feature_list[i-j]->keypose->is_mask == true || feature_list[i+j]->keypose->is_mask == true
                ||feature_list[i]->keypose->is_mask == true){
                break;
            }

            if (j == 0) {
                single_diff = pre_next_diff[i];

                full_diff += pre_next_diff[i];
            } else {
                full_diff += pre_next_diff[i-j];
                full_diff += pre_next_diff[i+j];
                // pre_half_diff += pre_next_diff[i-j];
                // next_half_diff += pre_next_diff[i+j];
            }
        }
        
        // case 1 ：如果滑窗内的差异为0， 说明是正常的过渡区域
        // case 2 ：如果滑窗中有点没有边界线，则不认为是分合流点
        // case 3 ：如果是仅仅是单点 >= 2, 说明是异常突变点，也不处理（可能是存在异常的道路边界线在道路中）
        // TODO：qzc 最好是判断边界线是否连续
        if (full_diff == 0 || 
            no_boundary_pose == true || 
            ((single_diff - full_diff == 0) && (abs(single_diff) >= 2))) {
            continue;
        } else {
            // 如果滑窗内的差异不为0， 则认为是有个跳变点
            // 是分合流的点
            InitSplitMergeIndex init_sm;
            init_sm.index = i;
            init_sm_cs_indexs.push_back(init_sm);
        }
    }

    //调试用
    KeyPose* start = hor_cross_line->start_keypose;
    for(auto& init_sm : init_sm_cs_indexs){
        auto it = init_sm_cs_indexs_before.find(start);
        if(it != init_sm_cs_indexs_before.end()){
            init_sm_cs_indexs_before[start].push_back(init_sm);
        }else{
            init_sm_cs_indexs_before.insert({start, {init_sm}});
        }
    }

    //移动分合流点到车道数变化出，并剔除右转的
    std::vector<std::pair<int,int>> skip_ranges2 = calc_skip_ranges_for_turn_right(session, hor_cross_line, 7);
    move_candidate_to_lanes_change(hor_cross_line, skip_ranges, init_sm_cs_indexs, skip_ranges2); 

    //调试用
    for(auto& init_sm : init_sm_cs_indexs){
        auto it = init_sm_cs_indexs_update.find(start);
        if(it != init_sm_cs_indexs_update.end()){
            init_sm_cs_indexs_update[start].push_back(init_sm);
        }else{
            init_sm_cs_indexs_update.insert({start, {init_sm}});
        }
    }
}

void RoadModelProcSplitMerge::refine_search_region(RoadModelSessionData* session, HorizonCrossLine* hor_cross_line,
                                                    const std::vector<InitSplitMergeIndex>& init_sm_cs_indexs,
                                                    std::vector<SearchBoundaryInfo>& prev_search_bd_infos,
                                                    std::vector<SearchBoundaryInfo>& next_search_bd_infos)
{    
    // 1. 从突变点往左右开始搜索，满状态的 cross_feature, 满足连续5个点，则该点就是分合流点
    int SEARCH_KEYPOSE_NUM = 20; // 20*2 = 40， 最多往左右搜索40米
    for (int i = 0; i < init_sm_cs_indexs.size(); i++) {
        auto feature_list = hor_cross_line->cross_feature_list;
        int feature_size = feature_list.size();

        int start_index = init_sm_cs_indexs[i].index;
        auto start_hor_cross_feature = feature_list[start_index];

        // 往左边搜索 SEARCH_KEYPOSE_NUM
        {
            int search_cnt =0; 
            SearchBoundaryInfo search_bd_info;
            search_bd_info.start_index = start_index;
            // start_index = start_index - 5 >=0 ? start_index-5 : start_index;
            for (int j = start_index; j >= 0 && search_cnt < SEARCH_KEYPOSE_NUM; j--, search_cnt++) {
                auto hor_cross_feature = feature_list[j];
                if (hor_cross_feature->is_full_lc) {
                    float left_theta = alg::calc_theta1(hor_cross_feature->road_boundary_pts[0]->pos,
                                start_hor_cross_feature->road_boundary_pts[0]->pos);
                    float right_theta = alg::calc_theta1(hor_cross_feature->road_boundary_pts[1]->pos,
                                start_hor_cross_feature->road_boundary_pts[1]->pos);
                    search_bd_info.left_theta.push_back(left_theta);
                    search_bd_info.right_theta.push_back(right_theta);
                    search_bd_info.search_index.push_back(j);                    
                }
            }

            int pre_index = -1;
            int continuous_cnt = 0;
            for (int j = 0; j < search_bd_info.search_index.size(); j++) {
                int cur_index = search_bd_info.search_index[j];
                if (pre_index == -1) {
                    search_bd_info.continuous_search_index.push_back(cur_index);
                    pre_index = cur_index;
                    continuous_cnt++;
                } else if (fabs(cur_index - pre_index) == 1) {
                    search_bd_info.continuous_search_index.push_back(cur_index);
                    pre_index = cur_index;
                    continuous_cnt++;
                } else {
                    // 如果有连续的5个点，则认为是可用的； 无需再往后遍历
                    if (search_bd_info.continuous_search_index.size() >= 5) {
                        break;
                    }
                    
                    search_bd_info.continuous_search_index.clear();
                    pre_index = -1;
                    continuous_cnt = 0;
                }
            }
            if (search_bd_info.continuous_search_index.size() < 5) {
                search_bd_info.continuous_search_index.clear();
                pre_index = -1;
                continuous_cnt = 0;
            }

            prev_search_bd_infos.push_back(search_bd_info);
        }
        
        // 往右边搜索 SEARCH_KEYPOSE_NUM
        {
            int search_cnt =0; 
            SearchBoundaryInfo search_bd_info;
            search_bd_info.start_index = start_index;
            for (int j = start_index; j < feature_size && search_cnt < SEARCH_KEYPOSE_NUM; j++, search_cnt++) {
                auto hor_cross_feature = feature_list[j];
                if (hor_cross_feature->is_full_lc) {
                    float left_theta = alg::calc_theta1(start_hor_cross_feature->road_boundary_pts[0]->pos,
                                hor_cross_feature->road_boundary_pts[0]->pos);
                    float right_theta = alg::calc_theta1(start_hor_cross_feature->road_boundary_pts[1]->pos,
                                hor_cross_feature->road_boundary_pts[1]->pos);
                    search_bd_info.left_theta.push_back(left_theta);
                    search_bd_info.right_theta.push_back(right_theta);
                    search_bd_info.search_index.push_back(j);
                }
            }

            int pre_index = -1;
            int continuous_cnt = 0;
            for (int j = 0; j < search_bd_info.search_index.size(); j++) {
                int cur_index = search_bd_info.search_index[j];
                if (pre_index == -1) {
                    search_bd_info.continuous_search_index.push_back(cur_index);
                    pre_index = cur_index;
                    continuous_cnt++;
                } else if (fabs(cur_index - pre_index) == 1) {
                    search_bd_info.continuous_search_index.push_back(cur_index);
                    pre_index = cur_index;
                    continuous_cnt++;
                } else {
                    // 如果有连续的5个点，则认为是可用的； 无需再往后遍历
                    if (search_bd_info.continuous_search_index.size() >= 5) {
                        break;
                    }

                    search_bd_info.continuous_search_index.clear();
                    pre_index = -1;
                    continuous_cnt = 0;
                }
            }
            if (search_bd_info.continuous_search_index.size() < 5) {
                search_bd_info.continuous_search_index.clear();
                pre_index = -1;
                continuous_cnt = 0;
            }

            next_search_bd_infos.push_back(search_bd_info);
        }
    }

    //调试用
    auto start = hor_cross_line->start_keypose;
    for(auto& info : prev_search_bd_infos){
        auto it = prev_search_bd_infos_before.find(start);
        if(it != prev_search_bd_infos_before.end()){
            prev_search_bd_infos_before[start].push_back(info);
        }else{
            prev_search_bd_infos_before.insert({start, {info}});
        }
    }

    // 2. 合并比较近的点
    int MIN_MERGE_NUM = 15; // 30米
    std::set<int> to_delete_set; // 待删除的点
    std::set<int> to_merge_set; // 待合并的点
    std::vector<std::vector<int>> to_merge_pairs;
    int diverge_and_converge_size = prev_search_bd_infos.size();
    for (int i = 0; i < diverge_and_converge_size; i++) {
        if (to_delete_set.count(i) > 0 || to_merge_set.count(i) > 0) {
            continue;
        }
        
        //标记空区间，要删除的
        int prev_size_i = prev_search_bd_infos[i].continuous_search_index.size();
        int next_size_i = next_search_bd_infos[i].continuous_search_index.size();
        if (prev_size_i == 0 || next_size_i == 0) {
            to_delete_set.insert(i);
            continue;
        }

        std::vector<int> to_merge_pair;
        to_merge_pair.push_back(i);
        to_merge_set.insert(i);
        int last_start_index = next_search_bd_infos[i].start_index;

        int index_distance = 0;
        for (int j = i + 1; j < diverge_and_converge_size; j++) {
            //剪枝，标记空区间，要删除的
            int prev_size_j = prev_search_bd_infos[j].continuous_search_index.size();
            int next_size_j = next_search_bd_infos[j].continuous_search_index.size();
            if (prev_size_j == 0 || next_size_j == 0) {
                to_delete_set.insert(j);
                continue;
            }

            index_distance = abs(next_search_bd_infos[j].start_index - last_start_index);
            std::cout << " last_start_index " << last_start_index << " cur " << next_search_bd_infos[j].start_index << " index_distance: " << index_distance << std::endl;
            if (index_distance <= MIN_MERGE_NUM) {
                to_merge_pair.push_back(j);
                to_merge_set.insert(j);
                LOG_INFO("merge {} (index:{}) -> {} (last index:{})", j, next_search_bd_infos[j].start_index, i, last_start_index);
                last_start_index = next_search_bd_infos[j].start_index;
            } else {
                break;
            }
        }
        
        to_merge_pairs.push_back(to_merge_pair);
    }

    std::vector<SearchBoundaryInfo> prev_search_bd_infos_out;
    std::vector<SearchBoundaryInfo> next_search_bd_infos_out;
    for (int i = 0; i < to_merge_pairs.size(); i++) { //所有合理的分合流点（包括待合并的点）
        auto& to_merge_pair = to_merge_pairs[i];
        if (to_merge_pair.size() == 1) {
            int left_index = to_merge_pair[0];
            int right_index = to_merge_pair[0];
            prev_search_bd_infos_out.push_back(prev_search_bd_infos[left_index]);
            next_search_bd_infos_out.push_back(next_search_bd_infos[right_index]);
        } else {
            // 取最左边的和最右边的匹配对
            int left_index = to_merge_pair[0];
            int right_index = to_merge_pair[to_merge_pair.size() - 1];
            prev_search_bd_infos_out.push_back(prev_search_bd_infos[left_index]);
            next_search_bd_infos_out.push_back(next_search_bd_infos[right_index]);
        }
    }
    
    prev_search_bd_infos.clear();
    next_search_bd_infos.clear();
    prev_search_bd_infos = prev_search_bd_infos_out;
    next_search_bd_infos = next_search_bd_infos_out;

    for(auto& info : prev_search_bd_infos){
        auto it = prev_search_bd_infos_update.find(start);
        if(it != prev_search_bd_infos_update.end()){
            prev_search_bd_infos_update[start].push_back(info);
        }else{
            prev_search_bd_infos_update.insert({start, {info}});
        }
    }

    LOG_INFO("分合流点数：{}, to_merge.size: {}", prev_search_bd_infos.size(), to_merge_pairs.size());

}


void RoadModelProcSplitMerge::refine_search_region2(RoadModelSessionData* session, HorizonCrossLine* hor_cross_line,
                                                    const std::vector<InitSplitMergeIndex>& init_sm_cs_indexs,
                                                    std::vector<SearchBoundaryInfo>& prev_search_bd_infos,
                                                    std::vector<SearchBoundaryInfo>& next_search_bd_infos)
{    
    // 1. 从突变点往左右开始搜索，满状态的 cross_feature, 满足连续5个点，则该点就是分合流点
    int SEARCH_KEYPOSE_NUM = 20; // 20*2 = 40， 最多往左右搜索40米
    int SEARCH_ROAD_NUM = 20; // 20*2 = 40， 最多往左右搜索40米
    double width_th = 0.3; //道路宽度差异   0.3 0.4比较好
    int CONT_NUM = 3;
    int MOVE_MIN_NUM = 4; //宽度不变至少移动的间隔
    int MOVE_MAX_NUM = 15;//宽度不变最多移动的间隔


    for (int i = 0; i < init_sm_cs_indexs.size(); i++) {
        auto feature_list = hor_cross_line->cross_feature_list;
        int feature_size = feature_list.size();

        int start_index = init_sm_cs_indexs[i].index;
        int status = init_sm_cs_indexs[i].status;
        auto start_hor_cross_feature = feature_list[start_index];

        bool is_steady1 = false,    is_steady2 = false;
        int left_steady_index = -1, right_steady_index = -1;
        int left_start, right_start;

        // 往左边搜索 SEARCH_KEYPOSE_NUM
        {
            //1.找车道宽度不变的位置
            for (int k = start_index; k >= 0 && k >= start_index - SEARCH_ROAD_NUM; k--) 
            {
                if(k-2 >= 0 ){
                    double cur_width = feature_list[k]->W_road;
                    double prev_width1 = feature_list[k-1]->W_road;
                    double prev_width2 = feature_list[k-2]->W_road;
                    
                    double diff1 = std::abs(cur_width - prev_width1);
                    double diff2 = std::abs(cur_width - prev_width2);
                    if(diff1 < width_th && diff2 < width_th){
                        is_steady1 = true;
                        left_steady_index = k;
                        break;
                    }
                }
            }

            //2. 检查最后外侧的点最小车道宽度是否>3m
            
            //3.更新搜索起点  并加入候选拟合点
            left_start = is_steady1 ?  left_steady_index : start_index;
            //至多移动15个点, 且车道数小的至少移动3个点
            left_start = (start_index - left_start <= MOVE_MAX_NUM) ? left_start : std::max((start_index - MOVE_MAX_NUM), 0); 
            if(status == 1){
                left_start = (start_index - left_start >= MOVE_MIN_NUM) ? left_start : std::max((start_index - MOVE_MIN_NUM), 0); 
            }

            //4.加点
            SearchBoundaryInfo search_bd_info;
            search_bd_info.start_index = start_index;
            for (int j = left_start; j >= 0 &&  j >= left_start - SEARCH_KEYPOSE_NUM; j--) {
                auto hor_cross_feature = feature_list[j];
                if (hor_cross_feature->is_full_lc) {
                    float left_theta = alg::calc_theta1(hor_cross_feature->road_boundary_pts[0]->pos,
                                start_hor_cross_feature->road_boundary_pts[0]->pos);
                    float right_theta = alg::calc_theta1(hor_cross_feature->road_boundary_pts[1]->pos,
                                start_hor_cross_feature->road_boundary_pts[1]->pos);
                    search_bd_info.left_theta.push_back(left_theta);
                    search_bd_info.right_theta.push_back(right_theta);
                    search_bd_info.search_index.push_back(j);                    
                }
            }

            //4. 加连续点
            int pre_index = -1;
            int continuous_cnt = 0;
            for (int j = 0; j < search_bd_info.search_index.size(); j++) {
                int cur_index = search_bd_info.search_index[j];
                if (pre_index == -1) {
                    search_bd_info.continuous_search_index.push_back(cur_index);
                    pre_index = cur_index;
                    continuous_cnt++;
                } else if (fabs(cur_index - pre_index) == 1) {
                    search_bd_info.continuous_search_index.push_back(cur_index);
                    pre_index = cur_index;
                    continuous_cnt++;
                } else {
                    // 如果有连续的5个点，则认为是可用的； 无需再往后遍历
                    if (search_bd_info.continuous_search_index.size() >= CONT_NUM) {
                        break;
                    }
                    
                    search_bd_info.continuous_search_index.clear();
                    pre_index = -1;
                    continuous_cnt = 0;
                }
            }
            if (search_bd_info.continuous_search_index.size() < CONT_NUM) {
                search_bd_info.continuous_search_index.clear();
                pre_index = -1;
                continuous_cnt = 0;
            }

            prev_search_bd_infos.push_back(search_bd_info);
        }
        
        // 往右边搜索 SEARCH_KEYPOSE_NUM

        {
            //找车道宽度不变的位置
            for (int k = start_index; k < feature_size && k <= start_index + SEARCH_ROAD_NUM; k++) 
            {
                if(k+2 < feature_size )
                {
                    double cur_width = feature_list[k]->W_road;
                    double next_width1 = feature_list[k+1]->W_road;
                    double next_width2 = feature_list[k+2]->W_road;

                    double diff1 = std::abs(cur_width - next_width1);
                    double diff2 = std::abs(cur_width - next_width2);
                    if(diff1 < width_th || diff2 < width_th){
                        is_steady2 = true;
                        right_steady_index = k;
                        break;
                    }
                }

        
            }

            right_start = is_steady2 ?  right_steady_index : start_index;
            //至多移动15个点, 且车道数小的至少移动3个点
            right_start = (right_start - start_index <= MOVE_MIN_NUM) ? right_start : std::min((start_index + MOVE_MAX_NUM), feature_size - 1); 
            if(status == 2){
                right_start = (right_start - start_index >= MOVE_MIN_NUM) ? right_start : std::min((start_index + MOVE_MIN_NUM), feature_size - 1); 
            }

            SearchBoundaryInfo search_bd_info;
            search_bd_info.start_index = start_index;
            for (int j = right_start; j < feature_size && j <= right_start + SEARCH_KEYPOSE_NUM; j++) {
                auto hor_cross_feature = feature_list[j];
                if (hor_cross_feature->is_full_lc) {
                    float left_theta = alg::calc_theta1(start_hor_cross_feature->road_boundary_pts[0]->pos,
                                hor_cross_feature->road_boundary_pts[0]->pos);
                    float right_theta = alg::calc_theta1(start_hor_cross_feature->road_boundary_pts[1]->pos,
                                hor_cross_feature->road_boundary_pts[1]->pos);
                    search_bd_info.left_theta.push_back(left_theta);
                    search_bd_info.right_theta.push_back(right_theta);
                    search_bd_info.search_index.push_back(j);
                }
            }

            int pre_index = -1;
            int continuous_cnt = 0;
            for (int j = 0; j < search_bd_info.search_index.size(); j++) {
                int cur_index = search_bd_info.search_index[j];
                if (pre_index == -1) {
                    search_bd_info.continuous_search_index.push_back(cur_index);
                    pre_index = cur_index;
                    continuous_cnt++;
                } else if (fabs(cur_index - pre_index) == 1) {
                    search_bd_info.continuous_search_index.push_back(cur_index);
                    pre_index = cur_index;
                    continuous_cnt++;
                } else {
                    // 如果有连续的5个点，则认为是可用的； 无需再往后遍历
                    if (search_bd_info.continuous_search_index.size() >= CONT_NUM) {
                        break;
                    }

                    search_bd_info.continuous_search_index.clear();
                    pre_index = -1;
                    continuous_cnt = 0;
                }
            }
            if (search_bd_info.continuous_search_index.size() < CONT_NUM) {
                search_bd_info.continuous_search_index.clear();
                pre_index = -1;
                continuous_cnt = 0;
            }

            next_search_bd_infos.push_back(search_bd_info);
        }

        //调试log
        auto start = hor_cross_line->start_keypose;
        auto it = road_steady_index.find(start);
        if(it != road_steady_index.end()){
            road_steady_index[start].push_back({left_steady_index, right_steady_index});
        }else{
            road_steady_index.insert({start, {{left_steady_index, right_steady_index}}});
        }
        //调试log
        auto it_update = road_steady_index_update.find(start);
        if(it_update != road_steady_index_update.end()){
            road_steady_index_update[start].push_back({left_start, right_start});
        }else{
            road_steady_index_update.insert({start, {{left_start, right_start}}});
        }
    }

    // for(auto& info :road_steady_index){
    //     for(auto& pair : info.second){
    //         std::cout << "pair:" << pair.first << "," << pair.second << std::endl;
    //     }
    // }

    //调试用
    auto start = hor_cross_line->start_keypose;
    for(auto& info : prev_search_bd_infos){
        auto it = prev_search_bd_infos_before.find(start);
        if(it != prev_search_bd_infos_before.end()){
            prev_search_bd_infos_before[start].push_back(info);
        }else{
            prev_search_bd_infos_before.insert({start, {info}});
        }
    }

    // 2. 合并比较近的点
    int MIN_MERGE_NUM = 15; // 原来是15个点 30米  ,   现在是 8米
    std::set<int> to_delete_set; // 待删除的点
    std::set<int> to_merge_set; // 待合并的点
    std::vector<std::vector<int>> to_merge_pairs;
    int diverge_and_converge_size = prev_search_bd_infos.size();
    for (int i = 0; i < diverge_and_converge_size; i++) {
        if (to_delete_set.count(i) > 0 || to_merge_set.count(i) > 0) {
            continue;
        }
        
        //标记空区间，要删除的
        int prev_size_i = prev_search_bd_infos[i].continuous_search_index.size();
        int next_size_i = next_search_bd_infos[i].continuous_search_index.size();
        if (prev_size_i == 0 || next_size_i == 0) {
            to_delete_set.insert(i);
            continue;
        }

        std::vector<int> to_merge_pair;
        to_merge_pair.push_back(i);
        to_merge_set.insert(i);
        int last_start_index = next_search_bd_infos[i].start_index;

        int index_distance = 0;
        for (int j = i + 1; j < diverge_and_converge_size; j++) {
            //剪枝，标记空区间，要删除的
            int prev_size_j = prev_search_bd_infos[j].continuous_search_index.size();
            int next_size_j = next_search_bd_infos[j].continuous_search_index.size();
            if (prev_size_j == 0 || next_size_j == 0) {
                to_delete_set.insert(j);
                continue;
            }

            index_distance = abs(next_search_bd_infos[j].start_index - last_start_index);
            std::cout << " last_start_index " << last_start_index << " cur " << next_search_bd_infos[j].start_index << " index_distance: " << index_distance << std::endl;
            if (index_distance <= MIN_MERGE_NUM) {
                to_merge_pair.push_back(j);
                to_merge_set.insert(j);
                LOG_INFO("merge {} (index:{}) -> {} (last index:{})", j, next_search_bd_infos[j].start_index, i, last_start_index);
                last_start_index = next_search_bd_infos[j].start_index;
            } else {
                break;
            }
        }
        
        to_merge_pairs.push_back(to_merge_pair);
    }

    std::vector<SearchBoundaryInfo> prev_search_bd_infos_out;
    std::vector<SearchBoundaryInfo> next_search_bd_infos_out;
    for (int i = 0; i < to_merge_pairs.size(); i++) { //所有合理的分合流点（包括待合并的点）
        auto& to_merge_pair = to_merge_pairs[i];
        if (to_merge_pair.size() == 1) {
            int left_index = to_merge_pair[0];
            int right_index = to_merge_pair[0];
            prev_search_bd_infos_out.push_back(prev_search_bd_infos[left_index]);
            next_search_bd_infos_out.push_back(next_search_bd_infos[right_index]);
        } else {
            // 取最左边的和最右边的匹配对
            int left_index = to_merge_pair[0];
            int right_index = to_merge_pair[to_merge_pair.size() - 1];
            prev_search_bd_infos_out.push_back(prev_search_bd_infos[left_index]);
            next_search_bd_infos_out.push_back(next_search_bd_infos[right_index]);
        }
    }
    
    prev_search_bd_infos.clear();
    next_search_bd_infos.clear();
    prev_search_bd_infos = prev_search_bd_infos_out;
    next_search_bd_infos = next_search_bd_infos_out;

    for(auto& info : prev_search_bd_infos){
        auto it = prev_search_bd_infos_update.find(start);
        if(it != prev_search_bd_infos_update.end()){
            prev_search_bd_infos_update[start].push_back(info);
        }else{
            prev_search_bd_infos_update.insert({start, {info}});
        }
    }

    LOG_INFO("分合流点数：{}, to_merge.size: {}", prev_search_bd_infos.size(), to_merge_pairs.size());

}

// prev_search_bd_infos: 从分合流点，往左依次推入的点
// next_search_bd_infos：从分合流点，往右依次推入的点
void RoadModelProcSplitMerge::recovery_point(RoadModelSessionData* session, KeyPose* first_keypose, 
                                            HorizonCrossLine* hor_cross_line, 
                                            const std::vector<SearchBoundaryInfo>& prev_search_bd_infos,
                                            const std::vector<SearchBoundaryInfo>& next_search_bd_infos)
{
    //找外侧连续点大于3m的点， 找到则退出，找不到用默认的
    auto find_bound = [&prev_search_bd_infos, &next_search_bd_infos, &hor_cross_line](int index, bool is_prev, int min_size, int& min_index, int& max_index) 
    {
        const double MIN_WIDTH_TH = 3;
        min_index = 0;
        max_index = min_index + min_size;
        int origin_size = is_prev ? prev_search_bd_infos[index].continuous_search_index.size() : next_search_bd_infos[index].continuous_search_index.size();
        double last_min_width = DBL_MAX;
        //从一开始选用的n个点的后面 开始遍历，到结束, 这之间最小车道宽度如果有小于3m的，则跳出
        for(int j = max_index; j < origin_size; j++) {
            int sub_index = is_prev ? prev_search_bd_infos[index].continuous_search_index[j] : next_search_bd_infos[index].continuous_search_index[j];
            auto hor_cross_feature = hor_cross_line->cross_feature_list[sub_index];
            
            double min_width = 5;
            std::shared_ptr<CrossPoint<LaneCenterGroupLine>> last_cp = nullptr;
            for (auto& cp : hor_cross_feature->lane_center_pts) {
                if (last_cp) {
                    double w = alg::calc_dis(cp->pos, last_cp->pos);
                    if(w < min_width) {
                        min_width = w;
                    }
                } 
                last_cp = cp;
            }

            //找到大于3m车道宽度的
            if(min_width > MIN_WIDTH_TH) {
                min_index = j - min_size;
                max_index = j;
                break;
            }

            //有变小趋势，则不往后寻找
            if(last_min_width < 5 && (min_width - last_min_width) < 0.2){
                min_index = j - min_size;
                max_index = j;
                break;
            }
            last_min_width = min_width;


        }
    };
    int MIN_FIT_NUM = 5;
    int in_out_status = first_keypose->in_out_status;
    auto feature_list = hor_cross_line->cross_feature_list;
    int feature_size = feature_list.size();

    int diverge_and_converge_size = prev_search_bd_infos.size();
    for (int i = 0; i < diverge_and_converge_size; i++) {
        int prev_size = prev_search_bd_infos[i].continuous_search_index.size();
        int next_size = next_search_bd_infos[i].continuous_search_index.size();
        if (prev_size == 0 || next_size == 0) {
            continue;
        }

        // 取最小的相同的点
        int min_size = std::min(std::min(prev_size, next_size), MIN_FIT_NUM);

        // 1 找到 prev 和 next 的线， 计算中心点，车道线平均朝向和垂直朝向
        // 1.1 首先找到最外侧点满足两两中心线之间的距离大于3m的位置 a， 那么内侧的位置为： a - min_size
        // 左侧
        int left_min, left_max;
        find_bound(i, true, min_size, left_min, left_max);

        Eigen::Vector3d center_point;
        center_point.setZero();
        int center_point_cnt = 0;
        Eigen::Vector3d prev_road_dir;
        for (int j = left_min; j < left_max; j++) {
            int prev_index = prev_search_bd_infos[i].continuous_search_index[j];
            auto hor_cross_feature = feature_list[prev_index];

            if (j == left_max - 1) {
                prev_road_dir = hor_cross_feature->road_dir;
            }

            for (auto& cp : hor_cross_feature->lane_center_pts) {
                center_point += cp->pos;
                center_point_cnt++;
            }
        }

        // 右侧
        int right_min, right_max;
        find_bound(i, false, min_size, right_min, right_max);
        
        Eigen::Vector3d next_road_dir;
        for (int j = right_min; j < right_max; j++) {
            int next_index = next_search_bd_infos[i].continuous_search_index[j];
            auto hor_cross_feature = feature_list[next_index];

            if (j == right_max - 1) {
                // TODO：qzc 检查 road_dir 的准确性
                next_road_dir = hor_cross_feature->road_dir;
            }

            for (auto& cp : hor_cross_feature->lane_center_pts) {
                center_point += cp->pos;
                center_point_cnt++;
            }
        }

        if (center_point_cnt == 0) {
            // 此处不应该进来
            continue;
        }
        
        int mid_index = prev_search_bd_infos[i].start_index;
        center_point = hor_cross_line->cross_feature_list[mid_index]->keypose->pos;; //用回原始的中心点
        // center_point /= center_point_cnt;

        Eigen::Vector3d mean_road_dir = (prev_road_dir + next_road_dir) / 2;
        Eigen::Vector3d mean_v_road_dir = alg::get_vertical_dir(mean_road_dir, false);
        LOG_INFO("center_point: {} {} {}, mean_road_dir: {} {} {}", 
                center_point.x(), center_point.y(), center_point.z(), mean_road_dir.x(), mean_road_dir.y(), mean_road_dir.z());

        // 2 匹配
        // 2.1 先通过点的方向和距离进行聚类
        match_prev_next(session, feature_list,
                        left_min, left_max, mid_index, right_min, right_max,
                        center_point, mean_road_dir, mean_v_road_dir,
                        prev_search_bd_infos[i], next_search_bd_infos[i]
                        );

        SplitMergePoints sm_fit_lines;
        sm_fit_lines.center = center_point;
        sm_fit_lines.road_dir = mean_road_dir;
        sm_fit_lines.v_road_dir = mean_v_road_dir;
        sm_fit_lines.in_out_status = in_out_status;

        bool is_valid = false;
        // 如果是预处理的分合流点
        std::vector<LaneCenterFeature*> serched_sms;
        hor_cross_line->sm_road_tree.search(center_point, 20, serched_sms);
        if (serched_sms.size() > 0) {
            is_valid = true; 
        } else {
            // 如果是交叉路口，则不处理
            std::vector<KeyPoseLine*> serched_links;
            hor_cross_line->link_fork_road.search(center_point, 20, serched_links);
            is_valid = serched_links.size() > 0 ? false : true;
        }
        is_valid = true;
        // TODO：晓峰，现在完全放开了，需要将有错误的分合流过滤掉

        auto& one_sm_match = session->sm_matchs.back();
        // 如果前后车道数相等，则不进行分合流识别
        int row_size = one_sm_match->prev_cp_lc.size();
        int col_size = one_sm_match->next_cp_lc.size();
        if(row_size == col_size) {
            is_valid = false;
        }
           
        // 3 拟合曲线
        bool is_add = false;
        int match_id = 0;
        for (auto match : one_sm_match->line_matchs) {
            std::vector<Eigen::Vector3d> pts, pts_no_z;
            std::vector<Eigen::Vector3d> pre_pts, pre_pts_no_z;
            std::vector<Eigen::Vector3d> next_pts, next_pts_no_z;

            LOG_INFO("match lc prev-next:{}<-->{}, left ll:{}<-->{}, right ll:{}<-->{}, status {}",
                    match.lc[0], match.lc[1], match.ll_left[0], match.ll_left[1], match.ll_right[0], match.ll_right[1],
                    std::bitset<8>(match.match_status).to_string());

            LaneCenterGroupLine* start_center_line  = nullptr;
            auto prev_it = one_sm_match->prev_cp_lc.begin();
            advance(prev_it, match.lc[0]);

            LaneCenterGroupLine* end_center_line  = nullptr;
            auto next_it = one_sm_match->next_cp_lc.begin();
            advance(next_it, match.lc[1]);

            std::vector<CrossPoint<LaneCenterGroupLine>*> lc_points; // 用于投票类型和颜色

            //prev
            for (int m = prev_it->second.size() - 1; m >= 0; m--) {
                if (m == prev_it->second.size() - 1 && prev_it->first != nullptr) {
                    start_center_line = prev_it->first;
                }
                
                auto pnt = prev_it->second[m].second;
                pre_pts.push_back(pnt->pos);
                pre_pts_no_z.push_back({pnt->pos.x(), pnt->pos.y(), 0});

                lc_points.push_back(pnt);
            }
            if(is_add){
                add_point(session,pre_pts,pre_pts_no_z,pre_pts.back(),center_point,true,
                            prev_it->first,next_it->first,one_sm_match->prev_cp_lc,one_sm_match->next_cp_lc);
            }

            //next
            for (int n = 0; n < next_it->second.size(); n++) {
                if (n == next_it->second.size() - 1 && next_it->first != nullptr) {
                    end_center_line = next_it->first;
                }

                auto pnt = next_it->second[n].second;
                next_pts.push_back(pnt->pos);
                next_pts_no_z.push_back({pnt->pos.x(), pnt->pos.y(), 0});

                lc_points.push_back(pnt);
            }
            if(is_add){
                add_point(session,next_pts,next_pts_no_z,next_pts.front(),center_point,false,
                prev_it->first,next_it->first,one_sm_match->prev_cp_lc,one_sm_match->next_cp_lc);
            }

            //拼接
            pts = pre_pts;
            pts.insert(pts.end(), next_pts.begin(), next_pts.end());
            pts_no_z = pre_pts_no_z;
            pts_no_z.insert(pts_no_z.end(), next_pts_no_z.begin(), next_pts_no_z.end());


            // 4 生成补点
            PolyFit fit = init_polyfit(pts_no_z); //初始化求解器
            std::vector<Eigen::Vector3d> fit_points= polyfit_points(pts, pts_no_z, fit); //一个分合流里面的多条连线
          
            std::vector<int> ret_type_color;
            int valid_num = alg::vote_type_color(lc_points, ret_type_color);

            if (fit_points.size() > 0) {
                std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>> new_line;
                for (int k = 0; k < fit_points.size(); k++) {
                    std::shared_ptr<CrossPoint<LaneCenterGroupLine>> new_point \
                        = std::make_shared<CrossPoint<LaneCenterGroupLine>>(PointStatus::FIT, fit_points[k], ret_type_color[0], ret_type_color[1]);
                    if(k == 0) {
                        new_point->src_line = start_center_line;
                    } else if(k == fit_points.size()-1) {
                        new_point->src_line = end_center_line;
                    }
                    new_line.push_back(new_point);
                }

                if(check_fit_points(new_line) == false) {
                    continue;
                }
                
                sm_fit_lines.center_lines.push_back(new_line);
                sm_fit_lines.valid_status.push_back(FitLineValidStatus(is_valid));
            }
        }

        // 5 识别断点
        if (sm_fit_lines.center_lines.size() > 0) {
            one_sm_match->sm_fit_lines = sm_fit_lines;
            // 处理车道线
        }
    }
}

void RoadModelProcSplitMerge::match_prev_next(RoadModelSessionData* session,
                    const std::vector<HorizonCrossFeature*>& feature_list,
                    int left_min, int left_max, int mid_index, int right_min, int right_max,
                    Eigen::Vector3d center_point, Eigen::Vector3d mean_road_dir, Eigen::Vector3d mean_v_road_dir,
                    const SearchBoundaryInfo& prev_search_bd_info,
                    const SearchBoundaryInfo& next_search_bd_info
)
{
    std::shared_ptr<OneSplitMergeMatch> one_sm_match = std::make_shared<OneSplitMergeMatch>();

    Eigen::Vector3d virtual_pnt = center_point + 50 * mean_v_road_dir;
    // 匹配
    // 1 中心綫
    // 1.1 先通过点的方向和距离进行聚类
    // key: 每条中心线，value：所有待匹配的点（投影点，原始交点）
    // std::map<LaneCenterGroupLine*, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> one_sm_match->prev_cp_lc;
    // prev：left_min 真实位置在 left_max 的右侧
    for (int j = left_min; j < left_max; j++) {
        int prev_index = prev_search_bd_info.continuous_search_index[j]; // 此处的索引是由大到小
        auto hor_cross_feature = feature_list[prev_index];

        Eigen::Vector3d cross_pt;
        for (auto& cp : hor_cross_feature->lane_center_pts) {
            bool has_intersect = alg::get_cross_point(center_point, mean_v_road_dir, cp->pos, mean_road_dir, cross_pt, false);
            if (has_intersect) {
                if (one_sm_match->prev_cp_lc.find(cp->src_line) != one_sm_match->prev_cp_lc.end()) {
                    one_sm_match->prev_cp_lc[cp->src_line].push_back({cross_pt, cp.get()});
                } else {
                    one_sm_match->prev_cp_lc[cp->src_line] = {{cross_pt, cp.get()}};
                }
            }
        }
    }
    // key: 每条中心线，value：所有待匹配的点（投影点，原始交点）
    // std::map<LaneCenterGroupLine*, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> one_sm_match->next_cp_lc;
    for (int j = right_min; j < right_max; j++) {
        int next_index = next_search_bd_info.continuous_search_index[j]; // 此处的索引是由小到大
        auto hor_cross_feature = feature_list[next_index];

        Eigen::Vector3d cross_pt;
        for (auto& cp : hor_cross_feature->lane_center_pts) {
            bool has_intersect = alg::get_cross_point(center_point, mean_v_road_dir, cp->pos, mean_road_dir, cross_pt, false);
            if (has_intersect) {
                if (one_sm_match->next_cp_lc.find(cp->src_line) != one_sm_match->next_cp_lc.end()) {
                    one_sm_match->next_cp_lc[cp->src_line].push_back({cross_pt, cp.get()});
                } else {
                    one_sm_match->next_cp_lc[cp->src_line] = {{cross_pt, cp.get()}};
                }
            }
        }
    }


    // 2 车道线
    // 2.1 先通过点的方向和距离进行聚类
    // key: 每条中心线，value：所有待匹配的点（投影点，原始交点）
    int origin_index_left = -1, origin_index_right = -1;
    int prev_size = prev_search_bd_info.continuous_search_index.size();
    int next_size = next_search_bd_info.continuous_search_index.size();
    if(prev_size > 0 && next_size > 0) {
        origin_index_left = prev_search_bd_info.continuous_search_index[prev_size-1];
        origin_index_right = next_search_bd_info.continuous_search_index[next_size-1];
    }
    LOG_INFO("origin_index_left:{}, left_min:{}, left_max:{}, mid_index:{}, right_min:{}, right_max:{}, origin_index_right:{}", 
                origin_index_left,
                prev_search_bd_info.continuous_search_index[left_min], 
                prev_search_bd_info.continuous_search_index[left_max],
                mid_index, 
                next_search_bd_info.continuous_search_index[right_min], 
                next_search_bd_info.continuous_search_index[right_max],
                origin_index_right);
    // for (int j = origin_index_left; j < mid_index; j++) {
    for (int j = mid_index-1; j >= 0 && j >= origin_index_left; j--) { // 从分割点开始，从右往左
        if(j > prev_search_bd_info.continuous_search_index[left_min]) continue; // 左侧起点保持和中心线一致

        auto hor_cross_feature = feature_list[j];

        Eigen::Vector3d cross_pt;
        int cnt = 0;
        for (auto& cp : hor_cross_feature->lane_pts) {
            bool has_intersect = alg::get_cross_point(center_point, mean_v_road_dir, cp->pos, mean_road_dir, cross_pt, false);
            if (has_intersect) {
                if (one_sm_match->prev_cp_ll.find(cp->src_line) != one_sm_match->prev_cp_ll.end()) {
                    // auto new_pair = std::make_pair<Eigen::Vector3d, CrossPoint<LaneCenterGroupLine>*>(cross_pt, cp.get());
                    // std::pair<Eigen::Vector3d, CrossPoint<LaneCenterGroupLine>*> new_pair(cross_pt, cp.get());
                    // one_sm_match->prev_cp_ll[cp->src_line].push_back(new_pair);
                    one_sm_match->prev_cp_ll[cp->src_line].push_back({cross_pt, cp.get()});
                } else {
                    one_sm_match->prev_cp_ll[cp->src_line] = {{cross_pt, cp.get()}};
                }
                cnt++;
            }
        }

        // LOG_INFO("prev j:{} {}, cnt:{}", j, hor_cross_feature->lane_pts.size(), cnt);
    }
    for (int j = mid_index+1; j <= origin_index_right; j++) { // 从分割点开始，从左往右
        if(j < next_search_bd_info.continuous_search_index[right_min]) continue; // 右侧起点保持和中心线一致

        auto hor_cross_feature = feature_list[j];

        Eigen::Vector3d cross_pt;
        int cnt = 0;
        for (auto& cp : hor_cross_feature->lane_pts) {
            bool has_intersect = alg::get_cross_point(center_point, mean_v_road_dir, cp->pos, mean_road_dir, cross_pt, false);
            if (has_intersect) {
                if (one_sm_match->next_cp_ll.find(cp->src_line) != one_sm_match->next_cp_ll.end()) {
                    one_sm_match->next_cp_ll[cp->src_line].push_back({cross_pt, cp.get()});
                } else {
                    one_sm_match->next_cp_ll[cp->src_line] = {{cross_pt, cp.get()}};
                }
                cnt++;
            }
        }
        // LOG_INFO("next j:{} {}, cnt:{}", j, hor_cross_feature->lane_pts.size(), cnt);
    }

    // 3 根据压盖关系 和 方向进行分组
    // 前后车道线，中心线投影到分割线上的点 与 虚拟点的距离
    std::vector<SortDistance> distance_proj;
    std::map<int, std::vector<double>> m_type_num;

    int row_size_lc = one_sm_match->prev_cp_lc.size();
    int col_size_lc = one_sm_match->next_cp_lc.size();
    for (int m = 0; m < row_size_lc; m++) {
        int type = 0;
        auto itr = one_sm_match->prev_cp_lc.begin();
        advance(itr, m);
        double dis = alg::calc_dis(itr->second[0].first, virtual_pnt); // 此处的second[0]：左侧最靠近分割线的点
        distance_proj.push_back(SortDistance(dis, type, m, itr->second[0].first));
        if (m_type_num.count(0) == 0) {
            m_type_num[type] = {{dis}};
        } else {
            m_type_num[type].push_back(dis);
        }
    }
    for (int m = 0; m < col_size_lc; m++) {
        int type = 1;
        auto itr = one_sm_match->next_cp_lc.begin();
        advance(itr, m);
        double dis = alg::calc_dis(itr->second[0].first, virtual_pnt); // 此处的second[0]：右侧最靠近分割线的点
        distance_proj.push_back(SortDistance(dis, type, m, itr->second[0].first));
        if (m_type_num.count(0) == 0) {
            m_type_num[type] = {{dis}};
        } else {
            m_type_num[type].push_back(dis);
        }
    }
    int row_size_ll = one_sm_match->prev_cp_ll.size();
    int col_size_ll = one_sm_match->next_cp_ll.size();
    for (int m = 0; m < row_size_ll; m++) {
        int type = 2;
        auto itr = one_sm_match->prev_cp_ll.begin();
        advance(itr, m);
        double dis = alg::calc_dis(itr->second[0].first, virtual_pnt);
        distance_proj.push_back(SortDistance(dis, type, m, itr->second[0].first));
        if (m_type_num.count(0) == 0) {
            m_type_num[type] = {{dis}};
        } else {
            m_type_num[type].push_back(dis);
        }
    }
    for (int m = 0; m < col_size_ll; m++) {
        int type = 3;
        auto itr = one_sm_match->next_cp_ll.begin();
        advance(itr, m);
        double dis = alg::calc_dis(itr->second[0].first, virtual_pnt);
        distance_proj.push_back(SortDistance(dis, type, m, itr->second[0].first));
        if (m_type_num.count(0) == 0) {
            m_type_num[type] = {{dis}};
        } else {
            m_type_num[type].push_back(dis);
        }
    }
    std::sort(distance_proj.begin(), distance_proj.end(), [](SortDistance& a, SortDistance& b) {
        return a.dis < b.dis;
    });

    // 4 左侧中心线和车道线的匹配关系
    // multi_matched_index： key-索引1， value-索引1的左右侧索引
    auto match_same_type = [&distance_proj, &m_type_num](int match_type1, int match_type2,
                                        std::vector<std::pair<int, int>>& matched_indexs,
                                        std::multimap<int, int>& multi_matched_index) {
        int proj_num = distance_proj.size();
        int match_type1_size = m_type_num[match_type1].size();
        int match_type2_size = m_type_num[match_type2].size();
        bool need_reverse = match_type2_size > match_type1_size;

        int tmp_match_type1, tmp_match_type2;
        if(need_reverse) {
            tmp_match_type1 = match_type2;
            tmp_match_type2 = match_type1;
        } else {
            tmp_match_type1 = match_type1;
            tmp_match_type2 = match_type2;
        }

        for (int i = 0; i < proj_num; i++) {
            auto& proj1 = distance_proj[i];
            if(proj1.type != tmp_match_type1) continue;

            double min_dis = DBL_MAX;
            int min_index = -1;
            for (int j = 0; j < proj_num; j++) {
                if(i == j) continue;
                auto& proj2 = distance_proj[j];
                if(proj2.type != tmp_match_type2) continue;

                // TODO：还能剪枝
                double dis = std::fabs(proj1.dis - proj2.dis);
                if (dis < min_dis) {
                    min_index = j;
                    min_dis = dis;
                }
            }

            if (min_index >= 0 && min_dis < 4) {
                auto& proj2 = distance_proj[min_index];
                if(need_reverse) {
                    matched_indexs.push_back({proj2.index, proj1.index});
                    multi_matched_index.insert({proj2.index, proj1.index});
                } else {
                    matched_indexs.push_back({proj1.index, proj2.index});
                    multi_matched_index.insert({proj1.index, proj2.index});
                }
            }
        }
    };
    
    // multi_matched_index： key-索引1， value-索引1的左右侧索引
    auto match_different_type = [&distance_proj, &m_type_num, &mean_road_dir](int match_type1, int match_type2,
                                        std::vector<std::tuple<int, int, int>>& matched_indexs,
                                        std::multimap<int, std::pair<int, int>>& multi_matched_index) {
        int proj_num = distance_proj.size();
        for (int i = 0; i < proj_num; i++) {
            auto& proj1 = distance_proj[i];
            if(proj1.type != match_type1) continue;

            double left_min_dis = DBL_MAX, right_min_dis = DBL_MAX;
            int left_index = -1, right_index = -1;
            for (int j = 0; j < proj_num; j++) {
                if(i == j) continue;
                auto& proj2 = distance_proj[j];
                if(proj2.type != match_type2) continue;

                // TODO：还能剪枝
                double dis = std::fabs(proj1.dis - proj2.dis);
                bool is_left = alg::judge_left2(proj2.pt, proj1.pt, mean_road_dir) >= 0;
                if (is_left) {
                    if (dis < left_min_dis) {
                        left_index = j;
                        left_min_dis = dis;
                    }
                    // LOG_INFO("qzc1 left_min_dis {}", left_min_dis);
                } else {
                    if(dis < right_min_dis) {
                        right_index = j;
                        right_min_dis = dis;
                    }
                    // LOG_INFO("qzc2 right_min_dis {}", right_min_dis);
                }
            }

            int matched_left_index = -1;
            if (left_index >= 0 && left_min_dis < 3) {
                auto& proj2 = distance_proj[left_index];
                matched_left_index = proj2.index;
            }
            int matched_right_index = -1;
            if (right_index >= 0 && right_min_dis < 3) {
                auto& proj2 = distance_proj[right_index];
                matched_right_index = proj2.index;
            }
            matched_indexs.push_back({proj1.index, matched_left_index, matched_right_index});
            multi_matched_index.insert({proj1.index, {matched_left_index, matched_right_index}});
        }
    };
    
    auto is_matched = [] (const std::multimap<int, int>& multi_matched_index, int prev_index, int next_index) {
        auto [lo, up] = multi_matched_index.equal_range(prev_index);
        return std::any_of(lo, up, [&](const auto& pair) {
            return pair.second == next_index;
        });
    };

    // prev 中心线 vs next 中心线
    std::vector<std::pair<int, int>> matched_indexs_lc;
    std::multimap<int, int> multi_matched_indexs_lc;
    int match_type1 = 0, match_type2 = 1;
    match_same_type(match_type1, match_type2, matched_indexs_lc, multi_matched_indexs_lc);

    // prev 车道线 vs next 车道线
    std::vector<std::pair<int, int>> matched_indexs_ll;
    std::multimap<int, int> multi_matched_indexs_ll;
    match_type1 = 2, match_type2 = 3;
    match_same_type(match_type1, match_type2, matched_indexs_ll, multi_matched_indexs_ll);
    std::multimap<int, int> multi_matched_indexs_ll_reverse;
    for (const auto& pair :  multi_matched_indexs_ll) {
        multi_matched_indexs_ll_reverse.insert({pair.second, pair.first});
    } 

    // prev 中心线 vs prev 车道线
    std::vector<std::tuple<int, int, int>> matched_indexs_prev_lc_ll;
    // multi_matched_indexs_prev_lc_ll：这个里面一定是中心线对应两条车道线，不会存在重复的key
    std::multimap<int, std::pair<int, int>> multi_matched_indexs_prev_lc_ll;
    match_type1 = 0, match_type2 = 2;
    match_different_type(match_type1, match_type2, matched_indexs_prev_lc_ll, multi_matched_indexs_prev_lc_ll);
    // for(auto& it : multi_matched_indexs_prev_lc_ll) {
    //     LOG_INFO("qzc left:{}<-->{},{}", it.first, it.second.first, it.second.second);
    // }

    // next 中心线 vs next 车道线
    std::vector<std::tuple<int, int, int>> matched_indexs_next_lc_ll;
    // multi_matched_indexs_next_lc_ll:这个里面一定是中心线对应两条车道线，不会存在重复的key
    std::multimap<int, std::pair<int, int>> multi_matched_indexs_next_lc_ll;
    match_type1 = 1, match_type2 = 3;
    match_different_type(match_type1, match_type2, matched_indexs_next_lc_ll, multi_matched_indexs_next_lc_ll);
    // for(auto& it : multi_matched_indexs_next_lc_ll) {
    //     LOG_INFO("qzc right:{}<-->{},{}", it.first, it.second.first, it.second.second);
    // }

    // 5 匹配中心线和车道线的匹配关系，并生成虚拟线
    for (const auto& lc : multi_matched_indexs_lc) {
        int prev_lc_index = lc.first;
        int next_lc_index = lc.second;
        // prev 中心线在 multi_matched_indexs_prev_lc_ll 是否存在
        // prev 中心线在 multi_matched_indexs_prev_lc_ll 是否有左右车道线
        int prev_ll_left_index = -1;
        int prev_ll_right_index = -1;
        //  multi_matched_indexs_prev_lc_ll：这个里面一定是中心线对应两条车道线，不会存在重复的key
        if(multi_matched_indexs_prev_lc_ll.count(prev_lc_index) > 0) {
            auto [lo, up] = multi_matched_indexs_prev_lc_ll.equal_range(prev_lc_index);
            for (auto it = lo; it != up; it++) {
                // 当前车道中心线粗匹配到的左右车道线
                prev_ll_left_index = it->second.first;
                prev_ll_right_index = it->second.second;
            }
        }

        // next 中心线在 multi_matched_indexs_next_lc_ll 是否存在
        // next 中心线在 multi_matched_indexs_next_lc_ll 是否有左右车道线
        int next_ll_left_index = -1;
        int next_ll_right_index = -1;
        // multi_matched_indexs_next_lc_ll:这个里面一定是中心线对应两条车道线，不会存在重复的key
        if(multi_matched_indexs_next_lc_ll.count(next_lc_index) > 0) {
            auto [lo, up] = multi_matched_indexs_next_lc_ll.equal_range(next_lc_index);
            for (auto it = lo; it != up; it++) {
                // 当前车道中心线粗匹配到的左右车道线
                next_ll_left_index = it->second.first;
                next_ll_right_index = it->second.second;
            }
        }

        // 通过车道线匹配的结果，检查 prev 和 next 中心线匹配到的车道线是否合理；如果合理，则保存该匹配关系
        OneLineMatch one_line_match;
        one_line_match.lc[0] = prev_lc_index;
        one_line_match.lc[1] = next_lc_index;
        one_line_match.match_status |= (1 << 0);

        // case1: 左侧有匹配的车道线prev+车道线next，车道线prev-next也有匹配关系，说明这个是合理的匹配关系，实线
        // case2：左侧有匹配的车道线prev+车道线next，车道线prev-next没有匹配关系，这条线应该是虚拟线
        // case3: 左侧的车道线prev或next缺失，车道线不完整，不正常
        if(multi_matched_indexs_ll.count(prev_ll_left_index) > 0 &&
            multi_matched_indexs_ll_reverse.count(next_ll_left_index) > 0) {
            one_line_match.ll_left[0] = prev_ll_left_index;
            one_line_match.ll_left[1] = next_ll_left_index;
            if (is_matched(multi_matched_indexs_ll, prev_ll_left_index, next_ll_left_index)) {
                one_line_match.match_status |= (1 << 3); // case 1
            } else {
                one_line_match.match_status |= (1 << 4); // case 2
            }
        } else {
            // case3: 车道线不完整，不正常
            // TODO:qzc
        }

        // case1: 右侧有匹配的车道线prev+车道线next，车道线prev-next也有匹配关系，说明这个是合理的匹配关系，实线
        // case2：右侧有匹配的车道线prev+车道线next，车道线prev-next没有匹配关系，这条线应该是虚拟线
        // case3: 左侧的车道线prev或next缺失，车道线不完整，不正常
        if(multi_matched_indexs_ll.count(prev_ll_right_index) > 0 &&
            multi_matched_indexs_ll_reverse.count(next_ll_right_index) > 0) {
            one_line_match.ll_right[0] = prev_ll_right_index;
            one_line_match.ll_right[1] = next_ll_right_index;
            if (is_matched(multi_matched_indexs_ll, prev_ll_right_index, next_ll_right_index)) {
                one_line_match.match_status |= (1 << 1); // case 1
            } else {
                one_line_match.match_status |= (1 << 2); // case 2
            }
        } else {
            // case3: 车道线不完整，不正常
            // TODO:qzc
        }

        // if(one_line_match.match_status > 1) {
            one_sm_match->line_matchs.push_back(one_line_match);
        // }
    }

    // if(one_sm_match->line_matchs.size() > 0) {
        session->sm_matchs.push_back(one_sm_match);
    // }

#if 1
    for(auto& it : one_sm_match->line_matchs) {
    }
#endif

    // 这种策略暂时丢弃：通过prev-next端点，判断中心线和车道线是否相交，如果相交，则需要去除相交线对应的匹配关系
}

// case1 : 如果边界曲率存在较大的曲率变化，说明该曲线拟合的不对
// case2 ：如果 link 上表明此处是 分叉的小路口，也不处理
bool RoadModelProcSplitMerge::check_fit_points(std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& new_line)
{
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

        // Eigen::Vector2d v1 = (p0->pos - p1->pos).head(2);
        // Eigen::Vector2d v2 = (p2->pos - p1->pos).head(2);
        // double numerator = 4 * fabs(v1.x() * v2.y() - v1.y() * v2.x());
        // double denominator = (v1.norm() + v2.norm());
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
}

void RoadModelProcSplitMerge::add_point(RoadModelSessionData* session, std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& pts_no_z,
        Eigen::Vector3d inside_pt, Eigen::Vector3d center_pt, bool is_prev,LaneCenterGroupLine* prev_line, LaneCenterGroupLine* next_line, 
        const std::map<LaneCenterGroupLine*, std::vector<std::pair<Eigen::Vector3d, CrossPoint<LaneCenterGroupLine>* >>>& prev_cp_m,
        const std::map<LaneCenterGroupLine*, std::vector<std::pair<Eigen::Vector3d, CrossPoint<LaneCenterGroupLine>* >>>& next_cp_m)
{
    if(prev_line == nullptr || next_line == nullptr) return;
    if(prev_line == next_line) return; //同一条线，则不用加点

    if(is_prev)
    {
        //1.处理prevw为合入的情况
        LaneCenterGroupLine* cur_line = prev_line;
    
        bool prev_m_found =  prev_cp_m.find(cur_line) != prev_cp_m.end();
        bool next_m_found =  next_cp_m.find(cur_line) != next_cp_m.end();
        if(prev_m_found && next_m_found){
            return; //cur线，是直线，在左右都有， 不用加点
        }

        //cur线只出现在一侧, 需要加点
        LaneCenterFeature* cur;
        cur = find_candidate(session, inside_pt, cur_line, false, true);

        double length = 0;
        while(cur){
            double h_dis=alg::calc_hori_dis(center_pt,cur->pos,cur->dir,true);
            if(h_dis < 0) {
                break;
            }

            if(length > 2){
                auto pt = cur->pos;
                add_pts.push_back(pt);
                pts.push_back(pt);
                pts_no_z.push_back({pt.x(), pt.y(), 0});
                length = 0;
            }
            
            if(cur->next){
                length+=alg::calc_dis(cur->pos,cur->next->pos);
            }
            cur = cur->next;
        }
    }else
    {
        //2.处理next为分出的情况
        LaneCenterGroupLine* cur_line = next_line;
        LaneCenterFeature* end_lc =cur_line->list.front().get();
    
        bool prev_m_found =  prev_cp_m.find(cur_line) != prev_cp_m.end();
        bool next_m_found =  next_cp_m.find(cur_line) != next_cp_m.end();
        if(prev_m_found && next_m_found){
            return; //cur线，是直线，在左右都有， 不用加点
        }

        //cur线只出现在一侧, 需要加点
        LaneCenterFeature* cur;
        cur = find_candidate(session, inside_pt, cur_line, true, true);
        
        double length = 0;
        while(cur){
            double h_dis=alg::calc_hori_dis(center_pt,cur->pos,cur->dir,true);
            if(h_dis > 0) {
                break;
            }

            if(length > 2){
                auto pt = cur->pos;
                add_pts.push_back(pt);
                pts.insert(pts.begin(), pt);
                pts_no_z.insert(pts_no_z.begin(), {pt.x(), pt.y(), 0});
                length = 0;
            }

            if(cur->prev){
                length+=alg::calc_dis(cur->pos,cur->prev->pos);
            }

            cur = cur->prev;
        }
    }
}



void RoadModelProcSplitMerge::update_lane_center_sample_tree(RoadModelSessionData* session) {

    session->merge_lane_center_sample_tree.RemoveAll();
    for (auto line : session->merge_lane_center_list) {
        for (auto pt : line->list) {
            session->merge_lane_center_sample_tree.insert(pt->pos, pt.get());
        }
    }
}

void RoadModelProcSplitMerge::update_link_pos_tree(RoadModelSessionData* session) {

    for(auto link: session->link_sample_list) {
         for(auto poss:link->list) {  
            session->link_pos_tree.insert(poss->pos, poss);
        }
    }
}

bool RoadModelProcSplitMerge::find_center_bk_cross(RoadModelSessionData* session, 
                                                    LaneCenterFeature * lc,
                                                    KeyPose* nearst_pose,
                                                    bool is_forward,
                                                    Eigen::Vector3d& cross)
{
    bool ret = false;
    auto p1 = alg::get_hori_pos(nearst_pose->pos, nearst_pose->road_vertical_dir, +30);
    auto p2 = alg::get_hori_pos(nearst_pose->pos, nearst_pose->road_vertical_dir, -30);
    auto lc2 = is_forward ? lc->next : lc->prev;
    while(lc && lc2) {
        Eigen::Vector3d cross_tmp;
        bool find = alg::get_cross_point_by_point(p1, p2, lc->pos,lc2->pos, cross_tmp, false, 3); // TODO:qzc 3 改为 2？
        if(find){
            cross = cross_tmp; 
            ret = true;
            break;
        }
        
        lc = lc2;
        lc2 = is_forward ? lc2->next : lc2->prev;
    }

    return ret;
}

bool RoadModelProcSplitMerge::find_center_bk_cross_with_kdtree(RoadModelSessionData* session, 
                                                    Eigen::Vector3d pt,
                                                    LaneCenterGroupLine* src_line,
                                                    KeyPose* nearst_pose,
                                                    Eigen::Vector3d& cross)
{
    bool ret = false;
    auto p1 = alg::get_hori_pos(nearst_pose->pos, nearst_pose->road_vertical_dir, +30);
    auto p2 = alg::get_hori_pos(nearst_pose->pos, nearst_pose->road_vertical_dir, -30);

    double search_radius = 50;
    std::vector<LaneCenterFeature*> lcfs;
    session->merge_lane_center_sample_tree.search(pt, search_radius, lcfs);

    if(src_line == NULL){
        LOG_WARN("find_center_bk_cross_with_kdtree src_line == nullptr, pt:{},{},{}",pt.x(), pt.y(), pt.z());
        return ret;
    }

    for (auto& lc : lcfs) {
        if(!lc || !lc->next) {
            continue;
        }
        if (lc->group_line != src_line) {
            continue;
        }

        Eigen::Vector3d cross_tmp;
        bool find = alg::get_cross_point_by_point(p1, p2, lc->pos,lc->next->pos, cross_tmp, false, 0);
        if(find){
            cross = cross_tmp; 
            ret = true;
            break;
        }
    }

    return ret;
}


bool RoadModelProcSplitMerge::get_refined_cross_link(RoadModelSessionData* session, 
                                                    RTreeProxy<KeyPose*, float, 2>& link_tree,
                                                    RTreeProxy<LaneCenterFeature *, float, 2>& lane_center_tree,
                                                    Eigen::Vector3d bk_pos,
                                                    KeyPose*& nearst_pose_out)
{
    auto find_link_cross = [&link_tree, &nearst_pose_out](Eigen::Vector3d bk_pos,
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
                    nearst_pose_out = nearst_pose;
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

    update_link_pos_tree(session);

    return true;
}

//顺着行车方向，例如src_line点的顺序为 ---  A front B C back D E  -->
//用法1：session, front, src_line, true, true ;    找的front外侧的点A ， 限制角度15度
//用法2：session, front, src_line, false, true;    找的front内侧的点B ， 限制角度15度
//用法3：session, back,  src_line, false,false;    找的back外侧的点 D， 不限制角度， 用于粗略的pt点传入，不在src_line的时候
//用法4：session, back,  src_line, true, true;     找的back外侧的点 C， 限制角度15度
LaneCenterFeature* RoadModelProcSplitMerge::find_candidate(RoadModelSessionData *session, 
            Eigen::Vector3d& pt, LaneCenterGroupLine* src_line, bool is_front, bool check_angle)
{
    std::vector<LaneCenterFeature*> lcfs;
    double search_radius = FLAGS_sample_line_sample_pose_gap * 2;
    session->merge_lane_center_sample_tree.search(pt, search_radius, lcfs);
    double closest_dis = DBL_MAX;
    LaneCenterFeature* closest_lcf = NULL;

    for (auto& lc : lcfs) {
        if(src_line == nullptr){
            LOG_WARN("split_merge_pts src_line == nullptr, pt:{},{},{}",pt.x(), pt.y(), pt.z());
        }
        if (lc->group_line != src_line) {
            continue;
        }

        if (alg::judge_front(pt, lc->pos, lc->dir) == is_front) {
            
            // 防止过近的点，导致后面计算朝向的时候误差过大 
            auto dir = is_front ? alg::get_dir(pt, lc->pos) : alg::get_dir(lc->pos, pt);
            if (check_angle && alg::calc_theta1(lc->dir, dir, true) > 15) {
                continue;
            }

            // 计算距离并更新最近点
            double dis = alg::calc_dis(pt, lc->pos);
            if (dis < closest_dis) {
                closest_dis = dis;
                closest_lcf = lc;
            }
        }
    }
    return closest_lcf;
};

int RoadModelProcSplitMerge:: connect_split_merge_center_line(RoadModelSessionData *session)
{
    // bool open_debug = true;
    bool open_debug = false;
    update_lane_center_sample_tree(session);

    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list) {
        for (auto pt : line->list) {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }

    auto find_road_vertical_dir = [&lane_center_tree] (Eigen::Vector3d pos_in, Eigen::Vector3d& road_vertical_dir) {
        std::vector<Eigen::Vector3d> dirs;
        std::vector<LaneCenterFeature *> lcfs;
        lane_center_tree.search(pos_in, 10, lcfs);
        if (!lcfs.empty()) {
            for (const auto lc : lcfs) {
                auto vdir = alg::get_vertical_dir(lc->dir);
                dirs.push_back(vdir);
            }
            road_vertical_dir = alg::get_direction(dirs);
            return true;
        } else {
            return false;
        }
    };


    auto align_and_gen_bk = [&] (std::shared_ptr<OneSplitMergeMatch>& one_sm_match, 
                                std::map<std::string, std::vector<std::vector<CandidateSeed<LaneCenterFeature>>>>& _sm_cs_classify_m) {
        // 找到分合流靠近中间的两侧边界
        int right_min_index  = INT_MAX;
        int left_max_index = -1;
        Eigen::Vector3d right_cross_candidate;right_cross_candidate.setZero();
        Eigen::Vector3d left_cross_candidate;left_cross_candidate.setZero();
        for(auto& pair : _sm_cs_classify_m)  {
            if(pair.first == "split") {
                for(auto& cs_list : pair.second) {
                    Eigen::Vector3d cross;cross.setZero();
                    int break_index = -1;
                    for(int i = cs_list[0].points.size() - 1; i > 0; i--){
                        auto p0_1 = cs_list[0].points[i-1].pos;
                        auto p0_2 = cs_list[0].points[i].pos;
                        
                        auto p1_1 = cs_list[1].points[i-1].pos;
                        auto p1_2 = cs_list[1].points[i].pos;
                        if (alg::get_cross_point_by_point(p0_1, p0_2, p1_1, p1_2, cross, false, 0)) {
                            break_index = i;
                            left_cross_candidate = cross;
                            break;
                        }
                    }

                    // 取靠近内侧的
                    if (break_index > 0 && break_index > left_max_index) {
                        left_max_index = break_index;
                    }
                }
            }

            if(pair.first == "merge") {
                for(auto& cs_list : pair.second) {
                    Eigen::Vector3d cross;cross.setZero();
                    int break_index = -1;
                    for(int i = 0; i < cs_list[0].points.size()-1; i++){
                        auto p0_1 = cs_list[0].points[i].pos;
                        auto p0_2 = cs_list[0].points[i+1].pos;
                        
                        auto p1_1 = cs_list[1].points[i].pos;
                        auto p1_2 = cs_list[1].points[i+1].pos;
                        if (alg::get_cross_point_by_point(p0_1, p0_2, p1_1, p1_2, cross, false, 0)) {
                            break_index = i;
                            right_cross_candidate = cross;
                            break;
                        }
                    }

                    // 取靠近内侧的
                    if (break_index > 0 && break_index < right_min_index) {
                        right_min_index = break_index;
                    }
                }
            }
        }

        LaneCenterFeature* ps_lc = NULL;
        LaneCenterFeature* pe_lc = NULL;
        for(auto& pair : _sm_cs_classify_m) {
            if(left_max_index >= 0 && right_min_index < 100) {
                LOG_WARN("在一处分合流处，同时出现分与合，逻辑暂未支持")
                continue;
            }

            if(pair.first == "split") {
                if(left_max_index < 0) {
                    continue;
                }

                KeyPose* nearst_pose = NULL;
                bool success = get_refined_cross_link(session, session->link_pos_tree, lane_center_tree, left_cross_candidate, nearst_pose);
                if(success == false) {
                    continue;
                }

                for(auto& cs_list : pair.second) {
                    LaneCenterGroupLine* src_line1 = cs_list[0].ps->group_line; // 用起始点 LaneCenterGroupLine*
                    LaneCenterGroupLine* src_line2 = cs_list[1].ps->group_line; // 用起始点 LaneCenterGroupLine*
                    Eigen::Vector3d center_link_cross1;
                    bool ret_cross1 = find_center_bk_cross_with_kdtree(session, left_cross_candidate, src_line1, nearst_pose, center_link_cross1);
                    Eigen::Vector3d center_link_cross2;
                    bool ret_cross2 = find_center_bk_cross_with_kdtree(session, left_cross_candidate, src_line2, nearst_pose, center_link_cross2);
                    if(ret_cross1 == false && ret_cross2 == false) {
                        continue;
                    }
                    LaneCenterFeature* lc_break_pt = NULL;
                    if(ret_cross1 && ret_cross2) {
                        LaneCenterFeature * lc_break_pt1 = find_candidate(session, center_link_cross1, src_line1, true, false); // 断点位置 LaneCenterFeature *
                        LaneCenterFeature * lc_break_pt2 = find_candidate(session, center_link_cross2, src_line2, true, false); // 断点位置 LaneCenterFeature *
                        if(lc_break_pt1 == nullptr || lc_break_pt2 == nullptr){
                            LOG_ERROR("split_merge_connect_candidates_m  split  not find ps: {},{},{}",right_cross_candidate.x(), right_cross_candidate.y(), right_cross_candidate.z());
                            continue;
                        }
                        Eigen::Vector3d center_cross = (center_link_cross1 + center_link_cross2) / 2.0; //两条线的ps平均
                        lc_break_pt1->pos = center_cross;
                        lc_break_pt1->v_road_dir = nearst_pose->road_vertical_dir;

                        lc_break_pt2->pos = center_cross;
                        lc_break_pt2->v_road_dir = nearst_pose->road_vertical_dir;

                        // TODO:中心点的朝向要更新
                        cs_list[0].ps = lc_break_pt1;
                        cs_list[1].ps = lc_break_pt2;

                        lc_break_pt = lc_break_pt1;
                    } else if(ret_cross1) {
                        LaneCenterFeature * lc_break_pt1 = find_candidate(session, center_link_cross1, src_line1, true, false); // 断点位置 LaneCenterFeature *
                        if(lc_break_pt1 == nullptr){
                            LOG_ERROR("split_merge_connect_candidates_m  split  not find ps: {},{},{}",right_cross_candidate.x(), right_cross_candidate.y(), right_cross_candidate.z());
                            continue;
                        }
                        lc_break_pt1->pos = center_link_cross1;
                        lc_break_pt1->v_road_dir = nearst_pose->road_vertical_dir;

                        // TODO:中心点的朝向要更新
                        cs_list[0].ps = lc_break_pt1;
                        cs_list[1].ps = lc_break_pt1;

                        lc_break_pt = lc_break_pt1;
                    } else if(ret_cross2) {
                        LaneCenterFeature * lc_break_pt2 = find_candidate(session, center_link_cross2, src_line2, true, false); // 断点位置 LaneCenterFeature *
                        if(lc_break_pt2 == nullptr){
                            LOG_ERROR("split_merge_connect_candidates_m  split  not find ps: {},{},{}",right_cross_candidate.x(), right_cross_candidate.y(), right_cross_candidate.z());
                            continue;
                        }
                        lc_break_pt2->pos = center_link_cross2;
                        lc_break_pt2->v_road_dir = nearst_pose->road_vertical_dir;

                        // TODO:中心点的朝向要更新
                        cs_list[0].ps = lc_break_pt2;
                        cs_list[1].ps = lc_break_pt2;

                        lc_break_pt = lc_break_pt2;
                    } 


                    cs_list[0].points.erase(cs_list[0].points.begin(), cs_list[0].points.begin() + left_max_index);
                    cs_list[1].points.erase(cs_list[1].points.begin(), cs_list[1].points.begin() + left_max_index);
                    // auto lc = lc_break_pt1;
                    // if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                    //     std::cout << lc->pos.transpose() << " lc:" << lc  << " group_line: " << lc->group_line << std::endl;
                    //     LOG_ERROR("qzc 3 : id:{}",  lc->group_line->id);
                    // }
                    ps_lc = lc_break_pt;
                    find_road_vertical_dir(cs_list[0].pe->pos, cs_list[0].pe->v_road_dir);
                    pe_lc =  cs_list[0].pe;
                }
            }
            
            if(pair.first == "merge") {
                if(right_min_index > 100) {
                    continue;
                }
                KeyPose* nearst_pose = NULL;
                bool success = get_refined_cross_link(session, session->link_pos_tree, lane_center_tree, right_cross_candidate, nearst_pose);
                if(success == false) {
                    continue;
                }

                for(auto& cs_list : pair.second) {
                    LaneCenterGroupLine* src_line1 = cs_list[0].pe->group_line; // 用起始点 LaneCenterGroupLine*
                    LaneCenterGroupLine* src_line2 = cs_list[1].pe->group_line; // 用起始点 LaneCenterGroupLine*
                    Eigen::Vector3d center_link_cross1;
                    bool ret_cross1 = find_center_bk_cross_with_kdtree(session, right_cross_candidate, src_line1, nearst_pose, center_link_cross1);
                    Eigen::Vector3d center_link_cross2;
                    bool ret_cross2 = find_center_bk_cross_with_kdtree(session, right_cross_candidate, src_line2, nearst_pose, center_link_cross2);
                    if(ret_cross1 == false || ret_cross2 == false) {
                        continue;
                    }


                    LaneCenterFeature* lc_break_pt = NULL;
                    if(ret_cross1 && ret_cross2) {
                        LaneCenterFeature * lc_break_pt1 = find_candidate(session, center_link_cross1, src_line1, false, false); // 断点位置 LaneCenterFeature *
                        LaneCenterFeature * lc_break_pt2 = find_candidate(session, center_link_cross2, src_line2, false, false); // 断点位置 LaneCenterFeature *
                        if(lc_break_pt1 == nullptr || lc_break_pt2 == nullptr){
                            LOG_ERROR("split_merge_connect_candidates_m  split  not find pe: {},{},{}",right_cross_candidate.x(), right_cross_candidate.y(), right_cross_candidate.z());
                            continue;
                        }
                        Eigen::Vector3d center_cross = (center_link_cross1 + center_link_cross2) / 2.0;
                        lc_break_pt1->pos = center_cross;
                        lc_break_pt1->v_road_dir = nearst_pose->road_vertical_dir;

                        lc_break_pt2->pos = center_cross;
                        lc_break_pt2->v_road_dir = nearst_pose->road_vertical_dir;

                        // TODO:中心点的朝向要更新
                        cs_list[0].pe = lc_break_pt1;
                        cs_list[1].pe = lc_break_pt2;

                        lc_break_pt = lc_break_pt1;
                    } else if(ret_cross1) {
                        LaneCenterFeature * lc_break_pt1 = find_candidate(session, center_link_cross1, src_line1, false, false); // 断点位置 LaneCenterFeature *
                        if(lc_break_pt1 == nullptr){
                            LOG_ERROR("split_merge_connect_candidates_m  split  not find pe: {},{},{}",right_cross_candidate.x(), right_cross_candidate.y(), right_cross_candidate.z());
                            continue;
                        }
                        lc_break_pt1->pos = center_link_cross1;
                        lc_break_pt1->v_road_dir = nearst_pose->road_vertical_dir;

                        // TODO:中心点的朝向要更新
                        cs_list[0].pe = lc_break_pt1;
                        cs_list[1].pe = lc_break_pt1;

                        lc_break_pt = lc_break_pt1;
                    } else if(ret_cross2) {
                        LaneCenterFeature * lc_break_pt2 = find_candidate(session, center_link_cross2, src_line2, false, false); // 断点位置 LaneCenterFeature *
                        if(lc_break_pt2 == nullptr){
                            LOG_ERROR("split_merge_connect_candidates_m  split  not find pe: {},{},{}",right_cross_candidate.x(), right_cross_candidate.y(), right_cross_candidate.z());
                            continue;
                        }
                        lc_break_pt2->pos = center_link_cross2;
                        lc_break_pt2->v_road_dir = nearst_pose->road_vertical_dir;

                        // TODO:中心点的朝向要更新
                        cs_list[0].pe = lc_break_pt2;
                        cs_list[1].pe = lc_break_pt2;

                        lc_break_pt = lc_break_pt2;
                    } 

                    cs_list[0].points.erase(cs_list[0].points.begin() + right_min_index + 1, cs_list[0].points.end());
                    cs_list[1].points.erase(cs_list[1].points.begin() + right_min_index + 1, cs_list[1].points.end());
                    
                    find_road_vertical_dir(cs_list[0].ps->pos, cs_list[0].ps->v_road_dir);
                    ps_lc = cs_list[0].ps;
                    pe_lc = lc_break_pt;
                }
            }
        }
        
        if(ps_lc && pe_lc) {

            // {
            //     session->set_display_name("test0");
            //     auto log1 = session->add_debug_log(utils::DisplayInfo::POINT, "test");
            //     log1->color= {255,0,0};
            //     auto& ele1=log1->add(ps_lc->pos);
                
            //     auto log2 = session->add_debug_log(utils::DisplayInfo::POINT, "test");
            //     log2->color= {0,0,255};
            //     auto& ele2=log2->add(pe_lc->pos);
            //     session->save_debug_info("test0");

            // }


            for(auto& pair : _sm_cs_classify_m)  {
                // 断点对齐后，将所有直行的线都进行对齐
                bool open_it = true;
                if(open_it && pair.first == "straight") {
                    for(auto& cs_lists : pair.second) {
                        for(auto& cs : cs_lists) {
                            std::vector<Eigen::Vector3d> fit_line_pts;
                            for(auto& pt :cs.points){
                                fit_line_pts.push_back(pt.pos);
                            }
                            auto p1 = alg::get_hori_pos(ps_lc->pos, ps_lc->v_road_dir, +20);
                            auto p2 = alg::get_hori_pos(ps_lc->pos, ps_lc->v_road_dir, -20);
                            
                            std::vector<UsefulPnt> res;
                            Eigen::Vector3d center_cross;
                            bool is_intersect = alg::get_cross_point_with_curve_segment(p1, p2, fit_line_pts, res, false, 0, 0, true); // 三角岛，或某一侧多边界，所以用true
                            LaneCenterFeature * found_lc_ps = NULL;
                            if(is_intersect) {
                                center_cross = res[0].pos;
                                LaneCenterGroupLine* src_line1 = cs.ps->group_line; // 用起始点 LaneCenterGroupLine*
                                found_lc_ps = find_candidate(session, center_cross, src_line1, true, true); // 断点位置 LaneCenterFeature *
                                if(found_lc_ps) {
                                    // std::cout << "cxf1 :" << std::endl;
                                    cs.ps = found_lc_ps;
                                }
                            }

                            p1 = alg::get_hori_pos(pe_lc->pos, pe_lc->v_road_dir, +20);
                            p2 = alg::get_hori_pos(pe_lc->pos, pe_lc->v_road_dir, -20);
                            is_intersect = alg::get_cross_point_with_curve_segment(p1, p2, fit_line_pts, res, false, 0, 0, true); // 三角岛，或某一侧多边界，所以用true
                            LaneCenterFeature * found_lc_pe = NULL;
                            if(is_intersect) {
                                center_cross = res[0].pos;
                                LaneCenterGroupLine* src_line1 = cs.pe->group_line; // 用起始点 LaneCenterGroupLine*
                                found_lc_pe = find_candidate(session, center_cross, src_line1, false, true); // 断点位置 LaneCenterFeature *
                                if(found_lc_pe) {
                                    // std::cout << "cxf2 :" << std::endl;
                                    cs.pe = found_lc_pe; //可能会比真实的断点位置 远一点
                                }
                            }

                            // {
                            //     session->set_display_name("test");
                            //     auto log1 = session->add_debug_log(utils::DisplayInfo::POINT, "test");
                            //     log1->color= {255,0,0};
                            //     auto& ele1=log1->add(cs.ps->pos);
                                
                            //     auto log2 = session->add_debug_log(utils::DisplayInfo::POINT, "test");
                            //     log2->color= {0,0,255};
                            //     auto& ele2=log2->add(cs.pe->pos);
                            //     session->save_debug_info("test");

                            // }

                            
                            std::vector<UsefulPnt> points_tmp;
                            for(auto& pt :cs.points){
                                if (alg::judge_front(pt.pos, cs.ps->pos, cs.ps->dir) == true && 
                                    alg::judge_front(pt.pos, cs.pe->pos, cs.pe->dir) == false) {
                                    points_tmp.push_back(pt);
                                }
                            }
                            // std::cout << "found_lc_ps :" << found_lc_ps << "found_lc_pe:" <<  found_lc_pe << std::endl;
                            if(found_lc_ps || found_lc_pe) {
                                cs.points.swap(points_tmp);
                            }
                        }
                    }
                }
            }

            LOG_INFO("find one split merge range")
            // 添加断点：起始
            BreakInfo*bk_s = new BreakInfo(BreakStatus::SPLIT_MERGE_FIT, ps_lc->pos, true);
            bk_s->v_dir = ps_lc->v_road_dir;
            bk_s->dir = ps_lc->dir;
            sm_break_candidate.push_back(bk_s);
            session->sm_break_candidate_tree.insert(bk_s->pos, bk_s);

            // 添加分割点：中间（！！！不会用来生成断点）
            auto first_element = _sm_cs_classify_m.begin();
            Eigen::Vector3d sm_split_center = first_element->second[0][0].center;
            Eigen::Vector3d v_road_dir = first_element->second[0][0].v_road_dir;
            BreakInfo*bk_m = new BreakInfo(BreakStatus::SPLIT_MERGE_FIT, sm_split_center, true);
            bk_m->v_dir = v_road_dir;
            bk_m->dir = ps_lc->dir;

            // 添加断点：结束
            BreakInfo*bk_e = new BreakInfo(BreakStatus::SPLIT_MERGE_FIT, pe_lc->pos, true);
            bk_e->v_dir = pe_lc->v_road_dir;
            bk_e->dir = pe_lc->dir;
            sm_break_candidate.push_back(bk_e);
            session->sm_break_candidate_tree.insert(bk_e->pos, bk_e);

            std::cout << "ps: " << ps_lc->pos.transpose() << " pe " << pe_lc->pos.transpose() << std::endl;
            session->split_merge_ranges.push_back(SplitMergeRange(bk_s, bk_m, bk_e));

            // 查找打断点与车道线的交点
            // int line_matchs_size = one_sm_match->line_matchs.size();
            // for (int i = 0; i < line_matchs_size; i++) {
            //     auto& line_match = one_sm_match->line_matchs[i];
            //     // 该中心线绑定的车道线存在
            //     if(line_matchs.ll_left >= 0) {
            //         LaneCenterGroupLine* start_center_line  = nullptr;
            //         // std::map<LaneLineSampleGroupLine*, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> prev_cp_ll;
            //         auto prev_it = one_sm_match->prev_cp_ll.begin();
            //         advance(prev_it, match.lc[0]);
            //     }
            // }
            
            // 拟合车道线
            // 匹配中心线和车道线
        }
    };

    int split_cnt = 0;
    int merge_cnt = 0;
    int straight_cnt = 0;

    //1. 转化为候选点
    // 所有分合流
    for(auto& one_sm_match : session->sm_matchs)
    {
        // 一处分合流
        auto& sm_fit_lines = one_sm_match->sm_fit_lines;
        // 预设重新连线后的vector大小
        int center_line_size = sm_fit_lines.center_lines.size();
        sm_fit_lines.center_lines_reconnected.resize(center_line_size);

        int index = 0;

        std::vector<CandidateSeed<LaneCenterFeature>> temp_sm_cs;
        std::map<LaneCenterFeature*, std::vector<CandidateSeed<LaneCenterFeature>>> split_map;
        std::map<LaneCenterFeature*, std::vector<CandidateSeed<LaneCenterFeature>>> merge_map;
        std::map<LaneCenterFeature*, std::vector<CandidateSeed<LaneCenterFeature>>> straight_map;
        std::map<std::string, std::vector<std::vector<CandidateSeed<LaneCenterFeature>>>> sm_cs_classify_m; 

        // 一处分合流的每一条中心线
        for(auto& line : sm_fit_lines.center_lines)
        {
            if (sm_fit_lines.valid_status[index++].is_valid == false) {
                continue;
            }
            //反转点的顺序， 和形成方向一致
            // if(sm_fit_lines.in_out_status == 1){
            //     std::reverse(line.begin(), line.end());
            // }

            CandidateSeed<LaneCenterFeature> cs;
            cs.ps = find_candidate(session, line.front()->pos, line.front()->src_line, true, true);//找candidate的ps 对应line的首点
            cs.pe = find_candidate(session, line.back()->pos, line.back()->src_line, false, true);//找candidate的pe 对应line的尾点

            if(cs.ps == nullptr){
                LOG_ERROR("split_merge_pts not find ps : {},{},{}",line.front()->pos.x(), line.front()->pos.y(), line.front()->pos.z())
                continue;
            }
            if(cs.pe == nullptr){
                LOG_ERROR("split_merge_pts not find pe : {},{},{}",line.back()->pos.x(), line.back()->pos.y(), line.back()->pos.z())
                continue;
            }

            auto lc = cs.ps;
            if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                std::cout << lc->pos.transpose() << " lc:" << lc  << " group_line: " << lc->group_line << std::endl;
                LOG_ERROR("qzc 1 : id:{}",  lc->group_line->id);
            }

            lc = cs.pe;
            if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                std::cout << lc->pos.transpose() << " lc:" << lc  << " group_line: " << lc->group_line << std::endl;
                LOG_ERROR("qzc 2 : id:{}",  lc->group_line->id);
            }


            for(auto& pt : line){
                auto new_pt = UsefulPnt(pt->pos, pt->dir, pt->type, pt->color);
                cs.points.push_back(new_pt);
            }
            cs.type = 3;
            cs.center = sm_fit_lines.center; // 分合流中心点
            cs.v_road_dir = sm_fit_lines.v_road_dir;
            // 匹配的车道线信息
            cs.match_id = index - 1;
            cs.one_sm_match = one_sm_match;
            
            sm_cs_origin_list.push_back(cs);
            temp_sm_cs.push_back(cs);
        }

        //2. 分类：分流，合流，直行
        std::map<LaneCenterGroupLine*, std::vector<CandidateSeed<LaneCenterFeature>>> split_map_temp;
        for(const auto& cs :temp_sm_cs){
            auto it = split_map_temp.find(cs.ps->group_line);
            if(it != split_map_temp.end()){
                it->second.push_back(cs);
            }else{
                split_map_temp[cs.ps->group_line] = {cs};
            }
        }
    
        //合流 
        std::map<LaneCenterGroupLine*, std::vector<CandidateSeed<LaneCenterFeature>>> merge_map_temp;
        for(auto split_it = split_map_temp.begin(); split_it != split_map_temp.end();){
            if(split_it->second.size() == 1)
            { 
                CandidateSeed<LaneCenterFeature> cs = split_it->second[0];
                auto merge_it = merge_map_temp.find(cs.pe->group_line);
                if(merge_it != merge_map_temp.end()){
                    merge_it->second.push_back(cs);
                }else{
                    merge_map_temp[cs.pe->group_line] = {cs};
                }
            }else{
                split_map.insert({split_it->second[0].ps, split_it->second});
            }
            split_it++;
        }
        
        //直行
        for(auto merge_it = merge_map_temp.begin(); merge_it != merge_map_temp.end();){
            if(merge_it->second.size() == 1) 
            { 
                CandidateSeed<LaneCenterFeature> cs = merge_it->second[0];
                straight_map[cs.ps] = {cs};
            }else{
                merge_map.insert({merge_it->second[0].ps, merge_it->second});
            }
            merge_it++;
        }

        for(const auto& pair : split_map){
            sm_cs_classify_m["split"].push_back(pair.second);
            split_cnt++;
        }
        for(const auto& pair : merge_map){
            sm_cs_classify_m["merge"].push_back(pair.second);
            merge_cnt++;
        }
        for(const auto& pair : straight_map){
            sm_cs_classify_m["straight"].push_back(pair.second);
            straight_cnt++;
        }

        //2.1 找断点  从后往前找， 避免翘起来 ，并去掉翘起来的点
        //一处地方的全部分合流
        align_and_gen_bk(one_sm_match, sm_cs_classify_m);

        split_merge_connect_candidates_list.push_back(sm_cs_classify_m);
    }
    
    LOG_INFO("sm_cs_origin_list.size : {}", sm_cs_origin_list.size());
    LOG_INFO("split_merge_scenes.size : {}", split_merge_connect_candidates_list.size());
    LOG_INFO("split_candidates size: {}",split_cnt);
    LOG_INFO("merge_candidates size: {}",merge_cnt);
    LOG_INFO("straight_candidates size : {}",straight_cnt);
    
    //3. 连接
    LOG_INFO("start connecting split_merge_connect_candidates_list")
    for(const auto& info_m : split_merge_connect_candidates_list){ // 所有分合流
        for(const auto& pair : info_m){ // 某一处分合流：key是string，直行，分，合
            for(const auto& cs_list : pair.second){ // 某一处分合流的直行或分或合的几条线，并不是该处所有的线
                connect_line(cs_list, session, pair.first);
            }
        }
    }
    LOG_INFO("finish connected split_merge_connect_candidates_list")
    update_lane_center_sample_tree(session);

    return fsdmap::SUCC;
}

int RoadModelProcSplitMerge::connect_split_merge_lane_line(RoadModelSessionData *session)
{
    // 1. 通过中心线的分合流线，找到匹配车道线
    // 1.1 计算左侧断点垂线与中心线的交点， 计算右侧断点垂线与中心线的交点
    // 1.2 计算左侧断点垂线与车道线的交点， 计算右侧断点垂线与车道线的交点
    // 1.3 计算匹配关系，与中心线交点的匹配关系
    // 1.4 计算中心线与车道线是否有相交(可能需要延长车道线的末端点) 或 都往分割线上投影，看是否有重合区间



    // 2. 拟合车道线

    // 3. 参考中心线的分合流信息，对车道线进行增删

    
    return fsdmap::SUCC;
}


int RoadModelProcSplitMerge::merge_sm_break_candidate(RoadModelSessionData *session)
{

    // 1. 剔除重复的分叉点 (这一步也是必须要的，可以当做一个缓冲区使用； 也就是分合流区间往外扩10的缓冲区，缓冲区内的分叉点也删除)
    int split_merge_size = session->split_merge_break_points.size();
    for (int i = 0; i < split_merge_size; i++) {
        auto bkp = session->split_merge_break_points[i];
        if (bkp->valid == false) {
            continue;
        }
        std::vector<BreakInfo*> bk_points;
        double search_radius = 10;
        session->sm_break_candidate_tree.search(bkp->pos, search_radius, bk_points);
        if (bk_points.size() > 0) {
            bkp->valid = false;
        }
    }
    
    // 2. 先初步用附近则中心线，追踪到道路方向 v1
    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto pt : line->list)
        {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }

    auto find_road_vertical_dir = [&lane_center_tree] (BreakInfo* bk) {
        std::vector<Eigen::Vector3d> dirs;
        std::vector<LaneCenterFeature *> lcfs;
        lane_center_tree.search(bk->pos, 10, lcfs);
        if (!lcfs.empty()) {
            for (const auto lc : lcfs) {
                auto vdir = alg::get_vertical_dir(lc->dir);
                dirs.push_back(vdir);
            }
            Eigen::Vector3d road_vertical_dir = alg::get_direction(dirs);
            bk->v_dir = road_vertical_dir;
            bk->dir = alg::get_vertical_dir(road_vertical_dir);
            return true;
        } else {
            return false;
        }
    };


    // 3. 过滤掉分合流区间中的交叉点
    auto check_bc_in_sm_range = [&session](Eigen::Vector3d pos_in, Eigen::Vector3d dir_in, Eigen::Vector3d vdir_in){
        for (auto& smr : session->split_merge_ranges) {
            auto& bk_from = smr.start;
            auto& bk_end = smr.end;

            auto p1 = alg::get_hori_pos(pos_in, vdir_in, +20);
            auto p2 = alg::get_hori_pos(pos_in, vdir_in, -20);

            auto dir = alg::get_dir(bk_from->pos, bk_end->pos);
            double theta = alg::calc_theta(dir, dir_in, true); //2.计算方向一致性

            Eigen::Vector3d cross;
            bool find = alg::get_cross_point_by_point(p1, p2, bk_from->pos, bk_end->pos, cross, false, 0);
            if(theta < 90 && find) { // 同方向 且 和分合流区间有交点，则直接过滤掉该断点
                return true;
            }
        }

        return false;
    };
    
    std::vector<BreakInfo*> split_merge_break_points_tmp;
    for (auto bk: session->split_merge_break_points) {
        if(!bk || !bk->valid) {
            continue;
        }

        // 刷新分叉点的 dir 和 v_dir
        if(!find_road_vertical_dir(bk)) {
            continue;
        }

        if(check_bc_in_sm_range(bk->pos, bk->dir, bk->v_dir)) {
            continue;
        }
        
        split_merge_break_points_tmp.push_back(bk);
    }
    session->split_merge_break_points.clear();
    session->split_merge_break_points.swap(split_merge_break_points_tmp);

    // 4. 过滤掉分合流中短的中心线
    auto get_end=[](LaneCenterFeature*lc,bool is_end,double& length)
    {
        length=0;
        LaneCenterFeature*next_lc=lc;
        int count=0;
        while(next_lc) {
            auto next=is_end?next_lc->next:next_lc->prev;
            if(next) {
                length+=alg::calc_dis(next->pos,next_lc->pos);
            }
            if(!next) {
                break;
            }
            next_lc=next;
        }
        return next_lc;
    };

    std::vector<LaneCenterGroupLine*> temp_merge_center_list;
    for (auto line : session->merge_lane_center_list) {
        const int size = line->list.size();
        if(size == 0) {
            continue;
        }

        double length_to_end = 0;
        get_end(line->list[0].get(), true, length_to_end);
        if (length_to_end > 10) {
            temp_merge_center_list.push_back(line);
            continue;
        }

        double hit_score = 0;
        for (int i = 0; i < size; i++) {
            auto& pt = line->list[i];
            auto v_dir = alg::get_vertical_dir(pt->dir);
            if(pt->point_status != 1 && check_bc_in_sm_range(pt->pos, pt->dir, v_dir)) {
                hit_score++;
            }
        }

        hit_score = hit_score/static_cast<double>(size);
        if(hit_score < 0.90) { // 有90以上的比例在分合流区间则丢弃
            temp_merge_center_list.push_back(line);
        }
    }
    std::swap(temp_merge_center_list,session->merge_lane_center_list);
    update_lane_center_sample_tree(session);

    // 5. 添加拟合的分合流打断点
    for (int i = 0; i < sm_break_candidate.size(); i++) {
        session->split_merge_break_points.push_back(sm_break_candidate[i]);
    }

    return fsdmap::SUCC;
}

void RoadModelProcSplitMerge::connect_line(const std::vector<CandidateSeed<LaneCenterFeature>>& cs_list, 
     RoadModelSessionData *session, std::string type)
{

    auto merge_delete_line_by_straight = [&](const CandidateSeed<LaneCenterFeature>& cs, bool delete_prev_pts)
    {
        // cs.is_used = true;
        // std::cout <<"5.1" <<std::endl;
        LaneCenterGroupLine* prev_line = cs.ps->group_line;
        LaneCenterGroupLine* next_line = cs.pe->group_line;
        if(prev_line == nullptr || next_line == nullptr){
            return;
        }
        // TODO: 晓峰，如果只存在一个点，应该要考虑

        LOG_WARN("[straight]delete prev、next: {}, start {},{},{}, end {},{},{}, prev_line_id:{}, next_line_id:{}", 
                delete_prev_pts,
                cs.ps->pos.x(), cs.ps->pos.y(), cs.ps->pos.z(), cs.pe->pos.x(), cs.pe->pos.y(), cs.pe->pos.z(),
                prev_line->id, next_line->id);
        int min_index = -1, max_index = -1;

        if(prev_line == next_line) {
            std::vector<std::shared_ptr<LaneCenterFeature>> added_list;

            int prev_index = -1;
            int next_index = -1;
            for(int i = 0; i< next_line->list.size(); i++) {
                auto& pt = next_line->list[i];
                if(pt.get() == cs.ps) {
                    prev_index = i;
                }
                if(pt.get() == cs.pe) {
                    next_index = i;
                }
            }
            if(prev_index == -1 || next_index == -1) {
                LOG_ERROR("prev_index:{} next_index:{}", prev_index, next_index);
                return;
            }
            
            // LOG_ERROR("1 prev_index:{} next_index:{}, size:{}", prev_index, next_index, next_line->list.size());
            for(int i = 0; i<= prev_index; i++) {
                auto& pt = next_line->list[i];
                added_list.push_back(pt);
                if(i == prev_index) {
                    pt->point_status = 1;
                    pt->endpoint_status = 2;
                    pt->v_road_dir = cs.v_road_dir;

                    min_index = added_list.size() - 1;
                }
            }
            // LOG_ERROR("2 prev_index:{} next_index:{}, size:{}", prev_index, next_index, added_list.size());
            // LOG_ERROR("2 prev_index:{} next_index:{}, size:{}", prev_index, next_index, cs.points.size());
            for(int i = 0; i < cs.points.size(); i++) {
                auto& pt = cs.points[i];
                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                
                // new_node->init(pt.get());

                new_node->pos = pt.pos;
                new_node->boundary_type = LaneType::SPLIT_MERGE_LC; 
                new_node->type = pt.type;
                new_node->color = pt.color;
                new_node->group_line = next_line;
                new_node->prev = NULL;
                new_node->next = NULL;
                
                // 分合流区间的中心点，后面进行边界识别
                new_node->point_status = 1;

                added_list.push_back(new_node);
            }
            // LOG_ERROR("3 prev_index:{} next_index:{}, size:{}", prev_index, next_index, added_list.size());

            for(int i = next_index; i < next_line->list.size(); i++) {
                auto& pt = next_line->list[i];
                added_list.push_back(pt);
                if(i == next_index) {
                    pt->point_status = 1;
                    max_index = added_list.size() - 1;

                    // TODO：是否需要设置
                    // pt->endpoint_status = 2;
                    // pt->v_road_dir = cs.v_road_dir;
                }
            }
            // LOG_ERROR("4 prev_index:{} next_index:{}, size:{}", prev_index, next_index, added_list.size());

            next_line->list.clear();
            next_line->list = added_list;

            {
                bool dbg = true;
                //设新点的dir和前驱后继
                std::shared_ptr<LaneCenterFeature> prev = nullptr;
                // for(int i = 0 ; i < add_points_num + 1; i++){ // 多加一个原始线上的点，为了计算每个新加点的方向
                for(int i = 0; i < next_line->list.size(); i++){ // 多加一个原始线上的点，为了计算每个新加点的方向
                    auto& cur_pt = next_line->list[i];

                    if (prev == nullptr) {
                        cur_pt->group_line = next_line;
                        prev = cur_pt;
                        continue;
                    }
                    cur_pt->dir = alg::get_dir(cur_pt->pos, prev->pos);
                    prev->dir = cur_pt->dir;
                    cur_pt->set_prev(prev.get());
                    cur_pt->group_line = next_line;
                    prev = cur_pt;

                    // 重新连接后的分合流线，保存到新的容器中
                    if(min_index <= i && i <= max_index && cur_pt->point_status == 1) {
                        cs.one_sm_match->sm_fit_lines.center_lines_reconnected[cs.match_id].push_back(cur_pt.get());
                        if(dbg) {
                            LOG_ERROR("dbg1:{}", dbg);
                            dbg = false;
                        }
                    }
                }
            }
        } else {
            //1、删除prev_line 、next_line多余的点
            auto ps_it = std::find_if(prev_line->list.begin(), prev_line->list.end(),
                [&](const std::shared_ptr<LaneCenterFeature>& pt)
                {return pt.get() == cs.ps;});

            if(ps_it == prev_line->list.end()) {
                LOG_WARN("[straight 0.1]ps_it is null");
                return;
            }
            
            auto pe_it = std::find_if(next_line->list.begin(), next_line->list.end(),
            [&](const std::shared_ptr<LaneCenterFeature>& pt)
            {return pt.get() == cs.pe;});
            if(pe_it == next_line->list.end()) {
                LOG_WARN("[straight 0.2]pe_it is null");
                return;
            }

            prev_line->list.erase(std::next(ps_it), prev_line->list.end());
            // 置空
            ps_it = prev_line->list.end();
            
            next_line->list.erase(next_line->list.begin(), pe_it); //左闭  右开
            LOG_WARN("[straight]delete line id prev {}, next:{}", prev_line->id, next_line->id);
            // 置空
            pe_it = next_line->list.end();

            // 注意后面：ps_it 和 pe_it 都不能再用了！！！！！
            prev_line->list.back()->point_status = 1;
            // prev_line->list.back()->endpoint_status = 2;
            next_line->list.front()->point_status = 1;
            // next_line->list.front()->endpoint_status = 2;
            LOG_WARN("[straight]delete line id prev {}, next:{}", prev_line->id, next_line->id);

            min_index = prev_line->list.size()-1;
            
            //2、prev 连接points
            for(auto& pt : cs.points){
                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                new_node->pos = pt.pos;
                new_node->boundary_type = LaneType::SPLIT_MERGE_LC; 
                new_node->type = pt.type;
                new_node->color = pt.color;
                new_node->group_line = prev_line;
                new_node->prev = NULL;
                new_node->next = NULL;
                
                // 分合流区间的中心点，后面进行边界识别k
                new_node->point_status = 1;

                prev_line->list.push_back(new_node);
            }
            max_index = prev_line->list.size(); // 注意这里不用减一，因为还需要包含 pe 点
            LOG_WARN("[straight2]delete line id prev {}, next:{}", prev_line->id, next_line->id);
            
            //设新点的dir和前驱后继
            std::shared_ptr<LaneCenterFeature> prev = nullptr;
            for(int i = 0 ; i <prev_line->list.size(); i++){
                auto& cur_pt = prev_line->list[i];
                
                if (prev == nullptr) {
                    prev = cur_pt;
                    continue;
                }
                cur_pt->dir = alg::get_dir(cur_pt->pos, prev->pos);
                prev->dir = cur_pt->dir;
                cur_pt->set_prev(prev.get());
                prev = cur_pt;
            }
            
            //3、连接next
            //更改next_line线和点的属性
            // next_line->id = prev_line->id; 
            for(int i = 0 ; i <next_line->list.size(); i++){
                auto& cur_pt = next_line->list[i];
                cur_pt->group_line = prev_line; // 已经包含cs.pe的group_line
            }
            // cs.pe->group_line = prev_line; 
            prev_line->list.back()->dir = alg::get_dir(next_line->list.front()->pos, prev_line->list.back()->pos);  //设置dir
            next_line->list.front()->set_prev(prev_line->list.back().get()); //设置连接点前驱后继
            prev_line->list.insert(prev_line->list.end(), next_line->list.begin(), next_line->list.end());

            //4、删线
            session->merge_lane_center_list.erase(
                std::remove(session->merge_lane_center_list.begin(), session->merge_lane_center_list.end(), next_line), 
                session->merge_lane_center_list.end());

            // 重新连接后的分合流线，保存到新的容器中
            bool dbg = true;
            for(int i = 0; i < prev_line->list.size(); i++){ // 多加一个原始线上的点，为了计算每个新加点的方向
                auto& cur_pt = prev_line->list[i];

                if(min_index <= i && i <= max_index && cur_pt->point_status == 1) {
                    cs.one_sm_match->sm_fit_lines.center_lines_reconnected[cs.match_id].push_back(cur_pt.get());
                    if(dbg) {
                        LOG_ERROR("dbg2:{}", dbg);
                        dbg = false;
                    }
                }
            }
        }
 
    };

    auto merge_delete_line_by_split = [&](const CandidateSeed<LaneCenterFeature>& cs, bool delete_prev_pts)
    {
        // bool open_debug = true;
        bool open_debug = false;
        // cs.is_used = true;
        LaneCenterGroupLine* prev_line = cs.ps->group_line;
        LaneCenterGroupLine* next_line = cs.pe->group_line;
        if(prev_line == nullptr || next_line == nullptr){
            return;
        }
        
        LOG_WARN("[split]delete prev、next: {}, start {},{},{}, end {},{},{}, prev_line_id:{}, next_line_id:{}", 
                delete_prev_pts,
                cs.ps->pos.x(), cs.ps->pos.y(), cs.ps->pos.z(), cs.pe->pos.x(), cs.pe->pos.y(), cs.pe->pos.z(),
                prev_line->id, next_line->id);

        int min_index = -1, max_index = -1;
        if(prev_line == next_line) {
            std::vector<std::shared_ptr<LaneCenterFeature>> added_list;

            int prev_index = next_line->list.size();
            int next_index = next_line->list.size();
            for(int i = 0; i< next_line->list.size(); i++) {
                auto& pt = next_line->list[i];
                if(pt.get() == cs.ps) {
                    prev_index = i;
                }
                if(pt.get() == cs.pe) {
                    next_index = i;
                }
            }
            if(prev_index == next_line->list.size() || next_index == next_line->list.size()) {
                LOG_ERROR("prev_index:{} next_index:{}", prev_index, next_index);
                return;
            }
            // LOG_ERROR("1 prev_index:{} next_index:{}, size:{}", prev_index, next_index, next_line->list.size());
            for(int i = 0; i<= prev_index; i++) {
                auto& pt = next_line->list[i];
                added_list.push_back(pt);
                if(i == prev_index) {
                    pt->point_status = 1;
                    pt->endpoint_status = 2;
                    pt->v_road_dir = cs.v_road_dir;

                    min_index = added_list.size() - 1;
                }
            }
            // LOG_ERROR("2 prev_index:{} next_index:{}, size:{}", prev_index, next_index, added_list.size());
            // LOG_ERROR("2 prev_index:{} next_index:{}, size:{}", prev_index, next_index, cs.points.size());
            for(int i = 0; i < cs.points.size(); i++) {
                auto& pt = cs.points[i];
                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                
                // new_node->init(pt.get());

                new_node->pos = pt.pos;
                new_node->boundary_type = LaneType::SPLIT_MERGE_LC; 
                new_node->type = pt.type;
                new_node->color = pt.color;
                new_node->group_line = next_line;
                new_node->prev = NULL;
                new_node->next = NULL;
                
                // 分合流区间的中心点，后面进行边界识别
                new_node->point_status = 1;

                added_list.push_back(new_node);
            }
            // LOG_ERROR("3 prev_index:{} next_index:{}, size:{}", prev_index, next_index, added_list.size());

            for(int i = next_index; i < next_line->list.size(); i++) {
                auto& pt = next_line->list[i];
                added_list.push_back(pt);
                if(i == next_index) {
                    pt->point_status = 1;
                    max_index = added_list.size()-1;

                    // TODO：是否需要设置
                    // pt->endpoint_status = 2;
                    // pt->v_road_dir = cs.v_road_dir;
                }
            }
            // LOG_ERROR("4 prev_index:{} next_index:{}, size:{}", prev_index, next_index, added_list.size());

            next_line->list.clear();
            next_line->list = added_list;
            
            // auto ps_it = std::find_if(next_line->list.begin(), next_line->list.end(),[&](const std::shared_ptr<LaneCenterFeature>& pt)
            //         {return pt.get() == cs.ps;});
            // auto pe_it = std::find_if(next_line->list.begin(), next_line->list.end(),[&](const std::shared_ptr<LaneCenterFeature>& pt)
            //         {return pt.get() == cs.pe;});
            // int ps_index = ps_it - next_line->list.begin(); //不要往下挪
            // next_line->list.erase(next_line->list.begin()+ps_index+1, pe_it); //左闭  右开
            // // 坑：使用了 erase 后，ps_it 已经失效，必须重新搜索； 或者用索引
            // next_line->list.insert(next_line->list.begin()+ps_index+1, added_list.begin(), added_list.end());

        } else {

            //1、删除prev_line多余的点 
            if(delete_prev_pts == true){
                auto ps_it = std::find_if(prev_line->list.begin(), prev_line->list.end(),[&](const std::shared_ptr<LaneCenterFeature>& pt)
                        {return pt.get() == cs.ps;});
                // int ps_index = ps_it - prev_line->list.begin(); //不要往下挪
                prev_line->list.erase(std::next(ps_it), prev_line->list.end());
                prev_line->list.back()->next = nullptr;
                LOG_WARN("[split] line id, delete prev {}, next:{}", prev_line->id, next_line->id);
                // 置空
                ps_it = prev_line->list.end();
            }

            //2、处理next线
            //2.1删除next线多余的点
            auto pe_it = std::find_if(next_line->list.begin(), next_line->list.end(),[&](const std::shared_ptr<LaneCenterFeature>& pt)
                    {return pt.get() == cs.pe;});
            next_line->list.erase(next_line->list.begin(), pe_it); //左闭  右开
            // 置空
            pe_it = next_line->list.end();

            //2.2 next 连接 头部points
            auto points_tmp = cs.points;
            points_tmp.insert(points_tmp.begin(), UsefulPnt(cs.ps->pos, cs.ps->dir, cs.ps->type, cs.ps->color));
            int add_points_num = points_tmp.size();
            for(int i = add_points_num-1; i >= 0; i--){

                auto& pt = points_tmp[i];
                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                new_node->pos = pt.pos;
                new_node->boundary_type = LaneType::SPLIT_MERGE_LC; 
                new_node->type = pt.type;
                new_node->color = pt.color;
                new_node->group_line = next_line;
                new_node->prev = NULL;
                new_node->next = NULL;
                
                // 分合流区间的中心点，后面进行边界识别
                new_node->point_status = 1;
                if(i == 0) {
                    new_node->endpoint_status = 2;
                    new_node->v_road_dir = cs.v_road_dir;
                    min_index = 0;
                    max_index = points_tmp.size(); // 这里不用减一，因为要包含pe点

                    auto lc = new_node;
                    if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                        std::cout << lc->pos.transpose() << " lc:" << lc  << " group_line: " << lc->group_line << std::endl;
                        LOG_ERROR("qzc 5 : id:{}",  lc->group_line->id);
                    }
                }

                next_line->list.insert(next_line->list.begin(), new_node);  // TODO：这里速度可以优化  insert是 O（n），改成先points后面加next的点？
            }
            cs.pe->point_status = 1;
        
            // auto new_node = session->add_ptr(session->lane_center_feature_ptr);
            // new_node->pos = cs.ps->pos;
            // next_line->list.insert(next_line->list.begin(), new_node);
        }

        //设新点的dir和前驱后继
        bool dbg = true;
        std::shared_ptr<LaneCenterFeature> prev = nullptr;
        // for(int i = 0 ; i < add_points_num + 1; i++){ // 多加一个原始线上的点，为了计算每个新加点的方向
        for(int i = 0; i < next_line->list.size(); i++){ // 多加一个原始线上的点，为了计算每个新加点的方向
            auto& cur_pt = next_line->list[i];

            if (prev == nullptr) {
                cur_pt->group_line = next_line;
                prev = cur_pt;
                continue;
            }
            cur_pt->dir = alg::get_dir(cur_pt->pos, prev->pos);
            prev->dir = cur_pt->dir;
            cur_pt->set_prev(prev.get());
            cur_pt->group_line = next_line;
            prev = cur_pt;

            // 重新连接后的分合流线，保存到新的容器中
            if(min_index <= i && i <= max_index && cur_pt->point_status == 1) {
                cs.one_sm_match->sm_fit_lines.center_lines_reconnected[cs.match_id].push_back(cur_pt.get());
                if(dbg) {
                    LOG_ERROR("dbg3:{}", dbg);
                    dbg = false;
                }
            }
        }
    };

    auto merge_delete_line_by_merge = [&](const CandidateSeed<LaneCenterFeature>& cs, bool delete_prev_pts)
    {
        // bool open_debug = true;
        bool open_debug = false;
        // cs.is_used = true;
        LaneCenterGroupLine* prev_line = cs.ps->group_line;
        LaneCenterGroupLine* next_line = cs.pe->group_line;
        if(prev_line == nullptr || next_line == nullptr){
            return;
        }

        LOG_WARN("[merge]delete prev、next: {}, start {},{},{}, end {},{},{}, prev_line_id:{}, next_line_id:{}", 
                delete_prev_pts,
                cs.ps->pos.x(), cs.ps->pos.y(), cs.ps->pos.z(), cs.pe->pos.x(), cs.pe->pos.y(), cs.pe->pos.z(),
                prev_line->id, next_line->id);
        
        int min_index = -1, max_index = -1;
        if(prev_line == next_line) {
            std::vector<std::shared_ptr<LaneCenterFeature>> added_list;

            int prev_index = prev_line->list.size();
            int next_index = next_line->list.size();
            for(int i = 0; i< prev_line->list.size(); i++) {
                auto& pt = prev_line->list[i];
                if(pt.get() == cs.ps) {
                    prev_index = i;
                }
                if(pt.get() == cs.pe) {
                    next_index = i;
                }
            }
            if(prev_index == prev_line->list.size() || next_index == prev_line->list.size()) {
                LOG_ERROR("prev_index:{} next_index:{}", prev_index, next_index);
                return;
            }
            // LOG_ERROR("1 prev_index:{} next_index:{}, size:{}", prev_index, next_index, prev_line->list.size());
            for(int i = 0; i<= prev_index; i++) {
                auto& pt = prev_line->list[i];
                added_list.push_back(pt);

                if(i == prev_index) {
                    pt->point_status = 1;

                    min_index = added_list.size() - 1;
                }
            }
            // LOG_ERROR("2 prev_index:{} next_index:{}, size:{}", prev_index, next_index, added_list.size());
            // LOG_ERROR("2 prev_index:{} next_index:{}, size:{}", prev_index, next_index, cs.points.size());
            for(int i = 0; i < cs.points.size(); i++) {
                auto& pt = cs.points[i];
                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                
                // new_node->init(pt.get());

                new_node->pos = pt.pos;
                new_node->boundary_type = LaneType::SPLIT_MERGE_LC; 
                new_node->type = pt.type;
                new_node->color = pt.color;
                new_node->group_line = prev_line;
                new_node->prev = NULL;
                new_node->next = NULL;
                
                // 分合流区间的中心点，后面进行边界识别
                new_node->point_status = 1;

                added_list.push_back(new_node);
            }
            // LOG_ERROR("3 prev_index:{} next_index:{}, size:{}", prev_index, next_index, added_list.size());

            for(int i = next_index; i < prev_line->list.size(); i++) {
                auto& pt = prev_line->list[i];
                added_list.push_back(pt);

                if(i == next_index) {
                    pt->point_status = 1;
                    pt->endpoint_status = 2;
                    pt->v_road_dir = cs.v_road_dir;

                    max_index = added_list.size() - 1;
                }
            }
            // LOG_ERROR("4 prev_index:{} next_index:{}, size:{}", prev_index, next_index, added_list.size());

            prev_line->list.clear();
            prev_line->list = added_list;
        } else {
            //1、删除next_line多余的点 
            if(delete_prev_pts == true){
                auto pe_it = std::find_if(next_line->list.begin(), next_line->list.end(),[&](const std::shared_ptr<LaneCenterFeature>& pt)
                        {return pt.get() == cs.pe;});
                // int pe_index = pe_it - next_line->list.begin(); //不要往下挪
                next_line->list.erase(next_line->list.begin(), pe_it);  //左闭  右开
                next_line->list.front()->prev = nullptr;
                LOG_WARN("[merge] line id, prev {}, delete next:{}", prev_line->id, next_line->id);
                // 置空
                pe_it = next_line->list.end();
            }
            //2、处理prev线
            //2.1删除prev线多余的点
            auto ps_it = std::find_if(prev_line->list.begin(), prev_line->list.end(),[&](const std::shared_ptr<LaneCenterFeature>& pt)
                    {return pt.get() == cs.ps;});
            prev_line->list.erase(std::next(ps_it), prev_line->list.end()); //左闭  右开
            // 置空
            ps_it = prev_line->list.end();

            min_index = prev_line->list.size() - 1;

            //2.2 prev 连接 头部points
            auto points_tmp = cs.points;
            points_tmp.push_back(UsefulPnt(cs.pe->pos, cs.pe->dir, cs.pe->type, cs.pe->color));
            int add_points_num = points_tmp.size();
            for(int i = 0 ; i < add_points_num; i++){
                auto& pt = points_tmp[i];
                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                new_node->pos = pt.pos;
                new_node->boundary_type = LaneType::SPLIT_MERGE_LC; 
                new_node->type = pt.type;
                new_node->color = pt.color;
                new_node->group_line = prev_line;
                new_node->prev = NULL;
                new_node->next = NULL;
                
                // 分合流区间的中心点，后面进行边界识别
                new_node->point_status = 1;
                if(i == add_points_num-1) {
                    new_node->endpoint_status = 2;
                    new_node->v_road_dir = cs.v_road_dir;

                    auto lc = new_node;
                    if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                        std::cout << lc->pos.transpose() << " lc:" << lc  << " group_line: " << lc->group_line << std::endl;
                        LOG_ERROR("qzc 6 : id:{}",  lc->group_line->id);
                    }

                }

                prev_line->list.push_back(new_node);  // TODO：这里速度可以优化  insert是 O（n），改成先points后面加next的点？
            }
            cs.ps->point_status = 1;

            max_index = prev_line->list.size() - 1; // 这里pe 已经加入，所以需要减一
        }
       
        bool dbg = true;
        //设新点的dir和前驱后继
        std::shared_ptr<LaneCenterFeature> prev = nullptr;
        // for(int i = prev_line->list.size() - add_points_num - 1; i < prev_line->list.size() ; i++){ // -1是为了统一， 防止第一个点跳过
        for(int i = 0; i < prev_line->list.size() ; i++){ // -1是为了统一， 防止第一个点跳过
            auto& cur_pt = prev_line->list[i];
            
            if (prev == nullptr) {
                cur_pt->group_line = prev_line;
                prev = cur_pt;
                continue;
            }
            cur_pt->dir = alg::get_dir(cur_pt->pos, prev->pos);
            prev->dir = cur_pt->dir;
            cur_pt->set_prev(prev.get());
            cur_pt->group_line = prev_line;
            prev = cur_pt;

            // 重新连接后的分合流线，保存到新的容器中
            if(min_index <= i && i <= max_index && cur_pt->point_status == 1) {
                cs.one_sm_match->sm_fit_lines.center_lines_reconnected[cs.match_id].push_back(cur_pt.get());
                if(dbg) {
                    LOG_ERROR("dbg4:{}", dbg);
                    dbg = false;
                }
            }

            auto lc = cur_pt;
            if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                std::cout << lc->pos.transpose() << " lc:" << lc  << " group_line: " << lc->group_line << std::endl;
                LOG_ERROR("qzc 7 : id:{}",  lc->group_line->id);
            }
        }
 
    };

    auto is_prev_next_same_line = [&](const std::vector<CandidateSeed<LaneCenterFeature>>& cs_list, int& same_index) -> bool
    {   
        if(cs_list.empty()){
            return false;
        }
        //TODO:cxf 目前矢量化给过来不会出现两条prev-next一样的， 默认最多有一条一样。如果前面优化了，这里要改进。
        for(int i = 0 ; i < cs_list.size(); i++){ // 同类型（直行 或 分 或 合）的多条线
            // std::cout <<"s1" <<std::endl;
            // std::cout << "cs_list[i].ps->group_line" << cs_list[i].ps->group_line <<std::endl;
            // std::cout << "cs_list[i].pe->group_line" << cs_list[i].pe->group_line <<std::endl;
            if(cs_list[i].ps->group_line == cs_list[i].pe->group_line){
                // std::cout <<"s2" <<std::endl;
                same_index = i;
                return true;
            }
        }
        return false;
    };

    auto break_oneline_to_twoline = [&](const CandidateSeed<LaneCenterFeature>& cs)
    {
        // cs.is_used = true;
        auto& old_line = cs.ps->group_line;

        auto break_it = std::find_if(old_line->list.begin(), old_line->list.end(),
            [&](const std::shared_ptr<LaneCenterFeature>& pt)
            {return pt.get() == cs.ps;});

        if(break_it != old_line->list.end())
        {
            int break_index = break_it - old_line->list.begin();
            
            //1. 新线处理
            auto new_line = session->add_ptr(session->lane_center_group_line_ptr);  

            new_line->id = std::to_string(session->lane_center_id++); // //待修改id统一
            new_line->boundary_type = LaneType::SPLIT_MERGE_LC;

            //打断点地方，单独新创建一个点给新线
            auto new_node = session->add_ptr(session->lane_center_feature_ptr);
            new_node->pos = old_line->list[break_index]->pos;;
            new_node->group_line = new_line.get();

            //新线加点
            new_line->list.insert(new_line->list.end(), old_line->list.begin(), break_it - 1); 
            new_line->list.push_back(new_node); //末尾追加 断点

            //设置前驱后继属性、dir、group_line
            std::shared_ptr<LaneCenterFeature> prev = nullptr; 
            for(auto& cur_pt : new_line->list){
                if (prev == nullptr) {
                    cur_pt->group_line = new_line.get();
                    prev = cur_pt;
                    continue;
                }
                cur_pt->dir = alg::get_dir(cur_pt->pos, prev->pos);
                prev->dir = cur_pt->dir;
                cur_pt->set_prev(prev.get());
                cur_pt->group_line = new_line.get();
                prev = cur_pt;
            }

            if(!new_line->list.empty())
            {
                session->merge_lane_center_list.push_back(new_line.get());
            }

            //2. 旧线处理，剔除分给新线的点
            old_line->list[break_index]->prev = nullptr; //取消原线打断点处的 prev指向
            old_line->list.erase(old_line->list.begin(), break_it - 1); 
            old_line->list.back()->next = nullptr;
        }else{
            LOG_WARN("分合流ps pe同一条线这里有异常");
            return;
        }
    };

    if(type == "split")
    {
        //1 要先判断next 和prev相等的情况  ，有则拆线， 
        int same_index = -1;
        bool has_same_line = is_prev_next_same_line(cs_list, same_index);
        if(has_same_line == true){
            // std::cout << "has_same_line" <<std::endl;
            // std::cout << "00 split break_index:" << cs_list[0].ps->pos.transpose() <<std::endl;
            // std::cout << "01 split break_index:" << cs_list[1].ps->pos.transpose() <<std::endl;


            //1.1首点相同，prev变为新线，并更改原线状态  prev的特征点也会随着变为指向新线。
            // break_oneline_to_twoline(cs_list[same_index]); //现在同一条线不需要打断

            //1.2连接剩余的线
            for(int i = 0; i <cs_list.size(); i++){
                // if(i == same_index){
                //     continue;
                // }
                merge_delete_line_by_split(cs_list[i], false);
            }

        }else{ 
            //2.1 连第一次的时候，先删除prev线的点，  
            //2.2 继续处理剩下的prev-next都不为一条线  ， 不需要删除prev线的点
            bool delete_prev_pts = true;
            for(int i = 0; i <cs_list.size(); i++){
                merge_delete_line_by_split(cs_list[i], delete_prev_pts);
                delete_prev_pts = false;
            }
        }

        //===========下面弃用=================
        // double min_hor_dis = DBL_MAX;
        // int straight_index = -1;
        // std::cout << "cs_list.size()" << cs_list.size() <<std::endl;
        // for(int i = 0; i < cs_list.size(); i++ ){
        //     auto cs = cs_list[i];
        //     double dis = alg::calc_hori_dis(cs.ps->pos,cs.pe->pos,cs.v_road_dir,false);
        //     if(dis < min_hor_dis){
        //         min_hor_dis = dis;
        //         straight_index = i;
        //         continue;
        //     }
        // }
        // std::cout << "straight_index: " << straight_index <<std::endl;
        // //连接直线那条
        // std::cout << "1" <<std::endl;
        // merge_delete_line_by_straight(cs_list[straight_index]);
        // std::cout << "-1" <<std::endl;


    }else if(type == "merge"){
        //1 要先判断next 和prev相等的情况  ，有则拆线， 
        int same_index = -1;
        bool has_same_line = is_prev_next_same_line(cs_list, same_index);
        if(has_same_line == true){
            // std::cout << "has_same_line" <<std::endl;
            // std::cout << "00 split break_index:" << cs_list[0].ps->pos.transpose() <<std::endl;
            // std::cout << "01 split break_index:" << cs_list[1].ps->pos.transpose() <<std::endl;

            //1.1首点相同，prev变为新线，并更改原线状态  prev的特征点也会随着变为指向新线。
            // break_oneline_to_twoline(cs_list[same_index]);//现在同一条线不需要打断

            //1.2连接剩余的线
            for(int i = 0; i <cs_list.size(); i++){
                // if(i == same_index){
                //     continue;
                // }
                merge_delete_line_by_merge(cs_list[i], false);
            }

        }else{ 
            //2.1 连第一次的时候，先删除prev线的点，  
            //2.2 继续处理剩下的prev-next都不为一条线  ， 不需要删除prev线的点
            bool delete_prev_pts = true;
            for(int i = 0; i <cs_list.size(); i++){
                merge_delete_line_by_merge(cs_list[i], delete_prev_pts);
                delete_prev_pts = false;
            }
        }
    }else if(type == "straight")
    {
        // std::cout <<"5" <<std::endl;
        //1 要先判断next 和prev相等的情况 ，有则拆线， 
        int same_index = -1;
        bool has_same_line = is_prev_next_same_line(cs_list, same_index);
        if(has_same_line == true) {
            // 冗余代码，后面的老铁可以直接删除
            for(int i = 0; i <cs_list.size(); i++){
                bool delete_prev_pts = true;
                merge_delete_line_by_straight(cs_list[i], delete_prev_pts);
            }
        } else{
            for(int i = 0; i <cs_list.size(); i++){
                bool delete_prev_pts = true;
                merge_delete_line_by_straight(cs_list[i], delete_prev_pts);
            }
        }

        // if(has_same_line == false){
        //     for(int i = 0; i <cs_list.size(); i++){
        //         bool delete_prev_pts = true;
        //         merge_delete_line_by_straight(cs_list[i], delete_prev_pts);
        //     }
        // }
        // std::cout <<"6" <<std::endl;
    }
}



int RoadModelProcSplitMerge::save_debug_info(RoadModelSessionData* session, int stage) {
    if (!FLAGS_sm_save_data_enable) {
        return fsdmap::SUCC;
    }

    std::string display_name = "split_merge" + std::to_string(stage);
    session->set_display_name(display_name.c_str());

    //任务的中心点
    auto center_log = session->add_debug_log(utils::DisplayInfo::POINT, "center");
    center_log->color = {235, 52, 235};
    center_log->add(session->data_processer->_center_link_pos, 10);  
    auto center_text_log= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", "site_center");
    center_text_log->color = {235, 52, 235}; 
    center_text_log->add(session->data_processer->_center_link_pos);


    if(stage == 7){
        // 打印所有没有前驱后继的分合流点
        for (int i = 0; i < session->classified_lc_points.size(); i++) {
            auto log_sm_classified = session->add_debug_log(utils::DisplayInfo::LINE, "split_merge_no_prev_next {}", i);
            log_sm_classified->color ={0, 255, 0};// 绿色
            Eigen::Vector3d center = session->classified_lc_points[i]->pos;
            Eigen::Vector3d v_road_dir = alg::get_vertical_dir(session->classified_lc_points[i]->dir);
            Eigen::Vector3d s = center + v_road_dir * 5;
            Eigen::Vector3d e = center - v_road_dir * 5;
            log_sm_classified->add(s);
            log_sm_classified->add(e);
        }

        auto log_sm_matched = session->add_debug_log(utils::DisplayInfo::POINT, "split_merge_no_prev_next");
        log_sm_matched->color ={100, 255, 0};// 浅绿色
        for (int i = 0; i < classified_lc_points_for_visual_debug.size(); i++) {
            Eigen::Vector3d center = classified_lc_points_for_visual_debug[i]->pos;
            log_sm_matched->add(center);
        }
    }

    //可视化道路边界曲率
    if(stage == 7){
        // 打印所有非主路口的分岔路口
        // auto log_fork_road_pts = session->add_debug_log(utils::DisplayInfo::POINT, "log_fork_road_pts");
        for (int i = 0; i < raw_link_fork_points_for_visual.size(); i++) {
            // log_fork_road_pts->color ={255, 150, 100};// 浅橘色
            // log_fork_road_pts->add(raw_link_fork_points_for_visual[i]);

            auto log_split_merge_vertial_link = session->add_debug_log(utils::DisplayInfo::LINE, "split_merge_link {}", i);
            log_split_merge_vertial_link->color ={255, 150, 100};// 浅橘色
            Eigen::Vector3d center = raw_link_fork_points_for_visual[i];
            Eigen::Vector3d v_road_dir;
            if (session->sm_matchs.size() > 0 && 
                session->sm_matchs[0] != nullptr && 
                session->sm_matchs[0]->sm_fit_lines.center_lines.size() > 0) {
                v_road_dir = session->sm_matchs[0]->sm_fit_lines.v_road_dir;
            } else {
                v_road_dir = {1, 0, 0};
            }
            Eigen::Vector3d s = center + v_road_dir * 5;
            Eigen::Vector3d e = center - v_road_dir * 5;
            log_split_merge_vertial_link->add(s);
            log_split_merge_vertial_link->add(e);
        }
        

        
        auto log_split_merge_pts = session->add_debug_log(utils::DisplayInfo::POINT, "split_merge");
        log_split_merge_pts->color = {255, 255, 0}; // 黄色
        // for (int i = 0; i < split_merge_points.points.size(); i++) {
        //     auto& split_merge_point = split_merge_points.points[i];
        //     for (int j = 0; j < split_merge_point.size(); j++) {
        //         log_split_merge_pts->add(split_merge_point[j]);
        //     }
        // }
        int cnt =0; 
        for (int i = 0; i < session->sm_matchs.size(); i++) {
            auto& one_sm_match = session->sm_matchs[i];
            auto& sm_fit_lines = one_sm_match->sm_fit_lines;
            for (int j = 0; j < sm_fit_lines.center_lines.size(); j++) {
                if (sm_fit_lines.valid_status[j].is_valid == false) {
                    continue;
                }
                auto& lc_pnts = sm_fit_lines.center_lines[j];
                for (int k = 0; k < lc_pnts.size(); k++) {
                    // 打印点
                    if(k == 0){
                        auto start_text_log= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", cnt);
                        start_text_log->color = {255, 255, 0}; 
                        start_text_log->add(lc_pnts[k]->pos);

                        auto start_log = session->add_debug_log(utils::DisplayInfo::POINT, "split_merge");
                        start_log->color = {255, 0, 0}; 
                        start_log->add(lc_pnts[k]->pos);
                    }else if(k != lc_pnts.size() - 1 && k != 0){
                        auto &ele = log_split_merge_pts->add(lc_pnts[k]->pos);
                        if(j % 2 == 0) {
                            ele.color = {255, 255, 0}; //黄色
                        } else {
                            ele.color = {255, 128, 0}; //橙色
                        }
                    }else{
                        auto end_text_log= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", cnt);
                        end_text_log->color = {255, 255, 0}; 
                        end_text_log->add(lc_pnts[k]->pos);

                        auto end_log = session->add_debug_log(utils::DisplayInfo::POINT, "split_merge");
                        end_log->color = {0, 0, 255}; 
                        end_log->add(lc_pnts[k]->pos);
                    }
                    cnt++;
              

                    // 打印曲率
                    {
                        // double curvature = std::round(lc_pnts[k]->curvature * 1000)/1000;
                        int curvature = std::round(lc_pnts[k]->curvature); // 注意此处为了打印好看，用了int
                        auto log_bd_corvature= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", curvature);
                        log_bd_corvature->color = {255, 0, 128}; 
                        log_bd_corvature->add(lc_pnts[k]->pos);
                    }
                }
            }

            // 打印分合流点
            {
                auto log_split_merge_vertial = session->add_debug_log(utils::DisplayInfo::LINE, "split_merge_vertial {}", i);
                log_split_merge_vertial->color = {255, 0, 0}; // 红色
                Eigen::Vector3d center = sm_fit_lines.center;
                Eigen::Vector3d v_road_dir = sm_fit_lines.v_road_dir;
                Eigen::Vector3d s = center + v_road_dir * 7;
                Eigen::Vector3d e = center - v_road_dir * 7;
                log_split_merge_vertial->add(s);
                log_split_merge_vertial->add(e);
            }
        }

        #if 1
        for(auto &info : session->hor_corss_line_m){
            for(auto& road_cross_pt : info.second->cross_feature_list){
                if(!road_cross_pt->road_boundary_pts.empty()){
                    for(int j = 0; j < 2; j++){
                        if(road_cross_pt->road_boundary_pts[j]->status == PointStatus::RAW){ 
                            // double curvature = std::round(road_cross_pt->road_boundary_pts[j]->curvature * 1000)/1000;
                            int curvature = std::round(road_cross_pt->road_boundary_pts[j]->curvature);
                            auto log_bd_corvature= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", curvature);
                            log_bd_corvature->color = {255, 0, 128}; 
                            log_bd_corvature->add(road_cross_pt->road_boundary_pts[j]->pos);
                        }
                    }
                }
            }               
        }
        #endif
    }



    //=========merge_feature的线==============
    int cnt = 0;
    for (auto& group_list : session->merge_boundary_line_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_boundary");
        if (stage == 0) {
            log = session->add_debug_log(utils::DisplayInfo::POINT, "merge_boundary");
        }
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

            // if(j < 2 || d>=4 || j==group_list->list.size()-1 || group_list->boundary_type == LaneType::ISLAND_RB) {
            if(1) {
                auto &ele = log->add(pt->pos, 1);
                ele.label.opt_label = j;
                ele.label.intensity_opt = group_list->cur_line_id;
                ele.label.score =cnt;
                if (pt->invalid()) {
                    ele.color = {255, 255, 100};
                }
                ele.label.label = 1; // 临时弄的
                prev=pt.get();
            }
        }
        cnt++;
    }
    #if 1
    cnt = 0;
    for (auto& group_list : session->merge_lane_line_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane");
        if (stage == 0) {
            log = session->add_debug_log(utils::DisplayInfo::POINT, "merge_lane");
        }
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

            // if(j<2 || d>=4 || j==group_list->list.size()-1) {
            if(1) {
                auto &ele = log->add(pt->pos, 1);
                ele.label.opt_label = j;
                ele.label.intensity_opt = group_list->cur_line_id;
                ele.label.score = cnt;
                ele.label.label = 2; // 临时弄的
                // ele.label.score = pt->score;
                if (pt->invalid()) {
                    ele.color = {255, 0, 0};
                }
                ele.label.cloud_pano_seg=pt->attr.type;
                ele.label.cloud_line_seg=pt->attr.color;
                // 
                // ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                // LOG_INFO("merge color:{} type:{}",pt->attr.color,pt->attr.type)
                prev=pt.get();
            }
        }
        cnt++;
    }

    cnt = 0;
    for (auto& group_list : session->merge_lane_center_list) {
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
            if(j<2 || d>=4 || j==group_list->list.size()-1) {
            // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                auto &ele = log->add(pt->pos, 1);
                ele.label.opt_label = j;
                ele.label.intensity_opt = group_list->cur_line_id;
                ele.label.score = cnt;
                ele.label.label = 3; // 临时弄的
                if (pt->invalid()) {
                    ele.color = {100, 100, 100};
                }
                // ele.label.intensity_opt=cnt++;
                prev=pt.get();
            } 
        }
        cnt++;
    }
    #endif

    session->save_debug_info(display_name.c_str());

    return fsdmap::SUCC;
}

int RoadModelProcSplitMerge::save_debug_info_connect(RoadModelSessionData* session, int stage) {
    int count=0;

    std::string display_name = "split_merge_connect_" + std::to_string(stage);
    session->set_display_name(display_name.c_str());
    for(auto& one_sm_match : session->sm_matchs) {
        for(auto& it : one_sm_match->line_matchs) {
            LOG_INFO("print match lc prev-next:{}<-->{}, left ll:{}<-->{}, right ll:{}<-->{}, status {}",
                it.lc[0], it.lc[1], it.ll_left[0], it.ll_left[1], it.ll_right[0], it.ll_right[1],
                std::bitset<8>(it.match_status).to_string());


            // 1. 中心线
            {
                int prev_index = it.lc[0];
                int next_index = it.lc[1];
                auto prev_cp_lc_itr = one_sm_match->prev_cp_lc.begin();
                advance(prev_cp_lc_itr, prev_index);
                auto left_pt = prev_cp_lc_itr->second[0].second->pos; // 此处的second[0]：左侧最靠近分割线的点

                auto next_cp_lc_itr = one_sm_match->next_cp_lc.begin();
                advance(next_cp_lc_itr, next_index);
                auto right_pt = next_cp_lc_itr->second[0].second->pos; // 此处的second[0]：右侧最靠近分割线的点
            
                auto log_sm_match = session->add_debug_log(utils::DisplayInfo::LINE, "sm_match lc {}", count++);
                log_sm_match->add(left_pt);
                log_sm_match->add(right_pt);
                log_sm_match->color ={0, 255, 0};// 绿色
            }

            // 2. 车道线(左)
            {
                int prev_index = it.ll_left[0];
                int next_index = it.ll_left[1];
                if(prev_index >= 0 && next_index >= 0) {
                    auto prev_cp_ll_itr = one_sm_match->prev_cp_ll.begin();
                    advance(prev_cp_ll_itr, prev_index);
                    auto left_pt = prev_cp_ll_itr->second[0].second->pos; // 此处的second[0]：左侧最靠近分割线的点

                    auto next_cp_ll_itr = one_sm_match->next_cp_ll.begin();
                    advance(next_cp_ll_itr, next_index);
                    auto right_pt = next_cp_ll_itr->second[0].second->pos; // 此处的second[0]：右侧最靠近分割线的点

                    auto log_sm_match = session->add_debug_log(utils::DisplayInfo::LINE, "sm_match ll left {}", count++);
                    log_sm_match->add(left_pt);
                    log_sm_match->add(right_pt);
                    if(alg::is_bit_set(it.match_status, 3)) {
                        log_sm_match->color ={0, 0, 255};// 蓝色, 左实线
                    } else if(alg::is_bit_set(it.match_status, 4)) {
                        log_sm_match->color = {127, 255, 212}; // 蓝晶，左虚线
                    }
                }
            }

            // 3. 车道线(右)
            {
                int prev_index = it.ll_right[0];
                int next_index = it.ll_right[1];
                if(prev_index >= 0 && next_index >= 0) {
                    auto prev_cp_ll_itr = one_sm_match->prev_cp_ll.begin();
                    advance(prev_cp_ll_itr, prev_index);
                    auto left_pt = prev_cp_ll_itr->second[0].second->pos; // 此处的second[0]：左侧最靠近分割线的点

                    auto next_cp_ll_itr = one_sm_match->next_cp_ll.begin();
                    advance(next_cp_ll_itr, next_index);
                    auto right_pt = next_cp_ll_itr->second[0].second->pos; // 此处的second[0]：右侧最靠近分割线的点

                    auto log_sm_match = session->add_debug_log(utils::DisplayInfo::LINE, "sm_match ll right {}", count++);
                    log_sm_match->add(left_pt);
                    log_sm_match->add(right_pt);
                    if(alg::is_bit_set(it.match_status, 1)) {
                        log_sm_match->color ={255, 255, 255};// 白色, 右实线
                    } else if(alg::is_bit_set(it.match_status, 2)) {
                        log_sm_match->color ={128, 128, 128};// 灰色，右虚线
                    }
                }
            }
        }
    }

    for (int i = 0; i < sm_break_candidate.size(); i++) {
        if (sm_break_candidate[i]->valid == false) {
            continue;
        }
        
        auto log_split_merge_vertial_start_end = session->add_debug_log(utils::DisplayInfo::LINE, "split_merge_start_end {}", count++);
        log_split_merge_vertial_start_end->color ={255, 0, 0};// 红色
        Eigen::Vector3d center = sm_break_candidate[i]->pos;
        Eigen::Vector3d v_road_dir = sm_break_candidate[i]->v_dir;
        Eigen::Vector3d s = center + v_road_dir * 5;
        Eigen::Vector3d e = center - v_road_dir * 5;
        log_split_merge_vertial_start_end->add(s);
        log_split_merge_vertial_start_end->add(e);
        std::cout << "qzc sm_break_candidate.size(): " << sm_break_candidate.size() << " i " << i 
                    << " center: " << center.transpose() << " dir: " << v_road_dir.transpose() << std::endl;
    }
    session->save_debug_info(display_name.c_str());

    
    return fsdmap::SUCC;
}


int RoadModelProcSplitMerge::save_debug_info_all(RoadModelSessionData* session, int stage)
{
    if (!FLAGS_sm_detial_img_enable) {
        return fsdmap::SUCC;
    }

    int count=0;

    
    session->set_display_name("sm_cs_origin_list");
    for(auto line:sm_cs_origin_list)
    {
        auto llog = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        llog->color={227, 227, 39}; //黄色
     

        auto log1 = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        log1->color= {255,0,0};
        auto& ele1 = log1->add(line.ps->pos);
        ele1.label.score=1;
        ele1.label.label=1;

        int i = 2;

        for(auto p:line.points)
        {
            auto& ele2 =llog->add(p.pos);
            ele2.label.score=i++;
            ele2.label.label=100;
        }

        auto log3 = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        log3->color= {0,0,255};
        auto& ele3=log3->add(line.pe->pos);
        ele3.label.score=300;
        ele3.label.label=300;
    }
    session->save_debug_info("sm_cs_origin_list");

    session->set_display_name("split_merge_connect_candidates_list");
    for(auto info_m : split_merge_connect_candidates_list){
        for(auto pair:info_m)
        {
            for(auto line_list : pair.second){
                for(auto line : line_list){
                    auto llog = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
                    if(pair.first=="split"){
                        llog->color={227, 227, 39}; //黄色
                    }
                    if(pair.first=="merge"){
                        llog->color={0,255,0};
                    }
                    if(pair.first=="straight"){
                        llog->color={0,255,255};
                    }
    
                    auto log1 = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
                    log1->color= {255,0,0};
                    auto& ele1 = log1->add(line.ps->pos);
                    ele1.label.score=1;
                    ele1.label.label=1;

                    int i = 2;
                    for(auto p:line.points)
                    {
                        auto& ele2 =llog->add(p.pos);
                        ele2.label.score=i++;
                        ele2.label.label=100;
                    }

                    auto log3 = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
                    log3->color= {0,0,255};
                    auto& ele3=log3->add(line.pe->pos);
                    ele3.label.score=300;
                    ele3.label.label=300;
                }
            }

        }
    }
    session->save_debug_info("split_merge_connect_candidates_list");

    session->set_display_name("sm_cs_origin_list");
    for(auto line:sm_cs_origin_list)
    {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
        
        log->color={255,255,255};
        auto llog = session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
        llog->color= log->color;
        auto &ele=llog->add(line.ps->pos);
        for(auto p:line.points)
        {
            llog->add(p.pos);
            ele.label.score=0;
            ele.label.label=100;
        }
        llog->add(line.pe->pos);
    }
    session->save_debug_info("sm_cs_origin_list");



    session->set_display_name("split_merge_point");
    auto log_start_split_merge_pts = session->add_debug_log(utils::DisplayInfo::POINT, "split_merge");
    int cnt =0; 
    for (int i = 0; i < session->sm_matchs.size(); i++) {
        auto& one_sm_match = session->sm_matchs[i];
        auto& sm_fit_lines = one_sm_match->sm_fit_lines;
        
        for (int j = 0; j < sm_fit_lines.center_lines.size(); j++) {
            if (sm_fit_lines.valid_status[j].is_valid == false) {
                continue;
            }
            auto& lc_pnts = sm_fit_lines.center_lines[j];
            auto log_split_merge_pts = session->add_debug_log(utils::DisplayInfo::POINT, "split_merge{}", i);
            log_split_merge_pts->color = {255, 255, 0}; // 黄色
            for (int k = 0; k < lc_pnts.size(); k++) {
                // 打印点
                if(k == 0){
                    auto start_text_log= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", cnt);
                    start_text_log->color = {255, 255, 0}; 
                    start_text_log->add(lc_pnts[k]->pos);

                    auto start_log = session->add_debug_log(utils::DisplayInfo::POINT, "split_merge");
                    start_log->color = {255, 0, 0}; 
                    start_log->add(lc_pnts[k]->pos);
                }else if(k != lc_pnts.size() - 1 && k != 0){
                    auto &ele = log_split_merge_pts->add(lc_pnts[k]->pos);
                    if(j % 2 == 0) {
                        ele.color = {255, 255, 0}; //黄色
                    } else {
                        ele.color = {255, 128, 0}; //橙色
                    }
                }else{
                    auto end_text_log= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", cnt);
                    end_text_log->color = {255, 255, 0}; 
                    end_text_log->add(lc_pnts[k]->pos);

                    auto end_log = session->add_debug_log(utils::DisplayInfo::POINT, "split_merge");
                    end_log->color = {0, 0, 255}; 
                    end_log->add(lc_pnts[k]->pos);
                }
            }
            cnt++;

        }

        // 打印分合流点
        {
            auto log_split_merge_vertial = session->add_debug_log(utils::DisplayInfo::LINE, "split_merge_vertial {}", i);
            log_split_merge_vertial->color = {255, 0, 0}; // 红色
            Eigen::Vector3d center = sm_fit_lines.center;
            Eigen::Vector3d v_road_dir = sm_fit_lines.v_road_dir;
            Eigen::Vector3d s = center + v_road_dir * 7;
            Eigen::Vector3d e = center - v_road_dir * 7;
            log_split_merge_vertial->add(s);
            log_split_merge_vertial->add(e);
        }
    }
    session->save_debug_info("split_merge_point");
    
    //1.打印分合流 初始的识别点
    LOG_INFO("start polt init_sm_cs_indexs_before")
    session->set_display_name("init_sm_cs_indexs_before");
    for(auto info:session->hor_corss_line_m)
    {
        auto it = init_sm_cs_indexs_before.find(info.first);
        if(it == init_sm_cs_indexs_before.end()){
            continue;
        }

        for(auto init_sm : it->second){
            auto log= session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
            log->color={255, 0, 0};
            auto &ele=log->add(info.second->cross_feature_list[init_sm.index]->keypose->pos);
            ele.label.label=1;
        }
     
    }
    session->save_debug_info("init_sm_cs_indexs_before");

    
    LOG_INFO("start polt init_sm_cs_indexs_update")
    session->set_display_name("init_sm_cs_indexs_update");
    for(auto info:session->hor_corss_line_m)
    {
        auto it = init_sm_cs_indexs_update.find(info.first);
        if(it == init_sm_cs_indexs_update.end()){
            continue;
        }

        for(auto init_sm : it->second){
            auto log= session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
            log->color={255, 255, 0};
            auto &ele=log->add(info.second->cross_feature_list[init_sm.index]->keypose->pos);
            ele.label.label=1;
        }
     
    }
    session->save_debug_info("init_sm_cs_indexs_update");

    //2.打印refine后的起始点
    LOG_INFO("start polt prev_search_bd_infos_before")
    session->set_display_name("prev_search_bd_infos_before");
    for(auto info:session->hor_corss_line_m)
    {
        auto it = prev_search_bd_infos_before.find(info.first);
        if(it == prev_search_bd_infos_before.end()){
            continue;
        }

        for(auto bd_info : it->second){
            info.second[bd_info.start_index];
            // auto log= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", index);
            auto log= session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
            log->color={0, 0, 255};
            auto &ele=log->add(info.second->cross_feature_list[bd_info.start_index]->keypose->pos);
            ele.label.label=1;
        }
     
    }
    session->save_debug_info("prev_search_bd_infos_before");

    LOG_INFO("start polt prev_search_bd_infos_update")
    session->set_display_name("prev_search_bd_infos_update");
    for(auto info:session->hor_corss_line_m)
    {
        auto it = prev_search_bd_infos_update.find(info.first);
        if(it == prev_search_bd_infos_update.end()){
            continue;
        }

        for(auto bd_info : it->second){
            info.second[bd_info.start_index];
            // auto log= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", index);
            auto log= session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
            log->color={0, 0, 255};
            auto &ele=log->add(info.second->cross_feature_list[bd_info.start_index]->keypose->pos);
            ele.label.label=1;
        }
     
    }
    session->save_debug_info("prev_search_bd_infos_update");

    LOG_INFO("start polt add_pts")
    session->set_display_name("add_pts");
    for(auto pt:add_pts)
    {
        auto log= session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        log->color={255, 0, 0};
        auto &ele=log->add(pt);
        ele.label.label=1;
     
    }
    session->save_debug_info("add_pts");

    LOG_INFO("start polt road_steady_index")
    session->set_display_name("road_steady_index");
    for(auto info:session->hor_corss_line_m)
    {
        auto it = road_steady_index.find(info.first);
        if(it == road_steady_index.end()){
            continue;
        }

        for(auto road_info : it->second){
            if(road_info.first == -1 || road_info.second == -1){
                continue;
            }

            auto log= session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
            log->color={255, 0, 0};
            log->add(info.second->cross_feature_list[road_info.first]->keypose->pos);
            log->add(info.second->cross_feature_list[road_info.second]->keypose->pos);
        }
     
    }
    session->save_debug_info("road_steady_index");

    LOG_INFO("start polt road_steady_index_update")
    session->set_display_name("road_steady_index_update");
    for(auto info:session->hor_corss_line_m)
    {
        auto it = road_steady_index_update.find(info.first);
        if(it == road_steady_index_update.end()){
            continue;
        }

        for(auto road_info : it->second){
            if(road_info.first == -1 || road_info.second == -1){
                continue;
            }

            auto log= session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
            log->color={255, 0, 0};
            log->add(info.second->cross_feature_list[road_info.first]->keypose->pos);
            log->add(info.second->cross_feature_list[road_info.second]->keypose->pos);
        }
     
    }
    session->save_debug_info("road_steady_index_update");

    #if 0
    LOG_INFO("start polt turn_right_endpoint_m")
    session->set_display_name("turn_right_endpoint_m");
    for(auto& entry:turn_right_endpoint_m)
    {
        for(auto& kp : entry.second){
            auto log= session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
            log->color={255, 0, 0};
            log->add(kp->pos);
        }
    }
    session->save_debug_info("turn_right_endpoint_m");

    LOG_INFO("start polt lane_change_indexs_plt")
    session->set_display_name("lane_change_indexs_plt");
    for(auto info:session->hor_corss_line_m)
    {
        auto it = lane_change_indexs_plt.find(info.first);
        if(it == lane_change_indexs_plt.end()){
            continue;
        }

        for(auto index : it->second){
            info.second[index];
            // auto log= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", index);
            auto log= session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
            log->color={255, 0, 0};
            auto &ele=log->add(info.second->cross_feature_list[index]->keypose->pos);
            ele.label.label=1;
        }
     
    }
    session->save_debug_info("lane_change_indexs_plt");
    #endif

    return fsdmap::SUCC;
}

        
}    
}