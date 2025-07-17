

#include "road_model_proc_identify_break_point.h"

DEFINE_bool(identify_break_point_enable, true, "identify_break_point_enable");
DEFINE_bool(identify_break_point_save_data_enable, true, "identify_break_point_save_data_enable");
DEFINE_double(lane_center_end_point_search_dis, 30, "lane_center_end_point_search_dis"); //6 
DEFINE_double(stopline_avr, 1, "stopline_avr"); 
DEFINE_double(crosswalk_avr, 1, "crosswalk_avr"); 
DEFINE_double(lane_center_avr, 1, "lane_center_avr"); 
DEFINE_double(intersection_avr, 1, "intersection_avr"); 
DEFINE_double(stopline_to_crosswalk_dis, 1, "stopline_to_crosswalk_dis"); 


namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcIdentifyBreakPoint::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_identify_break_point_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
 
    CHECK_FATAL_PROC(indentify_break_point(session), "indentify_break_point");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}


int RoadModelProcIdentifyBreakPoint::indentify_break_point(RoadModelSessionData* session) {
    for (auto& inter : session->intersections){
        for (int lane_side = 0; lane_side < 1; ++lane_side){     //只看进入
        // for (int lane_side = 0; lane_side < 2; ++lane_side){
            //处理进入和退出  ， 0进入 1退出
            const auto &link_data = lane_side == 0 ? inter->in_neighbour : inter->out_neighbour;

            for (auto& entry: link_data){
                RoadSegment* road_segment = entry.first;
                std::vector<KeyPose*> key_poses = entry.second;
    
                if (key_poses.empty()) {
                    continue;
                }
                
                //key_poses 这里是  同一段的id要求一样和长度不能超过50，且是修改后是 框半径+35m范围内， 所以会比roadSegment->pos_sample_list  少了很多点

                // 1. 创建空的对象，取出KeyPose绑定的物体 ,sl停止线，cw人行横道，lc车道线中线段
                //TODO:cxf  改为一对一， 不用vector,  现在由于人行横道形状会块状，用vector以防报错
                std::vector<RoadObjectInfo*> sl_objs, cw_objs; 
    
                for (auto &poss : key_poses) {
                // for (auto &poss : road_segment->pos_sample_list) {
                    session->debug_pos(poss->pos);
                    for (auto &object : poss->object_list){
                        if(object->ele_type == 6) {
                            sl_objs.push_back(object);
                        }
                        if(object->ele_type == 5){
                            cw_objs.push_back(object);
                        }
                    }
    
                }
    
                std::cout << "------------------------------------- " << std::endl;
                std::cout << " sl_objs.size() : " << sl_objs.size() << std::endl;
                std::cout << " cw_objs.size()  : " << cw_objs.size() << std::endl;
    
            
    
                //2. 计算交点位置
                KeyPose* ref_pose = nullptr;
                std::vector<LaneCenterFeature*> lc_pts;
                Eigen::Vector3d lc_pts_avg;
                double lc_pts_sd = 0.0;
                
                //2.1 车道中心线末端点
                for (int i = key_poses.size() - 1; i >= 0; --i){
                    if(ref_pose == nullptr && key_poses[i]->filled_lane_sample.size() != 0){
                        ref_pose = key_poses[i];
                    }
                    
                    if (ref_pose != nullptr){
                        double dis = alg::calc_dis(key_poses[i]->pos, ref_pose->pos, true);
                        if(dis > FLAGS_lane_center_end_point_search_dis){
                            break;
                        }
                        
                        for(int j  = 0; j < key_poses[i]->filled_lane_sample.size(); j++){
                            auto &lc = key_poses[i]->filled_lane_sample[j];
                            if(lc->fill_next == nullptr && lc->road_index == 0){
                                lc_pts.push_back(lc);
                            }
                            
                        }
                        
                    }
                }

                std::vector<Eigen::Vector3d> lc_pts_eigen;
                for(auto &lc : lc_pts){
                    lc_pts_eigen.push_back(lc->pos);

                    if(lane_side == 0){
                        session->lc_pts_eigen_in.push_back(lc->pos);
                    }else{
                        session->lc_pts_eigen_out.push_back(lc->pos);
                    }
                }

            
                //找到的车道线末端点合成一个
                if (!lc_pts.empty()) {
                    lc_pts_avg = alg::cal_average(lc_pts_eigen);
                    lc_pts_sd = (alg::cal_standard_deviation(lc_pts_eigen, lc_pts_avg)).norm();
                }

                
                //TODO:cxf  1、这里进入车道 带上右转的，需要过滤
                //TODO:cxf  2、末端点似乎是无效的， 目前找到的是倒数第二、三个点，有时候找到倒数第一个，road_model_proc_identify_road有同样的问题
                
                
                Eigen::Vector3d sl_cross_pt, cw_cross_pt, inter_cross_pt;
                //2.2 人行横道交点
                if (cw_objs.size() == 1) {
                    //取出crosswalk的link点
                    KeyPose* crosswalk_keypose = nullptr;
                    for (auto& key_pose : cw_objs[0]->bind_links) {
                        if (key_pose && std::find(key_poses.begin(), key_poses.end(), key_pose) != key_poses.end()) {
                            crosswalk_keypose = key_pose;
                            break;
                        }
                    }
                
                    if (crosswalk_keypose != nullptr) {
                        //计算交点
                        std::vector<Eigen::Vector3d> cw_cross_pts;
                        for (int i = 0; i < cw_objs[0]->list.size() - 1; i++) {
                            Eigen::Vector3d from1 = cw_objs[0]->list[i]->pos;
                            Eigen::Vector3d to1 = cw_objs[0]->list[i + 1]->pos;
                            if (crosswalk_keypose->next != nullptr) {
                                Eigen::Vector3d ret;
                                bool res = alg::get_cross_point_by_point(from1, to1, crosswalk_keypose->pos, crosswalk_keypose->next->pos, ret, true, 1);
                                if (res) {
                                    cw_cross_pts.push_back(ret);
                                }
                            }
                        }
                
                        double max_dis = 0;
                        for (auto& point : cw_cross_pts) {
                            double dis = alg::calc_dis(point, inter->pos);
                            if (dis > max_dis) {
                                max_dis = dis;
                                cw_cross_pt = point;
                            }
                        }
                        
                        if(lane_side == 0){
                            session->cw_cross_pt_in.push_back(cw_cross_pt);
                        }else{
                            session->cw_cross_pt_out.push_back(cw_cross_pt);
                        }

                        std::cout << " cw_cross_pt: " << cw_cross_pt.transpose() << std::endl;
                    } else {
                        LOG_WARN("No valid crosswalk keypose found");
                    }
                } else if (cw_objs.size() > 1) {
                    LOG_WARN("link keypose bind crosswalk num :[{}]", cw_objs.size());
                }
                

                //2.3 停止线交点
                if (sl_objs.size() == 1) {
                    KeyPose* stopline_keypose = sl_objs[0]->bind_links.empty() ? nullptr : sl_objs[0]->bind_links[0];
                    if (stopline_keypose != nullptr && stopline_keypose->next != nullptr) {
                        Eigen::Vector3d pt2;
                        bool res = alg::get_cross_point_by_point(sl_objs[0]->list[0]->pos, sl_objs[0]->list[1]->pos, 
                                stopline_keypose->pos, stopline_keypose->next->pos, pt2, true, 1);
                        if (res) {
                            sl_cross_pt = pt2;
                        }
                    }

                    if(lane_side == 0){
                        session->sl_cross_pt_in.push_back(sl_cross_pt);
                    }else{
                        session->sl_cross_pt_out.push_back(sl_cross_pt);
                    }

                    std::cout << " sl_cross_pt : " << sl_cross_pt.transpose() << std::endl;   
                } else if (sl_objs.size() > 1) {
                    LOG_WARN("link keypose bind stopline num :[{}]", sl_objs.size());
                }

                //2.4路口交点
                std::vector<Eigen::Vector3d> inter_cross_pts;
                for(int i = 0; i< inter->point_info.size() - 1; i++){
                    Eigen::Vector3d from1 = inter->point_info[i]->pos;
                    Eigen::Vector3d to1 = inter->point_info[i + 1]->pos;
                    if(key_poses[0] != nullptr && key_poses[key_poses.size()-1]!= nullptr){
                        Eigen::Vector3d ret;
                        bool res = alg::get_cross_point_by_point(from1, to1, key_poses[0]->pos, key_poses[key_poses.size()-1]->pos, ret, true, 1);
                        if(res){
                            inter_cross_pts.push_back(ret);
          
                        }

                    }
                }

                std::cout << " inter_cross_pts.size() : " << inter_cross_pts.size() << std::endl;

                //dir方向拿到最近的交点
                if(inter_cross_pts.size() != 0){
                    Eigen::Vector3d dir;
                    if(lane_side ==0){
                        dir = key_poses[0]->dir;
                        inter_cross_pt = alg::project_points_to_dir(key_poses[0]->pos, dir, inter_cross_pts, 1, true);
                    }else{
                        dir = -key_poses[key_poses.size()-1]->dir;
                        inter_cross_pt = alg::project_points_to_dir(key_poses[key_poses.size()-1]->pos, dir, inter_cross_pts, 1, true);
                    }


                    if(lane_side == 0){
                        session->inter_cross_pt_in.push_back(inter_cross_pt);
                    }else{
                        session->inter_cross_pt_out.push_back(inter_cross_pt);
                    }      

                    std::cout << " inter_cross_pt : " << inter_cross_pt.transpose() << std::endl; 
                }


                
                //3. 计算位置,   权重优先级：1停止线，2车道线末端点 ,3人行横道 , 4路口边缘
                //3.1 基准点和方向
                std::shared_ptr<KeyPose> point0 = std::make_shared<KeyPose>();
                point0->pos = cal_weight_avg(sl_cross_pt, lc_pts, cw_cross_pt, inter_cross_pt);

                std::vector<LaneLineSample*> search_sec; 
                session->lane_line_sample_tree.search(point0->pos, 2, search_sec);  // TODO:cxf, 方案待修改这里可能会存在Bug, 搜到对向车道去？ 
                
                int count = 0;
                Eigen::Vector3d dir_avg(0, 0, 0);
                for(auto &lc : search_sec){
                    dir_avg += lc->dir;
                    count++;
                }
                if (count > 0) {
                point0->dir = dir_avg / count;
                }

                if(lane_side == 0){
                    session->point0_in.push_back(point0->pos);
                }else{
                    session->point0_out.push_back(point0->pos);
                }
                std::cout << " point0 : " << point0->pos.transpose() << std::endl;
                std::cout << " point0 dir : " << point0->dir.transpose() << std::endl;



               
                // //3.2 投影
                // double sl_cross_pt_pr, lc_pts_avg_pr, cw_cross_pt_pr, inter_cross_pt_pr, point0_pr, break_pt_pr;
                double sl_cross_pt_pr = std::numeric_limits<double>::quiet_NaN();
                double lc_pts_avg_pr = std::numeric_limits<double>::quiet_NaN();
                double cw_cross_pt_pr = std::numeric_limits<double>::quiet_NaN();
                double inter_cross_pt_pr = std::numeric_limits<double>::quiet_NaN();
                double point0_pr = std::numeric_limits<double>::quiet_NaN();
                double break_pt_pr = std::numeric_limits<double>::quiet_NaN();

                if(!sl_cross_pt.isZero()){
                    sl_cross_pt_pr = alg::project_point_to_line(point0->pos, point0->dir, sl_cross_pt, true);
                }
                if(!lc_pts_avg.isZero()){
                    lc_pts_avg_pr = alg::project_point_to_line(point0->pos, point0->dir, lc_pts_avg, true);
                }
                if(!cw_cross_pt.isZero()){
                    cw_cross_pt_pr = alg::project_point_to_line(point0->pos, point0->dir, cw_cross_pt, true);
                }
                if(!inter_cross_pt.isZero()){
                    inter_cross_pt_pr = alg::project_point_to_line(point0->pos, point0->dir, inter_cross_pt, true);
                }
                if(!point0->pos.isZero()){
                    point0_pr = alg::project_point_to_line(point0->pos, point0->dir, point0->pos, true);
                }

                //高斯计算打断点     
                Eigen::Vector3d break_pt;
                break_pt_pr = cal_break_point(point0_pr, sl_cross_pt_pr, lc_pts_avg_pr, lc_pts_sd, cw_cross_pt_pr, inter_cross_pt_pr);

                //还原真实点位置
                break_pt = alg::get_hori_pos(point0->pos, point0->dir, break_pt_pr, true);
                if(lane_side == 0){
                    session->break_pt_in.push_back(break_pt);
                }else{
                    session->break_pt_out.push_back(break_pt);
                }

                std::cout << " break_pt_pr : " << break_pt_pr << std::endl;
                std::cout << " break_pt : " << break_pt.transpose() << std::endl;
            
                //5 .设置最近link的状态和方向
                std::shared_ptr<BreakPointKeyPose> break_keypose = std::make_shared<BreakPointKeyPose>();
                int index = 0;
                double dis = std::numeric_limits<double>::max();
                for(int i = 0; i < key_poses.size(); i++){
                    double dis_tmp = alg::calc_dis(key_poses[i]->pos, break_pt, true);
                    if(dis_tmp < dis){
                        dis = dis_tmp;
                        index = i;
                    }
                }
                break_keypose->key_pose  = key_poses[index];  
                break_keypose->break_status = lane_side; //0进入，1退出
                session->break_point_key_pose_list.push_back(break_keypose);


            }
    
        }
 
    }

    return fsdmap::SUCC;
}


Eigen::Vector3d RoadModelProcIdentifyBreakPoint::cal_weight_avg(
    const Eigen::Vector3d& sl_cross_pt, 
    const std::vector<LaneCenterFeature*>& lc_pts, 
    const Eigen::Vector3d& cw_cross_pt, 
    const Eigen::Vector3d& inter_cross_pt) {
    
    Eigen::Vector3d weighted_point(0, 0, 0);

    double stopline_weight = 0.6;
    double lc_weight = 0.2;
    double crosswalk_weight = 0.1;
    double inter_weight = 0.1;

    if (!sl_cross_pt.isZero()) {
        weighted_point += stopline_weight * sl_cross_pt;
    }

    if (!lc_pts.empty()) {
        Eigen::Vector3d lc_avg_point(0, 0, 0);
        for (const auto& lc : lc_pts) {
            lc_avg_point += lc->pos;
        }
        lc_avg_point /= lc_pts.size();
        weighted_point += lc_weight * lc_avg_point;
    }

    if (!cw_cross_pt.isZero()) {
        weighted_point += crosswalk_weight * cw_cross_pt;
    }

    if (!inter_cross_pt.isZero()) {
        weighted_point += inter_weight * inter_cross_pt;
    }

    return weighted_point;
}

double RoadModelProcIdentifyBreakPoint::cal_break_point(double point0_pr, double sl_cross_pt_pr, 
    double lc_pts_avg_pr, double lc_pts_avr,double cw_cross_pt_pr, double inter_cross_pt_pr) {

    double mu_merged = 0.0; 
    double sigma_merged = 1.0;

    std::vector<std::pair<double, double>> points = {
        {sl_cross_pt_pr - point0_pr, FLAGS_stopline_avr},
        {lc_pts_avg_pr - point0_pr, FLAGS_lane_center_avr},
        {cw_cross_pt_pr - point0_pr - FLAGS_stopline_to_crosswalk_dis, FLAGS_crosswalk_avr},
        {inter_cross_pt_pr - point0_pr, FLAGS_intersection_avr}
    };

    for(const auto& point : points){
        if(!std::isnan(point.first)){
            auto result = gaussian_fusion_multiply(mu_merged, sigma_merged, point.first, point.second);
            mu_merged = result.first;
            sigma_merged = result.second;

        }
    }

    return mu_merged;
}


std::pair<double, double> RoadModelProcIdentifyBreakPoint::gaussian_fusion_multiply(const double& mu1, const double& sigma1, const double& mu2, const double& sigma2) {
    double var1 = sigma1 * sigma1;
    double var2 = sigma2 * sigma2;

    double mu_merged = (var2 * mu1 + var1 * mu2) / (var1 + var2);
    double var_merged = var1 * var2 / (var1 + var2);
    double sigma_merged = std::sqrt(var_merged);

    return std::make_pair(mu_merged, sigma_merged);
}



int RoadModelProcIdentifyBreakPoint::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_identify_break_point_save_data_enable) {
        return fsdmap::SUCC;
    }

    session->set_display_name("break_point");
    //车道线
    for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
        auto &group_list = session->lane_center_line_group_list_raw[i];
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            session->debug_pos(pt->pos);

            double length = pt->context.next_max_length + pt->context.prev_max_length;
            for (int k = 0; k < pt->context.all_next.size(); ++k) {
                auto &next_ptr = pt->context.all_next[k];
                auto &next = next_ptr.src;
                auto tlog = session->add_debug_log(utils::DisplayInfo::LINE, "break_point");
                tlog->color = {255, 255, 0};
                if (next_ptr.status == 2) {
                    tlog->color = {0, 255, 0};
                }
                if (pt->invalid() || next->invalid() 
                        || next_ptr.invalid()) {
                    tlog->color = {255, 0, 0};
                }
                auto score = next_ptr.score;
                auto &ele1 = tlog->add(pt->pos, 1, next_ptr.status);
                ele1.label.score = score;
                ele1.label.cloud_bev_label_score = pt->score;
                ele1.label.opt_label = pt->filter_status;
                ele1.label.intensity = pt->context.next_max_length;
                ele1.label.intensity_opt = pt->context.prev_max_length;
                auto &ele2 = tlog->add(next->pos, 1, next_ptr.status);
                ele2.label.score = score;
                ele2.label.cloud_bev_label_score = next->score;
                ele2.label.opt_label = pt->filter_status;
                ele2.label.intensity = pt->context.next_max_length;
                ele2.label.intensity_opt = pt->context.prev_max_length;
            }
            for (int k = 0; k < pt->context.invalid_next.size(); ++k) {
                auto &next_ptr = pt->context.invalid_next[k];
                auto &next = next_ptr.src;

                auto tlog = session->add_debug_log(utils::DisplayInfo::LINE, "break_point");
                tlog->color = {255, 0, 0};
                if (next_ptr.status == 2) {
                    tlog->color = {0, 255, 0};
                }
                auto score = next_ptr.score;
                auto &ele1 = tlog->add(pt->pos, 1, next_ptr.status);
                ele1.label.score = score;
                ele1.label.cloud_bev_label_score = pt->score;
                ele1.label.opt_label = pt->filter_status;
                ele1.label.intensity = pt->context.next_max_length;
                ele1.label.intensity_opt = pt->context.prev_max_length;
                auto &ele2 = tlog->add(next->pos, 1, next_ptr.status);
                ele2.label.score = score;
                ele2.label.cloud_bev_label_score = next->score;
                ele2.label.opt_label = pt->filter_status;
                ele2.label.intensity = pt->context.next_max_length;
                ele2.label.intensity_opt = pt->context.prev_max_length;
            }
            if (pt->context.all_next.size() == 0) {
                auto tlog = session->add_debug_log(utils::DisplayInfo::POINT, "break_point");
                tlog->color = {100, 100, 100};
                auto &ele = tlog->add(pt->pos);
                ele.label.opt_label = pt->filter_status;
                ele.label.intensity_opt = length;
            }
            if (pt->context.all_prev.size() == 0) {
                auto tlog = session->add_debug_log(utils::DisplayInfo::POINT, "break_point");
                tlog->color = {0, 100, 100};
                auto &ele = tlog->add(pt->pos);
                ele.label.opt_label = pt->filter_status;
                ele.label.intensity_opt = length;
            }
        }
    }



    //调试可视化
    for (auto& inter : session->intersections){
        for (int lane_side = 0; lane_side < 2; ++lane_side){    
            const auto &link_data = lane_side == 0 ? inter->in_neighbour : inter->out_neighbour;
            for (auto& entry: link_data){
                RoadSegment* road_segment = entry.first;
                std::vector<KeyPose*> key_poses = entry.second;

                if (key_poses.empty()) {
                    continue;
                }
                
                //进入退出link
                auto key_log = session->add_debug_log(utils::DisplayInfo::POINT, "link_pose");
                if(lane_side == 0){
                    key_log->color = {0, 255, 0}; //绿色
                }else{
                    key_log->color = {255, 192, 203}; //粉红色
                }
                for (int i = key_poses.size() - 1; i >= 0; --i){
                    key_log->add(key_poses[i]->pos);

                    // auto log3 = session->add_debug_log(utils::DisplayInfo::POINT, "end_point");
                    // log3->color = {0, 0, 255};
                    // for(int j  = 0; j < key_poses[i]->lane_center_list.size(); j++){
                    //     auto &lc = key_poses[i]->lane_center_list[j];
                    //     if(lc->road_index == 0){
                    //         log3->add(lc->pos);
                    //     }
                    // }
                }

                


                // //车道线末端点
                // KeyPose* ref_pose = nullptr;
                // std::vector<LaneCenterFeature*> lc_pts;
                // for (int i = key_poses.size() - 1; i >= 0; --i){
                    
                //     auto log = session->add_debug_log(utils::DisplayInfo::POINT, "lane_point");
                //     log->color = {255, 255, 0};
        
                //     if(ref_pose == nullptr && key_poses[i]->filled_lane_sample.size() != 0){
                //         ref_pose = key_poses[i];
                //     }

                //     if (ref_pose != nullptr){
                //         double dis = alg::calc_dis(key_poses[i]->pos, ref_pose->pos, true);
                //         if(dis > FLAGS_lane_center_end_point_search_dis){
                //             break;
                //         }

                //         for(int j  = 0; j < key_poses[i]->filled_lane_sample.size(); j++){
                //             auto &lc = key_poses[i]->filled_lane_sample[j];
                //             if(lc->fill_next == nullptr && lc->road_index == 0){
                //                 log->add(lc->pos);
                //                 lc_pts.push_back(lc);
                //             }
                            
                //         }

                //     }
                // }

                // auto log2 = session->add_debug_log(utils::DisplayInfo::POINT, "end_point");
                // log2->color = {0, 0, 255};
                // for(auto &lc : lc_pts){
                //     // std::cout << " lc->next: " << lc->next << std::endl;
                //     // std::cout << " lc->prev: " << lc->prev << std::endl;
                //     // std::cout << " lc->fill_next: " << lc->fill_next << std::endl;
                //     while(lc->next != nullptr){
                //         lc = lc->next;
                //         log2->add(lc->pos);
                //     }
                // }

   
            }
        }
    }
    //人行横道
    for (auto& object : session->raw_object_ret_list) {
        if(object->ele_type == 6){
            auto log_sl_all = session->add_debug_log(utils::DisplayInfo::LINE, "stopline");
            log_sl_all->color = {255, 0, 255};
            for(auto& pt : object->list){
                log_sl_all->add(pt->pos, 10);
            }
        }
    }

    //整个人行横道
    for (auto& object : session->raw_object_ret_list) {
        if(object->ele_type == 5){
            auto log_cw_all = session->add_debug_log(utils::DisplayInfo::LINE, "cross_walk");
            log_cw_all->color = {207, 145, 145};
            for(auto& pt : object->list){
                log_cw_all->add(pt->pos, 10);
            }
        }
    }

    //路口
    for (auto &inter : session->raw_intersections){
        auto log_inter_all = session->add_debug_log(utils::DisplayInfo::LINE, "inter");
        log_inter_all->color = {255, 0, 0}; // 红色
        for (auto &pt : inter->lukou_poly_pts){
            log_inter_all->add(pt);
        }
    }


    //2.中间过程
    //停止线
    auto log_sl = session->add_debug_log(utils::DisplayInfo::POINT, "sl_cross_pt");
    log_sl->color = {230, 127, 48}; //橙色
    for (int i = session->sl_cross_pt_in.size() - 1; i >= 0; --i){
        log_sl->add(session->sl_cross_pt_in[i]);
    }

    //车道线末端点
    auto log_lc_in = session->add_debug_log(utils::DisplayInfo::POINT, "lc_cross_pt");
    log_lc_in->color = {255, 255, 0}; //黄色
    for (int i = session->lc_pts_eigen_in.size() - 1; i >= 0; --i){
        log_lc_in->add(session->lc_pts_eigen_in[i]);
    }
    auto log_lc_out = session->add_debug_log(utils::DisplayInfo::POINT, "lc_cross_pt");
    log_lc_out->color = {116, 122, 1}; //深黄色
    for (int i = session->lc_pts_eigen_out.size() - 1; i >= 0; --i){
        log_lc_out->add(session->lc_pts_eigen_out[i]);
    }



    //人行横道
    auto log_cw_in = session->add_debug_log(utils::DisplayInfo::POINT, "cw_cross_pt");
    log_cw_in->color = {161, 245, 5}; //浅绿色
    for (int i = session->cw_cross_pt_in.size() - 1; i >= 0; --i){
        log_cw_in->add(session->cw_cross_pt_in[i]);
    }
    auto log_cw_out = session->add_debug_log(utils::DisplayInfo::POINT, "cw_cross_pt");
    log_cw_out->color = {72, 110, 2}; //深绿色
    for (int i = session->cw_cross_pt_in.size() - 1; i >= 0; --i){
        log_cw_out->add(session->cw_cross_pt_in[i]);
    }
    
    //路口
    auto log_inter_in = session->add_debug_log(utils::DisplayInfo::POINT, "inter_cross_pt_in");
    log_inter_in->color = {158, 3, 255}; //浅紫色
    for (int i = session->inter_cross_pt_in.size() - 1; i >= 0; --i){
        log_inter_in->add(session->inter_cross_pt_in[i]);
    }

    auto log_inter_out = session->add_debug_log(utils::DisplayInfo::POINT, "inter_cross_pt_out");
    log_inter_out->color = {68, 2, 110}; //深紫色
    for (int i = session->inter_cross_pt_out.size() - 1; i >= 0; --i){
        log_inter_out->add(session->inter_cross_pt_out[i]);
    }


    //基准点
    auto log_point0_in = session->add_debug_log(utils::DisplayInfo::POINT, "point0_in");
    log_point0_in->color = {255, 8, 156}; //浅粉色
    for (int i = session->point0_in.size() - 1; i >= 0; --i){
        log_point0_in->add(session->point0_in[i]);
    }

    auto log_point0_out = session->add_debug_log(utils::DisplayInfo::POINT, "point0_out");
    log_point0_out->color = {128, 1, 77}; //深粉色
    for (int i = session->point0_out.size() - 1; i >= 0; --i){
        log_point0_out->add(session->point0_out[i]);
    }
    

    //打断点
    auto log_break_pt_in = session->add_debug_log(utils::DisplayInfo::POINT, "break_pt_in");
    log_break_pt_in->color = {255, 0, 0}; //浅红
    for (int i = session->break_pt_in.size() - 1; i >= 0; --i){
        log_break_pt_in->add(session->break_pt_in[i]);
    }

    // auto log_break_pt_out = session->add_debug_log(utils::DisplayInfo::POINT, "break_pt_out");
    // log_break_pt_out->color = {237, 88, 93}; //浅浅红
    // for (int i = session->break_pt_out.size() - 1; i >= 0; --i){
    //     log_break_pt_out->add(session->break_pt_out[i]);
    // }









    session->save_debug_info("break_point");

    return fsdmap::SUCC;

    
}


}
}
