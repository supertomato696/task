#include "road_model_proc_cal_coverage.h"
#include <algorithm> // 包含 std::all_of 和 std::equal

DEFINE_bool(cal_coverage_enable, true, "cal_coverage_enable");
DEFINE_bool(cal_coverage_save_data_enable, true, "cal_coverage_save_data_enable");
DEFINE_double(standard_lane_width2, 3.5, "standard_lane_width2"); 
DEFINE_double(lane_width_scale, 1/2, "lane_width_scale"); 
DEFINE_double(search_num, 7, "search_num"); 
DEFINE_double(duplicate_pts_dis, 0.15, "duplicate_pts_dis"); 


namespace fsdmap {
    namespace road_model {
        
        fsdmap::process_frame::PROC_STATUS RoadModelProcCalCoverage::proc(
            RoadModelSessionData* session) {
                if (!FLAGS_cal_coverage_enable) {
                    return fsdmap::process_frame::PROC_STATUS_DISABLE;
                }
                
                CHECK_FATAL_PROC(cal_coverage(session), "cal_coverage");
                CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
                return fsdmap::process_frame::PROC_STATUS_SUCC;
            }
int RoadModelProcCalCoverage::cal_coverage(RoadModelSessionData* session) {
    if (!FLAGS_cal_coverage_enable) {
        return fsdmap::SUCC;
    }

    session->set_display_name("cal_coverage");
    
    //===================总流程开始 ===========================

    std::vector<KeyPose*> in_link_pts, out_link_pts; //4个 交点
    std::vector<Eigen::Vector3d> left_bd_pts, right_bd_pts;
    // std::vector<Eigen::Vector3d> in_road_center_points_all;  //交到的在道路内的实际车道中心线的 点
    std::map<KeyPose*, RoadObjectInfo*> stop_line_list_m; //路口内的4条进入停止线
    

    // 1. 找到进入退出的起始link->form=16的点
    // TODO:cxf 去掉 离路口4个点很远的  掉头的点  ，减少find_out_start_keypose的计算耗时；
    find_cross_link_points(session, in_link_pts, out_link_pts);
    
    // 2. 找到真正的起始点
    //   进入点：靠近停止线的keypose
    //   退出点：停止线交到的对向link
    for (auto& in_pt : in_link_pts) {
        KeyPose* in_start_keypose = find_in_start_keypose(session, in_pt, stop_line_list_m);
        KeyPose* out_start_keypose = find_out_start_keypose(session, in_pt, in_link_pts, stop_line_list_m);
        if (in_start_keypose) {
            session->key_pose_info_m[in_start_keypose] = {};
        }
        if (out_start_keypose) {
            session->key_pose_info_m[out_start_keypose] = {};
        }
    }
    std::cout << "session->key_pose_info_m: " << session->key_pose_info_m.size() << std::endl;

    //3.=================第一次遍历起始点，生成表=======================
    for(auto& info : session->key_pose_info_m){
        int flag = info.first->in_out_status;
        KeyPose* origin_start = info.first;
        KeyPose* start_keypose = info.first;

        std::string status = flag == 1 ? "enter_status" : "exit_status" ;
        std::cout << "status : " << status << std::endl;

        //  初始化起点车道数
        double W_lane_st = FLAGS_standard_lane_width2;
        
        // 4. 往路口内遍历每个KeyPose对象，
        bool first_iteration = true;
        while (start_keypose) {
            //初始化key_pose_info对象
            std::shared_ptr<RoadCrossPoints> road_cross_pts = std::make_shared<RoadCrossPoints>();
            road_cross_pts->keypose = start_keypose;


            //初始化过程变量
            int N_hope = 0;
            int N_link = start_keypose->from_raw_link->lanenum_sum;
            Eigen::Vector3d left_bf;  //左边界
            Eigen::Vector3d right_bf;  //右边界
            Eigen::Vector3d road_dir;  //修正link方向
            double W_road;   //道路宽度
            double W_lane_t;  //真实车道宽度


            //5. 计算修正真实的道路方向
            road_dir = cal_road_true_dir(session, start_keypose);
            road_cross_pts->road_dir = road_dir;

            //6.计算宽度
            bool width_flag = cal_road_width(session, start_keypose, road_dir, left_bf, right_bf ,W_road);
            if (width_flag == false) {
                road_cross_pts->is_deal = false;
                session->key_pose_info_m[origin_start].push_back(road_cross_pts);
                start_keypose = (flag == 1) ? start_keypose->prev : start_keypose->next;
                continue;
            }
            
            auto left = std::make_shared<CrossPoint>(false, left_bf);
            auto right = std::make_shared<CrossPoint>(false, right_bf);
            road_cross_pts->road_boundary_pts.push_back(left);
            road_cross_pts->road_boundary_pts.push_back(right);

    
            
            //7. 计算实际的车道中心线数， 实际平均宽度
            double search_radius = W_road;
            std::vector<Eigen::Vector3d> in_road_center_points;
            int N_lane_center = cal_actual_lane_center_count(session, start_keypose ,road_dir, search_radius, left_bf, right_bf, in_road_center_points);
            for(auto& pt : in_road_center_points){
                auto cross_pt = std::make_shared<CrossPoint>(false, pt);
                road_cross_pts->lane_center_pts.push_back(cross_pt);
            }
            
            //8. 计算交到的车道线形成的数量   N_lane = in_road_lane_points.size() -1
            std::vector<Eigen::Vector3d> in_road_lane_points;
            int N_lane = cal_actual_lane_count(session, start_keypose ,road_dir, search_radius, left_bf, right_bf, in_road_lane_points);
            for(auto& pt : in_road_lane_points){
                auto cross_pt = std::make_shared<CrossPoint>(false, pt);
                road_cross_pts->lane_pts.push_back(cross_pt);
            }
            
            //9. 计算通过道路宽度计算的车道数N_road， 和期望的车道数N_hope
            double N_road = W_road / W_lane_st;
            road_cross_pts->N_road = static_cast<int>(std::round(N_road));
            road_cross_pts->N_lane = N_lane;
            road_cross_pts->N_lane_center = N_lane_center;
            road_cross_pts->N_link = N_link;
            // if (N_road == N_lane && N_lane >= 1){
            //     std::cout << " N_road: " << N_road<< " N_lane: " << N_lane<< " N_lane_center: " << N_lane_center<< " N_link: " << N_link <<  std::endl;
            // }
            
            //10. 车道线存在， 则补一次点
            recovery_single_road_cross_point(road_cross_pts, first_iteration);
            first_iteration = false;
            N_hope = road_cross_pts->N_hope;

            // std::cout << "N_link : " << N_link << "   N_road : " << N_road << " N_hope : " << N_hope << std::endl;

            // std::cout << "N_diff : " << N_diff << "   N_lane_center : " << N_lane_center << std::endl;
            // std::cout << "----------------------"<< std::endl;

            //赋值
            W_lane_t = W_road/N_hope;
            road_cross_pts->W_lane_t = W_lane_t;
            road_cross_pts->W_lane_st = W_lane_st;
            road_cross_pts->is_deal = true;


            //11. 计算缺失的位置
            // if(N_diff > 0){
            //         //横向方向的缺失点
            //         cal_horizontal_miss_positions2(session, road_cross_pts,origin_start,start_keypose, road_dir, N_hope, N_lane_center, W_road, W_lane_t,
            //             left_bf, right_bf, in_road_center_points);
            //         // cal_triangle_island_miss_positions(start_keypose, N_hope, N_lane_center);
            // }

            session->key_pose_info_m[origin_start].push_back(road_cross_pts);
            start_keypose = (flag == 1) ? start_keypose->prev : start_keypose->next;
        }
        
    }

    
    //=================  进行第二次遍历，处理存在车道缺失的keypose=================
    #if 1
    for(auto& info : session->key_pose_info_m){
        int flag = info.first->in_out_status;
        KeyPose* origin_start = info.first;
        
        //1. 遍历cur
        // TODO 考虑存在环， 防止死循环
        auto cur_it = info.second.begin();
        while(cur_it != info.second.end()){

            //已经满的， 则跳过不处理，处理下一个
            if ((*cur_it)->is_full) {
                cur_it++;
                continue;
            }

            KeyPose* cur_keypose = (*cur_it)->keypose;
            int N_diff_cur = (*cur_it)->N_diff;
            int Nt_cur = (*cur_it)->N_lane_center;
            int N_hope_cur = (*cur_it)->N_hope;
            
   
            // 2. 找ref1 
            //只相信前后20m，  最远是前找7个keypose 
            // 往前找到N_hope_ref1 = N_hope_cur 且N_diff_ref1 = 0的点
            KeyPose* ref1_keypose = cur_keypose;
            auto ref1_it = cur_it;
            
            int count1 =0; 
            bool found_ref1 =false;
            while(ref1_keypose && count1 < FLAGS_search_num){
                ref1_keypose = (flag == 1) ? ref1_keypose->next : ref1_keypose->prev;

                if((*ref1_it)->keypose == origin_start) break; //回到起点了
                
                ref1_it--;
                if(!(*ref1_it)->is_deal) continue;

                // int N_diff_ref1 = (*ref1_it)->N_diff;
                // int N_hope_ref1 = (*ref1_it)->N_hope;
                //找到ref1
                // if(N_hope_ref1 == N_hope_cur && N_diff_ref1 == 0){
                if((*ref1_it)->is_full == true){
                    found_ref1 = true;
                    break;
                }
                count1++;
            }
            
            //没找到ref1，跳过后续所有步骤，  TODO： ref1找不到，用横向推的方法
            if(!found_ref1){
                cur_it++;
                continue;
            }
            // std::cout<< "1111111111111111111111"<< std::endl;

            //3. 找ref2
            // 往后找到N_hope_ref2>= N_hope_cur && Nt_ref2 >= Nt_cur的点
            KeyPose* ref2_keypose = cur_keypose;
            auto ref2_it = cur_it;
            int count2 =0;
            bool found_ref2 =false;
            while(ref2_keypose && count2 < FLAGS_search_num){
                ref2_keypose = (flag == 1) ? ref2_keypose->prev : ref2_keypose->next;
                ref2_it++;

                if(ref2_it == info.second.end()) break; // 如果已经遍历完还没找到，退出
                if(!(*ref2_it)->is_deal) continue; //未处理过则跳过

                // int N_hope_ref2 = (*ref2_it)->N_hope;
                // int Nt_ref2 = (*ref2_it)->N_lane_center;
                
                //找到ref2
                // if(N_hope_ref2 >= N_hope_cur && Nt_ref2 >= Nt_cur && Nt_ref2 >=N_hope_cur){
                if((*ref2_it)->is_full == true){
                    // std::cout << "N_hope_ref2: " << N_hope_ref2 <<  "N_hope_cur: " << N_hope_cur << std::endl;
                    // std::cout << "Nt_ref2: " << Nt_ref2 << "Nt_cur: " << Nt_cur << std::endl;
                    found_ref2 = true;
                    break;
                }
                count2++;
            }

            //找不到ref2跳过后面所有步骤
            if(!found_ref2){
                cur_it++;
                continue;
            }

            // std::cout<< "222222222222222"<< std::endl;
            
            
            // 4.在ref1和ref2之间寻找有缺失的keypose补点
            auto start_searching_it = ref1_it;
            start_searching_it++;
            int count3 =0;
            while(start_searching_it != ref2_it){
                KeyPose* start_searching = (*start_searching_it)->keypose;
                
                //这个中间的keypose 车道数点全的，跳过不需要补点
                if((*start_searching_it)->is_full ==true ){
                    start_searching_it++;
                    continue; 
                }
                // std::cout<< "777777777"<< std::endl;

                // 5. 连线补点逻辑
                double scale = FLAGS_lane_width_scale;
                // std::cout << "N_hope_cur: " << N_hope_cur <<"(*cur_it)->N_lane_center: " << (*cur_it)->N_lane_center <<std::endl;
                // std::cout << "(*ref1_it)->lane_center_pts.size():  " << (*ref1_it)->lane_center_pts.size() <<std::endl;
                // std::cout << "(*ref2_it)->lane_center_pts.size():  " << (*ref2_it)->lane_center_pts.size() <<std::endl;
                // std::cout << "(*ref1_it)->N_hope: " << (*ref1_it)->N_hope << "(*ref1_it)->N_lane_center: " << (*ref1_it)->N_lane_center <<std::endl;
                // std::cout << "(*ref2_it)->N_hope: " << (*ref2_it)->N_hope << "(*ref2_it)->N_lane_center: " << (*ref2_it)->N_lane_center <<std::endl;
                
                int N_hope = std::min((*ref1_it)->N_hope, (*ref2_it)->N_hope); // 取最小的
                // if((*cur_it)->is_deal == true){
                //     N_hope = (*cur_it)->N_hope;
                // }else{
                //     N_hope = std::min((*ref1_it)->N_hope, (*ref2_it)->N_hope);
                // }
                
                for(int i=0; i< N_hope; i++){
                    Eigen::Vector3d ret;
                    Eigen::Vector3d pt1 = (*ref1_it)->lane_center_pts[i]->point;
                    Eigen::Vector3d pt2 = (*ref2_it)->lane_center_pts[i]->point;
                    Eigen::Vector3d v_dir = alg::get_vertical_dir((*start_searching_it)->road_dir);
                    bool res = alg::get_cross_point_by_dir(start_searching->pos, v_dir,
                        pt1, pt2, ret, 1, true);
                    
                    if(res){
                        if((*start_searching_it)->is_deal == true){
                            //情况1: 原来keypose处理过，但有缺失的; 则判断算出来的点,是否原来就存在
                            bool found = false;
                            double W_lane_t = (*start_searching_it)->W_lane_t;
                            for(int j = 0; j < (*start_searching_it)->N_lane_center; ++j) {
                                Eigen::Vector3d pt = (*start_searching_it)->lane_center_pts[j]->point;
                                if(alg::calc_dis(ret, pt) < (W_lane_t * scale )) {
                                    found = true;
                                    break;
                                }
                            }
                            if(!found) {
                                auto cross_pt = std::make_shared<CrossPoint>(true, ret); // 设置为处理过的点
                                
                                (*start_searching_it)->lane_center_pts.push_back(cross_pt);
                                (*start_searching_it)->N_lane_center++;
                                (*start_searching_it)->N_diff--;
                            }
                            // std::cout<< "4444444444444444444444444"<< std::endl;
                        }else{
                            //情况2: 原来keypose 就未处理过，则算出来的点全部加入
                            // TODO：如果不满足最小的宽度，补空点
                            auto cross_pt = std::make_shared<CrossPoint>(true, ret); 
                            (*start_searching_it)->lane_center_pts.push_back(cross_pt);
                            // std::cout<< "33333333333333333"<< std::endl;
                            //是否需要补边界等？
                            
                        }
                    }
                    
                }     
                // std::cout<< "5555555555555555555555"<< std::endl;
                
                ///情况1: 之前处理过，但缺的， 补完点后，重新从左排到右
                if((*start_searching_it)->is_deal == true){
                    // TODO:cxf 优化找第几个的方法，直接插入， 不用重新排,  提升速度
                    std::sort((*start_searching_it)->lane_center_pts.begin(), (*start_searching_it)->lane_center_pts.end(), 
                    [left_bf = (*start_searching_it)->road_boundary_pts[0]->point] (const std::shared_ptr<CrossPoint> &lhs, const std::shared_ptr<CrossPoint> &rhs) -> bool {
                        return alg::calc_dis(lhs->point, left_bf) < alg::calc_dis(rhs->point, left_bf);
                    });
                    
                    //如果满了，则更改状态为full
                    if((*start_searching_it)->N_lane_center == (*start_searching_it)->N_hope){
                        (*start_searching_it)->is_full = true;
                    }

                    // std::cout<< "6666666666666666666"<< std::endl;
                }else{
                    ///情况2:  之前未处理过的，赋值
                    
                    //用N_hope小的 那个 作为未处理的参数
                    auto min_ref_it = (*ref1_it)->N_hope <= (*ref2_it)->N_hope ? ref1_it : ref2_it;
                    (*start_searching_it)->W_lane_st = (*min_ref_it)->W_lane_st;
                    (*start_searching_it)->W_lane_t = (*min_ref_it)->W_lane_t;
                    (*start_searching_it)->N_start = (*min_ref_it)->N_start;
                    (*start_searching_it)->N_hope = (*min_ref_it)->N_hope;
                    (*start_searching_it)->N_lane_center = (*min_ref_it)->N_lane_center;
                    (*start_searching_it)->N_diff = (*min_ref_it)->N_diff;

                    // TODO: 这里确定不了满不满 
                    
                    //算出左右边界点补全
                    Eigen::Vector3d road_dir = (*start_searching_it)->road_dir;
                    Eigen::Vector3d v_dir = alg::get_vertical_dir(road_dir);
                    Eigen::Vector3d left_ref = (*min_ref_it)->road_boundary_pts[0]->point;
                    Eigen::Vector3d right_ref = (*min_ref_it)->road_boundary_pts[1]->point;
                    
                    Eigen::Vector3d left_bf_cur, right_bf_cur;
                    bool res_left, res_right;
                    res_left = alg::get_cross_point(left_ref, road_dir, (*start_searching_it)->keypose->pos, v_dir, left_bf_cur, true);
                    res_right = alg::get_cross_point(right_ref, road_dir, (*start_searching_it)->keypose->pos, v_dir, right_bf_cur, true);
                    
                    if(res_left && res_right){
                        (*start_searching_it)->road_boundary_pts.push_back(std::make_shared<CrossPoint>(true, left_bf_cur));
                        (*start_searching_it)->road_boundary_pts.push_back(std::make_shared<CrossPoint>(true, right_bf_cur));
                    }
                    // std::cout<< "77777777777777777777777777"<< std::endl;
                }
                (*start_searching_it)->is_deal = true;
                // std::cout << "count3: "  << count3 <<std::endl;
                count3++;
                start_searching_it++;
            }
            
            cur_it++;
        }
    }
    #endif



    //=========================总流程结束 ====================================

    //==========================结果可视化====================================
    //link
    for (auto &line : session->link_sample_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "link");
        log->color = {255, 128, 128};
        for (auto &key_pose : line->list) {
            log->add(key_pose->pos, 10);  
        }
    }
    
    //link—16
    std::cout << "in_link_pts.size() : " << in_link_pts.size() << std::endl;
    std::cout << "out_link_pts.size() : " << out_link_pts.size() << std::endl;
    auto log1 = session->add_debug_log(utils::DisplayInfo::POINT, "in");
    log1->color = {0, 255, 0}; 
    for(auto &kp : in_link_pts){
        log1->add(kp->pos);
    }
    auto log2 = session->add_debug_log(utils::DisplayInfo::POINT, "out");
    log2->color = {255, 0, 0}; 
    for(auto &kp : out_link_pts){
        log2->add(kp->pos);
    }
    
    //停止线
    for (auto& object : session->raw_object_ret_list) {
        if(object->ele_type == 6){
            auto log3 = session->add_debug_log(utils::DisplayInfo::LINE, "stopline");
            log3->color = {255, 0, 255};
            for(auto& pt : object->list){
                log3->add(pt->pos, 10);
            }
            
            // //停止线对应的link点
            // for(auto& keypose : object->bind_links){
                //     log3_link->add(keypose->pos, 10);
                // }
                
            }
        }
        
    //起始点
    auto log_in_start = session->add_debug_log(utils::DisplayInfo::POINT, "in");
    log_in_start->color = {255, 255, 0};   //黄色
    auto log3_link = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "st_link");
    log3_link->color = {255, 165, 0};
    for(auto &info : session->key_pose_info_m){
        if(info.first->in_out_status == 1){
            log_in_start->add(info.first->pos);
            log3_link->add(info.first->pos);

        }
    }
    
    //退出点
    auto log_out = session->add_debug_log(utils::DisplayInfo::POINT, "out");
    log_out->color = {255, 255, 0};  //黄
    for(auto &info : session->key_pose_info_m){
        if(info.first->in_out_status == 2){
            log_out->add(info.first->pos);
        }
    }



    //左右边界点
    auto log_bd = session->add_debug_log(utils::DisplayInfo::POINT, "bd_point");
    log_bd->color = {74, 88, 240};  //浅蓝
    auto log_bd2 = session->add_debug_log(utils::DisplayInfo::POINT, "bd_point2");
    log_bd2->color = {255, 112, 224}; //粉
    auto log_bd_new = session->add_debug_log(utils::DisplayInfo::POINT, "bd_new");
    log_bd_new->color = {36, 30, 90}; //深蓝
    for(auto &info : session->key_pose_info_m){
        for(auto& road_cross_pt : info.second){
            if(!road_cross_pt->road_boundary_pts.empty()){
                if(road_cross_pt->road_boundary_pts[0]->is_new == true){ 
                    log_bd_new->add(road_cross_pt->road_boundary_pts[0]->point); //新增的
                }else{
                    log_bd->add(road_cross_pt->road_boundary_pts[0]->point);
                }

                if(road_cross_pt->road_boundary_pts[1]->is_new == true){
                    log_bd_new->add(road_cross_pt->road_boundary_pts[1]->point); //新增的
                }else{
                    log_bd2->add(road_cross_pt->road_boundary_pts[1]->point);
                }
            }
        }
           
    }

    
    //交到的车道中心线的点, 包含原有的， 和缺失的   
    auto log_center_pt = session->add_debug_log(utils::DisplayInfo::POINT, "center_pt");
    log_center_pt->color = {214, 255, 216}; //浅绿色
     auto log_miss_pts = session->add_debug_log(utils::DisplayInfo::POINT, "is_miss");
    log_miss_pts->color = {255, 0, 0}; //红
    for(auto &info : session->key_pose_info_m){
        for(auto& road_cross_pt : info.second){
            for(auto& coss_point : road_cross_pt->lane_center_pts)
                if(coss_point->is_new){
                    // std::cout<< "----------miss point--------------------" << std::endl;
                    log_miss_pts->add(coss_point->point);
                }else{
                    log_center_pt->add(coss_point->point);
                }
                    
        }
    }
    
    // //缺失点
    // auto log_miss_pts = session->add_debug_log(utils::DisplayInfo::POINT, "miss_pts");
    // log_miss_pts->color = {255, 0, 0}; //红
    // //计算出的期望点
    // auto log_exist_pts = session->add_debug_log(utils::DisplayInfo::POINT, "exist_pts");
    // log_exist_pts->color = {199, 91, 91}; //浅红
    // for(auto& object : session->lane_coverage_m){
    //     for(auto& pt : object.second.missed_pts){
    //         log_miss_pts->add(pt);
    //     }
    //     for(auto& pt2 : object.second.prev_exist_pts){
    //         log_exist_pts->add(pt2);
    //     }
    // }

    //merge_feature的线
    #if 1
    UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;
    for (auto &it : session->key_pose_map) {
        for (int64_t i = 0; i < it.second.boundary_line_group_list.size(); ++i) {
            auto &group_list = it.second.boundary_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_boundary");
            log->color = {81, 89, 240};
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                if(prev) {
                    if(j < 2 || alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1 || group_list->boundary_type == 1) {
                        auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                        ele.label.opt_label = i;
                        ele.label.intensity_opt = j;
                        ele.label.score = pt->score;
                        if (pt->invalid()) {
                            ele.color = {255, 255, 100};
                        }
                        ele.label.label = 1; // 临时弄的
                        prev=pt.get();
                    }

                } else {
                     prev=pt.get();
                }
            }

            trail_map[it.first].push_back(log);
        }
        #if 1
        for (int64_t i = 0; i < it.second.lane_line_group_list.size(); ++i) {
            auto &group_list = it.second.lane_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane");
            log->color = {255, 255, 255};
            LaneLineSample*prev=NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                if(prev) {
                    if(j<2 || alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
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
                } else {
                    prev=pt.get();
                }
            }

            trail_map[it.first].push_back(log);
        }

        for (int64_t i = 0; i < it.second.lane_center_line_group_list.size(); ++i) {
            auto &group_list = it.second.lane_center_line_group_list[i];
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
                if(prev) {
                    if(j<2 || alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                    // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                        auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                        ele.label.opt_label = i;
                        ele.label.score = pt->score;
                        ele.label.label = 3; // 临时弄的
                        if (pt->invalid()) {
                            ele.color = {100, 100, 100};
                        }
                        prev=pt.get();
                        ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                    } 
                } else {
                    prev=pt.get();
                }
              
            }
            trail_map[it.first].push_back(log);
        }
        #endif

        continue;
    }
    #endif

    session->save_debug_info("cal_coverage");

    return fsdmap::SUCC;
}


//找form16的进入 和退出点 
void RoadModelProcCalCoverage::find_cross_link_points(RoadModelSessionData* session, std::vector<KeyPose*>& in_link_pts, std::vector<KeyPose*>& out_link_pts) {
    std::vector<KeyPose*> cross_links;
    // std::cout << "session->link_sample_list.size() : " << session->link_sample_list.size() << std::endl;
    for(auto& line : session->link_sample_list){
        KeyPose* first_16 = nullptr;
        KeyPose* last_16 = nullptr;
        for(auto& keypose : line->list){
            if(keypose->from_raw_link->form == "16"){
                if(!first_16){
                    first_16 = keypose;
                }
                last_16 = keypose;
            }
        }

        if(first_16){
            in_link_pts.push_back(first_16); 
        }
        if(last_16){
            out_link_pts.push_back(last_16); 
        }
    }

}

// 找到靠近停止线的keypose
KeyPose* RoadModelProcCalCoverage::find_in_start_keypose(RoadModelSessionData* session, KeyPose* in_pt, 
    std::map<KeyPose*, RoadObjectInfo*>& stop_line_list_m) {
    
    KeyPose* src_pt = in_pt;
    KeyPose* in_start_keypose = nullptr;
    bool first_find = false;

    while(in_pt){
        // log5->add(in_pt->pos);
        for(auto& object : in_pt->object_list){
            if(object->ele_type == 6 && !in_pt->from_link->list.empty()){
                stop_line_list_m[src_pt]=object;
                
                //找最近的点
                std::vector<Eigen::Vector3d> link_pts;
                for(auto& keypose : in_pt->from_link->list){
                    link_pts.push_back(keypose->pos);
                }
                
                //计算交点
                std::vector<Eigen::Vector3d> cross_points;
                bool is_intersect = alg::get_cross_point_with_polygon(object->list[0]->pos, object->list[1]->pos, link_pts, cross_points);
                 
                if(!cross_points.empty()){
                    double min_dis = 1000000;
                    for(auto& keypose : in_pt->from_link->list){
                        double dis = alg::calc_dis(cross_points[0], keypose->pos);
                        if(dis < min_dis){
                            min_dis = dis;
                            in_start_keypose = keypose;
                            in_start_keypose->in_out_status = 1;
                        }
                    }
                    first_find =true;
                    break;
                }else{
                    LOG_WARN("link with stop_line cross_points is empty");
                }
            }
        }
        
        if(first_find){
            break;
        }

        if (in_pt->prev != nullptr){
            in_pt = in_pt->prev;
        }else{
            break;
        }
    }

    

    return in_start_keypose;
}


// 找退出车道的起始点
KeyPose* RoadModelProcCalCoverage::find_out_start_keypose(RoadModelSessionData* session, KeyPose* in_pt, std::vector<KeyPose*> in_link_pts,
     std::map<KeyPose*, RoadObjectInfo*>& stop_line_list_m) {
    KeyPose* out_start_keypose = nullptr;
    std::vector<KeyPoseLine*> rm_self_links; //剔除自身in_pt对应的link

    for(auto& pt : in_link_pts){
        if(pt != in_pt){
            rm_self_links.push_back(pt->from_link);
        }
    }


    auto it = stop_line_list_m.find(in_pt);
    if(it == stop_line_list_m.end()){
        LOG_ERROR("in_pt not found in stop_line_list_m");
        return nullptr;  //异常情况， 这个in_pt 是掉头的link
    }

    RoadObjectInfo* object = it->second;
    std::vector<KeyPose*> stopline_cross_keypose;


    for(auto& link : rm_self_links){
        //转为vector
        std::vector<Eigen::Vector3d> link_pts;
        for(auto& keypose : link->list){
            link_pts.push_back(keypose->pos);
        }
        // std::cout << "link_pts.size() : " << link_pts.size() << std::endl;
        
        //计算单条link的交点
        std::vector<Eigen::Vector3d> cross_points;
        bool is_intersect = alg::get_cross_point_with_polygon(object->list[0]->pos, object->list[1]->pos, 
            link_pts, cross_points, true, false, 30);
            
        if(is_intersect){
            double min_dis = 1000000;
            KeyPose* tmp_keypose = nullptr;
            for(auto& keypose : link->list){
                double dis = alg::calc_dis(cross_points[0], keypose->pos);
                if(dis < min_dis){
                    min_dis = dis;
                    tmp_keypose = keypose;
                }
            }
            stopline_cross_keypose.push_back(tmp_keypose);
            // std::cout << " out link: " << std::endl;
        } else {
            // LOG_WARN("No intersection found with the stop line for link");
        }
    }


    //多条link中找最近的交点，即为对向的keypose
    if (!stopline_cross_keypose.empty()) {
        double min_dis2 = 100000;
        for(auto& keypose : stopline_cross_keypose){
            double dis2 = alg::calc_dis(keypose->pos, object->list[0]->pos); 
            if(dis2 < min_dis2){
                min_dis2 = dis2;
                out_start_keypose = keypose;
                out_start_keypose->in_out_status = 2;
            }
        }
    } else {
        // LOG_WARN("No cross key poses found for the stop line");
    }

    
    return out_start_keypose;
}


Eigen::Vector3d RoadModelProcCalCoverage::cal_road_true_dir(RoadModelSessionData* session, KeyPose* kp){
    std::vector<LaneCenterFeature *> lcfs;
    std::vector<Eigen::Vector3d> dirs;
    Eigen::Vector3d road_dir;

    session->merge_lane_center_sample_tree.search(kp->pos, 5, lcfs);
    for (const auto lc : lcfs){   
        dirs.push_back(lc->dir);
    }

    if (!lcfs.empty()){
        road_dir = alg::get_direction(dirs);
        
    }else{
        road_dir = kp->dir;  // 防止报错
    }

    return road_dir;
}


// 计算道路宽度
bool RoadModelProcCalCoverage::cal_road_width(RoadModelSessionData* session, KeyPose* keypose, Eigen::Vector3d road_dir,
    Eigen::Vector3d& left_bf, Eigen::Vector3d& right_bf, double& W_road) {

    auto v_dir = alg::get_vertical_dir(road_dir);
    double dis_extend = keypose->from_raw_link->lanenum_sum * FLAGS_standard_lane_width2;
    Eigen::Vector3d pt1 = keypose->pos + v_dir * dis_extend;
    Eigen::Vector3d pt2 = keypose->pos - v_dir * dis_extend;
        
    //找全部的道路边界交点
    std::vector<Eigen::Vector3d> cross_points;
    for(const auto& bd_line : session->merge_boundary_line_list){
        //转化为vector
        std::vector<Eigen::Vector3d> bd_line_pts;
        for(auto& pt :bd_line->list){
            bd_line_pts.push_back(pt->pos);
        }
        
        std::vector<Eigen::Vector3d> res;
        bool is_intersect = alg::get_cross_point_with_polygon(pt1, pt2, bd_line_pts, res, true, true);
        for(auto& pt : res){
            cross_points.push_back(pt);
        }
    }

    //以keypose为中心，分左右
    std::vector<Eigen::Vector3d> left_cross_points, right_cross_points;
    for(auto& pt : cross_points){
        if(alg::judge_left2(pt, keypose->pos, road_dir) == 1){
            left_cross_points.push_back(pt);
        }else{
            right_cross_points.push_back(pt);
        }
    }

    //计算左右的距离keypose最近点的
    //png是左手系，从电脑向人为z方向，所以看起来会反过来， 有点别扭
    if(left_cross_points.size() > 0){
        left_bf = alg::find_closest_point(left_cross_points, keypose->pos);
    }else{
        return false;
    }

    if(right_cross_points.size() > 0){
        right_bf = alg::find_closest_point(right_cross_points, keypose->pos);
    }else{
        return false;
    }

    W_road = alg::calc_dis(left_bf, right_bf, true);
    return true;    
}

void RoadModelProcCalCoverage::sort_points_by_distance(std::vector<Eigen::Vector3d>& all_points, const Eigen::Vector3d& left_bf) {
    std::sort(all_points.begin(), all_points.end(), [&left_bf](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
        return alg::calc_dis(lhs, left_bf) < alg::calc_dis(rhs, left_bf);
    });
}

void RoadModelProcCalCoverage::rm_duplicate_pts(std::vector<Eigen::Vector3d>& points, double dis_threshold){
    auto it =points.begin();
    while(it != points.end()){
        auto next_it = std::next(it);
        while(next_it != points.end()){
            if(alg::calc_dis(*it, *next_it) < dis_threshold){
                next_it = points.erase(next_it);
            }else{
                next_it++;
            }
        }
        it++;
    }
}


// 实现计算实际的车道中心线数
int RoadModelProcCalCoverage::cal_actual_lane_center_count(RoadModelSessionData* session, KeyPose* keypose, 
    Eigen::Vector3d road_dir,double search_radius, Eigen::Vector3d left_bf, Eigen::Vector3d right_bf , 
    std::vector<Eigen::Vector3d>& in_road_center_points) {

    std::vector<LaneCenterFeature *> lcfs;
    session->merge_lane_center_sample_tree.search(keypose->pos, search_radius, lcfs);

    std::unordered_set<LaneCenterGroupLine*> lines;
    for(auto& lcf : lcfs){
        lines.insert(lcf->group_line);
    }

    auto v_dir = alg::get_vertical_dir(road_dir);
    double dis_extend = 50;
    Eigen::Vector3d pt1 = keypose->pos + v_dir * dis_extend;
    Eigen::Vector3d pt2 = keypose->pos - v_dir * dis_extend;
        
    //找全部的中心线交点
    std::vector<Eigen::Vector3d> cross_points;
    for(const auto& center_line : lines){
        //转化为vector
        std::vector<Eigen::Vector3d> center_line_pts;
        for(auto& pt :center_line->list){
            center_line_pts.push_back(pt->pos);
        }
        
        std::vector<Eigen::Vector3d> res;
        bool is_intersect = alg::get_cross_point_with_polygon(pt1, pt2, center_line_pts, res, true, false);
        for(auto& pt : res){
            cross_points.push_back(pt);
        }
    }
    
    //保留道路边界内的中心点
    for(auto &  pt : cross_points){
        if((alg::judge_left2(pt, left_bf, road_dir) < 1) && (alg::judge_left2(pt, right_bf, road_dir) > -1)){
            in_road_center_points.push_back(pt);
        }
    }

    // std::cout <<"lcfs " << lcfs.size()  << " line: " << lines.size() << " cross_points.size() " << cross_points.size() << " in_road_center_points.size() " << in_road_center_points.size() << std::endl;

    //去重
    rm_duplicate_pts(in_road_center_points, 0.25);

    //排序
    sort_points_by_distance(in_road_center_points, left_bf);
    int N_lane_center = in_road_center_points.size();

    return N_lane_center;
}


// 实现计算实际的车道线数
int RoadModelProcCalCoverage::cal_actual_lane_count(RoadModelSessionData* session, KeyPose* keypose, 
    Eigen::Vector3d road_dir,double search_radius, Eigen::Vector3d left_bf, Eigen::Vector3d right_bf , 
    std::vector<Eigen::Vector3d>& in_road_lane_points) {

    std::vector<LaneLineSample *> lfs; //车道线的点 lf  lane_feature
    session->lane_line_sample_tree.search(keypose->pos, search_radius, lfs);

    std::unordered_set<LaneLineSampleGroupLine*> lines;
    for(auto& lf : lfs){
        lines.insert(lf->group_line);
    }


    auto v_dir = alg::get_vertical_dir(road_dir);
    double dis_extend = 50;
    Eigen::Vector3d pt1 = keypose->pos + v_dir * dis_extend;
    Eigen::Vector3d pt2 = keypose->pos - v_dir * dis_extend;
        
    //找全部的车道线交点
    std::vector<Eigen::Vector3d> cross_points;
    for(const auto& lane_line : lines){
        //转化为vector
        std::vector<Eigen::Vector3d> lane_line_pts;
        for(auto& pt :lane_line->list){
            lane_line_pts.push_back(pt->pos);
        }
        
        std::vector<Eigen::Vector3d> res;
        bool is_intersect = alg::get_cross_point_with_polygon(pt1, pt2, lane_line_pts, res, true, false);
        for(auto& pt : res){
            cross_points.push_back(pt);
        }
    }
    
    //保留道路边界内的车道线点
    for(auto &  pt : cross_points){
        if((alg::judge_left2(pt, left_bf, road_dir) < 1) && (alg::judge_left2(pt, right_bf, road_dir) > -1)){
            in_road_lane_points.push_back(pt);
        }
    }

    //去重
    rm_duplicate_pts(in_road_lane_points, 0.15);
    //排序
    sort_points_by_distance(in_road_lane_points, left_bf);
    int N_lane = in_road_lane_points.size() -1;

    return N_lane;
}


//版本2
void RoadModelProcCalCoverage::cal_horizontal_miss_positions2(RoadModelSessionData* session, std::shared_ptr<RoadCrossPoints> &road_cross_pts, 
    KeyPose* origin_start, KeyPose* start_keypose, Eigen::Vector3d road_dir,
    int N_hope, int N_lane_center, double W_road, double W_lane_t, const Eigen::Vector3d& left_bf, const Eigen::Vector3d& right_bf,
    std::vector<Eigen::Vector3d>& in_road_center_points) {
        
    std::vector<Eigen::Vector3d> all_points; // 理论上的中心点
    std::vector<bool> found(N_hope, false); // 标记对应车道是否缺
    double scale = 1;
    Eigen::Vector3d search_dir = alg::get_dir(left_bf, right_bf);

    // 从 right_bf 开始计算理
    Eigen::Vector3d A = right_bf + search_dir * (W_lane_t / 2.0);
    all_points.push_back(A);

    int found_count = 0;

    for(const auto& pt : in_road_center_points) {
        if(alg::calc_dis(A, pt) < (W_lane_t / 2.0) * scale) {
            // 找到第一个实际车道点，从该点开始继续计算后续的理论中心点
            all_points.back() = pt;
            found[0] = true;
            found_count++;
                break;
            }
        }

    for(int i = 1; i < N_hope; ++i) {
        if(found[i - 1]) {
            // 如果上一个点找到了，基于上一个点的位置继续计算
            A = all_points.back() + search_dir * W_lane_t;
            all_points.push_back(A);
            found[i] = false;

            // 检查新点位置是否找到
            for(const auto& pt : in_road_center_points) {
                if(alg::calc_dis(A, pt) < (W_lane_t / 2.0) * scale) {
                    all_points.back() = pt;
                    found[i] = true;
                    found_count++;
                    break;
                }
            }
        }
        // 如果上一个点没有找到，则直接向 left_bf 方向计算理论点
        else {
            A = right_bf + search_dir * (i * W_lane_t + W_lane_t / 2.0);
            all_points.push_back(A);
            found[i] = false;

            for(const auto& pt : in_road_center_points) {
                if(alg::calc_dis(A, pt) < (W_lane_t / 2.0) * scale) {
                    all_points.back() = pt;
                    found[i] = true;
                    found_count++;
                    break;
                }
            }
        }
    }
    
    for(int i = 0; i < N_hope; ++i) {
        if(found[i]) {
            auto cross_pt = std::make_shared<CrossPoint>(false, all_points[i]);
            road_cross_pts->lane_center_pts.push_back(cross_pt);
        }
        else {
            auto cross_pt = std::make_shared<CrossPoint>(true, all_points[i]);
            road_cross_pts->lane_center_pts.push_back(cross_pt);
        }
    }
}


//版本1
void RoadModelProcCalCoverage::cal_horizontal_miss_positions(RoadModelSessionData* session, KeyPose* keypose, Eigen::Vector3d road_dir,
    int N_hope, int N_lane_center, double W_road, double W_lane_t, const Eigen::Vector3d& left_bf, const Eigen::Vector3d& right_bf,
    std::vector<Eigen::Vector3d>& in_road_center_points) {
        
    std::vector<Eigen::Vector3d> all_points; // 理论上的中心点
    Eigen::Vector3d road_center = (left_bf + right_bf) / 2;
    for(int i = 0; i < N_hope; ++i) {
        double offset = i * W_lane_t - W_road / 2 + W_lane_t / 2;
        Eigen::Vector3d pos = road_center + alg::get_vertical_dir(road_dir) * offset;
        all_points.push_back(pos);
    }
    
    //从左边界开始
    sort_points_by_distance(all_points, left_bf);

    LaneCoverage lane_coverage;
    
    // 遍历每条理论上存在的车道，检查实际位置中是否存在对应的车道
    for(int i = 0; i < N_hope; ++i) {
        Eigen::Vector3d lane_center = all_points[i]; 
        bool found = false;
        for(int j = 0; j < N_lane_center; ++j) {
            if(alg::calc_dis(lane_center, in_road_center_points[j]) < (W_lane_t / 2.0)*0.8 ) {
                // lane_coverage.prev_exist_pts.push_back(in_road_center_points[j]);
                lane_coverage.prev_exist_pts.push_back(all_points[i]);
                found = true;
                break;
            }
        }

        if(!found) {
            lane_coverage.missed_pts.push_back(all_points[i]);
        }
    }

    session->lane_coverage_m[keypose] = lane_coverage;
}


void RoadModelProcCalCoverage::cal_triangle_island_miss_positions(KeyPose* keypose, int N_hope, int N_lane_center){
}




bool all_elements_equal(const std::vector<int>& vec) {
    if (vec.empty()) return true; // 空向量视为所有元素相等
    return std::all_of(vec.begin(), vec.end(), [first = vec[0]](int element) { return element == first; });
}

int RoadModelProcCalCoverage::recovery_single_road_cross_point(std::shared_ptr<RoadCrossPoints>& road_cross_pts, bool first_iteration) {      
    //1. 初始化变量
    int N_road = road_cross_pts->N_road; // 道路边界算出的车道数
    int N_lane = road_cross_pts->N_lane; // 车道边界算出的车道数
    int N_lane_center = road_cross_pts->N_lane_center; // 车道中心线算出的车道数
    int N_link = road_cross_pts->N_link; // link上自带的车道数

    std::vector<int> all_predict_lane_num = {N_road, N_lane, N_lane_center, N_link};
    std::vector<int> all_predict_lane_num_except_link = {N_road, N_lane, N_lane_center};
    int count_non_zero = std::count_if(all_predict_lane_num_except_link.begin(), all_predict_lane_num_except_link.end(), [](int value){
        return value > 0;
    });

    // if (count_non_zero <= 1) {
    //     std::cout << "not enough info" << std::endl;
    //     return -1;
    // }

    road_cross_pts->N_hope = static_cast<int>(first_iteration ? (std::min(static_cast<int>(std::round(N_road)), N_link)) :
                            std::max(static_cast<int>(std::round(N_road)), N_link));
    
    static int cnt_center_line = 0;
    //2.  如果车道线、中心线、边界线都相等，则认为是已经完整了（如果 link 数也相等，就更好了）
    if ((all_elements_equal(all_predict_lane_num_except_link) == true && N_road > 0)
        || all_elements_equal(all_predict_lane_num) == true) {
        road_cross_pts->N_hope = N_road;
    } else if (N_road > 0) {
        //3. TODO
        if (N_road == N_lane_center && N_road > N_lane) {
            // if(N_road == 1) {
            //     double lane_width = 0;
            //     for (int i = 0; i < N_road; i++) {
            //         lane_width += alg::calc_dis(road_cross_pts->road_boundary_pts[i], road_cross_pts->road_boundary_pts[i+1]);
            //     }
            //     lane_width /= N_lane;
            //     double lane_width_half = lane_width/2;
            //     Eigen::vector3d mid = (road_cross_pts->lane_pts[i]+road_cross_pts->lane_pts[i+1])/2;
            //     pred[i] = std::make_shared<CrossPoint>(true, mid);
            // }
            // auto& left_bf = road_cross_pts->road_boundary_pts[0].pos;
            // // auto& right_bf = road_cross_pts->road_boundary_pts[1].pos;
            // std::sort(road_cross_pts->lane_center_pts.begin(), road_cross_pts->lane_center_pts.end(), 
            //         [left_bf] (const std::shared_ptr<CrossPoint> &lhs, const std::shared_ptr<CrossPoint> &rhs) -> bool {
            //             return alg::calc_dis(lhs->point, left_bf) < alg::calc_dis(rhs->point, left_bf);
            //         });
            // std::sort(road_cross_pts->lane_pts.begin(), road_cross_pts->lane_pts.end(), 
            //         [left_bf] (const std::shared_ptr<CrossPoint> &lhs, const std::shared_ptr<CrossPoint> &rhs) -> bool {
            //             return alg::calc_dis(lhs->point, left_bf) < alg::calc_dis(rhs->point, left_bf);
            //         });
            road_cross_pts->N_hope = N_road;
        } else if (N_road == N_lane && N_road > N_lane_center) {
            //4. 道路左右边界完整， 车道线也完整， 但车道中心线缺失，则补点
            auto& left_bf = road_cross_pts->road_boundary_pts[0]->point;
            // auto& right_bf = road_cross_pts->road_boundary_pts[1].pos;
            // std::sort(road_cross_pts->lane_pts.begin(), road_cross_pts->lane_pts.end(), 
            //         [left_bf] (const std::shared_ptr<CrossPoint> &lhs, const std::shared_ptr<CrossPoint> &rhs) -> bool {
            //             return alg::calc_dis(lhs->point, left_bf) < alg::calc_dis(rhs->point, left_bf);
            //         });
            // std::sort(road_cross_pts->lane_center_pts.begin(), road_cross_pts->lane_center_pts.end(), 
            //         [left_bf] (const std::shared_ptr<CrossPoint> &lhs, const std::shared_ptr<CrossPoint> &rhs) -> bool {
            //             return alg::calc_dis(lhs->point, left_bf) < alg::calc_dis(rhs->point, left_bf);
            //         });

            //4.1 计算需补点的位置
            double lane_width = 0;
            for (int i = 0; i < N_lane; i++) {
                lane_width += alg::calc_dis(road_cross_pts->lane_pts[i]->point, road_cross_pts->lane_pts[i+1]->point);
            }
            lane_width /= N_lane;
            double lane_width_half = lane_width/2;
            
            const auto lane_pt_0 = road_cross_pts->lane_pts[0]->point;
            std::vector<std::shared_ptr<CrossPoint>> predict_lane_centers(N_lane, nullptr);
            for (int i = 0; i < N_lane_center; i++) {
                double d = std::fabs(alg::calc_dis(road_cross_pts->lane_center_pts[i]->point, lane_pt_0) - lane_width_half) / lane_width;
                int target_index = int(d);
                predict_lane_centers[target_index] = road_cross_pts->lane_center_pts[i];
            }
            
            //4.2 补点逻辑
            int cur_add_cnt = 0;
            for (int i = 0; i < N_lane; i++) {
                auto& pred = predict_lane_centers[i];
                if(pred == nullptr) {
                    Eigen::Vector3d mid = (road_cross_pts->lane_pts[i]->point+road_cross_pts->lane_pts[i+1]->point)/2;
                    pred = std::make_shared<CrossPoint>(true, mid);

                    cur_add_cnt++;
                    cnt_center_line++;
                }
            }

            road_cross_pts->lane_center_pts = predict_lane_centers;
            road_cross_pts->N_hope = N_road;
            // std::cout << "cnt_center_line " << cnt_center_line << " cur: " << cur_add_cnt << " N_road: " << N_road<< " N_lane: " << N_lane<< " N_lane_center: " << N_lane_center<< " N_link: " << N_link <<  std::endl;

        } else {
            
            //5.  都无法完全确定
        }
    } else {
        // 如果没有道路边界线，就先不处理，后续增强
    }

    // TODO：双向道路处理
    if (first_iteration) {
        road_cross_pts->N_start = road_cross_pts->N_hope;
    }

    //6. 计算车道数差异  
    int N_diff = road_cross_pts->N_hope - N_lane_center;
    if (N_diff == 0) {
        road_cross_pts->is_full  = true;
    }
    road_cross_pts->N_diff = N_diff;


    return 0;
}


int RoadModelProcCalCoverage::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_cal_coverage_save_data_enable) {
        return fsdmap::SUCC;
    }

    // session->set_display_name("cal_coverage");
    // session->save_debug_info("cal_coverage");

    return fsdmap::SUCC;
}

}

}
