


#include "road_model_proc_bind_trail.h"


DEFINE_bool(bind_trail_enable, true, "bind_trail_enable");
DEFINE_bool(bind_trail_debug_pos_enable, true, "bind_trail_debug_enable");
DEFINE_bool(bind_trail_save_data_enable, true, "bind_trail_save_data_enable");
DEFINE_bool(bind_trail_save_data_save_detail_enable, false, "bind_trail_save_data_save_detail_enable");
DEFINE_double(bind_trail_turn_theta_threshold, 20, "bind_trail_turn_theta_threshold");
DEFINE_double(bind_trail_pos_match_lane_radius, 30, "bind_trail_pos_match_lane_radius");
DEFINE_double(bind_trail_pos_match_lane_z_max, 6, "bind_trail_pos_match_lane_z_max");
DEFINE_double(bind_trail_pos_match_lane_z_min, -2, "bind_trail_pos_match_lane_z_min");
DEFINE_double(bind_trail_pos_match_lane_theta, 30, "bind_trail_pos_match_lane_theta");
DEFINE_double(bind_trail_pos_match_crosswalk_radius, 15.0, "bind_trail_pos_match_crosswalk_radius");
DEFINE_double(bind_trail_pos_match_crosswalk_h_scope, 0.5, "bind_trail_pos_match_crosswalk_h_scope");
DEFINE_double(bind_trail_pos_match_crosswalk_v_scope, 5, "bind_trail_pos_match_crosswalk_v_scope");
DEFINE_double(bind_trail_bind_lc_width_buff, 0.8, "bind_trail_bind_lc_width_buff");
DEFINE_double(bind_trail_pos_match_stopline_theta_thres, 60, "bind_trail_pos_match_stopline_theta_thres");
DEFINE_double(bind_trail_pos_match_crosswalk_theta_thres, 60, "bind_trail_pos_match_crosswalk_theta_thres");
DECLARE_double(display_scope_buff);
DECLARE_double(display_scale_rate);


namespace fsdmap {
namespace road_model {


fsdmap::process_frame::PROC_STATUS RoadModelProcBindTrail::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_bind_trail_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_bind_trail_debug_pos_enable;

    CHECK_FATAL_PROC(get_poss_cross_point_mutil(session), "get_poss_cross_point_mutil");

    CHECK_FATAL_PROC(sort_new_lc(session), "sort_new_lc");

    // CHECK_FATAL_PROC(get_poss_cross_fls(session), "get_poss_cross_fls");

    CHECK_FATAL_PROC(get_poss_cross_object2(session), "get_poss_cross_object");


    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

// 遍历session->road_segment_list每个road_segment->pos_sample_list的poss,
// 选出poss附近30m以内的车道中心线点sec,这些点与自车该位置处方向向量夹角在30度以内,
// 对sec里面的每个车道中心点lc，以lc左侧车道点为起点，next左侧车道点与lc左侧车道点连线为方向向量，
// 与poss为起点poss方向的垂向为方向向量的交点作为新的左侧车道线点，右侧车道线点按同理构造，最后用这新生成的这两个左右侧车道线点构建新的车道中心点。
int RoadModelProcBindTrail::get_poss_cross_point_mutil(RoadModelSessionData* session) {
    // 定义缓冲区范围，用于判断车道线的宽度
    double buff_scope = FLAGS_bind_trail_bind_lc_width_buff;  // 0.8

    // 遍历所有的道路段
    for (auto &road_segment : session->road_segment_list) {
        // 遍历当前道路段下的每个位置样本
        for (auto &poss : road_segment->pos_sample_list) {
            // 在线程池中为每个位置样本执行任务
            session->thread_pool->schedule([&buff_scope, road_segment, poss, session, this](
                    utils::ProcessBar *process_bar) {

                // 调试输出当前的位置信息
                session->debug_pos(poss->pos);
                
                // 存储车道中心特征的列表
                std::vector<LaneCenterFeature*> secs;
                std::vector<LaneCenterFeature*> op_secs;
                
                // 获取当前样本的道路边界信息
                auto &boundary = poss->boundary;

                // 查找与当前样本位置附近 30 米范围内的车道中心线
                // LOG_INFO("bind trail secs:{}",secs.size());
                search_nearest_lc(session, poss, secs, op_secs);
                
                // 计算垂直点位置及方向
                Eigen::Vector3d poss_v_pt = alg::get_vertical_pos(poss->pos, poss->dir, 50, true);
                auto poss_v_dir = alg::get_vertical_dir(poss->dir);

                // 遍历所有符合条件的车道中心线点
                int cnt_0 (0), cnt_1 (0), cnt_2(0), cnt_3 (0), cnt_4 (0), cnt_5(0);
                for (auto &lc : secs) {
                    // 调试输出当前车道中心点与位置样本的关系
                    session->debug_pos2(poss->pos, lc->pos);
                    
                    // 获取道路边界的道路中心点
                    auto rc = boundary.get_road_center(lc->pos, 0);
                    // if (rc == NULL || rc->invalid()) {
                    if (rc == NULL) {
                        cnt_0 ++;
                        continue;
                    }

                    // 遍历车道中心点上下游的所有相关车道
                    for (auto &next_ptr : lc->context.all_next) {
                        // if (next_ptr.invalid()) {
                        //     cnt_1++;
                        //     continue;
                        // }
                        auto &next = next_ptr.src;
                        // if (next->invalid()) {
                        //     cnt_2++;
                        //     continue;
                        // }

                        // 计算当前车道方向与下游车道的夹角
                        double theta = alg::calc_theta(next->dir, poss->dir);
                        // 如果夹角大于 90 度，则跳过
                        if (theta > 90) {
                            cnt_3++;
                            continue;
                        }

                        // 计算交点
                        DisDirPoint cross_point;
                        alg::calc_dir(next->pos, lc->pos, cross_point.dir);
                        if (!alg::get_cross_point_for_segment(lc->pos, next->pos,
                                    poss->pos, poss_v_pt, cross_point.pos, 1)) {
                            cnt_4++;
                            continue;
                        }

                        // 计算车道中心点与交点之间的偏移量
                        double offset = alg::calc_dis(lc->pos, cross_point.pos);
                        
                        // 根据车道信息及交点方向构造新的车道中心点
                        auto new_lc = session->sample_new_lc(lc, next, poss->pos, poss_v_dir, NULL);
                        new_lc->road_segment = poss->road_segment;
                        new_lc->key_pose = poss;
                        
                        // 计算新的车道中心点与位置样本的水平距离
                        double h_dis = alg::calc_hori_dis(new_lc->pos, poss->pos, poss->dir);
                        if (h_dis > 0.1) {
                            int a = 1;
                        }

                        // 根据车道宽度和缓冲区设置限制新的车道中心点的范围
                        double buff = new_lc->width / 2 - buff_scope;
                        buff = buff < 0 ? 0 : buff;
                        auto rc = boundary.get_road_center(new_lc->pos, -buff);
                        // if (rc == NULL || rc->invalid()) {
                        if (rc == NULL) {
                            new_lc->filter_status = 13;
                            cnt_5++;
                            continue;
                        }

                        // 判断车道中心点相对于道路中心的位置，更新状态
                        new_lc->side_status = rc->in_scope(new_lc->pos, 0, 2) ? 2 : 1;
                        new_lc->road_index = rc->index;

                        // 在全局锁中，更新车道中心点的交叉点信息
                        session->lock_run([&](void)->void {
                            lc->cross_point[next].push_back(new_lc);
                            session->lane_center_feature_bind.push_back(lc); // 全局没被调用
                        });

                        // 将新的车道中心点添加到位置样本的车道列表中
                        poss->lane_center_list.push_back(new_lc);

                        // 更新进度条
                        process_bar->num_biz++;
                    }
                }
            });
        }
    }

    // 等待线程池中的所有任务完成
    session->thread_pool->wait(2, "process_bind_lane_center");

    // 遍历所有道路段和位置样本，将所有车道中心点插入到绑定树中
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            for (auto &lc : poss->lane_center_list) {
                session->bind_lane_center_tree.insert(lc->pos, lc);     // 全局未被调用
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::sort_new_lc(RoadModelSessionData* session) {
    // 遍历所有生成的车道中心特征
    for (auto &lc : session->lane_center_feature_gen) {
        // 如果车道中心特征无效，跳过当前循环
        if (lc->invalid()) {
            continue;
        }

        // 在线程池中为每个车道中心特征执行任务
        session->thread_pool->schedule([lc, session, this](utils::ProcessBar *process_bar) {
            std::vector<std::pair<LaneCenterFeature*, double> > dis_vec;

            // 遍历当前车道中心特征的所有交叉点
            for (auto &pair : lc->cross_point) {
                dis_vec.clear();
                dis_vec.reserve(pair.second.size());

                // 计算交叉点的距离
                for (auto &lc_sub : pair.second) {
                    double dis = alg::calc_dis(lc_sub->pos, lc->pos);  // 计算当前交叉点与车道中心特征的距离
                    dis_vec.push_back(std::make_pair(lc_sub, dis));  // 将车道中心特征及其距离存入 dis_vec
                }

                // 按照距离进行排序，距离近的车道中心特征排在前面
                SORT(dis_vec, 
                    [](const std::pair<LaneCenterFeature*, double> &l, const std::pair<LaneCenterFeature*, double> &r)->bool{
                        return l.second < r.second;  // 按照距离进行升序排序
                    });

                // 更新交叉点列表，使其按距离排序
                for (int i = 0; i < dis_vec.size(); ++i) {
                    pair.second[i] = dis_vec[i].first;
                }

                // 更新进度条
                process_bar->num_biz += dis_vec.size();
            }
        });
    }

    // 等待线程池中的任务完成
    session->thread_pool->wait(1, "sort_new_lc");

    // 对车道中心特征中的交叉点进行进一步排序
    for (auto lc : session->lane_center_feature_gen) {
        session->thread_pool->schedule([lc, session, this](utils::ProcessBar *process_bar) {
        // 不同road_segment 的不匹配
        // for (auto &road_segment : session->road_segment_list) {
            // LaneCenterFeature* min_dis_lc = NULL;
            // double min_dis = 100000;
            // for (auto &pair : lc->cross_point) {
            //     double dis = alg::calc_dis(lc->ls->center.pos, pair.second.front()->ls->center.pos);
            //     if (min_dis > dis) {
            //         min_dis = dis;
            //         min_dis_lc = pair.second.front();
            //     }
            // }
            // 第一个继承节点关系
            // min_dis_lc->
            // 只做内部排序，外部排序用raw的
            for (auto &pair : lc->cross_point) {
                LaneCenterFeature* lc_prev = NULL;

                // 对每个交叉点链表进行排序，构建前后继节点关系
                for (auto &lc_new : pair.second) {
                    if (lc_prev != NULL) {
                        lc_prev->next = lc_new;  // 将前一个车道中心特征的 next 指向当前车道中心特征
                        lc_new->prev = lc_prev;  // 将当前车道中心特征的 prev 指向前一个车道中心特征
                    }
                    lc_prev = lc_new;  // 更新 lc_prev 为当前车道中心特征
                }
            }
        // }
        });
    }
    session->thread_pool->wait(1, "sort_new_lc");
    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::get_poss_cross_fls(RoadModelSessionData* session) {
    double turn_theta_threshold = FLAGS_bind_trail_turn_theta_threshold;
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            session->thread_pool->schedule([turn_theta_threshold, road_segment, poss, session, this]() {
                    session->debug_pos(poss->pos);
                    std::vector<LaneLineSample*> secs;
                    search_nearest_fls(session, poss, secs);
                    Eigen::Vector3d poss_v_pt = alg::get_vertical_pos(poss->pos, poss->dir, 2);
                    // auto &boundary = poss->boundary;
                    for (auto &fls : secs) {
                        session->debug_pos2(poss->pos, fls->pos);
                        DisDirPoint cross_point;
                        alg::calc_dir(fls->next->pos, fls->pos, cross_point.dir);
                        if (!alg::get_cross_point_for_segment(fls->pos, fls->next->pos,
                                    poss->pos, poss_v_pt, cross_point.pos, 1)) {
                            continue;
                        }
                        if (alg::calc_theta(cross_point.dir, poss->dir) > 90) {
                            cross_point.dir = -cross_point.dir;
                        }
                        

                        if (road_segment->type >= 2) {
                            double theta = alg::calc_theta(fls->dir, poss->dir, false, true);
                            if (theta > turn_theta_threshold) {
                                continue;
                            }
                        }
                        auto new_fls = session->add_ptr(session->lane_line_sample_ptr, true);
                        new_fls->init(fls);
                        new_fls->src_fls = fls;
                        new_fls->pos = cross_point.pos;
                        new_fls->dir = cross_point.dir;
                        new_fls->key_pose = poss;
                        new_fls->bind_dis = alg::calc_dis(poss->pos, cross_point.pos);
                        if (alg::judge_left(new_fls->pos, poss->pos, poss->dir) < 0) {
                            new_fls->bind_dis = -new_fls->bind_dis;
                        }
                        poss->lane_line_feature_sample_list.push_back(new_fls.get());
                    }
            });
        }
    }
    session->thread_pool->wait();
    return fsdmap::SUCC;
}


int RoadModelProcBindTrail::get_poss_cross_object2(RoadModelSessionData* session) {
    // 定义匹配半径和方向角度差异阈值
    float radius = FLAGS_bind_trail_pos_match_crosswalk_radius;  // 15.0
    RTreeProxy<KeyPose*, float, 2> keypose_tree;  // 创建空间索引树

    // 将道路段的位置信息插入到空间索引树中
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            keypose_tree.insert(poss->pos, poss);
        }
    }

    // 遍历  原始特征数据  
    for (auto& object : session->raw_object_ret_list) {
        std::vector<KeyPose*> secs;  // 存储匹配的KeyPose
        // 遍历对象的所有位置点
        for (auto& object_pt : object->list) {
            std::vector<KeyPose*> secs_one_pt;
            keypose_tree.search(object_pt->get_pos(), radius, secs_one_pt);  // 搜索匹配位置
            // 计算距离并过滤
            for(auto& keypos : secs_one_pt) {
                session->debug_pos(keypos->pos);  // 调试输出
                double dis = alg::calc_dis(object_pt->get_pos(), keypos->pos, true);
                if(dis < radius) {
                    secs.push_back(keypos);
                }
            }
        }
        std::vector<KeyPose*> secs_pos;
        keypose_tree.search(object->pos, radius, secs_pos);  // 搜索匹配位置
        // 计算距离并过滤
        for(auto& keypos : secs_pos) {
            double dis = alg::calc_dis(object->pos, keypos->pos, true);
            if(dis < radius) {
                secs.push_back(keypos);
            }
        }

        // 如果没有匹配到位置，则跳过当前对象
        if (secs.empty()) {
            continue;
        }


   
        UMAP<RoadSegment*, std::vector<KeyPose*>> object_roadsegment_dis_map;
        for (auto& keypos : secs)
        {
            session->debug_pos(keypos->pos);
            // if (object->ele_type == 6) {
            //     double theta = alg::calc_theta(object->dir, keypos->dir, true);
            //     // if (theta < FLAGS_bind_trail_pos_match_stopline_theta_thres) {
            //     //     continue;
            //     // }
            // }
            
            //后面加了穿过约束，这里去掉了
            // if (object->ele_type == 5) {
            //     double theta = alg::calc_theta(object->dir, keypos->dir, true);
            //     if (theta < FLAGS_bind_trail_pos_match_crosswalk_theta_thres) {
            //         continue;
            //     }
            // }
            object_roadsegment_dis_map[keypos->road_segment].push_back(keypos);
          
        }
        

        //处理人行横道   一个人行横道对一个link， 若人行横道粘连，则会一个人行横道对两个link
        if (object->ele_type == 5) {
            double mindis = DBL_MAX;
            for (auto iter = object_roadsegment_dis_map.begin(); iter != object_roadsegment_dis_map.end(); iter++)
            {
                if(iter->second.size() == 1)
                {
                    iter->second.at(0)->object_list.push_back(object.get());
                }
                else
                {   double mindis = DBL_MAX;
                    int minIndex = 0;
                    for (int i = 0; i < iter->second.size(); i++)
                    {
                        double temp_dis = alg::calc_dis(object->pos, iter->second.at(i)->pos, true);
                        if(temp_dis < mindis)
                        {
                            mindis = temp_dis;
                            minIndex = i;
                        }
                    }
                    if(judge_keypose_cross_crosswalk(session, object.get(), iter->second.at(minIndex)) == true){
                        iter->second.at(minIndex)->object_list.push_back(object.get());
                    }
                }
            } 
        }
        else if(object->ele_type == 6){
            //处理停止线
            //TODO:cxf 可能只靠穿过，对于直线的case，不能完全解决
            KeyPose* minKeyPose = nullptr;
            double mindis = DBL_MAX;
            for (auto iter = object_roadsegment_dis_map.begin(); iter != object_roadsegment_dis_map.end(); iter++)
            {
                if(iter->second.size() == 1)
                {
                    iter->second.at(0)->object_list.push_back(object.get());
                }
                else
                {   
                    for (int i = 0; i < iter->second.size(); i++)
                    {
                        double temp_dis = alg::calc_dis(object->pos, iter->second.at(i)->pos, true);
                        if(temp_dis < mindis)
                        {
                            mindis = temp_dis;
                            minKeyPose = iter->second.at(i);
                        }
                    }
                }
            } 
            if (minKeyPose != nullptr) {
                Eigen::Vector3d cross_point2;
                //剔除右转帮到直线的
                bool is_intersect2 = alg::get_cross_point_by_dir(minKeyPose->pos, minKeyPose->dir, object->list[0]->pos, object->list[1]->pos, cross_point2, 1, true);
                if(is_intersect2){
                    minKeyPose->object_list.push_back(object.get());
                    object->bind_links.push_back(minKeyPose);
                }
            }
        }
        else{
            //处理箭头   对比所有roadsegment中的link点后， 找到最近的link点
            KeyPose* minKeyPose = nullptr;
            double mindis = DBL_MAX;
            for (auto iter = object_roadsegment_dis_map.begin(); iter != object_roadsegment_dis_map.end(); iter++)
            {
                if(iter->second.size() == 1)
                {
                    iter->second.at(0)->object_list.push_back(object.get());
                }
                else
                {   
                    for (int i = 0; i < iter->second.size(); i++)
                    {
                        double temp_dis = alg::calc_dis(object->pos, iter->second.at(i)->pos, true);
                        if(temp_dis < mindis)
                        {
                            mindis = temp_dis;
                            minKeyPose = iter->second.at(i);
                        }
                    }
                    // iter->second.at(minIndex)->object_list.push_back(object.get());
                }
            } 
            if (minKeyPose != nullptr) {
                minKeyPose->object_list.push_back(object.get());
                object->bind_links.push_back(minKeyPose);
            }
        }

    }

    // double turn_theta_threshold = FLAGS_bind_trail_turn_theta_threshold;
    // for (auto &road_segment : session->road_segment_list) {
    //     for (auto &poss : road_segment->pos_sample_list) {
    //         session->thread_pool->schedule([turn_theta_threshold, road_segment, poss, session, this]() {
    //                 session->debug_pos(poss->pos);
    //                 std::vector<RoadObjectInfo*> secs;
    //                 search_nearest_crosswalk(session, poss, secs);
    //                 for (auto &fls : secs) {
    //                     session->debug_pos2(poss->pos, fls->pos);
    //                     auto new_fls = session->add_ptr(session->object_ptr, true);
    //                     new_fls->init(fls);
    //                     new_fls->key_pose = poss;
    //                     new_fls->bind_dis = alg::calc_vertical_dis(
    //                             fls->pos, poss->pos, poss->dir, true);
    //                     poss->crosswalk_list.push_back(new_fls.get());
    //                 }
    //         });
    //     }
    // }
    // session->thread_pool->wait();
    return fsdmap::SUCC;
}


bool RoadModelProcBindTrail::judge_keypose_cross_crosswalk(RoadModelSessionData* session, RoadObjectInfo* object, KeyPose* keypose){
    for (int i = 0; i < object->list.size() - 1; ++i) {  // 处理从 0 到 n-2 个顶点
        Eigen::Vector3d poly_from = object->list[i]->pos;
        Eigen::Vector3d poly_to = object->list[i + 1]->pos;  // 下一顶点，形成一条边

        Eigen::Vector3d cross_point;
        bool is_intersect = alg::get_cross_point_by_dir(keypose->pos, keypose->dir, poly_from, poly_to, cross_point, 1, true);
        if (is_intersect) {
            return true;
        }
    }
    return false;
}


// //原始备份 弃用了
// int RoadModelProcBindTrail::get_poss_cross_object(RoadModelSessionData* session) {
//     // 定义匹配半径和方向角度差异阈值
//     float radius = FLAGS_bind_trail_pos_match_crosswalk_radius;  // 15.0
//     RTreeProxy<KeyPose*, float, 2> keypose_tree;  // 创建空间索引树

//     // 将道路段的位置信息插入到空间索引树中
//     for (auto &road_segment : session->road_segment_list) {
//         for (auto &poss : road_segment->pos_sample_list) {
//             keypose_tree.insert(poss->pos, poss);
//         }
//     }

//     // 遍历  原始特征数据  
//     for (auto& object : session->raw_object_ret_list) {
//         std::vector<KeyPose*> secs;  // 存储匹配的KeyPose
//         // 遍历对象的所有位置点
//         for (auto& object_pt : object->list) {
//             std::vector<KeyPose*> secs_one_pt;
//             keypose_tree.search(object_pt->get_pos(), radius, secs_one_pt);  // 搜索匹配位置
//             // 计算距离并过滤
//             for(auto& keypos : secs_one_pt) {
//                 session->debug_pos(keypos->pos);  // 调试输出
//                 double dis = alg::calc_dis(object_pt->get_pos(), keypos->pos, true);
//                 if(dis < radius) {
//                     secs.push_back(keypos);
//                 }
//             }
//         }
//         std::vector<KeyPose*> secs_pos;
//         keypose_tree.search(object->pos, radius, secs_pos);  // 搜索匹配位置
//         // 计算距离并过滤
//         for(auto& keypos : secs_pos) {
//             double dis = alg::calc_dis(object->pos, keypos->pos, true);
//             if(dis < radius) {
//                 secs.push_back(keypos);
//             }
//         }

//         // 如果没有匹配到位置，则跳过当前对象
//         if (secs.empty()) {
//             continue;
//         }
      
//         UMAP<RoadSegment*, std::vector<KeyPose*>> object_roadsegment_dis_map;
//         for (auto& keypos : secs)
//         {
//             session->debug_pos(keypos->pos);
//           if (object->ele_type == 6) {
//               double theta = alg::calc_theta(object->dir, keypos->dir, true);
//               if (theta < FLAGS_bind_trail_pos_match_stopline_theta_thres) {
//                   continue;
//               }
//           }
//           {
//             object_roadsegment_dis_map[keypos->road_segment].push_back(keypos);
//           }  
//         }
//         for (auto iter = object_roadsegment_dis_map.begin(); iter != object_roadsegment_dis_map.end(); iter++)
//         {
//             if(iter->second.size() == 1)
//             {
//                 iter->second.at(0)->object_list.push_back(object.get());
//             }
//             else
//             {   double mindis = DBL_MAX;
//                 int minIndex = 0;
//                 for (int i = 0; i < iter->second.size(); i++)
//                 {
//                     double temp_dis = alg::calc_dis(object->pos, iter->second.at(i)->pos, true);
//                     if(temp_dis < mindis)
//                     {
//                         mindis = temp_dis;
//                         minIndex = i;
//                     }
//                 }
//                 iter->second.at(minIndex)->object_list.push_back(object.get());
//             }
//         }  
//     }

//     // double turn_theta_threshold = FLAGS_bind_trail_turn_theta_threshold;
//     // for (auto &road_segment : session->road_segment_list) {
//     //     for (auto &poss : road_segment->pos_sample_list) {
//     //         session->thread_pool->schedule([turn_theta_threshold, road_segment, poss, session, this]() {
//     //                 session->debug_pos(poss->pos);
//     //                 std::vector<RoadObjectInfo*> secs;
//     //                 search_nearest_crosswalk(session, poss, secs);
//     //                 for (auto &fls : secs) {
//     //                     session->debug_pos2(poss->pos, fls->pos);
//     //                     auto new_fls = session->add_ptr(session->object_ptr, true);
//     //                     new_fls->init(fls);
//     //                     new_fls->key_pose = poss;
//     //                     new_fls->bind_dis = alg::calc_vertical_dis(
//     //                             fls->pos, poss->pos, poss->dir, true);
//     //                     poss->crosswalk_list.push_back(new_fls.get());
//     //                 }
//     //         });
//     //     }
//     // }
//     // session->thread_pool->wait();
//     return fsdmap::SUCC;
// }


int RoadModelProcBindTrail::search_nearest_crosswalk(RoadModelSessionData* session,
        KeyPose* poss, std::vector<RoadObjectInfo*> &secs) {
    float radius = FLAGS_bind_trail_pos_match_crosswalk_radius;
    double h_scope = FLAGS_bind_trail_pos_match_crosswalk_h_scope;
    double v_scope = FLAGS_bind_trail_pos_match_crosswalk_v_scope;
    Eigen::Vector3d pos = poss->pos;
    std::vector<RoadObjectInfo*> search_rets;
    session->object_tree.search(pos, radius, search_rets);
    for (auto& fp : search_rets) {
        // if (fp->type != 10) {
        //     continue;
        // }
        double h_dis = alg::calc_hori_dis(pos, fp->pos, fp->dir);
        if (h_dis > fp->width / 2 + h_scope) {
            continue;
        }
        double v_dis = alg::calc_vertical_dis(pos, fp->pos, fp->dir);
        if (h_dis > fp->length / 2 + v_scope) {
            continue;
        }
        // if (!alg::point_in_polygon(pos, fp->list)) {
        //     continue;
        // }
        secs.push_back(fp);
    }
    return fsdmap::SUCC;
}


int RoadModelProcBindTrail::search_nearest_fls(RoadModelSessionData* session,
        KeyPose* poss, std::vector<LaneLineSample*> &secs) {
    double radius = FLAGS_bind_trail_pos_match_lane_radius;
    double z_max = FLAGS_bind_trail_pos_match_lane_z_max;
    double z_min = FLAGS_bind_trail_pos_match_lane_z_min;
    double theta_threshold = FLAGS_bind_trail_pos_match_lane_theta;
    Eigen::Vector3d pos = poss->pos;
    std::vector<LaneLineSample*> search_rets;
    session->lane_line_sample_tree.search(pos, radius, search_rets);
    std::vector<std::pair<LaneLineSample*, double>> tmp_ret;
    auto &boundary = poss->boundary;
    Eigen::Vector3d poss_v_pt = alg::get_vertical_pos(poss->pos, poss->dir, 2, true);
    DisDirPoint cross_point;
    for (auto& fls : search_rets) {
        session->debug_pos2(poss->pos, fls->pos);
        if (fls->invalid()) {
            continue;
        }
        if (fls->next == NULL) {
            continue;
        }
        double dis = alg::calc_dis(pos, fls->pos, true);
        if (dis > radius) {
            continue;
        }
        double z_dis = pos.z() - fls->pos.z();
        // if (z_dis > z_max || z_dis < z_min) {
        //     continue;
        // }
        double theta = alg::calc_theta(fls->dir, poss->dir, true);
        if (theta >= theta_threshold) {
            continue;
        }
        // alg::calc_dir(fls->next->pos, fls->pos, cross_point.dir);
        // if (!alg::get_cross_point_for_segment(fls->pos, fls->next->pos,
        //             poss->pos, poss_v_pt, cross_point.pos, 1)) {
        //     continue;
        // }
        // double v_dis = alg::get_max_vertical_dis(fls->pos, fls->dir, poss->pos, poss->dir, true);
        //int is_left = alg::judge_left(fls->pos, poss->pos, poss->dir);
        //if (is_left < 0 && boundary.has_dis(true) && v_dis > boundary.get_dis(true, 2)) {
        //    continue;
        //} else if (is_left > 0 && boundary.has_dis(false) && v_dis > boundary.get_dis(false)) {
        //    continue;
        //}
        secs.push_back(fls);
    }
    return fsdmap::SUCC;
}

// 选出poss附近30m以内的车道中心线点。这些点与自车该位置处方向向量夹角在30度以内
int RoadModelProcBindTrail::search_nearest_lc(RoadModelSessionData* session,
        KeyPose* poss, std::vector<LaneCenterFeature*> &secs,
        std::vector<LaneCenterFeature*> &op_secs) {
    double radius = FLAGS_bind_trail_pos_match_lane_radius;  // 30
    double z_max = FLAGS_bind_trail_pos_match_lane_z_max;  //6
    double z_min = FLAGS_bind_trail_pos_match_lane_z_min;  //-2
    double theta_threshold = FLAGS_bind_trail_pos_match_lane_theta;  //30
    Eigen::Vector3d pos = poss->pos;
    std::vector<LaneCenterFeature*> search_rets;
    session->lane_center_gen_tree.search(pos, radius, search_rets);  // 30m以内的车道中心线点
    std::vector<std::pair<LaneCenterFeature*, double>> tmp_ret;
    DisDirPoint cross_point;
    auto &boundary = poss->boundary;
    for (auto& lc : search_rets) {
        session->debug_pos2(poss->pos, lc->pos);
        if (lc->src_status != 3) {
            continue;
        }
        if (lc->context.all_next.size() == 0) {
            continue;
        }
        // if (lc->invalid()) {
        //     continue;
        // }
        auto &center = *lc;
        double dis = alg::calc_dis(pos, center.pos, true);
        if (dis > radius) {
            continue;
        }
        double z_dis = pos.z() - center.pos.z();
        //if (z_dis > z_max || z_dis < z_min) {
        //    continue;
        //}
        double theta = alg::calc_theta(center.dir, poss->dir, true);
        if (theta > theta_threshold) {
            continue;
        }
        
        secs.push_back(lc);
    }
    return fsdmap::SUCC;
}


// int RoadModelProcBindTrail::search_nearest_pos(RoadModelSessionData* session,
//         LaneCenterFeature* lc, std::vector<KeyPose*> &secs) {
//     float radius = FLAGS_bind_trail_pos_match_lane_radius;
//     float theta_threshold = FLAGS_bind_trail_pos_match_lane_theta;
//     double max_z = FLAGS_bind_trail_pos_match_lane_z_max;
//     double min_z = FLAGS_bind_trail_pos_match_lane_z_min;
//     double scope = FLAGS_bind_trail_pos_match_scope;
//     auto &center = lc->ls->center;
//     Eigen::Vector3d pos = center.pos;
//     float minb[3] = {pos.x() - radius, pos.y() - radius, pos.z() + min_z};
//     float maxb[3] = {pos.x() + radius, pos.y() + radius, pos.z() + max_z};
//     std::vector<KeyPose*> search_rets;
//     session->pos_sample_tree.Search(minb, maxb, SearchResultCallback<KeyPose>, &search_rets);
//     for (auto& pos_get : search_rets) {
//         if (pos_get->segment == NULL) {
//             continue;
//         }
//         double dis = alg::calc_dis(pos, pos_get->pos, true);
//         if (dis > radius) {
//             continue;
//         }
//         double theta = alg::calc_theta(center.dir, pos_get->pos->dir);
//         if (theta >= theta_threshold) {
//             continue;
//         }
//         double v_dis = alg::calc_hori_dis(center.pos, pos_get->pos, pos_get->pos->dir, true);
//         if (v_dis > scope) {
//             continue;
//         }
//         secs.push_back(pos_get);
//     }
//     return fsdmap::SUCC;
// }
// 
// KeyPose* RoadModelProcBindTrail::get_lane_segment(RoadModelSessionData* session,
//         LaneCenterFeature* lc, std::vector<KeyPose*> &secs) {
//     bool has_segment = false;
//     double in_count = 0;
//     int out_count = 0;
//     auto &center = lc->ls->center;
//     std::vector<std::pair<KeyPose*, PosCacheData> > get_poss_list;
//     for (auto &poss : secs) {
//         for (auto &other_poss : poss->other_pos) {
//             int is_left = alg::judge_left(center.pos, other_poss->pos, other_poss->dir);
//             if (is_left == 0) {
//                 in_count++;
//                 continue;
//             }
//             double dis = alg::get_max_vertical_dis(center.pos, center.dir, other_poss->pos, other_poss->dir, true);
//             auto &boundary = other_poss->pos->boundary;
//             double boundary_dis = -1;
//             if (is_left < 0) {
//                 if (boundary.left != NULL) {
//                     boundary_dis = boundary.left_dis;
//                 } else if (boundary.left_oppo_pos != NULL) {
//                     boundary_dis = boundary.left_pos_dis;
//                 }
//             } else {
//                 if (boundary.right != NULL) {
//                     boundary_dis = boundary.right_dis;
//                 } else if (boundary.right_oppo_pos != NULL) {
//                     boundary_dis = boundary.right_pos_dis;
//                 }
//             }
//             if (boundary_dis < 0) {
//                 // 无边界的不参与计数
//                 continue;
//             }
//             if (boundary_dis >= 0 && dis > boundary_dis) {
//                 out_count++;
//                 continue;
//             }
//             in_count++;
//         }
//         if (out_count == poss->other_pos.size()) {
//             continue;
//         }
//         PosCacheData data;
//         data.score = in_count / (out_count + 1);
//         data.vertical_dis = alg::get_max_vertical_dis(center.pos, center.dir, poss->pos, poss->dir, true);
//         get_poss_list.push_back(std::make_pair(poss, data));
//     }
//     if (get_poss_list.size() == 0) {
//         return NULL;
//     }
//     SORT(get_poss_list,
//             [](const std::pair<KeyPose*, PosCacheData> &l, std::pair<KeyPose*, PosCacheData> &r)->bool {
//                 if (l.second.score == r.second.score) {
//                     return l.second.vertical_dis < r.second.vertical_dis;
//                 }
//                 return l.second.score > r.second.score;
//             });
//     return get_poss_list.front().first;
// }


int RoadModelProcBindTrail::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_bind_trail_save_data_enable) {
        return fsdmap::SUCC;
    }
    double scope_buff = FLAGS_display_scope_buff;
    session->set_display_name("bind_trail");
    for (auto &road_segment : session->road_segment_list) {
        for (auto &key_pose : road_segment->pos_sample_list) {         
            auto tar_pos = alg::get_hori_pos(key_pose->pos, key_pose->dir, 1);
            for (auto &lc : key_pose->lane_center_list) {

                // if(lc->left)
                // {
                //     LOG_INFO("bind_traild color:{} type:{}",lc->left->attr.color,lc->left->attr.type)
                // }
                // if(lc->right)
                // {
                //     LOG_INFO("bind_traild color:{} type:{}",lc->right->attr.color,lc->left->attr.type)
                // }

                auto log = session->add_debug_log(utils::DisplayInfo::LINE, "bind_lane_line");
                log->color = {255, 255, 255};
                if(key_pose->filter_status == 1 ){
                    log->color = {255, 0, 0};
                }
                else if(key_pose->filter_status == 2){
                    log->color = {0, 255, 0};
                }
                else if(key_pose->filter_status == 4 ){
                    log->color = {0, 0, 255};
                }
                else if(key_pose->filter_status > 4 ){
                    log->color = {255, 125, 0};
                }
                log->add(tar_pos, 2);
                auto &ele = log->add(lc->pos, 1);
                ele.label.label = lc->src_status;
            }
        }
    }
    for (auto &road_segment : session->road_segment_list) {
        for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            for (int j = 0; j < poss->lane_center_list.size(); ++j) {
                auto &lc = poss->lane_center_list[j];
                double h_dis = alg::calc_hori_dis(lc->pos, poss->pos, poss->dir);
                if (h_dis < 0.1) {
                    continue;
                }
                DLOG_POINT2(lc->pos, poss->pos, "debug bind h_dis[{},{}={}", i, j, h_dis);
            }
        }
    }
    session->save_debug_info("bind_trail");

    session->set_display_name("bind_obj");
    for (auto &road_segment : session->road_segment_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "road_segment");
        log->color = {255, 0, 0};
        for (auto &key_pose : road_segment->pos_sample_list) {
            if(key_pose->lane_center_list.size() == 0){
                log->color = {255, 255, 255};
            }
            log->add(key_pose->pos, 10);  
        }
    }
    

    for (auto &road_segment : session->road_segment_list) {
        for (auto &key_pose : road_segment->pos_sample_list) {
            for (auto &object : key_pose->object_list) {
                auto log = session->add_debug_log(utils::DisplayInfo::LINE, "bind_object");
                log->add(key_pose->pos, 10);
                log->add(object->pos, 10);
                log->color = {0, 255, 0};
                double dis = alg::calc_dis(key_pose->pos, object->pos, true);
                DLOG_POINT2(key_pose->pos, object->pos, "debug bind obj_dis={}", dis);
            }
        }
    }

    //停止线位置
    for (auto& object : session->raw_object_ret_list) {
        if(object->ele_type == 6){
            auto log2 = session->add_debug_log(utils::DisplayInfo::LINE, "stopline");
            log2->color = {255, 0, 255};
            for(auto& pt : object->list){
                log2->add(pt->pos, 10);
            }
        }
    }


    session->save_debug_info("bind_obj");

    return fsdmap::SUCC;
}

}
}
