

#include "road_model_proc_identify_attr.h"

namespace fsdmap {
namespace road_model {

DEFINE_bool(identify_attr_enable, true, "identify_attr_enable");
DEFINE_bool(identify_attr_debug_pos_enable, true, "identify_attr_debug_enable");
DEFINE_bool(identify_attr_save_data_enable, true, "identify_attr_save_data_enable");

DEFINE_double(identify_attr_vote_yellow_min_threshold, 2.5, "identify_attr_vote_yellow_min_threshold");
DEFINE_double(identify_attr_vote_yellow_max_threshold, 3.5, "identify_attr_vote_yellow_max_threshold");
DEFINE_double(identify_attr_vote_double_value_threshold, 0.7, "identify_attr_vote_double_value_threshold;");
DEFINE_double(identify_attr_vote_yellow_radius, 5, "identify_attr_vote_yellow_radius");
DEFINE_double(identify_attr_vote_yellow_dis_treshold, 1, "identify_attr_vote_yellow_dis_treshold");
DEFINE_double(identify_attr_vote_yellow_scope, 3, "identify_attr_vote_yellow_scope");
DEFINE_double(identify_attr_vote_yellow_dis_lambda, 0.5, "identify_attr_vote_yellow_dis_lambda");
DEFINE_double(identify_attr_vote_yellow_theta_treshold, 30, "identify_attr_vote_yellow_theta_treshold");


fsdmap::process_frame::PROC_STATUS RoadModelProcIdentifyAttr::proc(
        RoadModelSessionData* session) {
    session->enable_debug_pos = FLAGS_identify_attr_debug_pos_enable;
    if (!FLAGS_identify_attr_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }

    // 对于每个轨迹点，生成当前轨迹点的车道数据量，并找出变化点
    CHECK_FATAL_PROC(sync_feature_attr(session), "sync_feature_attr");
    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcIdentifyAttr::sync_feature_attr(RoadModelSessionData* session) {
    // for (auto &line : session->lane_line_sample_list_opt) {
    for (auto lc : session->lane_center_feature_merge_raw) {
        if (lc->left != NULL) { // 有左侧车道线, 判断左侧车道线的颜色以及是否双线、geo
            auto fls = lc->left;
            session->thread_pool->schedule([fls, session, this](utils::ProcessBar *process_bar) {
                    session->debug_pos(fls->pos);
                    int size = vote_attr_by_fls(session, fls);
                    process_bar->num_biz += size;
                    });
        }

        if (lc->right != NULL) { // 有右侧车道线, 判断右侧车道线的颜色以及是否双线、geo
            auto fls = lc->right;
            session->thread_pool->schedule([fls, session, this](utils::ProcessBar *process_bar) {
                    session->debug_pos(fls->pos);
                    int size = vote_attr_by_fls(session, fls);
                    process_bar->num_biz += size;
                    });
        }
    }
    //车道线属性投票
    // for (auto &line : session->boundary_line_sample_list_opt) {
    //     for (auto &fls : line->list) {
    //         session->thread_pool->schedule([fls, session, this](utils::ProcessBar *process_bar) {
    //                 session->debug_pos(fls->pos);
    //                 int size = vote_type_by_fls(session, fls); // 投票确定boundary类型
    //                 process_bar->num_biz += size;
    //                 });
    //     }
    // }
    session->thread_pool->wait(1, "process_attr_vote");
    return fsdmap::SUCC;
}

// 投票确定boundary类型
int64_t RoadModelProcIdentifyAttr::vote_type_by_fls(RoadModelSessionData* session,
        BoundaryFeature *base_fls) {
    double radius = FLAGS_identify_attr_vote_yellow_radius;  // 5
    double scope = FLAGS_identify_attr_vote_yellow_scope;  // 3
    double theta_threshold = FLAGS_identify_attr_vote_yellow_theta_treshold;  // 30
    double dis_threshold = FLAGS_identify_attr_vote_yellow_dis_treshold;  // 1
    auto &pos = base_fls->pos;
    std::vector<BoundaryFeature*> search_sec;
    session->boundary_line_sample_tree.search(pos, radius, search_sec);
    std::vector<BoundaryFeature*> attr_data_list;
    int64_t base_num = 0;
    UMAP<std::string, std::pair<BoundaryFeature*, double>> data_map;
    for (auto &fls : search_sec) {
        if (fls->src_status != 1) {
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
        if (MAP_NOT_FIND(data_map, fls->line_id)) {
            data_map[fls->line_id] = std::make_pair(fls, h_dis);
        } else if (data_map[fls->line_id].second > h_dis) {
            data_map[fls->line_id] = std::make_pair(fls, h_dis);
        }
    }
    for (auto &tit : data_map) {
        attr_data_list.push_back(tit.second.first);
    }
    int64_t num_type = vote_type(session, base_fls, attr_data_list); // 投票确定boundary类型
    return base_num;
}

// 投票确定boundary类型
int64_t RoadModelProcIdentifyAttr::vote_type(RoadModelSessionData* session,
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

// 根据计算base_fls附近的车道线点颜色判断base_fls的颜色以及是否双线、geo
int64_t RoadModelProcIdentifyAttr::vote_attr_by_fls(RoadModelSessionData* session,
        LaneLineSample *base_fls) {
    double radius = FLAGS_identify_attr_vote_yellow_radius;  // 5
    double scope = FLAGS_identify_attr_vote_yellow_scope;  // 3
    double theta_threshold = FLAGS_identify_attr_vote_yellow_theta_treshold;  // 30
    double dis_threshold = FLAGS_identify_attr_vote_yellow_dis_treshold;  // 1
    auto &pos = base_fls->pos;
    std::vector<LaneLineSample*> search_sec; // 5m范围内的车道线
    session->lane_line_sample_tree.search(pos, radius, search_sec);
    std::vector<LaneLineSample*> attr_data_list;
    int64_t base_num = 0;
    // 不同车道线上，与base_fls行驶方向上距离最短满足侧向距离小于1m的车道线点，
    // <车道线id, <与base_fls行驶方向山距离最短满足侧向距离小于1m的车道线点， 行驶方向山最短距离>>
    UMAP<std::string, std::pair<LaneLineSample*, double>> data_map;
    for (auto &fls : search_sec) {
        // if (fls->src->src_status != 1 && fls->src->src_status != 4 && fls->src->src_status != 5) {
        //     continue;
        // }
        double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir); //车道线点到车道线点base_fls方向向量的距离
        if (v_dis > dis_threshold) {
            continue;
        }
        double h_dis = alg::calc_hori_dis(fls->pos, base_fls->pos, base_fls->dir);  // 行驶方向的距离
        if (h_dis > scope) {
            continue;
        }
        double theta = alg::calc_theta(fls->dir, base_fls->dir, true);
        if (theta > theta_threshold) {
            continue;
        }
        ++base_num;
        // if (MAP_NOT_FIND(data_map, fls->src->line_id)) {
        //     data_map[fls->src->line_id] = std::make_pair(fls, h_dis);
        // } else if (data_map[fls->src->line_id].second > h_dis) {  // 选出行驶方向山最短距离的车道线点
        //     data_map[fls->src->line_id] = std::make_pair(fls, h_dis);
        // }
        attr_data_list.push_back(fls);
    }
    // for (auto &tit : data_map) {
    //     attr_data_list.push_back(tit.second.first);
    // }
    int64_t num_yellow = vote_yellow(session, base_fls, attr_data_list); // 判断base_fls车道线的颜色
    // int64_t num_double = vote_double(session, base_fls, attr_data_list);
    // int64_t num_geo = vote_geo(session, base_fls, attr_data_list);
    // LOG_INFO("num_yellow:{} search_sec:{} attr_data_list:{}",num_yellow,search_sec.size(),attr_data_list.size());
    return base_num;
}

// 根据计算base_fls附近的车道线点颜色判断base_fls的颜色
int64_t RoadModelProcIdentifyAttr::vote_yellow(RoadModelSessionData* session,
        LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list) {
    double dis_lambda = FLAGS_identify_attr_vote_yellow_dis_lambda;  // 0.5
    double min_thres = FLAGS_identify_attr_vote_yellow_min_threshold; // 1.5
    double max_thres = FLAGS_identify_attr_vote_yellow_max_threshold; // 2.5
    std::vector<std::pair<double, int>> data_list;

    double total_score = 0;
    double total_value = 0;
    int64_t valid_num = 0;
    std::map<int ,double > vote_map;
    for(auto p:attr_data_list)
    {
        double v_dis = alg::calc_vertical_dis(p->pos, base_fls->pos, base_fls->dir);
        double score = dis_lambda / (dis_lambda + v_dis);  // v_dis越小，score越大
        auto &vote=vote_map[p->attr.color];
        // LOG_INFO("vote_yellow COLOR:{}",p->attr.color);
        vote+=score;
    }
    int color=1;
    double max_score=-1;
    for(auto p:vote_map)
    {
        if(p.second>=max_score)
        {
         color=p.first;
         max_score=p.second;
        }
    }
    base_fls->attr.color =color;

    // for (auto &fls : attr_data_list) {
    //     // if (fls->src->src_status != 1) {
    //     //     continue;
    //     // }
    //     ++valid_num;
    //     double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir);
    //     double score = dis_lambda / (dis_lambda + v_dis);  // v_dis越小，score越大
    //     total_value += score * fls->src->attr.color;
    //     total_score += score;
    // }
    // if (valid_num <= 0) {
    //     return valid_num;
    // }
    // double value = total_value / total_score;
    // // if (value > min_thres && value < max_thres) {
    // //     base_fls->attr.color = 2;  // 黄色线
    // // } else if (value <= min_thres){
    // //     base_fls->attr.color = 1;  // 白线
    // // } else {
    // //     base_fls->attr.color = (int) (value + 0.5);
    // // }

    // if (value > min_thres && value < max_thres) {
    //     base_fls->attr.color = 2;  //   白线
    // } else if (value > max_thres){
    //     base_fls->attr.color = 3;  //   黄色线
    // } else {
    //     base_fls->attr.color = 1;  //   未知
    // }

    return valid_num;
}

// 根据计算base_fls附近的车道线点颜色判断base_fls是否是双线
int64_t RoadModelProcIdentifyAttr::vote_double(RoadModelSessionData* session,
        LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list) {
    double dis_lambda = FLAGS_identify_attr_vote_yellow_dis_lambda;  // 0.5
    double value_threshold = FLAGS_identify_attr_vote_double_value_threshold;  // 0.7
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
        double score = dis_lambda / (dis_lambda + v_dis);
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

int64_t RoadModelProcIdentifyAttr::vote_geo(RoadModelSessionData* session,
        LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list) {
    double dis_lambda = FLAGS_identify_attr_vote_yellow_dis_lambda;  // 0.5
    // double value_threshold = FLAGS_identify_attr_vote_geo_value_threshold;
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
        double score = dis_lambda / (dis_lambda + v_dis);
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


int RoadModelProcIdentifyAttr::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_identify_attr_save_data_enable) {
        return fsdmap::SUCC;
    }
    session->set_display_name("identify_attr");
    // LOG_INFO("lane_center_feature_merge_raw:{}",session->lane_center_feature_merge_raw.size())
    int i=0;
    auto log = session->add_debug_log(utils::DisplayInfo::POINT, "attr_lane_line");
    log->color = {1, 1, 1};
    for (auto lc : session->lane_center_feature_merge_raw)
    {
        if(lc->left)
        {
            int test=0;
            auto pt=lc->left;
            auto &ele = log->add(pt->pos);
            ele.label.cloud_pano_seg=pt->attr.type;
            ele.label.cloud_line_seg=pt->attr.color;
            switch(pt->attr.color)
            {
               case 2:
               {
                ele.color={255, 255, 255};
                break;
               }
               case 3:
               {
                ele.color={255, 140, 0};
                break;
               }
               default:
               {
                ele.color={0, 255, 0};
               }
            }
        }


        if(lc->right)
        {  
            int test=0;
            auto pt=lc->right;
            auto &ele = log->add(pt->pos);
            ele.label.cloud_pano_seg=pt->attr.type;
            ele.label.cloud_line_seg=pt->attr.color;
            switch(pt->attr.color)
            {
               case 2:  //白色
               {
                ele.color={255, 255, 255};
                break;
               }
               case 3: //黄
               {
                ele.color={255, 140, 0};
                break;
               }
               default:
               {
                ele.color={0, 255, 0};
               }
            }
            // LOG_INFO("identify color:{} type:{}",lc->right->attr.color,lc->right->attr.type)
        } 
    }
    session->save_debug_info("identify_attr");

    return fsdmap::SUCC;
}

}
}

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
