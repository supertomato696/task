

#include "road_model_proc_format_new_road.h"

DEFINE_bool(format_new_road_enable, true, "format_new_road_enable");
DEFINE_bool(format_new_road_debug_pos_enable, true, "format_new_road_debug_pos_enable");
DEFINE_bool(format_new_road_filter_green_light, true, "format_new_road_filter_green_light");
DEFINE_bool(format_new_road_save_data_enable, true, "format_new_road_save_data_enable");
DEFINE_bool(format_new_road_object_judgeDis_enable, false, "format_new_road_object_judgedis_enable");
DEFINE_bool(format_new_road_other_object_boundary, true, "format_new_road_other_object_boundary");
DEFINE_bool(format_new_road_stop_line_use_edge_enable, false, "format_new_road_stop_line_use_edge_enable");

DEFINE_double(format_new_road_max_same_dir_feature_search_radius, 20, "format_new_road_max_same_dir_feature_search_radius    ");
DEFINE_double(format_new_road_max_same_dir_feature_theta, 45, "format_new_road_max_same_dir_feature_theta");
DEFINE_double(format_new_road_max_same_dir_vertical_distance, 50, "format_new_road_max_same_dir_vertical_distance");
DEFINE_double(format_new_road_max_same_dir_length_lambda, 1, "format_new_road_max_same_dir_length_lambda");
DEFINE_double(format_new_road_tree_line_gap, 2, "format_new_road_tree_line_gap");
DEFINE_double(format_new_road_max_line_point_length, 50, "format_new_road_max_line_point_length");
DEFINE_string(format_new_road_traffic_sign_define_json_file, "./conf/traffic_sign_type_define.json", "traffic_sign_define_json_file");
DEFINE_string(format_new_road_obj_output_file, "../../data/", "obj_output_file");

DEFINE_double(format_new_road_high_curvature_gap, 3, "format_new_road_high_curvature_gap");
DEFINE_double(format_new_road_low_curvature_gap, 18, "format_new_road_low_curvature_gap");
DEFINE_double(format_new_road_match_groud_radius, 3, "format_new_road_match_groud_radius");
DEFINE_double(format_new_road_double_move_dis, 0.14, "format_new_road_double_move_dis");

DECLARE_double(format_lane_max_radius_threshold);
DECLARE_double(build_boundary_reline_boudary_by_pos_max_null_length);
DECLARE_double(build_boundary_same_lane_dis_threshold);
DECLARE_double(build_boundary_same_lane_theta_threshold);

namespace fsdmap
{
    namespace road_model
    {

        fsdmap::process_frame::PROC_STATUS RoadModelProcFormatNewRoad::proc(
            RoadModelSessionData *session)
        {
            session->enable_debug_pos = FLAGS_format_new_road_debug_pos_enable;
            if (!FLAGS_format_new_road_enable)
            {
                return fsdmap::process_frame::PROC_STATUS_DISABLE;
            }
            // 格式化护栏
            CHECK_FATAL_PROC(format_boundary(session), "format_boundary");

            // 格式化obj
            // CHECK_FATAL_PROC(format_obj(session), "format_obj");

            // 格式化路网
            CHECK_FATAL_PROC(format_lane_group(session), "format_lane_group");

            CHECK_FATAL_PROC(build_lane_relation(session, true), "build_lane_relation");

            CHECK_FATAL_PROC(build_lane_relation(session, false), "build_lane_relation");

            // z 处理
            // CHECK_FATAL_PROC(match_ground(session), "match_ground");
            CHECK_FATAL_PROC( check_connect(session),"check_connect");
            // 可视化结果
            CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
            return fsdmap::process_frame::PROC_STATUS_SUCC;
        }

        int RoadModelProcFormatNewRoad::match_ground(RoadModelSessionData *session)
        {
            if (session->opt_ground_pcd)
            {
                // for (auto &pt : session->opt_ground_pcd->points) {
                //     pt.intensity = pt.z;
                //     pt.z = 0;
                // }
                pcl::KdTreeFLANN<utils::CloudPoint> kdtree;
                kdtree.setInputCloud(session->opt_ground_pcd);
                for (auto &lg : session->new_lane_groups)
                {
                    for (auto &lb : lg->lane_boundary_info)
                    {
                        int start_index = -1;
                        int end_index = -1;
                        int status = 0;
                        for (int k = 0; k < lb->line_point_info.size(); ++k)
                        {
                            auto &lp = lb->line_point_info[k];
                            session->debug_pos(lp->pos);
                            auto gp = get_ground_point(session, kdtree, lp->pos);
                            if (gp != NULL)
                            {
                                lp->pos.z() = gp->intensity;
                                lp->ground_status = 1;
                                if (status == 0)
                                {
                                    start_index = k;
                                }
                                else if (status == 1)
                                {
                                    end_index = k;
                                    status = 2;
                                }
                            }
                            else
                            {
                                status = 1;
                            }
                            if (status == 2 || (k == lb->line_point_info.size() - 1 && status == 1))
                            {
                                if (start_index == -1 && end_index == -1)
                                {
                                    continue;
                                }
                                int tmp_s = start_index == -1 ? 0 : start_index + 1;
                                int tmp_e = end_index == -1 ? lb->line_point_info.size() : end_index;
                                for (int j = tmp_s; j < tmp_e; ++j)
                                {
                                    auto &lp = lb->line_point_info[j];
                                    session->debug_pos(lp->pos);
                                    if (start_index == -1)
                                    {
                                        auto &e_lp = lb->line_point_info[end_index];
                                        lp->pos.z() = e_lp->pos.z();
                                    }
                                    else if (end_index == -1)
                                    {
                                        auto &s_lp = lb->line_point_info[start_index];
                                        lp->pos.z() = s_lp->pos.z();
                                    }
                                    else if (start_index != -1 && end_index != -1)
                                    {
                                        auto &s_lp = lb->line_point_info[start_index];
                                        auto &e_lp = lb->line_point_info[end_index];
                                        double s_dis = alg::calc_dis(lp->pos, s_lp->pos);
                                        double e_dis = alg::calc_dis(lp->pos, e_lp->pos);
                                        double rate = s_dis / (s_dis + e_dis);
                                        double z = rate * (e_lp->pos.z() - s_lp->pos.z()) + s_lp->pos.z();
                                        lp->pos.z() = z;
                                    }
                                }
                                status = 0;
                                if (end_index != -1)
                                {
                                    start_index = end_index;
                                }
                            }
                        }
                    }
                }

                for (auto &bs : session->new_boundary_segment)
                {
                    RoadBoundaryPointInfo *prev_bp = NULL;
                    int start_index = -1;
                    int end_index = -1;
                    int status = 0;
                    for (int k = 0; k < bs->point_info.size(); ++k)
                    {
                        auto &bp = bs->point_info[k];
                        session->debug_pos(bp->pos);
                        auto gp = get_ground_point(session, kdtree, bp->pos);
                        if (gp != NULL)
                        {
                            bp->pos.z() = gp->intensity;
                            bp->ground_status = 1;
                            if (status == 0)
                            {
                                start_index = k;
                            }
                            else if (status == 1)
                            {
                                end_index = k;
                                status = 2;
                            }
                        }
                        else
                        {
                            status = 1;
                        }
                        if (status == 2 || (k == bs->point_info.size() - 1 && status == 1))
                        {
                            if (start_index == -1 && end_index == -1)
                            {
                                continue;
                            }
                            int tmp_s = start_index == -1 ? 0 : start_index + 1;
                            int tmp_e = end_index == -1 ? bs->point_info.size() : end_index;
                            for (int j = tmp_s; j < tmp_e; ++j)
                            {
                                auto &bp = bs->point_info[j];
                                session->debug_pos(bp->pos);
                                if (start_index == -1)
                                {
                                    auto &e_bp = bs->point_info[end_index];
                                    bp->pos.z() = e_bp->pos.z();
                                }
                                else if (end_index == -1)
                                {
                                    auto &s_bp = bs->point_info[start_index];
                                    bp->pos.z() = s_bp->pos.z();
                                }
                                else if (start_index != -1 && end_index != -1)
                                {
                                    auto &s_bp = bs->point_info[start_index];
                                    auto &e_bp = bs->point_info[end_index];
                                    double s_dis = alg::calc_dis(bp->pos, s_bp->pos);
                                    double e_dis = alg::calc_dis(bp->pos, e_bp->pos);
                                    double rate = s_dis / (s_dis + e_dis);
                                    double z = rate * (e_bp->pos.z() - s_bp->pos.z()) + s_bp->pos.z();
                                    bp->pos.z() = z;
                                }
                            }
                            status = 0;
                            if (end_index != -1)
                            {
                                start_index = end_index;
                            }
                        }
                    }
                }
            }
            return fsdmap::SUCC;
        }

        utils::CloudPoint *RoadModelProcFormatNewRoad::get_ground_point(RoadModelSessionData *session,
                                                                        pcl::KdTreeFLANN<utils::CloudPoint> &kdtree, Eigen::Vector3d &pos)
        {
            double radius = FLAGS_format_new_road_match_groud_radius;
            utils::CloudPoint searchPoint;
            searchPoint.x = pos.x();
            searchPoint.y = pos.y();
            searchPoint.z = pos.z();
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            double min_dis = DBL_MAX;
            utils::CloudPoint *min_pt = NULL;
            if (kdtree.radiusSearch(searchPoint, radius,
                                    pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                for (int64_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                {
                    auto &pt = session->opt_ground_pcd->points[pointIdxRadiusSearch[i]];
                    double dis = pointRadiusSquaredDistance[i];
                    if (dis < min_dis)
                    {
                        min_dis = dis;
                        min_pt = &pt;
                    }
                }
            }
            return min_pt;
        }

        int RoadModelProcFormatNewRoad::build_lane_relation(RoadModelSessionData *session, bool next)
        {
            // 端点 同时存在两条线，刷之前要刷一边
            double scope = 3;

            for (auto &srs : session->sub_road_segment_list)
            {
                for (auto &lg : srs->new_lane_groups)
                {
                    for (auto &lane : lg->lane_line_info)
                    {
                        if (lane->lane_center_feature_list.size() == 0)
                        {
                            continue;
                        }
                        auto &lc = next ? lane->lane_center_feature_list.back()
                                        : lane->lane_center_feature_list.front();
                        lc->road_lane_line = lane;
                    }
                }
            }

            // std::vector<LaneCenterFeature*> prev_list;
            std::vector<LaneCenterFeature *> context_list;
            for (auto &srs : session->sub_road_segment_list)
            {
                for (auto &lg : srs->new_lane_groups)
                {
                    for (int i = 0; i < lg->lane_line_info.size(); ++i)
                    {
                        auto &lane = lg->lane_line_info[i];
                        if (lane->lane_center_feature_list.size() == 0)
                        {
                            continue;
                        }
                        auto &lc = next ? lane->lane_center_feature_list.back()
                                        : lane->lane_center_feature_list.front();
                        session->debug_pos(lc->pos);
                        context_list.clear();
                        int mode = next ? 1 : 2;
                        bool has_valid_next = false;
                        // TODO 
                        lc->context.get_context_list(scope, lc, context_list, [lane](LaneCenterFeature *curr, ParamPair<LaneCenterFeature> &next_pair)
                                                     {
                        auto &next = next_pair.src;
                        if (next->invalid()) {
                            return false;
                        }
                        if (next->road_lane_line == lane) {
                            return false;
                        }
                        if (curr->road_lane_line != NULL && curr->road_lane_line != lane) {
                            return false;
                        }
                            return true; }, mode);

                        for (int j = 0; j < context_list.size(); ++j)
                        {
                            int index = next ? j : context_list.size() - 1 - j;
                            auto &next_lc = context_list[index];
                            if (next_lc->road_lane_line == lane)
                            {
                                continue;
                            }
                            if (next_lc->road_lane_line == NULL || next_lc->road_lane_line->lane_group_info == NULL)
                            {
                                continue;
                            }
                            if (next_lc->road_lane_line->lane_group_info->lane_line_info.size() == 0)
                            {
                                continue;
                            }
                            double dis = alg::calc_dis(lc->pos, next_lc->pos);
                            if (dis > 2)
                            {
                                int a = 1;
                            }
                            if (lg->oppo_status == 1)
                            {
                                int a = 1;
                            }
                            if (next ^ (lg->oppo_status == 1))
                            {
                                lane->context.set_next(next_lc->road_lane_line);
                                next_lc->road_lane_line->context.set_prev(lane);
                                lg->context.set_next(next_lc->road_lane_line->lane_group_info);
                                next_lc->road_lane_line->lane_group_info->context.set_prev(lg);
                            }
                            else
                            {
                                lane->context.set_prev(next_lc->road_lane_line);
                                next_lc->road_lane_line->context.set_next(lane);
                                lg->context.set_prev(next_lc->road_lane_line->lane_group_info);
                                next_lc->road_lane_line->lane_group_info->context.set_next(lg);
                            }
                            has_valid_next = true;
                            // break;
                        }
                        if (!has_valid_next)
                        {
                            int a = 1;
                        }
                    }
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatNewRoad::format_lane_group(RoadModelSessionData *session)
        {
            // 初始化一个有效的道路索引集合，表示处理的道路（0表示正向道路，-1表示反向道路）
            std::vector<int> valid_road_index = {0, -1};
            // std::vector<int> valid_road_index = {0}; // 可替换为只处理正向道路
            int b_idx = 0, global_lane_idx = 0;

            // 遍历每个有效的道路索引（正向和反向）
            for (auto &rc_index : valid_road_index)
            {
                // 初始化道路格式信息
                RoadFormatInfo road_format_info;
                road_format_info.init();
                road_format_info.road_index = rc_index;

                RoadLaneGroupInfo *prev_lg = NULL; // 记录上一个车道组的指针

                // 遍历每个道路段（sub_road_segment_list）
                for (auto &srs : session->sub_road_segment_list)
                {
                    // 如果当前道路段的索引不匹配，跳过
                    if (srs->road_index != rc_index)
                    {
                        continue;
                    }

                    // 对于反向道路，判断是否需要左侧道路，若不需要则跳过
                    if (rc_index == -1 && !srs->need_left_road)
                    {
                        continue;
                    }

                    // 判断道路段的链接方向是否为1（正向方向）
                    if (srs->link_direction != 1)
                    {
                        continue;
                    }

                    // 为当前道路段分配新的车道组指针
                    auto lane_group = session->add_ptr(session->lane_group_info_ptr);

                    // 设置当前道路段和车道组的关联信息
                    road_format_info.road_segment = srs;
                    road_format_info.lane_group = lane_group.get();

                    // 设置车道组是否是反向的标志
                    if (rc_index == 0)
                    {
                        road_format_info.format_oppo = false; // 正向
                    }
                    else if (rc_index == -1)
                    {
                        road_format_info.format_oppo = true; // 反向
                    }

                    // 根据反向标志设置车道组的状态
                    lane_group->oppo_status = road_format_info.format_oppo ? 1 : 0;
                    lane_group->src_status = 2; // 设置来源状态为2

                    // 格式化车道线信息
                    int lane_num = format_lane_line(session, road_format_info, global_lane_idx);

                    // 如果车道组没有车道线信息，跳过
                    if (lane_group->lane_line_info.size() == 0)
                    {
                        continue;
                    }

                    // LOG_INFO("lane_group index is {}, lane_group lane_line_info size:{}",  lane_group->group_index, lane_group->lane_line_info.size());

                    lane_group->road_center_line_info = lane_group->lane_line_info[0]->left_lane_boundary_info;

                    // 如果有上一个车道组，则设置前后链接
                    if (prev_lg != NULL)
                    {
                        prev_lg->context.set_next(lane_group.get());
                        lane_group->context.set_prev(prev_lg);
                    }

                    // 将当前车道组添加到新的车道组列表和对应的道路段的车道组列表中
                    session->new_lane_groups.push_back(lane_group.get());
                    srs->new_lane_groups.push_back(lane_group.get());

                    // 遍历当前道路段的边界段信息，并根据边界状态添加到左右障碍物列表
                    for (int k = 0; k < srs->new_boundary_segment.size(); ++k)
                    {
                        auto &bl = srs->new_boundary_segment[k];
                        if (bl->side_status == 1)
                        {
                            bl->boundary_id = b_idx;
                            lane_group->right_barrier_segment_info.push_back(bl);
                        }
                        else
                        {
                            bl->boundary_id = b_idx;
                            lane_group->left_barrier_segment_info.push_back(bl);
                        }
                        b_idx++;
                    }
                }

                // 处理第二个方向的道路段（`link_direction == 2`）
                for (auto &srs : session->sub_road_segment_list)
                {
                    auto lane_group = session->add_ptr(session->lane_group_info_ptr);
                    road_format_info.road_segment = srs;
                    road_format_info.lane_group = lane_group.get();

                    // 如果道路段的索引不匹配，跳过
                    if (srs->road_index != rc_index)
                    {
                        continue;
                    }

                    // 对于反向道路，判断是否需要左侧道路，若不需要则跳过
                    if (rc_index == -1 && !srs->need_left_road)
                    {
                        continue;
                    }

                    // 判断道路段的链接方向是否为2（反向方向）
                    if (srs->link_direction != 2)
                    {
                        continue;
                    }

                    // 设置车道组是否是反向的标志
                    if (rc_index == 0)
                    {
                        road_format_info.format_oppo = true; // 正向的反向
                    }
                    else if (rc_index == -1)
                    {
                        road_format_info.format_oppo = false; // 反向的正向
                    }

                    // 设置车道组的状态
                    lane_group->oppo_status = road_format_info.format_oppo ? 1 : 0;
                    lane_group->src_status = 2; // 设置来源状态为2

                    // 调用 `format_lane_line` 来格式化车道线信息
                    int lane_num = format_lane_line(session, road_format_info, global_lane_idx);

                    // 如果车道组没有车道线信息，跳过
                    if (lane_group->lane_line_info.size() == 0)
                    {
                        continue;
                    }
                    // LOG_INFO("lane_group index is {}, lane_group lane_line_info size:{}",  lane_group->group_index, lane_group->lane_line_info.size());
                    lane_group->road_center_line_info = lane_group->lane_line_info[0]->left_lane_boundary_info;

                    // 将当前车道组添加到新的车道组列表和对应的道路段的车道组列表中
                    session->new_lane_groups.push_back(lane_group.get());
                    srs->new_lane_groups.push_back(lane_group.get());

                    // 遍历当前道路段的边界段信息，并根据边界状态添加到右侧障碍物列表
                    for (int k = 0; k < srs->new_boundary_segment.size(); ++k)
                    {
                        auto &bl = srs->new_boundary_segment[k];
                        if (bl->side_status == 2)
                        {
                            bl->boundary_id = b_idx;
                            lane_group->right_barrier_segment_info.push_back(bl);
                        }
                        b_idx++;
                    }
                }
            }

            // 移除没有车道线信息的车道组
            for (int i = 0; i < session->new_lane_groups.size(); ++i)
            {
                auto &lg = session->new_lane_groups[i];
                lg->group_index = i;
                if (lg->lane_line_info.size() == 0)
                {
                    VEC_ERASE(session->new_lane_groups, i); // 删除当前索引的车道组
                }
            }

            int idx = 0;
            for (auto &srs : session->sub_road_segment_list)
            {
                for (int i = 0; i < srs->new_lane_groups.size(); ++i)
                {
                    auto &lg = srs->new_lane_groups[i];
                    lg->group_index = i;
                    if (lg->lane_line_info.size() == 0)
                    {
                        VEC_ERASE(srs->new_lane_groups, i); // 删除当前索引的车道组
                    }
                    idx++;
                }
            }
            return fsdmap::SUCC;
        }

        // int RoadModelProcFormatNewRoad::bind_boundary_to_lane_group(RoadModelSessionData* session,
        //         RoadLaneGroupInfo* lane_group, RoadSegment* road_segment) {
        //     for (auto &barrier : road_segment->barrier_segment) {
        //         lane_group->barrier_segment_info.push_back(barrier);
        //         barrier->lane_group_info = lane_group;
        //     }
        //     for (auto &curb : road_segment->curb_segment) {
        //         lane_group->curb_segment_info.push_back(curb);
        //         curb->lane_group_info = lane_group;
        //     }
        //     return fsdmap::SUCC;
        // }

        int RoadModelProcFormatNewRoad::format_lane_line(
            RoadModelSessionData *session,
            RoadFormatInfo &road_format_info, int &g_lane_idx)
        {
            SubRoadSegment *srs = road_format_info.road_segment;
            RoadLaneGroupInfo *lane_group = road_format_info.lane_group;
            int32_t lane_line_index = 0;
            int ret = 0;
            bool need_generate_lane_line = false;
            std::vector<int> status_matrix = {
                0, 0, 1, 0,
                1, 1, 1, 0};

            RoadLaneBoundaryInfo *prev_lane_boundary = NULL, *prev_center_line = NULL;

            int lane_status = 0;
            for (int i = 0; i < srs->valid_lane_list.size(); ++i)
            {
                int index = road_format_info.format_oppo ? srs->valid_lane_list.size() - i - 1 : i;
                auto &lane_line = srs->valid_lane_list[index];
                if (lane_line->invalid())
                {
                    continue;
                }
                if (lane_line->road_index != road_format_info.road_index)
                {
                    continue;
                }
                // int is_left = lane_line->side_status == 2 ? 0 : 1;
                // int is_oppo = srs->road_index == -1 ? 1 : 0;
                // // int road_type = srs->src_road_segment->road_type[srs->road_index] == 2 ? 1 : 0;
                // int road_type = srs->road_type == 2 ? 1 : 0;
                // int position = is_oppo * 4 + road_type * 2 + is_left;
                // bool oppo = status_matrix[position] == 1;
                // // bool oppo = (lane_line->oppo_status == 2) ^ (road_format_info.road_index == -1);
                // if (road_format_info.format_oppo ^ oppo) {
                //     continue;
                // }
                road_format_info.lane_line = lane_line;
                int lp_num = 0;
                auto tar_lane = session->add_ptr(session->road_lane_line_ptr);
                tar_lane->side_status = lane_line->side_status;
                tar_lane->lane_group_info = lane_group;
                if (prev_lane_boundary != NULL)
                {
                    tar_lane->left_lane_boundary_info = prev_lane_boundary;
                }
                // road_format_info->lane_info = tar_lane.get();
                auto lane_center_line = gen_lane_center_line(session, road_format_info);
                if (lane_center_line == NULL)
                {
                    continue;
                }

                if (is_center_line_missed(session, prev_center_line, lane_center_line))
                {
                    need_generate_lane_line = true;
                }
                
                // 如果是基线，则额外生成一个lane_boundary
                if (lane_line_index == 0 || need_generate_lane_line)
                {
                    session->debug_pos(lane_line->list[0]->pos);
                    // road_format_info.lane_line_index = lane_line_index;
                    road_format_info.lane_line_index = g_lane_idx;
                    road_format_info.gen_left = !road_format_info.format_oppo;
                    auto &attr = road_format_info.gen_left ? lane_line->left_attr : lane_line->right_attr;
                    road_format_info.double_left = attr.is_double_line;
                    auto lb = gen_lane_boundary(session, road_format_info);
                    if (lb != NULL)
                    {
                        ++lane_line_index;
                        ++g_lane_idx;
                        tar_lane->left_lane_boundary_info = lb;
                    }
                    need_generate_lane_line = false;
                }
                tar_lane->center_lane_boundary_info = lane_center_line;
                prev_center_line = lane_center_line;
                // 这里不能倒叙，不然fill_next不对
                // LaneCenterFeature* prev_lc = NULL;
                for (int k = 0; k < lane_line->list.size(); ++k)
                {
                    // int lc_index = road_format_info.format_oppo ?
                    //     lane_line->list.size() - 1 - k : k;
                    auto &lc = lane_line->list[k];
                    session->debug_pos(lc->pos);
                    if (lc->invalid())
                    {
                        continue;
                    }
                    lc->road_lane_line = tar_lane.get();
                    tar_lane->lane_center_feature_list.push_back(lc);
                    // if (prev_lc != NULL) {
                    //     prev_lc->dir = alg::get_dir(lc->pos, prev_lc->pos);
                    //     lc->dir = prev_lc->dir;
                    // }
                    // prev_lc = lc;
                }

                lane_line_index++;
                road_format_info.lane_line_index = g_lane_idx++;
                road_format_info.gen_left = road_format_info.format_oppo;
                auto &attr = road_format_info.gen_left ? lane_line->left_attr : lane_line->right_attr;
                road_format_info.double_left = attr.is_double_line;
                if (attr.is_double_line == 1)
                {
                    auto lb = gen_lane_boundary(session, road_format_info);
                    if (lb != NULL)
                    {
                        if (tar_lane->left_lane_boundary_info != NULL)
                        {
                            ++lane_line_index;
                            ++g_lane_idx;
                            tar_lane->right_lane_boundary_info = lb;
                            lane_group->lane_line_info.push_back(tar_lane.get());
                            tar_lane = session->add_ptr(session->road_lane_line_ptr);
                            tar_lane->side_status = lane_line->side_status;
                            tar_lane->lane_group_info = lane_group;
                            tar_lane->left_lane_boundary_info = lb;
                        }
                        else
                        {
                            continue;
                        }
                    }
                }
                auto lb = gen_lane_boundary(session, road_format_info);
                if (lb != NULL)
                {
                    if (tar_lane->left_lane_boundary_info != NULL)
                    {
                        ++lane_line_index;
                        ++g_lane_idx;
                        tar_lane->right_lane_boundary_info = lb;
                        lane_group->lane_line_info.push_back(tar_lane.get());
                        prev_lane_boundary = lb;
                    }
                }
            }
            return lane_line_index;
        }

        RoadLaneBoundaryInfo *RoadModelProcFormatNewRoad::gen_lane_center_line(
            RoadModelSessionData *session,
            RoadFormatInfo &road_format_info)
        {
            RoadLaneGroupInfo *lane_group = road_format_info.lane_group;

            auto lane_line = road_format_info.lane_line;
            auto lane_boundary = session->add_ptr(session->lane_boundary_info_ptr);
            lane_boundary->src_status = 2;
            lane_boundary->lane_id = road_format_info.lane_line_index;
            road_format_info.lane_boundary = lane_boundary.get();
            int point_num = format_lane_center_point(session, road_format_info);
            if (point_num < 2)
            {
                return NULL;
            }
            lane_group->lane_boundary_info.push_back(lane_boundary.get());
            return lane_boundary.get();
        }

        int RoadModelProcFormatNewRoad::format_lane_center_point(
            RoadModelSessionData *session,
            RoadFormatInfo &road_format_info)
        {
            double high_curvature_gap = FLAGS_format_new_road_high_curvature_gap;
            double low_curvature_gap = FLAGS_format_new_road_low_curvature_gap;
            double max_radius_threshold = FLAGS_format_lane_max_radius_threshold;
            RoadLaneGroupInfo *lane_group = road_format_info.lane_group;
            LaneCenterLine *lane_line = road_format_info.lane_line;
            RoadLaneBoundaryInfo *lane_boundary = road_format_info.lane_boundary;
            std::vector<LaneCenterFeature *> data_list;
            LaneCenterFeature *prev_lc = NULL;
            double gen_gap = low_curvature_gap;
            int ret = 0;
            int valid_last = 0;
            int real_last = 0;
            for (int i = 0; i < lane_line->list.size(); ++i)
            {
                int index = road_format_info.format_oppo ? lane_line->list.size() - i - 1 : i;
                auto &lc = lane_line->list[index];
                session->debug_pos(lc->pos);
                if (lc->invalid())
                {
                    continue;
                }
                valid_last = index;
                if (fabs(lc->curvature) < max_radius_threshold)
                {
                    gen_gap = high_curvature_gap;
                }
                else
                {
                    gen_gap = low_curvature_gap;
                }
                if (prev_lc != NULL)
                {
                    prev_lc->dir = alg::get_dir(lc->pos, prev_lc->pos);
                    if (road_format_info.format_oppo)
                    {
                        prev_lc->dir = -prev_lc->dir;
                    }
                    lc->dir = prev_lc->dir;

                    double lc_dis = alg::calc_dis(prev_lc->pos, lc->pos);
                    if (lc_dis < gen_gap)
                    {
                        continue;
                    }
                }
                real_last = index;
                data_list.push_back(lc);
                prev_lc = lc;
            }
            if (valid_last != real_last)
            {
                auto &last_lc = lane_line->list[valid_last]->pos;
                double dis = alg::calc_dis(data_list.back()->pos, lane_line->list[valid_last]->pos);
                // bool is_front = alg::judge_front(last_lc->pos, data_list.back()->pos, data_list.back()->dir);
                // if (dis > 0.1 && (is_front ^ road_format_info.format_oppo)) {
                if (dis > 0.1)
                {
                    data_list.push_back(lane_line->list[valid_last]);
                }
            }
            int32_t group_index = 0;
            RoadLinePointInfo *prev_pt = NULL;
            for (int i = 0; i < data_list.size(); ++i)
            {
                auto &lc = data_list[i];
                road_format_info.lc = lc;
                road_format_info.lc = lc;
                auto line_point = session->add_ptr(session->line_point_info_ptr);
                // road_format_info.gen_left = road_format_info.format_oppo;
                road_format_info.line_point = line_point.get();
                line_point->pos = lc->pos;
                line_point->src_status = 2;
                lane_boundary->line_point_info.push_back(line_point.get());
                line_point->lane_boundary = lane_boundary;
                line_point->lane_group = lane_group;
                group_index++;
                if (prev_pt != NULL)
                {
                    prev_pt->dir = alg::get_dir(lc->pos, prev_pt->pos);
                    line_point->dir = prev_pt->dir;
                }
                prev_pt = line_point.get();
            }
            return group_index;
        }

        RoadLaneBoundaryInfo *RoadModelProcFormatNewRoad::gen_lane_boundary(
            RoadModelSessionData *session,
            RoadFormatInfo &road_format_info)
        {
            RoadLaneGroupInfo *lane_group = road_format_info.lane_group;

            auto lane_line = road_format_info.lane_line;
            auto lane_boundary = session->add_ptr(session->lane_boundary_info_ptr);
            lane_boundary->src_status = 2;
            lane_boundary->lane_id = road_format_info.lane_line_index;
            LaneAttr *attr = &(lane_line->right_attr);
            if (road_format_info.gen_left)
            {
                attr = &(lane_line->left_attr);
            }
            if (attr->geo >= 0)
            {
                lane_boundary->geo = attr->geo;
            }
            else
            {
                lane_boundary->geo = 2;
            }
            if (attr->color > 0)
            {
                lane_boundary->color = attr->color;
            }
            else
            {
                lane_boundary->color = 1;
            }

            if (attr->type >0)
            {
                lane_boundary->type = attr->type;
            }
            else
            {
                lane_boundary->type = 1;
            }

            road_format_info.lane_boundary = lane_boundary.get();
            int point_num = format_lane_line_point(session, road_format_info);
            if (point_num < 2)
            {
                return NULL;
            }
            lane_group->lane_boundary_info.push_back(lane_boundary.get());
            return lane_boundary.get();
        }

        int RoadModelProcFormatNewRoad::format_lane_line_point(
            RoadModelSessionData *session,
            RoadFormatInfo &road_format_info)
        {
            double high_curvature_gap = FLAGS_format_new_road_high_curvature_gap;
            double low_curvature_gap = FLAGS_format_new_road_low_curvature_gap;
            double max_radius_threshold = FLAGS_format_lane_max_radius_threshold;
            RoadLaneGroupInfo *lane_group = road_format_info.lane_group;
            LaneCenterLine *lane_line = road_format_info.lane_line;
            RoadLaneBoundaryInfo *lane_boundary = road_format_info.lane_boundary;
            std::vector<LaneCenterFeature *> data_list;
            LaneCenterFeature *prev_lc = NULL;
            double gen_gap = low_curvature_gap;
            int32_t group_index = 0;
            int ret = 0;
            int valid_last = 0;
            int real_last = 0;
            for (int i = 0; i < lane_line->list.size(); ++i)
            {
                int index = road_format_info.format_oppo ? lane_line->list.size() - i - 1 : i;
                auto &lc = lane_line->list[index];
                session->debug_pos(lc->pos);
                if (lc->invalid())
                {
                    continue;
                }
                valid_last = index;
                if (fabs(lc->curvature) < max_radius_threshold)
                {
                    gen_gap = high_curvature_gap;
                }
                else
                {
                    gen_gap = low_curvature_gap;
                }
                if (prev_lc != NULL)
                {
                    prev_lc->dir = alg::get_dir(lc->pos, prev_lc->pos);
                    if (road_format_info.format_oppo)
                    {
                        prev_lc->dir = -prev_lc->dir;
                    }
                    lc->dir = prev_lc->dir;

                    double lc_dis = alg::calc_dis(prev_lc->pos, lc->pos);
                    if (lc_dis < gen_gap)
                    {
                        continue;
                    }
                }
                real_last = index;
                data_list.push_back(lc);
                prev_lc = lc;
            }
            if (valid_last != real_last)
            {
                double dis = alg::calc_dis(data_list.back()->pos, lane_line->list[valid_last]->pos);
                if (dis > 0.1)
                {
                    data_list.push_back(lane_line->list[valid_last]);
                }
            }
            if (data_list.size() < 2)
            {
                return 0;
            }
            for (int i = 0; i < data_list.size(); ++i)
            {
                auto &lc = data_list[i];
                session->debug_pos(lc->pos);
                road_format_info.lc = lc;
                // double tmp_theta = alg::calc_theta(prev_lc->dir, lc->dir);
                // if (tmp_theta > 90) {
                //     FLOG_POINT(lc->pos, "dir error[theta={}, d1={},{}, d2={},{}]",
                //             tmp_theta, prev_lc->dir.x(), prev_lc->dir.y()
                //             , lc->dir.x(), lc->dir.y());
                // }
                if (road_format_info.lane_line_index == 0)
                {
                    // 基线处理逻辑
                    auto line_point = session->add_ptr(session->line_point_info_ptr);
                    // road_format_info.gen_left = !road_format_info.format_oppo;
                    road_format_info.line_point = line_point.get();
                    if (trans_feature_to_lane_line_point(session, road_format_info) != fsdmap::SUCC)
                    {
                        continue;
                    }
                    lane_boundary->line_point_info.push_back(line_point.get());
                    line_point->lane_boundary = lane_boundary;
                    line_point->lane_group = lane_group;
                    // session->all_line_point.push_back(line_point.get());
                }
                else
                {
                    // 转换右边的
                    auto line_point = session->add_ptr(session->line_point_info_ptr);
                    // road_format_info.gen_left = road_format_info.format_oppo;
                    road_format_info.line_point = line_point.get();
                    if (trans_feature_to_lane_line_point(session, road_format_info) != fsdmap::SUCC)
                    {
                        continue;
                    }
                    lane_boundary->line_point_info.push_back(line_point.get());
                    line_point->lane_boundary = lane_boundary;
                    line_point->lane_group = lane_group;
                    // session->all_line_point.push_back(line_point.get());
                }
                group_index++;
                prev_lc = lc;
            }
            return group_index;
        }

        int RoadModelProcFormatNewRoad::trans_feature_to_lane_line_point(
            RoadModelSessionData *session,
            RoadFormatInfo &road_format_info)
        {
            double max_length = FLAGS_format_new_road_max_line_point_length;
            double move_dis = FLAGS_format_new_road_double_move_dis;
            // 需要用前后的长度推导出中心点
            RoadLinePointInfo *line_point = road_format_info.line_point;
            LaneCenterFeature *lc = road_format_info.lc;
            // LaneCenterFeature* prev_lc = road_format_info.prev_lc;
            // 需要先转换坐标后再计算距离和角度
            // 计算当前点
            Eigen::Vector3d curr_pos = road_format_info.gen_left ? lc->get_left() : lc->get_right();
            session->debug_pos(curr_pos);
            // Eigen::Vector3d prev_pos = road_format_info.gen_left ?
            //     prev_lc->get_left() : prev_lc->get_right();
            auto v_dir = alg::get_vertical_dir(lc->dir);
            if (road_format_info.double_left == 1)
            {
                if (road_format_info.gen_left)
                {
                    curr_pos = curr_pos + move_dis * v_dir;
                }
                else
                {
                    curr_pos = curr_pos - move_dis * v_dir;
                }
            }
            auto fls = road_format_info.gen_left ? lc->left : lc->right;
            if (fls != NULL)
            {
                line_point->real_status = fls->real_status;
            }

            // session->format_line_point(prev_fp_pos_11, curr_pos, line_point, true);
            line_point->pos = curr_pos;
            // if (fls != NULL && fls->fg != NULL && fls->fg->fp != NULL && fls->fg->fp->info != NULL) {
            //     session->global_stat_info.feature_source_map[fls->fg->fp->source_type]+= 1;
            // } else {
            //     session->global_stat_info.feature_source_map[0]+= 1;
            // }
            line_point->src_status = 2;
            if (road_format_info.gen_left)
            {
                // if (prev_lc->ls->left->fg != NULL) {
                //     prev_lc->ls->left->fg->fp->update_status = 3;
                //     session->valid_lane_feature_list.push_back(prev_lc->ls->left->fg);
                // }
                // if (lc->ls->left->fg != NULL) {
                //     lc->ls->left->fg->fp->update_status = 3;
                //     session->valid_lane_feature_list.push_back(lc->ls->left->fg);
                // }
            }
            else
            {
                // if (prev_lc->ls->right->fg != NULL) {
                //     prev_lc->ls->right->fg->fp->update_status = 3;
                //     session->valid_lane_feature_list.push_back(prev_lc->ls->right->fg);
                // }
                // if (lc->ls->right->fg != NULL) {
                //     lc->ls->right->fg->fp->update_status = 3;
                //     session->valid_lane_feature_list.push_back(lc->ls->right->fg);
                // }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatNewRoad::format_boundary(RoadModelSessionData *session)
        {
            // 从全局配置中读取相关参数
            double high_gap = FLAGS_format_new_road_high_curvature_gap;           // 高曲率下的最大间隔
            double low_gap = FLAGS_format_new_road_low_curvature_gap;             // 低曲率下的最大间隔
            double max_radius_threshold = FLAGS_format_lane_max_radius_threshold; // 最大半径阈值，低于此值的曲率被认为是高曲率

            // 允许的道路索引值，0 表示主路，-1 表示辅路
            std::vector<int> valid_road_index = {0, -1};

            // 状态矩阵（当前代码没有使用，可能是为将来扩展预留的）
            std::vector<int> status_matrix = {
                0, 0, 1, 0,
                1, 1, 1, 0};

            int idx = 0;

            // 遍历有效的道路索引（主路和辅路）
            for (auto &rc_index : valid_road_index)
            {
                // 遍历当前会话中的每个子路段
                for (auto &srs : session->sub_road_segment_list)
                {
                    // 遍历当前子路段中的所有边界线
                    for (int i = 0; i < srs->new_boundary_line_list.size(); ++i)
                    {
                        auto &bl = srs->new_boundary_line_list[i];

                        // 如果当前边界线的 road_index 和 rc_index 不一致，跳过
                        if (bl->road_index != rc_index)
                        {
                            continue;
                        }

                        // 如果 rc_index 为 -1 且该子路段不需要左侧道路，则跳过
                        if (rc_index == -1 && !srs->need_left_road)
                        {
                            continue;
                        }

                        // 判断是否需要反向处理（例如，根据连通方向来确定是否需要反向处理）
                        bool oppo = false;
                        if ((srs->link_direction == 2) ^ (rc_index == -1))
                        {
                            oppo = true;
                        }

                        // 创建一个新的边界段对象
                        auto boundary_segment = session->add_ptr(session->boundary_segment_info_ptr);

                        // 初始化边界段的前一个边界点
                        RoadBoundaryPointInfo *prev_bp = NULL;
                        BoundaryFeature *last_fls = NULL;

                        // 设置边界段的侧面状态
                        boundary_segment->side_status = bl->side_status;

                        // 根据边界线的第一个特征，设置边界段的子类型
                        if (bl->list[0]->attr_feature == nullptr)
                        {
                            boundary_segment->subtype = bl->list[0]->sub_type;
                        }
                        else
                        {
                            boundary_segment->subtype = bl->list[0]->attr_feature->sub_type;
                        }
                        // int is_left = bl->side_status == 2 ? 0 : 1;
                        // int is_oppo = rc_index == -1 ? 1 : 0;
                        //// int road_type = bl->road_segment->road_type[bl->road_index] == 2 ? 1 : 0;
                        // int road_type = bl->road_type == 2 ? 1 : 0;
                        // int position = is_oppo * 4 + road_type * 2 + is_left;
                        // bool oppo = status_matrix[position] == 1;
                        for (int j = 0; j < bl->list.size(); ++j)
                        {
                            // 如果需要反向处理，则从后往前处理边界线点
                            int index = oppo ? bl->list.size() - j - 1 : j;
                            auto &fls = bl->list[index];

                            // 调试输出每个边界点的位置
                            session->debug_pos(fls->pos);
                            if (fls->key_pose != NULL)
                            {
                                session->debug_pos(fls->key_pose->pos);
                            }
                            else
                            {
                                int a = 1; // 这个分支没有实际作用，可能是调试代码
                            }

                            // 如果上一个边界点存在，则计算当前边界点与上一个边界点的距离
                            if (prev_bp != NULL)
                            {
                                double dis = alg::calc_dis(prev_bp->pos, fls->pos);
                                double gen_gap = low_gap;

                                // 如果当前点的曲率较小，则采用高间隔
                                if (fabs(fls->curvature) < max_radius_threshold)
                                {
                                    gen_gap = high_gap;
                                }

                                // 如果当前点与前一个点的距离小于规定的间隔，则跳过当前点
                                if (dis < gen_gap)
                                {
                                    last_fls = fls;
                                    continue;
                                }
                            }

                            // 如果距离过大，则更新上一个有效边界点
                            last_fls = NULL;

                            // 创建新的边界点信息，并将其添加到边界段中
                            auto bp = session->add_ptr(session->boundary_point_info_ptr);
                            bp->pos = fls->pos;
                            boundary_segment->point_info.push_back(bp.get());

                            // 更新前一个边界点
                            prev_bp = bp.get();
                        }

                        // 如果最后一个有效边界点不为空，则更新最后一个点的位置
                        if (last_fls != NULL)
                        {
                            boundary_segment->point_info.back()->pos = last_fls->pos;
                        }
                        boundary_segment->boundary_id = idx;
                        // 将新的边界段添加到会话中的新边界段列表中
                        session->new_boundary_segment.push_back(boundary_segment.get());

                        // 同时将新的边界段添加到当前子路段的边界段列表中
                        srs->new_boundary_segment.push_back(boundary_segment.get());
                        idx++;
                    }
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatNewRoad::format_obj_single(RoadObjectInfo *road_obj_info, Feature *fp)
        {

            // road_obj_info->id.type = fsdmap::ELEMENT_OBJECT_LOGIC;
            road_obj_info->type = "roadmark";
            road_obj_info->subtype = "";
            road_obj_info->width = fp->width;
            // if (fp->id.type() == fsdmap::TRT_CROSS_WALK3D && road_obj_info->width < 3.0) {
            //     road_obj_info->width = 3.0;
            // }
            road_obj_info->height = fp->height;
            road_obj_info->length = fp->length;
            road_obj_info->src_status = 2;
            // road_obj_info->timestamp = fp->timestamp;
            Eigen::Vector3d &obj_dir = fp->dir;
            road_obj_info->dir = obj_dir;

            Eigen::Vector3d fp_pos_11 = fp->pos;
            int32_t center_l13_tile_id = 0;
            Eigen::Vector3d center_pos_wgs;
            Eigen::Vector3d fp_pos_13;
            road_obj_info->pos = fp_pos_11;

            return fsdmap::SUCC;
        }

        int RoadModelProcFormatNewRoad::format_obj(RoadModelSessionData *session)
        {
            // RoadModelSessionManager* sm = (RoadModelSessionManager*)session->_session_mgr;
            // std::map<Feature*, int> fp_sign;
            // static const std::unordered_map<int, std::string>
            //     s_k_mapof_feat_type_to_proxy_subtype = { {700, "504"}, {702, "503"}, {703, "501"}, {705, "502"}, {704, "504"} };
            // bsl::string traffic_sign_define_json_file;
            // traffic_sign_define_json_file.setf("%s%s", session->data_processer->get_module_path(), FLAGS_format_new_road_traffic_sign_define_json_file.c_str());
            // if (!init_traffic_sign_infos(session, traffic_sign_define_json_file.c_str())){
            //     CFATAL_LOG(" !! Can not init traffic sign infos");
            // }
            // for (int i = 0; i < session->new_object_feature_list.size(); i++){
            //     auto fp = session->new_object_feature_list[i];
            //     if (MAP_FIND(fp_sign, fp) == 1 || fp->id.type() == fsdmap::TRT_LINE_LINE3D){
            //         continue;
            //     }
            //     if (fp->id.type() != fsdmap::TRT_STOP_LINE3D &&
            //             fp->id.type() != fsdmap::TRT_CROSS_WALK3D &&
            //             fp->id.type() != fsdmap::TRT_NO_PARKING_AREA3D &&
            //             fp->id.type() != fsdmap::TRT_TRAFFIC_LIGHT3D &&
            //             fp->id.type() != fsdmap::TRT_GROUND_SPEED_LIMIT3D &&
            //             fp->id.type() != fsdmap::TRT_POLE_BOX3D &&
            //             fp->id.type() != fsdmap::TRT_SIGN_BOX3D &&
            //             fp->id.type() != fsdmap::TRT_ROAD_TEXT3D &&
            //             fp->id.type() != fsdmap::TRT_GROUND_ARROW3D){
            //         continue;
            //     }
            //     if (fp->id.type() == fsdmap::TRT_TRAFFIC_LIGHT3D && fp->source_type == 2 && FLAGS_format_new_road_filter_green_light) {
            //         UpdatePoint* issue_point = session->add_issue_point(fsdmap::ELEMENT_OBJECT_BOUNDING_BOX, fp->pos,
            //             "请检查红绿灯", road_model_pb::TRAFFIC_LIGHT);
            //         continue;
            //     }
            //     if (fp->id.type() == fsdmap::TRT_STOP_LINE3D && fp->stop_status == 1) {
            //         auto issue = session->add_debug_issue(fp->pos, "stop_line_pos");
            //     }
            //     fp_sign[fp] = 1;
            //     auto road_obj_ptr = std::make_shared<RoadObjectInfo>();
            //     format_obj_single(road_obj_ptr.get(), fp);

            //     if (fp->id.type() == fsdmap::TRT_GROUND_ARROW3D) {
            //         road_obj_ptr->is_ground_arrow = true;
            //         get_boundary(road_obj_ptr.get(), fp);
            //     } else if (fp->id.type() == fsdmap::TRT_STOP_LINE3D) {
            //         road_obj_ptr->is_stop_line = true;
            //         road_obj_ptr->stop_line_is_valid = 1;
            //         if(FLAGS_format_new_road_other_object_boundary) {
            //             //extract_boundary_for_new_boundary(session, road_obj_ptr.get(), fp);
            //             get_boundary_stop_obj(session, road_obj_ptr.get(), fp);
            //         }
            //         UpdatePoint* issue_point = session->add_issue_point(fsdmap::ELEMENT_OBJECT_BOUNDING_BOX, road_obj_ptr->pos,
            //             "请检查停止线", road_model_pb::STOP_LINE);
            //     } else if (fp->id.type() == fsdmap::TRT_ROAD_TEXT3D){
            //         road_obj_ptr->is_road_text = true;
            //         if(FLAGS_format_new_road_other_object_boundary) {
            //             get_boundary_other_obj(session, road_obj_ptr.get(), fp);
            //         }
            //     } else if (fp->id.type() == fsdmap::TRT_NO_PARKING_AREA3D){
            //         road_obj_ptr->is_no_parking_area = true;
            //         if(FLAGS_format_new_road_other_object_boundary) {
            //             get_boundary_other_obj(session, road_obj_ptr.get(), fp);
            //         }
            //         UpdatePoint* issue_point = session->add_issue_point(fsdmap::ELEMENT_OBJECT_BOUNDING_BOX, road_obj_ptr->pos,
            //             "请检查禁停区", road_model_pb::NO_PARKING);
            //     } else if (fp->id.type() == fsdmap::TRT_TRAFFIC_LIGHT3D){
            //         road_obj_ptr->is_traffic_light = true;
            //         if (fp->type == 0) {
            //             road_obj_ptr->traffic_light_type = 0;
            //         } else if (fp->type == 1) {
            //             road_obj_ptr->traffic_light_type = 1;
            //         }
            //         if(FLAGS_format_new_road_other_object_boundary) {
            //             get_boundary_other_obj_v(session, road_obj_ptr.get(), fp);
            //         }
            //         UpdatePoint* issue_point = session->add_issue_point(fsdmap::ELEMENT_OBJECT_BOUNDING_BOX, road_obj_ptr->pos,
            //             "请检查红绿灯", road_model_pb::TRAFFIC_LIGHT);
            //     } else if (fp->id.type() == fsdmap::TRT_GROUND_SPEED_LIMIT3D){
            //         road_obj_ptr->max_speed = "";
            //         road_obj_ptr->min_speed = "";
            //         if (!fp->traffic_element_properties.empty()) {
            //             int speed_type = fp->traffic_element_properties[0].property_id();
            //             std::string max_speed = _get_speed_limit_value(speed_type);
            //             road_obj_ptr->max_speed = max_speed;
            //             if (fp->traffic_element_properties.size() == 2) {
            //                 speed_type = fp->traffic_element_properties[1].property_id();
            //                 std::string min_speed = _get_speed_limit_value(speed_type);
            //                 road_obj_ptr->min_speed = min_speed;
            //             }
            //         }
            //         road_obj_ptr->is_ground_speed_limit = true;
            //         if(FLAGS_format_new_road_other_object_boundary) {
            //             get_boundary_other_obj(session, road_obj_ptr.get(), fp);
            //         }
            //     } else if (fp->id.type() == fsdmap::TRT_CROSS_WALK3D) {
            //         road_obj_ptr->is_cross_walk = true;
            //         get_boundary(road_obj_ptr.get(), fp);
            //         UpdatePoint* issue_point = session->add_issue_point(fsdmap::ELEMENT_OBJECT_BOUNDING_BOX, road_obj_ptr->pos,
            //             "请检查人行横道", road_model_pb::CROSS_WALK);
            //     } else if (fp->id.type() == fsdmap::TRT_POLE_BOX3D){
            //         road_obj_ptr->is_pole_box = true;
            //         //get_boundary_other_obj(session, road_obj_ptr.get(), fp);
            //         extract_boundary_for_new_boundary(session, road_obj_ptr.get(), fp);
            //         int pole_type = fp->type;
            //         auto pos = s_k_mapof_feat_type_to_proxy_subtype.find(pole_type);
            //         if (pos == s_k_mapof_feat_type_to_proxy_subtype.cend()) {
            //             // CFATAL_LOG(" !! Can not find pole type @[%d]", pole_type);
            //             // continue;
            //             road_obj_ptr->pole_logic_subtype = "504";
            //         } else {
            //             road_obj_ptr->pole_logic_subtype = pos->second;
            //         }

            //     } else if (fp->id.type() == fsdmap::TRT_SIGN_BOX3D) {
            //         const auto element_properties = fp->traffic_element_properties;
            //         if (!element_properties.empty()) {
            //             int feat_shape_value = element_properties[0].traffic_element_2d().shape();
            //             if (feat_shape_value < 0 || feat_shape_value > 5) {
            //                 CFATAL_LOG( " !! Bad sign shape id [feat_shape_value=%d]", feat_shape_value);
            //             }
            //             else {
            //                 int proxy_shape_value = feat_shape_value;
            //                 road_obj_ptr->sign_shape = proxy_shape_value;
            //             }
            //         }
            //         road_obj_ptr->is_sign_box = true;
            //         //get_boundary_other_obj(session, road_obj_ptr.get(), fp);
            //         extract_boundary_for_new_boundary(session, road_obj_ptr.get(), fp);
            //         if (!element_properties.empty()) {
            //             std::set<int> checked_speed_sign_set;
            //             std::set<int> checked_others_sign_set;
            //             for (int i = 0; i < element_properties.size(); ++i) {
            //                 const auto& element_property = element_properties[i];
            //                 int label = element_property.property_id();
            //                 auto pos = session->_mapof_traffic_sign_infos.find(label);
            //                 if (pos == session->_mapof_traffic_sign_infos.cend()) {
            //                     CFATAL_LOG( " !! Invalid feature property, label &d ", label);
            //                     continue;
            //                 }
            //                 const auto& sign_def_info = pos->second;
            //                 TrafficSignInfo sign_info;
            //                 sign_info.sign_def_info_ptr = &sign_def_info;
            //                 sign_info.element_property_ptr = &element_property;
            //                 if (sign_def_info.subtype == "限速") {
            //                     auto pos = checked_speed_sign_set.find(label);
            //                     if (pos == checked_speed_sign_set.cend()) {
            //                         road_obj_ptr->speed_sign_infos.push_back(sign_info);
            //                         checked_speed_sign_set.insert(label);
            //                     }
            //                 }
            //                 else {
            //                     auto pos = checked_others_sign_set.find(label);
            //                     if (pos == checked_others_sign_set.cend()) {
            //                         road_obj_ptr->others_sign_infos.push_back(sign_info);
            //                         checked_others_sign_set.insert(label);
            //                     }
            //                 }
            //             }
            //         }
            //     }
            //     session->object_ptr.push_back(road_obj_ptr);
            //     session->update_obj.push_back(road_obj_ptr.get());
            //     if (fp->id.type() == fsdmap::TRT_STOP_LINE3D && fp->stop_status == 1) {
            //         float minp[] = {road_obj_ptr->pos.x(), road_obj_ptr->pos.y(), road_obj_ptr->pos.z()};
            //         float maxp[] = {road_obj_ptr->pos.x(), road_obj_ptr->pos.y(), road_obj_ptr->pos.z()};
            //         session->stop_line_object_tree.Insert(minp, maxp, road_obj_ptr.get());
            //     }
            //     if (fp->id.type() == fsdmap::TRT_TRAFFIC_LIGHT3D) {
            //         float minp[] = {road_obj_ptr->pos.x(), road_obj_ptr->pos.y(), road_obj_ptr->pos.z()};
            //         float maxp[] = {road_obj_ptr->pos.x(), road_obj_ptr->pos.y(), road_obj_ptr->pos.z()};
            //         session->traffic_light_object_tree.Insert(minp, maxp, road_obj_ptr.get());
            //         session->traffic_light_obj.push_back(road_obj_ptr.get());
            //     }
            //     if (fp->id.type() == fsdmap::TRT_CROSS_WALK3D) {
            //         if (fp->traffic_element_properties.size() > 0) {
            //             int countor_points_size = fp->traffic_element_properties[0].countor_points().size();
            //             for (int i = 0; i < countor_points_size; ++i) {
            //                 const fsdmap::Point3D& src_pt = fp->traffic_element_properties[0].countor_points(i);
            //                 float minp[] = {src_pt.x() + fp->pos.x(), src_pt.y() + fp->pos.y(), src_pt.z() + fp->pos.z()};
            //                 float maxp[] = {src_pt.x() + fp->pos.x(), src_pt.y() + fp->pos.y(), src_pt.z() + fp->pos.z()};
            //                 session->cross_walk_boundary_tree.Insert(minp, maxp, road_obj_ptr.get());
            //             }
            //         }
            //     }
            // }
            return fsdmap::SUCC;
        }

        // void RoadModelProcFormatNewRoad::extract_boundary_for_new_boundary(RoadModelSessionData* session,
        //         RoadObjectInfo* box, Feature* fp) {
        //      if (fp->traffic_element_properties.size() > 0) {
        //            auto tf_elem = fp->traffic_element_properties[0];
        //            if (tf_elem.has_traffic_element_2d()) {
        //                auto shape = tf_elem.traffic_element_2d().shape();
        //                box->sign_shape = shape;
        //            }
        //
        //            for (auto& item : tf_elem.countor_points()) {
        //                auto x = item.x();
        //                auto y = item.y();
        //                auto z = item.z();
        //
        //                Eigen::Vector3d boundary_point(x, y, z);
        //                box->element_boundary_line.push_back(boundary_point);
        //            }
        // 	 }
        // }

        // void RoadModelProcFormatNewRoad::extract_boundary_for_new_boundary(RoadModelSessionData* session,
        //         RoadObjectInfo* box, Feature* fp) {
        //      if (fp->traffic_element_properties.size() > 0) {
        //            auto tf_elem = fp->traffic_element_properties[0];
        //            // Point3dList bds;
        //            auto cx = box->pos.x();
        //            auto cy = box->pos.y();
        //            auto z = box->pos.z();
        //            auto w = box->width;
        //            auto len = box->length;
        //            double a = box->heading / 180.0 * M_PI;
        //            if (tf_elem.has_traffic_element_2d()) {
        //                auto shape = tf_elem.traffic_element_2d().shape();
        //                box->sign_shape = shape;
        //            }
        //
        //            auto i = 0;
        //            for (auto& item : tf_elem.countor_points())
        //            {
        //                //auto pt = Point3d(item.x() + tile_11_local.x(), item.y() + tile_11_local.y(), item.z() + tile_11_local.z());
        //                //bds.emplace_back(CoordTransformer::cal_tile_13_coord_from_tile_11(tile_11, tile_11_local, tile_13_id));
        //                auto x = item.x();
        //                auto y = item.y();
        //                auto x1 = (x * cos(a) + y * sin(a)) * 2 / w;
        //                auto y1 = (-x * sin(a) + y * cos(a)) * 2 / len;
        //
        //                // auto bd = box->add_boundary();
        //                // bd->set_x(x1);
        //                // bd->set_y(y1);
        //                auto nz = item.z();
        //                nz = nz * 2 / (box->height > 0.01 ? box->height: 0.01);
        //                // bd->set_z(nz);
        //
        //                Eigen::Vector3d boundary_point(x1, y1, nz);
        //                box->element_boundary_line.push_back(boundary_point);
        //            }
        //	 }
        // }

        // bool RoadModelProcFormatNewRoad::init_traffic_sign_infos(RoadModelSessionData* session,
        //         std::string traffic_sign_define_json_file) {
        //     std::ifstream ifs(traffic_sign_define_json_file);
        //     if (!ifs.is_open()) {
        //         LOG(ERROR) << " !!! Can not open file : " << traffic_sign_define_json_file;
        //         return false;
        //     }
        //     rapidjson::IStreamWrapper ifsw(ifs);
        //     rapidjson::Document doc;
        //     doc.ParseStream(ifsw);
        //     if (!doc.IsObject()) {
        //         CFATAL_LOG( " !!! Parse json file %s failed.", traffic_sign_define_json_file);
        //         ifs.close();
        //         return false;
        //     }
        //     if (!doc.HasMember("Traffic_Sign_Type") ) {
        //         CFATAL_LOG( " !!! Has no member named Traffic_Sign_Type.");
        //         ifs.close();
        //         return false;
        //     }
        //     const auto& traffic_sign_infos = doc["Traffic_Sign_Type"];
        //     if (!traffic_sign_infos.IsArray()) {
        //         CFATAL_LOG( " !!! Traffic_Sign_Type must be array type.");
        //         ifs.close();
        //         return false;
        //     }
        //     for (int i = 0; i < traffic_sign_infos.Size(); ++i) {
        //         const auto& value = traffic_sign_infos[i];
        //         CHECK(value.HasMember("label"));
        //         CHECK(value.HasMember("type"));
        //         CHECK(value.HasMember("subtype"));
        //         int label = value["label"].GetInt();
        //         std::string type = value["type"].GetString();
        //         std::string subtype = value["subtype"].GetString();
        //         std::string logic_name = "";
        //         double logic_value = -1.0;
        //         if (value.HasMember("logic_name")) {
        //             logic_name = value["logic_name"].GetString();
        //         }
        //         if (value.HasMember("logic_value")) {
        //             logic_value = value["logic_value"].GetDouble();
        //         }
        //         TrafficSignDef info;
        //         info.label = label;
        //         info.type = type;
        //         info.subtype = subtype;
        //         info.logic_name = logic_name;
        //         info.logic_value = logic_value;
        //         session->_mapof_traffic_sign_infos.insert({label, info});
        //     }
        //     return true;
        // }
        // std::string RoadModelProcFormatNewRoad::_get_speed_limit_value(int speed_type) const
        // {
        //     std::string speed_value_str = "0";
        //
        //     switch (speed_type)
        //     {
        //         case 1:
        //             speed_value_str = "20";
        //             break;
        //         case 2:
        //             speed_value_str = "30";
        //             break;
        //         case 3:
        //             speed_value_str = "40";
        //             break;
        //         case 4:
        //             speed_value_str = "50";
        //             break;
        //         case 5:
        //             speed_value_str = "60";
        //             break;
        //         case 6:
        //             speed_value_str = "70";
        //             break;
        //         case 7:
        //             speed_value_str = "80";
        //             break;
        //         case 8:
        //             speed_value_str = "90";
        //             break;
        //         case 9:
        //             speed_value_str = "100";
        //             break;
        //         case 10:
        //             speed_value_str = "110";
        //             break;
        //         case 11:
        //             speed_value_str = "120";
        //             break;
        //         case 12:
        //             speed_value_str = "150";
        //             break;
        //         default:
        //             break;
        //     }
        //     return speed_value_str;
        // }

        // int RoadModelProcFormatNewRoad::get_boundary_other_obj_v(RoadModelSessionData* session, RoadObjectInfo* road_obj_info, Feature* fp){
        //     Eigen::Vector3d dir = fp->dir.normalized();
        //     Eigen::Vector3d norm_dir(-dir.y(), dir.x(), 1);
        //     double x = fp->pos.x() - fp->length / 2  * dir.x();
        //     double z = fp->pos.z() - fp->height / 2 ;
        //     double y = fp->pos.y() - fp->length / 2 * dir.y();
        //
        //     int32_t center_l13_pt = 0;
        //     Eigen::Vector3d pt_11(x, y, z);
        //     get_boundary_point_v(fp, road_obj_info, pt_11);
        //     pt_11.x() = pt_11.x() + dir.x() * fp->length;
        //     pt_11.y() = pt_11.y() + dir.y() * fp->length;
        //     get_boundary_point_v(fp, road_obj_info, pt_11);
        //     pt_11.z() = pt_11.z() + norm_dir.z() * fp->height;
        //     get_boundary_point_v(fp, road_obj_info, pt_11);
        //     pt_11.x() = pt_11.x() - dir.x() * fp->length;
        //     pt_11.y() = pt_11.y() - dir.y() * fp->length;
        //     get_boundary_point_v(fp, road_obj_info, pt_11);
        //
        //     return fsdmap::SUCC;
        // }
        //
        // int RoadModelProcFormatNewRoad::get_boundary_stop_obj(RoadModelSessionData* session, RoadObjectInfo* road_obj_info, Feature* fp){
        //     Eigen::Vector3d dir = fp->dir.normalized();
        //     if (judge_stop_line_dir(session, road_obj_info)) {
        //         dir = -dir;
        //     }
        //     Eigen::Vector3d norm_dir(-dir.y(), dir.x(), dir.z());
        //     Eigen::Vector3d tar_pos = fp->pos + fp->length / 2 * dir;
        //     tar_pos.z() = fp->pos.z();
        //     if (FLAGS_format_new_road_stop_line_use_edge_enable) {
        //         // x = fp->pos.x() + fp->length / 2 * dir.x() + fp->width / 2 * norm_dir.x();
        //         // y = fp->pos.y() + fp->length / 2 * dir.y() + fp->width / 2 * norm_dir.y();
        //         tar_pos += fp->width / 2 * norm_dir;
        //         tar_pos.z() = fp->pos.z();
        //     }
        //
        //
        //     int32_t center_l13_pt = 0;
        //     get_boundary_point(fp, road_obj_info, tar_pos);
        //     //auto issue = session->add_debug_issue(pt_11, "select stop first boiundary !!");
        //     //auto issue1 = session->add_debug_issue(fp->pos, alg::get_hori_pos(fp->pos, fp->dir, 1), "select stop direction!!");
        //     tar_pos += -dir * fp->length;
        //     tar_pos.z() = fp->pos.z();
        //     // pt_11.x() = pt_11.x() + -dir.x() * fp->length;
        //     // pt_11.y() = pt_11.y() + -dir.y() * fp->length;
        //     get_boundary_point(fp, road_obj_info, tar_pos);
        //     //pt_11.x() = pt_11.x() + -norm_dir.x() * fp->width * 2;
        //     //pt_11.y() = pt_11.y() + -norm_dir.y() * fp->width * 2;
        //     //get_boundary_point(fp, road_obj_info, pt_11);
        //     //pt_11.x() = pt_11.x() + dir.x() * fp->length * 2;
        //     //pt_11.y() = pt_11.y() + dir.y() * fp->length * 2;
        //     //get_boundary_point(fp, road_obj_info, pt_11);
        //     return fsdmap::SUCC;
        // }
        //
        // bool RoadModelProcFormatNewRoad::judge_stop_line_dir(RoadModelSessionData* session, RoadObjectInfo* road_obj_info){
        //     bool left_dir = true;
        //     double min_dis = 1000;
        //     PosSample* min_poss = NULL;
        //     Eigen::Vector3d pos = road_obj_info->pos;
        //     float minb[3] = {pos.x() - 50, pos.y() - 50, pos.z() - 5};
        //     float maxb[3] = {pos.x() + 50, pos.y() + 50, pos.z() + 5};
        //     std::vector<PosSample*> search_rets;
        //     session->pos_sample_tree.Search(minb, maxb, SearchResultCallback<PosSample>, &search_rets);
        //     if (search_rets.size() == 0) {
        //         return left_dir;
        //     }
        //     for (auto& pos_get : search_rets) {
        //         if (alg::calc_dis(pos_get->pos->local_pos, pos) < min_dis) {
        //             min_dis = alg::calc_dis(pos_get->pos->local_pos, pos);
        //             min_poss = pos_get;
        //         }
        //     }
        //     if (alg::calc_theta_with_dir(min_poss->pos->dir.normalized(), road_obj_info->dir.normalized()) < 0) {
        //         left_dir = false;
        //     }
        //     return left_dir;
        // }
        //
        // int RoadModelProcFormatNewRoad::get_boundary_other_obj(RoadModelSessionData* session, RoadObjectInfo* road_obj_info, Feature* fp){
        //     Eigen::Vector3d dir = fp->dir.normalized();
        //     Eigen::Vector3d norm_dir(-dir.y(), dir.x(), dir.z());
        //     double x = fp->pos.x() + fp->length / 2 * dir.x() + fp->width / 2 * norm_dir.x();
        //     double y = fp->pos.y() + fp->length / 2 * dir.y() + fp->width / 2 * norm_dir.y();
        //     double z = fp->pos.z();
        //
        //     int32_t center_l13_pt = 0;
        //     Eigen::Vector3d pt_11(x, y, z);
        //     get_boundary_point(fp, road_obj_info, pt_11);
        //     pt_11.x() = pt_11.x() + -dir.x() * fp->length;
        //     pt_11.y() = pt_11.y() + -dir.y() * fp->length;
        //     get_boundary_point(fp, road_obj_info, pt_11);
        //     pt_11.x() = pt_11.x() + -norm_dir.x() * fp->width;
        //     pt_11.y() = pt_11.y() + -norm_dir.y() * fp->width;
        //     get_boundary_point(fp, road_obj_info, pt_11);
        //     pt_11.x() = pt_11.x() + dir.x() * fp->length;
        //     pt_11.y() = pt_11.y() + dir.y() * fp->length;
        //     get_boundary_point(fp, road_obj_info, pt_11);
        //
        //     return fsdmap::SUCC;
        // }
        //
        // int RoadModelProcFormatNewRoad::get_boundary_point_v(Feature* fp, RoadObjectInfo* road_obj_info, Eigen::Vector3d pt_11){
        //     Eigen::Vector3d dir = fp->dir.normalized();
        //     Eigen::Vector3d norm_dir(-dir.y(), dir.x(), 1);
        //     Eigen::Vector3d center_pt;
        //     Eigen::Vector3d pt_13;
        //     int32_t center_l13_pt = 0;
        //     alg::trans_local2wgs(pt_11, center_pt, true);
        //     alg::trans_wgs2local13_by_id(center_pt, pt_13, road_obj_info->id.tile_id, false);
        //     double vx = pt_13.x() - road_obj_info->pos_13.x();
        //     double vy = pt_13.y() - road_obj_info->pos_13.y();
        //     double vz = pt_13.z() - road_obj_info->pos_13.z();
        //     double x_proj = vx * dir.x() + vy * dir.y() + vz * dir.z();
        //     double z_proj = vx * norm_dir.x() + vy * norm_dir.y() + vz * norm_dir.z();
        //
        //     x_proj /= (0.5 * fp->length);
        //     z_proj /= (0.5 * fp->height);
        //
        //     Eigen::Vector3d boundary_point(x_proj, 0.0, z_proj);
        //     road_obj_info->element_boundary_line.push_back(boundary_point);
        //     return fsdmap::SUCC;
        // }
        //
        // int RoadModelProcFormatNewRoad::get_boundary_point(Feature* fp, RoadObjectInfo* road_obj_info, Eigen::Vector3d pt_11){
        //     Eigen::Vector3d dir = fp->dir.normalized();
        //     Eigen::Vector3d norm_dir(-dir.y(), dir.x(), dir.z());
        //     Eigen::Vector3d center_pt;
        //     Eigen::Vector3d pt_13;
        //     int32_t center_l13_pt = 0;
        //     alg::trans_local2wgs(pt_11, center_pt, true);
        //     alg::trans_wgs2local13_by_id(center_pt, pt_13, road_obj_info->id.tile_id, false);
        //     double vx = pt_13.x() - road_obj_info->pos_13.x();
        //     double vy = pt_13.y() - road_obj_info->pos_13.y();
        //     double vz = pt_13.z() - road_obj_info->pos_13.z();
        //     double x_proj = vx * dir.x() + vy * dir.y() + vz * dir.z();
        //     double y_proj = vx * norm_dir.x() + vy * norm_dir.y() + vz * norm_dir.z();
        //
        //     x_proj /= (0.5 * fp->length);
        //     y_proj /= (0.5 * fp->width);
        //
        //     Eigen::Vector3d boundary_point(x_proj, y_proj, 0.0);
        //     road_obj_info->element_boundary_line.push_back(boundary_point);
        //     return fsdmap::SUCC;
        // }
        //
        // int RoadModelProcFormatNewRoad::get_boundary(RoadObjectInfo* road_obj_info, Feature* fp){
        //
        //     Eigen::Vector3d dir = fp->dir.normalized();
        //     Eigen::Vector3d norm_dir(-dir.y(), dir.x(), dir.z());
        //     Eigen::Vector3d rmin(FLT_MAX, FLT_MAX, FLT_MAX);
        //     Eigen::Vector3d rmax = -rmin;
        //     std::vector<cv::Point2d> norm_point;
        //     double diff_x = 0.0;
        //     double diff_y = 0.0;
        //     double tile_center_x = 0.0;
        //     double tile_center_y = 0.0;
        //
        //     //double offset_x = fp->pos.x() - fp->raw_pos.x();
        //     //double offset_y = fp->pos.y() - fp->raw_pos.y();
        //     if (fp->traffic_element_properties.size() == 0) {
        //         Eigen::Vector3d center_pos_wgs;
        //         alg::trans_local2wgs(fp->pos, center_pos_wgs, true);
        //         FLOG_POINT(center_pos_wgs, " boundary is not exist!!");
        //         return fsdmap::SUCC;
        //     }
        //     int countor_points_size = fp->traffic_element_properties[0].countor_points().size();
        //     for (int i = 0; i < countor_points_size; ++i) {
        //         const fsdmap::Point3D& src_pt = fp->traffic_element_properties[0].countor_points(i);
        //         //double x = src_pt.x() + offset_x;
        //         //double y = src_pt.y() + offset_y;
        //         double x = src_pt.x() + fp->pos.x();
        //         double y = src_pt.y() + fp->pos.y();
        //         double z = src_pt.z() + fp->pos.z();
        //         int32_t center_l13_pt = 0;
        //         Eigen::Vector3d pt_11(x, y, z);
        //
        //         Eigen::Vector3d center_pt;
        //         Eigen::Vector3d pt_13;
        //         alg::trans_local2wgs(pt_11, center_pt, true);
        //         alg::trans_wgs2local13_by_id(center_pt, pt_13, road_obj_info->id.tile_id, false);
        //         //alg::trans_wgs2local13(center_pt, pt_13, center_l13_pt, false);
        //         double vx = pt_13.x() - road_obj_info->pos_13.x();
        //         double vy = pt_13.y() - road_obj_info->pos_13.y();
        //         double vz = pt_13.z() - road_obj_info->pos_13.z();
        //         double x_proj = vx * dir.x() + vy * dir.y() + vz * dir.z();
        //         double y_proj = vx * norm_dir.x() + vy * norm_dir.y() + vz * norm_dir.z();
        //         rmin.x() = std::min(rmin.x(), x_proj);
        //         rmin.y() = std::min(rmin.y(), y_proj);
        //         rmax.x() = std::max(rmax.x(), x_proj);
        //         rmax.y() = std::max(rmax.y(), y_proj);
        //
        //         x_proj /= (0.5 * fp->length);
        //         y_proj /= (0.5 * fp->width);
        //         cv::Point2d temp_pt;
        //         temp_pt.x = x_proj;
        //         temp_pt.y = y_proj;
        //         norm_point.push_back(temp_pt);
        //         if (x_proj < -1.0) {
        //             diff_x = x_proj - (-1.0);
        //         }
        //         else if (x_proj > 1.0) {
        //             diff_x = x_proj - 1.0;
        //         }
        //         if (y_proj < -1.0) {
        //             diff_y = y_proj - (-1.0);
        //         }
        //         else if (y_proj > 1.0) {
        //             diff_y = y_proj - 1.0;
        //         }
        //     }
        //
        //     if (!fp->traffic_element_properties.empty() && fp->id.type() == fsdmap::TRT_GROUND_ARROW3D) {
        //         static const int s_k_feat_id_to_detailed_type_tab[] = { 201, 204, 203, 206, 211, 210, 205, 202, 207, 209, 208 };
        //         int detailed_type = fp->traffic_element_properties[0].property_id();
        //         if (detailed_type < 0 || detailed_type > 10) {
        //             CFATAL_LOG( " !! Bad Garrow feature property id [detailed_type=%d]", detailed_type);
        //             return fsdmap::SUCC;
        //         }
        //         else {
        //             road_obj_info->detailed_type = s_k_feat_id_to_detailed_type_tab[detailed_type];
        //         }
        //     }
        //
        //     for (size_t i = 0; i < norm_point.size(); i++) {
        //         double x = norm_point[i].x - diff_x;
        //         double y = norm_point[i].y - diff_y;
        //         Eigen::Vector3d boundary_point(x, y, 0.0);
        //         road_obj_info->element_boundary_line.push_back(boundary_point);
        //     }
        //     double width = rmax.x() - rmin.x();
        //     double length = rmax.y() - rmin.y();
        //     //road_obj_info->width = length;
        //     //if (fp->id.type() == fsdmap::TRT_CROSS_WALK3D && road_obj_info->width < 3.0) {
        //     //    road_obj_info->width = 3.0;
        //     //}
        //     //road_obj_info->height = 0.01;
        //     //road_obj_info->length = width;
        //
        //     return fsdmap::SUCC;
        // }

        // int RoadModelProcFormatNewRoad::gen_issue(RoadModelSessionData* session) {
        //     for (auto &road_segment : session->final_road_segment) {
        //         for (auto &lane_line : road_segment->valid_lane_list) {
        //             if (lane_line->invalid()) {
        //                 continue;
        //             }
        //             for (auto &lc : lane_line->list) {
        //                 session->add_issue_for_dece(road_segment, lc, true);
        //                 session->add_issue_for_dece(road_segment, lc, false);
        //             }
        //         }
        //     }
        //     return fsdmap::SUCC;
        // }

        int  RoadModelProcFormatNewRoad::check_connect(RoadModelSessionData *session)
        {
            auto calc_dis=[&](Eigen::Vector3d &pos,RoadLaneInfo* prev,RoadLaneInfo*next,int type)->int 
            {
                if(!prev||!next)
                {
                    return 0 ;
                }
                // RoadLinePointInfo* prev_point=NULL;
                // RoadLinePointInfo* next_point=NULL;
                RoadLaneBoundaryInfo*prev_info=NULL;
                RoadLaneBoundaryInfo*next_info=NULL;
                switch(type)
                {
                   case 0:
                   {
                    prev_info=prev->left_lane_boundary_info;
                    next_info=next->left_lane_boundary_info;
                    break;
                   }
                   case 1:
                   {
                    prev_info=prev->right_lane_boundary_info;
                    next_info=next->right_lane_boundary_info;
                    break;
                   }
                   case 2:
                   {
                    prev_info=prev->center_lane_boundary_info;
                    next_info=next->center_lane_boundary_info;
                   }
                };
               if(!prev_info|| !next_info
                ||prev_info->line_point_info.empty()
                ||next_info->line_point_info.empty())
               {
                 return 0;
               }
               auto prev_point=prev_info->line_point_info.back();
               auto next_point=next_info->line_point_info.front();
               pos=prev_point->pos;
               return alg::calc_dis(prev_point->pos,next_point->pos)*100;
            };

           for (auto lane_group: session->new_lane_groups)
           {
             for(auto lane_line:lane_group->lane_line_info)
             {
                auto all_next=lane_line->context.all_next;
                if(all_next.empty())
                {
                    continue;
                }
                for(auto next:all_next)
                {
                    for(auto type:{0,1,2})
                    {
                       Eigen::Vector3d pos;
                       int dis=calc_dis(pos,lane_line,next.src,type);
                       if( dis!=0)
                       {
                        BindKeyPosePoint new_point;
                        new_point.pos=pos;
                        new_point.bind_dis=dis;
                        debug_dis_connect_point.push_back(new_point);
                        // LOG_WARN("debug_dis_connect_point dis: {}",dis);
                       }
                    }
                   
                }
             }
           }
        //    LOG_WARN("debug_dis_connect_point:{}",debug_dis_connect_point.size());
           return fsdmap::SUCC;
        }

        int RoadModelProcFormatNewRoad::is_center_line_missed(RoadModelSessionData *session, RoadLaneBoundaryInfo *prev_center_line, RoadLaneBoundaryInfo *current_center_line)
        {
            double average_distance = 0, dis = FLT_MAX;
            Eigen::Vector3d nearst_pt;
            if(prev_center_line == nullptr)
            {
                return fsdmap::SUCC;
            }
            for(auto pt : current_center_line->line_point_info)
            {
                dis = FLT_MAX;
                for(auto pt_prev : prev_center_line->line_point_info)
                {
                    if(alg::calc_dis(pt->pos, pt_prev->pos) < dis)
                    {
                        dis = alg::calc_dis(pt->pos, pt_prev->pos);
                        nearst_pt = pt_prev->pos;
                    }
                }

                average_distance += fabs(alg::calc_vertical_dis(nearst_pt, pt->pos, pt->dir, true)) / current_center_line->line_point_info.size();
            }

            if(average_distance > 5)
            {
                return fsdmap::FAIL;
            }
            return fsdmap::SUCC;

        }

        int RoadModelProcFormatNewRoad::save_debug_info(RoadModelSessionData *session)
        {
            if (!FLAGS_format_new_road_save_data_enable)
            {
                return fsdmap::SUCC;
            }
            session->set_display_name("format_new_road");
            // for (auto &link : session->raw_links) {
            //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
            //     log->color = {223, 0, 154};
            //     for (auto &pt : link->list) {
            //         log->add(pt->pos);
            //     }
            // }
            for (auto &rs : session->road_segment_list)
            {
                auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
                log->color = {223, 0, 154};
                for (auto &poss : rs->pos_sample_list)
                {
                    if(poss->invalid())
                    {
                        log->color = {0, 0, 255};
                    }
                    log->add(poss->pos);
                }
            }
            for (int i = 0; i < session->new_lane_groups.size(); ++i)
            {
                auto &lg = session->new_lane_groups[i];
                for (int j = 0; j < lg->lane_line_info.size(); ++j)
                {
                    auto &ll = lg->lane_line_info[j];
                    for (auto &p_ll_ptr : ll->context.all_prev)
                    {
                        auto &p_ll = p_ll_ptr.src;
                        auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "");
                        log->color = {255, 0, 0};
                        auto &p_lp = p_ll->center_lane_boundary_info->line_point_info.back();
                        auto &n_lp = ll->center_lane_boundary_info->line_point_info.front();
                        auto p_1 = alg::get_hori_pos(p_lp->pos, p_lp->dir, -1);
                        auto p_2 = alg::get_hori_pos(n_lp->pos, n_lp->dir, 1);
                        log->add(p_1, 2);
                        log->add(p_2, 2);
                    }
                    if (ll->context.all_prev.size() == 0)
                    {
                        auto log = session->add_debug_log(utils::DisplayInfo::POINT, "");
                        log->color = {255, 0, 0};
                        log->add(ll->center_lane_boundary_info->line_point_info.front()->pos);
                    }
                    if (ll->context.all_next.size() == 0)
                    {
                        auto log = session->add_debug_log(utils::DisplayInfo::POINT, "");
                        log->color = {255, 0, 0};
                        log->add(ll->center_lane_boundary_info->line_point_info.back()->pos);
                    }
                }
                auto lg_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "new lane group{}", i);
                auto le_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "new lane group{}", i);
                lg_log->color = {0, 255, 0};
                le_log->color = {255, 255, 0};
                for (int j = 0; j < lg->lane_line_info.size(); ++j)
                {
                    auto &ll = lg->lane_line_info[j];
                    for (int si = 0; si < 2; ++si)
                    {
                        auto &lb = si == 0 ? ll->left_lane_boundary_info : ll->right_lane_boundary_info;
                        auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "new lane boundary{}", j);
                        for (int k = 0; k < lb->line_point_info.size(); ++k)
                        {
                            auto &lp = lb->line_point_info[k];
                            auto &ele = log->add(lp->pos, lb->geo);
                            ele.label.opt_label = lb->color;
                            //
                            ele.label.cloud_pano_seg=lb->color;
                            ele.label.cloud_line_seg=lb->type;

                            if (k == 0)
                            {
                                lg_log->add(lp->pos);
                            }
                            if (k == lb->line_point_info.size() - 1)
                            {
                                le_log->add(lp->pos);
                            }
                        }
                    }
                }
                for (int j = 0; j < lg->lane_line_info.size(); ++j)
                {
                    auto &ll = lg->lane_line_info[j];
                    auto &lb = ll->center_lane_boundary_info;
                    if (lb == NULL)
                    {
                        continue;
                    }
                    auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "new lane boundary{}", j);
                    log->color = {0, 255, 0};
                    for (int k = 0; k < lb->line_point_info.size(); ++k)
                    {
                        auto &lp = lb->line_point_info[k];
                        auto &ele = log->add(lp->pos);
                    }
                }
            }
            for (auto &bs : session->new_boundary_segment)
            {
                auto blog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "new road_boundary");
                // blog->color = {81, 89, 240};
                blog->color = {152, 99, 60};
                for (auto &bp : bs->point_info)
                {
                    blog->add(bp->pos);
                }
            }
            for (auto &bs : session->raw_object_ret_list)
            {
                if (bs->ele_type == 5) // 人行横道
                {
                    auto blog = session->add_debug_log(utils::DisplayInfo::POLYGEN, "new crosswalks");
                    blog->color = {165, 42, 42};
                    for (auto &bp : bs->list)
                    {
                        blog->add(bp->pos);
                    }
                }
            }
            for (auto &bs : session->raw_object_ret_list)
            {
                if (bs->ele_type == 6) // 停止线
                {
                    auto blog = session->add_debug_log(utils::DisplayInfo::LINE, "new stoplines");
                    blog->color = {255, 0, 255};
                    for (auto &bp : bs->list)
                    {
                        blog->add(bp->pos);
                    }
                }
            }
            for (auto &bs : session->raw_object_ret_list)
            {
                if (bs->ele_type == 3) // 箭头
                {
                    auto blog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "new arrow");
                    blog->color = {255, 0, 0};
                    for (auto &bp : bs->list)
                    {
                        blog->add(bp->pos);
                    }
                }
            }
            for (auto &bs : session->raw_object_ret_list)
            {
                if (bs->ele_type == 4) // 注意行人
                {
                    auto blog = session->add_debug_log(utils::DisplayInfo::POLYGEN, "new other object");
                    blog->color = {139, 58, 58};
                    for (auto &bp : bs->list)
                    {
                        blog->add(bp->pos);
                    }
                }
            }

            // ////////////////////////obj格式输出//////////////////////////
            // std::string road_path = FLAGS_format_new_road_obj_output_file + "roadboundarys_bev.obj";
            // std::string lane_path = FLAGS_format_new_road_obj_output_file + "laneboundarys_bev.obj";
            // std::ofstream ofs_road(road_path,std::ofstream::trunc);
            // std::ofstream ofs_lane(lane_path,std::ofstream::trunc);
            // int lane_point_idx = 1;
            // std::map<std::string, std::vector<int>> line_road_map;
            // std::map<std::string, std::vector<int>> line_lane_map;
            //
            // // std::cout<<"lane groups size is "<<session->new_lane_groups.size()<<std::endl;
            // for (int i = 0; i < session->new_lane_groups.size(); ++i) {
            //     auto &lg = session->new_lane_groups[i];
            //     // std::cout<<"lane boundary size is "<<lg->lane_boundary_info.size()<<std::endl;
            //     for (int j = 0; j < lg->lane_boundary_info.size(); ++j) {
            //         auto &lb = lg->lane_boundary_info[j];
            //         std::string lane_id = std::to_string(100*i+j);
            //         // std::cout<<"lane point size is "<<lb->line_point_info.size()<<std::endl;
            //         for (int k = 0; k < lb->line_point_info.size(); ++k) {
            //             auto &lp = lb->line_point_info[k];
            //             Eigen::Vector3d wgs;
            //             session->data_processer->local2wgs(lp->pos,wgs,true);
            //             ofs_lane.precision(16);
            //             ofs_lane<<"v "<<wgs[0]<<" "<<wgs[1]<<" "<<wgs[2]<<std::endl;
            //             line_lane_map[lane_id].push_back(lane_point_idx);
            //             lane_point_idx +=1;
            //         }
            //     }
            // }
            // for(auto per_line:line_lane_map){
            //     if(per_line.second.size()<2){
            //         continue;
            //     }
            //     ofs_lane<<"l";
            //     for(auto point_idx_l:per_line.second){
            //         ofs_lane<<" "<<point_idx_l;
            //     }
            //     ofs_lane<<std::endl;
            // }
            // ofs_lane.close();

            session->save_debug_info("format_new_road");

            // 
            session->set_display_name("format_lane_group_debug");
            int add=0;
            for(auto p:debug_dis_connect_point)
            {
              auto log = session->add_debug_log(utils::DisplayInfo::POINT, "point:{}",add++);
              log->color = {255, 0, 0};
              auto ele= log->add(p.pos);
              ele.label.score=p.bind_dis;    
            }
            session->save_debug_info("format_lane_group_debug");
            
            return fsdmap::SUCC;
        }
    }
}
/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
