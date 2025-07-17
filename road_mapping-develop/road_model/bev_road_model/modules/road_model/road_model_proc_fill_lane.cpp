

#include "road_model_proc_fill_lane.h"

DEFINE_bool(fill_lane_enable, true, "fill_lane_enable");
DEFINE_bool(fill_lane_smooth_lane_enable, false, "fill_lane_smooth_lane_enable");
// DEFINE_bool(fill_lane_use_template_lc_enable, true, "fill_lane_use_template_lc_enable");
DEFINE_bool(fill_lane_debug_pos_enable, true, "fill_lane_debug_pos_enable");
DEFINE_bool(fill_lane_save_data_enable, true, "fill_lane_save_data_enable");
// DEFINE_bool(fill_lane_save_data_local_enable, false, "fill_lane_save_data_local_enable");
// DEFINE_bool(fill_lane_save_data_filter_lane_by_boundary_enable, false, "fill_lane_save_data_filter_lane_by_boundary_enable");
DEFINE_bool(fill_lane_save_data_learning, false, "fill_lane_save_data_learning");
DEFINE_bool(fill_lane_save_data_show_template_enable, false, "fill_lane_save_data_show_template_enable");
DEFINE_bool(fill_lane_pre_fill_enable, true, "fill_lane_pre_fill_enable");
DEFINE_bool(fill_lane_score_width_variance_by_length, false, "fill_lane_score_width_variance_by_length");
DEFINE_double(fill_lane_train_data_true_scope, 2, "fill_lane_train_data_true_scope");
DEFINE_double(fill_lane_train_data_true_radius, 5, "fill_lane_train_data_true_radius");
DEFINE_double(fill_lane_train_data_true_max_theta, 20, "fill_lane_train_data_true_max_theta");
DEFINE_double(fill_lane_train_data_max_num, 10000, "fill_lane_train_data_max_num");
DEFINE_double(fill_lane_learning_data_not_match_dft, 5, "fill_lane_learning_data_not_match_dft");
DEFINE_double(fill_lane_learning_data_not_length_dft, 10, "fill_lane_learning_data_not_length_dft");
DEFINE_double(fill_lane_train_data_true_score_threshold, 100, "fill_lane_train_data_true_score_threshold");
DEFINE_double(fill_lane_train_sample_lc_max_size, 50000, "fill_lane_train_sample_lc_max_size");
DEFINE_double(fill_lane_fill_road_dis_lambda, 0.5, "fill_lane_fill_road_dis_lambda");
DEFINE_double(fill_lane_line_score_length_lambda, 100, "fill_lane_line_score_length_lambda");
DEFINE_double(fill_lane_line_score_length_min_factor, 0.5, "fill_lane_line_score_length_min_factor");
DEFINE_double(fill_lane_line_score_width_variance_lambda, 100, "fill_lane_line_score_width_variance_lambda");
DEFINE_double(fill_lane_line_score_width_standard_lambda, 40, "fill_lane_line_score_width_standard_lambda");
DEFINE_double(fill_lane_line_score_width_trangle_lambda, 3, "fill_lane_line_score_width_trangle_lambda");
DEFINE_double(fill_lane_line_score_width_standard, 3.75, "fill_lane_line_score_width_standard");
DEFINE_double(fill_lane_line_score_theta_lambda, 10, "fill_lane_line_score_theta_lambda");
DEFINE_double(fill_lane_line_score_gap_variance_lambda, 0.3, "fill_lane_line_score_gap_variance_lambda");
DEFINE_double(fill_lane_calc_dis_threshold, 50, "fill_lane_calc_dis_threshold");
DEFINE_double(fill_lane_calc_front_dis, 50, "fill_lane_calc_front_dis");
DEFINE_double(fill_lane_calc_back_dis, 50, "fill_lane_calc_back_dis");
DEFINE_double(fill_lane_calc_sample_gap, 2, "fill_lane_calc_sample_gap");
DEFINE_double(fill_lane_width_scale_factor, 100, "fill_lane_width_scale_factor");
DEFINE_double(fill_lane_gap_variance_factor, 100, "fill_lane_gap_variance_factor");
DEFINE_double(fill_lane_same_edge_dis_threshold, 0.3, "fill_lane_same_edge_dis_threshold");
DEFINE_double(fill_lane_same_edge_theta_threshold, 20, "fill_lane_same_edge_theta_threshold");
DEFINE_double(fill_lane_same_lane_dis_threshold, 10, "fill_lane_same_lane_dis_threshold");
DEFINE_double(fill_lane_same_lane_theta_threshold, 30, "fill_lane_same_lane_theta_threshold");
DEFINE_double(fill_lane_same_lane_min_width_threshold, 0.3, "fill_lane_same_lane_min_width_threshold");
DEFINE_double(fill_lane_same_lane_max_width_threshold, 1, "fill_lane_same_lane_max_width_threshold");
DEFINE_double(fill_lane_gap_base_valid_scope, 0.001, "fill_lane_gap_base_valid_scope");
DEFINE_double(fill_lane_gap_base_lambda, 1, "fill_lane_same_lane_width_threshold");
DEFINE_double(fill_lane_valid_lane_length, 2, "fill_lane_valid_lane_length");
DEFINE_double(fill_lane_valid_lane_min_width_threshold, 2, "fill_lane_valid_lane_min_width_threshold");
DEFINE_double(fill_lane_spread_valid_lc_scope, 25, "fill_lane_spread_valid_lc_scope");
DEFINE_double(fill_lane_max_inter_length, 15, "fill_lane_max_inter_length");
DEFINE_double(fill_lane_get_next_lc_radius, 50, "fill_lane_get_next_lc_radius");
DEFINE_double(fill_lane_get_next_lc_theta_thres, 10, "fill_lane_get_next_lc_theta_thres");
DEFINE_int32(fill_lane_gap_pow_factor, 3, "fill_lane_gap_pow_factor");
DEFINE_int32(fill_lane_get_max_score_variance_sigma, 1, "fill_lane_get_max_score_variance_sigma");
DEFINE_int64(fill_lane_lc_get_lane_line_base_max_size, 1, "fill_lane_lc_get_lane_line_base_max_size");
DEFINE_int64(fill_lane_lc_get_lane_line_random_max_size, 1, "fill_lane_lc_get_lane_line_random_max_size");
DEFINE_int64(fill_lane_lc_get_lane_line_score_max_size, 1, "fill_lane_lc_get_lane_line_score_max_size");
DECLARE_double(match_lane_feature_line_sample_min_dis_threshold);
DECLARE_double(match_lane_feature_line_sample_max_dis_theta_threshold);
DECLARE_double(gen_lane_line_edge_score_thres);

namespace fsdmap
{
    namespace road_model
    {

        fsdmap::process_frame::PROC_STATUS RoadModelProcFillLane::proc(
            RoadModelSessionData *session)
        {
            if (!FLAGS_fill_lane_enable)
            {
                return fsdmap::process_frame::PROC_STATUS_DISABLE;
            }
            session->enable_debug_pos = FLAGS_fill_lane_debug_pos_enable;

            // 给lane line 计算个置信度然后排序
            CHECK_FATAL_PROC(sort_lane_line(session), "sort_lane_line");

            // 从置信度最高的开始的开始在轨迹点范围内填充，互斥原则
            // CHECK_FATAL_PROC(fill_road_by_spread(session), "fill_road_by_spread");

            // 从置信度最高的开始的开始在轨迹点范围内填充，互斥原则
            CHECK_FATAL_PROC(prev_fill_road(session), "prev_fill_road");

            // 从置信度最高的开始的开始在轨迹点范围内填充，互斥原则
            CHECK_FATAL_PROC(fill_road_with_lane_line(session), "make_lane_sample_group_tree");

            // 模板化内容对接
            CHECK_FATAL_PROC(fill_road_with_template(session), "fill_road_with_template");
            //
            //
            CHECK_FATAL_PROC(reline_road_segment(session), "reline_road_segment");
            // // 根据已经填充的lane前后重新串线
            CHECK_FATAL_PROC(reline_lane_sample(session), "reline_lane_sample");

            CHECK_FATAL_PROC(process_fork_merge(session, true), "process_fork_merge");

            CHECK_FATAL_PROC(process_fork_merge(session, false), "process_fork_merge");

            CHECK_FATAL_PROC(fresh_multi_relation(session), "fresh_multi_relation");

            // 对于无边界的部分进行过滤
            // CHECK_FATAL_PROC(filter_lane_without_boundary(session), "filter_lane_without_boundary");

            // 保存到结果用于观察
            CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");

            return fsdmap::process_frame::PROC_STATUS_SUCC;
        }

        int RoadModelProcFillLane::fresh_multi_relation(RoadModelSessionData *session)
        {
            for (auto &road_segment : session->road_segment_list)
            {
                for (auto &poss : road_segment->pos_sample_list)
                {
                    session->debug_pos(poss->pos);
                    if (poss->invalid())    // 此处不应该被注释，待排查
                    {
                        continue;
                    }
                    for (auto &lc : poss->lane_center_list)
                    {

                        if (lc->invalid())
                        {
                            continue;
                        }
                        if (lc->fill_next != NULL)
                        {
                            lc->context.set_next(lc->fill_next);
                        }

                        if (lc->fill_prev != NULL)
                        {
                            lc->context.set_prev(lc->fill_prev);
                        }
                    }
                }
            }

            return fsdmap::SUCC;
        }

        bool RoadModelProcFillLane::is_conflict(RoadModelSessionData *session,
                                                LaneCenterFeature *left, LaneCenterFeature *right)
        {
            double dis_lambda = FLAGS_fill_lane_fill_road_dis_lambda; //  0.5
            // if (left->is_same_src(right)) {
            //     return true;
            // }
            // if (left->group_id > 0 && left->group_id == right->group_id) {
            //     // 同组的不冲突, 主要对于多车道的情况
            //     return false;
            // }
            auto &lls = left;
            auto &rls = right;
            Eigen::Vector3d l_center_pos = lls->pos;
            Eigen::Vector3d r_center_pos = rls->pos;
            double l_width = lls->width;
            double r_width = rls->width;
            if (left->src_status == 3)
            {
                l_width = lls->raw_width;
            }

            if (right->src_status == 3)
            {
                r_width = rls->raw_width;
            }
            // if (lls->match_level.lane_num > 1) {
            //     l_center_pos = lls->org_center.pos;
            //     l_width = lls->org_center.width;
            // }
            // if (rls->match_level.lane_num > 1) {
            //     r_center_pos = rls->org_center.pos;
            //     r_width = rls->org_center.width;
            // }
            double iou = alg::segment_iou(left->get_left(), left->get_right(),
                                          right->get_left(), right->get_right());

            double dis_1 = alg::calc_vertical_dis(l_center_pos, r_center_pos, rls->dir);
            double dis_2 = alg::calc_vertical_dis(r_center_pos, l_center_pos, lls->dir);
            double dis = std::min(dis_1, dis_2);
            double avg_width = (l_width + r_width) / 2;
            // int64_t l_line_id = left->line == NULL ? -1 : left->line->id;
            // int64_t r_line_id = right->line == NULL ? -1 : right->line->id;
            bool match = false;
            if (dis + dis_lambda < avg_width)
            { // 两个车道中心点之间的距离小于车道宽度，这两个属于一个车道
                match = true;
            }
            DLOG_POINT2(left->pos, right->pos,
                        "judge conflict[match={}, dis={}, avg_width={}, iou={}",
                        match, dis, avg_width, iou);
            return match;
        }

        // 判断lc是否与lane_sample_list内的某个车道中心点同属一个车道
        bool RoadModelProcFillLane::has_conflict(RoadModelSessionData *session,
                                                 LaneCenterFeature *lc, KeyPose *poss,
                                                 std::vector<LaneCenterFeature *> &lane_sample_list)
        {
            for (auto &filled : lane_sample_list)
            {
                if (is_conflict(session, filled, lc))
                { // lc与filled是否是一个车道内的车道中心点
                    filled->conflict_vec.push_back(lc);
                    return true;
                }
            }
            return false;
        }

        bool sort_lc_by_score_and_level1(const LaneCenterFeature *l, const LaneCenterFeature *r)
        {
            return l->context.max_length > r->context.max_length;
        }

        // 车道数量一样，车道类型一样，分数高的排前面；
        // 车道数量一样，车道类型不一样，一车道的正常车道排前面；
        // 车道数量不一样，车道数量少的排前面
        bool sort_lc_by_score_and_level(const LaneCenterFeature *l, const LaneCenterFeature *r)
        {
            bool l_has_lane = l->line_param.lane_size > 0;
            bool r_has_lane = r->line_param.lane_size > 0;
            if (l_has_lane != r_has_lane)
            {
                return l_has_lane;
            }
            // if (l->line_param.lane_size > 0 && r->line_param.lane_size == 0) {
            //     return true;
            // }
            // if (l->line_param.lane_size == 0 && r->line_param.lane_size > 0) {
            //     return false;
            //
            // }
            if (l->src_status != r->src_status)
            {
                return l->src_status == 2;
            }

            if (l->match_level.lane_num == r->match_level.lane_num)
            {
                if (l->match_level.lane_type == r->match_level.lane_type)
                { // 车道数量一样，车道类型一样，返回分数高的
                    return l->line_param.score > r->line_param.score;
                }
                return l->match_level.lane_type < r->match_level.lane_type; // 车道数量一样，车道类型不一样，返回一车道的正常车道
            }
            return l->match_level.lane_num < r->match_level.lane_num;
        }

        void RoadModelProcFillLane::fill_with_same_edge(RoadModelSessionData *session,
                                                        LaneCenterFeature *first, KeyPose *poss)
        {
            for (auto &lc : poss->lane_center_list)
            {
                if (lc == first)
                {
                    continue;
                }
                session->debug_pos(lc->pos);
                if (lc->invalid())
                {
                    continue;
                }
                if (!is_valid_lane(lc))
                {
                    continue;
                }
                if (lc->is_filled())
                {
                    continue;
                }
                if (!lc->has_same_edge(first))
                {
                    continue;
                }
                if (has_conflict(session, lc, poss, poss->score_filled_lane_sample))
                {
                    continue;
                }
                if (lc->match_level.lane_num != first->match_level.lane_num)
                {
                    continue;
                }
                lc->fill_status = 1;
                poss->score_filled_lane_sample.push_back(lc);
                fill_with_same_edge(session, lc, poss);
            }
        }

        bool RoadModelProcFillLane::lane_sample_on_same_edge(RoadModelSessionData *session,
                                                             LaneCenterFeature *left, LaneCenterFeature *right)
        {
            double dis_threshold = FLAGS_fill_lane_same_edge_dis_threshold;
            double theta_threshold = FLAGS_fill_lane_same_edge_theta_threshold;
            double dis_1 = alg::calc_vertical_dis(left->right->pos, right->left->pos, right->left->dir);
            double dis_2 = alg::calc_vertical_dis(right->left->pos, left->right->pos, left->right->dir);
            double dis = std::max(dis_1, dis_2);
            double theta = alg::calc_theta(left->right->dir, right->left->dir);
            bool ret = true;
            do
            {
                if (left->match_level.lane_num != right->match_level.lane_num)
                {
                    ret = true;
                    break;
                }
                if (dis > dis_threshold)
                {
                    ret = false;
                    break;
                }
                if (theta > theta_threshold)
                {
                    ret = false;
                    break;
                }
            } while (0);
            DLOG_POINT2(left->pos, right->pos, "on same edge[dis={}, theta={}]",
                        dis, theta);
            return ret;
        }

        // int RoadModelProcFillLane::fill_road_by_spread(RoadModelSessionData* session) {
        //     for (int64_t i = 0; i < session->lane_center_list_all.size(); ++i) {
        //         auto &lc = session->lane_center_list_all[i];
        //         auto &poss = lc->key_pose;
        //         session->debug_pos(poss->pos);
        //
        //     }
        //     return fsdmap::SUCC;
        // }

        // 遍历session->road_segment_list里面的每个位置poss，在poss范围内的lane_center_list
        // 选出每个车道的唯一车道中心点（直道应该只有一个点，对于弯道会有几个点）
        int RoadModelProcFillLane::prev_fill_road(RoadModelSessionData *session)
        {
            if (!FLAGS_fill_lane_pre_fill_enable)
            {
                return fsdmap::SUCC;
            }
            double width_scale = FLAGS_fill_lane_width_scale_factor; // 100
            // 从lane_center_list里面选出每个车道的唯一车道中心点（直道应该只有一个点，对于弯道会有几个点）
            for (auto &road_segment : session->road_segment_list)
            {
                for (auto &poss : road_segment->pos_sample_list)
                {
                    session->debug_pos(poss->pos);
                    if (poss->invalid())
                    {
                        continue;
                    }
                    // 防止错位导致feature过滤无效的问题，需要覆盖gap
                    // 先把共边的都fill
                    for (auto &lc : poss->lane_center_list)
                    {
                        session->debug_pos(lc->pos);
                        if (!is_valid_lane(lc))
                        {
                            continue;
                        }
                        if (lc->invalid())
                        {
                            continue;
                        }
                        // 判断lc是否与poss->pre_filled_lane_sample内的某个车道中心点同属一个车道
                        if (has_conflict(session, lc, poss, poss->pre_filled_lane_sample))
                        {
                            continue;
                        }
                        poss->pre_filled_lane_sample.push_back(lc);
                    }
                }
            }
            std::vector<double> width_vec;
            for (auto &road_segment : session->road_segment_list)
            {
                for (int i = 0; i < road_segment->pos_sample_list.size(); ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    int start_index = i - 2;
                    int end_index = i + 2;
                    if (start_index < 0)
                    {
                        start_index = 0;
                    }
                    if (end_index >= road_segment->pos_sample_list.size())
                    {
                        end_index = road_segment->pos_sample_list.size() - 1;
                    }

                    for (int j = start_index; j < end_index; ++j)
                    {
                        auto &tmp_poss = road_segment->pos_sample_list[j];
                        width_vec.clear();
                        for (auto &lc : tmp_poss->pre_filled_lane_sample)
                        {
                            if (lc->invalid())
                            {
                                continue;
                            }
                            if (lc->src_status != 2)
                            {
                                continue;
                            }
                            width_vec.push_back(lc->width);
                        }
                    }
                    if (width_vec.size() <= 1)
                    {
                        continue;
                    }
                    poss->lane_param.avg_lane_width = alg::calc_median(width_vec);

                    for (auto &lc : poss->lane_center_list)
                    {
                        session->debug_pos(lc->pos);
                        lc->line_param.avg_width = poss->lane_param.avg_lane_width * width_scale;
                        calc_lane_line_score(session, lc, lc->line_param, 2);
                    }
                    SORT(poss->lane_center_list, sort_lc_by_score_and_level);
                }
            }

            return fsdmap::SUCC;
        }

        int RoadModelProcFillLane::fill_road_with_template(RoadModelSessionData *session)
        {
            for (auto &road_segment : session->road_segment_list)
            {
                for (auto &poss : road_segment->pos_sample_list)
                {
                    session->debug_pos(poss->pos);
                    bool use_template = false;
                    poss->filled_lane_sample.clear();
                    poss->filled_lane_sample.insert(poss->filled_lane_sample.end(),
                                                    poss->score_filled_lane_sample.begin(), poss->score_filled_lane_sample.end());
                }
            }
            return fsdmap::SUCC;
        }

        // 遍历session->road_segment_list的每个道路段road_segment，对road_segment->pos_sample_list里的每个poss,
        // 从该poss的lane_center_list里面选出每个车道的唯一车道中心点（直道应该只有一个点，对于弯道会有几个点）
        // 这些车道中心点按照从最左车道到最右车道排序
        int RoadModelProcFillLane::fill_road_with_lane_line(RoadModelSessionData *session)
        {
            for (auto &road_segment : session->road_segment_list)
            {
                for (auto &poss : road_segment->pos_sample_list)
                {
                    session->debug_pos(poss->pos);
                    // if (poss->invalid()) {       // 此处不应该被注释，待排查
                    //     continue;
                    // }
                    // // 防止错位导致feature过滤无效的问题，需要覆盖gap
                    // // if (FLAGS_fill_lane_use_template_lc_enable) {
                    // //     poss->lane_center_list.insert(poss->lane_center_list.begin(),
                    // //             poss->template_lane_sample.begin(), poss->template_lane_sample.end());
                    // // }
                    // // 先把共边的都fill
                    for (auto &lc : poss->lane_center_list)
                    {
                        session->debug_pos(lc->pos);
                        // if (!is_valid_lane(lc)) {
                        //     continue;
                        // }
                        if (lc->invalid())
                        {
                            continue;
                        }
                        if (lc->is_filled())
                        {
                            continue;
                        }
                        if (has_conflict(session, lc, poss, poss->score_filled_lane_sample))
                        {
                            continue;
                        }
                        // 记录最左和最右
                        double dis = alg::calc_dis(poss->pos, lc->pos, true);
                        int is_left = alg::judge_left(lc->pos, poss->pos, poss->dir); // 1: 表示lc->pos在poss->pos的右侧
                        // if (is_left < 0 && poss->lane_scope.left_dis < dis) {
                        //     poss->lane_scope.left = lc;
                        //     poss->lane_scope.left_dis = dis;
                        // } else if (is_left >= 0 && poss->lane_scope.right_dis < dis) {
                        //     poss->lane_scope.right = lc;
                        //     poss->lane_scope.right_dis = dis;
                        // }

                        lc->fill_status = 1;
                        poss->score_filled_lane_sample.push_back(lc);
                        // 先把共边的都fill, 按照match_level分别对待
                        // fill_with_same_edge(session, lc, poss);
                    }

                    // // 补全单线的情况
                    // if (poss->score_filled_lane_sample.size() == 0) {
                    //     double max_length  = 0;
                    //     LaneLineSample * valid_fls = NULL;
                    //     for (auto &fls : poss->lane_line_feature_sample_list) {
                    //         fls->fill_status = 0;
                    //         if (fls->line->length > max_length) {
                    //             valid_fls = fls;
                    //             max_length = valid_fls->line->length;
                    //         }
                    //     }
                    //     if (valid_fls != NULL) {
                    //         valid_fls->fill_status = 1;
                    //         // DLOG_POINT(valid_fls->pos, "fill fls[line_id={}, id={}]",
                    //         //        valid_fls->line_id, valid_fls->id);
                    //     }
                    // }

                    // DLOG_POS(poss->pos, "finish fill pos[pos={}]", (uint64_t)poss);
                    // 对poss->score_filled_lane_sample的车道中心点按照从最左车道到最右车道排序
                    SORT(poss->score_filled_lane_sample,
                         [&poss](const LaneCenterFeature *l, const LaneCenterFeature *r) -> bool
                         {
                             return alg::judge_left(l->pos, r->pos, poss->dir) < 0;
                         });
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFillLane::sort_lane_line(RoadModelSessionData *session)
        {
            int64_t total_lc_size = 0;

            for (auto &road_segment : session->road_segment_list)
            {
                for (auto &poss : road_segment->pos_sample_list)
                {
                    if (poss->invalid())
                    {
                        continue;
                    }
                    // DLOG_POS(pos->pos, "start calc lane sample score");
                    for (int j = 0; j < poss->lane_center_list.size(); ++j)
                    {
                        auto lc = poss->lane_center_list[j];
                        session->thread_pool->schedule([&, j, lc, poss, session, this](utils::ProcessBar *bar)
                                                       {
                session->debug_pos(lc->pos);
                GenLaneLineParam param;
                param.lane_size = gen_lane_line_with_lc(session, poss, lc, param);
                if (param.lane_size > 10) {
                    int a = 1;
                }
                if (param.lane_size == 0) {
                    return;
                }
                for (auto &ll : param.lane_line_ret) {
                    calc_lane_line_param(session, ll.get(), param); // 统计ll对应的车道平均宽度、方差等信息
                    calc_lane_line_score(session, lc, ll->line_param, 1); // 根据ll->line_param里面的参数计算车道中心线得分
                }
                bool has_valid = false;
                
                SORT(param.lane_line_ret, 
                        [](const std::shared_ptr<LaneCenterLine> &l, const std::shared_ptr<LaneCenterLine> &r) {
                    return l->line_param.score > r->line_param.score;
                    });
                // 保留分数最高的
                auto &max_score_ll = param.lane_line_ret.front();
                // session->add_vec(session->lane_center_line_ptr, max_score_ll, true);

                lc->line_param = max_score_ll->line_param;  // lane_line_ret里面score最高的赋给他
                lc->line_param.lane_size = param.lane_size;

                // // std::swap(lc->line, max_score_ll);
                // // stat log
                // // poss_lane_line_size += lc_size;
                bar->num_biz += param.lane_size; });
                    }
                }
            }
            session->thread_pool->wait(2, "sort lane_line");

            for (auto &road_segment : session->road_segment_list)
            {
                for (auto &poss : road_segment->pos_sample_list)
                {
                    session->debug_pos(poss->pos);
                    VEC_INSERT_ALL(session->lane_center_list_all, poss->lane_center_list);
                    SORT(poss->lane_center_list, sort_lc_by_score_and_level);
                }
            }

            SORT(session->lane_center_list_all, sort_lc_by_score_and_level);
            return fsdmap::SUCC;
        }

        //  从lc->raw_from_lc往前继续寻找车道中心线prev_line，
        //  lc->raw_to_lc往后继续寻找车道中心线next_line，
        // 把prev_line与next_line组合形成多条新的车道中心线
        int64_t RoadModelProcFillLane::gen_lane_line_with_lc(RoadModelSessionData *session,
                                                             KeyPose *poss, LaneCenterFeature *lc, GenLaneLineParam &param)
        {
            double scope = FLAGS_fill_lane_calc_dis_threshold;                          // 50
            int64_t base_max_size = FLAGS_fill_lane_lc_get_lane_line_base_max_size;     // 1
            int64_t random_max_size = FLAGS_fill_lane_lc_get_lane_line_random_max_size; // 1
            int64_t score_max_size = FLAGS_fill_lane_lc_get_lane_line_score_max_size;   // 1
            std::vector<std::shared_ptr<LaneCenterLine>> prev_lane_line;
            std::vector<std::shared_ptr<LaneCenterLine>> next_lane_line;
            double from_dis = alg::calc_dis(lc->pos, lc->raw_from_lc->pos); // 产生该车道中心点lc的车道线的起始点raw_from_lc跟lc的距离
            double to_dis = alg::calc_dis(lc->pos, lc->raw_to_lc->pos);
            DLOG_POINT(lc->pos, "start lc lane candidate[prev_dis={:.2f}, next_dis={:.2f}]",
                       from_dis, to_dis);

            param.sample_mode = 1;
            for (int i = 0; i < base_max_size; ++i)
            {
                auto prev_line = std::make_shared<LaneCenterLine>();
                // lc->raw_from_lc往前继续寻找车道中心线
                if (!gen_lane_line1(session, lc->raw_from_lc, scope - from_dis, false, param, prev_line.get()))
                {
                    break;
                }
                prev_lane_line.push_back(prev_line);
            }
            for (int i = 0; i < score_max_size; ++i)
            {
                auto next_line = std::make_shared<LaneCenterLine>();
                // lc->raw_to_lc往后继续寻找车道中心线
                if (!gen_lane_line1(session, lc->raw_to_lc, scope - to_dis, true, param, next_line.get()))
                {
                    break;
                }
                next_lane_line.push_back(next_line);
            }
            param.sample_mode = 2;
            for (int i = 0; i < random_max_size; ++i)
            {
                auto prev_line = std::make_shared<LaneCenterLine>();
                if (!gen_lane_line1(session, lc->raw_from_lc, scope - from_dis, false, param, prev_line.get()))
                {
                    break;
                }
                prev_lane_line.push_back(prev_line);
            }
            for (int i = 0; i < random_max_size; ++i)
            {
                auto next_line = std::make_shared<LaneCenterLine>();
                if (!gen_lane_line1(session, lc->raw_to_lc, scope - to_dis, true, param, next_line.get()))
                {
                    break;
                }
                next_lane_line.push_back(next_line);
            }
            param.sample_mode = 3;
            for (int i = 0; i < base_max_size; ++i)
            {
                auto prev_line = std::make_shared<LaneCenterLine>();
                if (!gen_lane_line1(session, lc->raw_from_lc, scope - from_dis, false, param, prev_line.get()))
                {
                    break;
                }
                prev_lane_line.push_back(prev_line);
            }
            for (int i = 0; i < base_max_size; ++i)
            {
                auto next_line = std::make_shared<LaneCenterLine>();
                if (!gen_lane_line1(session, lc->raw_to_lc, scope - to_dis, true, param, next_line.get()))
                {
                    break;
                }
                next_lane_line.push_back(next_line);
            }

            DLOG_POINT(lc->pos, "lc candidate size[prev_size={}, next_size={}]",
                       prev_lane_line.size(), next_lane_line.size());
            if (next_lane_line.size() < 1)
            {
                int a = 1;
            }
            // 拼接前后line
            for (int i = 0; i < prev_lane_line.size(); ++i)
            {
                auto &prev_line = prev_lane_line[i];
                std::reverse(prev_line->list.begin(), prev_line->list.end());
                for (int j = 0; j < next_lane_line.size(); ++j)
                {
                    auto &next_line = next_lane_line[j];
                    auto curr_line = session->add_ptr(param.lane_line_ret);
                    curr_line->list.reserve(prev_line->list.size() + next_line->list.size());
                    curr_line->list.insert(curr_line->list.end(), prev_line->list.begin(), prev_line->list.end());
                    // curr_line->push_back(lc);
                    curr_line->list.insert(curr_line->list.end(), next_line->list.begin(), next_line->list.end());
                    if (prev_line->list.size() == 0 || next_line->list.size() == 0)
                    {
                        int a = 1;
                    }
                    curr_line->start_lc = lc;
                    curr_line->center_index = prev_line->list.size() - 1;
                    curr_line->center_offset = from_dis;
                    curr_line->line_param.length = prev_line->line_param.length + next_line->line_param.length;
                    if (curr_line->line_param.length > 80)
                    {
                        int a = 1;
                    }
                    DLOG_POINT(lc->pos, "lc candidate[length={:.2f}, lc_size={}]",
                               curr_line->line_param.length, curr_line->list.size());
                }
            }
            if (param.lane_line_ret.size() < 1)
            {
                int a = 1;
            }
            return param.lane_line_ret.size();
        }

        bool RoadModelProcFillLane::gen_lane_line1(RoadModelSessionData *session, LaneCenterFeature *lc,
                                                   double scope, bool front, GenLaneLineParam &param, LaneCenterLine *curr_line)
        {
            double edge_score_thres = FLAGS_gen_lane_line_edge_score_thres;
            // 考虑到数量太大，所以优先选择gap大的分支，每次处理当前的候选，同时将下一次的gap记录，用于选取下次的记录分支
            auto curr_lc = lc;
            // auto curr_line = std::make_shared<LaneCenterLine>();
            curr_line->list.push_back(curr_lc);
            // std::vector<LaneCenterLine*> next_line;
            auto &all_vec = front ? curr_lc->context.all_next : curr_lc->context.all_prev;
            bool ret_bool = false;
            if (all_vec.size() == 0 || scope < 0)
            {
                auto status = param.single_status_map[lc];
                param.single_status_map[lc] = status + 1;
                if (status == 0)
                {
                    return true;
                }
                return false;
            }
            else if (all_vec.size() == 1)
            {
                auto &next_lc_ptr = all_vec[0];
                auto &next_lc = next_lc_ptr.src;
                double dis = alg::calc_dis(curr_lc->pos, next_lc->pos);
                auto status = param.status_map[std::make_pair(curr_lc, next_lc)];
                param.status_map[std::make_pair(curr_lc, next_lc)] = status + 1;
                curr_line->line_param.length += dis;                                               // 累加车道线的长度
                ret_bool = gen_lane_line1(session, next_lc, scope - dis, front, param, curr_line); // 递归
                if (status == 0)
                {
                    return true;
                }
                return ret_bool;
            }
            else
            {
                // 只有在对多时，且下面再无候选时，才要假如used
                int min_status = 10000000;
                // 选出被前面匹配过最少的
                for (auto &next_lc_ptr : all_vec)
                {
                    auto &next_lc = next_lc_ptr.src;
                    auto status = param.status_map[std::make_pair(curr_lc, next_lc)];
                    min_status = std::min(min_status, status);
                }
                int return_index = 0;
                std::vector<int> can_vec;
                can_vec.reserve(all_vec.size());
                double max_score = 0;
                int max_score_index = 0;
                // 选出被前面匹配过最少的后继节点里面，score最高的
                for (int i = 0; i < all_vec.size(); ++i)
                {
                    auto &next_lc_ptr = all_vec[i];
                    auto &next_lc = next_lc_ptr.src;
                    auto status = param.status_map[std::make_pair(curr_lc, next_lc)];
                    if (status == min_status)
                    {
                        can_vec.push_back(i);
                        if (next_lc_ptr.score > max_score)
                        {
                            max_score_index = i;
                            max_score = next_lc_ptr.score;
                        }
                    }
                }
                if (param.sample_mode == 1)
                { // 选出被前面匹配过最少的后继节点里面的第一个
                    return_index = can_vec.front();
                }
                else if (param.sample_mode == 2)
                { // 选出被前面匹配过最少的后继节点里面，score最高的
                    return_index = max_score_index;
                }
                else if (param.sample_mode == 3)
                { // 随机选择
                    int choose_index = rand() % can_vec.size();
                    return_index = can_vec[choose_index];
                }
                else
                {
                    return false;
                }
                auto &next_lc = all_vec[return_index].src;
                double dis = alg::calc_dis(curr_lc->pos, next_lc->pos);
                auto status = param.status_map[std::make_pair(curr_lc, next_lc)];
                param.status_map[std::make_pair(curr_lc, next_lc)] = status + 1;
                curr_line->line_param.length += dis;
                ret_bool = gen_lane_line1(session, next_lc, scope - dis, front, param, curr_line);
                if (can_vec.size() > 1)
                {
                    ret_bool = true;
                }
                return ret_bool;
            }
            return ret_bool;
        }

        int RoadModelProcFillLane::gen_lane_line(RoadModelSessionData *session, LaneCenterFeature *lc,
                                                 double scope, bool front, GenLaneLineParam &param, std::vector<std::shared_ptr<LaneCenterLine>> &ret)
        {
            // 考虑到数量太大，所以优先选择gap大的分支，每次处理当前的候选，同时将下一次的gap记录，用于选取下次的记录分支
            auto curr_lc = lc;
            double total_dis = 0;
            // auto curr_line = session->add_ptr(session->tmp_lane_line_list_ptr);
            auto curr_line = std::make_shared<LaneCenterLine>();
            curr_line->list.push_back(curr_lc);
            std::vector<std::shared_ptr<LaneCenterLine>> next_line;
            while (true)
            {
                LaneCenterFeature *next_single_lc = NULL;
                // if (curr_lc->is_raw_start(!front)) {
                // auto &all_vec = front ? curr_lc->raw_from_lc->context.all_next : curr_lc->raw_from_lc->context.all_prev;
                auto &all_vec = front ? curr_lc->context.all_next : curr_lc->context.all_prev;
                if (all_vec.size() == 0)
                {
                    break;
                }
                else if (all_vec.size() == 1)
                {
                    auto &next_single_ptr = all_vec[0];
                    next_single_lc = next_single_ptr.src;
                    if (front)
                    {
                        curr_line->list.push_back(next_single_lc);
                    }
                    else
                    {
                        curr_line->list.insert(curr_line->list.begin(), next_single_lc);
                    }
                    double dis = alg::calc_dis(curr_lc->pos, next_single_lc->pos);
                    total_dis += dis;
                    if (total_dis > scope)
                    {
                        break;
                    }
                    curr_lc = next_single_lc;
                    continue;
                }
                for (auto &next_lc_ptr : all_vec)
                {
                    auto &next_lc = next_lc_ptr.src;
                    double dis = alg::calc_dis(curr_lc->pos, next_lc->pos);
                    total_dis += dis;
                    if (total_dis > scope)
                    {
                        continue;
                    }
                    gen_lane_line(session, next_lc, scope - total_dis, front, param, next_line);
                }
                break;
            }
            if (next_line.size() == 0)
            {
                ret.push_back(curr_line);
                return fsdmap::SUCC;
            }
            // 拼接
            for (int i = 0; i < next_line.size(); ++i)
            {
                auto &concat_line = next_line[i];
                if (front)
                {
                    concat_line->list.insert(concat_line->list.begin(), curr_line->list.begin(), curr_line->list.end());
                }
                else
                {
                    concat_line->list.insert(concat_line->list.end(), curr_line->list.begin(), curr_line->list.end());
                }
                ret.push_back(concat_line);
            }
            return fsdmap::SUCC;
        }

        bool RoadModelProcFillLane::is_valid_lane(LaneCenterFeature *lc)
        {
            double valid_lane_length = FLAGS_fill_lane_valid_lane_length;
            double min_width_threshold = FLAGS_fill_lane_valid_lane_min_width_threshold;
            // auto &line = lc->line;
            // if (lc->src_status >= 1 && lc->fresh_status >= 1) {
            //     return true;
            // }
            if (lc->src_status != 3)
            {
                return false;
            }
            if (lc->line_param.lane_size == 0)
            {
                return false;
            }
            if (lc->line_param.length < valid_lane_length)
            {
                return false;
            }
            if (lc->line_param.max_width < min_width_threshold)
            {
                return false;
            }
            return true;
        }

        // 从lane_line->center_index处开始往前按2m的间距，生成车道中心采样点，最长采样距离不超过50m，
        // 从lane_line->start_lc处开始往后按2m的间距，生成车道中心采样点，最长采样距离不超过50m，
        // 其实就是对产生lane_line的原始车道线，按2m间距根据左右车道线点重新生成车道中心线sample_line，最大长度不超过100m
        int RoadModelProcFillLane::sample_lane_line(RoadModelSessionData *session,
                                                    LaneCenterLine *lane_line, LaneCenterLine *sample_line, GenLaneLineParam &param)
        {
            double scope = FLAGS_fill_lane_calc_dis_threshold; // 50
            double gap = FLAGS_fill_lane_calc_sample_gap;      // 2
            // 对前后采样，期望能得到一致的结果
            int lc_index = lane_line->center_index;
            // 当前lc并没有加入ll，只加入了他的原始前后，所以反推需要offset
            auto curr_lc = lane_line->start_lc;
            double center_offset = lane_line->center_offset;
            double total_length = 0;
            double curr_dis = 0;
            // 从lane_line->center_index处开始往前按2m的间距，生成车道中心采样点，最长采样距离不超过scope
            while (true)
            {
                if (lc_index < 0)
                {
                    break;
                }
                auto next_lc = lane_line->list[lc_index--];
                double dis = alg::calc_dis(curr_lc->pos, next_lc->pos);
                curr_dis += dis;
                // 在curr_lc与next_lc节点之间按2m的间距，生成车道中心采样点
                while (curr_dis >= gap)
                {
                    total_length += gap;
                    if (total_length > scope)
                    {
                        break;
                    }
                    double offset = curr_dis - gap;
                    Eigen::Vector3d break_pos = next_lc->pos + next_lc->dir * offset;
                    auto break_dir = alg::get_vertical_dir(next_lc->dir);
                    auto sample_lc = session->sample_new_lc(next_lc, curr_lc, break_pos, break_dir, &param);
                    sample_line->list.insert(sample_line->list.begin(), sample_lc);
                    sample_line->line_param.length += gap;
                    curr_dis -= gap;
                }
                if (total_length > scope)
                { // 离lane_line->start_lc的距离超过scope，不在采样
                    break;
                }
                curr_lc = next_lc;
            }
            // 因为放着是raw_to_lc, 所以取下一个
            lc_index = lane_line->center_index + 1;
            curr_lc = lane_line->start_lc;
            sample_line->center_index = sample_line->list.size();
            if (sample_line->list.size() > 25)
            {
                int a = 1;
            }
            sample_line->list.push_back(curr_lc);
            total_length = 0;
            curr_dis = 0;
            // 从lane_line->start_lc处开始往后按2m的间距，生成车道中心采样点，最长采样距离不超过scope
            while (true)
            {
                if (lc_index == lane_line->list.size())
                {
                    break;
                }
                auto next_lc = lane_line->list[lc_index++];
                double dis = alg::calc_dis(curr_lc->pos, next_lc->pos);
                curr_dis += dis;
                while (curr_dis >= gap)
                {
                    total_length += gap;
                    if (total_length > scope)
                    {
                        break;
                    }
                    double offset = dis - curr_dis + gap;
                    Eigen::Vector3d break_pos = curr_lc->pos + curr_lc->dir * offset;
                    auto break_dir = alg::get_vertical_dir(curr_lc->dir);
                    auto sample_lc = session->sample_new_lc(curr_lc, next_lc, break_pos, break_dir, &param);
                    sample_line->list.push_back(sample_lc);
                    sample_line->line_param.length += gap;
                    curr_dis -= gap;
                }
                if (total_length > scope)
                {
                    break;
                }
                curr_lc = next_lc;
            }
            sample_line->list.front()->prev = NULL;
            sample_line->list.back()->next = NULL;
            for (int i = 1; i < sample_line->list.size(); ++i)
            {
                auto &prev_lc = sample_line->list[i - 1];
                auto &next_lc = sample_line->list[i];
                prev_lc->next = next_lc;
                next_lc->prev = prev_lc;
            }
            if (sample_line->list.size() - sample_line->center_index > 26)
            {
                int a = 1;
            }
            DLOG_POINT(lane_line->start_lc->pos, "lc lane sample[length={:.2f}, list={}]",
                       sample_line->line_param.length, sample_line->list.size());
            return 0;
        }

        // 统计lane_line对应的车道平均宽度、方差等信息
        int RoadModelProcFillLane::calc_lane_line_param(RoadModelSessionData *session, LaneCenterLine *lane_line, GenLaneLineParam &param)
        {
            double front_dis = FLAGS_fill_lane_calc_front_dis;                    // 50
            double back_dis = FLAGS_fill_lane_calc_back_dis;                      // 50
            double width_scale = FLAGS_fill_lane_width_scale_factor;              // 100
            double variance_sigma = FLAGS_fill_lane_get_max_score_variance_sigma; // 1
            auto &lc = lane_line->start_lc;
            session->debug_pos(lc->pos);

            // lane_line->line_param.sample_lane = std::make_shared<LaneCenterLine>();
            // auto &sample_line = lane_line->line_param.sample_lane;
            auto sample_line = std::make_shared<LaneCenterLine>();
            // 对产生lane_line的原始车道线，按2m间距根据左右车道线点重新生成车道中心线sample_line，最大长度不超过100m
            sample_lane_line(session, lane_line, sample_line.get(), param);
            int lc_index = sample_line->center_index;
            auto &line_param = lane_line->line_param;

            line_param.length = sample_line->line_param.length;
            // 组装数据
            std::vector<double> width_vec;
            std::vector<double> all_gap_vec;
            all_gap_vec.reserve(40); // 计算某点到他前后节点组成的向量prev_next的距离,负数表示lc在prev左侧
            std::vector<double> all_width_vec;
            all_width_vec.reserve(41);
            double width_gap = 0;
            double next_gap = 0;
            line_param.min_width = 1000;
            for (int i = 0; i < sample_line->list.size(); ++i)
            {
                auto &calc_lc = sample_line->list[i];
                width_gap = calc_lc->width * width_scale;
                all_width_vec.push_back(width_gap);
                next_gap = format_center_gap(calc_prev_next_pos_gap(calc_lc, calc_lc->prev, calc_lc->next));
                all_gap_vec.push_back(next_gap);
                line_param.max_width = std::max(calc_lc->width, line_param.max_width);
                line_param.min_width = std::min(calc_lc->width, line_param.min_width);
            }
            // 将gap里的0向前补齐，
            double prev_gap = 0;
            for (int32_t i = 0; i < all_gap_vec.size(); ++i)
            {
                auto &gap = all_gap_vec[i];
                if (gap != 0)
                {
                    prev_gap = gap;
                }
                else
                {
                    all_gap_vec[i] = prev_gap;
                }
            }
            prev_gap = 0;
            for (int32_t i = all_gap_vec.size() - 1; i >= 0; --i)
            {
                auto &gap = all_gap_vec[i];
                if (gap != 0)
                {
                    prev_gap = gap;
                }
                else
                {
                    all_gap_vec[i] = prev_gap;
                }
            }

            int32_t calc_width_size = std::min(lc_index, (int)(all_width_vec.size() - lc_index));
            if (calc_width_size >= 3)
            {
                // 计算all_width_vec中[lc_index - calc_width_size, lc_index + calc_width_size + 1]的车道宽度中位数
                line_param.avg_width = alg::calc_median(all_width_vec, 1, 0.5,
                                                        lc_index - calc_width_size, lc_index + calc_width_size + 1);
            }
            else
            {
                if (lc_index > all_width_vec.size() / 2)
                {
                    line_param.avg_width = alg::calc_median(all_width_vec, 1, 0.5, lc_index - 4, 0);
                }
                else
                {
                    line_param.avg_width = alg::calc_median(all_width_vec, 1, 0.5, 0, lc_index + 5);
                }
            }
            line_param.front_avg_width = alg::calc_median(all_width_vec, 1, 0.5,
                                                          lc_index, lc_index + calc_width_size + 1);
            line_param.back_avg_width = alg::calc_median(all_width_vec, 1, 0.5,
                                                         lc_index - calc_width_size, lc_index);
            // 计算车道宽度方差
            line_param.width_variance = alg::calc_line_variance(all_width_vec, lc_index, variance_sigma);
            // line_param.width_variance = alg::calc_line_standard(all_width_vec, lc_index, variance_sigma);
            line_param.line_theta = calc_prev_next_pos_theta(lc, lc->prev, lc->next);
            line_param.center_gap_variance = alg::calc_variance(all_gap_vec, lc_index, variance_sigma);
            line_param.lc_width = lc->width * width_scale;
            // DLOG_POINT(lc->pos,
            //         "calc lane line sample param[length={}, width={}, width_variance={},"
            //         " front_avg_width={}, back_avg_width={}, radius={},"
            //         " center_gap={}, center_gap_variance={}]",
            //         line_param.length, lc->width, line_param.width_variance,
            //         line_param.front_avg_width, line_param.back_avg_width, line_param.radius,
            //         line_param.center_gap, line_param.center_gap_variance);
            return fsdmap::SUCC;
        }

        // 综合param.length_factor 、 param.width_factor *、param.dir_factor，计算车道得分
        double RoadModelProcFillLane::calc_lane_line_score(RoadModelSessionData *session,
                                                           LaneCenterFeature *lc, LaneLineParam &param, int times)
        {
            double length_lambda = FLAGS_fill_lane_line_score_length_lambda;                 // 100
            double length_min_factor = FLAGS_fill_lane_line_score_length_min_factor;         // 0.5
            double width_lambda = FLAGS_fill_lane_line_score_width_variance_lambda;          // 100
            double width_standard_lambda = FLAGS_fill_lane_line_score_width_standard_lambda; // 40
            double width_trangle_lambda = FLAGS_fill_lane_line_score_width_trangle_lambda;   // 3
            double width_standard = FLAGS_fill_lane_line_score_width_standard;               // 3.75
            double theta_lambda = FLAGS_fill_lane_line_score_theta_lambda;                   // 10
            double gap_variance_lambda = FLAGS_fill_lane_line_score_gap_variance_lambda;     // 0.3
            // 长度、宽度方差、曲率方差(距离方差)
            param.length_factor = 1 / (1 + pow(length_lambda / param.length, 2));
            // double length_factor = 1 / (1 + exp(length_lambda / length)) + length_min_factor;
            if (FLAGS_fill_lane_score_width_variance_by_length)
            {
                param.width_variance_factor = width_lambda / (width_lambda + param.width_variance / param.length);
            }
            else
            {
                param.width_variance_factor = width_lambda / (width_lambda + param.width_variance);
            }
            // 宽度系数考虑2个因素，1：是否标准宽度，2：前后宽度方差
            double tran_width_standard = param.avg_width;
            double width_standard_sigma = lc->match_level.lane_type == 1 ? width_standard_lambda : width_trangle_lambda;
            param.width_standard_factor = alg::calc_score_by_gaussian(
                fabs(param.lc_width - tran_width_standard), width_standard_sigma);
            if (lc->match_level.lane_type != 1)
            {
                param.width_factor = param.width_variance_factor;
            }
            else
            {
                param.width_factor = sqrt(param.width_variance_factor * param.width_standard_factor);
            }
            // gap系数考虑2个因素，1：前后gap方差，2：两边夹角
            param.theta_factor = theta_lambda / (theta_lambda + lc->theta);
            param.gap_factor = gap_variance_lambda / (gap_variance_lambda + param.center_gap_variance);
            param.dir_factor = sqrt(param.theta_factor * param.gap_factor);
            param.score = sqrt(param.length_factor * param.width_factor * param.dir_factor);
            // param.score = param.length;
            auto is_debug = session->debug_pos(lc->key_pose->pos);
            DLOG_POINT2(lc->key_pose->pos, lc->pos,
                        "calc lane line score[d={}, t={}, match_level={}, score={},"
                        " width_variance_factor={}, width_standard_factor={}, width_factor={},"
                        " gap_factor={}, theta_factor={}, dir_factor={}, length_factor={}]"
                        "[length={}, width={}, width_variance={},"
                        "avg_width={}, front_avg_width={}, back_avg_width={}, radius={},"
                        " center_gap={}, center_gap_variance={}]",
                        is_debug, times, lc->match_level.lane_type, param.score,
                        param.width_variance_factor, param.width_standard_factor, param.width_factor,
                        param.gap_factor, param.theta_factor, param.dir_factor, param.length_factor,
                        param.length, lc->width, param.width_variance,
                        param.avg_width, param.front_avg_width, param.back_avg_width, param.radius,
                        param.center_gap, param.center_gap_variance);

            return param.score;
        }

        double RoadModelProcFillLane::format_center_gap(double center_gap)
        {
            // 同向gap采用3次根号方缩小差距，适应拐弯的情况
            // 对向gap采用增加基数增加差距，突出来回扭的case
            double gap_pow_factor = FLAGS_fill_lane_gap_pow_factor; // 3
            gap_pow_factor = 1 / gap_pow_factor;
            double gap_base_lambda = FLAGS_fill_lane_gap_base_lambda;           // 1
            double gap_base_valid_scope = FLAGS_fill_lane_gap_base_valid_scope; // 0.001
            double gap_factor = FLAGS_fill_lane_gap_variance_factor;            // 100
            bool is_positive = center_gap > 0;
            if (fabs(center_gap) < gap_base_valid_scope)
            {
                return 0;
            }
            double dis = fabs(pow(fabs(center_gap * gap_factor), gap_pow_factor));
            dis += gap_base_lambda;
            return is_positive ? dis : -dis;
        }

        // 计算lc->pos到向量prev_next的距离,负数表示lc在prev左侧
        double RoadModelProcFillLane::calc_prev_next_pos_gap(LaneCenterFeature *lc,
                                                             LaneCenterFeature *prev, LaneCenterFeature *next)
        {
            if (prev != NULL && next != NULL)
            {
                Eigen::Vector3d dir;
                alg::calc_dir(next->pos, prev->pos, dir, true);
                double dis = alg::calc_vertical_dis(lc->pos, prev->pos, dir, true); // 计算lc->pos到向量dir的距离
                int32_t is_left = alg::judge_left(lc->pos, prev->pos, dir);         // lc->pos是否在prev->pos左侧
                if (is_left < 0)
                {
                    return -dis;
                }
                return dis;
            }
            return 0;
        }

        // 向量prev_next与lc->dir的夹角
        double RoadModelProcFillLane::calc_prev_next_pos_theta(LaneCenterFeature *lc,
                                                               LaneCenterFeature *prev, LaneCenterFeature *next)
        {
            if (prev != NULL && next != NULL)
            {
                Eigen::Vector3d dir;
                alg::calc_dir(next->pos, prev->pos, dir, true);
                double theta = alg::calc_theta(lc->dir, dir, true);
                return theta;
            }
            return 0;
        }

        int RoadModelProcFillLane::reline_road_segment(RoadModelSessionData *session)
        {
            for (auto &road_segment : session->road_segment_list)
            {
                bool prev_same = 0;
                KeyPose *prev_oppo_poss = NULL;
                KeyPose *prev_poss = NULL;
                for (int i = 0; i < road_segment->pos_sample_list.size(); ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    if (poss->invalid())
                    {
                        continue;
                    }
                    auto &boundary = poss->boundary;
                    auto &oppo_poss = boundary.left_oppo_pos.src;
                    bool oppo_same = oppo_poss != NULL && oppo_poss->filter_status == 6; // 看情况都是false
                    if (i == 0)
                    {
                        prev_same = oppo_same;
                        prev_oppo_poss = oppo_poss;
                        prev_poss = poss;
                        continue;
                    }
                    if (prev_same == oppo_same)
                    {
                        continue;
                    }
                    // if (prev_oppo_poss == NULL) {
                    //     continue;
                    // }
                    // 好像走不到这里
                    if (!prev_same)
                    {
                        if (prev_oppo_poss != NULL)
                        {
                            prev_oppo_poss->context.set_prev(poss);
                            poss->context.set_prev(prev_oppo_poss);
                        }
                    }
                    else if (!oppo_same)
                    {
                        if (oppo_poss != NULL)
                        {
                            oppo_poss->context.set_next(prev_poss);
                            prev_poss->context.set_next(oppo_poss);
                        }
                    }
                    prev_same = oppo_same;
                    prev_oppo_poss = oppo_poss;
                    prev_poss = poss;
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFillLane::reline_lane_sample(RoadModelSessionData *session)
        {
            // 前后重新串线
            for (auto &road_segment : session->road_segment_list)
            {
                int64_t end_index = road_segment->pos_sample_list.size() - 1;
                for (int64_t i = 1; i <= end_index; ++i)
                {
                    auto &poss_prev = road_segment->pos_sample_list[i - 1];
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    
                    for (auto &lc : poss->filled_lane_sample)
                    {
                        session->debug_pos(lc->pos);
                        if (lc->invalid())
                        {
                            continue;
                        }
                        // 修正方向
                        double theta = alg::calc_theta(lc->dir, poss->dir);
                        // if (theta > 90) {
                        //     FLOG_POINT(lc->pos, "error lc dir");
                        //     lc->dir = -lc->dir;
                        // }
                        for (auto &lc_prev : poss_prev->filled_lane_sample)
                        {
                            if (lc_prev->fill_next != NULL || lc_prev->invalid())
                            {
                                continue;
                            }
                            if (FLAGS_fill_lane_smooth_lane_enable && judge_same_lane_line(session, lc_prev, lc, false, 1))
                            {
                                lc_prev->all_fill_next.push_back(lc);
                                lc->all_fill_prev.push_back(lc_prev);
                            }

                            if (judge_same_lane_line(session, lc_prev, lc, false))
                            {
                                lc_prev->fill_next = lc;
                                lc->fill_prev = lc_prev;
                                // alg::calc_dir(lc->pos, lc_prev->pos, lc_prev->dir);
                                break;
                            }
                        }
                    }        
                }
            }
            for (auto &road_segment : session->road_segment_list)
            {
                int64_t end_index = road_segment->pos_sample_list.size() - 1;
                for (int64_t i = 0; i <= end_index; ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    for (auto &lc : poss->filled_lane_sample)
                    {
                        session->debug_pos(lc->pos);
                        if (lc->invalid())
                        {
                            continue;
                        }
                        if (lc->fill_next != NULL && lc->fill_prev != NULL)
                        {
                            continue;
                        }
                        std::vector<LaneCenterFeature *> context_list;
                        lc->get_context_list(30, context_list, [](LaneCenterFeature *curr) -> LaneCenterFeature *
                                             {
                        if (curr->fill_prev == NULL || curr->fill_prev->invalid()) {
                        return NULL;
                        }
                        return curr->fill_prev; }, [](LaneCenterFeature *curr) -> LaneCenterFeature *
                                             {
                        if (curr->fill_next == NULL || curr->fill_next->invalid()) {
                        return NULL;
                        }
                        return curr->fill_next; }, 0);
                        lc->filled_context_count = context_list.size();
                    }
                }
            }
            for (auto &road_segment : session->road_segment_list)
            {
                int64_t end_index = road_segment->pos_sample_list.size() - 1;
                for (int64_t i = 0; i <= end_index; ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    for (auto &lc : poss->filled_lane_sample)
                    {
                        session->debug_pos(lc->pos);
                        if (lc->invalid())
                        {
                            continue;
                        }
                        if (lc->fill_next != NULL)
                        {
                            continue;
                        }

                        for (int j = 1; j < 8; ++j)
                        {
                            int next_index = i + j;
                            if (next_index > end_index)
                            {
                                break;
                            }
                            bool has_next = false;
                            auto &next_pos = road_segment->pos_sample_list[next_index];
                            for (auto &lc_next : next_pos->filled_lane_sample)
                            {
                                if (lc_next->fill_prev != NULL || lc_next->invalid())
                                {
                                    continue;
                                }
                                if (lc->raw_from_lc->context.on_same_line(
                                        20, lc->raw_from_lc, lc_next->raw_from_lc, true,
                                        [](ParamPair<LaneCenterFeature> &c)
                                        {
                                            return true;
                                        },
                                        NULL))
                                {
                                    lc->fill_next = lc_next;
                                    lc_next->fill_prev = lc;
                                    has_next = true;
                                    break;
                                }
                                if (judge_same_lane_line(session, lc, lc_next, false, 1))
                                {
                                    lc->fill_next = lc_next;
                                    lc_next->fill_prev = lc;
                                    has_next = true;
                                    break;
                                }
                            }
                            if (has_next)
                            {
                                break;
                            }
                        }         
                    }
                }
            }
            // // 跨road_segment
            // for (auto &road_segment : session->road_segment_list) {
            //     int64_t end_index = road_segment->pos_sample_list.size() - 1;
            //     for (int64_t i = 0; i <= end_index; ++i) {
            //         auto &poss = road_segment->pos_sample_list[i];
            //         session->debug_pos(poss->pos);
            //         if (poss->context.valid_next_size() == 0) {
            //             continue;
            //         }
            //         for (auto& lc : poss->filled_lane_sample) {
            //             session->debug_pos(lc->pos);
            //             if (lc->invalid()) {
            //                 continue;
            //             }
            //             if (lc->fill_next != NULL) {
            //                 continue;
            //             }
            //             auto next_poss = poss->context.next();
            //             auto tar_road_segment = next_poss->road_segment;
            //             double total_length = 0;
            //             auto prev_poss = next_poss;
            //             while (next_poss != NULL) {
            //                 for (auto &lc_next : next_poss->filled_lane_sample) {
            //                     if (lc_next->fill_next != NULL || lc_next->invalid()) {
            //                         continue;
            //                     }
            //                     if (judge_same_lane_line(session, lc, lc_next, false)) {
            //                         lc->context.set_next(lc_next);
            //                         lc_next->context.set_next(lc);
            //                         has_next = true;
            //                         break;
            //                     }
            //                 }
            //                 if (has_next) {
            //                     break;
            //                 }
            //             }
            //         }
            //     }
            // }
            return fsdmap::SUCC;
        }

        int RoadModelProcFillLane::process_fork_merge(RoadModelSessionData *session, bool next)
        {
            for (auto &road_segment : session->road_segment_list)
            {
                int64_t end_index = road_segment->pos_sample_list.size() - 1;
                for (int64_t i = 0; i <= end_index; ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    if (poss->invalid())
                    {
                        continue;
                    }
                    for (auto &lc : poss->filled_lane_sample)
                    {

                        session->debug_pos(lc->pos);
                        if (lc->invalid())
                        {
                            continue;
                        }
                        auto fill_next = next ? lc->fill_next : lc->fill_prev;
                        if (fill_next != NULL)
                        {
                            continue;
                        }
                        auto next_lc = get_next_valid_lc(session, road_segment, i, lc, next);
                        if (next_lc != NULL)
                        {
                            continue;
                        }
                        next_lc = get_next_reline_lc(session, road_segment, i, lc, next);
                        if (next_lc != NULL)
                        {
                            continue;
                        }
                        lc->fill_status = 3;
                    }
                }
            }

            return fsdmap::SUCC;
        }

        LaneCenterFeature *RoadModelProcFillLane::get_next_reline_lc(RoadModelSessionData *session,
                                                                     RoadSegment *road_segment, int64_t poss_index, LaneCenterFeature *lc, bool next)
        {
            double radius = FLAGS_fill_lane_get_next_lc_radius;
            double theta_thres = FLAGS_fill_lane_get_next_lc_theta_thres;
            int start_index = next ? poss_index + 1 : poss_index - 1;
            int end_index = next ? road_segment->pos_sample_list.size() - 1 : 0;
            int poss_num = fabs(end_index - poss_index);
            if (poss_num <= 1)
            {
                return NULL;
            }
            auto prev_poss = road_segment->pos_sample_list[poss_index];
            int valid_next_index = -1;
            double total_length = 0;
            LaneCenterFeature *valid_next_lc = NULL;
            for (int i = 0; i < poss_num; ++i)
            {
                int real_index = next ? start_index + i : start_index - i;
                auto &next_poss = road_segment->pos_sample_list[real_index];
                if (next_poss->invalid())
                {
                    prev_poss = next_poss;
                    continue;
                }
                total_length += alg::calc_dis(prev_poss->pos, next_poss->pos);
                if (total_length > radius)
                {
                    break;
                }
                for (auto &next_lc : next_poss->filled_lane_sample)
                {
                    if (next_lc->invalid())
                    {
                        continue;
                    }
                    auto tmp_dir = alg::get_dir(next_lc->pos, lc->pos);
                    double theta = alg::calc_theta(tmp_dir, lc->dir);
                    if (theta > theta_thres)
                    {
                        continue;
                    }
                    if (judge_same_lane_line(session, lc, next_lc, false))
                    {
                        valid_next_lc = next_lc;
                        break;
                    }
                    if (valid_next_lc != NULL)
                    {
                        lc->fill_status = 4;
                        if (next)
                        {
                            if (valid_next_lc->fill_prev == NULL)
                            {
                                valid_next_lc->fill_prev = lc;
                                lc->fill_next = valid_next_lc;
                            }
                            else
                            {
                                valid_next_lc->context.set_prev(lc);
                                lc->context.set_next(valid_next_lc);
                            }
                        }
                        else
                        {
                            if (valid_next_lc->fill_next == NULL)
                            {
                                valid_next_lc->fill_next = lc;
                                lc->fill_prev = valid_next_lc;
                            }
                            else
                            {
                                valid_next_lc->context.set_next(lc);
                                lc->context.set_prev(valid_next_lc);
                            }
                        }
                        break;
                    }
                }
            }
            return valid_next_lc;
        }
        LaneCenterFeature *RoadModelProcFillLane::get_next_valid_lc(RoadModelSessionData *session,
                                                                    RoadSegment *road_segment, int64_t poss_index, LaneCenterFeature *lc, bool next)
        {
            double radius = FLAGS_fill_lane_get_next_lc_radius;
            double theta_thres = FLAGS_fill_lane_get_next_lc_theta_thres;
            int start_index = next ? poss_index + 1 : poss_index - 1;
            int end_index = next ? road_segment->pos_sample_list.size() - 1 : 0;
            int poss_num = fabs(end_index - poss_index);
            if (poss_num <= 1)
            {
                return NULL;
            }
            auto prev_poss = road_segment->pos_sample_list[poss_index];
            UMAP<LaneCenterFeature *, std::vector<LaneCenterFeature *>> line_map;
            int valid_next_index = -1;
            double total_length = 0;
            std::vector<LaneCenterFeature *> valid_sample_list;
            LaneCenterFeature *valid_next_lc = NULL;
            for (int i = 0; i < poss_num; ++i)
            {
                int real_index = next ? start_index + i : start_index - i;
                auto &next_poss = road_segment->pos_sample_list[real_index];
                if (next_poss->invalid())
                {
                    prev_poss = next_poss;
                    continue;
                }
                total_length += alg::calc_dis(prev_poss->pos, next_poss->pos);
                if (total_length > radius)
                {
                    break;
                }
                for (auto &next_lc : next_poss->filled_lane_sample)
                {
                    if (next_lc->invalid())
                    {
                        continue;
                    }
                    line_map.clear();
                    auto tmp_dir = alg::get_dir(next_lc->pos, lc->pos);
                    double theta = alg::calc_theta(tmp_dir, lc->dir, true);
                    if (theta > theta_thres)
                    {
                        continue;
                    }
                    if (!lc->raw_from_lc->context.on_same_line(
                            2 * radius, lc->raw_from_lc, next_lc->raw_from_lc, next, &line_map))
                    {
                        continue;
                    }
                    valid_next_lc = next_lc;
                    valid_next_index = real_index;
                    break;
                }
                if (valid_next_lc != NULL)
                {
                    break;
                }
                prev_poss = next_poss;
            }
            if (valid_next_lc == NULL)
            {
                for (int i = 0; i < poss_num; ++i)
                {
                    int real_index = next ? start_index + i : start_index - i;
                    auto &next_poss = road_segment->pos_sample_list[real_index];
                    if (next_poss->invalid())
                    {
                        prev_poss = next_poss;
                        continue;
                    }
                    total_length += alg::calc_dis(prev_poss->pos, next_poss->pos);
                    if (total_length > radius)
                    {
                        break;
                    }
                    for (auto &next_lc : next_poss->filled_lane_sample)
                    {
                        if (next_lc->invalid())
                        {
                            continue;
                        }
                        line_map.clear();
                        auto tmp_dir = alg::get_dir(next_lc->pos, lc->pos);
                        double theta = alg::calc_theta(tmp_dir, lc->dir);
                        if (theta > theta_thres)
                        {
                            continue;
                        }
                        if (!lc->raw_from_lc->context.on_same_line(
                                2 * radius, lc->raw_from_lc, next_lc->raw_from_lc, next,
                                [](ParamPair<LaneCenterFeature> &c)
                                {
                                    return true;
                                },
                                &line_map))
                        {
                            continue;
                        }
                        valid_next_lc = next_lc;
                        valid_next_index = real_index;
                        break;
                    }
                    if (valid_next_lc != NULL)
                    {
                        break;
                    }
                    prev_poss = next_poss;
                }
            }
            if (valid_next_lc != NULL)
            {
                UMAP<LaneCenterFeature *, int> used_map;
                for (auto &it : line_map)
                {
                    auto &from_lc = it.first;
                    for (auto &to_lc : it.second)
                    {
                        auto &tmp_vec = next ? from_lc->cross_point[to_lc] : to_lc->cross_point[from_lc];
                        for (auto &s_lc : tmp_vec)
                        {
                            if (MAP_FIND(used_map, s_lc))
                            {
                                continue;
                            }
                            used_map[s_lc] = 1;
                            if (s_lc->road_segment != road_segment)
                            {
                                continue;
                            }
                            if (s_lc->invalid())
                            {
                                continue;
                            }
                            if (s_lc->fill_status > 0)
                            {
                                continue;
                            }
                            valid_sample_list.push_back(s_lc);
                        }
                    }
                }
                fill_with_lc_list(session, road_segment, poss_index, valid_next_index,
                                  valid_sample_list, next, lc, valid_next_lc);
            }
            return valid_next_lc;
        }

        int RoadModelProcFillLane::fill_with_lc_list(RoadModelSessionData *session,
                                                     RoadSegment *road_segment, int start_index, int end_index,
                                                     std::vector<LaneCenterFeature *> &line_vec, bool next,
                                                     LaneCenterFeature *from_lc, LaneCenterFeature *to_lc)
        {
            auto to_dir = alg::get_dir(to_lc->pos, from_lc->pos);
            int poss_num = fabs(end_index - start_index) - 1;
            LaneCenterFeature *prev_lc = from_lc;
            for (int i = 0; i < poss_num; ++i)
            {
                int real_index = next ? start_index + i + 1 : start_index - i - 1;
                auto &poss = road_segment->pos_sample_list[real_index];
                if (poss->invalid())
                {
                    continue;
                }
                LaneCenterFeature *min_lc = NULL;
                double min_dis = DBL_MAX;
                for (auto &lc : line_vec)
                {
                    if (lc->key_pose != poss)
                    {
                        continue;
                    }
                    double v_dis = alg::calc_vertical_dis(lc->pos, from_lc->pos, to_dir);
                    if (v_dis < min_dis)
                    {
                        min_dis = v_dis;
                        min_lc = lc;
                    }
                }
                if (min_lc == NULL)
                {
                    continue;
                }
                bool has_filled = false;
                for (int j = 0; j < poss->filled_lane_sample.size(); ++j)
                {
                    auto &tmp_lc = poss->filled_lane_sample[j];
                    if (tmp_lc->invalid())
                    {
                        continue;
                    }
                    int is_left = alg::judge_left(min_lc->pos, tmp_lc->pos, poss->dir);
                    if (is_left >= 0)
                    {
                        continue;
                    }
                    VEC_INSERT_INDEX(poss->filled_lane_sample, j, min_lc);
                    min_lc->fill_status = 2;
                    has_filled = true;
                    break;
                }
                if (!has_filled)
                {
                    poss->filled_lane_sample.push_back(min_lc);
                    min_lc->fill_status = 2;
                }
                if (prev_lc != NULL)
                {
                    if (next)
                    {
                        prev_lc->fill_next = min_lc;
                        min_lc->fill_prev = prev_lc;
                    }
                    else
                    {
                        prev_lc->fill_prev = min_lc;
                        min_lc->fill_next = prev_lc;
                    }
                }
                prev_lc = min_lc;
            }
            if (prev_lc != NULL)
            {
                if (next)
                {
                    if (to_lc->fill_prev == NULL)
                    {
                        to_lc->fill_prev = prev_lc;
                        prev_lc->fill_next = to_lc;
                    }
                    else
                    {
                        to_lc->context.set_prev(prev_lc);
                        prev_lc->context.set_next(to_lc);
                    }
                }
                else
                {
                    if (to_lc->fill_next == NULL)
                    {
                        to_lc->fill_next = prev_lc;
                        prev_lc->fill_prev = to_lc;
                    }
                    else
                    {
                        to_lc->context.set_next(prev_lc);
                        prev_lc->context.set_prev(to_lc);
                    }
                }
            }
            return fsdmap::SUCC;
        }

        bool RoadModelProcFillLane::judge_same_lane_line(RoadModelSessionData *session,
                                                         LaneCenterFeature *lc_prev, LaneCenterFeature *lc, bool use_org, int mode)
        {
            // 中心点方向夹角，距离，宽度
            if (lc_prev->next == lc)
            {
                return true;
            }
            // if (lc_prev->road_center != NULL && lc_prev->road_center->line_id != lc->road_center->line_id) {
            //     return false;
            // }
            int is_match = 0;
            double dis_threshold = FLAGS_fill_lane_same_lane_dis_threshold;             // 10
            double theta_threshold = FLAGS_fill_lane_same_lane_theta_threshold;         // 30
            double max_width_threshold = FLAGS_fill_lane_same_lane_max_width_threshold; // 1
            double min_width_threshold = FLAGS_fill_lane_same_lane_min_width_threshold; // 0.3
            auto &prev_center = *lc_prev;
            auto &curr_center = *lc;
            double dis = alg::calc_dis(prev_center.pos, curr_center.pos);
            Eigen::Vector3d dir;
            alg::calc_dir(curr_center.pos, prev_center.pos, dir);
            double theta_1 = alg::calc_theta(curr_center.dir, dir, true);
            double theta_2 = alg::calc_theta(prev_center.dir, dir, true);
            double theta_avg = (theta_1 + theta_2) / 2;
            double width_gap = curr_center.width - prev_center.width;
            bool has_trangle = lc->match_level.lane_type != 1 ? true : false;
            has_trangle = has_trangle || lc_prev->match_level.lane_type != 1 ? true : false;
            do
            {
                if (dis > dis_threshold)
                {
                    break;
                }
                if (theta_avg > theta_threshold)
                {
                    break;
                }
                if (has_trangle)
                {
                    // 三角尖角对应的只能更窄
                    if ((lc_prev->match_level.join_out == 2 || lc->match_level.join_out == 2) && width_gap > min_width_threshold)
                    {
                        break;
                    }
                    if ((lc_prev->match_level.join_out == 1 || lc->match_level.join_out == 1) && width_gap < -min_width_threshold)
                    {
                        break;
                    }
                }
                else if (fabs(width_gap) > max_width_threshold)
                {
                    if (mode == 0)
                    {
                        break;
                    }
                }
                is_match = 1;
            } while (0);

            DLOG_POINT2(curr_center.pos, prev_center.pos,
                        "judge same line[match={}, dis={}, theta_avg={}, width_gap={}, theta_1={}, theta_2={}]",
                        is_match, dis, theta_avg, width_gap, theta_1, theta_2);
            return is_match > 0;
        }

        void RoadModelProcFillLane::spread_valid_lc(RoadModelSessionData *session, LaneCenterFeature *lc, bool front, double total_dis)
        {
            double scope = FLAGS_fill_lane_spread_valid_lc_scope;
            auto &all_next = front ? lc->all_fill_next : lc->all_fill_prev;
            for (auto &next : all_next)
            {
                total_dis += alg::calc_dis(lc->pos, next->pos, true);
                if (total_dis > scope)
                {
                    continue;
                }
                if (next->boundary_status > 0)
                {
                    continue;
                }
                next->boundary_status = 2;
                spread_valid_lc(session, next, front, total_dis);
            }
        }

        int RoadModelProcFillLane::spread_valid_lc(RoadModelSessionData *session, KeyPose *poss, LaneCenterFeature *lc)
        {
            double scope = FLAGS_fill_lane_spread_valid_lc_scope;
            double total_dis = 0;
            auto prev = lc;
            while (prev->fill_prev != NULL)
            {
                total_dis += alg::calc_dis(prev->pos, prev->fill_prev->pos, true);
                if (total_dis > scope)
                {
                    break;
                }
                if (prev->fill_prev->boundary_status == 1)
                {
                    break;
                }
                prev->fill_prev->boundary_status = 2;
                prev = prev->fill_prev;
            }
            total_dis = 0;
            auto next = lc;
            while (next->fill_next != NULL)
            {
                total_dis += alg::calc_dis(next->pos, next->fill_next->pos, true);
                if (total_dis > scope)
                {
                    break;
                }
                if (next->fill_next->boundary_status == 1)
                {
                    break;
                }
                next->fill_next->boundary_status = 2;
                next = next->fill_next;
            }
        }

        // void RoadModelProcFillLane::mark_valid_lc(RoadModelSessionData* session, KeyPose* poss, LaneCenterFeature* lc) {
        //     auto &boundary = poss->boundary;
        //     bool is_left = alg::judge_left(lc->pos, poss->pos, poss->dir) < 0;
        //     auto &b_fls = is_left ? boundary.left.src : boundary.right.src;
        //     auto &y_fls = is_left ? boundary.left_yellow.src : boundary.right_yellow.src;
        //     if (b_fls == NULL && y_fls == NULL) {
        //         return;
        //     }
        //     double theta = alg::calc_theta(b_fls->dir, poss->dir);
        //     if (theta > 90) {
        //         return;
        //     }
        //     lc->boundary_status = 1;
        // }

        int RoadModelProcFillLane::save_debug_info(RoadModelSessionData *session)
        {
            if (!FLAGS_fill_lane_save_data_enable)
            {
                return fsdmap::SUCC;
            }
            session->set_display_name("fill_lane_prev");
            int64_t lc_id = 0;
            for (auto &road_segment : session->road_segment_list)
            {
                for (int i = 0; i < road_segment->pos_sample_list.size(); ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    for (int j = 0; j < poss->pre_filled_lane_sample.size(); ++j)
                    {
                        auto &lc = poss->pre_filled_lane_sample[j];
                        double score = lc->line_param.score;
                        Eigen::Vector3d tar_pos = lc->pos + lc->dir * 0.5;
                        auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                                                          "fill_lane_line");
                        log->color = {255, 255, 0};
                        log->add(poss->pos, 2);
                        auto &ele = log->add(tar_pos);
                        ele.label.label = lc->src_status;
                        lc_id++;
                        auto log_ptr = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                                                              "fill_lane_line");
                        auto log_ptr1 = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                                                               "fill_lane_line");
                        log_ptr->color = {255, 255, 255};
                        log_ptr1->color = {0, 255, 0};
                        if (lc->src_status == 2)
                        {
                            log_ptr->color = {255, 100, 100};
                            log_ptr1->color = {0, 100, 200};
                        }

                        log_ptr->add(lc->pos, 1).label.intensity = score;
                        log_ptr->add(lc->get_left(), 1).label.intensity = score;
                        log_ptr1->add(lc->pos, 1).label.intensity = score;
                        log_ptr1->add(lc->get_right(), 1).label.intensity = score;
                        if (lc->fill_next != NULL)
                        {
                            log_ptr->add(lc->fill_next->pos, 1).label.intensity = score;
                            log_ptr1->add(lc->fill_next->pos, 1).label.intensity = score;
                            // } else if (lc->fill_prev != NULL) {
                            //     log_ptr->add(lc->fill_prev->pos, 1);
                            //     log_ptr1->add(lc->fill_prev->pos, 1);
                        }
                    }
                }
            }
            session->save_debug_info("fill_lane_prev");
            session->set_display_name("fill_lane");
            lc_id = 0;
            for (auto &road_segment : session->road_segment_list)
            {
                for (int i = 0; i < road_segment->pos_sample_list.size(); ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    for (int j = 0; j < poss->filled_lane_sample.size(); ++j)
                    {
                        auto &lc = poss->filled_lane_sample[j];
                        double score = lc->line_param.score;
                        Eigen::Vector3d tar_pos = lc->pos + lc->dir * 0.5;
                        auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                                                          "fill_lane_line");
                        log->color = {100, 100, 0};
                        log->add(poss->pos, 2);
                        auto &ele = log->add(tar_pos);
                        ele.label.label = lc->src_status;
                        lc_id++;
                        auto log_ptr = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                                                              "fill_lane_line");
                        // auto log_ptr1 = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                        //         "fill_lane_line");
                        log_ptr->color = {255, 255, 255};
                        // log_ptr1->color = {0, 255, 0};
                        // if (lc->src_status == 2) {
                        if (lc->fill_status == 2)
                        {
                            log_ptr->color = {0, 255, 0};
                            // log_ptr1->color = {0, 100, 200};
                        }
                        if (lc->fill_status == 4)
                        {
                            log_ptr->color = {0, 255, 255};
                            // log_ptr1->color = {0, 100, 200};
                        }
                        if (lc->road_index != 0)
                        {
                            log_ptr->color[0] = 100;
                            // log_ptr1->color = {0, 100, 100};
                        }

                        // log_ptr->add(lc->pos, 1, lc->match_level.lane_type).label.intensity = score;
                        log_ptr->add(lc->get_left(), 1, lc->match_level.lane_type).label.intensity = score;
                        log_ptr->add(lc->get_right(), 1, lc->match_level.lane_type).label.intensity = score;
                        // log_ptr1->add(lc->pos, 1, lc->match_level.lane_type).label.intensity = score;
                        // log_ptr1->add(lc->get_right(), 1, lc->match_level.lane_type).label.intensity = score;
                        bool has_next = false;
                        if (lc->fill_next != NULL)
                        {
                            log_ptr->add(lc->fill_next->pos, 1).label.intensity = score;
                            has_next = true;
                        }
                        if (lc->context.all_next.size() > 0)
                        {
                            for (auto &next_ptr : lc->context.all_next)
                            {
                                auto &next_lc = next_ptr.src;
                                if (next_lc == lc->fill_next)
                                {
                                    continue;
                                }
                                auto log1 = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "");
                                log1->color = {255, 255, 0};
                                log1->add(lc->pos, 1).label.intensity = score;
                                log1->add(next_lc->pos, 1).label.intensity = score;
                            }
                            has_next = true;
                        }
                        if (!has_next)
                        {
                            auto next_pos = alg::get_hori_pos(lc->pos, lc->dir, 0.5);
                            log_ptr->add(next_pos, 1).label.intensity = score;
                            //     log_ptr1->add(lc->fill_prev->pos, 1);
                        }
                        if (lc->fill_status == 3)
                        {
                            auto t_log = session->add_debug_log(utils::DisplayInfo::POINT,
                                                                "fill_lane_line");
                            t_log->color = {255, 0, 0};
                            t_log->add(lc->pos);
                        }
                    }
                }
            }
            session->save_debug_info("fill_lane");

            session->set_display_name("lane_center_list");
            int64_t lc_idd = 0;
            for (auto &road_segment : session->road_segment_list)
            {
                for (int i = 0; i < road_segment->pos_sample_list.size(); ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    for (int j = 0; j < poss->filled_lane_sample.size(); ++j)
                    {
                        auto &lc = poss->filled_lane_sample[j];

                        double score = lc->line_param.score;
                        Eigen::Vector3d tar_pos = lc->pos + lc->dir * 0.5;
                        auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                                                          "lane_center_list");
                        log->color = {255, 0, 0};
                        log->add(poss->pos, 2);
                        auto &ele = log->add(tar_pos);
                        ele.label.label = lc->src_status;
                        lc_idd++;
                        auto log_ptr = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                                                              "lane_center_list");
                        auto log_ptr1 = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                                                               "lane_center_list");
                        log_ptr->color = {123, 0, 255};
                        log_ptr1->color = {122, 59, 67};
                        if (lc->src_status == 2)
                        {
                            log_ptr->color = {25, 150, 100};
                            log_ptr1->color = {0, 100, 0};
                        }

                        log_ptr->add(lc->pos, 1).label.intensity = score;
                        log_ptr->add(lc->get_left(), 1).label.intensity = score;
                        log_ptr1->add(lc->pos, 1).label.intensity = score;
                        log_ptr1->add(lc->get_right(), 1).label.intensity = score;
                        if (lc->fill_next != NULL)
                        {
                            log_ptr->add(lc->fill_next->pos, 1).label.intensity = score;
                            log_ptr1->add(lc->fill_next->pos, 1).label.intensity = score;
                        }
                    }
                }
            }
            session->save_debug_info("lane_center_list");

            return fsdmap::SUCC;
        }

    }
}
