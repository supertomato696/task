

#include "road_model_proc_format_lane.h"

DEFINE_bool(format_lane_enable, true, "format_lane_enable");
DEFINE_bool(format_lane_debug_pos_enable, true, "format_lane_debug_pos_enable");
DEFINE_bool(format_lane_save_data_enable, true, "format_lane_save_data_enable");
DEFINE_bool(format_lane_align_end_point_enable, true, "format_lane_align_end_point_enable");
DEFINE_bool(format_lane_align_lane_local_enable, false, "format_lane_align_lane_local_enable");
DEFINE_bool(format_lane_move_base_line_enable, true, "format_lane_move_base_line_enable");
DEFINE_bool(format_lane_double_line_use_raw_gap_enable, false, "format_lane_double_line_use_raw_gap_enable");
DEFINE_double(format_lane_fill_road_dis_lambda, 0.2, "format_lane_fill_road_dis_lambda");
DEFINE_double(format_lane_max_radius_threshold, 650, "format_lane_max_radius_threshold");
DEFINE_double(format_lane_lane_line_min_lane_length, 7.9, "format_lane_lane_line_min_lane_length");
DEFINE_double(format_lane_lane_line_max_length_threshold, 50, "fmrmat_lane_lane_line_max_length_threshold");
DEFINE_double(format_lane_lane_line_max_width_threshold, 1.5, "format_lane_lane_line_max_width_threshold");
DEFINE_double(format_lane_lane_line_min_gap_threshold, 0.5, "format_lane_lane_line_min_gap_threshold");
DEFINE_double(format_lane_lane_line_min_length_gap_threshold, 25, "format_lane_lane_line_min_length_gap_threshold");
DEFINE_double(format_lane_align_point_min_dis_threshold, 0.5, "format_lane_align_point_min_dis_threshold");
DEFINE_double(format_lane_align_point_max_dis_threshold, 20, "format_lane_align_point_max_dis_threshold");
DEFINE_double(format_lane_ll_conflict_rate_threshold, 0.2, "format_lane_ll_conflict_rate_threshold");
DEFINE_double(format_lane_get_segment_max_distance, 10, "format_lane_get_segment_max_distance");

DEFINE_bool(format_lane_gen_feature_dash_enable, false, "format_lane_gen_feature_dash_enable");
DEFINE_bool(format_lane_gen_feature_left_enable, false, "format_lane_gen_feature_left_enable");
DEFINE_bool(format_lane_gen_feature_break_low_prob_enable, false, "format_lane_gen_feature_break_low_prob_enable");
DEFINE_double(format_lane_gen_feature_length, 2, "format_lane_gen_feature_length");
DEFINE_double(format_lane_gen_feature_width, 0.1, "format_lane_gen_feature_width");
DEFINE_double(format_lane_gen_feature_height, 0.01, "format_lane_gen_feature_height");
DEFINE_double(format_lane_gen_feature_z_delta, 0, "format_lane_gen_feature_z_delta");
DEFINE_double(format_lane_prob_score_threshold, 0.7, "format_lane_prob_score_threshold");
DEFINE_double(format_lane_fork_merge_dis_threshold, 1, "format_lane_fork_merge_dis_threshold");
DEFINE_double(format_lane_left_move_dis, 0.00, "format_lane_left_move_dis");
DEFINE_double(format_lane_left_double_move_dis, 0.1, "format_lane_left_double_move_dis");
DEFINE_double(format_lane_left_move_scope, 1, "format_lane_left_move_scope");
DEFINE_int32(format_lane_break_near_count, 10, "format_lane_break_near_count");
DEFINE_double(format_lane_oppo_rate_threshold, 0.6, "format_lane_oppo_rate_threshold");
DEFINE_double(format_lane_complete_rate_threshold, 0.7, "format_lane_complete_rate_threshold");
DEFINE_double(format_lane_valid_side_gap_threshold, 2, "format_lane_valid_side_gap_threshold");
DEFINE_double(format_lane_calc_curvature_radius, 25, "format_lane_calc_curvature_radius");
DECLARE_double(init_trail_prob_min_threshold);
DECLARE_bool(download_data_update_enable);

namespace fsdmap
{
    namespace road_model
    {

        fsdmap::process_frame::PROC_STATUS RoadModelProcFormatLane::proc(
            RoadModelSessionData *session)
        {
            if (!FLAGS_format_lane_enable)
            {
                return fsdmap::process_frame::PROC_STATUS_DISABLE;
            }
            session->enable_debug_pos = FLAGS_format_lane_debug_pos_enable;
            // 初始化
            CHECK_FATAL_PROC(init_param(session), "init_param");
            // 生成整条lane
            CHECK_FATAL_PROC(fill_lane_line(session), "fill_lane_line");

            CHECK_FATAL_PROC(filter_lane_line(session), "filter_lane_line"); 

            CHECK_FATAL_PROC(merge_sub_road_segment(session), "merge_sub_road_segment");

            // 生成valid_lane_sample_group_tree
            CHECK_FATAL_PROC(vote_valid_side(session), "vote_valid_side");

            CHECK_FATAL_PROC(split_oppo_lane(session), "split_oppo_lane");

            // CHECK_FATAL_PROC(clear_lane_attr(session), "clear_lane_attr");

            // CHECK_FATAL_PROC(update_lc_opt_dir(session), "update_lc_opt_dir");

            CHECK_FATAL_PROC(align_end_point(session), "align_end_point");

            // 生成valid_lane_sample_group_tree
            CHECK_FATAL_PROC(make_valid_lc_tree(session), "make_valid_lc_tree");

            // 转换数据到路网格式
            CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
            return fsdmap::process_frame::PROC_STATUS_SUCC;
        }

        int RoadModelProcFormatLane::split_oppo_lane(RoadModelSessionData *session)
        {
            double rate_threshold = FLAGS_format_lane_oppo_rate_threshold;
            for (auto &srs : session->sub_road_segment_list)
            {
                auto &road_segment = srs->src_road_segment;
                int64_t valid_num = 0;
                int64_t oppo_num = 0;
                for (int i = srs->start_index; i <= srs->end_index; ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    if (poss->invalid())
                    {
                        continue;
                    }
                    ++valid_num;
                    session->debug_pos(poss->pos);
                    auto &boundary = poss->boundary;
                    RoadCenterFeature *curr = boundary.get_road_center(srs->road_index);
                    if (curr != NULL && curr->yellow_boundary.src != NULL)
                    {
                        ++oppo_num;
                    }
                }
                srs->road_type = 1;
                if (valid_num > 0)
                {
                    float rate = (float)oppo_num / valid_num;
                    if (rate > rate_threshold)
                    {
                        srs->road_type = 2;
                    }
                }
                for (auto &ll : srs->valid_lane_list)
                {
                    if (ll->invalid())
                    {
                        continue;
                    }
                    vote_direction(session, ll, srs);
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::vote_direction(RoadModelSessionData *session,
                                                    LaneCenterLine *ll, SubRoadSegment *srs)
        {
            double rate_threshold = FLAGS_format_lane_oppo_rate_threshold;
            int oppo_num = 0;
            for (auto &lc : ll->list)
            {
                if (lc->dir_status > 0)
                {
                    if ((lc->dir_status == 2) ^ (srs->road_index < 0))
                    {
                        ++oppo_num;
                    }
                }
                else
                {
                    auto &poss = lc->key_pose;
                    auto &rc = poss->boundary.curr;
                    if (rc->yellow_boundary.src == NULL)
                    {
                        continue;
                    }
                    if (rc->in_scope(lc->pos, 0, 2))
                    {
                        ++oppo_num;
                    }
                }
            }
            float rate = (float)oppo_num / ll->list.size();
            // if ((rate > rate_threshold) ^ (ll->road_index == -1)) {
            if ((rate > rate_threshold))
            {
                ll->side_status = 2;
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::align_end_point(RoadModelSessionData *session)
        {
            if (!FLAGS_format_lane_align_end_point_enable)
            {
                return fsdmap::SUCC;
            }
            // 需要根据之前预测的打断点进行平齐
            std::vector<int> valid_rc_index = {0, -1, 1};
            for (auto &rc_index : valid_rc_index)
            {
                for (int side = 1; side < 3; ++side)
                {

                    for (auto &srs : session->sub_road_segment_list)
                    {
                        if (srs->road_index != rc_index)
                        {
                            continue;
                        }

                        if (srs->link_direction != side)
                        {
                            continue;
                        }
                        auto &road_segment = srs->src_road_segment;
                        auto &start_poss = road_segment->pos_sample_list[srs->start_index];
                        auto &end_poss = road_segment->pos_sample_list[srs->end_index];
                        session->debug_pos(start_poss->pos);
                        session->debug_pos(end_poss->pos);

                        clear_invalid_lc(srs);

                        for (auto &lane_line : srs->valid_lane_list)
                        {
                            for (int i = 0; i < lane_line->list.size(); ++i)
                            {
                                auto &lc = lane_line->list[i];
                                // session->debug_pos(lc->pos);
                            }
                            double next_com_dis = session->align_point(lane_line, srs->start_break_pos, true,
                                                                       FLAGS_format_lane_align_point_max_dis_threshold);
                            double prev_com_dis = session->align_point(lane_line, srs->end_break_pos, false,
                                                                       FLAGS_format_lane_align_point_max_dis_threshold);
                            lane_line->line_param.complete_length = next_com_dis + prev_com_dis;
                            for (int i = 0; i < lane_line->list.size(); ++i)
                            {
                                auto &lc = lane_line->list[i];
                                // session->debug_pos(lc->pos);
                            }
                        }

                        clear_invalid_lc(srs);
                        // 去重
                        // for (int i = 0; i < srs->valid_lane_list.size(); ++i) {
                        //     auto &ll1 = srs->valid_lane_list[i];
                        //     for (int j = i + 1; j < srs->valid_lane_list.size(); ++j) {
                        //         auto &ll2 = srs->valid_lane_list[j];
                        //         auto lc11 = ll1->list.front();
                        //         auto lc21 = ll2->list.front();
                        //         auto lc12 = ll1->list.back();
                        //         auto lc22 = ll2->list.back();

                        //         session->debug_pos(lc11->pos);
                        //         session->debug_pos(lc21->pos);
                        //         double dis1 = alg::calc_dis(lc11->get_right(), lc21->get_right());
                        //         double dis2 = alg::calc_dis(lc12->get_right(), lc22->get_right());
                        //         if (dis1 < 0.5 || dis2 < 0.5) {
                        //             int a = 1;
                        //         }
                        //         if (is_conflict(ll1, ll2)) {
                        //             double reline_length1 = ll1->get_reline_length();
                        //             double reline_length2 = ll2->get_reline_length();
                        //             if (reline_length1 > reline_length2) {
                        //                 ll1->filter_status = 5;
                        //             } else if (reline_length1 < reline_length2) {
                        //                 ll2->filter_status = 5;
                        //             } else {
                        //                 if (ll1->line_param.complete_length <= ll2->line_param.complete_length) {
                        //                     ll2->filter_status = 5;
                        //                 } else {
                        //                     ll1->filter_status = 5;
                        //                 }
                        //             }
                        //         }
                        //     }
                        // }
                        for (auto &ll : srs->valid_lane_list)
                        {
                            filter_lane_line_by_length(session, ll, 6);
                        }

                        clear_invalid_lc(srs);
                        SORT(srs->valid_lane_list,
                             [&start_poss](const LaneCenterLine *l, const LaneCenterLine *r) -> bool
                             {
                                 auto &l_lc = l->list.front();
                                 auto &r_lc = r->list.front();
                                 return alg::judge_left(l_lc->get_right(),
                                                        r_lc->get_right(), start_poss->dir) < 0;
                             });
                        // 对齐fork点
                        // align_fork_point(srs, true);
                        // align_fork_point(srs, false);
                    }
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::filter_lane_line_by_length(
            RoadModelSessionData *session, LaneCenterLine *ll, double min_lane_length)
        {
            auto prev_fn = [](LaneCenterFeature *lc) -> LaneCenterFeature *
            {
                auto prev = lc->fill_prev;
                if (prev == NULL || prev->invalid() || prev->road_index != lc->road_index || prev->side_status != lc->side_status)
                {
                    return NULL;
                }
                return prev;
            };
            auto next_fn = [](LaneCenterFeature *lc) -> LaneCenterFeature *
            {
                auto next = lc->fill_next;
                if (next == NULL || next->invalid() || next->road_index != lc->road_index || next->side_status != lc->side_status)
                {
                    return NULL;
                }
                return next;
            };

            std::vector<LaneCenterFeature *> context_list;
            ll->line_param.length = 0;
            LaneCenterFeature *prev_lc = NULL;
            for (auto &lc : ll->list)
            {
                session->debug_pos(lc->pos);
                if (prev_lc == NULL)
                {
                    prev_lc = lc;
                    continue;
                }
                ll->line_param.length += alg::calc_dis(lc->pos, prev_lc->pos);
                prev_lc = lc;
            }
            double prev_length = 0;
            double next_length = 0;
            auto &front_lc = ll->list.front();
            if (front_lc->break_status != 3)
            {
                prev_length = front_lc->get_context_list(min_lane_length * 2,
                                                         context_list,
                                                         prev_fn, next_fn,
                                                         2);
            }
            auto &back_lc = ll->list.back();
            // if (back_lc->key_pose->inter_status < 2 && back_lc->break_status != 3) {
            if (back_lc->break_status != 3)
            {
                double next_length = back_lc->get_context_list(min_lane_length * 2,
                                                               context_list,
                                                               prev_fn, next_fn,
                                                               1);
            }

            double final_length = prev_length + next_length + ll->line_param.length;
            if (final_length < min_lane_length)
            {
                ll->filter_status = 2;
                return false;
            }
            return true;
        }

        int RoadModelProcFormatLane::align_fork_point(SubRoadSegment *srs, bool front)
        {
            double fork_merge_dis_threshold = FLAGS_format_lane_fork_merge_dis_threshold;
            Eigen::Vector3d prev_pos;
            bool is_first = true;
            for (int32_t i = 0; i < srs->valid_lane_list.size(); ++i)
            {
                auto &ll_curr = srs->valid_lane_list[i];
                auto &front_lc = front ? ll_curr->list.front() : ll_curr->list.back();
                if (is_first)
                {
                    prev_pos = front_lc->get_left();
                    i--;
                    is_first = false;
                    continue;
                }
                double dis = alg::calc_dis(front_lc->get_right(), prev_pos);
                if (dis > fork_merge_dis_threshold)
                {
                    prev_pos = front_lc->get_right();
                    continue;
                }
                front_lc->width = 0;
                front_lc->pos = prev_pos;
                // front_lc->left_fls->pos.z() = prev_pos.z();
                // front_lc->right_fls->pos.z() = prev_pos.z();
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::clear_invalid_lc(SubRoadSegment *road_segment)
        {
            for (int i = 0; i < road_segment->valid_lane_list.size(); ++i)
            {
                auto &ll = road_segment->valid_lane_list[i];
                for (int j = 0; j < ll->list.size(); ++j)
                {
                    auto &lc = ll->list[j];
                    if (lc->invalid())
                    {
                        ll->list.erase(ll->list.begin() + j--);
                        continue;
                    }
                }
                if (ll->invalid() || ll->list.size() == 0)
                {
                    road_segment->valid_lane_list.erase(road_segment->valid_lane_list.begin() + i--);
                    continue;
                }
            }
            return fsdmap::SUCC;
        }

        bool RoadModelProcFormatLane::get_context_poss_break(RoadModelSessionData *session,
                                                             RoadSegment *road_segment, bool front)
        {
            // double max_length = FLAGS_format_lane_get_segment_max_distance;

            // auto &poss = front ? road_segment->pos_sample_list.back() : road_segment->pos_sample_list.front();
            // std::vector<RoadSegment*> other_road_segment;
            // road_segment->get_context_road_segment(front, max_length, other_road_segment);

            // if (other_road_segment.size() == 0) {
            //     return false;
            // }
            // PosSample* break_poss = NULL;
            // for (auto &tit : other_road_segment) {
            //     auto &other_poss = front ? tit->pos_sample_list.front() : tit->pos_sample_list.back();
            //     if (other_poss->lane_param.has_break_pos) {
            //         break_poss = other_poss;
            //         break;
            //     }
            // }
            // if (break_poss != NULL) {
            //     RoadModelSessionData::set_poss_break_pos(poss, break_poss->lane_param.break_pos);
            //     for (auto &tit : other_road_segment) {
            //         auto &other_poss = front ? tit->pos_sample_list.front() : tit->pos_sample_list.back();
            //         RoadModelSessionData::set_poss_break_pos(other_poss, break_poss->lane_param.break_pos);
            //     }
            //     return true;
            // }

            // if (other_road_segment.size() > 1) {
            //     RoadModelSessionData::set_poss_break_pos(poss, poss);
            //     for (auto &tit : other_road_segment) {
            //         auto &other_poss = front ? tit->pos_sample_list.front() : tit->pos_sample_list.back();
            //         RoadModelSessionData::set_poss_break_pos(other_poss, poss);
            //     }
            // } else {
            //     for (auto &tit : other_road_segment) {
            //         auto &other_poss = front ? tit->pos_sample_list.front() : tit->pos_sample_list.back();
            //         RoadModelSessionData::set_poss_break_pos(other_poss, other_poss);
            //         RoadModelSessionData::set_poss_break_pos(poss, other_poss);
            //     }
            // }
            return true;
        }

        int RoadModelProcFormatLane::is_conflict(LaneCenterLine *ll1, LaneCenterLine *ll2)
        {
            double rate_threshold = FLAGS_format_lane_ll_conflict_rate_threshold;
            std::vector<Eigen::Vector3d> poly1;
            std::vector<Eigen::Vector3d> poly2;
            for (auto &lc : ll1->list)
            {
                if (lc->break_status == 2)
                {
                }
                if (poly1.size() > 0)
                {
                    auto &last_pt = poly1.back();
                    auto pt = lc->get_right();
                    if (alg::calc_dis(last_pt, pt) < 0.1)
                    {
                        continue;
                    }
                }
                poly1.push_back(lc->get_right());
            }
            for (int i = ll1->list.size() - 1; i >= 0; --i)
            {
                auto &lc = ll1->list[i];
                if (poly1.size() > 0)
                {
                    auto &last_pt = poly1.back();
                    auto pt = lc->get_left();
                    if (alg::calc_dis(last_pt, pt) < 0.1)
                    {
                        continue;
                    }
                }
                poly1.push_back(lc->get_left());
            }
            for (auto &lc : ll2->list)
            {
                if (poly2.size() > 0)
                {
                    auto &last_pt = poly2.back();
                    auto pt = lc->get_right();
                    if (alg::calc_dis(last_pt, pt) < 0.1)
                    {
                        continue;
                    }
                }
                poly2.push_back(lc->get_right());
            }
            for (int i = ll2->list.size() - 1; i >= 0; --i)
            {
                auto &lc = ll2->list[i];
                if (poly2.size() > 0)
                {
                    auto &last_pt = poly2.back();
                    auto pt = lc->get_left();
                    if (alg::calc_dis(last_pt, pt) < 0.1)
                    {
                        continue;
                    }
                }
                poly2.push_back(lc->get_left());
            }
            if (poly1.size() <= 2 || poly2.size() <= 2)
            {
                return false;
            }
            try
            {
                double rate = alg::calc_area_rate(poly1, poly2, 1);
                return rate >= rate_threshold;
            }
            catch (const std::exception &e)
            {
                // std::string file = utils::fmt("error_list_{}_{}", global::global_tile_id, ll1->list.front()->get_wgs());
                // save_point_to_log(poly1, file.c_str());
                // file = utils::fmt("error_list_{}_{}", global::global_tile_id, ll2->list.front()->get_wgs());
                // save_point_to_log(poly2, file.c_str());
                LOG_ERROR("failed to get intersection[{}, ll1={}, ll2={}]",
                          e.what(), ll1->list.front()->wgs(), ll2->list.front()->wgs());
                return false;
            }
        }

        int RoadModelProcFormatLane::init_param(RoadModelSessionData *session)
        {
            // for (auto &road_segment : session->merge_road_segment) {
            //     session->final_road_segment.push_back(road_segment);
            //     for (auto &poss : road_segment->pos_sample_list) {
            //         for (auto &lc : poss->filled_lane_sample) {
            //             if (lc->invalid()) {
            //                 continue;
            //             }
            //             lc->identify_segment = road_segment;
            //         }
            //     }
            // }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::fill_lane_line(RoadModelSessionData *session)
        {
            std::map<LaneCenterFeature *, LaneCenterLine *> lane_map;
            for (int srs_index = 0; srs_index < session->sub_road_segment_list.size(); ++srs_index)
            {
                auto &srs = session->sub_road_segment_list[srs_index];
                auto &road_segment = srs->src_road_segment;
                lane_map.clear();
         
                for (int i = srs->start_index; i <= srs->end_index; ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    auto &boundary = poss->boundary;
                    int lane_index = -1;
                    for (auto &lc : poss->filled_lane_sample)
                    {
                        session->debug_pos(lc->pos);
                        if (lc->invalid())
                        {
                            continue;
                        }
                        if (i == srs->start_index && lc->context.all_next.size() == 0)
                        {
                            continue;
                        }
                        if (i == srs->end_index && lc->context.all_prev.size() == 0)
                        {
                            continue;
                        }
                        if (lc->road_index != srs->road_index)
                        {
                            continue;
                        }
                        if (lc->side_status != srs->link_direction)
                        {
                            continue;
                        }
                        lc->lane_index = ++lane_index;
                        auto &prev_list = lc->context.all_prev;
                        std::vector<LaneCenterFeature *> valid_list;
                        bool new_line = false;
                        if (i != srs->start_index)
                        {
                            if (lc->break_status == 2 && prev_list.size() > 1)
                            {
                                // merge
                                for (auto &tmp_pair : prev_list)
                                {
                                    auto &tmp_lc = tmp_pair.src;
                                    valid_list.push_back(tmp_lc);
                                }
                            }
                            else if (lc->fill_prev == NULL && prev_list.size() == 1)
                            {
                                // fork
                                new_line = true;
                                auto &tmp_lc = prev_list.front().src;
                                valid_list.push_back(tmp_lc);
                            }
                            else if (lc->fill_prev != NULL && prev_list.size() == 1)
                            {
                                // normal
                                auto &tmp_lc = prev_list.front().src;
                                valid_list.push_back(tmp_lc);
                            }
                        }
                        LaneCenterFeature *valid_new_prev = NULL;
                        for (int tmp_index = 0; tmp_index < valid_list.size(); ++tmp_index)
                        {
                            auto &tmp_lc = valid_list[tmp_index];
                            bool is_valid = false;
                            do
                            {
                                if (tmp_lc->key_pose->line_index < srs->start_index)
                                {
                                    break;
                                }
                                if (MAP_NOT_FIND(lane_map, tmp_lc))
                                {
                                    break;
                                }
                                if (new_line)
                                {
                                    valid_new_prev = valid_list.front();
                                    break;
                                }
                                is_valid = true;
                            } while (0);
                            if (is_valid)
                            {
                                continue;
                            }
                            VEC_ERASE(valid_list, tmp_index);
                        }
                        if (valid_list.size() == 0)
                        {
                            auto ll = session->add_ptr(session->lane_center_line_ptr, false);
                            ll->start_lc = lc;
                            ll->end_lc = lc;
                            ll->road_index = srs->road_index;
                            if (srs->filled_lane_list.size() > lane_index)
                            {
                                VEC_INSERT_INDEX(srs->filled_lane_list, lane_index, ll.get());
                            }
                            else
                            {
                                srs->filled_lane_list.push_back(ll.get());
                            }
                            lane_map[lc] = ll.get();
                            if (valid_new_prev != NULL)
                            {
                                ll->list.push_back(valid_new_prev);
                            }
                            ll->list.push_back(lc);
                            // lc->fill_line = ll.get();
                        }
                        else
                        {
                            for (int k = 0; k < valid_list.size(); ++k)
                            {
                                auto &prev_lc = valid_list[k];
                                auto &ll = lane_map[prev_lc];
                                auto &last_lc = ll->list.back();
                                if (last_lc == lc)
                                {
                                    continue;
                                }
                                ll->list.push_back(lc);
                                ll->end_lc = lc;
                                lane_map[lc] = ll;
                                // lc->fill_line = ll;
                            }
                        }
                    }
                }
            }

            for (int srs_index = 0; srs_index < session->sub_road_segment_list.size(); ++srs_index)
            {
                auto &srs = session->sub_road_segment_list[srs_index];
                for (int lane_index = 0; lane_index < srs->filled_lane_list.size(); ++lane_index)
                {
                    auto &ll = srs->filled_lane_list[lane_index];
                    std::vector<int> index_vec;
                    for (auto &lc : ll->list)
                    {
                        index_vec.push_back(lc->lane_index);
                    }
                    int index = lane_index;
                    double rate = 0;
                    if (alg::calc_max_count_int(index_vec, index, rate))
                    {
                        ll->lane_index = index;
                    }
                }
                SORT(srs->filled_lane_list,
                     [](const LaneCenterLine *l, const LaneCenterLine *r)
                     {
                         return l->lane_index < r->lane_index;
                     });
            }

            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::update_lc_opt_dir(RoadModelSessionData *session)
        {
            // // 更新lc的对齐方向
            // double min_dis_threshold = FLAGS_format_lane_align_point_min_dis_threshold;
            // double left_move_dis = FLAGS_format_lane_left_move_dis;
            // double left_double_move_dis = FLAGS_format_lane_left_double_move_dis;
            // double left_move_scope = FLAGS_format_lane_left_move_scope;
            // if (FLAGS_format_lane_move_base_line_enable) {
            //     for (auto &road_segment : session->final_road_segment) {
            //         for (auto &poss : road_segment->pos_sample_list) {
            //             session->debug_pos(poss->pos);
            //             auto &boundary  = poss->boundary;
            //             if (boundary.left == NULL) {
            //                 continue;
            //             }
            //             if (boundary.left->fg->fp->group_type != GROUP_TYPE_LANE_LINE) {
            //                 continue;
            //             }
            //             if (MAP_FIND(session->double_geo_type_group, boundary.left->attr.geo)) {
            //                 DLOG_POINT(boundary.left->fg->fp->pos, "double_line_gap[raw={}, fix={}]",
            //                         boundary.left->fg->fp->info->width / 2, left_double_move_dis);
            //                 if (FLAGS_format_lane_double_line_use_raw_gap_enable) {
            //                     left_move_dis = boundary.left->fg->fp->info->width / 2;
            //                 } else {
            //                     left_move_dis = left_double_move_dis;
            //                 }
            //             }
            //             LaneCenterFeature* left_lc = NULL;
            //             double min_dis = 1000;
            //             for (auto &lc : poss->filled_lane_sample) {
            //                 double dis = alg::calc_vertical_dis(lc->get_left(), boundary.left->pos, boundary.left->dir, true);
            //                 if (dis < min_dis) {
            //                     left_lc = lc;
            //                     min_dis = dis;
            //                 }
            //             }
            //             if (left_lc == NULL) {
            //                 continue;
            //             }
            //             int is_left = alg::judge_left(left_lc->get_left(), boundary.left->pos, left_lc->dir);
            //             if (is_left > 0 &&  min_dis > left_move_scope) {
            //                 continue;
            //             }
            //             Eigen::Vector3d cross_point;
            //             Eigen::Vector3d right_pos = left_lc->get_right();
            //             Eigen::Vector3d left_pos;

            //             Eigen::Vector3d v_dir = alg::get_vertical_dir(left_lc->dir, true);
            //             if (alg::get_cross_point(boundary.left->pos, boundary.left->dir,
            //                    right_pos, v_dir, cross_point, true)) {
            //                 left_pos = cross_point + left_move_dis * v_dir;
            //             } else {
            //                 left_pos = boundary.left->pos + left_move_dis * v_dir;
            //             }
            //             Eigen::Vector3d new_center = (left_pos + right_pos) / 2;
            //             new_center.z() = left_lc->pos.z();
            //             double new_width = alg::calc_dis(left_pos, right_pos, true);
            //             left_lc->pos = new_center;
            //             left_lc->width = new_width;
            //         }
            //     }
            // }
            // for (auto &road_segment : session->final_road_segment) {
            //     for (auto &lane_line : road_segment->valid_lane_list) {
            //         for (int i = 0; i < lane_line->list.size(); ++i) {
            //             auto lc = lane_line->list[i];
            //             auto next_lc = lc->opt_next;
            //             if (i > 0) {
            //                 auto prev_lc = lane_line->list[i - 1];
            //                 double tmp_theta = alg::calc_theta(prev_lc->dir, lc->dir);
            //                 if (tmp_theta > 90) {
            //                     FLOG_POINT(lc->pos, "dir error[theta={}, d1={},{},{}, d2={},{},{}]",
            //                             tmp_theta, prev_lc->dir.x(), prev_lc->dir.y(), prev_lc->dir.z()
            //                             , lc->dir.x(), lc->dir.y(), lc->dir.z());
            //                 }
            //             }

            //             if (i == 0 && lc->opt_prev != NULL
            //                     && alg::calc_dis(lc->opt_prev->pos, lc->pos, true) >= min_dis_threshold) {
            //                 next_lc = lc->opt_prev;
            //             }
            //             if (next_lc == NULL
            //                     || alg::calc_dis(next_lc->pos, lc->pos, true) < min_dis_threshold) {
            //                 if (lc->opt_prev != NULL
            //                         && alg::calc_dis(lc->opt_prev->pos, lc->pos, true) >= min_dis_threshold) {
            //                     next_lc = lc->opt_prev;
            //                 } else if (i < lane_line->list.size() - 2) {
            //                     next_lc = lane_line->list[i + 2];
            //                 } else if (i > 1) {
            //                     next_lc = lane_line->list[i - 2];
            //                 }
            //             }
            //             if (next_lc == NULL) {
            //                 if (lc->ls->left != NULL) {
            //                     lc->left_dir = lc->ls->left->dir;
            //                 } else {
            //                     lc->left_dir = lc->dir;
            //                 }
            //                 if (lc->ls->right != NULL) {
            //                     lc->right_dir = lc->ls->right->dir;
            //                 } else {
            //                     lc->right_dir = lc->dir;
            //                 }
            //             } else {
            //                 alg::calc_dir(lc->get_left(), next_lc->get_left(), lc->left_dir);
            //                 alg::calc_dir(lc->get_right(), next_lc->get_right(), lc->right_dir);
            //             }
            //             // 纠正方向
            //             if (alg::calc_theta(lc->left_dir, lc->dir) > 90) {
            //                 lc->left_dir = -lc->left_dir;
            //             }
            //             if (alg::calc_theta(lc->right_dir, lc->dir) > 90) {
            //                 lc->right_dir = -lc->right_dir;
            //             }
            //         }
            //     }
            // }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::clear_lane_attr(RoadModelSessionData *session)
        {
            for (auto &road_segment : session->sub_road_segment_list)
            {
                for (int index = 0; index < road_segment->valid_lane_list.size(); ++index)
                {
                    auto &ll = road_segment->valid_lane_list[index];
                    // 基线没有加粗和虚线
                    // if (index == 0) {
                    //     if (ll->left_attr.geo == 0 || ll->left_attr.geo == (int)fsdmap::LINE_TYPE_BROKEN) {
                    //         ll->left_attr.geo = (int)fsdmap::LINE_TYPE_SOLID;
                    //     }
                    //     if (ll->left_attr.is_bold == 1) {
                    //         ll->left_attr.is_bold = 0;
                    //     }
                    // }
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::vote_lane_attr(RoadModelSessionData *session, LaneCenterLine *ll)
        {
            double radius = FLAGS_format_lane_calc_curvature_radius;
            std::vector<int32_t> left_geo_type_list;
            std::vector<int32_t> left_color_type_list;
            std::vector<int32_t> right_geo_type_list;
            std::vector<int32_t> right_color_type_list;
            std::vector<int32_t> left_is_bold_list;
            std::vector<int32_t> right_is_bold_list;
            std::vector<int32_t> left_is_double_list;
            std::vector<int32_t> right_is_double_list;
            std::vector<int32_t> left_type_list;
            std::vector<int32_t> right_type_list;

            double total_length = 0;
            LaneCenterFeature *prev_lc = NULL;
            for (int i = 0; i < ll->list.size(); ++i)
            {
                auto &lc = ll->list[i];
                session->debug_pos(lc->pos);
                if (lc->invalid())
                {
                    VEC_ERASE(ll->list, i);
                    continue;
                }
                if (prev_lc != NULL)
                {
                    total_length += alg::calc_dis(lc->pos, prev_lc->pos);
                }
                lc->line_length = total_length;
                if (lc->left_attr.geo <= 0)
                {
                    lc->left_attr.geo = lc->left->attr.geo;
                }
                if (lc->left_attr.color <= 0)
                {
                    lc->left_attr.color = lc->left->attr.color;
                }
                if (lc->left_attr.is_bold <= 0)
                {
                    lc->left_attr.is_bold = lc->left->attr.is_bold;
                }
                if (lc->left_attr.is_double_line <= 0)
                {
                    lc->left_attr.is_double_line = lc->left->attr.is_double_line;
                }
                if (lc->left_attr.type <= 0)
                {
                    lc->left_attr.type = lc->left->attr.type;
                }

                if (lc->right_attr.geo <= 0)
                {
                    lc->right_attr.geo = lc->right->attr.geo;
                }
                if (lc->right_attr.color <= 0)
                {
                    lc->right_attr.color = lc->right->attr.color;
                }
                if (lc->right_attr.is_bold <= 0)
                {
                    lc->right_attr.is_bold = lc->right->attr.is_bold;
                }
                if (lc->right_attr.is_double_line <= 0)
                {
                    lc->right_attr.is_double_line = lc->right->attr.is_double_line;
                }
                if (lc->right_attr.type <= 0)
                {
                    lc->right_attr.type = lc->right->attr.type;
                }


                if (lc->left_attr.geo > 0)
                {
                    left_geo_type_list.push_back(lc->left_attr.geo);
                }
                if (lc->left_attr.color > 0)
                {
                    left_color_type_list.push_back(lc->left_attr.color);
                }
                if (lc->left_attr.is_double_line >= 0)
                {
                    left_is_double_list.push_back(lc->left_attr.is_double_line);
                }
                if (lc->left_attr.is_bold >= 0)
                {
                    left_is_bold_list.push_back(lc->left_attr.is_bold);
                }
                if(lc->left_attr.type>=0)
                {
                    left_type_list.push_back(lc->left_attr.type);
                }

                if (lc->right_attr.geo > 0)
                {
                    right_geo_type_list.push_back(lc->right_attr.geo);
                }
                if (lc->right_attr.color > 0)
                {
                    right_color_type_list.push_back(lc->right_attr.color);
                }
                if (lc->right_attr.is_double_line >= 0)
                {
                    right_is_double_list.push_back(lc->right_attr.is_double_line);
                }
                if (lc->right_attr.is_bold >= 0)
                {
                    right_is_bold_list.push_back(lc->right_attr.is_bold);
                }
                if(lc->right_attr.type>=0)
                {
                    right_type_list.push_back(lc->right_attr.type);
                }
                prev_lc = lc;
            }
            ll->left_attr.geo = alg::calc_max_count_int_or<int32_t>(left_geo_type_list, -1);
            ll->left_attr.color = alg::calc_max_count_int_or<int32_t>(left_color_type_list, -1);
            ll->left_attr.is_double_line = alg::calc_max_count_int_or<int32_t>(left_is_double_list, -1);
            ll->left_attr.is_bold = alg::calc_max_count_int_or<int32_t>(left_is_bold_list, -1);
            ll->left_attr.type = alg::calc_max_count_int_or<int32_t>(left_type_list, -1);

            ll->right_attr.geo = alg::calc_max_count_int_or<int32_t>(right_geo_type_list, -1);
            ll->right_attr.color = alg::calc_max_count_int_or<int32_t>(right_color_type_list, -1);
            ll->right_attr.is_double_line = alg::calc_max_count_int_or<int32_t>(right_is_double_list, -1);
            ll->right_attr.is_bold = alg::calc_max_count_int_or<int32_t>(right_is_bold_list, -1);
            ll->right_attr.type = alg::calc_max_count_int_or<int32_t>(right_type_list, -1);

            std::vector<LaneCenterFeature *> context_list;
            std::vector<Eigen::Vector3d> center_pos_list;
            for (int i = 0; i < ll->list.size(); ++i)
            {
                auto &lc = ll->list[i];
                session->debug_pos(lc->pos);
                context_list.clear();
                center_pos_list.clear();
                lc->get_context_list(radius, context_list, [](LaneCenterFeature *curr) -> LaneCenterFeature *
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
                for (auto &tmp_lc : context_list)
                {
                    center_pos_list.push_back(tmp_lc->pos);
                }
                double c_x = 0;
                double c_y = 0;
                double c_r = 1000;
                if (alg::fit_circle(center_pos_list, c_x, c_y, c_r))
                {
                    lc->curvature = c_r;
                }
                else
                {
                    lc->curvature = 1000;
                }
            }
            // if (lane_line->left_attr.geo < 0 || lane_line->right_attr.geo < 0) {
            //     int a = 1;
            // }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::filter_lane_line(RoadModelSessionData *session)
        {
            double min_lane_length = FLAGS_format_lane_lane_line_min_lane_length;
            double max_length_threshold = FLAGS_format_lane_lane_line_max_length_threshold;
            double max_width_threshold = FLAGS_format_lane_lane_line_max_width_threshold;
            double min_gap_threshold = FLAGS_format_lane_lane_line_min_gap_threshold;
            double min_length_gap_threshold = FLAGS_format_lane_lane_line_min_length_gap_threshold;
            std::vector<LaneCenterLine *> length_vec;

            for (auto &srs : session->sub_road_segment_list)
            {
                auto &road_segment = srs->src_road_segment;
                length_vec.clear();
                for (auto &lane_line : srs->filled_lane_list)
                {
                    lane_line->line_param.length = 0;
                    lane_line->line_param.max_width = lane_line->list[0]->width;
                    lane_line->line_param.min_width = lane_line->line_param.max_width;

                    LaneCenterFeature *lc_prev = NULL;
                    for (int i = 0; i < lane_line->list.size(); ++i)
                    {
                        auto &lc = lane_line->list[i];
                        session->debug_pos(lc->pos);
                        if (lc_prev == NULL)
                        {
                            lc_prev = lc;
                            continue;
                        }
                        double prev_dis = alg::calc_dis(lc_prev->pos, lc->pos);
                        if (prev_dis < min_gap_threshold)
                        {
                            // 因前面补齐的会有lc在一起的情况，在此进行过滤
                            lc->filter_status = 7;
                            continue;
                        }
                        if (alg::judge_front(lc_prev->pos, lc->pos, lc->dir))
                        {
                            lc->filter_status = 8;
                            continue;
                        }
                        lane_line->line_param.length += prev_dis;
                        lane_line->line_param.max_width = std::max(lane_line->line_param.max_width, lc->width);
                        lane_line->line_param.min_width = std::min(lane_line->line_param.min_width, lc->width);

                        lc_prev = lc;
                    }
                    length_vec.push_back(lane_line);
                    // vote_lane_attr(session, lane_line);
                }
                std::sort(length_vec.begin(), length_vec.end(),
                          [](const LaneCenterLine *l, const LaneCenterLine *r) -> bool
                          {
                              return l->line_param.length < r->line_param.length;
                          });
                std::vector<LaneCenterFeature *> context_list;
                for (auto &ll : length_vec)
                {
                    for (auto &lc : ll->list)
                    {
                        session->debug_pos(lc->pos);
                    }
                    do
                    {
                        double length_gap = fabs(length_vec.back()->line_param.length - ll->line_param.length);
                        if (length_gap > min_length_gap_threshold && ll->line_param.length < max_length_threshold)
                        {
                            ll->filter_status = 2;
                            break;
                        }
                        if (ll->list.size() <= 1)
                        {
                            ll->filter_status = 3;
                            break;
                        }
                        filter_lane_line_by_length(session, ll, min_lane_length);
                        if (ll->line_param.max_width < max_width_threshold)
                        {
                            ll->filter_status = 4;
                            break;
                        }
                        ll->filter_status = 1;
                    } while (0);
                    // if (ll->invalid()) {
                    //     for (auto &lc : ll->list) {
                    //         session->debug_pos(lc->pos);
                    //         lc->filter_status = 6;
                    //         DLOG_POINT(lc->pos,
                    //                 "lane_line filter lc[length={}, max_length={}, group_size={}]",
                    //                 ll->line_param.length, length_vec.back()->line_param.length, ll->list.size());
                    //     }
                    // }
                    DLOG_POINT2(ll->start_lc->pos, ll->end_lc->pos,
                                "lane line filter[length={}, max_length={}, group_size={}]",
                                ll->line_param.length, length_vec.back()->line_param.length, ll->list.size());
                }
                int32_t road_lane_index = 0;
                for (auto &lane_line : srs->filled_lane_list)
                {
                    if (lane_line->invalid())
                    {
                        continue;
                    }
                    srs->valid_lane_list.push_back(lane_line);
                    lane_line->lane_index = road_lane_index;
                    for (auto &lc : lane_line->list)
                    {
                        lc->road_lane_index = road_lane_index;
                    }
                    road_lane_index++;
                }
                // for (int i = srs->start_index; i < srs->end_index; ++i) {
                //     auto &poss = srs->src_road_segment->pos_sample_list[i];
                //     for (auto &lc : poss->filled_lane_sample) {
                //         if (lc->invalid()) {
                //             continue;
                //         }
                //         poss->valid_lane_sample.push_back(lc);
                //     }
                // }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::merge_sub_road_segment(RoadModelSessionData *session)
        {
            for (auto &srs : session->sub_road_segment_list)
            {
                if (srs->prev == NULL)
                {
                    continue;
                }
                bool has_break = true;
                for (int i = 0; i < srs->valid_lane_list.size(); ++i)
                {
                    auto &ll = srs->valid_lane_list[i];
                    if (ll->invalid())
                    {
                        continue;
                    }
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::vote_valid_side(RoadModelSessionData *session)
        {
            // 两种情况
            // 1：统一展示的几何
            // 2：补全空挡
            double gap_threshold = FLAGS_format_lane_valid_side_gap_threshold;
            for (auto &srs : session->sub_road_segment_list)
            {
                auto &road_segment = srs->src_road_segment;
                for (int i = srs->start_index; i <= srs->end_index; ++i)
                {
                    auto &poss = road_segment->pos_sample_list[i];
                    session->debug_pos(poss->pos);
                    if (poss->invalid())
                    {
                        continue;
                    }
                    // bug 
                    SORT(poss->filled_lane_sample,
                         [&poss](const LaneCenterFeature *l, const LaneCenterFeature *r)
                         {
                             double l_dis = alg::calc_vertical_dis(l->pos, poss->pos, poss->dir, true);
                             double r_dis = alg::calc_vertical_dis(r->pos, poss->pos, poss->dir, true);
                             return l_dis < r_dis;
                         });
                    LaneCenterFeature *left_lc = NULL;
                    for (int j = 0; j < poss->filled_lane_sample.size(); ++j)
                    {
                        auto &lc = poss->filled_lane_sample[j];
                        session->debug_pos(lc->pos);
                        if (lc->invalid())
                        {
                            continue;
                        }
                        if (lc->road_index != srs->road_index)
                        {
                            left_lc = NULL;
                            continue;
                        }
                        if (left_lc == NULL)
                        {
                            left_lc = lc;
                            continue;
                        }
                        if (lc->fill_status == 2)
                        {
                            int a = 1;
                        }
                        left_lc->fill_right = lc;
                        lc->fill_left = left_lc;
                        double gap = alg::calc_vertical_dis(
                            lc->get_left(), left_lc->get_right(), poss->dir, true);
                        if (gap <= gap_threshold && lc->fill_status == 1 && left_lc->fill_status == 1)
                        {
                            // 情况1
                            if (lc->src_status == 2 && left_lc->src_status != 2)
                            {
                                left_lc->init(left_lc->get_left(), left_lc->left_dir,
                                              lc->get_left(), lc->left_dir);
                            }
                            else
                            {
                                lc->init(left_lc->get_right(), left_lc->right_dir,
                                         lc->get_right(), lc->right_dir);
                            }
                            lc->gen_status = 3;
                        }
                        else
                        {
                            // 情况2
                            lc->left_valid_status = 1;
                        }
                        left_lc = lc;
                    }
                }
                // for (int i = 0; i < srs->valid_lane_list.size(); ++i) {
                //     auto &ll = srs->valid_lane_list[i];
                //     if (ll->invalid()) {
                //         continue;
                //     }
                //     auto new_ll = vote_valid_side_by_laneline(session, ll, true);
                //     if (new_ll == NULL) {
                //         continue;
                //     }
                //     VEC_INSERT_INDEX(srs->valid_lane_list, i, new_ll);
                //     ++i;
                // }
                for (int i = 0; i < srs->valid_lane_list.size(); ++i)
                {
                    auto &ll = srs->valid_lane_list[i];
                    if (ll->invalid())
                    {
                        continue;
                    }
                    vote_lane_attr(session, ll);
                    // auto new_ll = vote_valid_side_by_laneline(session, ll, true);
                    // VEC_INSERT_INDEX(srs->valid_lane_list, i, new_ll);
                    // ++i;
                }
            }
            return fsdmap::SUCC;
        }

        LaneCenterLine *RoadModelProcFormatLane::vote_valid_side_by_laneline(RoadModelSessionData *session,
                                                                             LaneCenterLine *ll, bool left)
        {
            double rate_threshold = FLAGS_format_lane_complete_rate_threshold;
            int total_num = 0;
            int valid_num = 0;
            for (int j = 0; j < ll->list.size(); ++j)
            {
                auto &lc = ll->list[j];
                session->debug_pos(lc->pos);
                if (lc->invalid())
                {
                    continue;
                }
                ++total_num;
                auto valid_status = left ? lc->left_valid_status : lc->right_valid_status;
                if (valid_status == 1)
                {
                    ++valid_num;
                }
            }
            if (total_num == 0)
            {
                return NULL;
            }
            float rate = (float)valid_num / total_num;
            if (rate < rate_threshold)
            {
                return NULL;
            }
            auto new_ll = session->add_ptr(session->lane_center_line_ptr);
            for (int j = 0; j < ll->list.size(); ++j)
            {
                auto &lc = ll->list[j];
                if (lc->invalid())
                {
                    continue;
                }
                if (lc->fill_left == NULL)
                {
                    continue;
                }
                auto new_lc = session->add_ptr(session->lane_center_feature_ptr);
                new_lc->init(lc);
                new_lc->init(lc->fill_left->get_right(), lc->fill_left->right_dir,
                             lc->get_left(), lc->left_dir);
                session->debug_pos(new_lc->pos);
                // auto rc = lc->key_pose->boundary.get_road_center(new_lc->pos);
                // if (rc == NULL || rc->invalid()) {
                //     continue;
                // }
                new_lc->gen_status = 2;
                new_lc->road_index = lc->road_index;
                new_ll->list.push_back(new_lc.get());
            }
            return new_ll.get();
        }

        int RoadModelProcFormatLane::make_valid_lc_tree(RoadModelSessionData *session)
        {
            for (auto &srs : session->sub_road_segment_list)
            {
                for (auto &ll : srs->valid_lane_list)
                {
                    if (ll->invalid())
                    {
                        continue;
                    }
                    for (int i = 0; i < ll->list.size(); ++i)
                    {
                        auto &lc = ll->list[i];
                        if (lc->invalid())
                        {
                            continue;
                        }
                        auto &center = *lc;
                        session->valid_lane_sample_group_tree.insert(center.get_left(), lc);
                        session->valid_lane_sample_group_tree.insert(center.get_right(), lc);
                    }
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatLane::save_debug_info(RoadModelSessionData *session)
        {
            if (!FLAGS_format_lane_save_data_enable)
            {
                return fsdmap::SUCC;
            }
            session->set_display_name("format_lane");

            for (auto &srs : session->sub_road_segment_list)
            {
// #if 0
                for (int i = 0; i < srs->valid_lane_list.size(); ++i)
                {
                    auto &ll = srs->valid_lane_list[i];
                    auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "format_lane");
                    log->color = {200, 200, 200};
                    if (srs->road_index != 0)
                    {
                        log->color = {100, 100, 100};
                    }
                    for (int j = 0; j < ll->list.size(); ++j)
                    {
                        auto &lc = ll->list[j];
                        auto &ele = log->add(lc->pos, 2, lc->filter_status);
                        ele.label.label = lc->road_index;
                        if (lc->gen_status == 2)
                        {
                            ele.color = {0, 255, 0};
                            if (srs->road_index != 0)
                            {
                                ele.color = {0, 255, 100};
                            }
                        }
                        else if (lc->gen_status == 3)
                        {
                            ele.color = {255, 255, 0};
                            if (srs->road_index != 0)
                            {
                                ele.color = {255, 255, 100};
                            }
                        }
                    }
                }
// #endif
                // #if 0
                for (int i = 0; i < srs->filled_lane_list.size(); ++i)
                {
                    auto &ll = srs->filled_lane_list[i];
                    // if (!ll->invalid())
                    // {
                    //     continue;
                    // }
                    auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "format_lane");
                    log->color = {255, 0, 0};
                    if (srs->road_index != 0)
                    {
                        log->color = {0, 255, 0};
                    }
                    for (int j = 0; j < ll->list.size(); ++j)
                    {
                        auto &lc = ll->list[j];
                        auto &ele = log->add(lc->pos, 2, ll->filter_status);
                        ele.label.opt_label = lc->road_index;
                    }
                }
                // #endif
            }
// #if 0
            for (auto &srs : session->sub_road_segment_list)
            {
                for (int i = srs->start_index; i < srs->end_index; ++i)
                {
                    auto log = session->add_debug_log(utils::DisplayInfo::LINE, "invalid_lane");
                    auto &poss = srs->src_road_segment->pos_sample_list[i];
                    for (auto &lc : poss->filled_lane_sample)
                    {
                        // if (!lc->invalid()) {
                        //     continue;
                        // }
                        
                        auto &prev_list = lc->context.all_prev;
                        log->color = {255, 255, 255};
                        if (lc->invalid()) {
                            log->color = {255, 0, 0};
                        }
                        // 如果lc是sub_road_segment的起始点，且没有后续的lc，则跳过
                        if (i == srs->start_index && lc->context.all_next.size() == 0) {
                            log->color = {0, 255, 0};
                        } 
                        // 如果lc是sub_road_segment的终点，且没有前序的lc，则跳过
                        if (i == srs->end_index && lc->context.all_prev.size() == 0) {
                            log->color = {0, 0, 255};
                        }
                        // 如果lc的road_index和sub_road_segment的road_index不一致，则跳过
                        if (lc->road_index != srs->road_index) {
                            if(lc->context.all_prev.size() == 0)
                            {
                                log->color = {255, 255, 0};
                            }
                            else{
                                log->color = {30, 144, 255};
                            }
                            
                        }
                        // 如果lc的side_status和sub_road_segment的link_direction不一致，则跳过
                        if (lc->side_status != srs->link_direction) {
                            log->color = {0, 255, 255};
                        }

                        if(prev_list.size() == 0){
                            log->color = {255, 255, 0};
                        }
                        auto &ele = log->add(lc->pos, 2, lc->filter_status);
                        ele.label.opt_label = i;
                    }
                }
            }
// #endif
            session->save_debug_info("format_lane");
            return fsdmap::SUCC;
        }

    }
}
