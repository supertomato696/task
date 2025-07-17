


#define PCL_NO_PRECOMPILE
#include "road_model_proc_split_road.h"

DECLARE_double(display_scope_buff);

DEFINE_bool(split_road_debug_pos_enable, true, "split_road_debug_enable");
DEFINE_bool(split_road_save_data_enable, true, "split_road_save_data_enable");
DEFINE_bool(split_road_save_data_pcd_all_enable, true, "split_road_save_data_pcd_all_enable");

DEFINE_bool(split_road_split_road_by_link_enable, true, "split_road_split_road_by_link_enable");
DEFINE_bool(split_road_gen_by_inter_enable, false, "split_road_gen_by_inter_enable");

DEFINE_bool(split_road_load_pcd_enable, true, "split_road_load_pcd_enable");
DEFINE_double(split_road_extend_road_segment_scope, 20, "split_road_extend_road_segment_scope");
DEFINE_double(split_road_make_line_sample_gap, 2, "split_road_make_line_sample_gap");
DEFINE_double(split_road_intersection_match_radius, 35, "split_road_intersection_match_radius");
DEFINE_double(split_road_max_inter_gap, 10, "split_road_max_inter_gap");
DEFINE_double(split_road_mark_inter_poss_radius, 40, "split_road_mark_inter_poss_radius");

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcSplitRoad::proc(
        RoadModelSessionData* session) {
    session->enable_debug_pos = FLAGS_split_road_debug_pos_enable;
    if (!FLAGS_split_road_split_road_by_link_enable) {
        return fsdmap::process_frame::PROC_STATUS_SUCC;
    }
    // 根据交叉路口位置，划分 link 点到 不同的 link 线上 
    CHECK_FATAL_PROC(split_intersection(session), "split_intersection");
    // 遍历每条link线，判断link线里是不是有断点（相邻link点之间的距离大于10m）， 如果有，就在断点处分割成一段一段
    CHECK_FATAL_PROC(make_road_segment(session), "make_road_segment");

    CHECK_FATAL_PROC(match_road_and_intersection(session), "match_road_and_intersection");

    if (FLAGS_split_road_gen_by_inter_enable) {
        CHECK_FATAL_PROC(extend_intersection_road_segment(session), "extend_intersection_road_segment");

        CHECK_FATAL_PROC(extend_road_between_inter(session), "extend_road_between_inter");
    }

    CHECK_FATAL_PROC(sort_by_length(session), "sort_by_length"); // 按照road_segment的长度排序

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

// 在link_pos_tree中搜索在当前intersecion半径35m范围内的link点,
// 对这些link点根据不同的line id归类到不同的线上，
int RoadModelProcSplitRoad::split_intersection(RoadModelSessionData* session) {
    // 遍历所有的交叉口
    for (auto intersection : session->intersections) {
        // 存放当前交叉口附近的link点
        std::vector<KeyPose*> secs;
        
        // 调用搜索函数，在link_pos_tree(降采样后的)中查找在当前交叉口半径35m内的link点
        search_near_pg_by_intersecion(session, intersection, secs); 

        // 获取当前交叉口的子道路点集合，用于存放归类后的link点
        auto &pos_map = intersection->sub_trail_poses;
        // 把这些secs分别归类到不同的link线上，存放到intersection->sub_trail_poses，
        // 表示有很多条link线都离这个intersection在35m内
        for (auto &poss : secs) {
            
            if (!poss || !poss->from_raw_link) {
                continue;
            }

            // 设置当前link点的交叉口为当前的intersection
            poss->intersection = intersection;
            
            bool has_same_line = false; // 用于标记当前link点是否已经归类到已有的line中

            // 遍历交叉口的所有子道路线
            for (auto &tit : pos_map) {
                // 遍历当前子道路线上的所有link点
                for (auto &tmp_pos : tit.second.list) {
                    // 如果当前link点与已有的link点在同一条道路线，加入到同一条道路线；
                    if (tmp_pos->is_same_line(poss)) {//判断他们的id是否一样，和长度不能超过50
                        tit.second.list.push_back(poss); // 将当前link点加入到对应的道路线上
                        has_same_line = true; // 标记为已经归类
                        break;
                    }
                }
                if (has_same_line) {
                    break; // 如果已经归类到某条道路线，跳出内层循环
                }
            }

            // 如果当前link点没有被归类到任何已有的道路线中，则新建一条道路线
            if (!has_same_line) {
                pos_map[poss] = KeyPoseLine(); // 创建新的道路线
                pos_map[poss].list.push_back(poss); // 将当前link点加入到新的道路线中
                pos_map[poss].form = poss->from_raw_link->form; // 道路类型
                // std::cout << "poss->from_raw_link->form:" << poss->from_raw_link->form << std::endl;

            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcSplitRoad::make_road_segment(RoadModelSessionData* session) {
    double max_inter_gap = FLAGS_split_road_max_inter_gap; // 设定最大断点距离，单位为米（例如：10米）

    LOG_INFO("session->link_sample_list size:{}", session->link_sample_list.size());
    // 遍历每条link线，判断link线里是不是有断点（相邻link点之间的距离大于10m），
    // 如果有，就在断点处分割成一段一段
    for (auto &link : session->link_sample_list) {
        int64_t start_index = 0;  // 起始索引，用于定位每次处理的开始位置

        // 遍历每条link线，将其分割成多个road segment
        while (true) {
            // 创建一个新的RoadSegment实例，表示一段新的道路
            auto road_segment = std::make_shared<RoadSegment>();

            // 记录前一个link点，用于计算相邻link点之间的距离
            KeyPose* prev_lp = NULL;

            // 标记是否存在断点（link点间距大于max_inter_gap）
            bool has_break = false;

            // 遍历当前link线中的所有link点
            for (int64_t i = start_index; i < link->list.size(); ++i) {
                auto &lp = link->list[i];  //lp  link point

                // 如果启用通过交叉口来生成新segment，且当前link点是交叉口，则跳过此link点
                if (FLAGS_split_road_gen_by_inter_enable && lp->intersection != NULL) {
                    continue;
                }

                // 如果prev_lp不是NULL，则计算当前link点与前一个link点的距离
                if (prev_lp != NULL) {
                    double dis = alg::calc_dis(prev_lp->pos, lp->pos);
                    if (dis > max_inter_gap) { // 前后link点间距大于10m，过大，把这条link线从此处打断
                        has_break = true;
                        start_index = i; // 更新起始索引为当前断点的下一个位置
                        break;  // 跳出当前link点的遍历，开始处理下一段
                    }
                }

                // 将当前link点加入到当前RoadSegment中
                lp->road_segment = road_segment.get();
                road_segment->pos_sample_list.push_back(lp);
                prev_lp = lp;  // 更新前一个link点为当前的link点
            }
            if (road_segment->pos_sample_list.size() < 2) { // link切分到后面出现这段road_segment点数少于2个，就不再切分这个link了
                break;
            }

            // 设置RoadSegment的属性
            road_segment->type = 1;  //1 待开发?     目前只有两个点以上road_segment才会被赋为1
            road_segment->link = link;
            road_segment->link_direction = link->link_direction;  // 设置当前RoadSegment的方向

            // 将当前RoadSegment加入到session中的road segment列表
            session->road_segment_ptr.push_back(road_segment);
            session->road_segment_list.push_back(road_segment.get());

            // 如果没有发现断点，则表示该link线已经分割完成，跳出while循环
            if (!has_break) {
                break;
            }
        }
    }
    return fsdmap::SUCC;
}

// 遍历所有的intersections，遍历每个intersection对应的sub_trail_poses中的每个点poss，找到poss->next处对应的RoadSegment，并且点poss的位置在该intersection的后方，存入intersection->in_neighbour。
// 遍历所有的intersections，遍历每个intersection对应的sub_trail_poses中的每个点poss，找到poss->next处对应的RoadSegment，并且点poss的位置在该intersection的前方，存入intersection->out_neighbour。
// 找到与当前intersection关联的LinK线上的Link点 20m范围内前方的所有intersection，放入intersection->out_intersection。
// 找到与当前intersection关联的LinK线上的Link点 20m范围内后方的所有intersection，放入intersection->in_intersection。
int RoadModelProcSplitRoad::match_road_and_intersection(RoadModelSessionData* session) {
    // 遍历所有交叉口
    for (auto &intersection : session->intersections) {
        // 遍历每个交叉口的子路径（sub_trail_poses）
        for (auto &trail : intersection->sub_trail_poses) {
            // 遍历路径中的每一个link点（poss）

            //过滤左右转
            // std::cout << "trail.second.form: " << trail.second.form << std::endl;
            if(trail.second.form == "25" || trail.second.form == "26") {
                continue;
            }
            
            for (auto &poss : trail.second.list) {
                session->debug_pos(poss->pos);

                // 1. 获取与当前link点相连的下一个RoadSegment
                //    判断是否有下一个连接点，其位置与poss->next->pos的距离小于35mturn_road_segment
                // ps： 同一roadsegment上的link点，都塞入intersection->in_neighbour 或者 out_neighbour中
                auto next_road_segment = get_extend_road_segment(session, poss, true);
                if (next_road_segment != NULL) {
                    // 判断交叉口位置是否在当前link点的前方 
                    if (alg::judge_front(intersection->pos, poss->pos, poss->dir)) {
                        // 如果交叉口在前方，则将当前link点添加到该RoadSegment的邻居中
                        intersection->in_neighbour[next_road_segment].push_back(poss);
                        // 标记该link点与交叉口的关系，true表示前方
                        mark_inter_poss(session, intersection, poss, true);
                    } else {
                        // 如果交叉口在后方，则将当前link点添加到该RoadSegment的外部邻居中
                        intersection->out_neighbour[next_road_segment].push_back(poss);
                        // 标记该link点与交叉口的关系，false表示后方
                        mark_inter_poss(session, intersection, poss, false);
                    }
                }
                
        
                // 2. 获取当前link点前方的交叉口（20m以内的交叉口）
                auto next_intersection = get_extend_intersection(session, poss, true);
                if (next_intersection != NULL) {
                    // 如果有前方交叉口，则将当前link点添加到该交叉口的外部邻居列表
                    intersection->out_intersection[next_intersection].push_back(poss);
                }

                // 3. 获取当前link点后方的交叉口（20m以内的交叉口） 
                auto prev_intersection = get_extend_intersection(session, poss, false);
                if (prev_intersection != NULL) {
                    // 如果有后方交叉口，则将当前link点添加到该交叉口的内部邻居列表
                    intersection->in_intersection[prev_intersection].push_back(poss);
                }
            }
        }
    }
    return fsdmap::SUCC;
}

void RoadModelProcSplitRoad::mark_inter_poss(
        RoadModelSessionData* session, Intersection* inter, KeyPose* poss, bool next) {
    double radius = FLAGS_split_road_mark_inter_poss_radius; // 40
    double total_dis = 0;
    bool has_start = false;
    KeyPose* prev_poss = NULL;
    auto road_segment = poss->road_segment;
    for (int i = 0 ; i < road_segment->pos_sample_list.size(); ++i) {
        int index = next ? road_segment->pos_sample_list.size() - 1 - i : i;
        auto &tmp_poss = road_segment->pos_sample_list[index];
        if (tmp_poss == poss) {
            has_start = true;
        }
        if (has_start && prev_poss != NULL) {
            total_dis += alg::calc_dis(prev_poss->pos, tmp_poss->pos);
            if (total_dis > radius) {
                break;
            }
        }
        tmp_poss->intersection = inter;
        tmp_poss->inter_status = next ? 2 : 3;
        prev_poss = tmp_poss;
    }
}

// 找到poss的下一个连接点处对应的RoadSegment，与poss->next->pos的距离在35m以内
RoadSegment* RoadModelProcSplitRoad::get_extend_road_segment(
        RoadModelSessionData* session, KeyPose* poss, bool next) {
    // 获取扩展道路段的最大范围 (默认35米)
    double scope = FLAGS_split_road_extend_road_segment_scope;  

    // 根据next标志决定是向前延伸还是向后延伸
    auto next_poss = next ? poss->next : poss->prev;
    RoadSegment* next_road_segment = NULL;

    double total_dis = 0;

    // 记录上一个位置点
    auto last_poss = next_poss;

    // 开始遍历link点，查找符合条件的道路段
    while (next_poss != NULL) {
        
        // 如果当前link点已经有道路段，且其道路段的类型为1，则认为找到目标道路段
        if (next_poss->road_segment != NULL && next_poss->road_segment->type == 1) {
            next_road_segment = next_poss->road_segment;  // 记录该道路段
            last_poss->inter_status = 1;  // 设置上一个点的交叉口状态
            break;  // 找到目标道路段后跳出循环
        }

        // 如果当前link点属于不同的交叉口，停止遍历
        if (next_poss->intersection != NULL && next_poss->intersection != poss->intersection) {
            break;
        }

        // 获取下一个link点（根据next判断是向前还是向后）
        auto tmp_next_poss = next ? next_poss->next : next_poss->prev;
        
        // 如果下一个link点为空，说明已经到达链表的末尾，停止遍历
        if (tmp_next_poss == NULL) {
            break;
        }

        // 累加从当前点到下一个点的距离
        total_dis += alg::calc_dis(next_poss->pos, tmp_next_poss->pos, true);

        // 如果总距离已经超过了最大范围scope，停止遍历 
        if (total_dis > scope) {//35m
            break;
        }

        // 如果当前link点属于同一交叉口，更新上一个位置点
        if (next_poss->intersection == poss->intersection) {
            last_poss = next_poss;
        }

        // 更新next_poss为tmp_next_poss，继续遍历下一个link点
        next_poss = tmp_next_poss;
    }

    // 返回找到的扩展道路段，或者NULL（如果没有找到）
    return next_road_segment;
}


Intersection* RoadModelProcSplitRoad::get_extend_intersection(
        RoadModelSessionData* session, KeyPose* poss, bool next) {
    double scope = FLAGS_split_road_extend_road_segment_scope; // 20
    auto next_poss = next ? poss->next : poss->prev;
    Intersection* next_intersection = NULL;
    double total_dis = 0;
    while (next_poss != NULL) {
        if (next_poss->intersection != NULL && next_poss->intersection != poss->intersection) {
            next_intersection = next_poss->intersection;
            break;
        }
        if (next_poss->road_segment != NULL) {
            break;
        }
        auto tmp_next_poss = next ? next_poss->next : next_poss->prev;
        if (tmp_next_poss == NULL) {
            break;
        }
        total_dis += alg::calc_dis(next_poss->pos, tmp_next_poss->pos, true);
        if (total_dis > scope) {
            break;
        }
        next_poss = next ? next_poss->next : next_poss->prev;
    }
    return next_intersection;
}

int RoadModelProcSplitRoad::extend_intersection_road_segment(RoadModelSessionData* session) {
    // 遍历每个交叉口
    for (auto &intersection : session->intersections) {
        // 遍历交叉口的入邻接道路段 (in_neighbour)
        for (auto &road_segment : intersection->in_neighbour) {
            // 扩展入邻接道路段的坐标样本
            extand_pos_sample(session, intersection, road_segment.first, true);
        }

        // 遍历交叉口的出邻接道路段 (out_neighbour)
        for (auto &road_segment : intersection->out_neighbour) {
            // 扩展出邻接道路段的坐标样本
            extand_pos_sample(session, intersection, road_segment.first, false);
        }
    }
    // 返回成功标志
    return fsdmap::SUCC;
}


// 计算两个dir方向的交点A1，如果新生成的A1交点距离 A0 超过阈值， 则不进行扩展。
// 将A1和A0之间的点按照间隔阈值进行插值， 并将点加入到新的road_segment，  同时更新A1对应的road_segment
int RoadModelProcSplitRoad::extand_pos_sample(RoadModelSessionData* session,
        Intersection* inter, RoadSegment* road_segment, bool in) {

    // 设置位置间隔，决定每两个样本点之间的距离
    double pos_gap = FLAGS_split_road_make_line_sample_gap;//2 

    // 获取当前道路段的样本点列表
    auto &pos_sample_list = road_segment->pos_sample_list;
    // 根据是"入"道路段还是"出"道路段，选择最后一个或第一个样本点
    auto &last_poss = in ? pos_sample_list.back() : pos_sample_list.front();

    // 计算一个垂直方向的单位向量，用于找到交叉口方向
    Eigen::Vector3d v_dir = alg::get_vertical_dir(last_poss->dir, true);

    // 计算交叉口与最后一个样本点之间的交点（中心交叉点）
    Eigen::Vector3d center_cross;
    if (!alg::get_cross_point(last_poss->pos, last_poss->dir, inter->pos, v_dir,
                center_cross, true)) {
        return fsdmap::SUCC;
    }

    // 计算从样本点到交叉口的方向
    Eigen::Vector3d pos_dir = alg::get_dir(center_cross, last_poss->pos);

    // 将当前道路段作为新道路段（注释掉的部分代码可能表示曾尝试创建新道路段）
    auto new_road_segment = road_segment;

    // 设置前一个样本点为最后一个样本点的位置
    Eigen::Vector3d prev_pos = last_poss->pos;

    // 计算最大距离，用于插值生成新样本点
    double max_dis = alg::calc_dis(last_poss->pos, center_cross, true) - pos_gap;
    if (max_dis < pos_gap * 2) {
        // 如果最大距离小于2倍位置间隔，则不进行任何扩展
        return fsdmap::SUCC;
    }

    // 记录已处理的总距离
    double total_dis = 0;

    // 生成新的位置样本点
    while (total_dis < max_dis) {
        // 为新的样本点分配内存
        auto new_poss = session->add_ptr(session->key_pose_ptr, false);
        new_poss->pos = prev_pos + pos_gap * pos_dir;  // 计算新样本点的位置
        new_poss->dir = in ? pos_dir : -pos_dir;      // 设置新样本点的方向（入或出）

        // 设置该样本点属于当前道路段
        new_poss->road_segment = new_road_segment;

        // 根据方向将新样本点添加到样本点列表的头部或尾部
        if (in) {
            new_road_segment->pos_sample_list.push_back(new_poss.get());
        } else {
            VEC_INSERT(new_road_segment->pos_sample_list, new_poss.get());
        }

        // 将新样本点插入到位置树中（用于快速查找）
        session->link_pos_tree.insert(new_poss->pos, new_poss.get());

        // 更新前一个样本点为当前样本点的位置，并累加总距离
        prev_pos = new_poss->pos;
        total_dis += pos_gap;
    }

    // 更新所有样本点的前向指针以及线段长度
    KeyPose *poss_prev = NULL;
    for (auto &poss : new_road_segment->pos_sample_list) {
        if (poss_prev != NULL) {
            poss->set_prev(poss_prev);               // 设置前一个样本点
            poss->line_length = poss_prev->line_length + pos_gap; // 更新当前样本点的线段长度
        }
        poss_prev = poss;
    }

    // 将新的道路段添加到交叉口的入或出道路段列表中
    if (in) {
        // auto front_poss = new_road_segment->pos_sample_list.front();
        // front_poss->set_prev(last_poss);
        inter->in_road_segment.push_back(new_road_segment);
        // new_road_segment->intersection = inter;
    } else {
        // auto back_poss = new_road_segment->pos_sample_list.back();
        // last_poss->set_prev(back_poss);
        inter->out_road_segment.push_back(new_road_segment);
        // new_road_segment->intersection = inter;
    }
    // session->road_segment_list.push_back(new_road_segment.get());
    return fsdmap::SUCC;
}


int RoadModelProcSplitRoad::extend_road_between_inter(RoadModelSessionData* session) {

    // 遍历所有交叉口
    for (auto &intersection : session->intersections) {

        // 输出当前交叉口的位置（用于调试）
        session->debug_pos(intersection->pos);

        // 遍历该交叉口的所有外部相邻交叉口
        for (auto &other_inter : intersection->out_intersection) {

            // 根据当前交叉口和相邻交叉口生成一个新的道路段
            auto road_segment = gen_between_inter_road_segment(
                    session, other_inter.first, intersection);

            // 如果生成的道路段为空，跳过当前循环
            if (road_segment == NULL) {
                continue;
            }

            // 将道路段的交叉口设置为当前交叉口
            road_segment->intersection = intersection;

            // 将该道路段添加到当前交叉口的道路段集合中
            intersection->inter_road_segment.push_back(road_segment);

            // 将该道路段添加到全局道路段列表中
            session->road_segment_list.push_back(road_segment);
        }
    }
    return fsdmap::SUCC;
}

RoadSegment* RoadModelProcSplitRoad::gen_between_inter_road_segment(
        RoadModelSessionData* session,
        Intersection* in_inter, Intersection* out_inter) {
    double pos_gap = FLAGS_split_road_make_line_sample_gap;
    Eigen::Vector3d start_pos = in_inter->pos;
    Eigen::Vector3d end_pos = out_inter->pos;
    Eigen::Vector3d pos_dir = alg::get_dir(end_pos, start_pos);
    auto new_road_segment = session->add_ptr(session->road_segment_ptr);
    new_road_segment->type = 6;
    new_road_segment->intersection = in_inter;
    // new_road_segment->id = session->max_road_segment_id++;
    Eigen::Vector3d prev_pos = alg::get_vertical_pos(start_pos, pos_dir, 2);
    double max_dis = alg::calc_dis(start_pos, end_pos);
    if (max_dis < pos_gap * 2) {
        return NULL;
    }
    double total_dis = 0;
    int64_t pos_index = 0;
    while (total_dis < max_dis) {
        auto new_poss = session->add_ptr(session->key_pose_ptr);
        new_poss->pos = prev_pos + pos_index * pos_gap * pos_dir;
        new_poss->dir = pos_dir;
        new_poss->road_segment = new_road_segment.get();
        new_road_segment->pos_sample_list.push_back(new_poss.get());
        session->link_pos_tree.insert(new_poss->pos, new_poss.get());

        // prev_pos = new_poss->pos;
        total_dis += pos_gap;
        ++pos_index;
    }
    KeyPose * poss_prev = NULL;
    for (auto &poss : new_road_segment->pos_sample_list) {
        if (poss_prev != NULL) {
            poss->set_prev(poss_prev);
            poss->line_length = poss_prev->line_length + pos_gap;
        }
        poss_prev = poss;
    }
    session->road_segment_list.push_back(new_road_segment.get());
    return new_road_segment.get();
}

// 在link_pos_tree中搜索在当前intersecion半径35m范围内的link点
int RoadModelProcSplitRoad::search_near_pg_by_intersecion(RoadModelSessionData* session,
            Intersection* inter, std::vector<KeyPose*> &secs) {
    float radius = FLAGS_split_road_intersection_match_radius;  // 35
    radius += inter->radius;
    secs.clear();
    std::vector<KeyPose*> tmp_secs;
    session->link_pos_tree.search(inter->pos, radius, tmp_secs);
    for (auto& pos_get : tmp_secs) {
        double dis = alg::calc_dis(inter->pos, pos_get->pos);
        if (dis > radius) {
            continue;
        }
        secs.push_back(pos_get);
    }
    return fsdmap::SUCC;
}

// 按照road_segment的长度排序
int RoadModelProcSplitRoad::sort_by_length(RoadModelSessionData* session) {

    // 遍历所有道路段
    for (auto &rs : session->road_segment_list) {

        // 初始化道路段的总长度为0
        rs->length = 0;

        // 遍历道路段中的所有样本点（pos_sample_list）
        for (int i = 0; i < rs->pos_sample_list.size(); ++i) {
            auto &poss = rs->pos_sample_list[i];

            // 设置样本点的序号和所属道路段
            poss->line_index = i;
            poss->road_segment = rs;

            // 初始化过滤状态为0
            poss->filter_status = 0;

            // 将样本点插入到道路段链接树中
            session->road_segment_link_tree.insert(poss->pos, poss);

            // 第一个样本点的线长度为0，继续处理下一个样本点
            if (i == 0) {
                poss->line_length = 0;
                continue;
            }

            // 计算当前样本点与前一个样本点之间的距离，并累加到道路段总长度
            auto &prev_poss = rs->pos_sample_list[i - 1];
            rs->length += alg::calc_dis(prev_poss->pos, poss->pos);

            // 更新当前样本点的线长度为道路段的累计长度
            poss->line_length = rs->length;
        }
    }

    // 对道路段按照长度进行降序排序
    SORT(session->road_segment_list,
        [](const RoadSegment* l, const RoadSegment* r){
            return l->length > r->length;
        });

    // 遍历排序后的道路段，重新为每个道路段分配唯一的ID
    for (int i = 0; i < session->road_segment_list.size(); ++i) {
        auto &rs = session->road_segment_list[i];
        rs->id = i;
    }

    // 返回成功标志
    return fsdmap::SUCC;
}


int RoadModelProcSplitRoad::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_split_road_save_data_enable) {
        return fsdmap::SUCC;
    }
    session->set_display_name("split_road");

    for (int i = 0; i < session->road_segment_list.size(); ++i) {
        auto &road_segment = session->road_segment_list[i];
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "road_segment");
        log->color = {223, 130, 154};
        if (road_segment->type > 1) {
            log->color = {223, 0, 154};
        }
        for (auto &poss : road_segment->pos_sample_list) {
            auto &ele = log->add(poss->pos);
            ele.label.label = i;
            if (poss->inter_status == 2) {
                ele.color = {255, 165, 0};  //橙色
            } else if (poss->inter_status == 3) {
                ele.color = {255, 255, 255}; //白色
            }
        }
    }

    for (auto &inter : session->intersections) {
        auto log = session->add_debug_log(utils::DisplayInfo::POINT, "inter");
        log->color = {255, 0, 0};//路口中心点
        log->add(inter->pos);
    }
    for (auto &inter : session->intersections) {
        auto &pos_map = inter->sub_trail_poses;
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "inter_boundary");
        log->color = {255, 255, 0}; //黄色
        std::vector<KeyPose*> end_points;
        for (auto &tit : pos_map) {
            for (auto &tmp_pos : tit.second.list) {
                if (tmp_pos->inter_status == 1) {
                    end_points.push_back(tmp_pos); //结束点
                }
            }
        }
        Eigen::Vector3d north_dir = {0, 1, 0};
        SORT(end_points, [&](const KeyPose* l, const KeyPose* r)->bool {
                Eigen::Vector3d l_dir = alg::get_dir(l->pos, inter->pos);
                Eigen::Vector3d r_dir = alg::get_dir(r->pos, inter->pos);
                double l_theta = alg::calc_theta_with_dir(l_dir, north_dir);
                double r_theta = alg::calc_theta_with_dir(r_dir, north_dir);
                return l_theta < r_theta;
                });
        for (auto &pt : end_points) {
            log->add(pt->pos);
        }
    }
    
    
    for (auto& inter : session->intersections){
        for (int lane_side = 0; lane_side < 2; ++lane_side){    
            const auto &link_data = lane_side == 0 ? inter->in_neighbour : inter->out_neighbour;
            
            auto log = session->add_debug_log(utils::DisplayInfo::POINT, "split_inter_enter_exit");
            for (auto &entry : link_data){
                if(lane_side == 0){
                    log->color = {0, 255, 0};
                }else{
                    log->color = {255, 0, 0};
                }
                
                for (auto &poss : entry.second){
                    log->add(poss->pos);
                }
            }
            
        }
    }
    
    session->save_debug_info("split_road");
    
    // if(1){
        //     session->set_display_name("inter_enter_exit");
    //     for (auto& inter : session->intersections){
        //         for (int lane_side = 0; lane_side < 2; ++lane_side){     //只看进入
        //             const auto &link_data = lane_side == 0 ? inter->in_neighbour : inter->out_neighbour;
        
        //             auto log = session->add_debug_log(utils::DisplayInfo::POINT, "split_inter_enter_exit");
        //             for (auto &entry : link_data){
            //                 if(lane_side == 0){
                //                     log->color = {0, 255, 0};
                //                 }else{
    //                     log->color = {255, 0, 0};
    //                 }

    //                 for (auto &poss : entry.second){
    //                     log->add(poss->pos);
    //                 }
    //             }
                
    //         }
    //     }
    //     session->save_debug_info("split_inter_enter_exit");
    // }



    return fsdmap::SUCC;
}

}
}
