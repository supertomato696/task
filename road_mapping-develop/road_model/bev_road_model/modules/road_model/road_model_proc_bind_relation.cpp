

#include "road_model_proc_bind_relation.h"

#include <Eigen/Eigenvalues>
#include <osqp/osqp.h>

#include <gdal_priv.h>
#include <ogrsf_frmts.h>

DEFINE_bool(bind_relation_enable, true, "bind_relation_enable");
DEFINE_bool(joint_group_debug_enable, true, "joint_group_debug_enable");
DEFINE_bool(arrow_binding_debug_enable, true, "arrow_binding_debug_enable");
DEFINE_bool(stop_line_binding_debug_enable, true, "stop_line_binding_debug_enable");
DEFINE_bool(cross_walk_binding_debug_enable, true, "cross_walk_binding_debug_enable");
DEFINE_bool(intersection_binding_debug_enable, true, "intersection_binding_debug_enable");
DEFINE_bool(intersection_binding_cross_walk_debug_enable, true, "intersection_binding_cross_walk_debug_enable");

DEFINE_double(lukou_pos_search_radius, 100.0, "lukou_pos_search_radius");
DEFINE_double(arrow_search_radius_m, 7.0, "arrow_search_radius_m");
DEFINE_double(stop_line_search_radius_m, 7.0, "stop_line_search_radius_m");   // 停止线的参数
DEFINE_double(cross_walk_search_radius_m, 7.0, "cross_walk_search_radius_m"); // 人行横道的参数
DEFINE_double(junction_search_radius_m, 10.0, "junction_search_radius_m");    // 路口搜索的参数

DEFINE_double(intersection_search_radius_m, 60.0, "intersection_search_radius_m"); // 路口搜索的参数
DEFINE_double(intersection_closet_dis_m, 15.0, "intersection_closet_dis_m");       // 末端靠近路口边界的距离

DEFINE_double(broken_group_search_radius_m, 15.0, "group_search_radius");
DEFINE_double(group_search_degree, 30.0, "group_search_degree");
DEFINE_double(group_topology_search_radius_m, 3.0, "group_topology_search_radius_m");
DEFINE_double(attr_lukou_rb_search_radius, 1.0, "attr_lukou_rb_search_radius");

#define COLOR_GREEN "\033[32m"
#define COLOR_RESET "\033[0m"

namespace fsdmap::road_model
{

  fsdmap::process_frame::PROC_STATUS RoadModelProcBindRelation::proc(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    if (not FLAGS_bind_relation_enable)
    {
      return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }

    CHECK_FATAL_PROC(make_road_lane_center_tree(session), "make_road_lane_center_tree");

    CHECK_FATAL_PROC(bind_arrow_to_lane_section(session), "bind_arrow_to_lane_section");            // 箭头与车道绑定
    CHECK_FATAL_PROC(bind_stop_line_to_lane_section(session), "bind_stop_line_to_lane_section");    // 停止线与车道绑定
    CHECK_FATAL_PROC(bind_cross_walk_to_lane_section(session), "bind_cross_walks_to_lane_section"); // 人行道与车道绑定
    CHECK_FATAL_PROC(bind_junction_to_lane_section(session), "bind_junction_to_lane_section");      // 路口与车道绑定
    CHECK_FATAL_PROC(bind_intersection_to_lane_section(session), "bind_intersection_to_lane_section");  // 路口与车道绑定
    CHECK_FATAL_PROC(bind_intersection_to_cross_walk_section(session), "bind_intersection_to_cross_walk_section");  // 路口与人行道绑定

    CHECK_FATAL_PROC(attribute_for_lukoubd(session), "attribute_for_lukoubd"); // 将路口的框按照路口点的类型不同，打断成道路段存储

    CHECK_FATAL_PROC(delete_rbsegment_contined_lukoubd(session), "delete_rbsegment_contined_lukoubd");

    CHECK_FATAL_PROC(bind_areas_to_junction(session), "bind_areas_to_junction");

    CHECK_FATAL_PROC(joint_broken_lane_group(session), "joint_broken_lane_group");

    CHECK_FATAL_PROC(joint_invalid_topology(session), "joint_invalid_topology");

    CHECK_FATAL_PROC(delete_lanegroup_contined_lukoubd_withsuccs(session), "delete_lanegroup_contined_lukoubd_withsuccs");

    CHECK_FATAL_PROC(save_joint_debug_info(session), "save_joint_debug_info");

    CHECK_FATAL_PROC(save_arrow_binding_debug_info(session), "save_arrow_binding_debug_info");
    CHECK_FATAL_PROC(save_stop_line_binding_debug_info(session), "save_stop_line_binding_debug_info");
    CHECK_FATAL_PROC(save_cross_walk_binding_debug_info(session), "save_cross_walk_binding_debug_info");

    CHECK_FATAL_PROC(save_intersection_binding_debug_info(session), "save_intersection_binding_debug_info");
    CHECK_FATAL_PROC(save_intersection_in_out_binding_debug_info(session), "save_intersection_binding_debug_info");
    CHECK_FATAL_PROC(save_intersection_binding_cross_wallk_debug_info(session), "save_intersection_binding_cross_wallk_debug_info");

    // CHECK_FATAL_PROC(export_to_shape_file(session), "export_to_shape_file");

    return fsdmap::process_frame::PROC_STATUS_SUCC;
  }

  int RoadModelProcBindRelation::make_road_lane_center_tree(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);
    session->road_lane_center_tree.RemoveAll();
    for (const auto &lane_group : session->new_lane_groups)
    {
      // 遍历每个车道组中的车道信息
      for (const auto &lane : lane_group->lane_line_info)
      {
        // 遍历每个车道的车道中心特征列表
        //  int index = 0;
        for (const auto &lane_center : lane->lane_center_feature_list)
        {
          // 将车道中心点的位置和对应的车道中心特征插入到road_lane_center_tree
          session->road_lane_center_tree.insert(lane_center->pos, lane_center);
          // std::cout << COLOR_GREEN <<"index" << index << "lane_center->pos" << lane_center->pos << COLOR_RESET << "\n" << std::endl;
          // index ++;
        }
      }
    }
    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::bind_arrow_to_lane_section(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    for (const auto &object : session->raw_object_ret_list)
    {
      if (object->ele_type != 3)
      {
        continue;
      }
      auto &pos = object->pos;                     // 获取箭头的位置
      std::vector<LaneCenterFeature *> search_sec; // 存储搜索到的车道中心

      // 在 road_lane_center_tree 中搜索箭头附近的车道中心
      session->road_lane_center_tree.search(pos, static_cast<float>(FLAGS_arrow_search_radius_m), search_sec); // 搜到的不是顺序的，是无规律的

      // 遍历所有搜到的车道中心
      if (search_sec.size() == 0)
      {
        continue;
      }
      double min_dis = std::numeric_limits<double>::max();
      int index = 0;
      for (int i = 0; i < search_sec.size(); ++i)
      {
        const auto &lane_center = search_sec[i]; // 使用索引 i 获取每个车道中心

        // 更新最小距离和最小距离对应的车道中心位置
        const double dis = alg::calc_dis(pos, lane_center->pos);
        if (dis < min_dis)
        {
          min_dis = dis;
          index = i; 
        }
      }
      // 判断箭头是否真正位于车道中心的范围内
      auto lane_center = search_sec[index];
      if (lane_center->in_scope(pos))
      {
        // 如果箭头在范围内，将箭头添加到对应车道的箭头列表
        lane_center->road_lane_line->bind_arrow_list.emplace_back(object.get());
        object->obj_bind_lanes.emplace_back(lane_center->road_lane_line);

        //TODO: cxf箭头转向存在车道哪里，待讨论确定，需要放在中心点吗？ 
        for (const auto &lane_center_feature : lane_center->road_lane_line->lane_center_feature_list)
        {
          lane_center_feature->turn_type = object->type; // 将箭头的类型赋给车道特征点
     
        }
      }
    }
    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::bind_stop_line_to_lane_section(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    for (const auto &object : session->raw_object_ret_list)
    {
      if (object->ele_type != 6){ 
        continue;
      }

      auto &pos = object->pos; // 获取停止线的位置
      float length = object->length;
      std::vector<LaneCenterFeature *> search_sec; // 存储搜索到的车道中心

      // 在 road_lane_center_tree 中搜索停止线附近的车道中心
      session->road_lane_center_tree.search(pos, static_cast<float>(FLAGS_stop_line_search_radius_m), search_sec);

      for (const auto &lane_center : search_sec)
      { // 遍历搜索到的车道中心
        if (lane_center->in_scope_stop_line(pos, length))
        {
          // 如果停止线在范围内，将停止线添加到对应车道的停止线列表
          lane_center->stop_status = 1; // 1 代表停止状态
          lane_center->road_lane_line->bind_stop_line_list.emplace_back(object.get());
          object->obj_bind_lanes.emplace_back(lane_center->road_lane_line);
        }
      }
    }

    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::bind_cross_walk_to_lane_section(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    for (const auto &object : session->raw_object_ret_list)
    {
      if (object->ele_type != 5)
      { 
        continue;
      }

      auto &pos = object->pos;
      float length = object->length;
      std::vector<LaneCenterFeature *> search_sec; // 存储搜索到的车道中心

      // 在 road_lane_center_tree 中搜索人行道附近的车道中心
      session->road_lane_center_tree.search(pos, static_cast<float>(FLAGS_cross_walk_search_radius_m), search_sec);

      for (const auto &lane_center : search_sec){ 
        // 判断人行道是否真正位于车道中心的范围内
        if (lane_center->in_scope_stop_line(pos, length)){ 
          lane_center->road_lane_line->cross_walk_list.emplace_back(object.get());
          object->obj_bind_lanes.emplace_back(lane_center->road_lane_line);
        }
      }
    }

    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::bind_junction_to_lane_section(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    for (auto &junction : session->new_junction_list)
    {
      auto &pos = junction->pos;
      std::vector<LaneCenterFeature *> search_sec; // 存储搜索到的车道中心

      // 在 road_lane_center_tree 中搜索路口的车道中心
      session->road_lane_center_tree.search(pos, static_cast<float>(FLAGS_stop_line_search_radius_m), search_sec);

      for (const auto &lane_center : search_sec)
      { // 遍历搜索到的车道中心
        lane_center->road_lane_line->junction_list.emplace_back(junction.get());
      }
    }

    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::bind_intersection_to_lane_section(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    for (auto &intersection : session->raw_intersections)
    {
      auto &pos = intersection->pos;
      std::vector<LaneCenterFeature *> search_sec; // 存储搜索到的车道中心

      // 在 road_lane_center_tree 中搜索路口的车道中心
      session->road_lane_center_tree.search(pos, static_cast<float>(FLAGS_intersection_search_radius_m), search_sec);

      if (search_sec.empty()){
        continue;
      }

      for (const auto &lane_center : search_sec){ 
        //绑定靠近路口边界的最近车道
        double min_dis = 100000;
        for (auto &inter_point : intersection->lukou_poly_pts)
        {
          const double dis = alg::calc_dis(lane_center->pos, inter_point);
          if (dis < min_dis)
          {
            min_dis = dis;
          }
        }

        if (min_dis < FLAGS_intersection_closet_dis_m)
        {
          lane_center->road_lane_line->intersection_list.emplace_back(intersection.get());
          intersection->all_lanes.emplace_back(lane_center->road_lane_line);

          // 判断进入退出
          if (alg::judge_front(intersection->pos, lane_center->pos, lane_center->dir))
          {
            lane_center->road_lane_line->center_lane_boundary_info->is_entrance = true;
            intersection->in_lanes.emplace_back(lane_center->road_lane_line);
            intersection->in_loads.insert(lane_center->road_lane_line->lane_group_info);
          }
          else
          { 
            lane_center->road_lane_line->center_lane_boundary_info->is_exit = true;
            intersection->out_lanes.emplace_back(lane_center->road_lane_line);
            intersection->out_loads.insert(lane_center->road_lane_line->lane_group_info);
          }
        }
      }
    }
    return fsdmap::SUCC;
  }


  int RoadModelProcBindRelation::bind_intersection_to_cross_walk_section(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    for (auto &inter : session->raw_intersections){
      for(auto &object : session->raw_object_ret_list)
      {
        if (object->ele_type != 5){ 
          continue;
        }

        if(alg::point_in_polygon(object->pos, inter->lukou_poly_pts)){
          object->obj_bind_intersections.emplace_back(inter.get());
          inter->cross_walk_list.emplace_back(object.get());
        }
      }

    }
    return fsdmap::SUCC;
  }


  int RoadModelProcBindRelation::joint_broken_lane_group(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    assign_group_index(session, true); // 为当前会话分配车道组的索引

    // 如果前后车道组都为空，则说明车道组拓扑结构已经完成，直接返回
    if (pre_group_v_.empty() or post_group_v_.empty())
    {
      LOG_INFO("Total lane group's topology is complete.")
      return fsdmap::SUCC;
    }

    fix_lane_direction(session); // 修正车道的行驶方向（可能是基于交通规则或者道路设计）

    build_broken_group_pair(session); // 构建断裂的车道组对（可能是指道路之间的连接或拆分）

    // 如果没有找到新的车道组对，表示没有需要处理的车道组，直接返回
    if (broken_group_pre_2_post_set_.empty())
    {
      LOG_INFO("Don't find new group pair.")
      return fsdmap::SUCC;
    }

    build_new_group_boundary_info(session); // 构建新的车道组边界信息（可能是根据断裂车道组对创建新组）

    build_new_group(session); // 创建新的车道组，基于前面处理的数据

    return fsdmap::SUCC;
  }

  // 目的：为每个车道组分配所索引
  void RoadModelProcBindRelation::assign_group_index(RoadModelSessionData *session, bool flag)
  {
    CHECK_NOTNULL(session);

    pre_group_v_.clear();
    post_group_v_.clear();

    int lane_group_index = 1 * 1e8, line_index = 3 * 1e8, road_lane_index = 4 * 1e8, boundary_index = 2 * 1e8;

    // 遍历新的车道组列表，将车道组的索引分配给每个车道组
    for (int i = 0; i < session->new_lane_groups.size(); ++i)
    {
      // 获取当前车道组并分配组索引
      auto &lane_group = session->new_lane_groups[i];
      if(flag){
        lane_group->group_index = i; // 给予车道组一个索引（序号）
      }else{
        lane_group->group_index = lane_group_index ++; // 给予车道组一个索引（序号）
      }
      
      // 将车道组添加到前后拓扑列表中
      pre_group_v_.emplace_back(lane_group);  // 添加到前置拓扑组
      post_group_v_.emplace_back(lane_group); // 添加到后置拓扑组

      if(!flag){
        RoadLaneBoundaryInfo* rb = nullptr;
        for(int i = 0; i < lane_group->lane_line_info.size(); i ++){
          auto &lane_info = lane_group->lane_line_info[i];
          lane_info->road_lane_id = road_lane_index++;
          if(rb != lane_info->left_lane_boundary_info && i > 0)
          {
            line_index++;
          }
          lane_info->center_lane_boundary_info->lane_id = line_index;
          lane_info->left_lane_boundary_info->lane_id = line_index++;
          lane_info->right_lane_boundary_info->lane_id = line_index;
          rb = lane_info->right_lane_boundary_info;  
          if(i == lane_group->lane_line_info.size() - 1){
            line_index++;
          }
        }

        for (auto it = lane_group->left_barrier_segment_info.begin(); it != lane_group->left_barrier_segment_info.end();) 
        {
          auto barrier = *it;
          if (barrier->point_info.size() < 2) {
            it = lane_group->left_barrier_segment_info.erase(it); 
          } else {
            barrier->boundary_id = boundary_index++;
            ++it; 
          }
        }
        for (auto it = lane_group->right_barrier_segment_info.begin(); it != lane_group->right_barrier_segment_info.end();) 
        {
          auto barrier = *it;
          if (barrier->point_info.size() < 2) {
            it = lane_group->right_barrier_segment_info.erase(it); 
          } else {
            barrier->boundary_id = boundary_index++;
            ++it; 
          }
        }

      }
    }
  }

  void RoadModelProcBindRelation::fix_lane_direction(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    // 遍历会话中的所有车道组
    for (auto &lane_group : session->new_lane_groups)
    {

      // 遍历当前车道组中的所有车道
      for (auto &lane : lane_group->lane_line_info)
      {
        const int pt_size = lane->center_lane_boundary_info->line_point_info.size();

        // 如果车道的点数小于或等于1，则跳过该车道
        if (pt_size <= 1)
        {
          break;
        }

        // 确定前后第二个点的索引位置，用于计算方向
        const int front_second_size = pt_size > 2 ? 2 : 1; // 若车道点数大于2，则选择第三个点，否则选择第二个点
        const auto front_second_pt = lane->center_lane_boundary_info->line_point_info[front_second_size];

        const int back_second_size = pt_size > 2 ? pt_size - 3 : pt_size - 2; // 计算倒数第三个点的索引
        const auto back_second_pt = lane->center_lane_boundary_info->line_point_info[back_second_size];

        // 获取车道前后端点的位置
        auto &front_pt = lane->center_lane_boundary_info->line_point_info.front();
        auto &back_pt = lane->center_lane_boundary_info->line_point_info.back();

        // 计算车道前后两段的方向
        const auto front_line_dir = alg::get_dir(front_second_pt->pos, front_pt->pos);
        const auto back_line_dir = alg::get_dir(back_pt->pos, back_second_pt->pos);

        // 计算前后端点方向的夹角（单位：度）
        const auto front_degree_gap = std::acos(front_pt->dir.dot(front_line_dir)) * 180 / M_PI;
        const auto back_degree_gap = std::acos(back_pt->dir.dot(back_line_dir)) * 180 / M_PI;

        // 如果前端点的方向夹角大于90度，则修正其方向
        if (front_degree_gap > 90)
        {
          front_pt->dir = front_line_dir;
        }

        // 如果后端点的方向夹角大于90度，则修正其方向
        if (back_degree_gap > 90)
        {
          back_pt->dir = back_line_dir;
        }
      }
    }
  }

  /**
   * @brief 车道组配对
   * 1. 从前向后找，用后继的首点做树进行车道点匹配。
   * 2. 从后向前找，用前序的尾点做树进行车道点匹配。
   * 3. 配对条件：距离不超过15米，角度差不超过30度。
   * 4. 若有多个配对候选，选择最近的配对。
   */
  void RoadModelProcBindRelation::build_broken_group_pair(RoadModelSessionData *session)
  {
    // 检查session对象是否有效
    CHECK_NOTNULL(session);

    // 清空之前存储的车道组配对关系
    broken_group_pre_2_post_set_.clear();

    // ========================== 步骤1：从前向后找，后继的首点做树 ==========================
    lane_pt_tree_.RemoveAll(); // 清空当前的lane_pt_tree，用于存储后继车道组的首点信息

    // 遍历后继车道组(post_group_v_)
    for (const auto &lane_group : post_group_v_)
    {
      for (const auto &lane : lane_group->lane_line_info)
      {
        // 获取每条车道中心线的首点信息
        const auto &front_pt = lane->center_lane_boundary_info->line_point_info.front();
        // 将车道中心线的首点位置插入到lane_pt_tree中
        lane_pt_tree_.insert(front_pt->pos, front_pt);
      }
    }

    // ========================== 步骤2：搜索近邻 & 配对（前序车道组匹配后继车道组） ==========================
    // 遍历前序车道组(pre_group_v_)
    for (const auto &pre_group : pre_group_v_)
    {
      int match_group_index = -1;          // 记录匹配的后继车道组的索引
      double min_match_distance = FLT_MAX; // 记录最小匹配距离
      bool is_isolate = true;              // 标记当前车道组是否仍然是孤立的

      // 遍历前序车道组中的每一条车道
      for (const auto &lane : pre_group->lane_line_info)
      {
        if (not is_isolate)
        {
          break; // 如果已经找到配对，就退出当前车道的匹配过程
        }

        if (lane->center_lane_boundary_info->line_point_info.size() <= 1)
        {
          break; // 如果车道点少于2个，无法进行匹配，跳过该车道
        }

        // 获取前序车道的尾点信息
        const auto &pre_pt = lane->center_lane_boundary_info->line_point_info.back();

        // 查找在指定搜索半径范围内的后继车道点
        std::vector<RoadLinePointInfo *> post_pts;
        lane_pt_tree_.search(pre_pt->pos, static_cast<float>(FLAGS_broken_group_search_radius_m), post_pts);

        // 遍历找到的所有后继车道点
        for (const auto &post_pt : post_pts)
        {
          // 获取后继车道的组索引
          const auto &post_group_index = post_pt->lane_group->group_index;
          if (post_group_index == pre_group->group_index)
          {
            continue; // 跳过与当前前序车道组相同的车道组
          }

          cal_lane_topology(pre_pt, post_pt, post_group_index, &match_group_index, &is_isolate,
                            &min_match_distance);
          if (not is_isolate)
          {
            break; // 如果找到了配对，退出匹配过程
          }
        }
      }

      // 如果找到匹配的后继车道组
      if (match_group_index != -1)
      {
        // 将前序车道组与后继车道组的配对关系记录到map中
        broken_group_pre_2_post_set_[pre_group->group_index].insert(match_group_index);
        // 打印配对日志
        LOG(INFO) << "前序找到对应的车道组配对， pre index = " << pre_group->group_index << " post index = "
                  << match_group_index;
      }
    }

    // ========================== 步骤3：从后向前找，前序的尾点做树 ==========================
    lane_pt_tree_.RemoveAll(); // 清空lane_pt_tree，用于存储前序车道组的尾点信息

    // 遍历前序车道组(pre_group_v_)
    for (const auto &lane_group : pre_group_v_)
    {
      for (const auto &lane : lane_group->lane_line_info)
      {
        // 获取每条车道的尾点信息
        const auto &back_pt = lane->center_lane_boundary_info->line_point_info.back();
        // 将车道的尾点位置插入到lane_pt_tree中
        lane_pt_tree_.insert(back_pt->pos, back_pt);
      }
    }

    // ========================== 步骤4：搜索近邻 & 配对（后继车道组匹配前序车道组） ==========================
    // 遍历后继车道组(post_group_v_)
    for (const auto &post_group : post_group_v_)
    {
      int match_group_index = -1;          // 记录匹配的前序车道组的索引
      double min_match_distance = FLT_MAX; // 记录最小匹配距离
      bool is_isolate = true;              // 标记当前车道组是否仍然是孤立的

      // 遍历后继车道组中的每一条车道
      for (const auto &lane : post_group->lane_line_info)
      {
        if (not is_isolate)
        {
          break; // 如果已经找到配对，就退出当前车道的匹配过程
        }

        if (lane->center_lane_boundary_info->line_point_info.size() <= 1)
        {
          break; // 如果车道点少于2个，无法进行匹配，跳过该车道
        }

        // 获取后继车道的首点信息
        const auto &post_pt = lane->center_lane_boundary_info->line_point_info.front();

        // 查找在指定搜索半径范围内的前序车道点
        std::vector<RoadLinePointInfo *> pre_pts;
        lane_pt_tree_.search(post_pt->pos, static_cast<float>(FLAGS_broken_group_search_radius_m), pre_pts);

        // 遍历找到的所有前序车道点
        for (const auto &pre_pt : pre_pts)
        {
          // 获取前序车道的组索引
          const auto &pre_group_index = pre_pt->lane_group->group_index;
          if (pre_group_index == post_group->group_index)
          {
            continue; // 跳过与当前后继车道组相同的车道组
          }

          cal_lane_topology(pre_pt, post_pt, pre_group_index, &match_group_index, &is_isolate,
                            &min_match_distance);
          if (not is_isolate)
          {
            break; // 如果找到了配对，退出匹配过程
          }
        }
      }

      if (match_group_index != -1)
      {
        // 减少重复的log提示
        if (broken_group_pre_2_post_set_[match_group_index].count(post_group->group_index))
        {
          continue;
        }
        broken_group_found_from_post_.insert(std::pair<int, int>(match_group_index, post_group->group_index));
        broken_group_pre_2_post_set_[match_group_index].insert(post_group->group_index);
        LOG(INFO) << "后继找到对应的车道组配对， pre index = " << match_group_index << " post index = "
                  << post_group->group_index;
      }
    }
  }

  /**
   * @brief 计算两条车道的拓扑关系，判断是否符合配对条件。
   *
   * 该函数用于计算两条车道的角度差和距离，判断它们是否符合配对的条件。
   * 如果符合条件，则更新匹配的车道组索引、最小距离以及是否为孤立的状态。
   *
   * @param pre_pt 前序车道点（尾点或首点，取决于配对方向）
   * @param post_pt 后续车道点（尾点或首点，取决于配对方向）
   * @param group_index 当前后继车道组的索引
   * @param match_group_index 如果匹配成功，返回对应的前序车道组的索引
   * @param is_isolate 当前车道是否为孤立状态（即没有匹配）
   * @param min_match_distance 当前最小的车道配对距离，用于更新匹配条件
   */
  void RoadModelProcBindRelation::cal_lane_topology(const RoadLinePointInfo *pre_pt,
                                                    const RoadLinePointInfo *post_pt,
                                                    const int group_index,
                                                    int *match_group_index, bool *is_isolate,
                                                    double *min_match_distance)
  {

    // 计算两条车道线之间的角度误差（单位：度）
    const auto line_degree_gap = std::acos(pre_pt->dir.dot(post_pt->dir)) * 180 / M_PI;

    // 如果角度误差超过阈值，则不考虑此配对
    if (line_degree_gap > FLAGS_group_search_degree)
    {
      return;
    }

    // 计算由前序车道点和后序车道点构成的新的车道组方向
    const auto new_line_dir = alg::get_dir(post_pt->pos, pre_pt->pos);

    // 计算前序车道与新构建车道方向之间的角度误差
    const auto new_degree_gap = std::acos(pre_pt->dir.dot(new_line_dir)) * 180 / M_PI;

    // 计算两点之间的距离
    const double pt_distance = alg::calc_dis(post_pt->pos, pre_pt->pos);

    // 如果两点之间的距离过小，认为车道线之间已经连接，无法配对
    if (pt_distance < 1e-1)
    {
      *match_group_index = -1; // 重置匹配的车道组索引
      *is_isolate = false;     // 标记为非孤立
      return;
    }

    // 如果构建的车道组方向与前序车道的方向差异超过了允许的角度阈值，则跳过匹配
    if (new_degree_gap > FLAGS_group_search_degree)
    {
      return;
    }

    // 如果距离小于当前的最小匹配距离，更新匹配的车道组索引和最小匹配距离
    if (pt_distance < *min_match_distance)
    {
      *match_group_index = group_index;
      *min_match_distance = pt_distance;
    }
  }

  void RoadModelProcBindRelation::build_new_group_boundary_info(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);
    // 遍历所有已分割的车道组对（broken_group_pre_2_post_set_），其中 group_pair.first 是前组索引，group_pair.second 是后续车道组的索引集合
    for (const auto &group_pair : broken_group_pre_2_post_set_)
    {
      // 获取前组的车道信息
      const auto &pre_group = session->new_lane_groups[group_pair.first];

      // 初始化前组中央边界向量，并调用函数获取该组的中央边界信息
      std::vector<RoadLaneBoundaryInfo *> pre_central_boundary_v = {};
      get_group_central_boundary(pre_group, &pre_central_boundary_v);

      // 初始化前组侧边界向量，并调用函数获取该组的侧边界信息
      std::vector<RoadLaneBoundaryInfo *> pre_side_boundary_v = {};
      get_group_side_boundary(pre_group, &pre_side_boundary_v);

      // 遍历与前组匹配的所有后组
      for (const auto &post_group_index : group_pair.second)
      {
        // 获取后组的车道信息
        const auto &post_group = session->new_lane_groups[post_group_index];

        // 检查前组和后组的索引是否相同，若相同则跳过（避免自配对）
        CHECK_NE(pre_group->group_index, post_group->group_index);

        // 初始化后组中央边界向量，并获取该组的所有车道中心线
        std::vector<RoadLaneBoundaryInfo *> post_central_boundary_v = {};
        get_group_central_boundary(post_group, &post_central_boundary_v);

        // 初始化后组侧边界向量，并获取该组的所有车道线
        std::vector<RoadLaneBoundaryInfo *> post_side_boundary_v = {};
        get_group_side_boundary(post_group, &post_side_boundary_v);

        // 步骤 1：匹配车道中心线
        // 初始化一个边界指针向量和配对向量，用于存储匹配的结果
        std::vector<RoadLaneBoundaryInfo *> boundary_ptr_v = {};  // 存储根据前驱后继的车道中心点的配对关系，新建立的车道中心线
        std::vector<int> pair_v = {};   // 前后车道组车道中心线的连接关系
        std::vector<segment_t> new_central_segment = {};

        // 调用函数计算前后车道组中央边界的连接关系，车道中心线不判断交叉属性，new_central_segment为空
        cal_boundary_connect(pre_central_boundary_v, post_central_boundary_v, new_central_segment, &pair_v);  // 计算车道中心线的连接关系

        // 步骤 2：构建新的车道中心线
        // 调用函数，根据配对信息构建新的车道边界
        build_new_boundary(post_group, pre_central_boundary_v, post_central_boundary_v, pair_v, session, &boundary_ptr_v);  

        // 将新构建的中央线边界信息保存到映射表中
        new_group_2_central_boundary_[std::make_pair(group_pair.first, post_group_index)] = boundary_ptr_v;
        new_group_2_lane_pair_[std::make_pair(group_pair.first, post_group_index)] = pair_v;  // 配对的索引对

        // 步骤 3：更新新的车道中心线
        // 遍历所有新生成的车道中心线，将每条车道中心线的起点和终点信息添加到新的车道段中
        for (const auto &new_boundary : boundary_ptr_v)
        {
          const auto &total_pts = new_boundary->line_point_info;  // new_boundary->line_point_info就存了两个点
          bg_point front_pt{total_pts.front()->pos.x(), total_pts.front()->pos.y()};
          bg_point end_pt{total_pts.back()->pos.x(), total_pts.back()->pos.y()};
          new_central_segment.emplace_back(segment_t{front_pt, end_pt});   // 新构建的车道中心线段
        }

        // 步骤 4：匹配车道的左右边界线
        // 清空边界指针和配对向量，准备计算侧边界
        boundary_ptr_v.clear();
        pair_v.clear();

        // 调用函数计算前后车道组的侧边界的连接关系
        // TODO 阅读
        cal_boundary_connect(pre_side_boundary_v, post_side_boundary_v, new_central_segment, &pair_v, false);

        // 构建新的侧边界并保存到映射表中
        // TODO 阅读
        build_new_boundary(post_group, pre_side_boundary_v, post_side_boundary_v, pair_v, session, &boundary_ptr_v);
        new_group_2_side_boundary_[std::make_pair(group_pair.first, post_group_index)] = boundary_ptr_v;
      }
    }
  }

  // 获取车道组的中央边界信息（即每条车道的中心线）
  void RoadModelProcBindRelation::get_group_central_boundary(const RoadLaneGroupInfo *group,
                                                             std::vector<RoadLaneBoundaryInfo *> *central_boundary_v)
  {
    // 遍历车道组中的每条车道，获取并保存其中心线边界信息
    for (const auto &lane_data : group->lane_line_info)
    {
      central_boundary_v->emplace_back(lane_data->center_lane_boundary_info);
    }
  }

  // 获取车道组的车道线信息（即每条车道的左右车道线）
  void RoadModelProcBindRelation::get_group_side_boundary(const RoadLaneGroupInfo *group,
                                                          std::vector<RoadLaneBoundaryInfo *> *side_boundary_v)
  {
    // 获取车道组中第一条车道的左侧车道线
    side_boundary_v->emplace_back(group->lane_line_info.front()->left_lane_boundary_info);

    // 遍历车道组中的每条车道，获取并保存其右侧车道线
    for (const auto &lane_data : group->lane_line_info)
    {
      side_boundary_v->emplace_back(lane_data->right_lane_boundary_info);
    }
  }


 // 将前一段道路（pre_boundary_v）的边界与后一段道路（post_boundary_v）的边界进行匹配。匹配过程基于一系列约束条件，包括距离、交叉情况和角度
  void RoadModelProcBindRelation::cal_boundary_connect(const std::vector<RoadLaneBoundaryInfo *> &pre_boundary_v,
                                                       const std::vector<RoadLaneBoundaryInfo *> &post_boundary_v,
                                                       const std::vector<segment_t> &lane_segment_v,
                                                       std::vector<int> *boundary_pair_result,
                                                       bool is_cal_central)
  {
    CHECK_NOTNULL(boundary_pair_result);

    std::set<int> used_boundary_index = {}; // 防止重复
    std::vector<int> pair_result; // 存储匹配结果

    for (const auto &pre_boundary : pre_boundary_v)
    {
      const auto &pre_pt = pre_boundary->line_point_info.back();
      double min_match_degree = INT_MAX;
      int match_index = -1;

      // 计算pre_boundary的最后一个点与倒数第三（或第二个）个点的方向向量
      const int pre_pt_size = pre_boundary->line_point_info.size();
      const int back_second_size = pre_pt_size > 2 ? pre_pt_size - 3 : pre_pt_size - 2;
      const auto back_second_pt = pre_boundary->line_point_info[back_second_size];
      const auto pre_line_dir = alg::get_dir(pre_pt->pos, back_second_pt->pos);

      for (int j = 0; j < post_boundary_v.size(); ++j)
      {
        if (used_boundary_index.count(j)) // 对于每一个post_boundary_v，只匹配一次
        {
          continue;
        }

        // 距离约束
        const auto &post_pt = post_boundary_v[j]->line_point_info.front();
        double pt_distance = alg::calc_dis(post_pt->pos, pre_pt->pos);
        if (pt_distance > 1.5 * FLAGS_broken_group_search_radius_m) // 1.5 * 15m
        {
          continue;
        }

        // 交叉约束
        bool is_cross = false;
        if (not is_cal_central) // 如果是非车道中心线，检测是否存在交叉
        {
          bg_point front_pt{pre_pt->pos.x(), pre_pt->pos.y()};
          bg_point end_pt{post_pt->pos.x(), post_pt->pos.y()};
          segment_t new_boundary{front_pt, end_pt};

          for (const auto &lane_segment : lane_segment_v)
          {
            if (is_intersect(new_boundary, lane_segment))
            {
              is_cross = true;
              break;
            }
          }
        }
        if (is_cross)
        {
          continue;
        }

        // 角度约束
        const auto new_line_dir = alg::get_dir(post_pt->pos, pre_pt->pos);
        const auto degree_gap = std::acos(pre_line_dir.dot(new_line_dir)) * 180 / M_PI;
        if (degree_gap > FLAGS_group_search_degree) // 30
        {
          continue;
        }
        if (degree_gap < min_match_degree)  // 取最小的角度差，作为匹配结果
        {
          min_match_degree = degree_gap;
          match_index = j;
        }
      }

      used_boundary_index.insert(match_index);
      pair_result.emplace_back(match_index);
    }

    // 重新排序配对信息, 防止倾斜路口配对错误
    std::vector<int> tmp_result = pair_result;
    std::sort(tmp_result.begin(), tmp_result.end());  // 对tmp_result进行排序
    auto iter = std::upper_bound(tmp_result.begin(), tmp_result.end(), -1); // 在tmp_result中查找第一个大于-1的位置
    int start_index = std::distance(tmp_result.begin(), iter);  // 计算iter的索引

    // 最终pair_result，除了-1的位置，其他位置按照顺序排列，但是为啥要怎么做？？？？
    for (int &pair_index : pair_result)
    {
      if (pair_index == -1)
      {
        continue;
      }
      pair_index = tmp_result[start_index];
      start_index++;
    }
    *boundary_pair_result = pair_result;
  }

  bool RoadModelProcBindRelation::is_intersect(const segment_t &l1, const segment_t &l2)
  {
    if ((l1.first.x() > l1.second.x() ? l1.first.x() : l1.second.x()) < (l2.first.x() < l2.second.x() ? l2.first.x() : l2.second.x()) ||
        (l1.first.y() > l1.second.y() ? l1.first.y() : l1.second.y()) < (l2.first.y() < l2.second.y() ? l2.first.y() : l2.second.y()) ||
        (l2.first.x() > l2.second.x() ? l2.first.x() : l2.second.x()) < (l1.first.x() < l1.second.x() ? l1.first.x() : l1.second.x()) ||
        (l2.first.y() > l2.second.y() ? l2.first.y() : l2.second.y()) < (l1.first.y() < l1.second.y() ? l1.first.y() : l1.second.y()))
    {
      return false;
    }

    if ((((l1.first.x() - l2.first.x()) * (l2.second.y() - l2.first.y()) - (l1.first.y() - l2.first.y()) * (l2.second.x() - l2.first.x())) *
         ((l1.second.x() - l2.first.x()) * (l2.second.y() - l2.first.y()) - (l1.second.y() - l2.first.y()) * (l2.second.x() - l2.first.x()))) > 0 ||
        (((l2.first.x() - l1.first.x()) * (l1.second.y() - l1.first.y()) - (l2.first.y() - l1.first.y()) * (l1.second.x() - l1.first.x())) *
         ((l2.second.x() - l1.first.x()) * (l1.second.y() - l1.first.y()) - (l2.second.y() - l1.first.y()) * (l1.second.x() - l1.first.x()))) > 0)
    {
      return false;
    }

    return true;
  }

  void RoadModelProcBindRelation::build_new_boundary(const RoadLaneGroupInfo *post_group,
                                                     const std::vector<RoadLaneBoundaryInfo *> &pre_boundary_v,
                                                     const std::vector<RoadLaneBoundaryInfo *> &post_boundary_v,
                                                     const std::vector<int> &pair_data,
                                                     RoadModelSessionData *session,
                                                     std::vector<RoadLaneBoundaryInfo *> *new_boundary)
  {
    CHECK_NOTNULL(post_group);
    CHECK_NOTNULL(session);
    CHECK_NOTNULL(new_boundary);

    CHECK_EQ(pre_boundary_v.size(), pair_data.size());

    for (int i = 0; i < pre_boundary_v.size(); ++i)
    {
      const auto &pre_boundary = pre_boundary_v[i];
      const auto &pre_pt = pre_boundary->line_point_info.back();
      const int match_index = pair_data[i];

      // 构建新的车道线
      auto boundary_shared_ptr = session->add_ptr(session->lane_boundary_info_ptr);
      boundary_shared_ptr->geo = 9;

      if (match_index == -1)
      {
        // 计算相交的位置, 计算的原则是，（前驱的最后一个点与倒数第二或者倒数第三个点的连线）与（后继的第一个点对应的左右车道线的坐标）计算相交点的坐标，取前驱的最后一个中心线坐标与交点，构建新的车道中心线/车道线
        const auto &left_pt = post_group->lane_line_info.front()->left_lane_boundary_info->line_point_info.front();
        const auto &right_pt = post_group->lane_line_info.back()->right_lane_boundary_info->line_point_info.front();
        const auto &pre_sec_pt = pre_boundary->line_point_info[pre_boundary->line_point_info.size() - 2];
        Eigen::Vector3d cross_pt;
        if (not alg::get_cross_point_by_point(right_pt->pos, left_pt->pos, pre_sec_pt->pos, pre_pt->pos, cross_pt, true, 3))
        {
          LOG_WARN( "No Cross.");
          // continue;
          // ONLY NOT FAULT it maybe  solve  later 
          cross_pt=pre_pt->pos;
        }
        boundary_shared_ptr->line_point_info.emplace_back(pre_pt);

        auto new_point = session->add_ptr(session->line_point_info_ptr);
        new_point->pos = cross_pt;
        boundary_shared_ptr->line_point_info.emplace_back(new_point.get());
        build_topology<RoadLaneBoundaryInfo>(boundary_shared_ptr.get(), pre_boundary, nullptr);
      }
      else
      {
        // 构建出新的车道中心线/车道线（根据配对的索引，取前驱的最后一个点以及后继的第一个点构建）
        const auto &post_pt = post_boundary_v[match_index]->line_point_info.front();
        boundary_shared_ptr->line_point_info.emplace_back(pre_pt);
        boundary_shared_ptr->line_point_info.emplace_back(post_pt);
        build_topology<RoadLaneBoundaryInfo>(boundary_shared_ptr.get(), pre_boundary, post_boundary_v[match_index]);
      }
      new_boundary->emplace_back(boundary_shared_ptr.get());
    }
  }

  void RoadModelProcBindRelation::build_new_group(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);
    // 2. 遍历 new_group_2_central_boundary_ 中的每一个边界数据（即每一对前后车道组的中央边界数据）
    for (const auto &pair_data : new_group_2_central_boundary_)
    {

      // 3. 获取前后车道组和中央边界、侧边界数据
      // 从 session 中根据 pair_data.first 获取前后车道组
      auto &pre_group = session->new_lane_groups[pair_data.first.first];
      auto &post_group = session->new_lane_groups[pair_data.first.second];

      // 获取新车道组的中央边界和侧边界
      auto &new_central_boundary = pair_data.second;
      auto &new_side_boundary = new_group_2_side_boundary_[pair_data.first];

      // 4. 检查新车道组的中央边界和侧边界的数量关系是否正确
      CHECK_EQ(new_central_boundary.size(), new_side_boundary.size() - 1);

      // 5. 获取当前车道组的车道配对信息
      auto &lane_pair = new_group_2_lane_pair_[pair_data.first];
      CHECK_EQ(new_central_boundary.size(), lane_pair.size());

      // 6. 创建新车道组
      // 使用智能指针创建一个新的车道组指针，并设置该车道组的状态为“新建”
      auto group_shared_ptr = session->add_ptr(session->lane_group_info_ptr);
      auto new_group_ptr = group_shared_ptr.get();
      new_group_ptr->joint_status = DataStatus::kNew;

      // 7. 为新车道组建立前后车道组的拓扑关系
      build_topology<RoadLaneGroupInfo>(new_group_ptr, pre_group, post_group);

      // 8. 为中央边界和侧边界关联车道组信息
      // 遍历中央边界数据，并将每个中央边界的 lane_group 指向当前新建的车道组
      for (auto &central_boundary : new_central_boundary)
      {
        central_boundary->lane_group = new_group_ptr;
      }
      // 遍历侧边界数据，并将每个侧边界的 lane_group 指向当前新建的车道组
      for (auto &side_boundary : new_side_boundary)
      {
        side_boundary->lane_group = new_group_ptr;
      }

      // 9. 为每个中央边界对应的车道创建新的车道对象，并关联相关数据
      for (int i = 0; i < new_central_boundary.size(); ++i)
      {
        const int match_lane_index = lane_pair[i];

        // 如果 match_lane_index == -1 且后续车道组出现断裂，则跳过当前车道
        if (match_lane_index == -1 and broken_group_found_from_post_.count(pair_data.first))
        {
          continue;
        }

        // 10. 为新车道创建车道对象
        auto lane_shared_ptr = session->add_ptr(session->road_lane_line_ptr);
        auto new_lane_ptr = lane_shared_ptr.get();

        // 11. 关联车道组信息
        new_lane_ptr->lane_group_info = new_group_ptr;

        // 12. 关联车道线的中央和侧边界
        new_lane_ptr->center_lane_boundary_info = new_central_boundary[i];
        new_lane_ptr->left_lane_boundary_info = new_side_boundary[i];
        new_lane_ptr->right_lane_boundary_info = new_side_boundary[i + 1];

        // 13. 关联前后车道信息
        auto &pre_lane = pre_group->lane_line_info[i];
        if (match_lane_index == -1)
        {
          // 如果没有匹配的后车道，构建与后车道的拓扑关系为 null
          build_topology<RoadLaneInfo>(new_lane_ptr, pre_lane, nullptr);
        }
        else
        {
          // 否则，构建与匹配到的后车道的拓扑关系
          auto &post_lane = post_group->lane_line_info[match_lane_index];
          build_topology<RoadLaneInfo>(new_lane_ptr, pre_lane, post_lane);
        }

        // 14. 将新车道加入新车道组中
        new_group_ptr->lane_line_info.emplace_back(new_lane_ptr);
      }

      if (new_group_ptr->lane_line_info.size() > 0) // add by qzc
      {
        new_group_ptr->road_center_line_info = new_group_ptr->lane_line_info[0]->left_lane_boundary_info;

        // 15. 将新创建的车道组添加到 session 的车道组列表中
        session->new_lane_groups.emplace_back(new_group_ptr);
      }
    }
  }

  // 绑定路口相关的及添加路口的属性
  int RoadModelProcBindRelation::attribute_for_lukoubd(RoadModelSessionData *session)
  {
    LOG_DEBUG("process attribute_for_lukoubd ...... ");

    if (session->new_junction_list.empty())
    {

      LOG_DEBUG("new_junction_list 为空");
      return fsdmap::SUCC;
    }

    // 遍历所有新路口
    for (auto &junction : session->new_junction_list)
    {

      // 如果路口的点信息小于3个点，则跳过该路口
      if (junction->point_info.size() < 3)
      {
        continue;
      }

      // 初始化路口的所有点类型为0 (unknown)
      for (auto &pt : junction->point_info)
      {
        pt->type = 0;
      }

      // 以下代码被注释掉，可能是用于基于bev、人行横道、停止线等特征来为路口点设置属性
      // 根据属性（如bev）对路口点类型进行赋值，搜索特定半径内的边界特征，进行类型统计和选取
      /*
      UMAP<int, int> map_type_num;
      double radius = FLAGS_attr_lukou_rb_search_radius;
      for (auto& pt : junction->point_info)
      {
          auto &pos = pt->pos;
          session->debug_pos(pos);
          std::vector<BoundaryFeature*> search_sec;
          session->boundary_line_sample_tree.search(pos, radius, search_sec);
          // 删除非bev结果
          for (int i = 0; i < search_sec.size(); i++){
              if(search_sec[i]->src_status != 1){
                  VEC_ERASE(search_sec, i);
              }
          }

          if(search_sec.empty()){
              continue;
          }

    //   std::vector<BoundaryFeature*> attr_data_list;
    //   for (auto &fls : search_sec) {
    //       if(MAP_NOT_FIND(map_type_num,fls->sub_type)){
    //       map_type_num[fls->sub_type] = 0;
    //     }
    //     else{
    //       map_type_num[fls->sub_type]++;
    //    }
    //   }

          std::vector<std::pair<int, int>> temp_type(map_type_num.begin(), map_type_num.end());
          std::sort(temp_type.begin(), temp_type.end(), [](auto& a, auto& b) {return a.second >= b.second;});
          pt->type = temp_type[0].first; // 选取命中最多的作为最终类型
      }
      */

      // 根据路口点的类型，将路口边界划分为多个段
      int ori_type = junction->point_info[0]->type; // 初始类型
      std::vector<int> tmp_index_points;            // 临时存储索引
      std::vector<std::vector<int>> line_vec;       // 存储分段后的点索引

      // 遍历路口的点信息，将相同类型的点分为一组
      for (int i = 0; i < junction->point_info.size(); i++)
      {
        tmp_index_points.push_back(i); // 将当前点的索引加入临时列表
        // 如果当前点类型与前一个点的类型不同，说明新的一段开始
        if (junction->point_info[i]->type != ori_type)
        {
          if (tmp_index_points.size() > 1)
          {
            line_vec.push_back(tmp_index_points); // 将分段结果保存
          }
          tmp_index_points.clear();                 // 清空临时列表，开始新的段
          tmp_index_points.push_back(i);            // 新的一段开始，添加当前点索引
          ori_type = junction->point_info[i]->type; // 更新段的类型
        }
      }

      // 如果最后一段的点数大于1，将它加入分段列表
      if (tmp_index_points.size() > 1)
      {
        line_vec.push_back(tmp_index_points);
        tmp_index_points.clear();
      }

      // 根据分段信息重新组织路口边界
      for (int i = 0; i < line_vec.size(); i++)
      {
        auto road_segment = std::make_shared<RoadBoundarySegmentInfo>(); // 创建一个新的路段信息对象
        // 将该段的所有点加入路段
        for (int j = 0; j < line_vec[i].size(); j++)
        {
          int index = line_vec[i][j];
          auto feature = session->add_ptr(session->boundary_point_info_ptr); // 获取边界点信息
          feature->pos = junction->point_info[index]->pos;                   // 设置该点的位置信息
          road_segment->point_info.push_back(feature.get());                 // 将该点加入路段信息
        }
        road_segment->subtype = junction->point_info[line_vec[i][0]]->type; // 设置该路段的类型
        junction->lukou_bd_list.push_back(road_segment);                    // 将该路段加入路口的边界列表
      }

      // 输出路口边界段的数量
      LOG_INFO("junction lukou_bd_list size:{}", junction->lukou_bd_list.size()); // 目前只有1，因为每个点类型都一样
    }
    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::delete_rbsegment_contined_lukoubd(RoadModelSessionData *session)
  {
    LOG_DEBUG("process delete_rbsegment_contined_lukoubd ...... ");

    // 如果新路口列表或新边界段列表为空，直接返回成功
    if (session->new_junction_list.empty() || session->new_boundary_segment.empty())
    {
      LOG_DEBUG("new_junction_list 为空 or new_boundary_segment");
      return fsdmap::SUCC;
    }

    // 1. 计算每段边界线的中点，并删除点数少于2个的边界段
    for (int i = 0; i < session->new_boundary_segment.size(); i++)
    {
      int n = session->new_boundary_segment[i]->point_info.size();

      // 如果边界段的点数少于2个，则删除该边界段
      if (n < 2)
      {
        VEC_ERASE(session->new_boundary_segment, i) // 删除当前边界段
        continue;
      }

      // 计算当前边界段的中点（所有点的位置的平均值）
      Eigen::Vector3d pt_sum{0, 0, 0}; // 初始化位置和
      for (auto &pt : session->new_boundary_segment[i]->point_info)
      {
        pt_sum += pt->get_pos(); // 累加每个点的位置
        pt->line_index = i;      // 给每个点设置所在的边界段索引
      }

      // 更新边界段的中点位置
      session->new_boundary_segment[i]->pos = pt_sum / n;
    }

    // 建立道路边界的数据的搜索树
    RTreeProxy<RoadBoundarySegmentInfo *, float, 2> rbsegment_tree;
    for (const auto &rb : session->new_boundary_segment)
    {
      rbsegment_tree.insert(rb->pos, rb);
    }

    std::vector<int> deleteIndexs;

    // 3. 对每个路口，使用路口的中心点搜索附近的道路边界段
    for (auto &junction : session->new_junction_list)
    {
      std::vector<RoadBoundarySegmentInfo *> suces; // 存储查询到的边界段

      // 使用搜索树查找路口附近的道路边界段
      rbsegment_tree.search(junction->pos, FLAGS_lukou_pos_search_radius, suces);

      // 如果没有找到任何道路边界段，则跳过该路口
      if (suces.empty())
      {
        continue;
      }

      // 4. 对每个搜索到的边界段，判断该段是否在路口的边界内
      for (auto &rb : suces)
      {
        // 使用点在多边形内的算法判断该边界段是否被路口边界包含
        if (alg::point_in_polygon(rb->pos, junction->lukou_poly_pts))
        {
          int line_index = rb->point_info[0]->line_index; // 获取该边界段的索引

          // 如果该边界段的索引不在待删除的索引列表中，标记为删除
          if (!VEC_FIND(deleteIndexs, line_index))
          {
            DLOG_POINT(rb->pos, "将被删除的道路边界line_index:{}", line_index);
            deleteIndexs.push_back(line_index);
          }
        }
      }
    }

    // 5. 如果没有边界段需要删除，直接返回成功
    if (deleteIndexs.empty())
    {
      return fsdmap::SUCC;
    }

    sort(deleteIndexs.begin(), deleteIndexs.end()); // 将数组按照从大到小排序
    int n_rb = session->new_boundary_segment.size();
    for (int i = deleteIndexs.size() - 1; i >= 0; i--)
    {
      session->new_boundary_segment.erase(session->new_boundary_segment.begin() + deleteIndexs[i]);
    }

    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::bind_areas_to_junction(RoadModelSessionData *session)
  {
    LOG_DEBUG("process bind_areas_to_junction ...... ")
    if (session->new_junction_list.empty() || session->new_area_list.empty())
    {
      LOG_DEBUG("new_junction_list 为空 or new_area_list")
      return fsdmap::SUCC;
    }

    // 建立不可通行区域的搜索树
    RTreeProxy<ImpassableAreaInfo *, float, 2> area_tree;
    for (auto &area : session->new_area_list)
    {
      area_tree.insert(area->pos, area.get());
    }

    // 用路口中心点搜索被路口边界包含的不可通行区域
    for (auto &junction : session->new_junction_list)
    {
      std::vector<ImpassableAreaInfo *> suces;
      area_tree.search(junction->pos, FLAGS_lukou_pos_search_radius, suces);
      if (suces.empty())
      {
        continue;
      }
      for (auto &area : suces)
      {
        if (alg::point_in_polygon(area->pos, junction->lukou_poly_pts))
        {
          area->junction_info = junction.get();
          junction->areas.push_back(area);
        }
      }
    }
    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::joint_invalid_topology(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    // 给每个小组分配索引
    assign_group_index(session, true);

    // 计算各个小组的拓扑结构
    cal_group_topology(); // 车道组的配对，（流程与build_broken_group_pair类似，简化版）

    // 查找无效的拓扑
    pick_invalid_topology(session);

    // 如果有无效的前后组关系，合并和删除无效的拓扑
    if(!invalid_pre_group_2_post_group_.empty())
    {
      // 合并无效的拓扑
      merge_invalid_topology(session);

      // 删除合并后的无效小组
      delete_merged_group(session);
    }

    assign_group_index(session, false);
    // 返回成功标志
    return fsdmap::SUCC;
  }

  // 实现步骤与build_broken_group_pair差不多，只不过只需要从前向后找，用后继的首点做树，
  void RoadModelProcBindRelation::cal_group_topology()
  {

    // 清空之前存储的前组到后组的关系映射
    pre_group_2_post_set_.clear();

    // 清空之前存储的后组到前组的关系映射
    post_group_2_pre_set_.clear();

    // 清空 lane_pt_tree_，它用于存储与每条车道关联的点数据
    lane_pt_tree_.RemoveAll();

    // 遍历后车道组列表 post_group_v_，构建与车道前端点相关的空间索引
    for (const auto &lane_group : post_group_v_)
    {
      for (const auto &lane : lane_group->lane_line_info)
      {
        // 获取每个车道的起始边界点 (front point)
        const auto &front_pt = lane->center_lane_boundary_info->line_point_info.front();
        // 将边界点插入空间索引树（根据位置存储车道的前端点）
        lane_pt_tree_.insert(front_pt->pos, front_pt);
      }
    }
    // 遍历前车道组列表 pre_group_v_
    for (const auto &pre_group : pre_group_v_)
    {
      for (const auto &lane : pre_group->lane_line_info)
      {

        // 如果车道的边界点数量小于等于1，则跳过该车道
        if (lane->center_lane_boundary_info->line_point_info.size() <= 1)
        {
          break;
        }

        // 获取车道的末尾边界点 (pre point)
        const auto &pre_pt = lane->center_lane_boundary_info->line_point_info.back();

        // 用来存储与当前前点（pre_pt）附近的后点集合
        std::vector<RoadLinePointInfo *> post_pts;

        // 在 lane_pt_tree_ 中查找前点周围的后点，搜索半径由 FLAGS_group_topology_search_radius_m 控制
        lane_pt_tree_.search(pre_pt->pos, static_cast<float>(FLAGS_group_topology_search_radius_m), post_pts);

        // 遍历所有找到的后点
        for (const auto &post_pt : post_pts)
        {
          // 获取后车道组的索引
          const auto &post_group_index = post_pt->lane_group->group_index;

          // 如果前后车道组的索引相同，跳过
          if (post_group_index == pre_group->group_index)
          {
            continue;
          }

          // 计算前点和后点的距离
          const double pt_distance = alg::calc_dis(post_pt->pos, pre_pt->pos);

          // 如果前后点的距离小于阈值（例如 0.1 米），则认为它们是连接的
          if (pt_distance < 1e-1)
          {
            pre_group_2_post_set_[pre_group->group_index].insert(post_group_index);
            post_group_2_pre_set_[post_group_index].insert(pre_group->group_index);
          }
        }
      }
    }
  }

  void RoadModelProcBindRelation::pick_invalid_topology(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    // 遍历所有的前车道组到后车道组的拓扑关系
    for (const auto &pre_to_post_topology : pre_group_2_post_set_)
    {
      const auto &post_group_set = pre_to_post_topology.second;

      // 如果后车道组数量不为1或为空，跳过当前的拓扑关系
      if (post_group_set.size() > 1 or post_group_set.empty())
      {
        continue;
      }

      // 获取唯一的后车道组索引，并检查该后车道组的拓扑关系数量是否为1
      const int post_group_index = *(post_group_set.begin());
      const auto &post_topology_size = post_group_2_pre_set_[post_group_index].size();
      CHECK_NE(post_topology_size, 0); // 确保后车道组的拓扑关系不为0
      if (post_topology_size > 1)
      {
        continue; // 如果后车道组的拓扑关系数量大于1，跳过
      }

      // 获取前车道组索引，并检查前后车道组的车道数量是否一致
      const int pre_group_index = pre_to_post_topology.first;
      if (session->new_lane_groups[pre_group_index]->lane_line_info.size() !=
          session->new_lane_groups[post_group_index]->lane_line_info.size())
      {
        continue; // 如果车道数量不一致，跳过
      }

      const int lane_size = session->new_lane_groups[pre_group_index]->lane_line_info.size();
      const auto &pre_group = session->new_lane_groups[pre_group_index];
      const auto &post_group = session->new_lane_groups[post_group_index];

      // 标记当前前后车道组是否是有效的候选关系
      bool is_candidate = true;
      for (int i = 0; i < lane_size; ++i)
      {
        const auto &pre_lane = pre_group->lane_line_info[i];
        const auto &post_lane = post_group->lane_line_info[i];

        // 检查中心车道、左车道和右车道的连接关系
        if (not is_connected(pre_lane->center_lane_boundary_info->line_point_info.back(),
                             post_lane->center_lane_boundary_info->line_point_info.front()))
        {
          is_candidate = false;
          break; // 如果任何车道不连接，标记为无效并跳出循环
        }
        else if (not is_connected(pre_lane->left_lane_boundary_info->line_point_info.back(),
                                  post_lane->left_lane_boundary_info->line_point_info.front()))
        {
          is_candidate = false;
          break;
        }
        else if (not is_connected(pre_lane->right_lane_boundary_info->line_point_info.back(),
                                  post_lane->right_lane_boundary_info->line_point_info.front()))
        {
          is_candidate = false;
          break;
        }

        // 检查车道是否在路口区域内
        for (const auto &junction : session->new_junction_list)
        {
          if (alg::point_in_polygon(pre_lane->center_lane_boundary_info->line_point_info.back()->pos,
                                    junction->lukou_poly_pts) or
              alg::point_in_polygon(pre_lane->center_lane_boundary_info->line_point_info.front()->pos,
                                    junction->lukou_poly_pts))
          {

            is_candidate = false; // 如果车道在路口内，标记为无效
            LOG(INFO) << "路口内无效打断 pre index = " << pre_group_index << " post index = " << post_group_index
                      << " 不做合并.";
            break;
          }
        }
      }

      // 如果是有效候选关系，更新无效拓扑关系的映射
      if (not is_candidate)
      {
        continue; // 如果不是有效候选关系，跳过
      }

      invalid_pre_group_2_post_group_[pre_group_index] = post_group_index;
      invalid_post_group_2_pre_group_[post_group_index] = pre_group_index;
      LOG(INFO) << "无效打断 pre index = " << pre_group_index << " post index = " << post_group_index
                << " 需要合并.";
    }
  }

  void RoadModelProcBindRelation::merge_invalid_topology(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    // 计算拓扑链
    std::set<int> used_group_index_set;         // 用于存储已经处理过的车道组索引
    std::vector<std::list<int>> group_chains_v; // 存储拓扑链
    for (const auto &pair_info : invalid_pre_group_2_post_group_)
    {
      std::list<int> group_chain = {}; // 创建一个新的拓扑链

      int pre_index = pair_info.first;   // 获取前车道组的索引
      int post_index = pair_info.second; // 获取后车道组的索引

      if (used_group_index_set.count(pre_index))
      { // 如果前车道组已经处理过，跳过
        continue;
      }

      group_chain.push_back(pre_index);  // 将前车道组添加到链中
      group_chain.push_back(post_index); // 将后车道组添加到链中

      // 向后延伸拓扑链
      while (invalid_pre_group_2_post_group_.count(post_index))
      {
        post_index = invalid_pre_group_2_post_group_[post_index]; // 获取后车道组的下一个车道组
        group_chain.push_back(post_index);                        // 添加到链中
      }

      // 向前延伸拓扑链
      while (invalid_post_group_2_pre_group_.count(pre_index))
      {
        pre_index = invalid_post_group_2_pre_group_[pre_index]; // 获取前车道组的上一个车道组
        group_chain.push_front(pre_index);                      // 添加到链前端
      }

      // 将拓扑链中的车道组索引标记为已处理
      for (const auto &index : group_chain)
      {
        used_group_index_set.insert(index);
      }

      CHECK_GE(group_chain.size(), 2);          // 确保拓扑链长度大于等于2
      group_chains_v.emplace_back(group_chain); // 将拓扑链加入到拓扑链列表中
    }

    // 遍历所有拓扑链，合并无效的车道组拓扑
    for (auto &group_chain : group_chains_v)
    {
      int post_index = group_chain.back(); // 获取链尾的后车道组
      group_chain.pop_back();              // 移除链尾

      // 持续合并车道组，直到链为空
      while (not group_chain.empty())
      {
        int pre_index = group_chain.back(); // 获取链尾的前车道组
        group_chain.pop_back();             // 移除链尾

        auto &pre_group = session->new_lane_groups[pre_index];   // 获取前车道组
        auto &post_group = session->new_lane_groups[post_index]; // 获取后车道组

        // 1. 修复车道组的拓扑关系
        if(post_group->context.all_next.size() == 0)
        {
          RoadLaneGroupInfo* group_ptr = nullptr;
          merge_topology<RoadLaneGroupInfo>(pre_group, post_group, group_ptr);
        }else{
          for (auto &next_group : post_group->context.all_next)
          {
            merge_topology<RoadLaneGroupInfo>(pre_group, post_group, next_group.src);
          }
        }
        

        // 2. 修复车道组的车道线
        for (int i = 0; i < pre_group->lane_line_info.size(); ++i)
        {
          auto &pre_lane = pre_group->lane_line_info[i];
          auto &post_lane = post_group->lane_line_info[i];

          // 2.1 修复车道的拓扑
          if(post_lane->context.all_next.size() == 0)
          {
            RoadLaneInfo* lane_ptr = nullptr;
            merge_topology<RoadLaneInfo>(pre_lane, post_lane, lane_ptr);
          }
          else{
            for (auto &next_lane : post_lane->context.all_next)
            {
              merge_topology<RoadLaneInfo>(pre_lane, post_lane, next_lane.src);
            }
          }

          // 2.2 修复车道的车道线
          merge_boundary(pre_group, pre_lane->center_lane_boundary_info, post_lane->center_lane_boundary_info);
          merge_boundary(pre_group, pre_lane->left_lane_boundary_info, post_lane->left_lane_boundary_info);
          merge_boundary(pre_group, pre_lane->right_lane_boundary_info, post_lane->right_lane_boundary_info);
        }
        post_index = pre_index;
      }
    }
  }

  void RoadModelProcBindRelation::merge_boundary(RoadLaneGroupInfo *pre_group,
                                                 RoadLaneBoundaryInfo *pre_boundary,
                                                 RoadLaneBoundaryInfo *post_boundary)
  {
    // 1. 遍历后续边界的所有“下一边界”，并将前边界与后边界的拓扑关系合并

    if(post_boundary->context.all_next.size() == 0)
    {
      RoadLaneBoundaryInfo* lane_boundary_ptr = nullptr;
      merge_topology<RoadLaneBoundaryInfo>(pre_boundary, post_boundary, lane_boundary_ptr);
    }
    else{
      for (auto &next_boundary : post_boundary->context.all_next)
      {
        merge_topology<RoadLaneBoundaryInfo>(pre_boundary, post_boundary, next_boundary.src); // 合并拓扑关系
      }
    }
    

    // 2. 计算前边界和后边界的起始位置之间的距离
    double start_distance = alg::calc_dis(pre_boundary->line_point_info.back()->pos,
                                          post_boundary->line_point_info.front()->pos);
    // 如果距离大于 1e-1，则认为两个边界不应该合并，直接返回
    if (start_distance > 1e-1)
    {
      return;
    }

    // 3. 向前边界中添加后边界的点（从第二个点开始添加）
    for (int i = 1; i < post_boundary->line_point_info.size(); ++i)
    {
      // 将后边界的每个点添加到前边界的点集合中
      pre_boundary->line_point_info.emplace_back(post_boundary->line_point_info[i]);
      // 更新新加入的点的车道边界信息为前边界
      pre_boundary->line_point_info.back()->lane_boundary = pre_boundary;
      // 更新新加入的点的车道组信息为前车道组
      pre_boundary->line_point_info.back()->lane_group = pre_group;
    }

    // 4. 处理车道类型
    // 如果前边界的车道类型不是 "9"，则不需要处理，直接返回
    if (pre_boundary->geo != 9)
    {
      return;
    }
    else if (pre_boundary->geo == 9 && post_boundary->geo != 9)
    {
      // 如果前边界的车道类型是 "9" 而后边界的车道类型不是 "9"，则将前边界的车道类型修改为后边界的车道类型
      pre_boundary->geo = post_boundary->geo;
    }
  }

  void RoadModelProcBindRelation::delete_merged_group(RoadModelSessionData *session)
  {
    // 1. 初始化两个向量：删除组的索引 (delete_group_index_v) 和合并组的索引 (merge_group_index_v)
    std::vector<int> delete_group_index_v = {};
    std::vector<int> merge_group_index_v = {};

    // 2. 遍历 'invalid_pre_group_2_post_group_'，构建删除组和合并组的索引
    for (const auto &pair_data : invalid_pre_group_2_post_group_)
    {
      // 'pair_data.first' 是要合并的组索引，加入到 merge_group_index_v 中
      merge_group_index_v.emplace_back(pair_data.first);
      // 'pair_data.second' 是要删除的组索引，加入到 delete_group_index_v 中
      delete_group_index_v.emplace_back(pair_data.second);
    }

    // 3. 标记合并组为修改状态
    for (const auto &merge_index : merge_group_index_v)
    {
      // 将合并组的 'joint_status' 设置为 'kMod'（修改状态）
      session->new_lane_groups[merge_index]->joint_status = DataStatus::kMod;
    }

    // 4. 对删除组的索引进行降序排序
    std::sort(delete_group_index_v.begin(), delete_group_index_v.end(), std::greater<>());

    // 5. 删除合并后的车道组
    for (const auto &delete_index : delete_group_index_v)
    {
      // 如果删除的是最后一个组，直接弹出
      if (session->new_lane_groups.back()->group_index == delete_index)
      {
        delete_prev_lane_groups(session->new_lane_groups.back());
        session->new_lane_groups.pop_back();
        continue; // 继续处理下一个删除组
      }

      // 如果删除组不是最后一个组，将其替换为最后一个组，并弹出最后一个组
      delete_prev_lane_groups(session->new_lane_groups[delete_index]);
      delete_next_lane_groups(session->new_lane_groups[delete_index]);
      session->new_lane_groups[delete_index] = session->new_lane_groups.back();
      session->new_lane_groups.pop_back();
    }
  }

  bool RoadModelProcBindRelation::delete_prev_lane_groups(RoadLaneGroupInfo* to_be_deleted_group)
  {
    CHECK_NOTNULL(to_be_deleted_group);
    for (size_t i = 0; i < to_be_deleted_group->context.all_prev.size(); i++) 
    {
      auto &prev_group = to_be_deleted_group->context.all_prev[i].src;
      for(size_t j = 0; j < prev_group->context.all_next.size(); j++)
      {
        auto &prev_to_next = prev_group->context.all_next[j].src;
        if(prev_to_next->group_index == to_be_deleted_group->group_index)
        {
          VEC_ERASE(prev_group->context.all_next, j);
          break;
        }
      }
    }

    return true;
  }


  bool RoadModelProcBindRelation::delete_next_lane_groups(RoadLaneGroupInfo* to_be_deleted_group)
  {
    CHECK_NOTNULL(to_be_deleted_group);
    for (size_t i = 0; i < to_be_deleted_group->context.all_next.size(); i++) 
    {
      auto &next_group = to_be_deleted_group->context.all_next[i].src;
      for(size_t j = 0; j < next_group->context.all_prev.size(); j++)
      {
        auto &next_to_prev = next_group->context.all_prev[j].src;
        if(next_to_prev->group_index == to_be_deleted_group->group_index)
        {
          VEC_ERASE(next_group->context.all_prev, j);
          break;
        }
      }
    }

    return true;
  }


  int RoadModelProcBindRelation::save_joint_debug_info(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);
    if (not FLAGS_joint_group_debug_enable)
    {
      return fsdmap::SUCC;
    }
    session->set_display_name("joint_group");

    // 遍历所有的新车道组
    for (int i = 0; i < session->new_lane_groups.size(); ++i)
    {
      const auto &lane_group = session->new_lane_groups[i];
      // LOG_INFO("lane_group id is : {}", lane_group->group_index);

      // 添加车道组的索引日志，并设置为红色
      auto group_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "new lane group{}", i);
      group_index_log->color = {255, 0, 0}; // 红色

      // 遍历车道组中的车道线信息
      for (const auto &lane_info : lane_group->lane_line_info)
      {

        // 添加左右车道线的信息
        for (int lane_side = 0; lane_side < 2; ++lane_side)
        {
          const auto &lane_data =
              lane_side == 0 ? lane_info->left_lane_boundary_info : lane_info->right_lane_boundary_info;

          auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
          log->color = {255, 255, static_cast<double>(i)}; // 黄色

          // 对于新增或修改的车道组，左右车道边界设置为白色
          if (lane_group->joint_status == DataStatus::kNew || lane_group->joint_status == DataStatus::kMod)
          {
            log->color = {255, 255, 255};
          }

          for (int k = 0; k < lane_data->line_point_info.size(); ++k)
          {
            const auto &lane_point = lane_data->line_point_info[k];
            if (k == 0)
            {
              group_index_log->add(lane_point->pos); // 对第一个点添加到车道组索引日志
            }
            log->add(lane_point->pos);
          }
        }

        // 处理车道中心线信息
        const auto &lane_central_data = lane_info->center_lane_boundary_info;
        if (lane_central_data == nullptr)
        {
          continue;
        }
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {0, 255, static_cast<double>(i)}; // 绿色

        // 根据车道组的状态设置不同颜色
        if (lane_group->joint_status == DataStatus::kNew)
        {
          log->color = {223, static_cast<double>(i), 154}; // 特定颜色用于新增车道组
        }
        else if (lane_group->joint_status == DataStatus::kMod)
        {
          log->color = {255, 165, static_cast<double>(i)}; //紫色
        }

        for (const auto &lane_point : lane_central_data->line_point_info)
        {
          log->add(lane_point->pos);
        }
      }
    }

    // 处理道路边界信息
    for (auto &road_boundary_segment : session->new_boundary_segment)
    {
      auto log = session->add_debug_log(utils::DisplayInfo::LINE, "road_boundary_segments");
      log->color = {160, 32, 240}; // 紫色
      for (const auto &point : road_boundary_segment->point_info)
      {
        log->add(point->pos);
      }
    }

    // 处理路口边界和不可通行区域信息
    // std::cout << COLOR_GREEN << "session->new_junction_list:" << session->new_junction_list.size() << COLOR_RESET << "\n"
    //           << std::endl; //

    for (auto &junction : session->new_junction_list)
    {

      int index = 0;
      // 处理路口边界信息
      for (auto &rb : junction->lukou_bd_list)
      {
        if (index >= 1)
        {
          continue;
        }
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "new_junction_list");
        log->color = {255, static_cast<double>(10 * rb->subtype), 255}; // 粉红色
        // log->color = {255, 255, 255};
        for (auto &pt : rb->point_info)
        {
          log->add(pt->pos);
        }
      }

      // 处理不可通行区域信息
      for (const auto &area : junction->areas)
      {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "impassable_list");
        log->color = {255, 0, 0}; // 红色
        for (auto &pt : area->list)
        {
          log->add(pt->pos);
        }
      }
      // // 画原始路口信息, 可以正常显示，  add
      // for (const auto& pt : junction->point_info) {
      //   auto log = session->add_debug_log(utils::DisplayInfo::POLYGEN, "路口区域原始点");
      //   log->color = {255, 105, 180}; // 粉红色
      //   log->add(pt->pos);
      //   // std::cout << COLOR_GREEN << "relation:pt->pos:" << pt->pos << COLOR_RESET << "\n" << std::endl;//
      // }
    }

    for (auto &rs : session->road_segment_list)
    {
      auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
      log->color = {223, 0, 154};
      for (auto &poss : rs->pos_sample_list)
      {
          log->add(poss->pos);
      }
    }

    // 保存所有调试信息
    session->save_debug_info("joint_group");

    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::save_arrow_binding_debug_info(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    if (not FLAGS_arrow_binding_debug_enable)
    {
      return fsdmap::SUCC;
    }

    session->set_display_name("arrow_binding");

    // 遍历所有的新车道组
    for (int i = 0; i < session->new_lane_groups.size(); ++i)
    {
      const auto &lane_group = session->new_lane_groups[i];

      // 添加车道组的索引日志，并设置为红色
      auto group_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "stop_line_binding{}", i);
      group_index_log->color = {255, 0, 0}; // 红色

      // 遍历车道组中的车道线信息
      for (const auto &lane_info : lane_group->lane_line_info)
      {

        // 添加左右车道线的信息
        for (int lane_side = 0; lane_side < 2; ++lane_side)
        {
          const auto &lane_data =
              lane_side == 0 ? lane_info->left_lane_boundary_info : lane_info->right_lane_boundary_info;
          auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
          log->color = {255, 255, static_cast<double>(i)}; // 黄色

          // 对于新增或修改的车道组，设置为白色
          if (lane_group->joint_status == DataStatus::kNew || lane_group->joint_status == DataStatus::kMod)
          {
            log->color = {255, 255, 255}; // 白色
          }

          for (int k = 0; k < lane_data->line_point_info.size(); ++k)
          {
            const auto &lane_point = lane_data->line_point_info[k];
            if (k == 0)
            {
              group_index_log->add(lane_point->pos); // 对第一个点添加到车道组索引日志
            }
            log->add(lane_point->pos); // 添加车道点信息
          }
        }

        // 处理车道中心线信息
        const auto &lane_central_data = lane_info->center_lane_boundary_info;
        if (lane_central_data == nullptr)
        {
          continue;
        }
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {0, 255, static_cast<double>(i)}; // 绿色

        // // 根据车道组的状态设置不同颜色
        // if (lane_group->joint_status == DataStatus::kNew) {
        //   log->color = {223, static_cast<double>(i), 154}; // 特定颜色用于新增车道组
        // } else if (lane_group->joint_status == DataStatus::kMod) {
        //   log->color = {255, 165, static_cast<double>(i)}; // 修改状态的车道组颜色
        // }
        // int index = 0;
        // for (const auto& lane_point : lane_central_data->line_point_info) {
        //   //  std::cout << COLOR_GREEN << "index" << index << lane_point->pos << COLOR_RESET << "\n" << std::endl;
        //   log->add(lane_point->pos);
        //   index ++;
        // }

        // 中心线里有多个点
        //  std::cout << COLOR_GREEN << "size: " << lane_central_data->line_point_info.size() << COLOR_RESET << "\n" << std::endl;


        //处理箭头
        if (lane_info->bind_arrow_list.empty()) {
          continue; 
        }

        for (const auto &arrow : lane_info->bind_arrow_list)
        {
          // std::cout << COLOR_GREEN << "arrow :" << arrow->obj_id << COLOR_RESET << "\n" << std::endl;
          auto log = session->add_debug_log(utils::DisplayInfo::LINE, "arrow");
          log->color = {255, 0, 0}; // 红色
          for (auto &pt : arrow->list){
            log->add(pt->pos);
          }
          auto arrow_log1 = session->add_debug_log(utils::DisplayInfo::LINE, "arrow_relation1");
          arrow_log1->color = {255, 105, 180}; // 粉红色
          arrow_log1->add(arrow->pos);

          int middle_index = (lane_central_data->line_point_info.size()) / 2;
          auto pos1 = lane_central_data->line_point_info[middle_index]->pos;
          arrow_log1->add(pos1); // 中心线的中心点
        }
      }
    }
    session->save_debug_info("arrow_binding");
    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::save_stop_line_binding_debug_info(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    if (not FLAGS_stop_line_binding_debug_enable)
    {
      return fsdmap::SUCC;
    }

    session->set_display_name("stop_line_binding");

    // 遍历所有的新车道组
    for (int i = 0; i < session->new_lane_groups.size(); ++i)
    {
      const auto &lane_group = session->new_lane_groups[i];

      // // 添加车道组的索引日志，并设置为红色
      auto group_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "stop_line_binding{}", i);
      group_index_log->color = {255, 0, 0}; // 红色

      // 遍历车道组中的车道线信息
      for (const auto &lane_info : lane_group->lane_line_info)
      {

        // 添加左右车道线的信息
        for (int lane_side = 0; lane_side < 2; ++lane_side)
        {
          const auto &lane_data =
              lane_side == 0 ? lane_info->left_lane_boundary_info : lane_info->right_lane_boundary_info;
          auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
          log->color = {255, 255, static_cast<double>(i)};

          // 对于新增或修改的车道组，设置为白色
          if (lane_group->joint_status == DataStatus::kNew || lane_group->joint_status == DataStatus::kMod)
          {
            log->color = {255, 255, 255}; // 白色
          }

          // 遍历车道线的每个点并将其添加到调试日志中
          for (int k = 0; k < lane_data->line_point_info.size(); ++k)
          {
            const auto &lane_point = lane_data->line_point_info[k];
            if (k == 0)
            {
              group_index_log->add(lane_point->pos); // 对第一个点添加到车道组索引日志
            }
            log->add(lane_point->pos); // 添加车道点信息
          }
        }

        // 处理车道中心线信息
        const auto &lane_central_data = lane_info->center_lane_boundary_info; // 中心线
        if (lane_central_data == nullptr)
        {
          continue; // 如果没有中心线数据，则跳过
        }
        // 创建车道中心线调试日志对象
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {0, 255, static_cast<double>(i)}; // 默认颜色
        // // 根据车道组的状态设置不同颜色
        // if (lane_group->joint_status == DataStatus::kNew) {
        //   log->color = {223, static_cast<double>(i), 154}; // 特定颜色用于新增车道组
        // } else if (lane_group->joint_status == DataStatus::kMod) {
        //   log->color = {255, 165, static_cast<double>(i)}; // 修改状态的车道组颜色
        // }

        int index = 0;
        // 遍历车道中心线的每个点并将其添加到调试日志中
        for (const auto &lane_point : lane_central_data->line_point_info)
        {
          log->add(lane_point->pos);
          index++;
        }
        // 中心线里有多个点
        //  std::cout << COLOR_GREEN << "size: " << lane_central_data->line_point_info.size() << COLOR_RESET << "\n" << std::endl;

        // 处理车道中的---------停止线-----------
        if (lane_info->bind_stop_line_list.empty())
        {
          continue;
        }
        // 画出停止线
        for (const auto &stop_line : lane_info->bind_stop_line_list)
        {
          auto log = session->add_debug_log(utils::DisplayInfo::LINE, "stop_line");
          log->color = {255, 0, 255}; // 紫色
          for (auto &pt : stop_line->list){
            log->add(pt->pos);
          }
          // 添加箭头表示停止线和车道连接关系
          auto arrow_log1 = session->add_debug_log(utils::DisplayInfo::LINE, "stop_line_relation1");
          arrow_log1->color = {255, 105, 180}; // 粉红色
          arrow_log1->add(stop_line->pos);
          int point_num = lane_central_data->line_point_info.size();
          auto pos1 = lane_central_data->line_point_info[point_num - 2]->pos;
          arrow_log1->add(pos1); // 中心线的中心点
        }
      }
    }

    session->save_debug_info("stop_line_binding");
    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::save_cross_walk_binding_debug_info(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    if (not FLAGS_cross_walk_binding_debug_enable)
    {
      return fsdmap::SUCC;
    }
    session->set_display_name("cross_walk_binding");

    // 遍历所有的新车道组
    for (int i = 0; i < session->new_lane_groups.size(); ++i)
    {
      const auto &lane_group = session->new_lane_groups[i];

      // // 添加车道组的索引日志，并设置为红色
      // auto group_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "stop_line_binding{}", i);
      // group_index_log->color = {255, 0, 0}; // 红色

      // 遍历车道组中的车道线信息
      for (const auto &lane_info : lane_group->lane_line_info)
      {

        // 添加左右车道线的信息
        for (int lane_side = 0; lane_side < 2; ++lane_side)
        {
          const auto &lane_data =
              lane_side == 0 ? lane_info->left_lane_boundary_info : lane_info->right_lane_boundary_info;

          auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
          log->color = {255, 255, static_cast<double>(i)};

          // 对于新增或修改的车道组，设置为白色
          if (lane_group->joint_status == DataStatus::kNew || lane_group->joint_status == DataStatus::kMod)
          {
            log->color = {255, 255, 255}; // 白色
          }

          // 遍历车道线的每个点并将其添加到调试日志中
          for (int k = 0; k < lane_data->line_point_info.size(); ++k)
          {
            const auto &lane_point = lane_data->line_point_info[k];
            // if (k == 0) {
            //   group_index_log->add(lane_point->pos); // 对第一个点添加到车道组索引日志
            // }
            log->add(lane_point->pos); // 添加车道点信息
          }
        }

        // 处理车道中心线信息
        const auto &lane_central_data = lane_info->center_lane_boundary_info; // 中心线
        if (lane_central_data == nullptr)
        {
          continue;
        }

        // 创建车道中心线调试日志对象
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {0, 255, static_cast<double>(i)}; // 默认颜色

        // // 根据车道组的状态设置不同颜色
        // if (lane_group->joint_status == DataStatus::kNew) {
        //   log->color = {223, static_cast<double>(i), 154}; // 特定颜色用于新增车道组
        // } else if (lane_group->joint_status == DataStatus::kMod) {
        //   log->color = {255, 165, static_cast<double>(i)}; // 修改状态的车道组颜色
        // }
        int index = 0;
        // 遍历车道中心线的每个点并将其添加到调试日志中
        for (const auto &lane_point : lane_central_data->line_point_info)
        {
          //  std::cout << COLOR_GREEN << "index" << index << lane_point->pos << COLOR_RESET << "\n" << std::endl;
          log->add(lane_point->pos);
          index++;
        }
        // 中心线里有多个点
        //  std::cout << COLOR_GREEN << "size: " << lane_central_data->line_point_info.size() << COLOR_RESET << "\n" << std::endl;

        // 处理车道中的--------人行横道----------
        if (lane_info->cross_walk_list.empty())
        {
          continue;
        }
        for (const auto &cross_walk : lane_info->cross_walk_list)
        {
          auto log = session->add_debug_log(utils::DisplayInfo::POLYGEN, "cross_walk_binding");
          log->color = {165, 42, 42};
          for (auto &pt : cross_walk->list)
          {
            log->add(pt->pos);

          }
          // 绑定关系
          auto arrow_log1 = session->add_debug_log(utils::DisplayInfo::LINE, "cross_and_lane_relation");
          arrow_log1->color = {255, 105, 180}; // 粉红色
          arrow_log1->add(cross_walk->pos);
          int point_num = lane_central_data->line_point_info.size();
          auto pos1 = lane_central_data->line_point_info[point_num - 2]->pos;
          arrow_log1->add(pos1); // 中心线的中心点
        }
      }
    }

    session->save_debug_info("cross_walk_binding");
    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::save_intersection_binding_debug_info(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    if (not FLAGS_intersection_binding_debug_enable)
    {
      return fsdmap::SUCC;
    }

    session->set_display_name("intersection_binding");

    // 遍历所有的新车道组
    for (int i = 0; i < session->new_lane_groups.size(); ++i)
    {
      const auto &lane_group = session->new_lane_groups[i];

      // 添加车道组的索引日志，并设置为红色
      auto group_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "intersection_line_binding{}", i);
      group_index_log->color = {255, 0, 0}; // 红色

      // 遍历车道组中的车道线信息
      for (const auto &lane_info : lane_group->lane_line_info)
      {

        // 添加左右车道线的信息
        for (int lane_side = 0; lane_side < 2; ++lane_side)
        {
          const auto &lane_data =
              lane_side == 0 ? lane_info->left_lane_boundary_info : lane_info->right_lane_boundary_info;
          auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
          log->color = {255, 255, static_cast<double>(i)};

          // 对于新增或修改的车道组，设置为白色
          if (lane_group->joint_status == DataStatus::kNew || lane_group->joint_status == DataStatus::kMod)
          {
            log->color = {255, 255, 255}; // 白色
          }

          for (int k = 0; k < lane_data->line_point_info.size(); ++k)
          {
            const auto &lane_point = lane_data->line_point_info[k];
            if (k == 0)
            {
              group_index_log->add(lane_point->pos); // 对第一个点添加到车道组索引日志
            }
            log->add(lane_point->pos); // 添加车道点信息
          }
        }

        // 处理车道中心线信息
        const auto &lane_central_data = lane_info->center_lane_boundary_info;
        if (lane_central_data == nullptr)
        {
          continue;
        }
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {0, 255, static_cast<double>(i)}; // 绿色

        // // 根据车道组的状态设置不同颜色
        // if (lane_group->joint_status == DataStatus::kNew) {
        //   log->color = {223, static_cast<double>(i), 154}; // 特定颜色用于新增车道组
        // } else if (lane_group->joint_status == DataStatus::kMod) {
        //   log->color = {255, 165, static_cast<double>(i)}; // 修改状态的车道组颜色
        // }
        // int index = 0;
        // for (const auto& lane_point : lane_central_data->line_point_info) {
        //   //  std::cout << COLOR_GREEN << "index" << index << lane_point->pos << COLOR_RESET << "\n" << std::endl;
        //   log->add(lane_point->pos);
        //   index ++;
        // }

        // 中心线里有多个点
        //  std::cout << COLOR_GREEN << "size: " << lane_central_data->line_point_info.size() << COLOR_RESET << "\n" << std::endl;
        
      }
    }

    //修改画图，从路口找车道，  之前从车道找路口 会重复画路口，太慢了
    // 处理路口 
    for (auto &inter : session->raw_intersections){
      auto log = session->add_debug_log(utils::DisplayInfo::LINE, "inter");
      log->color = {255, 0, 0}; // 红色
      for (auto &pt : inter->lukou_poly_pts){
        log->add(pt);
      }

      if(inter->all_lanes.empty()){
        continue;
      }

      for(auto &lane : inter->all_lanes){
        auto inter_log1 = session->add_debug_log(utils::DisplayInfo::LINE, "relation");
        inter_log1->color = {255, 51, 204}; // 粉红色
        inter_log1->add(inter->pos);

        int middle_index = (lane->center_lane_boundary_info->line_point_info.size()) / 2;
        auto pos1 = lane->center_lane_boundary_info->line_point_info[middle_index]->pos;
        inter_log1->add(pos1); // 中心线的中心点
      }
    }
    session->save_debug_info("intersection_binding");
    return fsdmap::SUCC;
  }


  int RoadModelProcBindRelation::save_intersection_in_out_binding_debug_info(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);

    if (not FLAGS_intersection_binding_debug_enable)
    {
      return fsdmap::SUCC;
    }

    session->set_display_name("intersection_in_out_binding");

    // 遍历所有的新车道组
    for (int i = 0; i < session->new_lane_groups.size(); ++i)
    {
      const auto &lane_group = session->new_lane_groups[i];

      // 添加车道组的索引日志，并设置为红色
      auto group_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "intersection_line_binding{}", i);
      group_index_log->color = {255, 0, 0}; // 红色

      // 遍历车道组中的车道线信息
      for (const auto &lane_info : lane_group->lane_line_info)
      {

        // 添加左右车道线的信息
        for (int lane_side = 0; lane_side < 2; ++lane_side)
        {
          const auto &lane_data =
              lane_side == 0 ? lane_info->left_lane_boundary_info : lane_info->right_lane_boundary_info;
          auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
          log->color = {255, 255, static_cast<double>(i)};

          // 对于新增或修改的车道组，设置为白色
          if (lane_group->joint_status == DataStatus::kNew || lane_group->joint_status == DataStatus::kMod)
          {
            log->color = {255, 255, 255}; // 白色
          }

          for (int k = 0; k < lane_data->line_point_info.size(); ++k)
          {
            const auto &lane_point = lane_data->line_point_info[k];
            if (k == 0)
            {
              group_index_log->add(lane_point->pos); // 对第一个点添加到车道组索引日志
            }
            log->add(lane_point->pos); // 添加车道点信息
          }
        }

        // 处理车道中心线信息
        const auto &lane_central_data = lane_info->center_lane_boundary_info;
        if (lane_central_data == nullptr)
        {
          continue;
        }
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {0, 255, static_cast<double>(i)}; // 绿色

        // // 根据车道组的状态设置不同颜色
        // if (lane_group->joint_status == DataStatus::kNew) {
        //   log->color = {223, static_cast<double>(i), 154}; // 特定颜色用于新增车道组
        // } else if (lane_group->joint_status == DataStatus::kMod) {
        //   log->color = {255, 165, static_cast<double>(i)}; // 修改状态的车道组颜色
        // }
        // int index = 0;
        // for (const auto& lane_point : lane_central_data->line_point_info) {
        //   //  std::cout << COLOR_GREEN << "index" << index << lane_point->pos << COLOR_RESET << "\n" << std::endl;
        //   log->add(lane_point->pos);
        //   index ++;
        // }

        // 中心线里有多个点
        //  std::cout << COLOR_GREEN << "size: " << lane_central_data->line_point_info.size() << COLOR_RESET << "\n" << std::endl;
      }
    }
    //从路口找进入退出车道
    for (auto &inter : session->raw_intersections){
      auto log = session->add_debug_log(utils::DisplayInfo::LINE, "inter");
      log->color = {255, 0, 0}; // 红色
      for (auto &pt : inter->lukou_poly_pts){
        log->add(pt);
      }

      for(auto &in_lane : inter->in_lanes){
        auto inter_log1 = session->add_debug_log(utils::DisplayInfo::LINE, "in_relation");
        int middle_index = (in_lane->center_lane_boundary_info->line_point_info.size()) / 2;
        auto pos1 = in_lane->center_lane_boundary_info->line_point_info[middle_index]->pos;
        inter_log1->add(pos1); // 中心线的中心点

        inter_log1->color = {255, 51, 204}; // 粉红色
        inter_log1->add(inter->pos);
      }

      for(auto &out_lane : inter->out_lanes){
        auto inter_log2 = session->add_debug_log(utils::DisplayInfo::LINE, "out_relation");
        inter_log2->color = {0, 102, 204 }; //蓝色
        inter_log2->add(inter->pos);

        int middle_index2 = (out_lane->center_lane_boundary_info->line_point_info.size()) / 2;
        auto pos2 = out_lane->center_lane_boundary_info->line_point_info[middle_index2]->pos;
        inter_log2->add(pos2); 
      }
    }

    session->save_debug_info("intersection_in_out_binding");
    return fsdmap::SUCC;
  }

  int RoadModelProcBindRelation::save_intersection_binding_cross_wallk_debug_info(RoadModelSessionData *session)
  {
    CHECK_NOTNULL(session);
    if (not FLAGS_intersection_binding_cross_walk_debug_enable)
    {
      return fsdmap::SUCC;
    }

    session->set_display_name("intersection_binding_cross_walk");
    for (int i = 0; i < session->new_lane_groups.size(); ++i)
    {
      const auto &lane_group = session->new_lane_groups[i];
      // 添加车道组的索引日志，并设置为红色
      auto group_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "intersection_line_binding{}", i);
      group_index_log->color = {255, 0, 0}; // 红色
      // 遍历车道组中的车道线信息
      for (const auto &lane_info : lane_group->lane_line_info)
      {
        // 添加左右车道线的信息
        for (int lane_side = 0; lane_side < 2; ++lane_side)
        {
          const auto &lane_data =
              lane_side == 0 ? lane_info->left_lane_boundary_info : lane_info->right_lane_boundary_info;
          auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
          log->color = {255, 255, static_cast<double>(i)};

          // 对于新增或修改的车道组，设置为白色
          if (lane_group->joint_status == DataStatus::kNew || lane_group->joint_status == DataStatus::kMod)
          {
            log->color = {255, 255, 255}; // 白色
          }

          for (int k = 0; k < lane_data->line_point_info.size(); ++k)
          {
            const auto &lane_point = lane_data->line_point_info[k];
            if (k == 0)
            {
              group_index_log->add(lane_point->pos); // 对第一个点添加到车道组索引日志
            }
            log->add(lane_point->pos); // 添加车道点信息
          }
        }

        // 处理车道中心线信息
        const auto &lane_central_data = lane_info->center_lane_boundary_info;
        if (lane_central_data == nullptr)
        {
          continue;
        }
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {0, 255, static_cast<double>(i)}; // 绿色

        // // 根据车道组的状态设置不同颜色
        // if (lane_group->joint_status == DataStatus::kNew) {
        //   log->color = {223, static_cast<double>(i), 154}; // 特定颜色用于新增车道组
        // } else if (lane_group->joint_status == DataStatus::kMod) {
        //   log->color = {255, 165, static_cast<double>(i)}; // 修改状态的车道组颜色
        // }
        // int index = 0;
        // for (const auto& lane_point : lane_central_data->line_point_info) {
        //   //  std::cout << COLOR_GREEN << "index" << index << lane_point->pos << COLOR_RESET << "\n" << std::endl;
        //   log->add(lane_point->pos);
        //   index ++;
        // }

        // 中心线里有多个点
        //  std::cout << COLOR_GREEN << "size: " << lane_central_data->line_point_info.size() << COLOR_RESET << "\n" << std::endl;
      }
    }

    for (auto &inter : session->raw_intersections){
      auto log = session->add_debug_log(utils::DisplayInfo::LINE, "inter");
      log->color = {255, 0, 0}; // 红色
      for (auto &pt : inter->lukou_poly_pts){
        log->add(pt);
      }

      if(inter->cross_walk_list.empty()){
        continue;
      }

      for (const auto &cross_walk : inter->cross_walk_list)
      {
        auto log = session->add_debug_log(utils::DisplayInfo::POLYGEN, "cross_walk");
        log->color = {165, 42, 42};//浅红色
        for (auto &pt : cross_walk->list){
          log->add(pt->pos);
        }
        // 绑定关系
        auto arrow_log1 = session->add_debug_log(utils::DisplayInfo::LINE, "cross_and_intersection_relation");
        arrow_log1->color = {255, 105, 180}; // 粉红色
        arrow_log1->add(cross_walk->pos);
        arrow_log1->add(inter->pos); // 中心线的中心点
      }
    }

    session->save_debug_info("intersection_binding_cross_walk");
    return fsdmap::SUCC;
  }


  bool RoadModelProcBindRelation::junction_contains_lanegroup(const RoadLaneGroupInfo *road_lane_group, JunctionInfo *junction)
  {
    for (auto &lane_boundary : road_lane_group->lane_boundary_info)
    {
      for (auto &pt : lane_boundary->line_point_info)
      {
        if (!alg::point_in_polygon(pt->pos, junction->lukou_poly_pts))
        {
          return false;
        }
      }
    }
    return true;
  }

  int RoadModelProcBindRelation::delete_lanegroup_contined_lukoubd_withsuccs(RoadModelSessionData *session)
  {

    for (int i = 0; i < session->new_lane_groups.size(); i++)
    {
      for (auto &junction : session->new_junction_list)
      {
        // 1、判断车道组是否完全在路口边界中
        if (!junction_contains_lanegroup(session->new_lane_groups[i], junction.get()))
        {
          continue;
        }
        // 2、判断是否有后继
        if (session->new_lane_groups[i]->context.all_prev.size() > 0)
        {
          continue;
        }
        // // 3、进行删除操作
        // VEC_ERASE(session->new_lane_groups, i);
      }
    }

    return fsdmap::SUCC;
  }

  // int RoadModelProcBindRelation::export_to_shape_file(RoadModelSessionData *session)
  // {

  //   export_lane_boundary_to_shp(session);

  //   export_lane_centers_to_shp(session);

  //   export_road_boundary_to_shp(session);

  //   export_road_to_shp(session);

  //   export_lane_to_shp(session);

  //   // export_lane_adas_to_shp(session);

  //   // export_road_centers_to_shp(session);

  //   export_stop_line_to_shp(session);

  //   export_cross_walks_to_shp(session);

  //   export_road_marks_to_shp(session);

  //   export_intersection_to_shp(session);

  //   return fsdmap::SUCC;
//   // }

//   int RoadModelProcBindRelation::export_lane_boundary_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "lane boundaries 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("lane_boundaries", &poSpatialRef, wkbLineString, nullptr);
//     if (!layer)
//     {
//       std::cerr << "lane boundaries 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 ID 字段
//     OGRFieldDefn fieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries 创建字段 ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 color 字段
//     OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
//     if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 COLOR 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }
//     // 添加 shape 字段
//     OGRFieldDefn shapetypeFieldDefn("SHAPE_TYPE", OFTInteger);
//     if (layer->CreateField(&shapetypeFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 SHAPE_TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 is_safety 字段
//     OGRFieldDefn safetyFieldDefn("IS_SAFETY", OFTInteger);
//     // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&safetyFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 IS_SAFETY 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 lane_id 字段
//     OGRFieldDefn laneidFieldDefn("LANE_ID", OFTString);
//     // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&laneidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 LANE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 pre_id 字段
//     OGRFieldDefn preidFieldDefn("PRE_ID", OFTString);
//     // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 PRE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 next_id 字段
//     OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString);
//     // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 NEXT_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 frame_id 字段
//     OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 version 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 写入数据
//     int featureId = 0;
//     for (const auto &lane_group : session->new_lane_groups)
//     {
//       for (const auto &lane_info : lane_group->lane_line_info)
//       {
//         for (int lane_side = 0; lane_side < 2; ++lane_side)
//         {
//           const auto &lane_data =
//               lane_side == 0 ? lane_info->left_lane_boundary_info : lane_info->right_lane_boundary_info;

//           OGRLineString lineString;
//           for (const auto &lane_point : lane_data->line_point_info)
//           {
//             Eigen::Vector3d wgs;
//             session->data_processer->local2wgs(lane_point->pos, wgs);
//             // double lng_gcj2;
//             // double lat_gcj2;
//             // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

//             double lng_gcj2 = wgs.x();
//             double lat_gcj2 = wgs.y();
//             // TODO:qzc gcj01
//             // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//             // TODO:qzc gcj01

//             // Eigen::Vector3d lng_lat_02;
//             // lng_lat_02.x() = lng_gcj2;
//             // lng_lat_02.y() = lat_gcj2;
//             // lng_lat_02.z() = 0;
//             // Eigen::Vector3d utm_02;
//             // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
//             // lineString.addPoint(utm_02.x(), utm_02.y());

//             lineString.addPoint(lng_gcj2, lat_gcj2);

//             // lineString.addPoint(lane_point->pos.x(), lane_point->pos.y());
//           }

//           std::ostringstream prev_oss, next_oss;
//           for (size_t i = 0; i < lane_info->context.all_prev.size(); i++)
//           {
//             if (i > 0)
//               prev_oss << ",";
//             const auto &lane_prev_data =
//                 lane_side == 0 ? lane_info->context.all_prev[i].src->left_lane_boundary_info : lane_info->context.all_prev[i].src->right_lane_boundary_info;
//             prev_oss << lane_prev_data->lane_id;
//           }

//           for (size_t i = 0; i < lane_info->context.all_next.size(); i++)
//           {
//             if (i > 0)
//               next_oss << ",";
//             const auto &lane_next_data =
//                 lane_side == 0 ? lane_info->context.all_next[i].src->left_lane_boundary_info : lane_info->context.all_next[i].src->right_lane_boundary_info;
//             next_oss << lane_next_data->lane_id;
//           }

//           OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
//           feature->SetField("ID", lane_data->lane_id);
//           feature->SetField("COLOR", lane_data->color);
//           // feature->SetField("SHAPE_TYPE", lane_data->); // 暂时没有
//           // feature->SetField("IS_SAFETY", lane_data->geo);  // 暂时没有
//           std::string laneIdStr = std::to_string(lane_data->lane_id);
//           feature->SetField("LANE_ID", laneIdStr.c_str()); // 与车道线同ID，后续排查
//           feature->SetField("PRE_ID", prev_oss.str().c_str());
//           feature->SetField("NEXT_ID", next_oss.str().c_str());
//           // feature->SetField("FRAME_ID", );  // 暂时没有
//           // feature->SetField("VERSION", );  // 暂时没有

//           feature->SetGeometry(&lineString);

//           if (layer->CreateFeature(feature) != OGRERR_NONE)
//           {
//             std::cerr << "lane boundaries 创建要素失败" << std::endl;
//           }

//           OGRFeature::DestroyFeature(feature);
//         }
//       }
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }

//   int RoadModelProcBindRelation::export_lane_centers_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "lane centers 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("lane_centers", &poSpatialRef, wkbLineString, nullptr);
//     if (!layer)
//     {
//       std::cerr << "lane centers 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加字段
//     OGRFieldDefn fieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane centers 创建字段 ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 lane_id 字段
//     OGRFieldDefn laneidfieldDefn("LANE_ID", OFTInteger);
//     if (layer->CreateField(&laneidfieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane centers 创建字段 LANE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 type 字段
//     OGRFieldDefn typefieldDefn("TYPE", OFTInteger);
//     if (layer->CreateField(&typefieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane centers 创建字段 TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 frame_id 字段
//     OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 version 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 写入数据
//     int featureId = 0;
//     for (const auto &lane_group : session->new_lane_groups)
//     {
//       for (const auto &lane_info : lane_group->lane_line_info)
//       {
//         const auto &lane_central_data = lane_info->center_lane_boundary_info;

//         OGRLineString lineString;
//         for (const auto &lane_point : lane_central_data->line_point_info)
//         {
//           Eigen::Vector3d wgs;
//           session->data_processer->local2wgs(lane_point->pos, wgs);
//           // double lng_gcj2;
//           // double lat_gcj2;
//           // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

//           double lng_gcj2 = wgs.x();
//           double lat_gcj2 = wgs.y();
//           // TODO:qzc gcj01
//           // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//           // TODO:qzc gcj01

//           // Eigen::Vector3d lng_lat_02;
//           // lng_lat_02.x() = lng_gcj2;
//           // lng_lat_02.y() = lat_gcj2;
//           // lng_lat_02.z() = 0;
//           // Eigen::Vector3d utm_02;
//           // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
//           // lineString.addPoint(utm_02.x(), utm_02.y());

//           lineString.addPoint(lng_gcj2, lat_gcj2);

//           // lineString.addPoint(lane_point->pos.x(), lane_point->pos.y());
//         }

//         OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
//         feature->SetField("ID", lane_central_data->lane_id);
//         feature->SetField("LANE_ID", lane_central_data->lane_id);
//         // feature->SetField("TYPE", lane_central_data->color); // 暂时没有
//         // feature->SetField("SHAPE", lane_central_data->); // 暂时没有
//         // feature->SetField("FRAME_ID", );  // 暂时没有
//         // feature->SetField("VERSION", );  // 暂时没有
//         feature->SetGeometry(&lineString);

//         if (layer->CreateFeature(feature) != OGRERR_NONE)
//         {
//           std::cerr << "lane centers 创建要素失败" << std::endl;
//         }

//         OGRFeature::DestroyFeature(feature);
//       }
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }


//   int RoadModelProcBindRelation::export_road_boundary_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "road boundaries 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("road_boundaries", &poSpatialRef, wkbLineString, nullptr);
//     if (!layer)
//     {
//       std::cerr << "road boundaries 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加ID字段
//     OGRFieldDefn fieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road boundaries 创建字段失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 color 字段
//     OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
//     if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road boundaries  创建字段 COLOR 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加type字段
//     OGRFieldDefn typefieldDefn("TYPE", OFTInteger);
//     if (layer->CreateField(&typefieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road boundaries 创建字段 TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 is_safety 字段
//     OGRFieldDefn safetyFieldDefn("IS_SAFETY", OFTInteger);
//     // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&safetyFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road boundaries  创建字段 IS_SAFETY 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 road_id 字段
//     OGRFieldDefn roadidFieldDefn("ROAD_ID", OFTInteger);
//     // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&roadidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road boundaries  创建字段 ROAD_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 pre_id 字段
//     OGRFieldDefn preidFieldDefn("PRE_ID", OFTString);
//     // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road boundaries  创建字段 PRE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 next_id 字段
//     OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString);
//     // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road boundaries  创建字段 NEXT_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 frame_id 字段
//     OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road boundaries  创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 version 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road boundaries  创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 写入数据
//     int featureId = 0;
//     for (const auto &lane_group : session->new_lane_groups)
//     {
//       for (int lane_side = 0; lane_side < 2; ++lane_side)
//       {
//         const auto &barrier_info =
//             lane_side == 0 ? lane_group->left_barrier_segment_info : lane_group->right_barrier_segment_info;
//         for (auto &barrier_data : barrier_info)
//         {
//           OGRLineString lineString;

//           for (const auto &lane_point : barrier_data->point_info)
//           {
//             Eigen::Vector3d wgs;
//             session->data_processer->local2wgs(lane_point->pos, wgs);
//             // double lng_gcj2;
//             // double lat_gcj2;
//             // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

//             double lng_gcj2 = wgs.x();
//             double lat_gcj2 = wgs.y();
//             // TODO:qzc gcj01
//             // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//             // TODO:qzc gcj01

//             // Eigen::Vector3d lng_lat_02;
//             // lng_lat_02.x() = lng_gcj2;
//             // lng_lat_02.y() = lat_gcj2;
//             // lng_lat_02.z() = 0;
//             // Eigen::Vector3d utm_02;
//             // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
//             // lineString.addPoint(utm_02.x(), utm_02.y());

//             lineString.addPoint(lng_gcj2, lat_gcj2);

//             // lineString.addPoint(lane_point->pos.x(), lane_point->pos.y());
//           }

//           std::ostringstream prev_oss, next_oss;
//           for (size_t i = 0; i < barrier_data->context.all_prev.size(); i++)
//           {
//             if (i > 0)
//               prev_oss << ",";
//             prev_oss << barrier_data->context.all_prev[i].src->boundary_id;
//           }

//           for (size_t i = 0; i < barrier_data->context.all_next.size(); i++)
//           {
//             if (i > 0)
//               next_oss << ",";
//             next_oss << barrier_data->context.all_next[i].src->boundary_id;
//           }

//           OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
//           feature->SetField("ID", barrier_data->boundary_id);
//           // feature->SetField("COLOR", barrier_data->color); // 暂时没有
//           // feature->SetField("TYPE", lane_data->); // 暂时没有
//           // feature->SetField("IS_SAFETY", lane_data->geo);  // 暂时没有
//           feature->SetField("ROAD_ID", lane_group->group_index);
//           feature->SetField("PRE_ID", prev_oss.str().c_str());
//           feature->SetField("NEXT_ID", next_oss.str().c_str());

//           // feature->SetField("FRAME_ID", );  // 暂时没有
//           // feature->SetField("VERSION", );  // 暂时没有

//           feature->SetGeometry(&lineString);

//           if (layer->CreateFeature(feature) != OGRERR_NONE)
//           {
//             std::cerr << "road boundaries 创建要素失败" << std::endl;
//           }

//           OGRFeature::DestroyFeature(feature);
//         }
//       }
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }

//  int RoadModelProcBindRelation::export_road_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "road 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("road", &poSpatialRef, wkbPolygon, nullptr);
//     if (!layer)
//     {
//       std::cerr << "road 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 ID 字段
//     OGRFieldDefn fieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road 创建字段 ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 KIND 字段
//     OGRFieldDefn roadtypefieldDefn("KIND", OFTInteger);
//     if (layer->CreateField(&roadtypefieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road 创建字段 KIND 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 IS_BIDIR 字段
//     OGRFieldDefn isbidirfieldDefn("IS_BIDIR", OFTInteger);
//     if (layer->CreateField(&isbidirfieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road 创建字段 IS_BIDIR 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 SCEN_TYPE 字段
//     OGRFieldDefn scentypefieldDefn("SCEN_TYPE", OFTInteger);
//     if (layer->CreateField(&scentypefieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road 创建字段 SCEN_TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 TYPE 字段
//     OGRFieldDefn fowfieldDefn("TYPE", OFTInteger);
//     if (layer->CreateField(&fowfieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road 创建字段 TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 SPEED 字段
//     OGRFieldDefn speedfieldDefn("SPEED", OFTInteger);
//     if (layer->CreateField(&speedfieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road 创建字段 SPEED 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // // 添加 road_line 字段
//     // OGRFieldDefn roadlineFieldDefn("ROAD_LINE", OFTString);
//     // if (layer->CreateField(&roadlineFieldDefn) != OGRERR_NONE)
//     // {
//     //   std::cerr << "road  创建字段 ROAD_LINE 失败" << std::endl;
//     //   GDALClose(dataset);
//     //   return fsdmap::FAIL;
//     // }

//     // 添加 lane_id 字段
//     OGRFieldDefn laneidFieldDefn("LANE_ID", OFTString);
//     if (layer->CreateField(&laneidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 LANE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 left_bid 字段
//     OGRFieldDefn leftbidFieldDefn("LEFT_BID", OFTString);
//     if (layer->CreateField(&leftbidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 LEFT_BID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 right_bid 字段
//     OGRFieldDefn rightbidFieldDefn("RIGHT_BID", OFTString);
//     if (layer->CreateField(&rightbidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 RIGHT_BID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 in_id 字段
//     OGRFieldDefn inidFieldDefn("IN_ID", OFTString);
//     if (layer->CreateField(&inidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 IN_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 cross_id 字段
//     OGRFieldDefn crossidFieldDefn("CROSS_ID", OFTString);
//     if (layer->CreateField(&crossidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 CROSS_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 out_id 字段
//     OGRFieldDefn outidFieldDefn("OUT_ID", OFTString);
//     if (layer->CreateField(&outidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 OUT_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 pre_id 字段
//     OGRFieldDefn preidFieldDefn("PRE_ID", OFTString); // 工艺暂时不需要
//     // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 PRE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 next_id 字段
//     OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString); // 工艺暂时不需要
//     // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 NEXT_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 frame_id 字段
//     OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 version 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road  创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 写入数据
//     int featureId = 0;
//     for (const auto &srs : session->sub_road_segment_list)
//     {
//       for (const auto &lg : srs->new_lane_groups)
//       {

//         // 创建一个新的多边形对象
//         OGRPolygon *polygon = new OGRPolygon();

//         // 创建一个线性环（外环）
//         OGRLinearRing *outerRing = new OGRLinearRing();
//         Eigen::Vector3d wgs;

//         double first_point_x(0), first_point_y(0);

//         auto line = lg->lane_line_info[0]->left_lane_boundary_info;
//         for (size_t tt = 0; tt < line->line_point_info.size(); tt++)
//         {
//           session->data_processer->local2wgs(line->line_point_info[tt]->pos, wgs);

//           double lng_gcj2 = wgs.x();
//           double lat_gcj2 = wgs.y();
//           // TODO:qzc gcj01
//           // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//           // TODO:qzc gcj01

//           outerRing->addPoint(lng_gcj2, lat_gcj2);

//           if (tt == 0)
//           {
//             first_point_x = lng_gcj2;
//             first_point_y = lat_gcj2;
//           }
//         }

//         line = lg->lane_line_info[lg->lane_line_info.size() - 1]->right_lane_boundary_info;
//         for (int t = line->line_point_info.size() - 1; t >= 0; t--)
//         {

//           session->data_processer->local2wgs(line->line_point_info[t]->pos, wgs);

//           double lng_gcj2 = wgs.x();
//           double lat_gcj2 = wgs.y();
//           // TODO:qzc gcj01
//           // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//           // TODO:qzc gcj01

//           outerRing->addPoint(lng_gcj2, lat_gcj2);
//         }
//         outerRing->addPoint(first_point_x, first_point_y); // 闭合多边形

//         // 确保线性环闭合
//         outerRing->closeRings();

//         // 将线性环添加到多边形中
//         polygon->addRing(outerRing);

//         std::ostringstream lane_id_oss, left_bid_oss, right_bid_oss, all_prev_oss, all_next_oss;
//         for (size_t i = 0; i < lg->lane_line_info.size(); i++)
//         {
//           if (i > 0)
//           {
//             lane_id_oss << ",";
//           }
//           lane_id_oss << lg->lane_line_info[i]->center_lane_boundary_info->lane_id;
//         }
//         for (size_t i = 0; i < lg->left_barrier_segment_info.size(); i++) // 可能有问题
//         {
//           if (i > 0)
//           {
//             left_bid_oss << ",";
//           }
//           left_bid_oss << lg->left_barrier_segment_info[i]->boundary_id;
//         }
//         for (size_t i = 0; i < lg->right_barrier_segment_info.size(); i++)
//         {
//           if (i > 0)
//           {
//             right_bid_oss << ",";
//           }
//           right_bid_oss << lg->right_barrier_segment_info[i]->boundary_id;
//         }

//         for (size_t i = 0; i < lg->context.all_prev.size(); i++) // 工艺暂时不需要
//         {
//           if (i > 0)
//           {
//             all_prev_oss << ",";
//           }
//           all_prev_oss << lg->context.all_prev[i].src->group_index;
//         }

//         for (size_t i = 0; i < lg->context.all_next.size(); i++)
//         {
//           if (i > 0)
//           {
//             all_next_oss << ",";
//           }
//           all_next_oss << lg->context.all_next[i].src->group_index;
//         }

//         OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
//         feature->SetField("ID", lg->group_index);
//         // feature->SetField("KIND", lg->group_index); // 道路类型，暂时没有
//         // feature->SetField("IS_BIDIR", lg->group_index); // 是否双向道路，暂时没有
//         // feature->SetField("SCEN_TYPE", lg->group_index); // 规划等级，暂时没有
//         // feature->SetField("TYPE", lg->group_index); // 道路构成，暂时没有
//         // feature->SetField("SPEED", lg->group_index); // 道路限速，暂时没有
//         // // feature->SetField("ROAD_LINE", left_bid_oss.to_str()); //关联道路中心线
//         feature->SetField("LANE_ID", lane_id_oss.str().c_str());     // 关联车道ID
//         feature->SetField("LEFT_BID", left_bid_oss.str().c_str());   // 关联左侧道路边界
//         feature->SetField("RIGHT_BID", right_bid_oss.str().c_str()); // 关联右侧道路边界
//         // feature->SetField("IN_ID", lane_data->); // 进入路口编号，暂时没有
//         // feature->SetField("CROSS_ID", lane_data->color); // 通过路口编号，暂时没有
//         // feature->SetField("OUT_ID", lane_data->); // 退出路口编号，暂时没有
//         feature->SetField("PRE_ID", all_prev_oss.str().c_str());  // 前序道路
//         feature->SetField("NEXT_ID", all_next_oss.str().c_str()); // 后续道路

//         // feature->SetField("FRAME_ID", );  // 暂时没有
//         // feature->SetField("VERSION", );  // 暂时没有

//         feature->SetGeometry(polygon);

//         if (layer->CreateFeature(feature) != OGRERR_NONE)
//         {
//           std::cerr << "road 创建要素失败" << std::endl;
//         }

//         OGRFeature::DestroyFeature(feature);
//       }
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }

//   int RoadModelProcBindRelation::export_lane_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "lane 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("lane", &poSpatialRef, wkbPolygon, nullptr);
//     if (!layer)
//     {
//       std::cerr << "lane 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }


//     // 添加 id 字段
//     OGRFieldDefn idFieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&idFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 lane_type 字段
//     OGRFieldDefn lanetypeFieldDefn("LANE_TYPE", OFTInteger);
//     if (layer->CreateField(&lanetypeFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 LANE_TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 turn_type 字段
//     OGRFieldDefn turntypeFieldDefn("TURN_TYPE", OFTString);
//     if (layer->CreateField(&turntypeFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 TURN_TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 road_id 字段
//     OGRFieldDefn roadidFieldDefn("ROAD_ID", OFTInteger);
//     if (layer->CreateField(&roadidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 ROAD_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 left_ln 字段
//     OGRFieldDefn leftlnFieldDefn("LEFT_LN", OFTInteger);
//     if (layer->CreateField(&leftlnFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 LEFT_LN 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 right_ln 字段
//     OGRFieldDefn rightlnFieldDefn("RIGHT_LN", OFTInteger);
//     if (layer->CreateField(&rightlnFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 RIGHT_LN 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 left_r_ln 字段
//     OGRFieldDefn leftrlnFieldDefn("LEFT_R_LN", OFTInteger);
//     if (layer->CreateField(&leftrlnFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 LEFT_R_LN 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 right_r_ln 字段
//     OGRFieldDefn rightrlnFieldDefn("RIGHT_R_LN", OFTInteger);
//     if (layer->CreateField(&rightrlnFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 RIGHT_R_LN 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 left_bid 字段
//     OGRFieldDefn leftbidFieldDefn("LEFT_BID", OFTInteger);
//     if (layer->CreateField(&leftbidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 LEFT_BID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 right_bid 字段
//     OGRFieldDefn rightbidFieldDefn("RIGHT_BID", OFTInteger);
//     if (layer->CreateField(&rightbidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 RIGHT_BID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 obj_id 字段
//     OGRFieldDefn objidFieldDefn("OBJ_ID", OFTString);
//     if (layer->CreateField(&objidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 OBJ_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 obj_type 字段
//     OGRFieldDefn objtypeFieldDefn("OBJ_TYPE", OFTString);
//     if (layer->CreateField(&objtypeFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 OBJ_TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 inter_id 字段
//     OGRFieldDefn interidFieldDefn("INTER_ID", OFTInteger);
//     if (layer->CreateField(&interidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 INTER_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 pre_id 字段
//     OGRFieldDefn preidFieldDefn("PRE_ID", OFTString);
//     // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 PRE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 next_id 字段
//     OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString);
//     // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 NEXT_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 frame_id 字段
//     OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 version 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane  创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 写入数据
//     int featureId = 0;
//     for (const auto &lane_group : session->new_lane_groups)
//     {
//       for (size_t i = 0; i < lane_group->lane_line_info.size(); i++)
//       {
//         const auto &lane_info = lane_group->lane_line_info[i];

//         // 创建一个新的多边形对象
//         OGRPolygon *polygon = new OGRPolygon();

//         // 创建一个线性环（外环）
//         OGRLinearRing *outerRing = new OGRLinearRing();
//         Eigen::Vector3d wgs;

//         double first_point_x(0), first_point_y(0);

//         for (size_t tt = 0; tt < lane_info->left_lane_boundary_info->line_point_info.size(); tt++)
//         {
//           session->data_processer->local2wgs(lane_info->left_lane_boundary_info->line_point_info[tt]->pos, wgs);

//           double lng_gcj2 = wgs.x();
//           double lat_gcj2 = wgs.y();
//           // TODO:qzc gcj01
//           // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//           // TODO:qzc gcj01

//           outerRing->addPoint(lng_gcj2, lat_gcj2);

//           if (tt == 0)
//           {
//             first_point_x = lng_gcj2;
//             first_point_y = lat_gcj2;
//           }
//         }

//         for (int t = lane_info->right_lane_boundary_info->line_point_info.size() - 1; t >= 0; t--)
//         {

//           session->data_processer->local2wgs(lane_info->right_lane_boundary_info->line_point_info[t]->pos, wgs);

//           double lng_gcj2 = wgs.x();
//           double lat_gcj2 = wgs.y();
//           // TODO:qzc gcj01
//           // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//           // TODO:qzc gcj01

//           outerRing->addPoint(lng_gcj2, lat_gcj2);
//         }
//         outerRing->addPoint(first_point_x, first_point_y); // 闭合多边形

//         // 确保线性环闭合
//         outerRing->closeRings();

//         // 将线性环添加到多边形中
//         polygon->addRing(outerRing);

//         OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());

//         // LOG_INFO("lane center id is : {}, lane center feature list size :{}, lane center bind arrow  list size :{}", lane_info->center_lane_boundary_info->lane_id, lane_info->lane_center_feature_list.size(), lane_info->bind_arrow_list.size());

//         if (lane_info->lane_center_feature_list.size() > 0)
//         {
//           std::ostringstream turn_type_oss;
//           for (size_t i = 0; i < lane_info->bind_arrow_list.size(); i++)
//           {
//             if (i > 0)
//             {
//               turn_type_oss << ",";
//             }
//             turn_type_oss << lane_info->bind_arrow_list[i]->type;
//           }
//           feature->SetField("TURN_TYPE", turn_type_oss.str().c_str()); // 车道转向类型
//         }

//         feature->SetField("ID", lane_info->center_lane_boundary_info->lane_id); // 关联车道中心线ID
//         feature->SetField("ROAD_ID", lane_info->lane_group_info->group_index);     // 归属车道组ID
//         if (i > 0)
//         {
//           feature->SetField("LEFT_LN", lane_group->lane_line_info[i - 1]->center_lane_boundary_info->lane_id); // 左侧同向车道ID
//         }
//         if (i < lane_group->lane_line_info.size() - 1)
//         {
//           feature->SetField("RIGHT_LN", lane_group->lane_line_info[i + 1]->center_lane_boundary_info->lane_id); // 右侧同向车道ID
//         }

//         std::ostringstream prev_oss, next_oss;
//         for (size_t i = 0; i < lane_info->context.all_prev.size(); i++)
//         {
//           if (i > 0)
//             prev_oss << ",";
//           prev_oss << lane_info->context.all_prev[i].src->center_lane_boundary_info->lane_id;
//         }

//         for (size_t i = 0; i < lane_info->context.all_next.size(); i++)
//         {
//           if (i > 0)
//             next_oss << ",";
//           next_oss << lane_info->context.all_next[i].src->center_lane_boundary_info->lane_id;
//         }

//         // feature->SetField("LANE_TYPE", lane_data->); // 车道类型，暂时没有
//         // feature->SetField("LEFT_R_LN", lane_data->); // 左侧反向车道ID，暂时没有
//         // feature->SetField("RIGHT_R_LN", prev_oss.str().c_str());// 右侧反向车道ID，暂时没有
//         feature->SetField("LEFT_BID", lane_info->left_lane_boundary_info->lane_id);   // 左侧车道边界线ID
//         feature->SetField("RIGHT_BID", lane_info->right_lane_boundary_info->lane_id); // 右侧车道边界线ID
//         // feature->SetField("OBJ_ID", next_oss.str().c_str()); //关联定位要素ID，暂时没有
//         // feature->SetField("OBJ_TYPE", next_oss.str().c_str()); //关联定位要素ID，暂时没有
//         // feature->SetField("INTER_ID", next_oss); //所在路口编号，暂时没有
//         feature->SetField("PRE_ID", prev_oss.str().c_str());  // 前序车道
//         feature->SetField("NEXT_ID", next_oss.str().c_str()); // 后序车道

//         // feature->SetField("FRAME_ID", );  // 暂时没有
//         // feature->SetField("VERSION", );  // 暂时没有

//         feature->SetGeometry(polygon);

//         if (layer->CreateFeature(feature) != OGRERR_NONE)
//         {
//           std::cerr << "lane 创建要素失败" << std::endl;
//         }

//         OGRFeature::DestroyFeature(feature);
//       }
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }

//   int RoadModelProcBindRelation::export_lane_adas_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "lane adas 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("lane_adas", &poSpatialRef, wkbLineString, nullptr);
//     if (!layer)
//     {
//       std::cerr << "lane adas 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 cl_id 字段
//     OGRFieldDefn clidFieldDefn("CL_ID", OFTInteger);
//     if (layer->CreateField(&clidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane adas  创建字段 CL_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 index 字段
//     OGRFieldDefn indexFieldDefn("INDEX", OFTInteger);
//     if (layer->CreateField(&indexFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane adas  创建字段 INDEX 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 curvature 字段
//     OGRFieldDefn curvaturefieldDefn("CURVATURE", OFTInteger);
//     if (layer->CreateField(&curvaturefieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane adas 创建字段 CURVATURE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 写入数据
//     int featureId = 0;
//     for (const auto &lane_group : session->new_lane_groups)
//     {
//       for (const auto &lane_info : lane_group->lane_line_info)
//       {
//         for (int i = 0; i < lane_info->lane_center_feature_list.size(); i++)
//         {
//           double cura = lane_info->lane_center_feature_list[i]->curvature;
//           cura = 1 / cura * 1E5;
//           int K = cura > 0 ? static_cast<int>(cura) : static_cast<int>(std::floor(cura));

//           OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());

//           feature->SetField("CL_ID", lane_info->center_lane_boundary_info->lane_id);
//           // feature->SetField("INDEX", static_cast<int>(lane_info->lane_center_feature_list[i]->line_index)); // 有问题
//           feature->SetField("INDEX", i);
//           feature->SetField("CURVATURE", K);
//           // feature->SetGeometry(&lineString);

//           if (layer->CreateFeature(feature) != OGRERR_NONE)
//           {
//             std::cerr << "lane adas 创建要素失败" << std::endl;
//           }

//           OGRFeature::DestroyFeature(feature);
//         }
//       }
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }

//   int RoadModelProcBindRelation::export_road_centers_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "road centers 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("road_centers", &poSpatialRef, wkbLineString, nullptr);
//     if (!layer)
//     {
//       std::cerr << "road centers 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加字段
//     OGRFieldDefn fieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road centers 创建字段 ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 road_id 字段
//     OGRFieldDefn roadidfieldDefn("ROAD_ID", OFTInteger);
//     if (layer->CreateField(&roadidfieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "road centers 创建字段 road_id 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 frame_id 字段
//     OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 version 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "lane boundaries  创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 写入数据
//     int featureId = 0;
//     for (const auto &lane_group : session->new_lane_groups)
//     {
//       auto road_center = lane_group->road_center_line_info;
// #if 0
//       if (road_center == NULL)
//       {
//         LOG_INFO("road_center is not exist");
//       }
//       else
//       {
//         LOG_INFO("road_center is exist");
//       }
// #endif
//       if (road_center == NULL)
//       {
//         continue;
//       }

//       OGRLineString lineString;
//       for (const auto &lane_point : road_center->line_point_info)
//       {
//         Eigen::Vector3d wgs;
//         session->data_processer->local2wgs(lane_point->pos, wgs);

//         double lng_gcj2 = wgs.x();
//         double lat_gcj2 = wgs.y();
//         // TODO:qzc gcj01
//         // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//         // TODO:qzc gcj01

//         lineString.addPoint(lng_gcj2, lat_gcj2);
//       }

//       OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
//       feature->SetField("ID", road_center->lane_id);
//       feature->SetField("ROAD_ID", lane_group->group_index); // 暂时没有
//       // feature->SetField("FRAME_ID", );  // 暂时没有
//       // feature->SetField("VERSION", );  // 暂时没有
//       feature->SetGeometry(&lineString);

//       if (layer->CreateFeature(feature) != OGRERR_NONE)
//       {
//         std::cerr << "road centers 创建要素失败" << std::endl;
//       }

//       OGRFeature::DestroyFeature(feature);
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }

//   int RoadModelProcBindRelation::export_stop_line_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "stop lines 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("stop_lines", &poSpatialRef, wkbLineString, nullptr);
//     if (!layer)
//     {
//       std::cerr << "stop lines 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加ID字段
//     OGRFieldDefn fieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "stop lines 创建字段失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 停止线TYPE 字段
//     OGRFieldDefn lineTypeFieldDefn("TYPE", OFTInteger);
//     if (layer->CreateField(&lineTypeFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "stop lines 创建字段 TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 color 字段
//     OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
//     if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "stop lines 创建字段 COLOR 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }
//     // 添加 SHAPE_TYPE 字段
//     OGRFieldDefn shapeTypeFieldDefn("SHAPE_TYPE", OFTInteger);
//     if (layer->CreateField(&shapeTypeFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "stop lines 创建字段 SHAPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 LANE_ID 字段
//     OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
//     if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "stop lines 创建字段 LANE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 FRAME_ID 字段
//     OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
//     if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "stop lines 创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 VERSION 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "stop lines 创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }


//     // 写入数据
//     for (const auto &object : session->raw_object_ret_list)
//     {
//       if (object->ele_type == 6)
//       {
//         OGRLineString lineString;
//         for (const auto &point : object->list)
//         {
//           Eigen::Vector3d wgs;
//           session->data_processer->local2wgs(point->pos, wgs);
//           // double lng_gcj2;
//           // double lat_gcj2;
//           // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

//           double lng_gcj2 = wgs.x();
//           double lat_gcj2 = wgs.y();
//           // TODO:qzc gcj01
//           // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//           // TODO:qzc gcj01

//           // Eigen::Vector3d lng_lat_02;
//           // lng_lat_02.x() = lng_gcj2;
//           // lng_lat_02.y() = lat_gcj2;
//           // lng_lat_02.z() = 0;
//           // Eigen::Vector3d utm_02;
//           // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
//           // lineString.addPoint(utm_02.x(), utm_02.y());

//           lineString.addPoint(lng_gcj2, lat_gcj2);

//           // lineString.addPoint(point->pos.x(), point->pos.y());
//         }

//         OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
//         feature->SetField("ID", object->obj_id);
//         feature->SetField("TYPE", std::stoi(object->type));
//         // feature->SetField("LANE_ID", object->lane_id);
//         // feature->SetField("COLOR", object->color);
//         // feature->SetField("SHAPE", object->shape);
//         // feature->SetField("LENGTH", object->length);
//         feature->SetGeometry(&lineString);

//         if (layer->CreateFeature(feature) != OGRERR_NONE)
//         {
//           std::cerr << "stop lines 创建要素失败" << std::endl;
//         }

//         OGRFeature::DestroyFeature(feature);
//       }
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }

//   int RoadModelProcBindRelation::export_cross_walks_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "cross walks 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("cross_walks", &poSpatialRef, wkbPolygon, nullptr);
//     if (!layer)
//     {
//       std::cerr << "cross walks 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加ID字段
//     OGRFieldDefn fieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "cross walks 创建ID字段失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 LANE_ID 字段
//     OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
//     if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "cross walk创建字段 LANE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }
//     // 添加 JUNC_ID 字段
//     OGRFieldDefn juncIdFieldDefn("JUNC_ID", OFTString);
//     if (layer->CreateField(&juncIdFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "cross walk创建字段 JUNC_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 FRAME_ID 字段
//     OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
//     if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "cross walk创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 VERSION 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "cross walk创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }


//     // 写入数据
//     for (const auto &object : session->raw_object_ret_list)
//     {
//       if (object->ele_type == 5)
//       {
//         // 创建一个多边形对象来表示块状区域
//         OGRPolygon polygon;
//         // 创建一个线性环，用于定义多边形的边界
//         OGRLinearRing *ring = new OGRLinearRing();

//         for (const auto &point : object->list)
//         {
//           Eigen::Vector3d wgs;
//           session->data_processer->local2wgs(point->pos, wgs);

//           double lng_gcj2 = wgs.x();
//           double lat_gcj2 = wgs.y();
//           // TODO:qzc gcj01
//           // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//           // TODO:qzc gcj01
//           ring->addPoint(lng_gcj2, lat_gcj2);
//         }

//         // 确保线性环闭合
//         ring->closeRings();

//         // 将线性环添加到多边形中
//         polygon.addRing(ring);

//         // 创建要素并设置字段值
//         OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
//         feature->SetField("ID", object->obj_id);
//         // feature->SetField("LANE_ID", object->lane_id);  //
//         // feature->SetField("INTER_ID", object->obj_bind_intersections);

//         // 设置要素的几何对象为多边形
//         feature->SetGeometry(&polygon);

//         // 将要素添加到图层中
//         if (layer->CreateFeature(feature) != OGRERR_NONE)
//         {
//           std::cerr << "cross walks 创建要素失败" << std::endl;
//         }

//         OGRFeature::DestroyFeature(feature);
//         delete ring;
//       }
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }

//   int RoadModelProcBindRelation::export_road_marks_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "arrow创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建arrow图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("arrow", &poSpatialRef, wkbPolygon, nullptr);
//     if (!layer)
//     {
//       std::cerr << "arrow创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加字段
//     OGRFieldDefn fieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "arrow创建字段失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 箭头TYPE 字段
//     OGRFieldDefn typeFieldDefn("TYPE", OFTString);
//     if (layer->CreateField(&typeFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "arrow 创建字段 TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 LANE_ID 字段
//     OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
//     if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "arrow 创建字段 LANE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 FRAME_ID 字段
//     OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
//     if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "arrow 创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 VERSION 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "arrow 创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

 
//     // 写入数据
//     for (const auto &object : session->raw_object_ret_list)
//     {
//       if (object->ele_type == 3)
//       {
//         // 创建一个多边形对象来表示块状区域
//         OGRPolygon polygon;
//         // 创建一个线性环，用于定义多边形的边界
//         OGRLinearRing *ring = new OGRLinearRing();
//         for (const auto &point : object->list)
//         {
//           Eigen::Vector3d wgs;
//           session->data_processer->local2wgs(point->pos, wgs);
//           // double lng_gcj2;
//           // double lat_gcj2;
//           // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

//           double lng_gcj2 = wgs.x();
//           double lat_gcj2 = wgs.y();
//           // TODO:qzc gcj01
//           // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//           // TODO:qzc gcj01

//           // Eigen::Vector3d lng_lat_02;
//           // lng_lat_02.x() = lng_gcj2;
//           // lng_lat_02.y() = lat_gcj2;
//           // lng_lat_02.z() = 0;
//           // Eigen::Vector3d utm_02;
//           // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
//           // lineString.addPoint(utm_02.x(), utm_02.y());

//           ring->addPoint(lng_gcj2, lat_gcj2);

//           // lineString.addPoint(point->pos.x(), point->pos.y());
//         }
//         // 确保线性环闭合
//         ring->closeRings();
//         // 将线性环添加到多边形中
//         polygon.addRing(ring);

//         OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
//         feature->SetField("ID", object->obj_id);
//         feature->SetField("TYPE", object->type.c_str());
//         // feature->SetField("LANE_ID", object->lane_id);
//         // feature->SetField("LENGTH", object->length);
//         feature->SetGeometry(&polygon);

//         if (layer->CreateFeature(feature) != OGRERR_NONE)
//         {
//           std::cerr << "arrow创建要素失败" << std::endl;
//         }

//         OGRFeature::DestroyFeature(feature);
//       }
//     }

//     // 关闭数据集
//     GDALClose(dataset);

//     return fsdmap::SUCC;
//   }

//   int RoadModelProcBindRelation::export_intersection_to_shp(RoadModelSessionData *session)
//   {
//     // 注册所有的驱动
//     GDALAllRegister();

//     // 创建数据集
//     GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
//     GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
//     if (!dataset)
//     {
//       std::cerr << "junction 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
//       return fsdmap::FAIL;
//       ;
//     }

//     // 创建图层
//     OGRSpatialReference poSpatialRef;
//     poSpatialRef.importFromEPSG(4326);
//     OGRLayer *layer = dataset->CreateLayer("junction", &poSpatialRef, wkbPolygon, nullptr);
//     if (!layer)
//     {
//       std::cerr << "junction 创建图层失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加ID字段
//     OGRFieldDefn fieldDefn("ID", OFTInteger);
//     if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction 创建字段失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加TYPE 字段
//     OGRFieldDefn typeFieldDefn("TYPE", OFTInteger);
//     if (layer->CreateField(&typeFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction 创建字段 TYPE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 IS_EFFECT  字段
//     OGRFieldDefn iseffectFieldDefn("IS_EFFECT", OFTInteger);
//     if (layer->CreateField(&iseffectFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction 创建字段 IS_EFFECT 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 lane road  字段
//     OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
//     if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction 创建字段 LANE_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     OGRFieldDefn inRoadFieldDefn("IN_ROAD", OFTString);
//     if (layer->CreateField(&inRoadFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction 创建字段 IN_ROAD 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }
//     OGRFieldDefn outRoadFieldDefn("OUT_ROAD", OFTString);
//     if (layer->CreateField(&outRoadFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction 创建字段 IN_ROAD 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     OGRFieldDefn inLaneFieldDefn("IN_LANE", OFTString);
//     if (layer->CreateField(&inLaneFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction 创建字段 IN_LANE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     OGRFieldDefn outLaneFieldDefn("OUT_LANE", OFTString);
//     if (layer->CreateField(&outLaneFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction 创建字段 OUT_LANE 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 FRAME_ID 字段
//     OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
//     if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction创建字段 FRAME_ID 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }

//     // 添加 VERSION 字段
//     OGRFieldDefn versionFieldDefn("VERSION", OFTString);
//     if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
//     {
//       std::cerr << "junction 创建字段 VERSION 失败" << std::endl;
//       GDALClose(dataset);
//       return fsdmap::FAIL;
//     }


//     // 写入数据
//     for (const auto &inter : session->raw_intersections)
//     {
//       // 创建一个多边形对象来表示块状区域
//       OGRPolygon polygon;
//       // 创建一个线性环，用于定义多边形的边界
//       OGRLinearRing *ring = new OGRLinearRing();
//       for (auto &pt : inter->point_info)
//       {
//         Eigen::Vector3d wgs;
//         session->data_processer->local2wgs(pt->pos, wgs);
//         // double lng_gcj2;
//         // double lat_gcj2;
//         // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

//         double lng_gcj2 = wgs.x();
//         double lat_gcj2 = wgs.y();
//         // TODO:qzc gcj01
//         // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
//         // TODO:qzc gcj01

//         // Eigen::Vector3d lng_lat_02;
//         // lng_lat_02.x() = lng_gcj2;
//         // lng_lat_02.y() = lat_gcj2;
//         // lng_lat_02.z() = 0;
//         // Eigen::Vector3d utm_02;
//         // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
//         // lineString.addPoint(utm_02.x(), utm_02.y());
//         ring->addPoint(lng_gcj2, lat_gcj2);

//         // lineString.addPoint(pt->pos.x(), pt->pos.y());
//       }
//       // 确保线性环闭合
//       ring->closeRings();
//       polygon.addRing(ring);

//       OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
//       feature->SetField("ID", inter->id);
//       // feature->SetField("LANE_ID", object->lane_id);  //
//       // feature->SetField("IN_LANE", object->lane_id);  //
//       // feature->SetField("OUT_LANE", object->lane_id);  //
//       // feature->SetField("IN_ROAD", object->lane_id);  //
//       // feature->SetField("OUT_ROAD", object->lane_id);  //
//       for(auto &in_lane : inter->in_lanes){
       
//       }
//       feature->SetGeometry(&polygon);

//       if (layer->CreateFeature(feature) != OGRERR_NONE)
//       {
//         std::cerr << "junction 创建要素失败" << std::endl;
//       }

//       OGRFeature::DestroyFeature(feature);
//     }
//     // 关闭数据集
//     GDALClose(dataset);
//     return fsdmap::SUCC;
//   }


}
