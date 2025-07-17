

#pragma once

#include "road_model_session_data.h"

namespace fsdmap::road_model {

namespace bg = boost::geometry;
using bg_point = bg::model::d2::point_xy<double>;
using segment_t = bg::model::segment<bg_point>;

class RoadModelProcBindRelation :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {

 public:

  RoadModelProcBindRelation() = default;

  ~RoadModelProcBindRelation() override = default;

  const char* name() override {
    return "proc_bind_relation";
  }

  fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session) override;

 private:

  int make_road_lane_center_tree(RoadModelSessionData* session);

  int bind_arrow_to_lane_section(RoadModelSessionData* session);
  int bind_stop_line_to_lane_section(RoadModelSessionData* session);
  int bind_cross_walk_to_lane_section(RoadModelSessionData* session);
  int bind_junction_to_lane_section(RoadModelSessionData* session);
  int bind_intersection_to_lane_section(RoadModelSessionData* session);
  int bind_intersection_to_cross_walk_section(RoadModelSessionData* session);

  int joint_broken_lane_group(RoadModelSessionData* session);

  void assign_group_index(RoadModelSessionData* session, bool flag);

  // todo::暂时修复车道方向计算，后续前序结果优化后，该功能可下线
  void fix_lane_direction(RoadModelSessionData* session);

  void build_broken_group_pair(RoadModelSessionData* session);
  void cal_lane_topology(const RoadLinePointInfo* pre_pt, const RoadLinePointInfo* post_pt,
                         const int group_index, int* match_group_index, bool* is_isolate,
                         double* min_match_distance);

  void build_new_group_boundary_info(RoadModelSessionData* session);
  void get_group_central_boundary(const RoadLaneGroupInfo* group,
                                  std::vector<RoadLaneBoundaryInfo*>* central_boundary_v);
  void get_group_side_boundary(const RoadLaneGroupInfo* group,
                               std::vector<RoadLaneBoundaryInfo*>* side_boundary_v);

  /**
   * @brief 计算前序与后续车道组之间，boundary的匹配关系
   * 1. 优先查找近邻
   * 2. 转向角度小于30
   * 3. 对配对结果做顺序优化，避免倾斜路口的影响
   * */
  void cal_boundary_connect(const std::vector<RoadLaneBoundaryInfo*>& pre_total_boundary_v,
                            const std::vector<RoadLaneBoundaryInfo*>& post_total_boundary_v,
                            const std::vector<segment_t>& lane_segment,
                            std::vector<int>* boundary_pair_result,
                            bool is_cal_central = true);

  /**
 * @brief 生成新车道组的车道线
 * @param pre_total_boundary_v 前序的车道线
 * @param new_boundary 新生产的车道线，数量一定和前序保持一致
 * @param pair_data 前序车道线对应的后继车道线的index
 * */
  void build_new_boundary(const RoadLaneGroupInfo* post_group,
                          const std::vector<RoadLaneBoundaryInfo*>& pre_boundary_v,
                          const std::vector<RoadLaneBoundaryInfo*>& post_boundary_v,
                          const std::vector<int>& pair_data,
                          RoadModelSessionData* session,
                          std::vector<RoadLaneBoundaryInfo*>* boundary_result);

  void build_new_group(RoadModelSessionData* session);
  template<class T>
  void build_topology(T* new_ptr, T* pre_ptr, T* post_ptr) {
      // 确保传入的指针不为空
      CHECK_NOTNULL(new_ptr);
      CHECK_NOTNULL(pre_ptr);

      // 如果后继节点 post_ptr 不为空，则需要更新后继节点的前驱关系
      if (post_ptr != nullptr) {
          // 设置 new_ptr 的下一个节点为 post_ptr
          new_ptr->context.set_next(post_ptr);
          // 设置 post_ptr 的上一个节点为 new_ptr
          post_ptr->context.set_prev(new_ptr);

          // 遍历 post_ptr 的所有前驱节点，如果 pre_ptr 是 post_ptr 的前驱，则移除它
          for (int i = 0; i < post_ptr->context.all_prev.size(); ++i) {
              if (post_ptr->context.all_prev[i].src == pre_ptr) {
                  // 从 post_ptr 的前驱列表中删除 pre_ptr，避免无效链接
                  VEC_ERASE(post_ptr->context.all_prev, i)
                  break;  // 找到并删除后退出循环
              }
          }
      }

      // 设置 new_ptr 的前驱节点为 pre_ptr
      new_ptr->context.set_prev(pre_ptr);
      // 设置 pre_ptr 的下一个节点为 new_ptr
      pre_ptr->context.set_next(new_ptr);

      // 遍历 pre_ptr 的所有后继节点，如果 post_ptr 是 pre_ptr 的后继，则移除它
      for (int i = 0; i < pre_ptr->context.all_next.size(); ++i) {
          if (pre_ptr->context.all_next[i].src == post_ptr) {
              // 从 pre_ptr 的后继列表中删除 post_ptr，避免无效链接
              VEC_ERASE(pre_ptr->context.all_next, i)
              break;  // 找到并删除后退出循环
          }
      }
  }

  /**
   * @brief 修复车道数量相同的车道组打断
   * */
  int joint_invalid_topology(RoadModelSessionData* session);

  void cal_group_topology();

  void pick_invalid_topology(RoadModelSessionData* session);

  inline bool is_connected(const RoadLinePointInfo* pre_pt, const RoadLinePointInfo* post_pt) {
    const double pt_distance = alg::calc_dis(post_pt->pos, pre_pt->pos);
    return pt_distance < 1e-1;
  }

  // todo::优化实现
  bool is_intersect(const segment_t& l1, const segment_t& l2);

  void merge_invalid_topology(RoadModelSessionData* session);
  template<class T>
  void merge_topology(T* pre_ptr, T* post_ptr, T* next_ptr) {
    CHECK_NOTNULL(pre_ptr);
    CHECK_NOTNULL(post_ptr);
    // CHECK_NOTNULL(next_ptr);

    // pre_ptr->context.set_next(next_ptr);
    // for (int i = 0; i < pre_ptr->context.all_next.size(); ++i) {
    //   if (pre_ptr->context.all_next[i].src == post_ptr) {
    //     VEC_ERASE(pre_ptr->context.all_next, i)
    //     break;
    //   }
    // }

  if(next_ptr != nullptr){
    next_ptr->context.set_prev(pre_ptr);
    for (int i = 0; i < next_ptr->context.all_prev.size(); ++i) {
      if (next_ptr->context.all_prev[i].src == post_ptr) {
        VEC_ERASE(next_ptr->context.all_prev, i)
        break;
      }
    }
    pre_ptr->context.set_next(next_ptr);
  }

  for (int i = 0; i < pre_ptr->context.all_next.size(); ++i) {
      if (pre_ptr->context.all_next[i].src == post_ptr) {
        VEC_ERASE(pre_ptr->context.all_next, i)
        break;
      }
    }  
    
  }

  void merge_boundary(RoadLaneGroupInfo* pre_group,
                      RoadLaneBoundaryInfo* pre_boundary,
                      RoadLaneBoundaryInfo* post_boundary);

  void delete_merged_group(RoadModelSessionData* session);

  int save_joint_debug_info(RoadModelSessionData* session);

  int save_arrow_binding_debug_info(RoadModelSessionData* session);
  int save_stop_line_binding_debug_info(RoadModelSessionData* session);
  int save_cross_walk_binding_debug_info(RoadModelSessionData* session);
  int save_intersection_binding_debug_info(RoadModelSessionData* session);
  int save_intersection_in_out_binding_debug_info(RoadModelSessionData* session);
  int save_intersection_binding_cross_wallk_debug_info(RoadModelSessionData* session);

  //路口边界属性赋值
  int attribute_for_lukoubd(RoadModelSessionData* session);
  
  //将路口边界内的短的道路边界删除
  int delete_rbsegment_contined_lukoubd(RoadModelSessionData* session);

  //将不可通行区域绑定到路口边界上
  int bind_areas_to_junction(RoadModelSessionData* session);

   //将路口边界内的短的道路边界删除
  int delete_lanegroup_contined_lukoubd_withsuccs(RoadModelSessionData* session);
  
  bool junction_contains_lanegroup(const RoadLaneGroupInfo* road_lane_group, JunctionInfo* junction);

  bool delete_prev_lane_groups(RoadLaneGroupInfo* to_be_deleted_group);
  bool delete_next_lane_groups(RoadLaneGroupInfo* to_be_deleted_group);


  // int export_to_shape_file(RoadModelSessionData* session);
  // int export_lane_boundary_to_shp(RoadModelSessionData* session);
  // int export_lane_centers_to_shp(RoadModelSessionData* session);
  // int export_road_boundary_to_shp(RoadModelSessionData* session);
  // int export_road_to_shp(RoadModelSessionData* session);
  // int export_lane_to_shp(RoadModelSessionData* session);
  // int export_lane_adas_to_shp(RoadModelSessionData* session);
  // int export_road_centers_to_shp(RoadModelSessionData* session);
  // int export_stop_line_to_shp(RoadModelSessionData* session);
  // int export_cross_walks_to_shp(RoadModelSessionData* session);
  // int export_road_marks_to_shp(RoadModelSessionData* session);
  // int export_intersection_to_shp(RoadModelSessionData* session);

 private:
  std::vector<RoadLaneGroupInfo*> pre_group_v_;

  std::vector<RoadLaneGroupInfo*> post_group_v_;

  // 从前向后找到的断裂车道组的配对关系
  std::map<int, std::set<int>> broken_group_pre_2_post_set_;
  std::set<std::pair<int, int>> broken_group_found_from_post_;

  /**
   * @brief 新车道组的中心线
   * @param std::pair<int, int> pre车道组与post车道组的index信息
   * @param std::vector<RoadLaneBoundaryInfo*>> 新的车道中心线信息
   * */
  std::map<std::pair<int, int>, std::vector<RoadLaneBoundaryInfo*>> new_group_2_central_boundary_;
  std::map<std::pair<int, int>, std::vector<int>> new_group_2_lane_pair_; // 车道配对信息
  std::map<std::pair<int, int>, std::vector<RoadLaneBoundaryInfo*>> new_group_2_side_boundary_; // 左右车道线

  RTreeProxy<RoadLinePointInfo*, float, 2> lane_pt_tree_;

  // 前车道组与后车道组的拓扑
  std::map<int, std::set<int>> pre_group_2_post_set_;
  // 后车道组与前车道组的拓扑
  std::map<int, std::set<int>> post_group_2_pre_set_;

  std::map<int, int> invalid_pre_group_2_post_group_;
  std::map<int, int> invalid_post_group_2_pre_group_;
};

}
