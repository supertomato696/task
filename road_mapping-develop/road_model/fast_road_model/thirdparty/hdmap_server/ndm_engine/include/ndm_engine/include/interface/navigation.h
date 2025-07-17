/**
 * @file navigation.h
 * @author Fei Han (fei.han@horizon.ai)
 * @brief
 * @version 0.1
 * @date 2019-12-10
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_NAVIGATION_H_
#define INTERFACE_NAVIGATION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <memory>

#include "../interface/index.h"
#include "../interface/map_component.h"
namespace map_interface {
class Navigation {
 public:
  Navigation() { map_index_ = nullptr; }
  explicit Navigation(map_interface::Index* map_index);
  /**
   * @brief 根据Pose获得当前所在的Lane
   *
   * @param pose
   * @param lane_component
   * @return true
   * @return false
   */
  bool GetLaneByPose(const Eigen::Matrix4d& pose,
                     std::shared_ptr<map_interface::LogicalMapComponent>*
                     lane_component);

  /**
   * @brief 获得当前Lane的前一条Lane
   *
   * @param lane_component
   * @param pre_lane_component
   * @return true
   * @return false
   */
  bool GetPreLane(
        std::shared_ptr<map_interface::LogicalMapComponent> lane_component,
        std::shared_ptr<map_interface::LogicalMapComponent>*
        pre_lane_component);
  /**
   * @brief 获得当前Lane的下一条Lane
   *
   * @param lane_component
   * @param suc_lane_component
   * @return true
   * @return false
   */
  bool GetSucLane(
        std::shared_ptr<map_interface::LogicalMapComponent> lane_component,
        std::shared_ptr<map_interface::LogicalMapComponent>*
        suc_lane_component);
  /**
   * @brief 获得当前Lane在同一Section中的左侧Lane
   *
   * @param lane_component
   * @param section_component
   * @param left_lane_component
   * @return true
   * @return false
   */
  bool GetLeftLane(
        std::shared_ptr<map_interface::LogicalMapComponent> lane_component,
        std::shared_ptr<map_interface::LogicalMapComponent> section_component,
        std::shared_ptr<map_interface::LogicalMapComponent>
        left_lane_component);
  /**
   * @brief 获得当前Lane在同一Section中的右侧Lane
   *
   * @param lane_component
   * @param section_component
   * @param right_lane_component
   * @return true
   * @return false
   */
  bool GetRightLane(
        std::shared_ptr<map_interface::LogicalMapComponent> lane_component,
        std::shared_ptr<map_interface::LogicalMapComponent> section_component,
        std::shared_ptr<map_interface::LogicalMapComponent>
        right_lane_component);
  /**
   * @brief 判断一个点是否在Lane中
   *
   * @param pos
   * @param lane_component
   * @return true
   * @return false
   */
  bool PointInLane(const Eigen::Vector3d& pos,
                   std::shared_ptr<map_interface::LogicalMapComponent>
                   lane_component);
  /**
   * @brief 计算点到车道线的有向距离, 点在车道线左侧, 则距离为负;
   * 点在车道线右侧, 则距离为正;
   *
   * @param point
   * @param laneline_component
   * @param directed_dis
   * @return true
   * @return false
   */
  bool DirectedDisOfPointToLaneLine(
    const Eigen::Vector3d& point,
    std::shared_ptr<map_interface::StaticMapComponent> laneline_component,
    double* directed_dis);

 private:
  map_interface::Index* map_index_;
};

}  // end namespace map_interface
#endif  // INTERFACE_NAVIGATION_H_
