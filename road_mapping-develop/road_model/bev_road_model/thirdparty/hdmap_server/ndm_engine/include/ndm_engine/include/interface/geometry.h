/**
 * @file sampling.cc
 * @author Zeng Siyu(siyu.zeng@horizon.ai), Han Fei (fei.han@horizon.ai)
 * @brief  Sampling Interface
 * @version 3.2
 * @date 2020-06-29
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef INTERFACE_GEOMETRY_H_
#define INTERFACE_GEOMETRY_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>

#include "../interface/index.h"
#include "../interface/map_component.h"
#include "../interface/util.h"

namespace map_interface {
/**
 * @brief 判断一个点是否在Lane中
 *
 * @param pos
 * @param lane_component
 * @return true
 * @return false
 */
bool PointInLane(
    const Eigen::Vector3d& pos,
    std::shared_ptr<map_interface::StaticMapComponent> lane_component,
    map_interface::Index* index);

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

/**
 * @brief 获取componet的长度
 * @component 目标对象
 * @return double 长度值，单位：米
 */
double GetLength(std::shared_ptr<map_interface::StaticMapComponent> component,
                 map_interface::Index* index);

/**
* @brief 判断当前点是否在component之中
* @param point 目标点
* @param component 地图元素，要求地图元素具有polygon边界
* @param dist 距离，当前点到component的距离
* @return true 目标点在component之中
*/
bool InComponent(
    const Eigen::Vector3d point,
    const std::shared_ptr<map_interface::StaticMapComponent> component,
    double& dist);
/**
 * @brief 拟合车道线
 *
 * @param laneline_component
 * @return true
 * @return false
 */
bool FitCubicCurve(
    std::shared_ptr<map_interface::StaticMapComponent> laneline_component);
/**
 * @brief Get the Lane Curvature
 *
 * @param lane_comp     车道
 * @param index         地图索引
 * @param pos           位置
 * @param curvature     车道在pos处的曲率
 * @return true         计算成功
 * @return false        计算失败
 */
bool GetLaneCurvature(
    std::shared_ptr<map_interface::StaticMapComponent> lane_comp,
    map_interface::Index* index, const Eigen::Vector3d& pos, double& curvature);
/**
 * @brief Get the Lane Slope
 *
 * @param lane_comp 车道
 * @param index     地图索引
 * @param pos       位置
 * @param slope     车道在当前位置的坡度
 * @param dist_th   计算坡度时，水平距离阈值
 * @return true     计算成功
 * @return false    计算失败
 */
bool GetLaneSlope(std::shared_ptr<map_interface::StaticMapComponent> lane_comp,
                  map_interface::Index* index, const Eigen::Vector3d& pos,
                  double& slope, const double& dist_th = 50.0);

/**
 * @brief Get the offset of pose in Lane/Section
 *
 * @param pos       位置，与地图坐标系一致
 * @param component Lane/Section
 * @param index     地图索引
 * @param offset    当前位置所在Lane/Section相对起点的偏移量
 * @return false    计算失败
 */
bool GetOffset(const Eigen::Vector3d& pos,
               std::shared_ptr<map_interface::StaticMapComponent> component,
               map_interface::Index* index, double &offset);

/**
 * @brief Get the offset of pose in reference line
 *
 * @param pos       位置，与地图坐标系一致
 * @param component reference line, 多为driveline，折线段不区分虚实端点
 * @param index     地图索引
 * @param offset    当前位置所在ref line起点的偏移量
 * @return false    计算失败
 */
bool GetOffsetInRefLine(const Eigen::Vector3d& pos,
               std::shared_ptr<map_interface::StaticMapComponent> refline,
               map_interface::Index* index, double &offset);
}  // end namespace map_interface
#endif  // INTERFACE_GEOMETRY_H_
