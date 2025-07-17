/**
 * @file sampling.h
 * @author Zeng Siyu(siyu.zeng@horizon.ai), Han Fei (fei.han@horizon.ai)
 * @brief  Sampling Interface
 * @version 3.2
 * @date 2020-06-29
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef INTERFACE_SAMPLING_H_
#define INTERFACE_SAMPLING_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <vector>
#include <memory>

#include "../interface/map_component.h"

namespace map_interface {
/**
 * @brief 上采样，两点间按照设定的距离进行采样
 * @param p1 第一个点
 * @param p2 第二个点
 * @param filling 采样结果
 * @param dist 采样距离
 */
void PointsUpSampling(Eigen::Vector3d p1,
                      Eigen::Vector3d p2,
                      std::vector<Eigen::Vector3d> *filling,
                      float dist);

/**
 * @brief 上采样，根据点的顺序，两两间按照设定的距离进行采样
 * @param contour 点集
 * @param filling 采样结果
 * @param dist 采样距离
 */
void PointsUpSampling(std::vector<Eigen::Vector3d> contour,
                      std::vector<Eigen::Vector3d> *filling,
                      float dist);

/**
 * @brief 获取控制点，从component中获取其几何控制点
 * @param component 获取对象
 * @return vector 返回点集
 */
std::vector<Eigen::Vector3d> GetPoints(
      std::shared_ptr<map_interface::StaticMapComponent> component);
}  // namespace map_interface
#endif  // INTERFACE_SAMPLING_H_
