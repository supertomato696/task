/**
 * @file position_recognition.h
 * @author Zeng Siyu(siyu.zeng@horizon.ai)
 * @brief  Position Recognition Interface
 * @version 3.2
 * @date 2020-11-04
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef INTERFACE_POSITION_RECOGNITION_H_
#define INTERFACE_POSITION_RECOGNITION_H_

#include "../interface/map_component.h"
#include "../interface/geometry.h"
#include "../interface/index.h"
#include "../interface/util.h"

#include <memory>
#include <vector>
namespace map_interface {
class PositionRecognition {
public:
  PositionRecognition();
  ~PositionRecognition();

  bool UpdateFrame(Eigen::Matrix4d pose);
  bool UpdateCandidates(Eigen::Matrix4d pose);
  bool UpdateVotes(Eigen::Matrix4d pose);
  void FeedIndex(map_interface::Index* index);
  void Reset();
private:
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> candidates_;
  std::shared_ptr<map_interface::StaticMapComponent> winner_;
  std::vector<double> votes_;
  map_interface::Index* index_;

  /// 记录地图范围的距离约束,大于该阈值则认为超出地图范围
  double map_range_dist_;
  /// 记录位置识别时地图初始化搜索范围，采用的是半径搜索形式
  double map_range_init_;
  /// 记录初始化成功的阈值，与当前votes值对应
  double init_threshold_;
};
}  // end namespace map_interface
#endif  // INTERFACE_POSITION_RECOGNITION_H_
