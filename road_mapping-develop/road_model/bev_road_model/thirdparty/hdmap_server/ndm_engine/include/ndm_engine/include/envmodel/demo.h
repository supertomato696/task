/**
 * @file comm.h
 * @author Zeng Siyu(siyu.zeng@horizon.ai)
 * @brief  environment model demo
 * @version 3.2
 * @date 2020-07-30
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef ENVMODEL_DEMO_H_
#define ENVMODEL_DEMO_H_

#include "../interface/map_engine.h"
#include "../envmodel/envmodel.h"
#include "../../proto/ndm.pb.h"

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <glog/logging.h>

namespace map_interface {

/**
 * @brief get environment driveline msg
 * 
 * @param stubmap 输入的compoent
 * @param str     返回的str结果
 * @param index   经过ID初始化的索引
 * @return component 拆分后的component
 */
bool GetEnvDrivelineStr(
  const std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
  &stubmap,
  std::vector<std::string> &str,
  map_interface::Index *index);

bool GetEnvObjStr(
  const std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
  &stubmap,
  std::vector<std::string> &str,
  map_interface::Index *index);

bool UnPackRoadMsg(
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &roads,
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &objs,
  map_interface::Index* index);

bool UnPackRoadMsg(
  std::shared_ptr<map_interface::StaticMapComponent> &road,
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &objs,
  map_interface::Index* index);

bool GetTrafficSignInfo(
  Eigen::Vector3d &point,
  std::shared_ptr<map_interface::StaticMapComponent> obj,
  float &value,
  ndm_proto::TrafficSignType_Type &type);

bool GetTrafficLightInfo(
  Eigen::Vector3d &point,
  std::shared_ptr<map_interface::StaticMapComponent> obj,
  ndm_proto::TrafficLightState_State &state,
  ndm_proto::TrafficLightType_Type &type);

bool GetTrafficBoardInfo(
  Eigen::Vector3d &point,
  std::shared_ptr<map_interface::StaticMapComponent> obj,
  std::string &str);

bool GetLaneMarkingInfo(
  Eigen::Vector3d &point,
  std::shared_ptr<map_interface::StaticMapComponent> obj,
  ndm_proto::LaneMarkingType_Type &type,
  float &value);

std::string Point2Str(Eigen::Vector3d &point);
}  // namespace map_interface

#endif  // ENVMODEL_DEMO_H_
