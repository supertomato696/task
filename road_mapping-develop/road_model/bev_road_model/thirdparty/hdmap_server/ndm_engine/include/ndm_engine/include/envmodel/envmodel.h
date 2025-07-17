/**
 * @file comm.h
 * @author Zeng Siyu(siyu.zeng@horizon.ai)
 * @brief  environment model Interface
 * @version 3.2
 * @date 2020-07-30
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef ENVMODEL_ENVMODEL_H_
#define ENVMODEL_ENVMODEL_H_

#include "../interface/map_engine.h"

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <glog/logging.h>

namespace map_interface {

class EnvModel
{
public:
  EnvModel();
  ~EnvModel() {
    // delete engine_;
  }

  void SetEngine(map_interface::MapEngine* engine);
  void SetEnvMapDist(double front_dist, double back_dist);

  bool GetCurEnvlane(
    std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &stubmap);

  bool GetEnvMap(
    std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &stubmap);

private:
  map_interface::MapEngine* engine_;  //引擎

private:
  /// 环境模型的长度约束,依次为：车辆前方距离范围，车辆后方距离范围
  std::vector<double> map_radius_;
};

}  // namespace map_interface

#endif  // ENVMODEL_ENVMODEL_H_
