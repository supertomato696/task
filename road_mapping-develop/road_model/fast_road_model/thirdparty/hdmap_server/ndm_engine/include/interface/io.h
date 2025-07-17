/**
 * @file io.h
 * @author Han Fei
 * @brief  Map IO Interface
 * @version 0.1
 * @date 2019-08-28
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_IO_H_
#define INTERFACE_IO_H_

#include <glog/logging.h>
#include <memory>
#include <string>
#include <vector>

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <cstdio>
#include <fstream>
#include <iostream>

#include "../../proto/ndm.pb.h"
#include "../interface/index.h"
#include "../interface/util.h"

using ndm_proto::MapEnvMsg;
using std::string;

namespace map_interface {
class Index;

class IO {
 public:
  IO() { mapIndex_ = new map_interface::Index(); }
  explicit IO(ndm_proto::MapEnvMsg *map) : proto_map_(map) {
    mapIndex_ = new map_interface::Index();
  }
  ~IO() { delete mapIndex_; }
  bool MapInit();
  bool mapPhysicalInit(bool merge_pts = false);

  bool mapLogicalInit(bool merge_pts = false);

  bool mapTopologicalInit();

  bool checkIsMapInit();

  // std::vector<StaticMapComponent *> getMapComponent();
  std::vector<std::shared_ptr<StaticMapComponent>> getMapComponent();
  std::vector<std::shared_ptr<StaticMapComponent>> getPhysicalComponent();
  std::vector<std::shared_ptr<StaticMapComponent>> getLogicalComponent();

  void AddMapComponent(std::shared_ptr<StaticMapComponent> component);
  void AddMapComponents(
      std::vector<std::shared_ptr<StaticMapComponent>> components);

  void ClearMapComponent();

  void WriteComponentToProto(std::string save_path,
                             map_interface::Index *map_index);

  bool LoadFromTextFile(const std::string &file_path);
  bool SaveToTextFile(const std::string &save_path);
  bool SaveToTextFile(const std::string &save_path, ndm_proto::MapEnvMsg *map);

  bool LoadFromBinaryFile(const std::string &file_path);
  bool SaveToBinaryFile(const std::string &save_path);
  bool SaveToBinaryFile(const std::string &save_path,
                        ndm_proto::MapEnvMsg *map);

  std::string ToString();

 private:
  ndm_proto::MapEnvMsg *proto_map_;

  map_interface::Index *mapIndex_;

  // 状态值：记录地图是否加载
  bool map_state_loading_;

  // 状态值：记录地图是否初始化成功，已mapcomponent形式存储
  bool map_state_init_;

  int tag_physical_cmp;  // 记录最后一个物理层cmp的标志位
  int tag_logical_cmp;  // 记录最后一个物理层cmp的标志位

  std::vector<std::shared_ptr<StaticMapComponent>> components_map;
};
}  // end namespace map_interface
#endif  // INTERFACE_IO_H_
