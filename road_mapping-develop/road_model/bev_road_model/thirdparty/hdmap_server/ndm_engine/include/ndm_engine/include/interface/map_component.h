/**
 * @file map_component.h
 * @author Han Fei (fei.han@horizon.ai)
 * @brief  Map component class
 * @version 0.1
 * @date 2019-08-28
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_MAP_COMPONENT_H_
#define INTERFACE_MAP_COMPONENT_H_

#include <limits>
#include <string>
#include <vector>
#include <memory>
#include <glog/logging.h>

#include "../../proto/ndm.pb.h"
#include "../interface/geometry_3d.h"

namespace map_interface {
#define V_OFFSET 0.1
/**
 * @brief 定义Component类型
 *
 */
enum HDMapComponentType {
  Physical_Layer = 0,
  Physical_TrafficSign = 1,       // 交通指示牌
  Physical_TrafficLight = 2,      // 交通信号灯
  Physical_LaneLine = 3,          // 车道线
  Physical_RoadBorder = 4,        // 道路边界
  Physical_Guardrail = 5,         // 护栏、混凝土护栏、围栏、墙、遮棚
  Physical_LaneLineUnknown = 6,   // 无法判断类型的纵向标线
  Physical_Pole = 7,              // 竖杆
  Physical_CrossWalk = 8,         // 斑马线
  Physical_LaneMarking = 9,       // 路面标志
  Physical_Stopline = 10,         // 停止线
  Physical_Speedbump = 11,        // 减速带
  Physical_ParkingSlot = 12,      // 停车位
  Physical_Board = 13,            // 大的交通指示牌

  Logical_Layer = 14,
  Logical_Lane = 15,              // 车道
  Logical_ParkingSpace = 16,      // 停车区域
  Logical_Section = 17,           //
  Logical_Junction = 18,          // 路口
  Virtual_LaneLine = 19,          // 虚拟车道线
  Virtual_DriveLine = 20,         // 虚拟车道线
  Virtual_Lane = 21,              // 虚拟车道

  TopoLogical_Layer = 22,
  TopoLogical_Area = 23,          // 区域
  TopoLogical_Parking = 24,       // 停车区域
  TopoLogical_Road = 25,          // 道路
  TopoLogical_Tile = 26,          //
  TopoLogical_Node = 27,          //
  TopoLogical_Edge = 28,          //

  Dynamic_Layer = 40,             // 动态层
  Dynamic_StaticODD = 41,         // ODD信息
  Dynamic_TrafficEvent = 42,      // 交通事件
  Dynamic_TrafficFlow = 43,       // 交通流
  UNKNOWN = 99
};
/**
 * @brief 根据指定的componentType，获得对应的类型名称
 *
 * @param hlabel    component类型
 * @return std::string  类型名称
 */
std::string HDMapComponentToString(HDMapComponentType hlabel);

/**
 * @brief 静态地图元素，主要存储物理层的元素，包含具体的坐标形态信息
 *
 */
class StaticMapComponent {
 public:
  explicit StaticMapComponent(bool delete_proto_ptr = false);
  virtual ~StaticMapComponent() {
    if (delete_proto_ptr_ && protoMapComponent != nullptr) {
      DeleteProtoPtr();
    }
    // if (cpolygon != nullptr) {
    //   delete cpolygon;
    // }
    // if (ccylinder != nullptr) {
    //   delete ccylinder;
    // }
    // if (ccurveLine.size() != 0 && delete_curveline_ptr) {
    //   for (int i = 0; i < ccurveLine.size(); i++) {
    //     delete ccurveLine[i];
    //   }
    // }
  }
  /**
   * @brief 输出component的属性，包括类别，中心点、最大和最小顶点
   *
   */
  void ShowDetail();

  HDMapComponentType GetComponentType(StaticMapComponent meta);
  /**
   * @brief 调整最大和最小顶点的坐标
   *
   */
  void AdjustVertex();
  /**
   * @brief 得到该component对应的protoMap要素的指针
   *
   * @return void*
   */
  void* GetProtoMapComponent();

 public:
  HDMapComponentType component_type;  /// component类型

  std::string id;  /// component 对应要素的id

  Eigen::Vector3d position;       /// component的位置
  Eigen::Quaterniond quaternion;  /// component的朝向

  ndm_proto::Point center_position;  /// component中心点，除车道线外都有中心点

  ndm_proto::Point max_vertex;  /// component对应BoundingBox的最大顶点
  ndm_proto::Point min_vertex;  /// component对应BoundingBox的最小顶点

  // ndm_proto::Polygon* cpolygon;  /// component对应的多边形几何元素指针
  // ndm_proto::Cylinder* ccylinder;  /// component对应的圆柱几何元素指针
  /// component对应的多边形几何元素指针
  std::shared_ptr<ndm_proto::Polygon> cpolygon;
  /// component对应的圆柱几何元素指针
  std::shared_ptr<ndm_proto::Cylinder> ccylinder;
  // component对应的曲线几何元素指针
  std::vector<std::shared_ptr<ndm_proto::CurveLine>> ccurveLine;

  void* protoMapComponent;  /// component对应的protoMap要素的指针
  bool delete_proto_ptr_;
  bool delete_curveline_ptr;   /// 删除ccurveLine中对应的proto要素指针

  // component对应的有向包围盒
  // 除逻辑层在构建mapComponent时需要计算OBB外
  // 物理层的mapComponent只在要用的时候再计算
  map_interface::OBB obb_;
  map_interface::AABB aabb;     /// component对应的AA Bounding Box
  // 对于车道线保存它的前驱、后继，左右邻接
  // 对于逻辑要素，存储可通行的逻辑元素前、后、左、右的section, Lane
  std::vector<std::string> pred_ids;
  std::vector<std::string> succ_ids;
  std::vector<std::string> left_ids;
  std::vector<std::string> right_ids;

  uint64_t timestamp;
  bool is_virtual;
  double length_;

 private:
  void DeleteProtoPtr();
};

/**
     * @brief 动态地图元素，主要存储物理层动态元素，包含具体的坐标形态信息
     *
     */
class DynamicMapComponent : public StaticMapComponent {
 public:
  explicit DynamicMapComponent(bool delete_proto_ptr = false)
      : StaticMapComponent(delete_proto_ptr) {}
  ~DynamicMapComponent() {}
};

/**
 * @brief 逻辑层，更多的是记录逻辑连接关系
 *
 */
class LogicalMapComponent : public StaticMapComponent {
 public:
  explicit LogicalMapComponent(bool delete_proto_ptr = false)
      : StaticMapComponent(delete_proto_ptr) {}
  ~LogicalMapComponent() {
    // if (cpolygon != nullptr) delete cpolygon;
  }

  void updateVertex(const std::shared_ptr<StaticMapComponent> &component);

  void computeOBB();

 public:
  // vec: 存储section内部的mapcomponent
  std::vector<std::shared_ptr<StaticMapComponent>> components_inphysical;
};

/**
 * @brief 拓扑层，更多的是记录片状区域信息
 *
 */
class TopoLogicalMapComponent : public StaticMapComponent {
 public:
  explicit TopoLogicalMapComponent(bool delete_proto_ptr = false)
      : StaticMapComponent(delete_proto_ptr) {}
  ~TopoLogicalMapComponent() {}
};

}  // end namespace map_interface
#endif  // INTERFACE_MAP_COMPONENT_H_
