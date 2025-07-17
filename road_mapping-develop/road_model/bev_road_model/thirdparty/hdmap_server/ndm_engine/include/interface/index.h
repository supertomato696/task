/**
 * @file index.h
 * @author Han Fei (fei.han@horizon.ai)
 * @brief  Map Index Interface, based on R-Tree
 * @version 0.1
 * @date 2019-08-28
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_INDEX_H_
#define INTERFACE_INDEX_H_
#include "../interface/RTree.h"
#include "../interface/geometry_2d.h"
#include "../interface/map_component.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace map_interface {
class Index {
 public:
  /**
   * @brief 构造MapIndex对象
   *
   */
  Index();
  /**
   * @brief 构造MapIndex对象，并对传入的protoMap通过调用Init()进行初始化
   *
   * @param map
   */
  // explicit Index(ndm_proto::MapEnvMsg *map) : proto_map(map) {}
  /**
   * @brief Destroy the Map Index object
   *
   */
  ~Index() {}

  /**
  * @brief 初始化物理层的空间检索
  *
  * @return true     建立空间检索成功
  * @return false    建立空间检索失败
  */
  bool InitPhysicalSpatialIndex(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components);

  /**
 * @brief 初始化逻辑层的空间检索
 *
 * @return true     建立空间检索成功
 * @return false    建立空间检索失败
 */
  bool InitLogicalSpatialIndex(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components);

  /**
   * @brief 初始化ID检索
   * @detail ID不区分层级检索
   *
   * @return true 建立ID检索成功
   * @return false 建立ID检索失败
   */
  bool InitIDIndex(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components);

  /**
   * @brief 设置地图分区信息
   * @detail 设置Zone和地图的offset
   *
   * @return true 成功
   * @return false 失败
   */
  void SetZoneAndOffset(std::string zone, const Eigen::Vector3d &offset);

  /**
     * @brief 获取地图分区信息
     * @detail 获取Zone和地图的offset
     *
     * @return void
     */
  void GetZoneAndOffset(std::string &zone, Eigen::Vector3d &offset);

  /**
   * @brief 初始化ID Node-link 检索
   * @detail component存的是node，key则是其指向edge的id
   *
   * @return true 建立ID检索成功
   * @return false 建立ID检索失败
   */
  bool InitIDInLink(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components);

  /**
   * @brief 重置物理层的空间检索
   *
   * @return true     重置空间检索成功
   * @return false    重置空间检索失败
   */
  bool ResetPhysicalSpatialIndex();

  /**
 * @brief 重置逻辑层的空间检索
 *
 * @return true     重置空间检索成功
 * @return false    重置空间检索失败
 */
  bool ResetLogicalSpatialIndex();

  /**
   * @brief 重置ID检索
   * @detail ID不区分层级检索
   *
   * @return true 重置ID检索成功
   * @return false 重置ID检索失败
   */
  bool ResetIDIndex();

  /**
   * @brief 根据传入的位姿以及设置的搜索范围（通过SetBoundingBoxRange设置），
   * 进行空进地图物理层检索。注意这里的位姿为车体坐标系（x轴-前;y轴-左;z轴-上）
   * @detail 搜索区域为OOBB
   * @param pose  位姿
   * @param submap  检索结果，若检索失败，则返回为空
   * @return true 检索成功
   * @return false 检索失败
   */
  bool PhysicalSpatialIndex(
      const Eigen::Matrix4d &pose, std::vector<float> bbx_range,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *submap);

  /**
   * @brief 根据传入的位姿以及设置的搜索范围（通过SetBoundingBoxRange设置），
   * 进行空进地图物理层检索。注意这里的位姿为车体坐标系（x轴-前;y轴-左;z轴-上）
   * @detail 搜索区域为OOBB
   * @param pose  位姿
   * @param submap  检索结果，若检索失败，则返回为空
   * @return true 检索成功
   * @return false 检索失败
   */
  bool PhysicalSpatialIndex(
      const Eigen::Matrix4d &pose,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *submap);

  /**
   * @brief 根据传入的位姿以及设置的搜索范围（通过SetBoundingBoxRange设置），
   * 进行空间地图逻辑层检索。注意这里的位姿为车体坐标系（x轴-前;y轴-左;z轴-上）
   * @detail 搜索区域为OOBB
   * @param pose  位姿
   * @param submap  检索结果，若检索失败，则返回为空
   * @return true 检索成功
   * @return false 检索失败
   */
  bool LogicalSpatialIndex(
      const Eigen::Matrix4d &pose, std::vector<float> bbx_range,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *submap);

  /**
   * @brief 根据传入的位姿以及设置的搜索范围（通过SetBoundingBoxRange设置），
   * 进行空间地图逻辑层检索。注意这里的位姿为车体坐标系（x轴-前;y轴-左;z轴-上）
   * @detail 搜索区域为OOBB
   * @param pose  位姿
   * @param submap  检索结果，若检索失败，则返回为空
   * @return true 检索成功
   * @return false 检索失败
   */
  bool LogicalSpatialIndex(
      const Eigen::Matrix4d &pose,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *submap);
  /**
   * @brief 根据传入的位姿以及设置的搜索范围（通过SetBoundingBoxRange设置），
   * 进行空进地图物理层检索。注意这里的点为AABB的中心点,坐标系：前-北，右-东，上-天
   * @detail 搜索区域为AABB
   * @param pose  位姿
   * @param submap  检索结果，若检索失败，则返回为空
   * @return true 检索成功
   * @return false 检索失败
   */
  bool PhysicalSpatialIndex(
      const Eigen::Vector3d &pose, std::vector<float> bbx_range,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *submap);

  /**
   * @brief 根据传入的位姿以及设置的搜索范围（通过SetBoundingBoxRange设置），
   * 进行空间地图逻辑层检索。注意这里的点为AABB的中心点,坐标系：前-北，右-东，上-天
   * @detail 搜索区域为AABB
   * @param pose  位姿
   * @param submap  检索结果，若检索失败，则返回为空
   * @return true 检索成功
   * @return false 检索失败
   */
  bool LogicalSpatialIndex(
      const Eigen::Vector3d &pose, std::vector<float> bbx_range,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *submap);

  /**
   * @brief 物理层单点搜索
   * @param pose  单点（ｘ, ｙ, ｚ)
   * @param submap  检索结果，若检索失败，则返回为空
   * @return true 检索成功
   * @return false 检索失败
   */
  bool PhysicalSpatialIndex(
      const Eigen::Vector3d &pose,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *submap);

  /**
   * @brief 逻辑层单点搜索
   * @param pose  单点（ｘ, ｙ, ｚ)
   * @param submap  检索结果，若检索失败，则返回为空
   * @return true 检索成功
   * @return false 检索失败
   */
  bool LogicalSpatialIndex(
      const Eigen::Vector3d &pose,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *submap);

  /**
   * @brief 根据id进行地图检索
   *
   * @param id 要素id
   * @param component 返回检索结果，若检索失败，则为null
   * @return true 检索成功
   * @return false 检索失败
   */
  bool IDIndex(const std::string &id,
               std::shared_ptr<map_interface::StaticMapComponent> *component);

  /**
   * @brief 根据id进行地图检索
   *
   * @param id 要素id
   * @param component 返回检索结果，若检索失败，则为null
   * @return true 检索成功
   * @return false 检索失败
   */
  bool IDInLinkIndex(
      const std::string &id,
      std::shared_ptr<map_interface::StaticMapComponent> *component);

  /**
   * @brief 添加地图要素
   * 添加的地图元素将同步添加到检索库中（ID/Spatial)
   * @param component
   * @return true       添加成功
   * @return false      添加失败
   */
  bool Add(std::shared_ptr<map_interface::StaticMapComponent> component);

  /**
   * @brief 删除地图要素
   *
   * @param component ID/Spatial 索引删除该地图要素
   * @return true       删除成功
   * @return false      删除失败
   */
  bool Delete(std::shared_ptr<map_interface::StaticMapComponent> component);

  /**
 * @brief 添加地图要素
 * 添加的地图元素将同步添加到检索库中（ID/Spatial)
 * @param component
 * @return true     添加成功
 * @return false    添加失败
 */
  bool AddMapComponent(
      std::shared_ptr<map_interface::StaticMapComponent> component);

  /**
   * @brief 添加地图要素
   * 添加的地图元素将同步添加到检索库中（ID/Spatial)
   * @param components
   * @return true     添加成功
   * @return false    添加失败
   */
  bool AddMapComponent(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components);

  /**id
   * @brief 物理层空间检索状态查询
   *
   * @param components
   * @return true     初始成功
   * @return false    初始失败
   */
  bool CheckSpatialPhysicalInit() { return state_spatial_physical_init_; }

  /**
   * @brief 逻辑层空间检索状态查询
   *
   * @return true     初始成功
   * @return false    初始失败
   */
  bool CheckSpatialLogicalInit() { return state_spatial_logical_init_; }

  /**
   * @brief ID检索状态查询
   *
   * @param components
   * @return true     初始成功
   * @return false    初始失败
   */
  bool CheckIDInit() { return state_id_init_; }

  /**
 * @brief ID NodeLink 检索状态查询
 *
 * @return true     初始成功
 * @return false    初始失败
 */
  bool CheckIDInLinkInit() { return state_id_inlink_; }

  /**
   * @brief 设置检索范围
   *
   */
  void SetBoundingboxRange(std::vector<float> bbx_range);

  /**
 * @brief 检索前驱同类component
 * @param component
 * @return component
 */
  bool FindPred(
      std::shared_ptr<map_interface::StaticMapComponent> component,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &res);

  /**
   * @brief 检索后继同类component
   *
   * @param component
   * @return component
   */
  bool FindSucc(
      std::shared_ptr<map_interface::StaticMapComponent> component,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &res);

  /**
   * @brief 检索左边同类component
   * @param component
   * @return component
   */
  bool FindLeft(
      std::shared_ptr<map_interface::StaticMapComponent> component,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &res);

  /**
   * @brief 检索右边同类component
   *
   * @param component
   * @return component
   */
  bool FindRight(
      std::shared_ptr<map_interface::StaticMapComponent> component,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &res);

  /**
   * @brief 检索当前车道的driveline
   *
   * @param lane_id
   * @return component
   */
  bool FindDriveLine(
      std::string lane_id,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &res);

  /**
   * @brief 检索当前车道的driveline
   *
   * @param component
   * @return component
   */
  bool FindDriveLine(
      std::shared_ptr<map_interface::StaticMapComponent> component,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &res);

  /**
   * @brief 拆分component，主要把非物理层的component其包含元素找出来
   *
   * @param components 输入的compoent
   * @return component 拆分后的component
   */
  bool UnpackComponent(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &res);

  /**
   * @brief 拆分component，主要把非物理层的component其包含元素找出来
   *
   * @param component 输入的compoent
   * @return component 拆分后的component
   */
  bool UnpackComponent(
      std::shared_ptr<map_interface::StaticMapComponent> component,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &res);

  /**
   * @brief 获取相对地图的位姿
   *
   * @param global_pose 全局位姿
   * @param local_pose  相对位姿
   * @param section_id  当前所处section id
   * @return true 位姿计算成功
   */
  bool GetLocalPosition(
      Eigen::Matrix4d global_pose, Eigen::Matrix4d *local_pose,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> stubmap);

  bool FindStubMap(
      std::shared_ptr<map_interface::StaticMapComponent> root,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &stubmap,
      int level);

 private:
  int Search(
      RTree<StaticMapComponent *, double, 3> tree, double ExtentRangMin[3],
      double ExtentRangMax[3],
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *submap);

 private:
  bool state_spatial_physical_init_;  /// 记录物理层空间检索的初始化状态
  bool state_spatial_logical_init_;  /// 记录逻辑层空间检索的初始化状态
  bool state_id_init_;               /// 记录ID检索的初始化状态
  bool state_id_inlink_;             /// 记录node-edge的检索，找入边

  /// 地图空间物理层检索
  RTree<std::shared_ptr<map_interface::StaticMapComponent>, double, 3>
      spatial_physic_;

  /// 地图空间物理层检索
  RTree<std::shared_ptr<map_interface::StaticMapComponent>, double, 3>
      spatial_logic_;

  /// 地图ID检索
  std::map<std::string, std::shared_ptr<map_interface::StaticMapComponent>>
      idMap_;

  /// 地图NODE的检索，对应的key是其关联的edge_id
  std::map<std::string, std::shared_ptr<map_interface::StaticMapComponent>>
      id_in_link_;

  /// 空间检索，bounding box的范围，依次为：前、后、左、右、上、下
  std::vector<float> bbx_range_;

  // 以下为旧的API,待调整、拆分和删除  ——————————————————————————————————
 public:
  bool LoadMapCompnent(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components_map);

  bool RemoveMapComponent(
      std::shared_ptr<map_interface::StaticMapComponent> component);

  /**
   * @brief 根据components_map中每个component的Bounding Box的空间位置，
   * 基于R-Tree建立位置索引
   *
   * @return true     建立索引成功
   * @return false    建立索引失败
   */
  bool InitRtree();
  /**
   * @brief 基于RTree建立Id索引
   *
   * @return true 建立索引成功
   * @return false 建立索引失败
   */
  bool InitIdRTree();

  /**
   * @brief 根据传入的位姿以及搜索范围（通过SetBoundingBoxRange设置），
   * 进行地图检索。检索到的components，可以通过GetSubmap()获得
   *
   * @param pose  位姿
   * @return true 检索成功
   * @return false 检索失败
   */
  bool MapRetrieval(const Eigen::Matrix4d &pose);
  /**
   * @brief 根据id进行地图检索
   *
   * @param id 要素id
   * @return true 检索成功
   * @return false 检索失败
   */
  bool MapRetrievalById(const std::string &id);

  /**
   * @brief 对检索到的components进行采样，增加采样点
   *
   * @return true 采样成功
   * @return false 采样失败
   */
  bool SubmapSampling();
  /**
   * @brief 对曲线进行采样，增加采样点
   *
   * @param mcomponent 包括车道线
   */
  void CurveLineSampling(
      std::shared_ptr<map_interface::StaticMapComponent> mcomponent);
  /**
   * @brief 对多边形框进行采样，增加采样点
   *
   * @param mcomponent 包括StopLine, CrossWalkLine, CrossWalk,
   * Speedbump, LaneMarking, TrafficLight
   */
  void PolygonSampling(
      std::shared_ptr<map_interface::StaticMapComponent> mcomponent);
  /**
   * @brief 对圆柱体进行采样，增加采样点
   *
   * @param mcomponent 包括Pole
   */
  void CylinderSampling(
      std::shared_ptr<map_interface::StaticMapComponent> mcomponent);
  /**
   * @brief 获得在检索范围bounding box内部, 各components的点
   *
   * @param points    存储在检索范围bounding box内部,各components的点
   * @return true     points.size() >= 1
   * @return false    points.size() < 1
   */
  bool GetBoundingBoxPoints(std::vector<Eigen::Vector4d> *points);
  /**
   * @brief 获得在检索范围bounding box内部, 各components的点
   *
   * @param points    存储在检索范围bounding box内部,各components的点
   * @return true     points.size() >= 8
   * @return false    points.size() < 8
   */
  bool GetBoundingBoxPoints(std::vector<std::vector<Eigen::Vector4d>> *points);
  /**
   * @brief   对由MapRetrieval检索到的component,根据BoundingBox的范围进行裁剪
   * 将裁剪后的objs进行保存
   *
   * @param objs      保存裁剪后的obj
   * @param pose
   * @return true     objs.size() > 0
   * @return false    objs.size() == 0
   */
  bool GetBoundingBoxObjects(std::vector<std::vector<Eigen::Vector4d>> *objs,
                             Eigen::Matrix4d pose);

  /**
   * @brief   对由MapRetrieval检索到的component,根据BoundingBox的范围进行裁剪
   * 将裁剪后的objs进行保存
   *
   * @param objs      保存裁剪后的obj
   * @param pose
   * @param bbx      BoundingBox范围
   * @param submap   待裁剪的component
   * @return true     objs.size() > 0
   * @return false    objs.size() == 0
   */
  bool GetBoundingBoxObjects(
      std::vector<std::vector<Eigen::Vector4d>> *objs, Eigen::Matrix4d pose,
      std::vector<Eigen::Vector3d> bbx,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> submap);

  /**
   * @brief   对由MapRetrieval检索到的component,根据BoundingBox的范围进行裁剪
   * 将裁剪后的virtualline和driveline进行保存
   *
   * @param objs      保存裁剪后的obj
   * @param pose
   * @param bbx      BoundingBox范围
   * @param submap   待裁剪的component
   * @return true     objs.size() > 0
   * @return false    objs.size() == 0
   */
  bool GetBoundingBoxVirtualObjects(
      std::vector<std::vector<Eigen::Vector4d>> *objs, Eigen::Matrix4d pose,
      std::vector<Eigen::Vector3d> bbx,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> submap);

  /**
   * @brief
   * 对由MapRetrieval检索到的车道线，路沿元素,根据BoundingBox的范围进行裁剪
   * 将裁剪后的车道线，路沿元素进行保存
   *
   * @param objs      保存裁剪后的车道线，路沿元素
   * @param pose
   * @return true     objs.size() > 0
   * @return false    objs.size() == 0
   */
  bool GetBoundingBoxSolidLaneObjects(
      std::vector<std::vector<Eigen::Vector4d>> *objs, Eigen::Matrix4d pose);

  /**
   * @brief
   * 对由MapRetrieval检索到的车道线，路沿元素,根据BoundingBox的范围进行裁剪
   * 将裁剪后的车道线，路沿元素进行保存
   *
   * @param objs      保存裁剪后的车道线，路沿元素
   * @param pose
   * @param maps      待裁剪的components
   * @return true     objs.size() > 0
   * @return false    objs.size() == 0
   */
  bool GetBoundingBoxSolidLaneObjects(
      std::vector<std::vector<Eigen::Vector4d>> *objs, Eigen::Matrix4d pose,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> maps);

  /**
   * @brief   对3D多边形进行裁剪，只保存在boundingBox范围
   * （ 通过SetBoundingBoxRange设置）中的点
   *
   * @param mcomponent    正在处理的component
   * @param obj       存储裁剪后，object在bounding box范围内的点
   * @param pose      相机位姿
   * @return true     裁剪成功
   * @return false    裁剪失败
   */
  bool PolygonTailor3D(
      std::shared_ptr<map_interface::StaticMapComponent> cmcomponent,
      std::vector<Eigen::Vector4d> *obj, Eigen::Matrix4d pose);

  /**
   * @brief   对3D实曲线进行裁剪，只保存在boundingBox范围
   * 通过SetBoundingBoxRange设置）中的实线点
   *
   * @param mcomponent    正在处理的component
   * @param obj       存储裁剪后，object在bounding box范围内的实线点
   * @param pose      相机位姿
   * @return true     裁剪成功
   * @return false    裁剪失败
   */
  bool SolidLaneTailor3D(
      std::shared_ptr<map_interface::StaticMapComponent> mcomponent,
      std::vector<Eigen::Vector4d> *obj, Eigen::Matrix4d pose);

  /**
   * @brief   对3D曲线进行裁剪，只保存在boundingBox范围
   * 通过SetBoundingBoxRange设置）中的点
   *
   * @param mcomponent    正在处理的component
   * @param obj       存储裁剪后，object在bounding box范围内的点
   * @param pose      相机位姿
   * @return true     裁剪成功
   * @return false    裁剪失败
   */
  bool CurveLineTailor3D(
      std::shared_ptr<map_interface::StaticMapComponent> mcomponent,
      std::vector<std::vector<Eigen::Vector4d>> *obj, Eigen::Matrix4d pose);
  /**
   * @brief   对3D曲线进行裁剪，只保存在boundingBox范围
   * 通过SetBoundingBoxRange设置）中的点
   *
   * @param mcomponent    正在处理的component
   * @param obj       存储裁剪后，object在bounding box范围内的点
   * @param pose      相机位姿
   * @return true     裁剪成功
   * @return false    裁剪失败
   */
  bool CurveLineTailor3D(
      std::shared_ptr<map_interface::StaticMapComponent> mcomponent, int id,
      std::vector<Eigen::Vector4d> *obj, Eigen::Matrix4d pose);
  /**
   * @brief   对3D曲线进行裁剪，只保存在boundingBox范围
   * 通过SetBoundingBoxRange设置）中的点
   *
   * @param local_obj 待裁剪的局部坐标系下点的集合
   * @param obj       存储裁剪后，object在bounding box范围内的点
   * @param pose      相机位姿
   * @param type      点的类型
   * @return true     裁剪成功
   * @return false    裁剪失败
   */
  bool CurveLineTailor3D(std::vector<Eigen::Vector4d> local_obj,
                         std::vector<Eigen::Vector4d> *obj,
                         Eigen::Matrix4d pose, HDMapComponentType type);
  /**
   * @brief
   * 对3D圆柱进行裁剪，只保存在boundingBox范围（通过SetBoundingBoxRange设置）中的点
   *
   * @param mcomponent    正在处理的component
   * @param obj       存储裁剪后，object在bounding box范围内的点
   * @param pose      相机位姿
   * @return true     裁剪成功
   * @return false    裁剪失败
   */
  bool CylinderTailor3D(
      std::shared_ptr<map_interface::StaticMapComponent> mcomponent,
      std::vector<Eigen::Vector4d> *obj, Eigen::Matrix4d pose);

  /**
   * @brief   对实际同属一条实线按多个obj存储的元素进行合并
   */
  void MergeSolidLane(std::vector<std::vector<Eigen::Vector4d>> *objs);

  /**
   * @brief 设置检索BoundingBox的范围
   *
   * @param fblrtb 前后左右上下
   */
  void SetBoundingBoxRange(std::vector<float> fblrtb);

  void SetPlaneFilterSize(int size);
  void SetSamplintDis(float dist);

  /**
   * @brief 获得component中所有曲线的点
   *
   * @param points_vec    存储曲线点
   * @param mcomponent    component
   */
  void GetCurveLinePoints(std::vector<Eigen::Vector3d> *points_vec,
                          StaticMapComponent mcomponent);
  /**
   * @brief 获得component中所有多边形的点
   *
   * @param points_vec    存储多边形点
   * @param mcomponent    component
   */
  void GetPolygonPoints(std::vector<Eigen::Vector3d> *points_vec,
                        StaticMapComponent mcomponent);

  bool CheckIfUpdateMap() {}  // map state:Check if update submap
  // map state:Check if current pose is out of range
  bool CheckIfMapOutOfRange() {}
  /**
   * @brief 检查地图是否初始化成功
   *
   * @return true     地图初始化成功，
   * components保存到components_map中（可以通过GetMap()得到），构建R-Tree索引
   * @return false    地图初始化失败
   */
  bool CheckIfMapInit();

  /**
   * @brief 获得components集合
   *
   * @return std::vector<StaticMapComponent>
   */
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> GetMap();

  /**
   * @brief 获得地图搜索范围内的components集合
   *
   * @return std::vector<StaticMapComponent>
   */
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> GetSubmap();

  /**
   * @brief 获得地图搜索范围内的components集合
   *
   * @return std::vector<StaticMapComponent>
   */
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
      *GetSubmapAddr();

  std::vector<Eigen::Vector4d> GetPCSubmap();

  ndm_proto::MapEnvMsg *getProtoMap();

  //待调整，进行拆分、删除-------------------------------
  /**
   * @brief 对多边形、折线段进行裁剪，返回可视范围内的节点
   *
   */
  void PixelWiseFilling(std::vector<Eigen::Vector4d> contour,
                        std::vector<Eigen::Vector4d> *filling, float dist);
  // GET LOCAL INFORMATION FORM MAP: map height, map roll, map pitch
  bool getLocalPosition(Eigen::Matrix4d global_pose,
                        Eigen::Matrix4d *local_pose);
  bool fitPlane(std::vector<Eigen::Vector4d> ground, Eigen::Vector4d *plane);
  void SetGroundSearchRadius(float radius);
  void SetGroundSamplingDist(float dist);

  /**
   * @brief 重置地面方程估计的滑窗滤波
   *
   */
  void groundFilterReset();

  /**
   * @brief 计算当前位姿是否处于地图范围内
   *
   */
  bool inMapDataRange(Eigen::Matrix4d global_pose);

 private:
  /**
   * @brief   地图初始化，对相关元素进行整理，保存至components_map中，
   *          并调用构建地图索引
   *
   * @return true     初始化成功
   * @return false    初始化失败
   */
  // bool Init();

 public:
  Eigen::Matrix4d pose_map;  /// current pose, for update, init, check map etc

  /// 地图空间位置索引
  RTree<std::shared_ptr<map_interface::StaticMapComponent>, double, 3> myRtree;

  // RTree<StaticMapComponent*, uint64_t, 1> idRtree;  // 地图id索引
  std::map<std::string, std::shared_ptr<map_interface::StaticMapComponent>>
      idMap;

 private:
  ndm_proto::MapEnvMsg proto_map;  /// protoMap
  ndm_proto::MapEnvMsg proto_submap;

  bool map_state_mapdone;  /// map state: if map loading done

  bool map_state_init;  /// map state: if map initialization done

  bool map_state_submapdone;  /// map state: if pose loading done

  /// 地图元素集合，由Map Init得到，用于地图检索
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
      components_map;
  /// 子图元素集合，由map retrieval得到，用于map projection
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
      components_submap;
  /// vertex of bounding box, update when map retrieval done
  std::vector<Eigen::Vector3d> vbounding_box;

  float observe_front;   /// 地图搜索范围：前
  float observe_back;    /// 地图搜索范围：后
  float observe_left;    /// 地图搜索范围：左
  float observe_right;   /// 地图搜索范围：右
  float observe_top;     /// 地图搜索范围：上
  float observe_bottom;  /// 地图搜索范围：下

  float sampling_dist;  /// Sampling时的采样距离
  float groud_search_radius;
  float groud_sampling_dist;  // 地面滤波的采样距离

  int plane_filter_window_size;

  /// only for visualization debug...
  std::vector<Eigen::Vector4d> showpointcloud;
  /// sampling points in submap, update when map retrieval
  // std::vector<Eigen::Vector4d> pointcloud_submap;

  /// sampling points in submap, update when map retrieval
  std::vector<std::vector<Eigen::Vector4d>> pointcloud_submap;

  std::vector<Eigen::Vector4d> plane_filter_;

  bool plane_filter_init;

  std::string utm_zone_;
  Eigen::Vector3d map_offset_;
};
}  // namespace map_interface
#endif  // INTERFACE_INDEX_H_
