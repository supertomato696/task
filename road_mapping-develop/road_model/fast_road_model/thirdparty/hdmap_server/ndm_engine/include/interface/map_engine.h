/**
 * @file index.h
 * @author Han Fei(fei.han@horizon.ai), Zeng Siyu(siyu.zeng@horizon.ai)
 * @brief  MapEngine Interface
 * @version 3.1
 * @date 2020-02-27
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef INTERFACE_MAP_ENGINE_H_
#define INTERFACE_MAP_ENGINE_H_
#include "../interface/geometry.h"
#include "../interface/index.h"
#include "../interface/map_component.h"
#include "../interface/route.h"
// #include "../comm/comm.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

//最大缓存地图数据块
#define MAX_MAP_IDX 2

typedef std::pair<std::shared_ptr<map_interface::StaticMapComponent>, double>
    PairOfCmpAndOffset;

namespace map_interface {
/**
* @brief 地图引擎
* 加载、增删改地图元素，地图定位初始化（具体到某个section/junction/lane）
* 检索、导航，相对地图位姿估算，状态查询，地图分块加载等
*/
class MapEngine {
 public:
  /**
   * @brief 构造MapEngine对象
   *
   */
  MapEngine();

  /**
   * @brief Destroy the MapEngine object
   *
   */
  ~MapEngine() {
    for (int i = 0; i < MAX_MAP_IDX; i++) {
      delete index_[i];
      delete route_[i];
    }
    ClearMapComponent();
  }

  /**
   * @brief 初始引擎
   * @在地图中找到初始位置，分层道路越多，则收敛越慢
   * @param pose 车体坐标系
   * @return true
   * @return false
   */
  bool Init(Eigen::Matrix4d pose);

  /**
   * @brief 初始引擎
   * @在地图中找到初始位置，分层道路越多，则收敛越慢
   * @param pose 车体坐标系
   * @return true
   * @return false
   */
  bool Init(double lat, double lon, double height, double yaw);

  /**
   * @brief 初始引擎
   * @在地图中找到初始位置，分层道路越多，则收敛越慢
   * @param pose 车体坐标系
   * @return true
   * @return false
   */
  bool InitWithPose(Eigen::Matrix4d pose);

  /**
   * @brief 更新引擎 自动触发引擎的状态变化
   * @param pose 车体坐标系
   * @return true
   * @return false
   */
  bool Update(Eigen::Matrix4d pose);

  bool Update(double lat, double lon, double height, double yaw);

  /**
   * @brief 更新引擎状态 包括bbx,section,lane,stubmap
   * @param pose 车体坐标系
   * @return true
   * @return false
   */
  bool UpdateEngineState(Eigen::Matrix4d pose);

  /**
   * @brief 引擎地图匹配初始化
   * @根据GPS轨迹序列，找到地图中粒度为section的初始化位置
   * @param pose 车体坐标系
   * @return true
   * @return false
   */
  bool InitMapMatching(Eigen::Matrix4d pose);

  /**
   * @brief 引擎状态跟踪初始化
   * @引擎跟踪当前车辆所处section/lane的状态，当跟丢时，需要重新进行跟踪状态初始化
   * @param pose 车体坐标系
   * @return true
   * @return false
   */
  bool InitStateTracking(Eigen::Matrix4d pose);

  bool InitLaneGraph(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components);

  bool InitRouteGraph(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components,
      int attention_level);

  /**
   * @brief 加载地图component数据
   * @param components 地图数据
   * @param idx       对应下标
   * @return true     加载成功
   * @return false    加载失败
   */
  bool LoadMapCompnent(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
          components,
      int idx);

  /**
   * @brief 加载地图proto数据
   * @param map_proto 地图数据
   * @param idx       对应下标
   * @return true     加载成功
   * @return false    加载失败
   */
  bool LoadMapProto(std::shared_ptr<ndm_proto::MapEnvMsg> map_proto, int idx);

  /**
   * @brief 加载地图数据
   * @param map_proto 地图数据
   * @param components 地图数据
   * @param idx       对应下标
   * @return true     加载成功
   * @return false    加载失败
   */
  bool LoadMap(std::shared_ptr<ndm_proto::MapEnvMsg> map_proto,
               std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
                   components,
               int idx);

  /**
 * @brief 获取lane-level级的路线
 * @param start 起点
 * @param end 终点
 * @param path 路线
 */
  bool GetLaneRoute(Eigen::Vector3d start_point, Eigen::Vector3d end_point,
                    std::vector<std::string> *path);

  /**
   * @brief 获取lane-level级的路线
   * @return true 更新成功
   */
  bool UpdateLaneRoute();

  /**
   * @brief 获取road-level级的路线
   * @return false 更新失败
   */
  bool UpdateRoadRoute();

  /**
   * @brief 添加地图要素
   * 添加的地图元素将同步添加到检索库中（ID/Spatial)
   * @param component
   * @return true     添加成功
   * @return false    添加失败
   */
  // bool AddMapComponent(StaticMapComponent* component);

  /**
   * @brief 添加地图要素
   * 添加的地图元素将同步添加到检索库中（ID/Spatial)
   * @param components
   * @return true     添加成功
   * @return false    添加失败
   */
  // bool AddMapComponent(std::vector<StaticMapComponent* > components);

  /**
   * @brief 删除地图要素
   * 删除的地图元素将同步在检索库中删除（ID/Spatial)
   * @param component
   * @return true     添加成功
   * @return false    添加失败
   */
  // bool DeletMapComponent(StaticMapComponent* component);

  /**
   * @brief 删除地图要素
   * 删除的地图元素将同步在检索库中删除（ID/Spatial)
   * @param components
   * @return true     添加成功
   * @return false    添加失败
   */
  // bool DeletMapComponent(std::vector<StaticMapComponent* > components);

  /**
   * @brief 替换地图要素
   * 删除的地图元素将同步在检索库中删除（ID/Spatial)
   * @param component
   * @return true     添加成功
   * @return false    添加失败
   */
  // bool CoverMapComponent(StaticMapComponent* component);

  /**
   * @brief 替换地图要素
   * 删除的地图元素将同步在检索库中删除（ID/Spatial)
   * @param components
   * @return true     添加成功
   * @return false    添加失败
   */
  // bool CoverMapComponent(std::vector<StaticMapComponent* > components);
  /**
   * @brief 初始化物理层的空间检索
   *
   * @return true     建立空间检索成功
   * @return false    建立空间检索失败
   */
  // bool InitPhysicalSpatialIndex();

  /**
   * @brief 初始化逻辑层的空间检索
   *
   * @return true     建立空间检索成功
   * @return false    建立空间检索失败
   */
  // bool InitLogicalSpatialIndex();

  /**
   * @brief 初始化空间检索
   * 如果没有特别指定层级，则默认所有层级都要初始化
   * @return true     建立空间检索成功
   * @return false    建立空间检索失败
   */
  // bool InitSpatialIndex();

  /**
   * @brief 初始化ID检索
   * @detail ID不区分层级检索
   *
   * @return true 建立ID检索成功
   * @return false 建立ID检索失败
   */
  // bool InitIDIndex();

  /**
   * @brief 判断是否需要更新地图
   * @detail 终端地图为云端地图的子集，需动态加载。以tile划分判断
   * @param pose，车体坐标系
   * @return true 需要更新地图
   * @return false 不需要更新地图
   */
  bool CheckIfUpdateMap(Eigen::Matrix4d pose);

  /**
   * @brief 获取地图的更新状态
   * @detail 由另一个进程调用，当地图需要更新时，向云端发送请求并加载地图
   *
   * @return true 需要更新地图
   * @return false 不需要更新地图
   */
  bool CheckIfUpdateMap() { return state_update_map_; }

  /**
   * @brief 检查是否超出地图范围
   * @param pose，车体坐标系
   * @return true 超出地图范围
   * @return false 还在地图范围内
   */
  double CheckOutOfRange(Eigen::Matrix4d pose);

  /**
   * @brief 检查地图是否初始化成功
   * @return true     引擎状态初始化成功，
   * @return false    引擎状态初始化失败
   */
  bool CheckEngineInit() { return state_engine_init_; }

  /**
   * @brief 检查引擎是否初始化成功，有地图加载进来
   * @return true     引擎初始化成功，
   * @return false    引擎初始化失败
   */
  bool CheckInit();

  /**
   * @brief 查看是否按照既定路线走
   * @param pose  当前车辆位置
   * @param path  规划路径，并剔除已行驶过的节点（road/lane）
   * @return true
   * @return false
   */
  bool CheckOutOfRoute(const Eigen::Matrix4d pose,
                       std::vector<std::string> *path);

  /**
   * @brief 查看是否按照既定路线走
   * @param pose  当前车辆位置
   * @param path  规划路径，并剔除已行驶过的节点（road/lane）
   * @return true
   * @return false
   */
  bool CheckOutOfRoute(const Eigen::Vector3d pose);

  /**
   * @brief 查看section的状态是否变化
   * @return true 状态发生变化
   * @return false 状态没有发生变化
   */
  bool CheckSectionState() { return state_section_change_; }

  /**
   * @brief 查看lane的状态是否变化
   * @return true 状态发生变化
   * @return false 状态没有发生变化
   */
  bool CheckLaneState() { return state_lane_change_; }

  /**
   * @brief 查看是否处于ODD区域
   * @return true 处于ODD区域
   * @return false 处于非ODD区域
   */
  bool CheckIsODD() { return true; }

  /**
    * @brief 获得components集合
    *
    * @return std::vector<StaticMapComponent* >
    */
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> *GetMapAddr();

  /**
    * @brief 获得components集合
    *
    * @return std::vector<StaticMapComponent* >
    */
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> GetMap();

  /**
    * @brief 获得当前使用的protomap
    *
    * @return std::shared_ptr<ndm_proto::MapEnvMsg>
    */
  std::shared_ptr<ndm_proto::MapEnvMsg> GetProtoMap();

  /**
    * @brief 获得components集合
    *
    * @return std::vector<StaticMapComponent* >
    */
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
      *GetStubMapAddr() {
    return &stubmap_;
  }

  /**
    * @brief 获得components集合
    *
    * @return std::vector<StaticMapComponent* >
    */
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> GetStubMap() {
    return stubmap_;
  }

  /**
   * @brief 获取当前车体位姿在地图坐标系下的位姿
   * @param global_pose 全局位姿
   * @param local_pose 全局位姿，在map坐标系下
   * @return true 计算位姿成功
   * @return false 计算位姿失败
   */
  bool GetLocalPosition(Eigen::Matrix4d global_pose,
                        Eigen::Matrix4d *local_pose);

  /**
   * @brief 清除component
   */
  void ClearMapComponent();

  /**
   * @brief 获取索引指针
   */
  map_interface::Index *GetIndex();

  /**
  * @brief 获取导航指针
  */
  map_interface::Route *GetRoute();

  /**
   * @brief 获取当前所处逻辑分区（section/junction）
   * 只有在map engine初始化后才能进行
   * @param component 当前的逻辑分区
   * @return true 获取成功
   */
  bool GetCurSection(std::vector<PairOfCmpAndOffset> &res);

  /**
   * @brief 获取当前所处lane
   * 只有在map engine初始化后才能进行
   * @param component 当前的lane
   * @return true 获取成功
   */
  bool GetCurLane(std::vector<PairOfCmpAndOffset> &res);

  /**
   * @brief 重置当前所处逻辑分区（section/junction）
   * @param component 当前的逻辑分区
   * @return true 重置成功
   */
  bool ResetCurSection(PairOfCmpAndOffset res);

  /**
   * @brief 重置当前所处车道
   * @param component 当前的lane
   * @return true 重置成功
   */
  bool ResetCurLane(PairOfCmpAndOffset res);

  /**
   * @brief   对每帧stubmap的component,根据BoundingBox的范围进行裁剪
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
   * @brief   对每帧stubmap的component,根据BoundingBox的范围进行裁剪
   * 将裁剪后的virtualline和driveline进行保存
   *
   * @param objs      保存裁剪后的obj
   * @param pose
   * @return true     objs.size() > 0
   * @return false    objs.size() == 0
   */
  bool GetBoundingBoxVirtualObjects(
      std::vector<std::vector<Eigen::Vector4d>> *objs, Eigen::Matrix4d pose);

  /**
   * @brief
   * 对每帧stubmap的车道线，路沿元素,根据BoundingBox的范围进行裁剪
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
   * 对每帧stubmap的车道线，路沿元素,根据BoundingBox的范围进行裁剪
   * 将裁剪后的车道线，路沿元素进行保存
   *
   * @param objs      保存裁剪后的车道线，路沿元素
   * @param pose
   * @return true     objs.size() > 0
   * @return false    objs.size() == 0
   */
  bool SetSightViewRange(std::vector<float> fblrtb);

  /**
   * @brief 设置stubmap的长度，单位：米
   *
   * @return true
   * @return false
   */
  bool SetStubMapLength(double length) { stubmap_length_ = length; }

  /**
   * @brief 设置stubmap相对车辆位置的距离范围，单位：米
   * @param front_radius    车辆前方距离
   * @param back_radius     车辆后方距离
   */
  void SetStubMapRadius(double front_radius = 500, double back_radius = 0.0);

  /**
   * @brief 设置地图范围section级别的距离约束，单位：米
   *
   * @return true
   * @return false
   */
  bool SetMapRangeDist(double length) { map_range_dist_ = length; }

  /**
   * @brief 记录位置识别时地图初始化搜索范围，采用的是半径搜索形式，单位：米
   *
   * @return true
   * @return false
   */
  bool SetMapRangeInit(double length) { map_range_init_ = length; }

  /**
   * @brief 设置引擎关注的细分粒度，默认到secton层级
   *
   * @param attention_lane_level_ 当前所处的section id
   * @return true
   * @return false
   */
  bool SetAttentionLaneLevel(bool flag) { attention_lane_level_ = flag; }

  void SetSamplintDis(float dist);
  void SetPlaneFilterSize(int size);
  void SetGroundSearchRadius(float radius);
  void SetGroundSamplingDist(float dist);
  void SetAttributeConstraints(bool flag);
  void SetNavigationPreferences(NavigationPreferences type);
  /**
   * @brief 获取stub地图，即用于ADASIS中的局部地图发送
   *
   * @param section_id 当前所处的section id
   * @param stubmap  相对位姿
   * @return true 位姿计算成功
   */
  bool FindStubMap(
      std::shared_ptr<map_interface::StaticMapComponent> root,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &stubmap,
      double cur_length, double tar_length = 500);

  /**
   * @brief 获取stub地图道路分组(section)序列，即用于ADASIS中的局部地图发送
   *        section序列按车辆行驶方向从前往后排
   * @param root 当前车辆所处的section id
   * @param stubmap      返回局部地图计算结果，stubmap中存储的是section序列
   * @param cur_length   当前车辆相对所处section的起点位置的offset
   * @param map_radius   传入地图范围，单位：米
   *                     map_radius[0]为车辆前方距离，
   *                     map_radius[1]为车辆后方距离
   * @return true stubmap序列计算成功
   */
  bool FindStubMap(
      std::shared_ptr<map_interface::StaticMapComponent> root,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &stubmap,
      double cur_length, std::vector<double> map_radius = {500, 0});

  /**
   * @brief 获取stub地图道路分组(section)序列，即用于ADASIS中的局部地图发送
   *        车辆前方的stubmap序列计算
   * @param root 当前车辆所处的section id
   * @param stubmap      返回局部地图计算结果，stubmap中存储的是section序列
   * @param cur_length   当前车辆相对所处section的起点位置的offset
   * @param tar_length   传入车辆前方地图范围，单位：米
   * @return true stubmap序列计算成功
   */
  bool FindStubMapForward(
      std::shared_ptr<map_interface::StaticMapComponent> root,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &stubmap,
      double cur_length, double tar_length = 500);

  /**
   * @brief 获取stub地图道路分组(section)序列，即用于ADASIS中的局部地图发送
   *        车辆后方的stubmap序列计算，返回结果不包含当前所处的section id
   * @param root 当前车辆所处的section id
   * @param stubmap      返回局部地图计算结果，stubmap中存储的是section序列
   * @param cur_length   当前车辆相对所处section的起点位置的offset
   * @param tar_length   传入车辆后方地图范围，单位：米
   * @return true stubmap序列计算成功
   */
  bool FindStubMapBackward(
      std::shared_ptr<map_interface::StaticMapComponent> root,
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &stubmap,
      double cur_length, double tar_length = 500);

  /**
   * @brief 开启数据收发机制，目前设计是收发属于两个独立线程，因此独立启动
   *
   * @return true 成功开启
   */
  bool CommStartPublish();

  /**
   * @brief 发送环境模型数据
   *
   * @return true 成功发送
   */
  bool CommSendEnvironmentModel(
      std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &map,
      map_interface::Odometry *odom_mem, std::string projection);

  std::vector<std::string> *GetLaneRoute() { return &lane_route_; }

  /**
   * @brief 更新地图块，同时更新引擎内部的状态
   */
  void UpdateMap();

  /**
   * @brief 检查是否有地图块更新
   */
  bool CheckUpdateMap();

  /**
   * @brief 设置地图块更新的状态量
   */
  void SetUpdateMap(bool state);

  /**
   * @brief 获取下一个地图块的索引ID
   */
  int GetNextMapIdx();

  /**
   * @brief 获取当前地图块的索引ID
   */
  int GetCurMapIdx();

  /**
   * @brief 获取当前车辆所在的road id以及lane id
   * @param res <road_id, lane_id>
   * @return true 返回值有效
   */
  bool GetRelativePos(
      std::vector<std::pair<PairOfCmpAndOffset, PairOfCmpAndOffset>> &res);

  bool GetZoneAndOffset(std::string &UTMZone, Eigen::Vector3d &offset);
  void SetZoneAndOffset(std::string UTMZone, const Eigen::Vector3d &offset);

  /**
   * @brief 将WGS84转成UTM坐标系
   *
   * @param objs      保存裁剪后的车道线，路沿元素
   * @param pose
   * @return true     objs.size() > 0
   * @return false    objs.size() == 0
   */
  // bool TransformFromWGS84ToUTM(const double Lat, const double Long,
  // const double);
 private:
  bool UpdateSectionState(const Eigen::Vector3d point,
                          const double dist_threshole, PairOfCmpAndOffset &cur,
                          std::vector<PairOfCmpAndOffset> &candidates);

  bool UpdateLaneState(const Eigen::Vector3d point, const double dist_threshole,
                       PairOfCmpAndOffset &cur,
                       std::vector<PairOfCmpAndOffset> &candidates);

  void UpdateStubmapState(Eigen::Matrix4d pose);
  void UpdateStubmapState();

  /**
   * @brief 随着地图块的迁移，迁移当前lane的状态
   */
  void TransferLaneState();

  /**
   * @brief 随着地图块的迁移，迁移当前section的状态
   */
  void TransferSectionState();

  /**
   * @brief 随着地图块的迁移，迁移当前stubmap的状态
   */
  void TransferStubmapState();

 public:
  /// 检索库。共两份，用于地图块更新替换
  std::vector<map_interface::Index *> index_;
  /// 导航库。共两份，用于地图块更新替换
  std::vector<map_interface::Route *> route_;

 private:
  int map_idx_;
  bool out_of_route_;  /// 脱离规划路线
  /// 更新地图块（云端-端端）,若为true，表示地图数据有更新，则需要更新map_idx_;
  bool state_update_map_;
  bool state_init_;         /// 记录引擎初始化状态
  bool state_engine_init_;  /// 记录引擎逻辑状态是否初始化成功
  bool state_engine_init_mapmatching_;  /// 记录地图匹配初始化是否完成
  bool state_engine_init_statetracking_;  /// 记录引擎状态跟踪初始化是否完成
  bool state_lane_change_;  /// 记录是否需要换道，仅在初始化route后生效
  bool state_section_change_;  /// 记录section是否变化状态
  bool state_pub_envmodel_;    /// 记录是否需要发布环境模型
  bool attention_lane_level_;  /// 记录是否关注lane层级的状态连续性

  /// 记录地图范围的距离约束,大于该阈值则认为超出地图范围
  double map_range_dist_;
  /// 记录位置识别时地图初始化搜索范围，采用的是半径搜索形式
  double map_range_init_;

  std::mutex idx_lock_;        /// 线程锁，map_idx_
  std::mutex state_lock_;      /// 线程锁，state_update_map_
  std::mutex map_lock_;        /// 线程锁，地图块元素 map_
  std::mutex map_proto_lock_;  /// 线程锁，地图proto数据 map_databse_

  // std::string cur_lane_id_;  /// 当前车道的id
  // std::string cur_section_id_;  /// 当前所在的section/junction的id

  std::vector<PairOfCmpAndOffset> cur_lane_;
  std::vector<PairOfCmpAndOffset> cur_section_;

  std::vector<std::string> lane_route_;
  std::vector<std::string> road_route_;

  /// 记录原始的proto数据。共两份，用于地图块更新替换
  std::vector<std::shared_ptr<ndm_proto::MapEnvMsg>> map_databse_;

  /// 地图component数据。共两份，用于地图块更新替换
  std::vector<std::vector<std::shared_ptr<map_interface::StaticMapComponent>>>
      map_;

  /// 地图数据中的component要素,为当前位置附近发出的子图
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> stubmap_;

  /// 地图数据中的component要素,为当前位置附近发出的子图
  std::vector<std::shared_ptr<map_interface::StaticMapComponent>> candidates_;

  int plane_filter_window_size_;  /// 地图地面方程估算的滑窗窗口大小
  bool plane_filter_init_;        /// 滑窗滤波初始状态
  std::vector<Eigen::Vector4d> plane_filter_;  /// 地面方程估计窗口
  /// 空间检索，bounding box的范围，依次为：前、后、左、右、上、下
  std::vector<float> bbx_range_;
  /// 视距范围
  std::vector<Eigen::Vector3d> vbbx_;
  /// stub map的长度约束
  double stubmap_length_;
  /// stub map的长度约束,依次为：车辆前方距离范围，车辆后方距离范围
  std::vector<double> map_radius_;
};

/**
 * @brief 获取环境模型信息，环境模型中主要包含三个点：
 * ①限速信息 ②trafficSign ③TrafficBoard ④LaneMarking
 *
 * @param map 传入构成stubmap的components
 * @param pose  当前车辆位姿
 * @param envmodel 滤除后的信息
 * @return true 位姿计算成功
 */
bool GetEnvModel(
    std::vector<std::shared_ptr<map_interface::StaticMapComponent>> map,
    Eigen::Matrix4d pose,
    std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &envmodel,
    std::vector<double> &distance);

bool RemoveDuplicate(
    std::vector<std::shared_ptr<map_interface::StaticMapComponent>> raw,
    std::vector<std::shared_ptr<map_interface::StaticMapComponent>> &res);

bool RemoveDuplicate(std::vector<PairOfCmpAndOffset> raw,
                     std::vector<PairOfCmpAndOffset> &res);

bool DemoGetStubDriveline();
}  // namespace map_interface
#endif  // INTERFACE_MAP_ENGINE_H_
