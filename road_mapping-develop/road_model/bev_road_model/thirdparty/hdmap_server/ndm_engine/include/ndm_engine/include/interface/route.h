/**
 * @file index.h
 * @author Han Fei(fei.han@horizon.ai), Zeng Siyu(siyu.zeng@horizon.ai)
 * @brief  MapEngine Interface
 * @version 3.1
 * @date 2020-03-06
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef INTERFACE_ROUTE_H_
#define INTERFACE_ROUTE_H_
#include "../interface/map_component.h"
#include "../interface/index.h"
#include "../interface/util.h"
#include "../interface/geometry.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <queue>
#include <map>
#include <string>
#include <memory>
#include <vector>
#include <limits>

#define LANE_WIDTH 3.75
#define WORST_CHOICE_WEIGHT 1000
#define NO_PASS_WEIGHT 9999
#define MAX_LANE_NUM_IN_SECTION 7
#define MIN_DECISION_DISTANCE 300

namespace map_interface
{
struct Node;
struct Edge {
  double weight_;        // 记录当前边的权重
  Node* node_;           // 记录当前边指向的顶点
  // std::string edge_id_;  // 当前edge_id
  Edge():weight_(0), node_(NULL) {}
  Edge(double weight, Node* node):weight_(weight), node_(node) {}
};

struct Node {
public:
  double s_;  // 到达目的地的权重值
  double len_;  // 长度，若是edge则是edge的长度，若是lane则是lane的长度
  std::string node_id_;  // 当前顶点对应component的id，node\edge\lane
  std::vector<Edge> nexts_;  // 该顶点连接的下一个顶点集合
  Node *last_;     // 指向上一个顶点，一个destination对应一个结果
  bool closed_;  // 记录该顶点的最短路径是否已锁定，若锁定，则last_不会更改。

  Node():s_(std::numeric_limits<double>::max()), len_(1), node_id_(""),
          last_(NULL), closed_(false) {}
  Node(double len, std::string node_id):
       s_(std::numeric_limits<double>::max()), len_(len),
       node_id_(node_id), last_(NULL), closed_(false) {}
  friend bool operator<(Node a, Node b) {
    return a.s_ > b.s_;
  }
};

enum NavigationPreferences {
  NONE = 0,
  LaneChangeOnRamp = 1
};

/**
* @brief 地图导航路线：细致为road-level, lane-level, trajs
* road-level: edge-node-edge-node-edge-node.....
* lane-level: vector<component>. ps:component->type: lane
* trajs     : for planning. e.g.lane changing 
*/
class Route
{
public:
  /**
   * @brief 构造MapEngine对象
   * 
   */
  Route();

  explicit Route(int attention_level);

  /**
   * @brief Destroy the MapEngine object
   * 
   */
  ~Route() {
    // delete index_;
  }

  /**
   * @brief 填入索引
   *
   * @param index
   */
  void FeedIndex(void *index);

  /**
   * @brief 设置目的地
   * @根据当前的route中的attention级别，去搜索相应的目的地node节点。
   * @param pose 车体坐标系
   * @return true 设置成功
   * @return false 设置失败
   */
  bool SetDestination(Eigen::Vector3d p);

  /**
   * @brief 设置车道目的地
   * @param id 车道id
   * @return true 设置成功
   * @return false 设置失败
   */
  bool SetLaneDestination(std::string id);

  /**
   * @brief 构建道路图模型
   * @param components 构建图模型所需道路级地图元素
   * @return true 构建成功
   * @return false 构建失败
   */
  bool InitRoadGraph(
        std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
        components);

  /**
   * @brief 构建车道图模型
   * @param components 构建图模型所需车道级地图元素
   * @return true 构建成功
   * @return false 构建失败
   */
  bool InitLaneGraph(
        std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
        components);

  /**
   * @brief 重置道路图模型
   * @return true 重置成功
   * @return false 重置失败
   */
  void ResetRoadGraph();

  /**
   * @brief 重置车道图模型
   * @return true 重置成功
   * @return false 重置失败
   */
  void ResetLaneGraph();

  /**
   * @brief 清空路网中的历史记录
   * @return true 重置成功
   * @return false 重置失败
   */
  void ClearHistoryRecord(std::map<std::string, Node*> &graph);

  /**
   * @brief 加载导航proto
   */
  void LoadRouteProto(std::shared_ptr<ndm_proto::NavigationMsg> nav_proto);

  /**
   * @brief 检查是否超出地图范围
   * 在既定路线行走时，关闭已走过的路线
   * @param laneid 当前所处lane id
   * @param section_id 当前所处section或者junction的id
   * @return true 脱离路线
   * @return false 按规定路线行走
   */
  // bool CheckOutOfRange(std::string lane_id, std::string section_id);

  /**
   * @brief 检查是否初始化路网图
   * @return true     地图初始化成功，
   * @return false    地图初始化失败
   */
   bool CheckInit();

   /**
   * @brief 查看是否按照既定路线走
   * @return true
   * @return false
   */
   bool CheckOutOfRoute() { return out_of_route_;}

  /**
   * @brief 查看是否按照既定路线走
   * @param pose  当前车辆位置
   * @param path  规划路径，并剔除已行驶过的节点（road/lane）
   * @return true
   * @return false
   */
  bool CheckOutOfRoute(const Eigen::Vector3d point,
                        std::vector<std::string> *path);

  /**
   * @brief 查看是否按照既定路线走
   * @param pose  当前车辆位置
   * @param path  规划路径，并剔除已行驶过的节点（road/lane）
   * @return true
   * @return false
   */
  bool CheckOutOfRoute(const Eigen::Vector3d point,
                       int &derailment_level);

    /**
   * @brief 查看是否按照既定路线走
   * @param pose  当前车辆位置
   * @param path  规划路径，并剔除已行驶过的节点（road/lane）
   * @return true
   * @return false
   */
  bool CheckOutOfRoadRoute(const Eigen::Vector3d point);

  /**
   * @brief 查看是否按照既定路线走
   * @param pose  当前车辆位置
   * @param path  规划路径，并剔除已行驶过的节点（road/lane）
   * @return true
   * @return false
   */
  bool CheckOutOfLaneRoute(const Eigen::Vector3d point);

   /**
   * @brief 查看是否需要变换车道
   * @return true
   * @return false
   */
   bool CheckChangeLane() { return state_lane_change_;}

   /**
   * @brief 获取lane-level级的路线
   * @param start 起点
   * @param end 终点
   * @param path 路线
   */
   bool GetLaneRoute(std::string start_id, std::string end_id,
                     std::vector<std::string> *path);

  /**
   * @brief 获取lane-level级的路线，终点已事先设置
   * @param start 起点
   * @param path 路线
   */
   bool GetLaneRoute(std::string start_id,
                     std::vector<std::string> *path);

  /**
   * @brief 更新lane-level级的路线，终点已事先设置，路线为成员变量
   * @param start 起点
   */
   bool UpdateLaneRoute(std::string start_id);

  /**
   * @brief 获取road-level级的路线
   * @param start 起点
   * @param end 终点
   * @param path 路线
   */
   bool GetRoadRoute(std::string start_id, std::string end_id,
                     std::vector<std::string> *path);

  /**
   * @brief 获取road-level级的路线，终点已事先设置
   * @param start 起点
   * @param path 路线
   */
   bool GetRoadRoute(std::string start_id,
                     std::vector<std::string> *path);

  /**
   * @brief 更新road-level级的路线，终点已事先设置，路线为成员变量
   * @param start 起点
   */
   bool UpdateRoadRoute(std::string start_id);

   bool UpdateRoute(std::string start_id);

   /**
   * @brief 获取最短路线
   * @return 以lane为单位的列表。如果存在变道行为，则两条lane都在vec中
   */
   bool ShortestPath(Node* start_node, Node* end_node,
                     std::vector<std::string> *path);

  /**
   * @brief 设置导航粒度
   *        0为道路级导航
   *        1为车道级导航
   *        2为同时支持道路和车道级导航
   * @param attention_level 导航粒度
   */
   void SetAttentionLevel(int attention_level);

  /**
   * @brief 设置属性约束
   * @param flag true:开启车道线虚实属性约束
   *             false：关闭车道线虚实属性约束
   */
   void SetAttributeConstraints(bool flag);

  /**
   * @brief 设置导航用户喜好
   * @param type 喜好类型，枚举。针对不同的类型进行权重的设置
   */
   void SetNavigationPreferences(NavigationPreferences type);

  /**
   * @brief 获取车道级导航的图结构
   * @return 车道级导航图结构
   */
   std::map<std::string, Node*> GetLaneGraph();

  /**
   * @brief 获取道路级导航的图结构
   * @return 道路级导航图结构
   */
   std::map<std::string, Node*> GetRoadGraph();

  /**
   * @brief 获取车道级导航路线结果
   * @return 车道级导航路线
   */
   std::vector<std::string> GetLanePath();

  /**
   * @brief 获取道路级导航路线结果
   * @return 道路级导航路线
   */
   std::vector<std::string> GetRoadPath();

  /**
   * @brief 获取车道目的地ID
   * @return 道路级导航路线
   */
  bool GetLaneDestinationID(std::string &id);

   /**
   * @brief 获取导航的proto msg，仅用于更新车道级导航路径
   * @return true 获取成功
   * @return false 获取失败
   */
  bool UpdateLaneRouteInNavProto(
    std::shared_ptr<ndm_proto::NavigationMsg> &nav_msg,
    std::string id, uint64_t stamp);

  /**
   * @brief 获取导航路径，根据当前的road route和lane route进行填写
   * @return true 获取成功
   * @return false 获取失败
   */
  bool GetRouteInNavProto(
    std::shared_ptr<ndm_proto::NavigationMsg> &nav_msg,
    std::string id, uint64_t stamp);

  bool ProcessDestination(
    std::shared_ptr<map_interface::StaticMapComponent> section_dest,
    std::string &lane_dest);

  bool UpdateLaneDestination();

  bool SetPreferedLaneID(int num);
  bool SetDecisionDistance(double dist);
  bool RespondToLaneChangeCommd(int cmd);

  /**
   * @brief 获取导航路径，根据当前的road route和lane route进行填写
   * @return true 获取成功
   * @return false 获取失败
   */
  void FeedRouteRoute(std::vector<std::string> road_route);

private:
  /**
   * @brief 获取最短路径后，通过Node获取路线
   *
   * @param start_node 起始顶点
   * @param end_node 终止顶点
   * @param path 路线id
   * @return true 获取路线成功
   * @return false 获取路线失败
   */
  bool GetPath(Node* start_node, Node* end_node,
                std::vector<std::string> *path);

  bool Component2Node(
        std::shared_ptr<map_interface::StaticMapComponent> component,
        std::map<std::string, Node*> graph);

  bool Component2Node(
    std::shared_ptr<map_interface::StaticMapComponent> component,
    Node *node, std::map<std::string, Node*> graph);

  void WeightClassification(
    std::vector<std::shared_ptr<map_interface::StaticMapComponent>> components);

  void UpdatePreferences(
    std::vector<std::shared_ptr<map_interface::StaticMapComponent>> components);

  void LaneChangeOnRamp(
    std::vector<std::shared_ptr<map_interface::StaticMapComponent>> components);
private:
  map_interface::Index *index_;  // 索引
  bool out_of_route_;  /// 脱离规划路线
  bool state_init_;  /// 更新地图块（云端-端端）
  bool state_lane_change_;  /// 记录是否需要换道，仅在初始化route后生效
  bool state_lane_init_;
  bool state_road_init_;

  Node* lane_destination_;         /// 记录目的地信息
  Node* road_destination_;         /// 记录目的地信息
  std::string cur_lane_id_;  /// 当前车道的id
  std::string cur_section_id_;  /// 当前所在的section/junction的id

  std::vector<std::string> road_route_;
  std::vector<std::string> lane_route_;

  std::map<std::string, Node*> road_graph_;
  std::map<std::string, Node*> lane_graph_;

  std::shared_ptr<ndm_proto::NavigationMsg> nav_msg_;

private:
  // 参数配置记录
  /// 是否受车道线虚实属性约束
  bool attribute_constraints_;
  /// 导航粒度，0-道路级，1-车道级，2-道路+车道级
  int attention_level_;
  /// 用户导航喜好记录
  NavigationPreferences preference_;
  /// 偏好车道。从右侧车道开始数（特殊车道如应急车道，非机车道不算），默认第二条
  int prefered_lane_id_;
  /// 前方决策提醒距离（一般在该距离下进行提取变道提醒），默认2km，单位:米
  double decision_distance_;
};

void GetLogicalConnection(
      const std::vector<std::shared_ptr<map_interface::StaticMapComponent>>
      components,
      std::vector<Eigen::Vector3d> &sps,
      std::vector<Eigen::Vector3d> &eps,
      std::vector<std::string> &types,
      map_interface::Index *index);

void ShowRoutePath(std::vector<std::string> path);
}  // namespace map_interface
#endif  // INTERFACE_ROUTE_H_
