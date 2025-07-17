/**
 * @file gen_logical_relation.h
 * @author Fei Han (fei.han@horizon.ai)
 * @brief 生成逻辑关系
 * @version 0.1
 * @date 2019-10-12
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_GEN_LOGICAL_RELATION_H_
#define INTERFACE_GEN_LOGICAL_RELATION_H_
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include "../interface/index.h"
#include "../interface/io.h"
#include "../interface/map_component.h"

namespace map_interface {
/* ----------------------- 顶层 API ----------------------------*/
/**
 * @brief 预处理车道线
 * 1、对laneline中的line进行排序
 * 2、对laneline进行2米上采样
 * 3、滤除长度小于1的车道线 TODO（siyu）改为累计弧长
 *
 * @param components 地图元素
 * @param lanelines 处理后的车道线
 */
bool PreProcessLaneLine(
    std::vector<std::shared_ptr<StaticMapComponent>> components,
    std::vector<std::shared_ptr<StaticMapComponent>>& lanelines);
/**
* @brief 根据laneline的连接关系，生成联通分量
*
* @param laneline_comps 待处理的laneline
* @param res 联通分量存储
* @param index 地图索引
*/
void ClusterLaneLines(
    std::vector<std::shared_ptr<StaticMapComponent>>& laneline_comps,
    std::vector<std::vector<std::shared_ptr<StaticMapComponent>>>* res,
    map_interface::Index* index);

/**
* @brief 生成stopline和section之间的逻辑关系(stopline属于哪一个section)
*
* @param components
* @param section_components
*/
void GenStoplineLogicalRelation(
    const std::vector<std::shared_ptr<StaticMapComponent>>& components,
    std::vector<std::shared_ptr<LogicalMapComponent>>& section_components);

void GenSpeedbumpAndSectionLR(
    const std::vector<std::shared_ptr<StaticMapComponent>>& components,
    std::vector<std::shared_ptr<LogicalMapComponent>>& section_components);

void GenPoleAndSectionLR(
    const std::vector<std::shared_ptr<StaticMapComponent>>& components,
    std::vector<std::shared_ptr<LogicalMapComponent>>& section_components);

void GenBoardAndSectionLR(
    const std::vector<std::shared_ptr<StaticMapComponent>>& components,
    std::vector<std::shared_ptr<LogicalMapComponent>>& section_components);

void GenLanemarkingAndSectionLR(
    const std::vector<std::shared_ptr<StaticMapComponent>>& components,
    std::vector<std::shared_ptr<LogicalMapComponent>>& section_components,
    map_interface::Index* map_index);

void GenSignAndSectionLR(
    const std::vector<std::shared_ptr<StaticMapComponent>>& components,
    std::vector<std::shared_ptr<LogicalMapComponent>>& section_components,
    map_interface::Index* map_index);

void GenLightAndJunctionLR(
    const std::vector<std::shared_ptr<StaticMapComponent>>& components,
    std::vector<std::shared_ptr<LogicalMapComponent>>& junction_components);

void GenPolygonNormalInSection(
    std::shared_ptr<LogicalMapComponent> pSection_component,
    map_interface::Index* map_index);

void GenPolygonNormalInJunction(
    std::shared_ptr<LogicalMapComponent> pJunction_component,
    map_interface::Index* map_index);

/* ----------------------- 内部 API -----------------------------*/
/**
 * @brief 将车道线中的虚线段沿前进方向进行排序
 *
 * @param laneline_proto 待排序的车道线
 */
void SortLinesofLaneLine(ndm_proto::LaneLine* laneline_proto);

/**
 * @brief 对车道线中的点进行上采样
 *
 * @param laneline_comp 待处理的车道线
 * @param sample_dist 采样点之间的间隔
 */
void UpsampleLaneline(std::shared_ptr<StaticMapComponent> laneline_comp,
                      const double& sample_dist);
/**
* @brief 深度遍历
*
* @param laneline_comps 待处理的laneline
* @param res 联通分量存储
* @param index 地图索引
*/
void DFS(std::shared_ptr<StaticMapComponent> component,
         std::vector<std::shared_ptr<StaticMapComponent>>* res,
         std::map<std::string, bool>* is_visited, map_interface::Index* index);
/* --------------------------------------------------------------*/
/**
 * @brief Get the Refline Of Section object
 *
 * @param pSection_component 待处理的section component
 * @param pRefline_component 保存该section的refline component
 * @param map_index 用于ID索引搜索
 */
std::shared_ptr<StaticMapComponent> GetReflineOfSection(
    std::shared_ptr<LogicalMapComponent> pSection_component,
    map_interface::Index* map_index);
/**
 * @brief 根据指定方向来校正LaneLine中lines点的顺序
 *
 * @param refline_dir
 * @param laneline_pts
 */
void SortLinesofLaneLine(
    const Eigen::Vector3d& refline_dir,
    std::vector<std::vector<Eigen::Vector3d>>& laneline_pts);
/**
 * @brief 预处理各个section的车道线:
 * 1. 首先,假设section对应的refline中点的方向与前进方向一致
 * 2. 使section中各条LaneLine中curvelines的顺序与前进方向一致
 * 3. 使各条curveline中点的顺序与前进方向一致
 * 4. section的lane之间的顺序按从左往右的顺序排列
 *
 * @param section_components
 * @param map_index 用于ID索引搜索
 */
void PreProcessLanesOfSections(
    std::vector<std::shared_ptr<LogicalMapComponent>>* section_components,
    map_interface::Index* map_index);

/**
 * @brief Get the Nearest Section object
 *
 * @param pt
 * @param section_components
 * @param max_dist_th
 * Object距离section的最大距离阈值，小于阈值的，才被归属到section中
 * @return LogicalMapComponent* 返回距离点pt最近的section
 */
std::shared_ptr<LogicalMapComponent> GetNearestSection(
    const Eigen::Vector3d& pt,
    const std::vector<std::shared_ptr<LogicalMapComponent>> section_components,
    const double& max_dist_th = 20.0);

/**
 * @brief 计算点到线的累积弧长以及距离
 *
 * @param pt 3D点
 * @param line_pts 线的采样点
 * @param line_arc_lengths 线的弧长
 * @param length 点到线上的纵向弧长
 * @param width 点到线的横向距离
 * @return true 计算成功
 * @return false 计算失败
 */
bool CalLengthAndWidthOfPointToLine(
    const Eigen::Vector3d& pt, const std::vector<Eigen::Vector3d>& line_pts,
    const std::vector<double> line_arc_lengths, double* length, double* width);

/**
 * @brief 根据section、Lane之间的连接关系，连接车道线
 *
 * @param section_comps 待处理的sections
 * @param map_index 地图索引
 */
void ConnectLaneLines(
    std::vector<std::shared_ptr<LogicalMapComponent>>& section_comps,
    map_interface::Index* map_index);

/**
 * @brief 判断两条车道线是否相互连接
 *
 * @param laneline_comp_i   车道线i
 * @param laneline_comp_j   车道线j
 * @param long_dist_th      纵向距离阈值(m)
 * @param trans_dist_th     横向距离阈值(m)
 * @return int              返回0 不连接
 *                          返回1 连接方向为i->j
 *                          返回2 连接方向为j->i
 */
int isConnectedLaneline(std::shared_ptr<StaticMapComponent> laneline_comp_i,
                        std::shared_ptr<StaticMapComponent> laneline_comp_j,
                        const double& long_dist_th = 55,
                        const double& trans_dist_th = 1.5);

/**
 * @brief 判断两条车道线是否左右邻接
 *
 * @param laneline_comp_i 车道线i
 * @param laneline_comp_j 车道线j
 * @return int
 */
int isAdjLaneline(std::shared_ptr<StaticMapComponent> laneline_comp_i,
                  std::shared_ptr<StaticMapComponent> laneline_comp_j);

/**
 * @brief 连接两条车道线
 *
 * @param pred_laneline_proto 前驱车道线
 * @param succ_laneline_proto 后继车道线
 */
void ConnectLaneLine(ndm_proto::LaneLine* pred_laneline_proto,
                     ndm_proto::LaneLine* succ_laneline_proto,
                     const double& dist_th);

/**
 * @brief 将车道线的起点或终点平滑连接至给定点
 *
 * @param laneline_proto   车道线
 * @param intersect_pt     给定点
 * @param is_left          true: 车道线在给定点的左侧,
 *                         则将车道线的终点平滑连接至该点
 *                         false:
 *                         车道线在给定点的右侧，将车道线的起点平滑连接至该点
 */
void ConnectLaneLineToPoint(ndm_proto::LaneLine* laneline_proto,
                            const Eigen::Vector3d& intersect_pt,
                            const bool& is_left);
/**
 * @brief 根据分界线来切割车道线
 *
 * @param pLaneline_proto 待切割的车道线
 * @param divide_pt1 分界线的点
 * @param divide_pt2 分界线的点
 * @param is_left 车道线在分界线的左侧为true，否则为false
 * @param dotted_line_len_th
 * 根据距离判断是否是虚线段，如果是虚线的话，不延长到边界处
 */
void CutLaneline(ndm_proto::LaneLine* pLaneline_proto,
                 const Eigen::Vector3d& divide_pt1,
                 const Eigen::Vector3d& divide_pt2, const bool& is_left,
                 const double& dotted_line_len_th = 10.0);

bool CalLaneLine2LanelineDis(
    std::shared_ptr<StaticMapComponent> laneline_comp_i,
    std::shared_ptr<StaticMapComponent> laneline_comp_j, double* dis);
/**
 * @brief 计算车道线之间的平均横向距离
 *
 * @param laneline_comp_i 车道线 i
 * @param laneline_comp_j 车道线 j
 * @param dis 两条车道线之间的平均距离
 * @return true 计算成功
 * @return false 计算失败
 */
bool CalLaneLine2LanelineDis(ndm_proto::LaneLine* laneline_comp_i,
                             ndm_proto::LaneLine* laneline_comp_j, double* dis);

inline Eigen::Vector3d PointProto2Eigen(const ndm_proto::Point pt_proto) {
  Eigen::Vector3d pt_eigen(pt_proto.x(), pt_proto.y(), pt_proto.z());
  return pt_eigen;
}

}  // end namespace map_interface
#endif  // INTERFACE_GEN_LOGICAL_RELATION_H_
