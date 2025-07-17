/**
 * @file util.h
 * @author Han Fei (fei.han@horizon.ai)
 * @brief  Map Uitl APIs
 * @version 0.1
 * @date 2019-08-28
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_UTIL_H_
#define INTERFACE_UTIL_H_
#include "../../proto/ndm.pb.h"
#include "../interface/data_type.h"
#include "../interface/geometry_2d.h"
#include "../interface/geometry_3d.h"
#include "../interface/index.h"
#include "../interface/map_component.h"
#include "../version.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

namespace map_interface {
/**
 * @brief Set the CrossWalk Line object
 *
 * @param pCrossWalk    待处理的CrossWalk指针
 * @param id            CrossWalk中第id个child_border
 * @param mapComponent  根据pCrossWalk和id，构建CrossWalkLine类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetCrossWalkLine(ndm_proto::CrossWalk *pCrossWalk, int id,
                      std::shared_ptr<StaticMapComponent> mapComponent);
/**
 * @brief Set the Cross Walk object
 *
 * @param pCrossWalk    待处理的CrossWalk指针
 * @param mapComponent  根据pCrossWalk，构建rossWalk类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetCrossWalk(ndm_proto::CrossWalk *pCrossWalk,
                  std::shared_ptr<StaticMapComponent> mapComponent);
/**
 * @brief Set the Lane Marking object
 *
 * @param pLaneMarking  待处理的LaneMarking指针
 * @param mapComponent  根据pLaneMarking，构建LaneMarking类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetLaneMarking(ndm_proto::LaneMarking *pLaneMarking,
                    std::shared_ptr<StaticMapComponent> mapComponent);

/**
 * @brief Set the Speedbump object
 *
 * @param pSpeedBump    待处理的SpeedBump指针
 * @param mapComponent  根据pSpeedBump，构建SpeedBump类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetSpeedbump(ndm_proto::SpeedBump *pSpeedBump,
                  std::shared_ptr<StaticMapComponent> mapComponent);
/**
 * @brief Set the Traffic Sign object
 *
 * @param ptrafficSign  待处理的trafficSign指针
 * @param mapComponent  根据ptrafficSign，构建trafficSign类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetTrafficSign(ndm_proto::TrafficSign *ptrafficSign,
                    std::shared_ptr<StaticMapComponent> mapComponent);
/**
 * @brief Set the Traffic Light object
 *
 * @param ptrafficLight 待处理的trafficLight指针
 * @param mapComponent  根据ptrafficLight，构建trafficLight类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetTrafficLight(ndm_proto::TrafficLight *ptrafficLight,
                     std::shared_ptr<StaticMapComponent> mapComponent);
/**
 * @brief Set the Traffic Board object
 *
 * @param pTrafficBoard pTrafficBoard
 * @param mapComponent  根据pTrafficBoard，构建trafficBoard类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetTrafficBoard(ndm_proto::Board *pTrafficBoard,
                     std::shared_ptr<StaticMapComponent> mapComponent);
/**
 * @brief Set the Stop Line object
 *
 * @param pStopLine     待处理的StopLine指针
 * @param mapComponent  根据pStopLine，构建StopLine类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetStopLine(ndm_proto::StopLine *pStopLine,
                 std::shared_ptr<StaticMapComponent> mapComponent);
/**
 * @brief Set the Pole object
 *
 * @param pPole         待处理的Pole指针
 * @param mapComponent  根据pPole，构建Pole类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetPole(ndm_proto::Pole *pPole,
             std::shared_ptr<StaticMapComponent> mapComponent);
/**
 * @brief Set the Lane object
 *
 * @param pLaneLine     待处理的ndm_proto::LaneLine指针
 * @param id            Lane中第id段车道线
 * @param mapComponent  根据pLaneLine和id，构建LaneLine类型的component
 * @return true         设置成功
 * @return false        设置失败
 */
bool SetLaneLine(ndm_proto::LaneLine *pLaneLine,
                 std::shared_ptr<StaticMapComponent> mapComponent,
                 bool merge_pts = false);

bool mergeCurveLinePts(ndm_proto::CurveLine *in,
                       std::shared_ptr<ndm_proto::CurveLine> out);

template <typename T>
void TransformPoint(ndm_proto::Point *point, const Eigen::Matrix<T, 4, 4> &pose,
                    Eigen::Matrix<T, 6, 2> *min_max_point);

template <typename Scalar>
void TransformCrossWalk(ndm_proto::CrossWalk *pCrossWalk,
                        const Eigen::Matrix<Scalar, 4, 4> &pose,
                        Eigen::Matrix<Scalar, 6, 2> *min_max_point);

template <typename Scalar>
void TransformLaneLine(ndm_proto::LaneLine *pLaneLine,
                       const Eigen::Matrix<Scalar, 4, 4> &pose,
                       Eigen::Matrix<Scalar, 6, 2> *min_max_point);

template <typename Scalar>
void TransformLaneMarking(ndm_proto::LaneMarking *pLaneMarking,
                          const Eigen::Matrix<Scalar, 4, 4> &pose,
                          Eigen::Matrix<Scalar, 6, 2> *min_max_point);

template <typename Scalar>
void TransformPole(ndm_proto::Pole *pPole,
                   const Eigen::Matrix<Scalar, 4, 4> &pose,
                   Eigen::Matrix<Scalar, 6, 2> *min_max_point);

template <typename Scalar>
void TransformSpeedBump(ndm_proto::SpeedBump *pSpeedBump,
                        const Eigen::Matrix<Scalar, 4, 4> &pose,
                        Eigen::Matrix<Scalar, 6, 2> *min_max_point);

template <typename Scalar>
void TransformStopLine(ndm_proto::StopLine *pStopLine,
                       const Eigen::Matrix<Scalar, 4, 4> &pose,
                       Eigen::Matrix<Scalar, 6, 2> *min_max_point);

template <typename Scalar>
void TransformTrafficBoard(ndm_proto::Board *pTrafficBoard,
                           const Eigen::Matrix<Scalar, 4, 4> &pose,
                           Eigen::Matrix<Scalar, 6, 2> *min_max_point);

template <typename Scalar>
void TransformTrafficLight(ndm_proto::TrafficLight *pTrafficLight,
                           const Eigen::Matrix<Scalar, 4, 4> &pose,
                           Eigen::Matrix<Scalar, 6, 2> *min_max_point);

template <typename Scalar>
void TransformTrafficSign(ndm_proto::TrafficSign *pTrafficSign,
                          const Eigen::Matrix<Scalar, 4, 4> &pose,
                          Eigen::Matrix<Scalar, 6, 2> *min_max_point);

template <typename Scalar>
Eigen::Matrix<Scalar, 6, 2> TransformHdmap(
    ndm_proto::MapEnvMsg *pHdmap, const Eigen::Matrix<Scalar, 4, 4> &pose);

// coordinate transform from UTM to WGS84
void TransformPointFromUTMToWGS84(ndm_proto::Point *point, const char *UTMZone);

void TransformCurvelineFromUTMToWGS84(ndm_proto::CurveLine *pCurveline,
                                      const char *UTMZone);

void TransformPolygonFromUTMToWGS84(ndm_proto::Polygon *pPolygon,
                                    const char *UTMZone);

void TransformCylinderFromUTMToWGS84(ndm_proto::Cylinder *pCylinder,
                                     const char *UTMZone);

void TransformCrossWalkFromUTMToWGS84(ndm_proto::CrossWalk *pCrossWalk,
                                      const char *UTMZone);

void TransformLaneLineFromUTMToWGS84(ndm_proto::LaneLine *pLaneLine,
                                     const char *UTMZone);

void TransformLaneMarkingFromUTMToWGS84(ndm_proto::LaneMarking *pLaneMarking,
                                        const char *UTMZone);

void TransformPoleFromUTMToWGS84(ndm_proto::Pole *pPole, const char *UTMZone);

void TransformSpeedBumpFromUTMToWGS84(ndm_proto::SpeedBump *pSpeedBump,
                                      const char *UTMZone);

void TransformStopLineFromUTMToWGS84(ndm_proto::StopLine *pStopLine,
                                     const char *UTMZone);

void TransformTrafficBoardFromUTMToWGS84(ndm_proto::Board *pTrafficBoard,
                                         const char *UTMZone);

void TransformTrafficLightFromUTMToWGS84(ndm_proto::TrafficLight *pTrafficLight,
                                         const char *UTMZone);

void TransformTrafficSignFromUTMToWGS84(ndm_proto::TrafficSign *pTrafficSign,
                                        const char *UTMZone);

void TransformHdmapFromUTMToWGS84(ndm_proto::MapEnvMsg *pHdmap,
                                  const char *UTMZone);

// coordinate transform from WGS84 to UTM
void TransformPointFromWGS84ToUTM(ndm_proto::Point *point, int zoneID = -1);

void TransformCurvelineFromWGS84ToUTM(ndm_proto::CurveLine *pCurveline,
                                      int zoneID);

void TransformPolygonFromWGS84ToUTM(ndm_proto::Polygon *pPolygon, int zoneID);

void TransformCylinderFromWGS84ToUTM(ndm_proto::Cylinder *pCylinder,
                                     int zoneID);

void TransformCrossWalkFromWGS84ToUTM(ndm_proto::CrossWalk *pCrossWalk,
                                      int zoneID);

void TransformLaneLineFromWGS84ToUTM(ndm_proto::LaneLine *pLaneLine,
                                     int zoneID);

void TransformLaneMarkingFromWGS84ToUTM(ndm_proto::LaneMarking *pLaneMarking,
                                        int zoneID);

void TransformPoleFromWGS84ToUTM(ndm_proto::Pole *pPole, int zoneID);

void TransformSpeedBumpFromWGS84ToUTM(ndm_proto::SpeedBump *pSpeedBump,
                                      int zoneID);

void TransformStopLineFromWGS84ToUTM(ndm_proto::StopLine *pStopLine,
                                     int zoneID);

void TransformTrafficBoardFromWGS84ToUTM(ndm_proto::Board *pTrafficBoard,
                                         int zoneID);

void TransformTrafficLightFromWGS84ToUTM(ndm_proto::TrafficLight *pTrafficLight,
                                         int zoneID);

void TransformTrafficSignFromWGS84ToUTM(ndm_proto::TrafficSign *pTrafficSign,
                                        int zoneID);

void TransformHdmapFromWGS84ToUTM(ndm_proto::MapEnvMsg *pHdmap, int zoneID);

// 获取地图坐标
bool GetFirstPointOfHdMap(ndm_proto::MapEnvMsg *pHdmap,
                          ndm_proto::Point *pPoint);

bool GetFirstPointOfLaneline(ndm_proto::LaneLine *pLaneLine,
                             ndm_proto::Point *pPoint);

bool GetFirstPointOfCurvlline(ndm_proto::CurveLine *pCurveLine,
                              ndm_proto::Point *pPoint);

void ConvertOdomMem2Proto(const map_interface::Odometry &odom_mem,
                          ndm_proto::Odometry *odom_proto);

void ConvertOdomProto2Mem(const ndm_proto::Odometry &odom_proto,
                          map_interface::Odometry *odom_mem);
Eigen::Vector3d ConvertProto2Eigen(ndm_proto::Point p);
void ConvertProto2Eigen(ndm_proto::Point *p, Eigen::Vector3d &engine_p);
void ConvertProto2Eigen(std::shared_ptr<ndm_proto::Polygon> polygon,
                        std::vector<Eigen::Vector3d> &points);
void ConvertProto2Eigen(std::shared_ptr<ndm_proto::CurveLine> curveLine,
                        std::vector<Eigen::Vector3d> &points);

bool ConvertComponentToProto(
    std::vector<std::shared_ptr<StaticMapComponent>> &components,
    std::string timestamp, std::string projection, ndm_proto::MapEnvMsg *map);

bool ConvertComponentToProto(
    std::vector<std::shared_ptr<StaticMapComponent>> &components,
    std::shared_ptr<ndm_proto::MapEnvMsg> map);

bool ConvertComponentToProto(
    std::vector<std::shared_ptr<StaticMapComponent>> &components,
    ndm_proto::MapEnvMsg *map);

bool ConvertComponentToProto(std::shared_ptr<StaticMapComponent> &component,
                             ndm_proto::PhysicalLayer *physical_layer);

bool ConvertComponentToProto(std::shared_ptr<StaticMapComponent> &component,
                             ndm_proto::LogicalLayer *logical_layer);

bool ConvertComponentToProto(std::shared_ptr<StaticMapComponent> &component,
                             ndm_proto::TopologicalLayer *topological_layer);

bool ConvertComponentToProto(std::shared_ptr<StaticMapComponent> &component,
                             ndm_proto::DynamicLayer *dynamic_layer);

bool ConvertRouteToProto(std::vector<std::string> path, std::string id,
                         uint64_t stamp, float timecost, float distcost,
                         std::string link_id, ndm_proto::LaneRoute *route);

bool ConvertRouteToProto(std::vector<std::string> path, std::string id,
                         uint64_t stamp, float timecost, float distcost,
                         ndm_proto::RoadRoute *route);

bool ConvertNavigationToProto(ndm_proto::LaneRoute *&lane_route,
                              ndm_proto::RoadRoute *&road_route,
                              Eigen::Vector3d sp, Eigen::Vector3d ep,
                              std::string id, uint64_t stamp);

bool SettingMapHeader(ndm_proto::NDMHeader *pHead, std::string timestamp,
                      std::string projection, bool isfixed = false);

bool SettingMapHeader(ndm_proto::NDMHeader *pHead,
                      std::string date,
                      int64_t stamp,
                      std::string projection,
                      bool isfixed = false);

bool SetBBoxByBoundaryLine(std::shared_ptr<StaticMapComponent> comp,
                           map_interface::Index *map_index);

bool ResetBoundingBox(
    std::vector<std::shared_ptr<StaticMapComponent>> &components,
    map_interface::Index *map_index);

std::string GetVersion();
}  // namespace map_interface
#endif  // INTERFACE_UTIL_H_
