/**
 * @file geometry_3d.h
 * @author Han Fei (fei.han@horizon.ai)
 * @brief  3D Geometry API
 * @version 0.1
 * @date 2019-08-28
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_GEOMETRY_3D_H_
#define INTERFACE_GEOMETRY_3D_H_

// #include <glog/logging.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <memory>
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "../../proto/ndm.pb.h"

namespace map_interface {

class GEO_LINE3D {
 public:
  GEO_LINE3D() {}
  GEO_LINE3D(Eigen::Vector3f p1_, Eigen::Vector3f p2_);
  ~GEO_LINE3D() {}

  bool FixedPoint() { return p1 == p2; }
  void SetEndpointP1(Eigen::Vector3f p) { p1 = p; }
  void SetEndpointP2(Eigen::Vector3f p) { p2 = p; }

  // (x-p1[0])/a = (y-p1[1])/b = (z-p1[2])/c , p2-p1 = [a,b,c]
  Eigen::Vector3f vline;
  Eigen::Vector3f p1;
  Eigen::Vector3f p2;

  bool fixedpoint;
};

class GEO_POLYGON3D {
 public:
  explicit GEO_POLYGON3D(std::vector<Eigen::Vector3f> ps);
  ~GEO_POLYGON3D(){};

  std::vector<GEO_LINE3D> lines;
};

class GEO_CURVELINE3D {
 public:
  explicit GEO_CURVELINE3D(std::vector<Eigen::Vector3f> ps);
  ~GEO_CURVELINE3D(){};

  bool getSegment(int index, GEO_LINE3D &line);

  std::vector<GEO_LINE3D> lines;
};

class GEO_PLANE {
 public:
  GEO_PLANE(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);
  ~GEO_PLANE(){};

  Eigen::Vector4d coeffs;  // Ax + By + Cz + D = 0
  // normal vector
  Eigen::Vector3f vnormal;
};

class GeometryXYZ {
 public:
  GeometryXYZ() {}
  ~GeometryXYZ() {}

 public:
  float DistanceXYZ2XYZ(Eigen::Vector3f p1, Eigen::Vector3f p2);

  bool LinePlaneCross(GEO_LINE3D line, GEO_PLANE plane,
                      Eigen::Vector3f *crossp);
};

class AABB {
 public:
  AABB();

  explicit AABB(const std::vector<Eigen::Vector3d> &points);

  void GetMinMaxVertex(Eigen::Vector3d *min_vertex,
                       Eigen::Vector3d *max_vertex);

 private:
  void ComputeAABB(const std::vector<Eigen::Vector3d> &points);

 private:
  Eigen::Vector3d max_vertex_;  // AABB 最大顶点
  Eigen::Vector3d min_vertex_;  // AABB 最小顶点
};

/**
 * @brief 有向包围盒(Oriented Bounding Box)
 *
 */
class OBB {
 public:
  OBB();
  /**
   * @brief Construct a new OBB object, 并根据参数points来计算这些点的OBB
   *
   * @param points 点序列
   */
  explicit OBB(const std::vector<Eigen::Vector3d> &points);
  /**
   * @brief 计算包围points的OBB
   *
   * @param points
   */
  void ComputeOBB(const std::vector<Eigen::Vector3d> &points);
  /**
   * @brief 获得OBB的八个顶点;
   *
   * @param corners 保存Bounding Box的八个顶点
   * 顺序:Z轴正方向平面的四个顶点(俯视,左下角开始,逆时针);
   * 然后是Z轴负方向的四个顶点(俯视,左下角开始,逆时针))
   */
  void GetCorners(std::vector<Eigen::Vector3d> *corners);

  /**
   * @brief 判断一个点是否在OBB中
   *
   * @param point     待判断的Point
   * @param is2D      是否只考虑X和Y方向
   * @return true     在OBB中
   * @return false    不在OBB中
   */
  bool ContainPoint(const Eigen::Vector3d &point, bool is2D);

 private:
  Eigen::Matrix3d GetOBBOrientation(const std::vector<Eigen::Vector3d> &points);
  void ComputeExtAxis();

 public:
  Eigen::Vector3d center_;      // OBB 中心点
  Eigen::Vector3d max_vertex_;  // OBB 最大顶点
  Eigen::Vector3d min_vertex_;  // OBB 最小顶点
  Eigen::Vector3d xAxis_;       // OBB x轴方向单位矢量
  Eigen::Vector3d yAxis_;       // OBB y轴方向单位矢量
  Eigen::Vector3d zAxis_;       // OBB z轴方向单位矢量
  Eigen::Vector3d extents_;     // 存储3个1/2边长, 半长, 半宽, 半高

  Eigen::Vector3d extentX_;  // xAxis_ * extents_.x
  Eigen::Vector3d extentY_;  // yAxis_ * extents_.y
  Eigen::Vector3d extentZ_;  // zAxis_ * extents_.z
};

void ComputePolygonNormal(std::shared_ptr<ndm_proto::Polygon> pPolygon);
ndm_proto::Point GetPolygonCenter(std::shared_ptr<ndm_proto::Polygon> pPolygon);
ndm_proto::Point GetPolygonMaxVertex(
  std::shared_ptr<ndm_proto::Polygon> pPolygon);
ndm_proto::Point GetPolygonMinVertex(
  std::shared_ptr<ndm_proto::Polygon> pPolygon);
ndm_proto::Point GetPointSetMaxVertex(const std::vector<ndm_proto::Point> &pts);
ndm_proto::Point GetPointSetMinVertex(const std::vector<ndm_proto::Point> &pts);

//判断当前点是否在任意姿态的立方体内
bool InBoundingBox(std::vector<Eigen::Vector3d> pvertex, Eigen::Vector3d p);

// 输入当前位姿，bounding
// box的长,中间层距底部高度，中间层距顶部高度，半宽，输出bounding
// box最小顶点与最大顶点
void GetBoundingBox1(Eigen::Matrix4d pose, Eigen::Vector4d dbtw,
                     std::vector<Eigen::Vector3d> *pvertex);

// 输入当前位姿，以pose为中心，bounding box的前后左右上下距离，
// 输出bounding box的8个顶点
void GetBoundingBox(Eigen::Matrix4d pose, std::vector<Eigen::Vector3d> *pvertex,
                    std::vector<float> fblrud = {5, 5, 5, 5, 5, 5});

/**
 * @brief 判断两个double是否相等
 *
 * @param d1
 * @param d2
 * @return true 相等
 * @return false 不相等
 */
bool IsEqual(double d1, double d2);
/**
 * @brief   计算3D点A到点B的距离
 *
 * @param ptA   点A
 * @param ptB   点B
 * @return double   点A到点B的距离
 */
double DistanceOfPointToPoint(const Eigen::Vector3d &ptA,
                              const Eigen::Vector3d &ptB);

/**
 * @brief   3D点S到直线AB的距离
 *
 * @param linePtA   直线上的点A
 * @param linePtB   直线上的点B
 * @param ptS   3D点S
 * @return double   返回3D点S到直线AB的距离
 */
double DistanceOfPointToLine(const Eigen::Vector3d &linePtA,
                             const Eigen::Vector3d &linePtB,
                             const Eigen::Vector3d &ptS);

/**
 * @brief   计算3D点S在直线AB上的投影点
 *
 * @param linePtA   直线上的点A
 * @param linePtB   直线上的点B
 * @param ptS   3D点S
 * @return Eigen::Vector3d  返回3D点S在直线AB上的投影点
 */
Eigen::Vector3d PointProjectionToLine(const Eigen::Vector3d &linePtA,
                                      const Eigen::Vector3d &linePtB,
                                      const Eigen::Vector3d &ptS);

/**
 * @brief   计算3D点S到线段AB的距离
 *
 * @param segPtA    线段的端点A
 * @param segPtB    线段的端点B
 * @param ptS   3D点S
 * @param dist  3D点S到线段AB的距离，也是3D点S到ptNearest的距离
 * @param ptNearest 若3D点S投影在线段AB内部，则ptNearest为投影点，
 *      若投影到线段外部，则为端点A、B中距离较近的点
 */
void DistanceOfPointToSegment(const Eigen::Vector3d &segPtA,
                              const Eigen::Vector3d &segPtB,
                              const Eigen::Vector3d &ptS, double *dist,
                              Eigen::Vector3d *ptNearest);

/**
 * @brief   计算3D点S到多边形的距离
 *
 * @param ptS   3D点S
 * @param polyPoints    多边形的顶点集合，按逆时针(or 顺时针?)顺序存储
 * @param dist  点到多边形各条边的最近距离
 * @param ptNearest 与点S距离最近的多边形边上，距离最近的点
 * @param nSegIndex 与点S距离最近的多边形边的index(范围：0-polyPoints.size()-1)
 */
void DistanceOfPointToPolyLine(const Eigen::Vector3d &ptS,
                               const std::vector<Eigen::Vector3d> &polyPoints,
                               double *dist, Eigen::Vector3d *ptNearest,
                               int *nSegIndex);

/**
 * @brief   计算直线line1 到直线line2 的平均距离
 *
 * @param vecLine1  line1上的3D点
 * @param vecLine2  line2上的3D点
 * @return double   直线line1 到直线line2 的平均距离
 */
double DistanceMeanOfLineToLine(const std::vector<Eigen::Vector3d> &vecLine1,
                                const std::vector<Eigen::Vector3d> &vecLine2);

/**
 * @brief   计算线到射线的最近距离
 *
 * @param vecLine   直线上的一系列点
 * @param rayAxis   射线方向
 * @param rayOrigin 射线起点
 * @param dist      直线到射线的最近距离
 * @param nSegNum   最近距离点所在的线段在直线vecLine的index
 * @param dSegRatio 最近距离点在它所在线段
 *                  (vecLine[nSegNum], vecLine[nSegNum + 1])的位置
 * ptNearest=vecLine[nSegNum]+dSegRatio*(vecLine[nSegNum + 1]-vecLine[nSegNum])
 * @param ptNearest 最近距离点在它所在线段(vecLine[nSegNum],vecLine[nSegNum+1])
 *                  的位置
 */
void DistanceLineToRay(const std::vector<Eigen::Vector3d> &vecLine,
                       const Eigen::Vector3d &rayAxis,
                       const Eigen::Vector3d &rayOrigin, double *dist,
                       int *nSegNum, double *dSegRatio,
                       Eigen::Vector3d *ptNearest);

/**
 * @brief   线段AB到射线的最近距离
 *
 * @param ptA       线段AB的端点A
 * @param ptB       线段AB的端点B
 * @param rayAxis   射线方向
 * @param rayOrigin 射线起点
 * @param dist      线段到射线的最近距离
 * @param dSegRatio 最近距离点在线段AB上的位置
 * @param ptNearest 最近距离点在线段AB上的位置，
 *                  ptNearest = ptA + dSegRatio * (ptB - ptA)
 */
void DistanceSegToSeg(const Eigen::Vector3d &ptA, const Eigen::Vector3d &ptB,
                      const Eigen::Vector3d &rayAxis,
                      const Eigen::Vector3d &rayOrigin, double *dist,
                      double *dSegRatio, Eigen::Vector3d *ptNearest);

/**
 * @brief   计算直线的采样点距离射线的最短距离
 *
 * @param vecLine   存储直线的采样点
 * @param rayAxis   射线方向
 * @param rayOrigin 射线起点
 * @param dist      直线上采样点到射线的最近距离
 * @param nSegNum   最近距离采样点在vecline中的下标
 * @param dSegRatio 始终等于1
 * @param ptNearest 最近距离点vecLine[nSegNum]
 */
void DistanceLineNodeToRay(const std::vector<Eigen::Vector3d> &vecLine,
                           const Eigen::Vector3d &rayAxis,
                           const Eigen::Vector3d &rayOrigin, double *dist,
                           int *nSegNum, double *dSegRatio,
                           Eigen::Vector3d *ptNearest);

/**
 * @brief   计算线段AB到线段CD的距离
 *
 * @param ptA   线段AB的端点A
 * @param ptB   线段AB的端点B
 * @param ptC   线段CD的端点C
 * @param ptD   线段CD的端点D
 * @param dist  两条线段的最近距离
 * @param dRatioAB  线段AB中最近距离对应点S在AB上的位置 AS = A + dRatioAB * AB
 * @param dRatioCD  线段CD中最近距离对应点T在CD上的位置 CT = C + dRatioCD * CD
 * @param bToRay    是否将线段CD是视为以C为起点，方向向量为CD的射线
 */
void DistanceSegToSeg(const Eigen::Vector3d &ptA, const Eigen::Vector3d &ptB,
                      const Eigen::Vector3d &ptC, const Eigen::Vector3d &ptD,
                      double *dist, double *dRatioAB, double *dRatioCD,
                      bool bToRay = false);

/**
 * @brief   计算线段AB到线段CD的距离
 *
 * @param x1    线段AB的端点A.x
 * @param y1    线段AB的端点A.y
 * @param z1    线段AB的端点A.z
 * @param x2    线段AB的端点B.x
 * @param y2    线段AB的端点B.y
 * @param z2    线段AB的端点B.z
 * @param x3    线段CD的端点C.x
 * @param y3    线段CD的端点C.y
 * @param z3    线段CD的端点C.z
 * @param x4    线段CD的端点D.x
 * @param y4    线段CD的端点D.y
 * @param z4    线段CD的端点D.z
 * @param dist  两条线段的最近距离
 * @param dRatio12  线段AB中最近距离对应点S在AB上的位置 AS = A + dRatioAB * AB
 * @param dRatio34  线段CD中最近距离对应点T在CD上的位置 CT = C + dRatioCD * CD
 * @param bToRay    是否将线段CD是视为以C为起点，方向向量为CD的射线
 */
void DistanceSegToSeg(double x1, double y1, double z1, double x2, double y2,
                      double z2, double x3, double y3, double z3, double x4,
                      double y4, double z4, double *dist, double *dRatio12,
                      double *dRatio34, bool bToRay = false);

/**
 * @brief       计算线段AB与线段CD的交点
 *              交点在两个线段中间才返回true；在任何一个线段的起点前或尾点后都返回false
 *
 * @param ptA   起点A
 * @param ptB   尾点B
 * @param ptC   起点C
 * @param ptD   尾点D
 * @param dist  3D空间中两条线段的最近距离
 * @param dRatioAB  交点在线段AB之间的比率，
 *                  设交点为S，则S = A + dRatioAB * AB,(0<=dRatioAB<=1)
 * @param dRatioCD  交点在线段CD之间的比率，
 *                  设交点为T，则T = C + dRatioCD * CD,(0<=dRatioCD<=1)
 * @return true     交点在线段AB、CD内部
 * @return false    交点在线段AB外部或线段CD外部
 */
bool CommonIntersectSegToSeg(const Eigen::Vector3d &ptA,
                             const Eigen::Vector3d &ptB,
                             const Eigen::Vector3d &ptC,
                             const Eigen::Vector3d &ptD, double *dist,
                             double *dRatioAB, double *dRatioCD);
/**
 * @brief       计算线段AB与线段CD的交点
 *
 * @param x1    A.x
 * @param y1    A.y
 * @param z1    A.z
 * @param x2    B.x
 * @param y2    B.y
 * @param z2    B.z
 * @param x3    C.x
 * @param y3    C.y
 * @param z3    C.z
 * @param x4    D.x
 * @param y4    D.y
 * @param z4    D.z
 * @param dist  3D空间中两条线段的最近距离
 * @param dRatio12  交点在线段AB之间的比率，
 *                  设交点为S，则S = A + dRatio12*AB,(0<=dRatio12<=1)
 * @param dRatio34  交点在线段CD之间的比率，
 *                  设交点为T，则T = C + dRatio34*CD,(0<=dRatio34<=1)
 * @return true     交点在线段AB、CD内部
 * @return false    交点在线段AB外部或线段CD外部
 */
bool CommonIntersectSegToSeg(double x1, double y1, double z1, double x2,
                             double y2, double z2, double x3, double y3,
                             double z3, double x4, double y4, double z4,
                             double *dist, double *dRatio12, double *dRatio34);

//
/**
 * @brief   已知平面法向量vecNormal及其上任一点ptPlane，
 *          求目标点ptTarget到平面的距离
 *          也就是求向量(ptPlane->ptTarget)在法向量上的投影长度
 * @param vecNormal     平面法向量
 * @param ptPlane       平面上一点
 * @param ptTarget      目标点
 * @return double       目标点到平面的距离
 */
double DistancePointToPlane(const Eigen::Vector3d &vecNormal,
                            const Eigen::Vector3d &ptPlane,
                            const Eigen::Vector3d &ptTarget);

/**
 * @brief   计算目标点在平面上的投影点
 *
 * @param vecNormal     平面法向量
 * @param ptPlane       平面上一点
 * @param ptTarget      目标点
 * @return Eigen::Vector3d  返回目标点在平面上的投影点
 */
Eigen::Vector3d PointProjectionToPlane(const Eigen::Vector3d &vecNormal,
                                       const Eigen::Vector3d &ptPlane,
                                       const Eigen::Vector3d &ptTarget);

/**
 * @brief   判断点query是否在直径为segStart-segEnd的圆内部
 *
 * @param segStart  直径起点
 * @param segEnd    直径终点
 * @param ptQuery   目标点
 * @return true     目标点在圆内部
 * @return false    目标点在圆上或外部
 */
bool InCircle(const Eigen::Vector3d &segStart, const Eigen::Vector3d &segEnd,
              const Eigen::Vector3d &ptQuery);

/**
 * @brief 返回向量vec对应的单位向量
 *
 * @param vec   向量vec
 * @return Eigen::Vector3d  向量vec对应的单位向量
 */
Eigen::Vector3d NormalizeVec(const Eigen::Vector3d &vec);

/**
 * @brief   返回向量vec的模长
 *
 * @param vec   向量vec
 * @return double   向量vec的模长
 */
double NormalValue(const Eigen::Vector3d &vec);

/**
 * @brief 计算从向量before旋转到向量after的旋转角度(单位为弧度)
 *
 * @param before    旋转前向量
 * @param after     旋转后向量
 * @return double   旋转角度（单位为弧度）
 */
double RotationAngle(const Eigen::Vector3d &before,
                     const Eigen::Vector3d &after);

/**
 * @brief   计算从向量before旋转到向量after的旋转轴
 *
 * @param before    旋转前向量
 * @param after     旋转后向量
 * @return Eigen::Vector3d  旋转轴
 */
Eigen::Vector3d RotationAxis(const Eigen::Vector3d &before,
                             const Eigen::Vector3d &after);

//
/**
 * @brief       根据XYZ平移求平移矩阵 返回指针用完free
 *
 * @param dx    x轴平移
 * @param dy    y轴平移
 * @param dz    z轴平移
 * @return double*  返回平移矩阵的double数组表示
 */
double *TranslateMatrix(double dx, double dy, double dz);

bool UpsampleCurveLinePts(const std::vector<Eigen::Vector3d> &origin_line_pts,
                          std::vector<Eigen::Vector3d> &dest_line_pts,
                          const double &sample_dist);

bool BezierCurveLineSmooth(const std::vector<Eigen::Vector3d> &origin_line_pts,
                           std::vector<Eigen::Vector3d> &smoothed_line_pts);

/**
 * @brief   判断点是否在任意多边形内部
 *
 * @param ptS        目标点
 * @param polyPoints 多边形点
 * @return true     目标点在多边形内部或多边形上
 * @return false    目标点在多边形外部
 */
bool InPolygon(const Eigen::Vector3d &ptS,
               const std::vector<Eigen::Vector3d> &polyPts);
}  // end namespace map_interface
#endif  // INTERFACE_GEOMETRY_3D_H_
