/**
 * @file geometry_2d.h
 * @author Han Fei (fei.han@horizon.ai)
 * @brief  2D Geometry API
 * @version 0.1
 * @date 2019-08-28
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_GEOMETRY_2D_H_
#define INTERFACE_GEOMETRY_2D_H_

#include <iostream>
#include <vector>
// #include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace map_interface {
class GEO_LINE2D {
 public:
  GEO_LINE2D() {}
  GEO_LINE2D(Eigen::Vector2f p1, Eigen::Vector2f p2);
  ~GEO_LINE2D() {}

  bool FixedPoint() { return p1 == p2; }
  void SetEndpointP1(Eigen::Vector2f p) { p1 = p; }
  void SetEndpointP2(Eigen::Vector2f p) { p2 = p; }

  Eigen::Vector3d coeffs;
  Eigen::Vector2f p1;
  Eigen::Vector2f p2;

  bool fixedpoint;
};

class GEO_POLYGON2D {
 public:
  GEO_POLYGON2D() {}
  explicit GEO_POLYGON2D(std::vector<Eigen::Vector2f> ps);
  ~GEO_POLYGON2D(){};

  bool FixedPoint();
  bool FixedLineSeg(GEO_LINE2D &lineseg);

  std::vector<GEO_LINE2D> lines;

  bool fixedpoint;
  bool fixedlineseg;

  GEO_LINE2D fixed_lineseg;
};

class GEO_CURVELINE2D {
 public:
  GEO_CURVELINE2D() {}
  explicit GEO_CURVELINE2D(std::vector<Eigen::Vector2f> ps);
  ~GEO_CURVELINE2D(){};

  bool FixedPoint();
  bool FixedLineSeg(GEO_LINE2D &lineseg);

  std::vector<GEO_LINE2D> lines;

  bool fixedpoint;
  bool fixedlineseg;

  GEO_LINE2D fixed_lineseg;
};

class GEO_RECTANGLE2D {
 public:
  GEO_RECTANGLE2D() {}
  //  left - top - right - bottom
  GEO_RECTANGLE2D(float left, float top, float right, float bottom);
  ~GEO_RECTANGLE2D() {}

  void SetLeftTop(Eigen::Vector2f v) { leftTop = v; }
  void SetRightTop(Eigen::Vector2f v) { rightTop = v; }
  void SetLeftBottom(Eigen::Vector2f v) { leftBottom = v; }
  void SetRightBottom(Eigen::Vector2f v) { rightBottom = v; }

  Eigen::Vector2f leftTop;
  Eigen::Vector2f rightTop;
  Eigen::Vector2f rightBottom;
  Eigen::Vector2f leftBottom;
};

class GeometryXY {
 public:
  GeometryXY() {}
  ~GeometryXY() {}

 public:
  float DistanceXY2XY(Eigen::Vector2f p1, Eigen::Vector2f p2);
  float DistanceXY2Line(Eigen::Vector2f p, GEO_LINE2D line);
  float DistanceXY2Line(Eigen::Vector2f p, GEO_LINE2D line,
                        Eigen::Vector2f *proj);
  float DistanceXY2LineSeg(Eigen::Vector2f p, GEO_LINE2D line);
  float DistanceXY2LineSeg(Eigen::Vector2f p, GEO_LINE2D line,
                           Eigen::Vector2f *proj);
  float DistanceXY2Polygon(Eigen::Vector2f p, GEO_POLYGON2D polygon);
  float DistanceXY2PolygonBorder(Eigen::Vector2f p, GEO_POLYGON2D polygon);
  float DistanceXY2PolygonBorder(Eigen::Vector2f p, GEO_POLYGON2D polygon,
                                 Eigen::Vector2f *proj);
  float DistanceXY2CurveLineSeg(Eigen::Vector2f p, GEO_CURVELINE2D curveline);
  float DistanceXY2CurveLineSeg(Eigen::Vector2f p, GEO_CURVELINE2D curveline,
                                Eigen::Vector2f *proj);
  // float DistanceRectangle2Rectancle(GEO_RECTANGLE2D rect1, GEO_RECTANGLE2D
  // rect2);

  float DistanceXY2CurveLineRay(Eigen::Vector2f p, GEO_CURVELINE2D curveline,
                                Eigen::Vector2f *proj);

  float DistanceXY2CurveLineSegWithIndex(Eigen::Vector2f p,
                                         GEO_CURVELINE2D curveline, int *index,
                                         Eigen::Vector2f *proj);

  bool XYInPolygon(Eigen::Vector2f p, GEO_POLYGON2D polygon);
  bool XYInLine(Eigen::Vector2f p, GEO_LINE2D line);
  bool XYInLineSeg(Eigen::Vector2f p, GEO_LINE2D line);
  bool XYInCurveLine(Eigen::Vector2f p, GEO_CURVELINE2D curveline);

  Eigen::Vector2f ProjectionXY2Line(Eigen::Vector2f p, GEO_LINE2D line);
  Eigen::Vector2f ProjectionXY2LineSeg(Eigen::Vector2f p, GEO_LINE2D lineseg);
  Eigen::Vector2f ProjectionXY2CurveLine(Eigen::Vector2f p,
                                         GEO_CURVELINE2D curveline);
  Eigen::Vector2f ProjectionXY2Polygon(Eigen::Vector2f p,
                                       GEO_POLYGON2D polygon);

  bool LineLineCross(GEO_LINE2D line1, GEO_LINE2D line2, Eigen::Vector2f *p);
  bool LineLineSegCross(GEO_LINE2D line, GEO_LINE2D lineseg,
                        Eigen::Vector2f *p);
  bool LineSegLineSegCross(GEO_LINE2D lineseg1, GEO_LINE2D lineseg2,
                           Eigen::Vector2f *p);
  bool LineLineCollinear(GEO_LINE2D line1, GEO_LINE2D line2);
  bool LineLineSegCollinear(GEO_LINE2D line, GEO_LINE2D lineseg);
  bool LineSegLineSegCollinear(GEO_LINE2D lineseg1, GEO_LINE2D lineseg2);

  bool LinePolygonCross(GEO_LINE2D line, GEO_POLYGON2D polygon,
                        Eigen::Vector2f *p);
  bool LineSegPolygonCross(GEO_LINE2D lineseg, GEO_POLYGON2D polygon,
                           Eigen::Vector2f *p);

  bool LineCurveLineCross(GEO_LINE2D line, GEO_CURVELINE2D curveline,
                          Eigen::Vector2f *p);
  bool LineSegCurveLineCross(GEO_LINE2D lineseg, GEO_CURVELINE2D curveline,
                             Eigen::Vector2f *p);

 private:
};
// 除内部接口外，传入的vector2f的数值需要是非负整数
class GeometryUV {
 public:
  explicit GeometryUV(float a = 1, float b = 1.5);
  ~GeometryUV() {}

 public:
  float DistanceUV2UV(Eigen::Vector2i p1, Eigen::Vector2i p2);
  float DistanceUV2Line(Eigen::Vector2i p, GEO_LINE2D line);
  float DistanceUV2LineSeg(Eigen::Vector2i p, GEO_LINE2D line);
  float DistanceUV2Polygon(Eigen::Vector2i p, GEO_POLYGON2D polygon);
  float DistanceUV2PolygonBorder(Eigen::Vector2i p, GEO_POLYGON2D polygon);
  float DistanceUV2CurveLine(Eigen::Vector2i p, GEO_CURVELINE2D curveline);

  bool UVInPolygon(Eigen::Vector2i p, GEO_POLYGON2D polygon);
  bool UVInLine(Eigen::Vector2i p, GEO_LINE2D line);
  bool UVInLineSeg(Eigen::Vector2i p, GEO_LINE2D line);
  bool UVInCurveLine(Eigen::Vector2i p, GEO_CURVELINE2D curveline);

  Eigen::Vector2i ProjectionUV2Line(Eigen::Vector2i p, GEO_LINE2D line);
  Eigen::Vector2i ProjectionUV2LineSeg(Eigen::Vector2i p, GEO_LINE2D line);
  Eigen::Vector2i ProjectionUV2CurveLine(Eigen::Vector2i p,
                                         GEO_CURVELINE2D curveline);

  bool LineLineCross(GEO_LINE2D line1, GEO_LINE2D line2, Eigen::Vector2i *p);
  bool LineLineSegCross(GEO_LINE2D line, GEO_LINE2D lineseg,
                        Eigen::Vector2i *p);
  bool LineSegLineSegCross(GEO_LINE2D lineseg1, GEO_LINE2D lineseg2,
                           Eigen::Vector2i *p);

 private:
  GeometryXY geoxy;
  //     |b|a|b|
  //     |a|x|a|
  //     |b|a|b|
  float step_a, step_b;
};

}  // end namespace map_interface
#endif  // INTERFACE_GEOMETRY_2D_H_
