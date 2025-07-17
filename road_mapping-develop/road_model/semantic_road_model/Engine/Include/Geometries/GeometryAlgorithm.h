/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:GeometryAlgorithm.h
简要描述:提供对Geometry的各种运算和操作方法
******************************************************************/

#ifndef ENGINE_GEOMETRIES_GEOMETRYALGORITHM_H_
#define ENGINE_GEOMETRIES_GEOMETRYALGORITHM_H_

#include "Export.h"
#include "Base/Types.h"
#include "Base/Array.h"
#include "Coordinate.h"

namespace Engine
{
        namespace Base
        {
                class String;
        }

        namespace Geometries
        {
                class Geometry;
                class Coordinate;
                class LineString;
                class Point;
                class Polygon;
                class LinearRing;
                class MultiLineString;
                // 平面点法式表达 [9/4/2018 wufuzheng]
                struct PointNormalPlane
                {
                        Geometries::Coordinate coord;        // 平面上一点
                        Geometries::Coordinate normalVector; // 平面法向量
                };
                // 提供对Geometry的各种运算和操作方法
                class Geometries_API GeometryAlgorithm
                {
                public:
                        // 从Geometry转换为WKT文本
                        static Base::String *GeometryToWKT(const Geometry *geom);

                        // 从WKT文本转换为Geometry
                        static Geometry *WKTToGeometry(const Base::String *wkt);

                        // 从Geometry转换为WKB文本
                        static Base::String *GeometryToWKB(const Geometry *geom);

                        // 从WKB文本转换为Geometry
                        static Geometry *WKBToGeometry(const Base::String *wkb);

                        // coord中存储计算出来的geom重心,返回false表示无法计算重心,只能计算二维重心
                        // 如果coord为nullptr,返回false
                        static Base::Bool GetCentroid(const Geometry *geom, Coordinate *coord);

                        // 计算点到线的最近距离和最近的点,coord中存储计算出来的距离最近点,计算时只考虑二维坐标
                        // 注意:如果只需要距离,不需要最近点,coord请传递NULL
                        static Base::Double GetMinDistance(const Point *pt, const LineString *ls, Coordinate *coord);

                        // 三维点到线段的距离
                        static Base::Double DistancePtToLinesegment(
                            const Engine::Geometries::Point *pt,
                            const Engine::Geometries::Point *ptStart,
                            const Engine::Geometries::Point *ptEnd);

                        // 计算点位于LineString的哪个形状边上
                        // 当isOnLine为false时,表示点不在LineString上,且isOnVertex和返回值无意义
                        // 当isVertex为true时,返回值表示coord与第index个点重合;
                        // 当isVertex为false时,返回值表示coord在第index个点之后的形状边上;
                        // 计算时只考虑二维坐标
                        static Base::SizeT IndexAtLineString(const Coordinate *coord, const LineString *ls, Base::Bool *isOnVertex, Base::Bool *isOnLine);

                        // 如果geom1与geom2相交,返回true,否则返回false.计算时只考虑二维坐标
                        static Base::Bool Intersects(const Geometry *geom1, const Geometry *geom2);

                        // LineString与polygon是否相交
                        static Base::Bool IntersectsLineSegmentPolygon(const Engine::Geometries::Geometry *geom1, const Engine::Geometries::Geometry *geom2);

                        // 计算三个点构成的三维夹角,单位弧度 值域[0 , π],
                        // 注意:如果只需要计算二维夹角,请把所有点的z设置为0
                        static Base::Double ComputeAngle(Coordinate coord1, Coordinate coord2, Coordinate coord3);

                        // 将度转换为弧度
                        static Base::Double DegreeToRadian(Base::Double a);

                        // 将弧度转换度
                        static Base::Double RadianToDegree(Base::Double a);

                        // 打断线，找到线上一点，该点距离线外一点距离最近，用线上的该点打断线。
                        // pntHitTest，线外一点；ls，一条线；
                        // lsS，打断后的前一条线；lsS，打断后的后一条线
                        static Base::Bool PtBreakLine(
                            const Engine::Geometries::Coordinate *pntHitTest,
                            Engine::Geometries::LineString *ls,
                            Engine::Geometries::LineString *&lsS,
                            Engine::Geometries::LineString *&lsE);

                        // 根据传入的2点，截取折线上的一段子折线段
                        // 输入：pntS，首点；pntE，末点；
                        // ls，原始线
                        // 返回：截取的折线
                        static Engine::Geometries::LineString *InterceptPartLine(
                            const Engine::Geometries::Coordinate *pntS,
                            const Engine::Geometries::Coordinate *pntE,
                            Engine::Geometries::LineString *ls);

                        // 三维点打断折线段，找到折线段上一点，该点距离三维点的距离最近，用三维点上的该点打断折线段
                        // 输入：pntHitTest，三维点；coordinates，折线段
                        // 返回：lsS，打断后的前一条折线段；lsS，打断后的后一条折线段
                        static Base::Bool PtBreakLineSegments(
                            const Engine::Geometries::Coordinate *pntHitTest,
                            const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
                            Engine::Geometries::LineString *&lsS,
                            Engine::Geometries::LineString *&lsE);

                        // 生成偏移线  2D only
                        //@param vecDistance ,偏移距离 , 正距离代表线左侧，负距离代表线右侧
                        //@param vecOffsetLines ,偏移线结果
                        // 注意:需要由调用者释放所返回的指针
                        static Base::Void GenerateOffsetLines(const LineString *pLineString, const Base::Array<Base::Double> &vecDistance,
                                                              Base::Array<LineString *> &vecOffsetLines);

                        // 生成偏移线 2D only
                        //@param distance > 0 生成左侧偏移线 < 0, 生成线在右侧
                        static LineString *GenerateOffsetLine(const LineString *pLineString, Base::Double distance);

                        // 生成偏移面 2donly, distance必须大于0，表示往外扩
                        static Polygon *GenerateOffsetPolygon(const Polygon *pPolygon, Base::Double distance);

                        // 提取中心线，成功返回中心线，失败返回空指针
                        // 输入：pLineStringL、pLineStringR，两个输入线
                        static Engine::Geometries::LineString *GenerateCenterLine(
                            const Engine::Geometries::LineString *pLineStringL,
                            const Engine::Geometries::LineString *pLineStringR);

                        // 点是否多边形上。
                        // 多边形线上返回true，多边形内返回false。
                        // pt返回撞边到多边形的精确值
                        // 输入：polygon，多边形；pt，测试点,write by xueyufei
                        static Base::Bool PtOnLineRing(
                            const Geometries::LinearRing *lring,
                            Engine::Geometries::Coordinate *pt,
                            Base::Double tolerance = 2.0);

                        // 点在多边形内的判断。多边形外或多边形线上返回false，多边形内返回true。
                        // 输入：polygon，多边形；pt，测试点
                        static Base::Bool PtInPolygon(
                            const Engine::Geometries::Polygon *polygon,
                            const Engine::Geometries::Coordinate *pt);

                        // 点在图幅内外的判断。图幅外或图幅线上返回true，图幅内返回false。
                        // 输入：polygon，图幅；pt，测试点；pntProject
                        // 返回：点在图幅线上的投影点
                        static Base::Bool OutOnMapProjection(
                            const Engine::Geometries::Polygon *polygon,
                            Engine::Geometries::Coordinate *pt,
                            Engine::Geometries::Coordinate &pntProject);

                        // 两个道路线的宽度
                        // 输入：pLineStringL和pLineStringR，两个道路线
                        // 返回：sWidth，首点的距离；eWidth，末点的距离；
                        // averageWidth，平均距离
                        static Base::Bool RoadWidth(
                            const Engine::Geometries::LineString *pLineStringL,
                            const Engine::Geometries::LineString *pLineStringR,
                            Base::Double &sWidth,
                            Base::Double &eWidth,
                            Base::Double &averageWidth);

                        /*!
                         *\brief 折线上最小的偏转角低于阈值
                         *\ param const LineString * pLineString
                         *\ param const Double dAngle 角度值，值域[0-180]
                         *\ Returns:   Base::Bool
                         */
                        static Base::Bool MinTurnAngleLessThan(const LineString *pLineString, const Base::Double dAngle);

                        // 在线集合中，计算其中的某个线在线集合的索引号
                        // 输入：pLine，线集合中的某个线；arrMarkLines，线集合
                        // 返回：该线在线集合的索引号
                        static Base::Int32 GetIndexInLines(Base::Int32 nIndex, const Base::Array<Engine::Geometries::LineString *> &arrLines);
                        // 通过投影最近点判断
                        static Base::Int32 GetIndexInLinesByPrj(Base::Int32 nIndex, const Base::Array<Engine::Geometries::LineString *> &arrLines);
                        static Base::Bool GetIndexInLines(const Base::Array<Engine::Geometries::LineString *> &arrLines, Base::Array<Base::Int32> &arrIndexs);
                        // 判断几何体是否需要接边，两个几何体的节点距离小于dTolerance时需要接边，接边返回true，并修改两几何体的节点的坐标值，不接边返回false
                        // 输入：geometry1、geometry2，两几何体
                        // 返回：geometry1、geometry2，如果需要接边，修改节点坐标值后的两几何体
                        static Base::Bool EdgeMatch(
                            Engine::Geometries::Geometry *&geometry1,
                            Engine::Geometries::Geometry *&geometry2,
                            Base::Double dTolerance, Base::Double &dMinDis, Base::Bool bImposeMatch = true);

                        // 0 成功 其他失败 1 对象几何数据为空 2 点顺序错误
                        static Base::Int32 EdgeUnion(Engine::Geometries::Geometry *geometry1, Engine::Geometries::Geometry *geometry2);

                        static Base::Bool PointsMatch(Base::Array<Engine::Geometries::Coordinate *> *pCoordinates, const Base::Array<Engine::Geometries::LineString *> &arrMarkLines);

                        /*!
                         *\brief 在 x-y 平面上 用 @polygon 裁剪  @lineString,
                         *\ param Geometries::LineString & lineString 待裁剪linestring
                         *\ param Geometries::Polygon & polygon 参考polygon
                         *\ param Base::Array<LineString * > & vecClipResult 结果 LineString* 归传入者处理
                         *\ Returns:   Base::Void
                         */
                        static Base::Void ClipLineStringByPolygon(Geometries::LineString &lineString, Geometries::Polygon &polygon, Base::Array<LineString *> &vecClipResult);

                        // 两个折线段比较方位
                        // 如果lhs在rhs左边返回1,否则返回2
                        static Base::Int32 LineOrientatedWithLine(Geometries::LineString *lhs, Geometries::LineString *rhs);

                        // 判断左右关系（适用于中央隔离带的创建）
                        // 判断rhs上点在lhs的哪侧（点向线做投影，判断点在投影点的哪侧），如果rhs上两头点分别在lhs的两侧，返回-1。如果rhs在lhs的左边返回2，右侧返回1。
                        static Base::Int32 LineOrientatedWithLineByProj(Geometries::LineString *lhs, Geometries::LineString *rhs);

                        /*******************************************************************
                         * @brief: 向LineString中距离起点或终点dDis处插入一个形状点(线性插值，不改变线的形状)
                         *		如果与端点最近的形状点与端点距离小于dDis，则不进行插值直接返回true
                         * @author: wufuzheng
                         * @date:2018/04/13
                         * @param[in]: pLineStr：待插点点串，插点后会直接放入该指针中
                         * @param[in]: dDis：距离>0，sNode：true起始点，false终止点
                         * @return: true：插值成功：false：插值失败
                         ******************************************************************/
                        static Base::Bool InsertLineStringPoint(Geometries::LineString *pLineStr, Base::Double dDis, Base::Bool sNode);

                        /*******************************************************************
                         * @brief: 按照法向量方向，将Polygon平移距离offsetDis(3维)
                         * @author: wufuzheng
                         * @date:2018/09/04
                         * @param[in]: pPolygon：待平移多边形，normalVector：多边形法向量，offsetDis：平移距离
                         * @return: 返回平移后的多边形，需外部释放内存
                         *******************************************************************/
                        static Geometries::Polygon *GenerateOffsetPolygon(Geometries::Polygon *pPolygon,
                                                                          Geometries::Coordinate normalVector, Base::Double offsetDis);

                        /*******************************************************************
                         * @brief: 计算多边形的几何中心点
                         * @author: wufuzheng
                         * @date:2018/09/04
                         * @param[in]: pPolygon1：待计算多边形
                         * @param[Out]:coordCenter:多边形的几何中心点
                         * @return: true:计算成功，false:计算失败
                         *******************************************************************/
                        static Base::Bool CalcCenterOfPolygon(
                            Geometries::Polygon *pPolygon, Geometries::Coordinate &coordCenter);

                        /*******************************************************************
                         * @brief: 计算两个平面组成的空间多面体（多面体使用每个面的点法式表达）
                         * @author: wufuzheng
                         * @date:2018/09/04
                         * @param[in]: pPolygon1：多边形1，pPolygon2：多边形2
                         * @param[Out]:arrPlane:多面体的点法式表达
                         * @return: true:计算成功，false:计算失败
                         *******************************************************************/
                        static Base::Bool CalcPolyhedronWithTwoPolygon(
                            Geometries::Polygon *pPolygon1, Geometries::Polygon *pPolygon2,
                            Base::Array<PointNormalPlane> &arrPlane);

                        /*******************************************************************
                         * @brief: 计算圆的外接矩形
                         * @author: wufuzheng
                         * @date:2018/09/04
                         * @param[in]: centerCoord：圆心，radius：圆半径
                         * @param[Out]:arrCoords:外接矩形的的四个角点，有序（需要外部释放内存）
                         * @return: true:计算成功，false:计算失败
                         *******************************************************************/
                        static Base::Bool CalcOuterRectOfCircle(Geometries::Coordinate centerCoord, Base::Double radius,
                                                                Base::Array<Geometries::Coordinate *> &arrCoords);

                        /*******************************************************************
                         * @brief: 判断点是否在空间多面体内
                         * @author: wufuzheng
                         * @date:2018/09/04
                         * @param[in]: checkCoord：待判断点，arrPlane：多面体的点法式表发
                         * @param[Out]:arrPlane:多面体的点法式表达
                         * @return: true:在多面体内，false:在多面体外
                         *******************************************************************/
                        static Base::Bool IsPointInPolyhedron(Geometries::Coordinate checkCoord,
                                                              Base::Array<PointNormalPlane> arrPlane);
                };
        }
}

#endif // ENGINE_GEOMETRIES_GEOMETRYALGORITHM_H_
