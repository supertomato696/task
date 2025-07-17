/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:IRCreatLaneAlgorithm.h
简要描述: 本文件内包含参考IR数据绘制车道参考线及生成标线时会使用到的部分算法
******************************************************************/

#ifndef ENGINE_GEOMETRIES_IRCREATLANEALGORITHM_H_
#define ENGINE_GEOMETRIES_IRCREATLANEALGORITHM_H_

#include "Geometries/Geometry.h"
#include "Geometries/Polygon.h"
#include "Base/Array.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

namespace Engine
{
        namespace Geometries
        {
                class Geometries_API IRCreatLaneAlgorithm
                {
                public:
                        // 点在多段线上是否有投影点
                        // 输入：pHitPoint
                        // 输入：pMultLine
                        // 返回：有true、无false
                        static bool IsPointPrjToMultLine(
                            const Engine::Geometries::Coordinate *pHitPoint,
                            const Engine::Geometries::LineString *pMultLine);

                        // 计算线1到线2的XY平均距离
                        // 输入：line1
                        // 输入：line2
                        // 返回：平均距离,计算失败返回-1.0
                        static double CalShortLAvgDstToLongL(
                            const Engine::Geometries::LineString *line1,
                            const Engine::Geometries::LineString *line2,
                            double &dMaxWidth, double &dMinWidth);

                        // 计算线1到线2的XY平均距离
                        // 输入：line1
                        // 输入：line2
                        // 返回：平均距离,计算失败返回-1.0
                        static double CalShortLAvgDstToLongL(
                            const Engine::Geometries::LineString *line1,
                            const Engine::Geometries::LineString *line2);

                        // 按在参照线上投影点位置排序并去重
                        // 输入：lineRefLine			参照线
                        // 输入：pOrgLines			需要排序的线
                        // 返回：排序后的线
                        static void SortAndRemoveRepByPrjMeasure(
                            Engine::Geometries::LineString *lineRefLine,
                            Array<Geometries::LineString *> &pOrgLines);

                        // 按在参照线上投影点位置排序并去重
                        // 输入：lineRefLine			参照线
                        // 输入：pOrgLines			需要排序的线
                        // 返回：排序后的线
                        static void SortAndRemoveRepByPrjMeasure(
                            Engine::Geometries::LineString *lineRefLine,
                            Array<Geometries::LineString *> &pOrgLines,
                            double &dPrjLength, double &dPrjScale);

                        // 按投影点的投影位置排序
                        // 输入：lineRefLine			参照线
                        // 输入：pPrjPts				投影点
                        // 返回：排序后的线
                        static void SortPrjPtByPrjMeasure(
                            Engine::Geometries::LineString *lineRefLine,
                            Array<Geometries::Coordinate *> &pPrjPts);

                        // 截取线位于多边形内的部分（含交点、支持截取多条）
                        // 输入：polygon		多边形
                        // 输入：pLineOrg	需要截取的线
                        // 带出：pLinesInPlg	截取到的线数组(输入必须是空数组，外部维护内存)
                        // 返回：是否截取到
                        static bool InterceptLinesInPolygon(
                            Geometries::Polygon *polygon,
                            Geometries::LineString *pLineOrg,
                            Array<Geometries::LineString *> &pLinesInPlg);

                        // 截取线段位于多边形内/外的部分
                        // 输入：polygon		多边形
                        // 输入：pCoorS	需要截取的线段起点
                        // 输入：pCoorE	需要截取的线段终点
                        // 输入：bInRing	线段终点
                        // 带出：pLinesInPlg	截取到的线数组(输入必须是空数组，外部维护内存)
                        // 返回：是否截取到
                        static void GetLineSegmentInOrOutPolygonParts(Geometries::Polygon *polygon,
                                                                      Geometries::Coordinate *pCoorS, Geometries::Coordinate *pCoorE,
                                                                      Base::Bool bInRing, Array<Geometries::LineString *> &pRstLines);

                        // 计算线段与多段线的交点
                        // 输入：ring		圈
                        // 输入：pCoorS	线段起点
                        // 输入：pCoorE	线段终点
                        // 带出：arrCoor	交点数组(返回结果已排序)
                        // 带出：arrMeasure	交点在线段上的位置(返回结果已排序)
                        // arrCoor 与 arrMeasure 数目必须一致
                        // 返回：空
                        static void GetInterceptLineSegmentWithMultLine(Geometries::LineString *multLine,
                                                                        Geometries::Coordinate *pCoorS, Geometries::Coordinate *pCoorE,
                                                                        Array<Geometries::Coordinate *> &arrCoor, Array<Double> &arrMeasure);

                        // 多段线平移
                        // 多段线沿首末点向量垂直方向水平移动（Z值不变）
                        // 输入：pLineOrg	原始多段线点串
                        // 输入：dLength		移动距离，左正右负
                        // 返回：平移后的线
                        static Geometries::LineString *MultLineTranslationXY(Geometries::LineString *pLineOrg, Base::Double dLength);

                        // 多段线光滑平移
                        // 多段线平滑移动至指定首末点位置
                        // 输入：pCoorDstS	目标首点
                        // 输入：pCoorDstE	目标末点
                        // 输入：pLineOrg	原始多段线点串
                        // 返回：平移后的多段线点串
                        static Geometries::LineString *MultLineSmothTranslationXY(
                            const Geometries::Coordinate *pCoorDstS, const Geometries::Coordinate *pCoorDstE,
                            Geometries::LineString *pLineOrg);

                        // 直线与平面交点
                        // 多段线平滑移动至指定首末点位置（Z值不变）
                        // 输入：pntLine		直线上一点
                        // 输入：coorLineV	直线向量
                        // 输入：pntPlane	平面上一点
                        // 输入：coorPlaneV	平面法向量
                        // 输入：pntResult	交点
                        // 返回：存在交点返回true，不存在返回false
                        static Base::Bool InterPtOfStrtLineAndPlane(
                            const Engine::Geometries::Coordinate pntLine,
                            const Engine::Geometries::Coordinate coorLineV,
                            const Engine::Geometries::Coordinate pntPlane,
                            const Engine::Geometries::Coordinate coorPlaneV,
                            Engine::Geometries::Coordinate &pntResult);

                        // 点到直线距离(XY)
                        // 输入：pt
                        // 输入：ptStart
                        // 输入：ptEnd
                        // 返回：距离
                        static Base::Double DistancePtToStrtLineXY(
                            const Coordinate pt, const Coordinate ptStart,
                            const Coordinate ptEnd);

                        // 两直线交点(XY)
                        // 输入：pntLine1S
                        // 输入：pntLine1E
                        // 输入：pntLine2S
                        // 输入：pntLine2E
                        // 输入：pntResult	交点
                        // 返回：存在交点返回true，不存在返回false
                        static Base::Bool InterPtOfStrtLines(
                            const Engine::Geometries::Coordinate pntLine1S,
                            const Engine::Geometries::Coordinate pntLine1E,
                            const Engine::Geometries::Coordinate pntLine2S,
                            const Engine::Geometries::Coordinate pntLine2E,
                            Engine::Geometries::Coordinate &pntResult);

                        // 直线与线段交点(XY)(交点Z值从线段获取)
                        // StrtLine:直线；LineSeg：线段
                        // 输入：pntStrtLineS
                        // 输入：pntStrtLineE
                        // 输入：pntLineSegS
                        // 输入：pntLineSegE
                        // 输入：pntResult	交点
                        // 返回：存在交点返回true，不存在返回false
                        static Base::Bool InterPtOfStrtLineAndLineSeg(
                            const Engine::Geometries::Coordinate pntStrtLineS,
                            const Engine::Geometries::Coordinate pntStrtLineE,
                            const Engine::Geometries::Coordinate pntLineSegS,
                            const Engine::Geometries::Coordinate pntLineSegE,
                            Engine::Geometries::Coordinate &pntResult);

                        // 射线与线段交点(XY)(交点Z值从线段获取)
                        // RadLine:射线；LineSeg：线段
                        // 输入：pntRadLineS
                        // 输入：pntRadLineN
                        // 输入：pntLineSegS
                        // 输入：pntLineSegE
                        // 输入：pntResult	交点
                        // 返回：存在交点返回true，不存在返回false
                        static Base::Bool InterPtOfRadLineAndLineSeg(
                            const Engine::Geometries::Coordinate pntRadLineS,
                            const Engine::Geometries::Coordinate pntRadLineN,
                            const Engine::Geometries::Coordinate pntLineSegS,
                            const Engine::Geometries::Coordinate pntLineSegE,
                            Engine::Geometries::Coordinate &pntResult);

                        // 两线段交点(XY)(交点Z值为0)，两线段的端点重合不算相交
                        // StrtLine:直线；LineSeg：线段
                        // 输入：pntStrtLineS1
                        // 输入：pntStrtLineE1
                        // 输入：pntStrtLineS2
                        // 输入：pntStrtLineE2
                        // 输入：pntResult	交点
                        // 返回：存在交点返回true，不存在返回false
                        static Base::Bool InterPtOfLineSegs(
                            const Engine::Geometries::Coordinate pntLineSegS1,
                            const Engine::Geometries::Coordinate pntLineSegE1,
                            const Engine::Geometries::Coordinate pntLineSegS2,
                            const Engine::Geometries::Coordinate pntLineSegE2,
                            Engine::Geometries::Coordinate &pntResult);
                        // 判断两线段是否相交，并计算交点(XY)(交点Z值为0)，两线段的端点重合也算相交
                        // StrtLine:直线；LineSeg：线段
                        // 输入：pntStrtLineS1
                        // 输入：pntStrtLineE1
                        // 输入：pntStrtLineS2
                        // 输入：pntStrtLineE2
                        // 输入：pntResult	交点
                        // 返回：存在交点返回true，不存在返回false
                        static Base::Bool IsLineSegsIntersect(
                            const Engine::Geometries::Coordinate pntLineSegS1,
                            const Engine::Geometries::Coordinate pntLineSegE1,
                            const Engine::Geometries::Coordinate pntLineSegS2,
                            const Engine::Geometries::Coordinate pntLineSegE2,
                            Engine::Geometries::Coordinate &pntResult);
                        // 判断两线段(交点Z值为0)是否相交(二维XY)
                        static Base::Bool IsLineSegsCrossed(
                            const Engine::Geometries::Coordinate pntLineSegS1,
                            const Engine::Geometries::Coordinate pntLineSegE1,
                            const Engine::Geometries::Coordinate pntLineSegS2,
                            const Engine::Geometries::Coordinate pntLineSegE2,
                            Base::Double tolerance = Geometries_NEP);
                        // 计算二维直线（或线段）的交点（前提是两线段相交，需要结合函数IsLineSegsCrossed使用）
                        static Base::Void LineSegsCrossPnt(
                            const Engine::Geometries::Coordinate pntLineSegS1,
                            const Engine::Geometries::Coordinate pntLineSegE1,
                            const Engine::Geometries::Coordinate pntLineSegS2,
                            const Engine::Geometries::Coordinate pntLineSegE2,
                            Engine::Geometries::Coordinate &crossPnt);
                        // 计算线段所在直线(二维XY)的一般方程aX+bY+c=0（x:a,y:b,z:c）
                        static Engine::Geometries::Coordinate calcLineSegParameters(
                            const Engine::Geometries::Coordinate pntLineSegS,
                            const Engine::Geometries::Coordinate pntLineSegE);
                        // 直线与多段线交点(XY)(交点Z值从多段线获取)(只返回第一个交点)
                        // StrtLine:直线
                        // 输入：pntStrtLineS
                        // 输入：pntStrtLineE
                        // 输入：pMultLine	多段线
                        // 输入：pntResult	交点
                        // 返回：存在交点返回true，不存在返回false
                        static Base::Bool InterPtOfStrtLineAndMultLine(
                            const Engine::Geometries::Coordinate pntStrtLineS,
                            const Engine::Geometries::Coordinate pntStrtLineE,
                            const Engine::Geometries::LineString *pMultLine,
                            Engine::Geometries::Coordinate &pntResult);

                        // 射线与多段线交点(XY)(交点Z值从多段线获取)(只返回第一个交点)
                        // RadLine:射线
                        // 输入：pntRadLineS
                        // 输入：pntRadLineN
                        // 输入：pMultLine	多段线
                        // 输入：pntResult	交点
                        // 返回：存在交点返回true，不存在返回false
                        static Base::Bool InterPtOfRadLineAndMultLine(
                            const Engine::Geometries::Coordinate pntRadLineS,
                            const Engine::Geometries::Coordinate pntRadLineN,
                            const Engine::Geometries::LineString *pMultLine,
                            Engine::Geometries::Coordinate &pntResult);

                        // 生成多段线外扩缓冲区(Z为0)
                        // 输入：pMultline	原始多段线
                        // 输入：pPgResult	带出生成的缓冲区
                        // 输入：distanceL	左侧扩距(正数)
                        // 输入：distanceR	右侧扩距(负数)
                        // 输入：distanceSE	端点前后延伸(正数)
                        // 返回：成功返回true，失败返回false
                        static Base::Bool GenerateMultlineExternalBuffer(
                            Engine::Geometries::LineString *pMultline,
                            Engine::Geometries::Polygon *&pPgResult,
                            Base::Double distanceL, Base::Double distanceR, Base::Double distanceSE = 0.0);

                        // 线几何更新(根据pLine2在pLine1上的投影情况，更新pLine1)
                        // 输入：pLine1
                        // 输入：pLine2
                        // 返回：更新返回true，未更新返回false
                        static Base::Bool RefreshLineByAnotherThroughPrj(
                            Engine::Geometries::LineString *&pLine1,
                            Engine::Geometries::LineString *pLine2,
                            Base::Double dThreshold = 0.10);

                        // 线几何更新(根据pLine2在pLine1上的投影情况，更新pLine1)
                        // 输入：pLine1
                        // 输入：pLine2
                        // 输入：pLineRst 带出参数（更新结果）
                        // 输入：iRefreshType 更新类型（）
                        // 输入：dThreshold 首尾原始标线保留最小长度
                        // 输入：dSmoothScale 直角拐弯处平滑比例
                        // 输出：0（更新成功）1（方向不一致）2（无投影重叠部分）3(输入参数异常)4(更新结果异常)
                        // 返回：更新返回true，未更新返回false
                        static Base::Int32 RefreshLineByAnotherInOverlap(
                            Engine::Geometries::LineString *pLine1,
                            Engine::Geometries::LineString *pLine2,
                            Engine::Geometries::LineString *&pLineRst,
                            Base::Int32 iRefreshType = 0, Base::Double dThreshold = 0.10, Base::Double dSmoothScale = 10.0);

                        // 线几何更新(根据pLine2在pLine1上的投影情况，更新pLine1)
                        // 输入：pLine1
                        // 输入：pLine2
                        // 返回：更新返回true，未更新返回false
                        static Base::Bool MergeTowLinehroughPrj(
                            Engine::Geometries::LineString *&pLine1,
                            Engine::Geometries::LineString *pLine2);

                        // 根据刻度值获取线上的点
                        // 输入：pLine
                        // 输入：dMeasure
                        // 返回：线上的点
                        static Engine::Geometries::Coordinate *GetPtInLineByMeasure(
                            Engine::Geometries::LineString *pLine,
                            Base::Double dMeasure);

                        // 获取中心线，成功返回中心线，失败返回空指针
                        // 输入：pLineStringL、pLineStringR，两个输入线
                        static Engine::Geometries::LineString *GetCenterLineOfLRLine(
                            Engine::Geometries::LineString *pLineStringL,
                            Engine::Geometries::LineString *pLineStringR);

                        // Summary:  判断点与多边形位置关系（水平投影下）
                        // Parameters:
                        //        polygon : 多边形.
                        //        pCoor : 点.
                        // Return : 在多边形内部返回0，在多边形上返回1，在多边形外返回2.
                        static int PtPositionWithPolygon(
                            const Engine::Geometries::Polygon *polygon,
                            const Engine::Geometries::Coordinate *pCoor);

                        // Summary:  获取线上距离测试点最近的点(带出Z值)
                        // Parameters:
                        //        ptTest :	测试点.
                        //        ls :		线.
                        //        coord :	最近点.
                        // Return ：成功返回true，失败返回false
                        static Base::Bool GetNearestPtOnMultLine(
                            const Engine::Geometries::Coordinate *ptTest,
                            const Engine::Geometries::LineString *ls,
                            Engine::Geometries::Coordinate &coord,
                            int &iIndex);

                        // Summary:  根据平面打断多段线
                        // Parameters:
                        // Return ：成功返回true，失败返回false
                        static Base::Bool BreakMultLineByPlane(
                            const Geometries::LineString *pMultLine,
                            const Engine::Geometries::Coordinate ptInPlane,
                            const Engine::Geometries::Coordinate normalVector,
                            Geometries::LineString *&pLsS,
                            Geometries::LineString *&pLsE);

                        // Summary:  根据到参照线的距离裁剪多段线
                        // Parameters:
                        //        pLineRef :		参照线.
                        //        pMultLine :	多段线.
                        //        arrClipLines :	带出参数（输入空数组），裁剪出的线数组（到参照线的距离大于裁剪距离的部分）.
                        //        dDistence :	裁剪距离.
                        // Return ：成功返回true，失败返回false
                        static void ClipLineByDistence(
                            Engine::Geometries::LineString *pLineRef,
                            const Engine::Geometries::LineString *pMultLine,
                            Array<Geometries::LineString *> &arrClipLines,
                            double dDistence);

                        // Summary:  判断线位于参照线的哪一边
                        // Parameters:
                        //        pLineRef :		参照线.
                        //        pLineTest :	测试线.
                        // Return ：-1 判断失败或交叉；0 重叠；1 左边； 2 右边
                        static int LineOnWhichSideOfAnother(const Engine::Geometries::LineString *pLineRef,
                                                            const Engine::Geometries::LineString *pLineTest, bool bJudgeOnce = true);

                        // Summary:  计算线数组在参照线的投影比例
                        // Parameters:
                        //        pLineRef :		参照线.
                        //        arrClipLines :	线数组
                        // Return ：投影比例
                        static double CalPrjMeasureScale(
                            Engine::Geometries::LineString *pLineRef,
                            Array<Geometries::LineString *> &arrClipLines);

                        // Summary:  根据到参照线的距离裁剪多段线
                        // Parameters:
                        //        pLineRef :		参照线.
                        //        pMultLine :	多段线.
                        //        arrClipLines :	带出参数（输入空数组），裁剪出的线数组（到参照线的距离大于裁剪距离的部分）.
                        //        dDistence :	裁剪距离.
                        // Return ：成功返回true，失败返回false
                        static bool ClipLineByPts(
                            Engine::Geometries::LineString *pLine,
                            Array<Geometries::Coordinate *> arrCoors,
                            Array<Geometries::LineString *> &arrLines);

                        // Summary:  多段线是否自相交
                        // Parameters:
                        //        pMultLine :	多段线.
                        //        iInterIndex1 :	相交线段编号（前）.
                        //        iInterIndex2 :	相交线段编号（后）.
                        // Return ：自相交返回true，否则返回false
                        static bool IsMultLineSelfIntersection(
                            Engine::Geometries::LineString *pLine,
                            Int32 &iInterIndex1, Int32 &iInterIndex2);

                        // Summary:  精简多段线（小于精简长度的线段合并）
                        // Parameters:
                        //        pMultLine :	多段线.
                        //        dSimpleLength :	精简长度.
                        // Return ：自相交返回true，否则返回false
                        static void SimpleMultLine(
                            Engine::Geometries::LineString *&pLine,
                            double dSimpleLength);

                        // Summary:  点演所给向量垂直方向平移（Z值不变）
                        // Parameters:
                        //        pOrgCoor :	初始坐标.
                        //        pOrgVector :	原始向量.
                        //        pDisCoor :	结果坐标.
                        //		 dOffsetDis ：平移距离（左正右负）.
                        // Return ：自相交返回true，否则返回false.
                        static bool GetVerticalOffsetPoint(
                            Engine::Geometries::Coordinate pOrgCoor,
                            Engine::Geometries::Coordinate pOrgVector,
                            Engine::Geometries::Coordinate &pDisCoor,
                            Base::Double dOffsetDis);

                        // Summary:  获取点集的外接矩形（Z值为0）
                        // Parameters:
                        //        pOrgCoor :	初始坐标.
                        //        pOrgVector :	原始向量.
                        //        pDisCoor :	结果坐标.
                        //		 dOffsetDis ：平移距离（左正右负）.
                        // Return ：自相交返回true，否则返回false.
                        static Engine::Geometries::Polygon *GetExternalRectangle(
                            const Array<Coordinate *> *pntsSource);

                        // Summary:  判断线在参照线上的投影方向一致性
                        // Parameters:
                        //        pLineRef :	参照线
                        //        pLineTest : 测试线
                        // Return ：测试线起终点在参照线上存在投影点且位置顺序与参照线点序相同，返回true， 否则返回false.
                        static bool JudgePrjLineDirection(
                            Engine::Geometries::LineString *pLineRef,
                            Engine::Geometries::LineString *pLineTest);

                        // Summary:  计算多段线的平均方向向量
                        // Parameters:
                        //        pLine :	多段线
                        //        vector :	平均向量
                        // Return ：计算成功返回true， 否则返回false.
                        static bool GetAverageVector2DOfLineString(
                            Engine::Geometries::LineString *pLine,
                            Engine::Geometries::Coordinate &vector);

                        // Summary:  计算多段线的平均Z
                        // Parameters:
                        //        pLine :	多段线
                        //        vector :	平均Z
                        // Return ：计算成功返回true， 否则返回false.
                        static bool GetAverageZOfLineString(
                            Engine::Geometries::LineString *pLine,
                            Engine::Base::Double &valueZ);

                        // Summary:  获取两条多段线的平均向量夹角
                        // Parameters:
                        //        pLine1 :	多段线1
                        //        pLine2 :	多段线2
                        //        dAngle :	夹角（带出）0~180
                        // Return ：计算成功返回true， 否则返回false.
                        static bool GetTowLineAverageVectorAngle2D(
                            Engine::Geometries::LineString *pLine1,
                            Engine::Geometries::LineString *pLine2,
                            Engine::Base::Double &dAngle);

                        // Summary: 处理线上两端拐角（夹角小于90°）
                        // Parameters:
                        //        pLineD :待处理线
                        // Return ：void
                        static void DealHECornerInLine(Engine::Geometries::LineString *pLineD);
                };
        }
}

#endif // ENGINE_GEOMETRIES_IRCREATLANEALGORITHM_H_