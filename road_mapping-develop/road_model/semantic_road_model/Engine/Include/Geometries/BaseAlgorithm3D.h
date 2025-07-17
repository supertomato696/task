/******************************************************************
作者: test
日期: 2021-8-18 11:19
******************************************************************/

#ifndef ENGINE_GEOMETRIES_BASEALGORITHM3D_H_
#define ENGINE_GEOMETRIES_BASEALGORITHM3D_H_

#include "Export.h"
#include "Base/Array.h"
#include "Base/Macros.h"
#include "Geometries/LineString.h"
#include "Polygon.h"
#include "Base/Map.h"
namespace Engine
{
        namespace Geometries
        {
                class Coordinate;
                class Envelope3D;
                class Polygon;

                class Geometries_API BaseAlgorithm3D
                {
                public:
                        // ����xyƽ������ĳ����תĳ�Ƕ�
                        // ���룺pntOrg�����Ƶ���ά�㣻dAngle����ת�ĽǶȣ���ֵ����ʱ��ת����ֵ��˳ʱ��ת��pntResult������ת����ά��
                        // ���أ�pntResult����ά����תĳ�ǶȺ�Ľ����ά��
                        static Base::Void RotatePointXY(const Engine::Geometries::Coordinate &pntOrg, Base::Double dAngle,
                                                        Engine::Geometries::Coordinate &pntResult);

                        // ����xzƽ������ĳ����תĳ�Ƕ�
                        // ���룺pntOrg�����Ƶ���ά�㣻dAngle����ת�ĽǶȣ���ֵ����ʱ��ת����ֵ��˳ʱ��ת��pntResult������ת����ά��
                        // ���أ�pntResult����ά����תĳ�ǶȺ�Ľ����ά��
                        static Base::Void RotatePointXZ(const Engine::Geometries::Coordinate &pntOrg, Base::Double dAngle,
                                                        Engine::Geometries::Coordinate &pntResult);

                        // ����yzƽ������ĳ����תĳ�Ƕ�
                        // ���룺pntOrg�����Ƶ���ά�㣻dAngle����ת�ĽǶȣ���ֵ����ʱ��ת����ֵ��˳ʱ��ת��pntResult������ת����ά��
                        // ���أ�pntResult����ά����תĳ�ǶȺ�Ľ����ά��
                        static Base::Void RotatePointYZ(const Engine::Geometries::Coordinate &pntOrg, Base::Double dAngle,
                                                        Engine::Geometries::Coordinate &pntResult);

                        // �㼯��xyƽ������ĳ����תĳ�Ƕ�
                        // ���룺pntOrg�����Ƶ���ά�㣻dAngle����ת�ĽǶȣ���ֵ����ʱ��ת����ֵ��˳ʱ��ת��pntsResult������ת����ά�㼯
                        // ���أ�pntsResult����ά�㼯��תĳ�ǶȺ�Ľ����ά�㼯
                        static Base::Void RotatePointXY(const Engine::Geometries::Coordinate &pntOrg, Base::Double dAngle,
                                                        Engine::Geometries::Coordinate *pntsResult, Base::Int32 nPointCount);

                        // �㼯��xzƽ������ĳ����תĳ�Ƕ�
                        // ���룺pntOrg�����Ƶ���ά�㣻dAngle����ת�ĽǶȣ���ֵ����ʱ��ת����ֵ��˳ʱ��ת��pntsResult������ת����ά�㼯
                        // ���أ�pntsResult����ά�㼯��תĳ�ǶȺ�Ľ����ά�㼯
                        static Base::Void RotatePointXZ(const Engine::Geometries::Coordinate &pntOrg, Base::Double dAngle,
                                                        Engine::Geometries::Coordinate *pntsResult, Base::Int32 nPointCount);

                        // �㼯��yzƽ������ĳ����תĳ�Ƕ�
                        // ���룺pntOrg�����Ƶ���ά�㣻dAngle����ת�ĽǶȣ���ֵ����ʱ��ת����ֵ��˳ʱ��ת��pntsResult������ת����ά�㼯
                        // ���أ�pntsResult����ά�㼯��תĳ�ǶȺ�Ľ����ά�㼯
                        static Base::Void RotatePointYZ(const Engine::Geometries::Coordinate &pntOrg, Base::Double dAngle,
                                                        Engine::Geometries::Coordinate *pntsResult, Base::Int32 nPointCount);

                        // ��ά����ĳ��ά����תĳ�Ƕ�
                        // ���룺pntOrg�����Ƶ���ά�㣻dAngle����ת�ĽǶȣ���ֵ����ʱ��ת����ֵ��˳ʱ��ת��pntResult������ת����ά��
                        // ���أ�pntResult����ά����ĳ��ά����תĳ�ǶȺ�Ľ����ά��
                        static Base::Void RotatePoint(const Engine::Geometries::Coordinate &pntOrg,
                                                      Base::Double dAngle,
                                                      Engine::Geometries::Coordinate &pntNormalVector,
                                                      Engine::Geometries::Coordinate &pntResult);

                        // �������������ļн�,���ػ���
                        static Base::Double CalAzimuth3D(const Coordinate *pPointFrom, const Coordinate *pPointTo);

                        // static Base::Void RotatePoint2(const  Engine::Geometries::Coordinate &pntOrg,
                        //	const  Engine::Geometries::Coordinate &pntCenter,
                        //	Base::Double dAngle,
                        //	 Engine::Geometries::Coordinate &pntNormalVector,
                        //	 Engine::Geometries::Coordinate &pntResult);

                        // �жϵ���ֱ�ߵ���໹���Ҳ࣬��෵��
                        // ���룺pntLineFrom��ֱ�ߵ���㣻pntLineTo��ֱ�ߵ��յ㣻pnt����ά�㣻vec_Normal��ֱ������ƽ��ķ�����
                        static Base::UInt16 PntMatchLine(
                            const Engine::Geometries::Coordinate &pntLineFrom,
                            const Engine::Geometries::Coordinate &pntLineTo,
                            const Engine::Geometries::Coordinate &pnt,
                            const Engine::Geometries::Coordinate &vec_Normal);

                        // ��ά�㵽��άֱ�ߵľ���
                        // ���룺pntS��ֱ�ߵ���㣻pntE��ֱ�ߵ��յ�
                        // ���أ���ά�㵽��άֱ�ߵľ���
                        static Base::Double DisPtToLine(
                            const Engine::Geometries::Coordinate &pntS,
                            const Engine::Geometries::Coordinate &pntE,
                            const Engine::Geometries::Coordinate &pnt,
                            Base::Double tolerance = Geometries_EP);

                        // ��ά�㵽��ά�߶εľ���
                        // ���룺pntS���߶ε���㣻pntE���߶ε��յ�
                        // ���أ���ά�㵽��ά�߶εľ���
                        static Base::Double DisPtToLineSegment(
                            const Engine::Geometries::Coordinate &pntSegFrom,
                            const Engine::Geometries::Coordinate &pntSegTo,
                            const Engine::Geometries::Coordinate &pnt,
                            Base::Double tolerance = Geometries_EP);

                        // �ҵ������߾��������޷�Χ�ڲ��Ҿ��������������ĵ�
                        // ���룺pntRayS�����ߵ���㣻pntRayE�����ߵ��յ㣻
                        // pPnts����ά�㼯�ϣ�nPointCount����ά�����Ŀ��
                        // tolerance������
                        // ���أ�pntNearest�������߾��������޷�Χ�ڲ��Ҿ��������������ĵ�
                        static Base::Bool GetNearestPntToRayOrigin(
                            const Engine::Geometries::Coordinate &pntRayS,
                            const Engine::Geometries::Coordinate &pntRayE,
                            const Engine::Geometries::Coordinate *pPnts,
                            Base::UInt32 nPointCount,
                            Engine::Geometries::Coordinate &pntNearestToS,
                            Base::Double tolerance = Geometries_EP);

                        // �жϵ���ƽ����Ϸ������·����Ϸ�����1���·�����2�����򷵻�0
                        // ���룺pntTest����ά�㣻pnt��ƽ����һ�㣻vec_Normal��ƽ��ķ�����
                        static Base::UInt16 PntMatchPlane(
                            const Engine::Geometries::Coordinate &pntTest,
                            const Engine::Geometries::Coordinate &pnt,
                            Engine::Geometries::Coordinate vec_Normal);

                        // ��õ������������ϵ�ͶӰ��
                        // ���룺pnt0Triangle��pnt1Triangle��pnt2Triangle�������������������
                        // ���أ�pntProject��ͶӰ��
                        static Base::Bool GetProjectpntToTriangle(
                            const Engine::Geometries::Coordinate &pnt0Triangle,
                            const Engine::Geometries::Coordinate &pnt1Triangle,
                            const Engine::Geometries::Coordinate &pnt2Triangle,
                            const Engine::Geometries::Coordinate &pnt,
                            Engine::Geometries::Coordinate &pntProject);

                        // ����㵽��������Ĵ�ֱ����
                        // ���룺pnt0Triangle��pnt1Triangle��pnt2Triangle�������������������
                        static Base::Double DisPtToTriangle(
                            const Engine::Geometries::Coordinate &pnt0Triangle,
                            const Engine::Geometries::Coordinate &pnt1Triangle,
                            const Engine::Geometries::Coordinate &pnt2Triangle,
                            const Engine::Geometries::Coordinate &pnt);

                        // ������ά�㵽ƽ���ͶӰ��
                        // ���룺pnt����ά�㣻pntOrg��ƽ����һ�㣻normalVector��ƽ��ķ�����
                        // ���أ�pntProject����ά�㵽ƽ���ͶӰ��
                        static Base::Bool GetProjectpntToPlane(const Engine::Geometries::Coordinate &pnt,
                                                               const Engine::Geometries::Coordinate &pntOrg,
                                                               Engine::Geometries::Coordinate &normalVector,
                                                               Engine::Geometries::Coordinate &pntProject);

                        // ƽ�滯������ƽ�滯������true������ƽ�滯������false
                        // ���룺pPoints��������ĵ㼯��nPointcount��������ĵ�������� tolerance,�㵽ƽ���������
                        // ���أ�pPoints�������ĵ㼯
                        static Base::Bool Complanation(Engine::Geometries::Coordinate *pPoints,
                                                       Base::Int32 nPointcount, Base::Double tolerance = Geometries_EP);

                        // ƽ�滯������ƽ�滯������true������ƽ�滯������false
                        // ���룺pPoints��������ĵ����飻 tolerance,�㵽ƽ���������
                        // ���أ�pPoints�������ĵ㼯
                        static Base::Bool Complanation(Base::Array<Engine::Geometries::Coordinate *> *pPoints,
                                                       Base::Double tolerance = Geometries_EP);

                        static Base::Bool ComplanationFit(Engine::Geometries::Coordinate *pPoints,
                                                          Base::Int32 nPointcount, Base::Double tolerance = Geometries_EP);

                        // ǿ��ƽ�滯����
                        // ���룺pPoints��������ĵ㼯��nPointcount��������ĵ������
                        // ���أ�pPoints�������ĵ㼯
                        static Base::Bool ImposeComplanation(Engine::Geometries::Coordinate *pPoints,
                                                             Base::Int32 nPointcount);

                        // ǿ��ƽ�滯����
                        // ���룺pPoints��������ĵ����飻nPointcount��������ĵ������
                        // ���أ�pPoints�������ĵ㼯
                        static Base::Bool ImposeComplanation(Base::Array<Engine::Geometries::Coordinate *> *pPoints);

                        // �����������
                        // ���룺pPoints���㼯��nPointCount���㼯�ĵ������
                        // ���أ�envelope���������
                        static Base::Bool GetEnvelope(const Engine::Geometries::Coordinate *pPoints, Base::Int32 nPointCount, Envelope3D &envelope);

                        /*//��֪�߶Ρ���ά�㣬�߶�����ƽ����z��ƽ�С�������ά�����߶�����ƽ���ͶӰ�㣬�߶���㡢�յ��ͶӰ������µľ������
                        //���룺pntSegFrom���߶���㣻pntSegTo���߶��յ㣻pnt����ά��
                        //���أ�arrRectangle���µľ��ε��ĸ��ǵ���ɵ�����
                        static Base::Bool GenerateRectangle(const  Engine::Geometries::Coordinate &pntSegFrom,
                            const  Engine::Geometries::Coordinate &pntSegTo,
                            const  Engine::Geometries::Coordinate &pnt,
                            Base::Array< Engine::Geometries::Coordinate> &arrRectangle);*/

                        /*//��֪��ά��A��B��C��B��C��A���ڵ�ƽ�棨��ƽ����z�ᴹֱ����ͶӰ����B'��C'��A��B'��C'����µ�Բ�������Բ��Բ�ģ�����Բ�ġ�B'��C'
                        //���룺pnt1����һ����ά�㣻pnt2��pnt3����������ά��
                        //���أ�pntCircleCenter���µ�Բ��Բ��
                        static Base::Bool GenerateCircle(const  Engine::Geometries::Coordinate &pnt1,
                             Engine::Geometries::Coordinate &pnt2,
                             Engine::Geometries::Coordinate &pnt3,
                             Engine::Geometries::Coordinate &pntCircleCenter);*/

                        // �ж���ֱ��ƽ�л��ߣ�ƽ�л��߷���true����ƽ�л��߷���false
                        // ���룺pntLine1From��pntLine2From��ֱ�ߵ���㣻pntLine1To��pntLine2To��ֱ�ߵ��յ�
                        static Base::Bool IsParallelCollinearLines(
                            const Engine::Geometries::Coordinate &pntLine1From,
                            const Engine::Geometries::Coordinate &pntLine1To,
                            const Engine::Geometries::Coordinate &pntLine2From,
                            const Engine::Geometries::Coordinate &pntLine2To,
                            Base::Double tolerance = Geometries_EP);

                        // �ж����ߺ��߶��Ƿ��ཻ���ཻ����true�����ཻ����false
                        // ���룺pntRayS�����ߵ���㣻pntRayE�����ߵ��յ㣻
                        // pntSegFrom���߶ε���㣻pntSegTo���߶ε��յ�
                        static Base::Bool IsIntersectRayLineSegment(
                            const Engine::Geometries::Coordinate &pntRayS,
                            const Engine::Geometries::Coordinate &pntRayE,
                            const Engine::Geometries::Coordinate &pntSegFrom,
                            const Engine::Geometries::Coordinate &pntSegTo,
                            Base::Double tolerance = Geometries_EP);
                        // ��չ�����죩�ߴ����ȣ������յ��������߶η���������չ
                        // ���룺lineString���ߴ���length�����쳤�ȣ�
                        // ���أ�true�ӳ��ɹ�,fasle:�ӳ�ʧ��
                        static Base::Bool ExtendedLineString(
                            Engine::Geometries::LineString &lineString,
                            Base::Double length,
                            Base::Double tolerance = Geometries_EP);
                        // �ж����ߺ������Ƿ��ཻ���ཻ����true�����ཻ����false
                        // ���룺pntRayS�����ߵ���㣻pntRayE�����ߵ��յ㣻
                        // pPntSegments�����ߣ�nPointCount�����ߵĽڵ����Ŀ
                        static Base::Bool IsIntersectRayLineSegments(
                            const Engine::Geometries::Coordinate &pntRayS,
                            const Engine::Geometries::Coordinate &pntRayE,
                            const Engine::Geometries::Coordinate *pPntSegments,
                            Base::UInt32 nPointcount,
                            Base::Double tolerance = Geometries_EP);

                        // �������ߺ��߶εĽ��㣬��ʱ�ȷ���һ������
                        // ���룺pntRayS�����ߵ���㣻pntRayE�����ߵ��յ㣻
                        // pntSegFrom���߶ε���㣻pntSegTo���߶ε��յ�
                        // ���أ�pntResult������
                        static Base::Bool IntersectionRayLineSegment(
                            const Engine::Geometries::Coordinate &pntRayS,
                            const Engine::Geometries::Coordinate &pntRayE,
                            const Engine::Geometries::Coordinate &pntSegFrom,
                            const Engine::Geometries::Coordinate &pntSegTo,
                            Engine::Geometries::Coordinate &pntResult,
                            Base::Double tolerance = Geometries_EP);

                        // �������ߺ����ߵĽ��㣬��ʱֻ���ص�һ������
                        // ���룺pntRayS�����ߵ���㣻pntRayE�����ߵ��յ㣻
                        // pPntSegments�����ߣ�nPointCount�����ߵĽڵ����Ŀ
                        // ���أ�pntResult����һ������
                        static Base::Bool IntersectionRayLineSegments(
                            const Engine::Geometries::Coordinate &pntRayS,
                            const Engine::Geometries::Coordinate &pntRayE,
                            const Engine::Geometries::Coordinate *pPntSegments,
                            Base::UInt32 nPointcount,
                            Engine::Geometries::Coordinate &pntResult,
                            Base::Double tolerance = Geometries_EP);

                        // �ж����ߴ�Խ�������棬��Խ����true������Խ����false
                        // ���룺pntRayS�����ߵ���㣻pntRayE�����ߵ��յ㣻
                        // pnt0Triangle��pnt1Triangle��pnt2Triangle�������������������
                        static Base::Bool IsIntersectRayTriangle(
                            const Engine::Geometries::Coordinate &pntRayS,
                            const Engine::Geometries::Coordinate &pntRayE,
                            const Engine::Geometries::Coordinate &pnt0Triangle,
                            const Engine::Geometries::Coordinate &pnt1Triangle,
                            const Engine::Geometries::Coordinate &pnt2Triangle);

                        // �ж����ߺ��������洹ֱ����ֱ����true������ֱ����false
                        // ���룺pntRayS�����ߵ���㣻pntRayE�����ߵ��յ㣻
                        // pnt0Triangle��pnt1Triangle��pnt2Triangle�������������������
                        static Base::Bool IsVerticalRayTriangle(
                            const Engine::Geometries::Coordinate &pntRayS,
                            const Engine::Geometries::Coordinate &pntRayE,
                            const Engine::Geometries::Coordinate &pnt0Triangle,
                            const Engine::Geometries::Coordinate &pnt1Triangle,
                            const Engine::Geometries::Coordinate &pnt2Triangle,
                            Base::Double tolerance);

                        // ������������������Ľ���
                        // ���룺pntRayS�����ߵ���㣻pntRayE�����ߵ��յ㣻
                        // pnt0Triangle��pnt1Triangle��pnt2Triangle�������������������
                        // ���أ�pntResult������
                        static Base::Bool IntersectionRayTriangle(
                            const Engine::Geometries::Coordinate &pntRayS,
                            const Engine::Geometries::Coordinate &pntRayE,
                            const Engine::Geometries::Coordinate &pnt0Triangle,
                            const Engine::Geometries::Coordinate &pnt1Triangle,
                            const Engine::Geometries::Coordinate &pnt2Triangle,
                            Engine::Geometries::Coordinate &pntResult);

                        // �ж����ߴ�Խ������棬��Խ����true������Խ����false
                        // ���룺pntRayS�����ߵ���㣻pntRayE�����ߵ��յ㣻pntsPolygon������εĽڵ㼯��nPointcount������εĽڵ���
                        static Base::Bool IsIntersectRayPolygon(
                            const Engine::Geometries::Coordinate &pntRayS,
                            const Engine::Geometries::Coordinate &pntRayE,
                            const Engine::Geometries::Coordinate *pntsPolygon,
                            Base::UInt32 nPointcount);

                        // �ж��߶κ�ƽ���Ƿ��ཻ���ཻ����true�����ཻ����false
                        // ���룺pntSegFrom���߶ε���㣻pntSegTo���߶ε��յ㣻pntOrg��ƽ����һ�㣻normalVector��ƽ��ķ�����
                        static Base::Bool IsIntersectLineSegmentPlane(const Engine::Geometries::Coordinate &pntSegFrom,
                                                                      const Engine::Geometries::Coordinate &pntSegTo,
                                                                      const Engine::Geometries::Coordinate &pntOrg,
                                                                      const Engine::Geometries::Coordinate &normalVector);

                        // �����߶���ƽ��Ľ��㣬�ཻ����true�����ཻ����false
                        // ���룺pntSegFrom���߶ε���㣻pntSegTo���߶ε��յ㣻pntOrg��ƽ����һ�㣻normalVector��ƽ��ķ�����
                        // ���أ�pntResult������
                        static Base::Bool IntersectionLineSegmentPlane(const Engine::Geometries::Coordinate &pntSegFrom,
                                                                       const Engine::Geometries::Coordinate &pntSegTo,
                                                                       const Engine::Geometries::Coordinate &pntOrg,
                                                                       const Engine::Geometries::Coordinate &normalVector,
                                                                       Engine::Geometries::Coordinate &pntResult);

                        // ����ֱ����ƽ��Ľ��㣬�ཻ����true�����ཻ����false
                        // ���룺pntSegFrom��ֱ�ߵ��׵㣻pntSegTo��ֱ�ߵ�β�㣻pntOrg��ƽ����һ�㣻normalVector��ƽ��ķ�����
                        // ���أ�pntResult������
                        static Base::Bool IntersectionLinePlane(const Engine::Geometries::Coordinate &pntFrom,
                                                                const Engine::Geometries::Coordinate &pntTo,
                                                                const Engine::Geometries::Coordinate &pntOrg,
                                                                const Engine::Geometries::Coordinate &normalVector,
                                                                Engine::Geometries::Coordinate &pntResult);

                        /*******************************************************************
                         * @brief: �������������ƽ�淨����
                         * @author: wufuzheng
                         * @date:2018/04/25
                         * @param[in]: pnt1��pnt2��pnt3��ȷ��ƽ���������
                         * @param[out]: normalVector��ƽ��ķ�����
                         * @return: true:����ɹ���false������ʧ��
                         ******************************************************************/
                        static Base::Bool CalcPlaneNormalVector(
                            const Engine::Geometries::Coordinate &pnt1,
                            const Engine::Geometries::Coordinate &pnt2,
                            const Engine::Geometries::Coordinate &pnt3,
                            Engine::Geometries::Coordinate &normalVector);

                        /*******************************************************************
                         * @brief: ��������ƽ��Ľ���
                         * @author: wufuzheng
                         * @date:2018/04/25
                         * @param[in]: pnt1����1�ϵĵ㣬normalVector1����1�ķ�����
                         * @param[in]: pnt2����2�ϵĵ㣬normalVector2����2�ķ�����
                         * @param[out]: pntFrom��pntTo:�����ϵ�����
                         * @return: true:����ɹ���false������ʧ��
                         ******************************************************************/
                        static Base::Bool CalcPlaneIntersectionLine(
                            const Engine::Geometries::Coordinate &pnt1,
                            const Engine::Geometries::Coordinate &normalVector1,
                            const Engine::Geometries::Coordinate &pnt2,
                            const Engine::Geometries::Coordinate &normalVector2,
                            Engine::Geometries::Coordinate &pntFrom,
                            Engine::Geometries::Coordinate &pntTo);

                        // ������ά�㵽���߶ε�������룬��ȷ���ؾ���ֵ�����󷵻�-1.0
                        // ���룺pnt����ά�㣻coordinates�����ߵĽڵ㼯
                        // ���أ�pntProject����ά�㵽���߶ε�ͶӰ�㣻nSegIndex��ͶӰ���������߶ε�������
                        static Base::Double GetDistancePointToLinesegments(
                            const Engine::Geometries::Coordinate &pnt,
                            const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
                            Engine::Geometries::Coordinate &pntProject,
                            Base::Int32 &nSegIndex,
                            Base::Double tolerance = Geometries_EP);

                        static Base::Double GetDistancePointToLinesegments(
                            const Engine::Geometries::Coordinate &pnt,
                            const Base::Array<Engine::Geometries::Coordinate> coordinates,
                            Engine::Geometries::Coordinate &pntProject,
                            Base::Int32 &nSegIndex,
                            Base::Double tolerance = Geometries_EP);

                        // ������ά�㵽���߶ε�������룬��ȷ���ؾ���ֵ�����󷵻�-1.0
                        // ���룺pnt����ά�㣻coordinates�����ߵĽڵ㼯
                        // ���أ�pntProject����ά�㵽���߶ε�ͶӰ�㣻nSegIndex��ͶӰ���������߶ε�������, bFindInLine,ͶӰ�Ƿ������߶���
                        static Base::Double GetDistancePointToLinesegments(
                            const Engine::Geometries::Coordinate &pnt,
                            const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
                            Engine::Geometries::Coordinate &pntProject,
                            Base::Int32 &nSegIndex,
                            bool &bFindInLine,
                            Base::Double tolerance = Geometries_EP);
                        /*******************************************************************
                         * @brief: ������ά�㵽���߶ε��������(XY)����ȷ���ؾ���ֵ�����󷵻�-1.0
                         * @author: wufuzheng
                         * @date:2018/09/24
                         * @param[in]: pnt����ά�㣻coordinates�����ߵĽڵ㼯
                         * @param[out]: pntProject����ά�㵽���߶ε�ͶӰ��(������XY)��nSegIndex��ͶӰ���������߶ε�������
                         * @param[out]: bFindInLine,ͶӰ�Ƿ������߶���
                         * @return: �������(XY)
                         * @other: ������ע��Ϣ������˵��
                         *******************************************************************/
                        static Base::Double GetDistanceXYPointToLinesegments(
                            const Engine::Geometries::Coordinate &pnt,
                            const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
                            Engine::Geometries::Coordinate &pntProject,
                            Base::Int32 &nSegIndex,
                            bool &bFindInLine,
                            Base::Double tolerance = Geometries_EP);
                        // �������߶�1�����߶�2��ƽ�����룬(��������
                        // �����߶�1�ϵĵ������߶�2��ͶӰ��������2�ڲ��������ȥ�ĵ�ľ���
                        static Base::Bool GetDistanceLinesegmentsToLinesegments(
                            const Base::Array<Engine::Geometries::Coordinate *> *coords1,
                            const Base::Array<Engine::Geometries::Coordinate *> *coords2,
                            Base::Double &averageDis,
                            Base::Double tolerance = Geometries_EP);

                        // �������߶�1�����߶�2��ƽ�����룬
                        // �����߶�1�ϵĵ������߶�2��ͶӰ��������2�ڲ��������ȥ�ĵ�ľ���
                        static Base::Bool GetAverageDistanceLinesegmentsToLinesegments(
                            const Base::Array<Engine::Geometries::Coordinate *> *coords1,
                            const Base::Array<Engine::Geometries::Coordinate *> *coords2,
                            Base::Double &averageDis,
                            Base::Double tolerance = Geometries_EP);

                        // �ж���ֱ�߹��棬���淵��true�������淵��false
                        // ���룺pntLine0S��pntLine1S����ֱ�ߵ���㣻pntLine0E��pntLine1E����ֱ�ߵ��յ�
                        static Base::Bool IsLinesOnPlane(
                            const Engine::Geometries::Coordinate &pntLine0S,
                            const Engine::Geometries::Coordinate &pntLine0E,
                            const Engine::Geometries::Coordinate &pntLine1S,
                            const Engine::Geometries::Coordinate &pntLine1E,
                            Base::Double tolerance);

                        // ������ά����ֱ�ߵ�ͶӰ��
                        // ���룺pnt����ά�㣻pntLineS��ֱ�ߵ���㣻pntLineE��ֱ�ߵ��յ�
                        // ���أ�pntProject����ά����ֱ�ߵ�ͶӰ��
                        static Base::Bool GetProjectpntToLine(const Engine::Geometries::Coordinate &pnt,
                                                              const Engine::Geometries::Coordinate &pntLineS,
                                                              const Engine::Geometries::Coordinate &pntLineE,
                                                              Engine::Geometries::Coordinate &pntProject);

                        // ������άֱ�ߵ����ߵ�������룬��ȷ���ؾ���ֵ�����󷵻�-1.0
                        // ���룺pntLineS����άֱ�ߵ���㣻pntLineE����άֱ�ߵ��յ㣻coordinates�����ߵĽڵ㼯
                        // ���أ�pntProject����άֱ�ߵ����ߵ�ͶӰ�㣻nSegIndex��ͶӰ���������߶ε�������
                        static Base::Double GetDistanceLineToLinesegments(
                            const Engine::Geometries::Coordinate &pntLineS,
                            const Engine::Geometries::Coordinate &pntLineE,
                            const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
                            Engine::Geometries::Coordinate &pntProject,
                            Base::Int32 &nSegIndex,
                            Base::Double tolerance = Geometries_EP);

                        // �ж������߶��Ƿ��ཻ
                        static Base::Bool IsIntersectLineSegments(
                            const Engine::Geometries::Coordinate &pntSegS,
                            const Engine::Geometries::Coordinate &pntSegE,
                            const Engine::Geometries::Coordinate &pntSegFrom,
                            const Engine::Geometries::Coordinate &pntSegTo,
                            Base::Double tolerance = Geometries_EP);

                        // ������ת����0~pi��Χ��
                        static Base::Double StandardAngle(const Base::Double &angle);
                        // ����һ����������С�ļнǣ����ػ���
                        static void CalLineStringMinMaxAngle(const Geometries::LineString *pLineString,
                                                             Base::Double &maxAngel, Base::Double &minAngle);
                        // �����߶κ���ά����εı��γɵ�ƽ��Ľ��㡣��ƽ�����㣺������ά����εıߣ���xoyƽ�洹ֱ
                        // ���룺pntSegFrom����ά�߶ε���㣻pntSegTo����ά�߶ε��յ㣻coordinates����ά�����
                        // ���أ�pntResult���߶κ���ά����εı��γɵ�ƽ��Ľ���
                        static Base::Bool IntersectionLineSegmentPlanes(
                            const Engine::Geometries::Coordinate &pntSegFrom,
                            const Engine::Geometries::Coordinate &pntSegTo,
                            const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
                            Engine::Geometries::Coordinate &pntResult,
                            Base::Double tolerance = Geometries_EP);

                        static Base::Bool Flexibility3D(Base::Array<Coordinate> &arrPoints, Base::Int32 nIndex,
                                                        const Engine::Geometries::Coordinate &pntNewPos);

                        /*******************************************************************
                         * @brief: ���������Ӿ���.
                         * @author: xugz
                         * @date:2018/05/02
                         * @param[in]: pObject��Ҫ����������
                         * @param[in/out]: coordinates����Ӿ��ε�����
                         * @return: true������ɹ���false:����ʧ��
                         * @other: coordinates����Ҫ�ⲿ�ͷ��ڴ�
                         ******************************************************************/
                        static Base::Bool GetPolygonRectangle(Geometries::Polygon *pObject, Base::Array<Engine::Geometries::Coordinate *> &coordinates);

                        /*******************************************************************
                         * @brief: ����Linestring�����ཻ����
                         * @author: wufuzheng
                         * @date:2018/08/15
                         * @param[in]: pLineString:�����㼸��
                         * @return: ���ཻ���㼯�ϣ�Key�����������߶ε�������Value:���㣩
                         *******************************************************************/
                        static Base::Array<Geometries::Coordinate> CalcPointOfLinestrSelfIntersect(
                            const Geometries::LineString *pLineString);
                        /*******************************************************************
                         * @brief: �������߶���linestring�ཻ�Ľ��� ����ά��
                         * @author: xugezi
                         * @date:2018/08/21
                         * @param[in]: ����1˵�������͡�����
                         * @param[out]: ����2˵�������͡�����
                         * @return:�ཻ�Ľ��㣺���û���㷵��ֵΪ��
                         * @other: ������ע��Ϣ������˵��
                         ********************************************************************/
                        static Base::Array<Geometries::Coordinate> calIntersectPointOfLines(Geometries::LineString *pLineStringF, const Geometries::LineString *pLineStringS);
                };

        }
}

#endif // ENGINE_GEOMETRIES_BASEALGORITHM3D_H_
