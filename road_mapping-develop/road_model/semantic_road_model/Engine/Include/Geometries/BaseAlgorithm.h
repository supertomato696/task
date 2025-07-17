/******************************************************************
作者: test
日期: 2021-8-18 11:19
******************************************************************/

#ifndef ENGINE_GEOMETRIES_BASEALGORITHM_H_
#define ENGINE_GEOMETRIES_BASEALGORITHM_H_

#include "Geometries/Coordinate.h"
#include "Base/Array.h"
#include "Base/Macros.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

namespace Engine
{
        namespace Geometries
        {
                class Geometries_API BaseAlgorithm
                {
                public:
                        // ���㷽λ��
                        static Base::Double CalAzimuth(const Coordinate *pPointFrom, const Coordinate *pPointTo);

                        // 3��н�,���ػ��ȵ�λ,֧��3ά����
                        static Double ComputeAngle(const Coordinate &coord1, const Coordinate &coord2, const Coordinate &coord3);

                        // �����ת
                        static Base::Void RotatePoint(Coordinate *pPointOrg, double dAngle, Coordinate *pPointResult);

                        // �㵽ֱ�߾���
                        static Base::Double DistancePtToLine(
                            const Coordinate *pt, const Coordinate *ptStart,
                            const Coordinate *ptEnd);
                        static Base::Double DistancePtToLine2D(
                            const Coordinate *pt, const Coordinate *ptStart,
                            const Coordinate *ptEnd);

                        static Base::Bool IsProjectToLineset(
                            const Engine::Geometries::Coordinate *pntHitTest,
                            const Engine::Geometries::Coordinate *pntLinsectStart,
                            const Engine::Geometries::Coordinate *pntLinsectEnd);

                        static Engine::Geometries::Coordinate GetPtToLine(
                            const Engine::Geometries::Coordinate *pntStart,
                            const Engine::Geometries::Coordinate *pntEnd,
                            const Engine::Geometries::Coordinate *pntHitTest);

                        // ��ά��͵�ľ������
                        static Base::Double DistancePtToPt(
                            const Engine::Geometries::Coordinate *pntFrom,
                            const Engine::Geometries::Coordinate *pntTo);

                        // �Ƿ�Ϊ0���жϷ���
                        static Base::Bool Is0(const double &dValue);

                        // �жϵ��Ƿ���ֱ����
                        static Base::Bool IsPointOnLine(
                            const Engine::Geometries::Coordinate &pntFrom,
                            const Engine::Geometries::Coordinate &pntTo,
                            const Engine::Geometries::Coordinate &pntTest);

                        // ����0  ��ֱ����
                        // ����1  ��ֱ�����
                        // ����2  ��ֱ���Ҳ�
                        static Base::UInt16 PntMatchLine(
                            const Engine::Geometries::Coordinate &pntFrom,
                            const Engine::Geometries::Coordinate &pntTo,
                            const Engine::Geometries::Coordinate &pntTest);

                        // ����0  ��������
                        // ����1  ���������
                        // ����2  �������Ҳ�
                        static Base::UInt16 PntMatchLineSegments(
                            const Base::Array<Engine::Geometries::Coordinate *> *pPoints,
                            const Engine::Geometries::Coordinate &pntTest);

                        // ������Ͼ�����������ĵ�
                        static Base::Bool GetNearestPntToLineset(
                            const Engine::Geometries::Coordinate *pntHitTest,
                            const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
                            Engine::Geometries::Coordinate &pntProject,
                            Int32 &nSegIndex);
                        static Base::Bool GetNearestPntToLineset(
                            const Engine::Geometries::Coordinate &pntHitTest,
                            const Base::Array<Engine::Geometries::Coordinate> &coordinates,
                            Engine::Geometries::Coordinate &pntProject,
                            Int32 &nSegIndex);
                        /*******************************************************************
                         * @brief: �������pntHitTest 2D��������ĵ㣬������3D������
                         * @author: wufuzheng
                         * @date:2018/04/19
                         * @param[in]: pntHitTest��������һ�㣬coordinates������
                         * @param[out]: pntProject������pntHitTest����ĵ�
                         * @return: true:�ɹ���false������ʧ��
                         ******************************************************************/
                        static Base::Bool GetNearestPntToLineset2D(
                            const Engine::Geometries::Coordinate *pntHitTest,
                            const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
                            Engine::Geometries::Coordinate &pntProject);
                        // ����㼯���������
                        // ���룺pntsSource��ԭʼ�㼯
                        // ���أ�pntsSource��ԭʼ�㼯��������ߵĵ㼯
                        static Void PointsPackage(
                            const Array<Coordinate> pntsSource,
                            Array<Coordinate> &pntsRezult);

                        // ����㼯���������
                        // ���룺pntsSource��ԭʼ�㼯
                        // ���أ�pntsSource��ԭʼ�㼯��������ߵĵ㼯
                        static Void PointsPackage(
                            const Array<Coordinate *> *pntsSource,
                            Array<Coordinate *> *pntsRezult);

                        /*!
                         *\brief ���� hitPointȡ ������ ����� p,���� p �� ���ߴ��ߣ��õ� ���ߵ� p0,p1
                         *\ param const Coordinate & hitPoint ��׽��
                         *\ param const Base::Array<Navinfo::Engine::Geometries::Coordinate * > * coordinates ��������
                         *\ param Coordinate & p0 ֱ�ߵ� 0
                         *\ param Coordinate & p1 ֱ�ߵ� 1
                         *\ Returns:   Base::Bool �Ƿ�ɹ�
                         */
                        static Base::Bool PerpendicularLine(const Coordinate &hitPoint,
                                                            const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
                                                            Coordinate &p0, Coordinate &p1);
                };
        }
}

#endif // ENGINE_GEOMETRIES_BASEALGORITHM_H_
