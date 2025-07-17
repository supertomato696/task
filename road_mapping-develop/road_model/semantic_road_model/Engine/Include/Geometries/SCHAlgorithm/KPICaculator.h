/******************************************************************
Copyright(c) 2015-2020 Navinfo
All rights reserved.

����: dongjian04955
����: 2016-5-30 16:29
�ļ�����: KPICaculator.h
��Ҫ����: KPI �������
******************************************************************/
#ifndef _H_GEOMETRY_KPICACULATOR_
#define _H_GEOMETRY_KPICACULATOR_
#include "Geometries/Export.h"
#include "Base/Types.h"
#include "Base/Array.h"
#include "Geometries/Coordinate.h"

namespace Engine
{
    namespace Geometries
    {
        class LineString;
        class Spline;
        /*-
        KPI ��
        */
        struct KPINode
        {
            Coordinate point;  //������
            Base::UChar flag; //0x00-��״���Ӧ��KPI�㣬0x01-�����ڲ�� , 0x04 ������ֵ
            Base::UInt32 index; //type Ϊ ��״��ʱ����Ӧ ��״�������
            Base::Double value; //KPI ֵ
        };

        /*
         *	KPI ����
         */
        enum KPIType
        {
            Azimuth = 0, //����
            Curvature, //����
            CrossSlope, //����
            Slope, //����
            Other
        };

        /*
         *	KPI �������
         */

        class Geometries_API KPICacualtor
        {
        private:
            /**���������ϵ���������*/
            const LineString* m_pLineString;
            /**�������ݵ��׵�֮ǰ��һ���㣬Ϊ�������ߵ������ԡ�����ΪNULL*/
            const Coordinate* m_pHeadPoint;
            /**�������ݵ�β��֮���һ���㣬Ϊ�������ߵ������ԡ�����ΪNULL*/
            const Coordinate* m_pTailPoint;
            /**kpi ��ֵ,�ⲿ����*/
            Base::Double* m_dThreshold;

        public:
            KPICacualtor();
            ~KPICacualtor();

            /*!
             *\brief ��������
            *\ param const LineString * pLineString ��ֵ����
            *\ param const Coordinate * pHeadPoint �������ݵ��׵�֮ǰ��һ����
            *\ param const Coordinate * pTailPoint �������ݵ�β��֮���һ����
            *\ Returns:   Base::Void
            */
            Base::Void SetData(const LineString* pLineString,const Coordinate* pHeadPoint,const Coordinate* pTailPoint );

            /*!
             *\brief ������ֵ��ֵ
            *\ param Base::Double threshold
            *\ Returns:   Base::Void
            */
            Base::Void SetThreshold(Base::Double threshold);

            /*!
             *\brief  �������KPI���ߡ������������ϵ�ķ��������򣬸���KPIֵ��Сƫ��
             һ������õ�����KPI�㡣
            *\ param KPIType kpiָ������
            *\ param Base::Double maxOffset ���ƫ�ƾ��룬��λ���������굥λһ��
            *\ param Base::Double & maxKpi ���KPI ֵ ���޷���
            *\ param Base::Array<KPINode> & vecKPINodes KPI �����
            *\ Returns:   Base::Void
            */
            Base::Void AdjointCurve(KPIType eType, Base::Double maxOffset, Base::Double& maxKpi, Base::Array<KPINode>& vecKPINodes) const;

            /*!
             *\brief �������KPI���� ��KPIֵ����ϵ�����ⲿָ��
            *\ param KPIType kpiָ������
            *\ param Base::Double maxOffset
            *\ param Base::Double scale
            *\ param Base::Array<KPINode> & vecKPINodes
            *\ Returns:   Base::Void
            */
            Base::Void AdjointCurveFixedScale(KPIType eType,Base::Double maxOffset, Base::Double scale, Base::Array<KPINode>& vecKPINodes) const;

            /*!
             *\brief ��ȡ��ֵ������ɢ��֮�������
            *\ param Base::Double tolerance ���ߴ������߻�������Ҹ�
            *\ Returns:   LineString* ��ɢ��������
            */
            LineString*  GetSplineTesselation(Base::Double tolerance) const;

        private:
            /**�������� */
            Base::Void AdjointCurvatureCurve(Base::Double maxOffset, Base::Bool fixedScale, Base::Double* scale,
                                             Base::Double* maxCurvature,Base::Array<KPINode>& vecKPINodes) const;

            /**��λ������*/
            Base::Void AdjointAzimuthCurve(Base::Double maxOffset, Base::Bool fixedScale, Base::Double* scale,
                                           Base::Double* maxAzimuth, Base::Array<KPINode>& vecKPINodes) const;

            /**��������*/
            Base::Void AdjointSlopeCurve(Base::Double maxOffSet, Base::Bool fixedScale, Base::Double* scale,
                                         Base::Double* maxSlope,Base::Array<KPINode>& vecKPINodes) const;

            /**������������*/
            Base::Bool AdjointCurve(const Base::Array<Coordinate>& vecLocation, const Base::Array<Base::Double>& vecValues,
                                    const Base::Array<Vector3d>& vecNormalVectors, Base::Double scale, Base::Array<KPINode>& vecNodes) const;

            /*�����ߵİ�������*/
            Base::Void AdjointCurveLineSegment(KPIType eType, Base::Double maxOffset, Base::Double& maxKpi,Base::Bool bFixedScale,Base::Double dScale ,Base::Array<KPINode>& vecKPINodes) const;

            Base::Void AdjointCurveFixedScaleLineSegment(KPIType eType, Base::Double maxOffset, Base::Double scale, Base::Array<KPINode>& vecKPINodes) const;
        };

        /*
         *	POSITION  ��KPI������
         */
        class Geometries_API PositionKPICacualtor
        {
        private:
            /**���������ϵ���������*/
            const LineString* m_pLineString;
            /**�������ݵ��׵�֮ǰ��һ���㣬Ϊ�������ߵ������ԡ�����ΪNULL*/
            const Coordinate* m_pHeadPoint;
            /**�������ݵ�β��֮���һ���㣬Ϊ�������ߵ������ԡ�����ΪNULL*/
            const Coordinate* m_pTailPoint;

        public:
            static const Base::Double c_POSITION_INTERVEL; //KPI��λ�ü��

            struct PositionKPI
            {
                Coordinate Pos;
                Base::Double dAzimuth;			//�����
                Base::Double dCurvature;		// ����
                Base::Double dSlope;				//����
                Base::Double dCrossSlope;		//���� 2015.6.13 ��δʵ��
                Base::Int32 nSeqNum;				//position ���
                //positionKPI ��ʼ������
                PositionKPI();
            };

            /**����/��������*/
            PositionKPICacualtor();
            ~PositionKPICacualtor();

            /*!
             *\brief ��������
            *\ param const LineString * pLineString �������position ����
            *\ param const Coordinate * pHeadPoint �� pLineString ���������Ե� ��һ·�������ڶ���
            *\ param const Coordinate * pTailPoint  �� pLineString ���������Ե� ��һ·���� �ڶ���
            *\ Returns:   Base::Void
            */
            Base::Void SetData(const LineString* pLineString, const Coordinate* pHeadPoint, const Coordinate* pTailPoint);

            /*!
             *\brief ����KPI
            *\ param Base::Array<PositionKPI> & vecPosKPIs
            *\ Returns:   Base::Bool
            */
            Base::Bool CaculateKPI(Base::Array<PositionKPI>& vecPosKPIs) const;

        private:
            struct InternalPositionNode :public PositionKPI
            {
                Base::Double dXYParameter; //��Ӧ�����ϵĲ���ֵ
                Base::Double dZParameter;
                Base::Double dDistance; //������������ڵ�ľ���
                Base::Double dXYDistance;//xyƽ���ھ����������ڵ�ľ���
                Base::UInt32 nIndex; //�����������ڵ�������
            };

            /**���������ϵ㵽 startIndex �� distance �õ����� pSpline �ϵ� parameter */
            Base::Double GetParamByDistance(const Spline* pSpline,Base::UInt32 startIndex, Base::Double distance) const;

            /**����position ��*/
            Base::Void GeneratePositionNode(Base::Array<InternalPositionNode>& vecPositions) const;

            /** XY ���� parameter ,���ڼ��� ��������*/
            Base::Void SolveParameterValue_XY(const Spline* pSpline,Base::Array<InternalPositionNode>& vecPositions) const;

            /**XY Length ��Z���� parameter ,���ڼ��� ����*/
            Base::Void SolveParameterValue_Z(const Spline* pSpline, Base::Array<InternalPositionNode>& vecPositions) const;

            /**���㷽λ��/����*/
            Base::Void CaculateAzimuth(const Spline* pSpline, Base::Array<InternalPositionNode>& vecPositions)const;

            /**��������*/
            Base::Void CaculateCurvature(const Spline* pSpline, Base::Array<InternalPositionNode>& vecPositions)const;

            /**�������� by ��С���˷�*/
            //Base::Void CaculateCurvature(Base::Array<InternalPositionNode>& vecPositions) const;

            //��С���˷�����SCH
            Base::Void CaculateSCHbyLeastSquareFit(Base::Array<InternalPositionNode>& vecPositions) const;


            /**��������*/
            Base::Void CaculateSlope(const Spline* pSpline, Base::Array<InternalPositionNode>& vecPositions)const;
        };

    }
}




#endif //_H_GEOMETRY_KPICACULATOR_
