/******************************************************************
作者: test
日期: 2021-8-18 11:19
******************************************************************/
#ifndef ENGINE_GEOMETRIES_SPLINE_H_
#define ENGINE_GEOMETRIES_SPLINE_H_
#include "Curve.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API Spline : public Curve
        {
        private:
            /**����*/
            Base::UInt16 m_nDegree;
            /**�ڵ�ʸ��*/
            Base::Array<Base::Double> m_vecKnots;
            /**���Ƶ�����*/
            Base::Array<Coordinate> m_vecControlVertexes;

        public:
            Spline();

            /*!
             *\brief ���캯��
             *\ param Base::UInt16 degree ����
             *\ param const Base::Array<Base::Double> & vecKnots �ڵ�ʸ��
             *\ param const Base::Array<Coordinate> & vecControlVertexes ���Ƶ�����
             *\ Returns:
             */
            Spline(Base::UInt16 degree, const Base::Array<Base::Double> &vecKnots,
                   const Base::Array<Coordinate> &vecControlVertexes);

            /*!
             *\brief ���캯��
             *\ param const Spline & rhs
             *\ Returns:
             */
            Spline(const Spline &rhs);

            ~Spline();

        public:
            /*!
             *\brief ��ȡ��������
             *\ Returns:   Engine::Geometries::GeometryType
             */
            GeometryType GetGeometryType() const;

            /*!
             *\brief ��ȿ���
             *\ Returns:   Geometry* const
             */
            Geometry *const Clone() const;

            /*!
             *\brief ��ȡenvelope
             *\ Returns:   Envelope* const
             */
            Envelope *const GetEnvelope() const;

            /*!
             *\brief �����u ��Ӧ�ĵ� point
             *\ param Base::Double param ����ֵ
             *\ param Coordinate & point ��
             *\ Returns:   Base::Bool param�Ƿ�Ϸ�
             */
            Base::Bool Evaluate(Base::Double param, Coordinate &point) const;

            /*!
             *\brief һ�׵���
             *\ param Base::Double param ����ֵ
             *\ param Vector3d & firstDeriv ����ֵ
             *\ Returns:   Base::Bool ������Ч��
             */
            Base::Bool GetFirstDeriv(Base::Double param, Vector3d &firstDeriv) const;

            /*!
             *\brief һ�׵��� ע�⣺B�����ĵ���Ϊ�״μ�һ��B�������ߣ����Ƶ�������һ��
             *\ Returns:   Engine::Geometries::Spline
             */
            Spline GetFirstDeriv() const;

            /*!
             *\brief ���׵���
             *\ param Base::Double param ����ֵ
             *\ param Vector3d & secondDeriv ����ֵ
             *\ Returns:   Base::Bool ������Ч��
             */
            Base::Bool GetSecondDeriv(Base::Double param, Vector3d &secondDeriv) const;

            /*!
             *\brief ��ɢ��������
             *\ param Base::Double tolerance ���ߴ������߻�������Ҹ�
             *\ Returns:   LineString* ��ɢ��������
             */
            LineString *Tesselation(Base::Double tolerance) const;

            /*!
             *\brief ��ȡ��ʼ param
             *\ Returns:   Base::Double
             */
            virtual Base::Double GetStartParam() const;

            /*!
             *\brief ��ȡ��ֹparam
             *\ Returns:   Base::Double
             */
            virtual Base::Double GetEndParam() const;

            /*!
             *\brief	������ֵ�㼰��β���������ڲ����ߣ��ڲ�������ͨ����ֵ��
                            ����������Ϊ�ۼ��ҳ���(chord length)
            *\ param const Base::Array<Coordinate * > * pPoints ��ֵ������
            *\ param const Vector3d & headTangent �׵㴦��ʸ��
            *\ param const Vector3d & tailTangent ĩ�㴦��ʸ��
            *\ Returns:   Base::Void
            */
            Base::Void Interpolate3Degree(const Base::Array<Coordinate *> *pPoints, const Vector3d &headTangent, const Vector3d &tailTangent);

            /*!
             *\brief ������״���ڲ�3����������,��ĩ����ʸ������ bessel ��������
             *\ param const Base::Array<Coordinate * > * pPoints
             *\ Returns:   Base::Void
             */
            Base::Void Interpolate3Degree(const Base::Array<Coordinate *> *pPoints);

            /*!
             *\brief ��������������ߡ������������ϵ�ķ��������򣬸�������ֵ��Сƫ��
             һ������õ��������ʵ�������Ӻ�����ߡ�
            *\ param Base::Double maxOffset ������������������ߵ����ƫ�ƾ��룬��λ���������굥λһ��
            *\ param Base::Bool fixedScale �Ƿ�ʹ�ù̶�ϵ����
                false: ����ϵ�����̶�����ǰ ����ϵ�� s Ϊ  maxoffset /maxCurvature ����ǰ������� maxcurvature ��Ϊ��������
                true: ����ϵ���̶���naxcurvature ��Ϊ�����������ǰ����ϵ�� s Ϊ maxoffset /maxCurvature
            *\ param Base::Double & maxCurvature �������ֵ ���޷���
            *\ Returns:   LineString* ���ʰ�������
            */
            LineString *AdjointCurvatureCurve(Base::Double maxOffset, Base::Bool fixedScale, Base::Double &maxCurvature) const;

            /** p0,p1,p2 Ϊ��������㣬indexΪ���㵼���ĵ�������� index ����[0,2]*/
            /*!
             *\brief Bessel �������㵼�� ����������Լ���Ķ���B������ֵ���ߵ��о�_��ǫ��
             *\ param const Coordinate * p0 ���˳��Ϊ����������
             *\ param Coordinate * p1
             *\ param Coordinate * p2
             *\ param Base::UInt16 index ��������
             *\ Returns:   Engine::Geometries::Vector3d ��ʸ��
             */
            static Vector3d BesselDerivative(const Coordinate *p0, const Coordinate *p1, const Coordinate *p2, Base::UInt16 index);

            /*!
             *\brief ��ȡ�ڵ�ʸ��
             *\ Returns:   const Base::Array<Base::Double>&
             */
            const Base::Array<Base::Double> &GetKnotsVector() const;

            /*!
             *\brief ��ȡ���Ƶ�����
             *\ Returns:   const Base::Array<Coordinate>&
             */
            const Base::Array<Coordinate> &GetControlVertexes() const;

            /*!
             *\brief ����
             *\ Returns:   Base::UInt16
             */
            Base::UInt16 Degree() const;

            /*!
             *\brief ����(�з��� �� һ�׵�������׵������ z ��������ȷ��)
             *\ param const Base::Array<Base::Double> & params
             *\ param Base::Array<Base::Double> & curvatures
             *\ Returns:   Base::Void
             */
            Base::Void Curvature(const Base::Array<Base::Double> &params, Base::Array<Base::Double> &curvatures) const;

            /**�������*/
            Base::Void Clear();

        private:
            /**��ɢ��*/
            Base::UInt32 _Tesselation(Base::SizeT begin, Base::SizeT end, Base::Double tolerance,
                                      Base::Array<Coordinate> &vecControlVertexes, Base::Array<Base::Double> &vecKnots) const;
            /**�Ҹ�С��limit*/
            Base::Bool ChordHeightLessThan(const Base::Array<Coordinate> &vecCVs, Base::UInt32 begin, Base::UInt32 end, Base::Double limit) const;
            /**B���������� u: ����u ; nSpan : uλ�� ���� [nSpan , nSpan + 1] */
            Base::Void BasisFuns(Base::Double u, Base::UInt32 nSpan, Base::Array<Base::Double> &vecBasis) const;
            /**Bessel �������Ƹ����㷨*/
            static Vector3d Bessel_detP(const Coordinate *p0, const Coordinate *p1, const Coordinate *p2, int nIndex);
            static Base::Double Bessel_detT(const Coordinate *p0, const Coordinate *p1, const Coordinate *p2, int nIndex);
            /**�����Ƿ����*/
            Base::Bool IsParameterValid(Base::Double param) const;
        };
    }
}

#endif // ENGINE_GEOMETRIES_SPLINE_H_