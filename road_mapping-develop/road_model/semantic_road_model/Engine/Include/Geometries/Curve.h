/******************************************************************
作者: test
日期: 2021-8-18 11:19
******************************************************************/
#ifndef ENGINE_GEOMETRIES_CURVE_H_
#define ENGINE_GEOMETRIES_CURVE_H_
#include "Geometry.h"

namespace Engine
{
    namespace Geometries
    {
        class LineString;
        class Geometries_API Curve : public Geometry
        {
        public:
            Curve() {}
            ~Curve() {}

            /*!
             *\brief �����u ��Ӧ�ĵ� point
             *\ param Base::Double param ����ֵ
             *\ param Coordinate & point ��
             *\ Returns:   Base::Bool param�Ƿ�Ϸ�
             */
            virtual Base::Bool Evaluate(Base::Double param, Coordinate &point) const = 0;

            /*!
             *\brief ��ɢ��������
             *\ param Base::Double tolerance ��ɢ���ݲ�(�Ӿ�������ȷ������)
             *\ Returns:   LineString*
             */
            virtual LineString *Tesselation(Base::Double tolerance) const = 0;

            /*!
             *\brief һ�׵���
             *\ param Base::Double param ����ֵ
             *\ param Vector3d & firstDeriv ����ֵ
             *\ Returns:   Base::Bool ������Ч��
             */
            virtual Base::Bool GetFirstDeriv(Base::Double param, Vector3d &firstDeriv) const = 0;

            /*!
             *\brief ���׵���
             *\ param Base::Double param ����ֵ
             *\ param Vector3d & secondDeriv ����ֵ
             *\ Returns:   Base::Bool ������Ч��
             */
            virtual Base::Bool GetSecondDeriv(Base::Double param, Vector3d &secondDeriv) const = 0;

            /*!
             *\brief ��ȡ��ʼ param
             *\ Returns:   Base::Double
             */
            virtual Base::Double GetStartParam() const = 0;

            /*!
             *\brief ��ȡ��ֹparam
             *\ Returns:   Base::Double
             */
            virtual Base::Double GetEndParam() const = 0;
        };
    }
}

#endif // ENGINE_GEOMETRIES_CURVE_H_