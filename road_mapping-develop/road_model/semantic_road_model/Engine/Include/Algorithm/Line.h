/******************************************************************
作者: test
日期: 2021-8-18 11:19

******************************************************************/
#ifndef _3EF59A27_A645_479E_9E12_C58DABF5E271
#define _3EF59A27_A645_479E_9E12_C58DABF5E271
#include "Algorithm/Algorithm.h"
#include "Geometries/Coordinate.h"
#include "Base/Types.h"

namespace geos
{
    namespace geom
    {
        class Geometry;
    }
}

namespace Engine
{
    namespace Algorithm
    {
        class Algorithm_API LineAlgorithm
        {
        public:
            static Base::Bool LineEquation(const Geometries::Coordinate &c0, const Geometries::Coordinate &c1, Base::Double &A, Base::Double &B, Base::Double &C);

            static geos::geom::Geometry *CreateGeosLineString(const Base::Array<Geometries::Coordinate> &vecCoords);

            static geos::geom::Geometry *CreateLineStringBuffer(Base::Array<Geometries::Coordinate> &vecCoords, Base::Double dBufferDis, Base::Bool roundCap = false);

            /*!
             *\brief DouglasPeucker ���Դ������ݻ���
             *\ param const Base::Array<Geometries::Coordinate> & vecInput �����
             *\ param const Base::Double dTolerance �ݲ�
             *\ param Base::Array<Geometries::Coordinate> & vecOutput �����
             *\ Returns:   Base::Void
             */
            static Base::Void DouglasPeuckerSimplify(const Base::Array<Geometries::Coordinate> &vecInput, const Base::Double dTolerance, Base::Array<Geometries::Coordinate> &vecOutput);

            /*!
             *\brief ɾ���ظ���״��
             *\ param Base::Array<Geometries::Coordinate * > & vecInput
             *\ param const Base::Double dTolerance
             *\ Returns:   Base::Void
             */
            static Base::Void RemoveDuplicatePoints(Base::Array<Geometries::Coordinate *> &vecInput, const Base::Double dTolerance);
            static Base::Void RemoveDuplicatePoints(Base::Array<Geometries::Coordinate> &vecInput, const Base::Double dTolerance);

            static Base::Void SmoothSTurnSegments(Base::Array<Geometries::Coordinate> &vecInput, const Base::Double s_turn_segment_max_length,
                                                  const Base::Double s_turn_angle_degree, const Base::Double chord_height);
        };
    }
}
#endif // !_3EF59A27_A645_479E_9E12_C58DABF5E271
