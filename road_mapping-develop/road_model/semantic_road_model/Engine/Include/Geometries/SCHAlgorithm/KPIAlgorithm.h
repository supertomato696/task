/******************************************************************
Copyright(c) 2015-2020 Navinfo
All rights reserved.

作者:dongjian
日期:2016-06-13 21:38:32
文件名称:KPIAlgorithm.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_KPIALGORITHM_H_ 
#define ENGINE_GEOMETRIES_KPIALGORITHM_H_  

#include "Base/Array.h"
#include "Base/Types.h"
#include "Geometries/Export.h"

namespace Engine
{
    namespace Geometries
    {
        class Spline;
        class Coordinate;

        /*
         *	样条曲线内插
         */
        class  Geometries_API SplineInterpolateUtility
        {
        public:

            /*!
             *\brief XY 坐标内插生成曲线 （Z值为0）
            *\ param Spline * pSpline 待内插曲线
            *\ param const Base::Array<Coordinate * > * pCoorArray 坐标
            *\ param const Coordinate * pHeadPoint 与 pLineString 保持连贯性的 上一路链倒数第二点
            *\ param const Coordinate * pTailPoint 与 pLineString 保持连贯性的 下一路链的 第二点
            *\ Returns:   Base::Bool
            */
            static Base::Bool InterpolateFlatZSpline(Spline* pSpline,const Base::Array<Coordinate*>* pCoorArray,const Coordinate* pHeadPoint,const Coordinate* pTailPoint);
            /*!
             *\brief XY-LENGTH ，Z值 曲线，用于计算坡度
            *\ param Spline * pSpline 待内插曲线
            *\ param const Base::Array<Coordinate * > * pCoorArray 坐标
            *\ param const Coordinate * pHeadPoint pLineString 保持连贯性的 上一路链倒数第二点
            *\ param const Coordinate * pTailPoint 与 pLineString 保持连贯性的 下一路链的 第二点
            *\ Returns:   Base::Bool
            */
            static Base::Bool InterpolateSlopeSpline(Spline* pSpline, const Base::Array<Coordinate*>* pCoorArray, const Coordinate* pHeadPoint, const Coordinate* pTailPoint);
        };
    }
}

#endif //ENGINE_GEOMETRIES_KPIALGORITHM_H