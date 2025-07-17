/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称: GeometricFit.h
简要描述:
******************************************************************/
#ifndef _99FE2AD4_CA95_4F56_AFB8_946E92DFCFDF
#define _99FE2AD4_CA95_4F56_AFB8_946E92DFCFDF
#include "Algorithm/Algorithm.h"
#include "Geometries/Coordinate.h"
#include "Base/Types.h"

namespace Engine
{
    namespace Algorithm
    {
        /************************************************************************/
        /* 注意：
            GeometricFit 不会在内部对坐标做平移处理，
            如果坐标绝对值较大，建议将坐标整体平移后传入 GeometricFit
            以保证结果的精度
        */
        /************************************************************************/

        class Algorithm_API GeometricFit
        {
        public:
            /*!
             *\brief  最小二乘法3维直线拟合 方程 l = pointOnLine + directVector*t;
             *\ param const Base::Array<Geometries::Coordinate> & vecPoints
             *\ param Geometries::Vector3d & directVector 方向向量
             *\ param Geometries::Coordinate & pointOnLine 线上一点，这里取的是 vecPoints的中心点
             *\ Returns:   Base::Bool 成功与否
             */
            static Base::Bool LeastSquare3DLineFit(const Base::Array<Geometries::Coordinate> &vecPoints, Geometries::Vector3d &directVector,
                                                   Geometries::Coordinate &pointOnLine);

            /*!
             *\brief 最小二乘法直线提取 方程： Ax + By + C = 0
             *\ param Base::Array<Base::Coordinate> & vecPoints
             *\ param Base::Double & dA
             *\ param Base::Double & dB
             *\ param Base::Double & dC
             *\ Returns:   Base::Void
             */
            static Base::Bool LeastSquareLineFit(const Base::Array<Geometries::Coordinate> &vecPoints,
                                                 Base::Double &dA, Base::Double &dB, Base::Double &dC);

            static Base::Bool LeastSquareLineFit(const Base::Array<Geometries::Coordinate> &vecPoints,
                                                 Base::Int32 nStartIndex, Base::Int32 nEndIndex,
                                                 Base::Double &dA, Base::Double &dB, Base::Double &dC);

            /*!
             *\brief 最小二乘法线段拟合，计算线段首尾，注意：首尾顺序与输入点顺序无关
             *\ param const Base::Array<Geometries::Coordinate> & vecPoints
             *\ param Geometries::Coordinate & pntStart
             *\ param Geometries::Coordinate & pntEnd
             *\ Returns:   Base::Bool
             */
            static Base::Bool LeastSquareLineFit(const Base::Array<Geometries::Coordinate> &vecPoints,
                                                 Geometries::Coordinate &pntStart, Geometries::Coordinate &pntEnd);

            /*!
             *\brief 几何拟合，涉及迭代，效率相对低，解最精确，适合一般圆拟合以及
             非常规拟合（如 半径无限大）
            *\ param const Base::Array<Geometries::Coordinate> & vecPoints
            *\ param Base::Double & x  圆心 X
            *\ param Base::Double & y  圆心 y
            *\ param Base::Double & r 半径 当 isfinite(r) == false 时，拟合结果为直线
            *\ Returns:   Base::Bool
            */
            static Base::Bool LeastSquareCircleFit_Geometric(const Base::Array<Geometries::Coordinate> &vecPoints, Base::Double &x, Base::Double &y, Base::Double &r);

            /*!
             *\brief 代数拟合，效率高，解在普通拟合（数据误差远小于半径，且半径范围正常）中很精确
             *\ param const Base::Array<Geometries::Coordinate> & vecPoints
             *\ param Base::Double & x 圆心 X
             *\ param Base::Double & y 圆心 y
             *\ param Base::Double & r 半径 当 isfinite(r) == false 时，拟合结果为直线
             *\ Returns:   Base::Bool
             */
            static Base::Bool LeastSquareCircleFit_Algebraic(const Base::Array<Geometries::Coordinate> &vecPoints, Base::Double &x, Base::Double &y, Base::Double &r);

        private:
            static Base::Void AverageXYZ(const Base::Array<Geometries::Coordinate> &vecPoints, Base::Double &x, Base::Double &y, Base::Double &z);

            static Base::Double DeviationAnalysis(const Base::Array<Geometries::Coordinate> &vecPoints, const Base::Double &x, const Base::Double &y, const Base::Double &r);
        };
    }
}
#endif //_99FE2AD4_CA95_4F56_AFB8_946E92DFCFDF