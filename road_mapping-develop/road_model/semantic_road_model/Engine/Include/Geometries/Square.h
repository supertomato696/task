/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Square.h
简要描述:正方形
******************************************************************/

#ifndef ENGINE_GEOMETRIES_SQUARE_H_
#define ENGINE_GEOMETRIES_SQUARE_H_

#include "Rectangle.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API Square : public Rectangle
        {
        public:
            // 构造函数
            Square(const Engine::Geometries::Coordinate &pntSegFrom,
                   const Engine::Geometries::Coordinate &pntSegTo,
                   const Engine::Geometries::Coordinate &pnt);

            Square(const Engine::Base::Array<Engine::Geometries::Coordinate> &controlPoints);

            // 析构函数
            ~Square();

            // 				//复制构造函数
            // 				Square(const Square &rhs);
            //
            // 				//赋值操作符
            // 				Square& operator =(const Square &rhs);

            // 返回GeometryType::SQUARE
            GeometryType GetGeometryType() const;

            // 返回Square对象的外包框
            Envelope *const GetEnvelope() const;

            // 获得正方形的四个角点
            Base::Array<Coordinate> GetCoordinates();

            Base::Array<Coordinate> GetControlPoints();

            // 返回Square对象的深拷贝对象
            // 注意：需要由调用者释放所返回的指针
            // Geometry* const Clone() const;

            // 获得正方形的中心
            Coordinate GetCenter();

            // 正方形的平移
            Base::Void Offset(Base::Double dx, Base::Double dy, Base::Double dz);

            // 正方形的缩放
            Base::Bool Resize(Base::UInt16 index, const Engine::Geometries::Coordinate &pnt);

            // 正方形的旋转
            Base::Void Rotate(Base::Double angle);

            Base::Void Rotate(Base::UInt16 index, Base::Double angle);

            // 获得长，即正方形的边长
            Base::Double GetLength();

            // 设置长，即正方形的边长
            Base::Void SetLength(Base::Double length);

            // 根据点旋转 //add by duanzhikang 2016-8-11
            Base::Void RotateByCoord(Base::UInt16 index, const Engine::Geometries::Coordinate &pt);

            // 返回控制点对侧的控制点序号 //add by duanzhikang 2016-8-11
            Base::Int32 GetOppositeControlPointIndex(Base::Int32 index);

            // 返回控制点旋转面的3个点 //add by duanzhikang 2016-8-11
            Base::Void GetControlPointFaceCoords(Base::UInt16 index, Engine::Geometries::Coordinate &pt1,
                                                 Engine::Geometries::Coordinate &pt2,
                                                 Engine::Geometries::Coordinate &pt3);

            // 返回控制点所在边的向量 //add by duanzhikang 2016-8-11
            Engine::Geometries::Coordinate GetControlPointFaceVector(Base::Int32 index);

        private:
            // 正方形的四个角点
            Base::Array<Coordinate> m_coordinates;

            // 正方形的中心
            Coordinate m_center;

            // 正方形平面的法向量
            Coordinate m_normalVector;

            // 计算正方形的中心
            Base::Void ComputeCenter();

            // 计算正方形平面的法向量
            Base::Void ComputeNormalVector();
        };
    }
}

#endif // ENGINE_GEOMETRIES_SQUARE_H_
