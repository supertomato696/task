/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Circle.h
简要描述:圆
******************************************************************/

#ifndef ENGINE_GEOMETRIES_CIRCLE_H_
#define ENGINE_GEOMETRIES_CIRCLE_H_

#include "Geometry.h"

namespace Engine
{
    namespace Geometries
    {
        class Polygon;

        class Geometries_API Circle : public Geometry
        {
        public:
            // 构造函数
            // 返回：圆。满足：圆上第一个点是构造函数的第一个传入参数，
            // 圆上第二个点，是以圆上第一个点为端点的直径的另一个端点，
            // 圆上第三个点，是圆上第二个点逆时针旋转90度后的点
            Circle(const Engine::Geometries::Coordinate &pnt1,
                   const Engine::Geometries::Coordinate &pnt2,
                   const Engine::Geometries::Coordinate &pnt3);

            Circle(const Engine::Base::Array<Coordinate> &controlPoints);

            // 析构函数
            ~Circle();

            // 				//复制构造函数
            // 				Circle(const Circle &rhs);
            //
            // 				//赋值操作符
            // 				Circle& operator =(const Circle &rhs);

            // 返回GeometryType::CIRCLE
            GeometryType GetGeometryType() const;

            // 返回Circle对象的外包框
            Envelope *const GetEnvelope() const;

            // 返回Circle对象的4个控制点
            Base::Array<Coordinate> GetControlPoints();

            // 返回Circle对象的深拷贝对象
            // 注意：需要由调用者释放所返回的指针
            Geometry *const Clone() const;

            // 获得圆上的三个顶点
            Base::Array<Coordinate> GetCoordinates();

            // 获得圆心
            Coordinate GetCenter();

            // 获得圆半径
            Base::Double GetRadius();

            // 圆的平移
            Base::Void Offset(Base::Double dx, Base::Double dy, Base::Double dz);

            // 圆的缩放
            Base::Void Resize(Base::Double radius);

            virtual Base::Void Rotate(Base::UInt16 index, Base::Double angle);

            // 从Polygon构造
            static Circle *FromPolygon(Geometries::Geometry *pGeometry);

            // 根据分段数将圆分割，生成新的多边形
            Polygon *ToPolygon(Base::UInt16 subsectionnum);

            // 获得圆平面的法向量
            Coordinate GetNormalVector();

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
            // 圆上三点
            Coordinate m_coordinate1;
            Coordinate m_coordinate2;
            Coordinate m_coordinate3;

            // 圆心
            Coordinate m_center;

            // 圆平面的法向量
            Coordinate m_normalVector;

            // 圆半径
            Base::Double m_radius;

            // 计算圆心
            Base::Void ComputeCenter();

            // 计算圆平面的法向量
            Base::Void ComputeNormalVector();
        };
    }
}

#endif // ENGINE_GEOMETRIES_CIRCLE_H_
