/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Triangle.h
简要描述:三角形
******************************************************************/

#ifndef ENGINE_GEOMETRIES_TRIANGLE_H_
#define ENGINE_GEOMETRIES_TRIANGLE_H_

#include "Geometry.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API Triangle : public Geometry
        {
        public:
            // 构造函数
            // 输入：pnt1，A；pnt2：B；pnt3，C
            // 返回：等边三角形。满足：线段AB为底，点C到AB的距离为高，等边三角形的第三个顶点与点C在直线AB的同一侧
            Triangle(const Engine::Geometries::Coordinate &pnt1,
                     const Engine::Geometries::Coordinate &pnt2,
                     const Engine::Geometries::Coordinate &pnt3);

            Triangle(const Engine::Base::Array<Coordinate> &controlPoints, bool isInv = false /*倒三角*/, bool fromInv = false /*从倒三角转*/);

            // 析构函数
            ~Triangle();

            // 复制构造函数
            Triangle(const Triangle &rhs);

            // 赋值操作符
            Triangle &operator=(const Triangle &rhs);

            // 返回GeometryType::TRIANGLE
            GeometryType GetGeometryType() const;

            // 返回Triangle对象的外包框
            Envelope *const GetEnvelope() const;

            // 返回Triangle对象的4个控制点。
            // 前两个控制点是等边三角形的前两个点；后两个控制点，与等边三角形的第三个顶点在等边三角形底的同一侧，
            // 并且到等边三角形底的距离等于等边三角形的高
            Base::Array<Coordinate> GetControlPoints();

            // 返回Triangle对象的深拷贝对象
            // 注意：需要由调用者释放所返回的指针
            Geometry *const Clone() const;

            // 获得三角形的三个顶点
            Base::Array<Coordinate> GetCoordinates();

            // 获得三角形外接圆的圆心
            Coordinate GetCenter();

            // 三角形的平移
            Base::Void Offset(Base::Double dx, Base::Double dy, Base::Double dz);

            // 三角形的缩放
            Base::Void Resize(Base::UInt16 index, const Engine::Geometries::Coordinate &pt);

            // 三角形的旋转
            Base::Void Rotate(Base::Double angle);

            Base::Void Rotate(Base::UInt16 index, Base::Double angle);

            // 获得长，即三角形的边长
            Base::Double GetLength();

            // 设置长，即三角形的边长
            Base::Void SetLength(Base::Double length);

            // 获得三角形平面的法向量
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
            // 三角形的三个顶点
            Coordinate m_coordinate1;
            Coordinate m_coordinate2;
            Coordinate m_coordinate3;

            // 三角形外接圆的圆心
            Coordinate m_center;

            // 三角形平面的法向量
            Coordinate m_normalVector;

            // 计算三角形外接圆的圆心
            Base::Void ComputeCenter();

            // 计算三角形平面的法向量
            Base::Void ComputeNormalVector();
        };
    }
}

#endif // ENGINE_GEOMETRIES_TRIANGLE_H_
