/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Rectangle.h
简要描述:矩形
******************************************************************/

#ifndef ENGINE_GEOMETRIES_RECTANGLE_H_
#define ENGINE_GEOMETRIES_RECTANGLE_H_

#include "Geometry.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API Rectangle : public Geometry
        {
        public:
            Rectangle();

            // 构造函数
            // 输入：pntSegFrom，A；pntSegTo：B；pnt，C
            // 返回：矩形，满足：线段AB为一边，点C到AB的距离为另一边的长度，矩形的另两个顶点与点C在直线AB的同一侧
            Rectangle(const Engine::Geometries::Coordinate &pntSegFrom,
                      const Engine::Geometries::Coordinate &pntSegTo,
                      const Engine::Geometries::Coordinate &pnt);

            Rectangle(const Engine::Base::Array<Coordinate> &controlPoints);

            Base::Bool Make(const Engine::Geometries::Coordinate &pntSegFrom,
                            const Engine::Geometries::Coordinate &pntSegTo,
                            Base::Double width, Base::Double lendth);

            // 析构函数
            ~Rectangle();

            // 				//复制构造函数
            // 				Rectangle(const Rectangle &rhs);
            //
            // 				//赋值操作符
            // 				Rectangle& operator =(const Rectangle &rhs);

            // 返回GeometryType::RECTANGLE
            GeometryType GetGeometryType() const;

            // 返回Rectangle对象的外包框
            Envelope *const GetEnvelope() const;

            // 返回Rectangle对象的8个控制点，4个角点的索引分别为0到3，4个边的中点的索引分别为4到7
            virtual Base::Array<Coordinate> GetControlPoints();

            // 返回Rectangle对象的深拷贝对象
            // 注意：需要由调用者释放所返回的指针
            Geometry *const Clone() const;

            // 获得矩形的四个角点
            Base::Array<Coordinate> GetCoordinates();

            // 获得矩形的中心
            Coordinate GetCenter();

            // 矩形的平移
            Base::Void Offset(Base::Double dx, Base::Double dy, Base::Double dz);

            // 拉动矩形的角点或边的中点，做矩形的缩放。四个角点的索引依次为0到3，四个边的中点的索引依次为4到7。
            // 拉动矩形的角点，除了对角线的角点外，其它角点的位置更新；拉动矩形的边的中点，除了对边的两端的角点外，其它角点的位置更新
            // 输入：index，点的索引号；pt，点的新的位置
            Base::Bool Resize(Base::UInt16 index, const Engine::Geometries::Coordinate &pt);

            // 矩形的旋转
            Base::Void Rotate(Base::Double angle);

            virtual Base::Void Rotate(Base::UInt16 index, Base::Double angle);

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

            // 获得长，即矩形的第一个点和第二个点的距离
            Base::Double GetLength();

            // 设置长，即矩形的第一个点和第二个点的距离
            Base::Void SetLength(Base::Double length);

            // 获得宽，即矩形的第二个点和第三个点的距离
            Base::Double GetWidth();

            // 设置宽，即矩形的第二个点和第三个点的距离
            Base::Void SetWidth(Base::Double width);

            // 获得矩形平面的法向量
            virtual Coordinate GetNormalVector();

        private:
            // 矩形的四个角点
            Base::Array<Coordinate> m_coordinates;

            // 矩形的中心
            Coordinate m_center;

            // 矩形平面的法向量
            Coordinate m_normalVector;

            // 计算矩形的中心
            Base::Void ComputeCenter();

            // 计算矩形平面的法向量
            Base::Void ComputeNormalVector();
        };
    }
}

#endif // ENGINE_GEOMETRIES_RECTANGLE_H_
