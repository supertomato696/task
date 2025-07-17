/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Octagon.h
简要描述:八角形
******************************************************************/

#ifndef ENGINE_GEOMETRIES_OCTAGON_H_
#define ENGINE_GEOMETRIES_OCTAGON_H_

#include "Geometry.h"

namespace Engine
{
    namespace Geometries
    {
        class Polygon;

        class Geometries_API Octagon : public Geometry
        {
        public:
            // 默认构造函数
            Octagon();

            // 析构函数
            virtual ~Octagon();

            // 构造函数
            // 输入：pntSegFrom，A；pntSegTo：B；pnt，C
            // AB确定八角形边长,C与AB确定方向
            Octagon(const Engine::Geometries::Coordinate &pntSegFrom,
                    const Engine::Geometries::Coordinate &pntSegTo,
                    const Engine::Geometries::Coordinate &pnt);

            Octagon(const Engine::Base::Array<Engine::Geometries::Coordinate> &controlPoints);
            // 返回八角形对象的8个控制点，4个角点的索引分别为0到3，4个边的中点的索引分别为4到7
            Base::Array<Coordinate> GetControlPoints();

            // 获得八角形的八个顶点
            Base::Array<Coordinate> GetCoordinates();

            // 返回GeometryType::OCTAGON
            GeometryType GetGeometryType() const;

            // 平移
            Base::Void Offset(Base::Double dx, Base::Double dy, Base::Double dz);

            // 缩放 length 外切圆半径
            Base::Void Resize(Base::Double length);

            Base::Bool Resize(Base::UInt16 index, const Engine::Geometries::Coordinate &pnt);

            // 旋转
            Base::Void Rotate(Base::UInt16 index, Base::Double angle);

            Base::Void RotateByCoord(Base::UInt16 index, const Engine::Geometries::Coordinate &pt);

            // 从Polygon构造
            Octagon *FromPolygon(Geometries::Geometry *pGeometry);

            // 导出到Polygon
            Polygon *ToPolygon();

            // 返回Octagon对象的外包框
            Envelope *const GetEnvelope() const;

            // 返回Octagon对象的深拷贝对象
            // 注意：需要由调用者释放所返回的指针
            Geometry *const Clone() const;

            // 设置边长
            Base::Void SetLength(Base::Double length);

        protected:
            void ComputerCenter(const Engine::Geometries::Coordinate &pntSegFrom,
                                const Engine::Geometries::Coordinate &pntSegTo,
                                const Engine::Geometries::Coordinate &pnt);

            void ComputerNormal(const Engine::Geometries::Coordinate &pntSegFrom,
                                const Engine::Geometries::Coordinate &pntSegTo,
                                const Engine::Geometries::Coordinate &pnt);

        private:
            // 八个点
            Base::Array<Coordinate> m_arrCoordinates;

            // 外接圆中心
            Coordinate m_coordCenter;

            // 法向量
            Coordinate m_vectorNormal;
        };
    }
}

#endif // ENGINE_GEOMETRIES_OCTAGON_H_
