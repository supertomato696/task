/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Polygon.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_POLYGON_H_
#define ENGINE_GEOMETRIES_POLYGON_H_

#include "Geometry.h"
#include "Envelope3D.h"
#include "LinearRing.h"

namespace Engine
{
    namespace Geometries
    {
        // class LinearRing;
        class Geometries_API Polygon : public Geometry
        {
        public:
            Polygon();
            // 使用给定的外壳构造内部无洞的多边形
            Polygon(LinearRing *shell);

            // 使用给定的外壳和洞构造多边形
            Polygon(LinearRing *shell, Base::Array<LinearRing *> *holes);

            // 析构函数,负责释放内存
            ~Polygon();

            // 复制构造函数,深拷贝
            Polygon(const Polygon &rhs);

            // 赋值操作符,深拷贝
            Polygon &operator=(const Polygon &rhs);

            // 返回GeometryType::POLYGON
            virtual GeometryType GetGeometryType() const;

            // 返回Polygon对象的最小二维外包框
            virtual Envelope *const GetEnvelope() const;

            // 返回Polygon对象的最小三维外包框，by lizhao 2017.3.22
            virtual Envelope3D *const GetEnvelope3D() const;

            // 返回Polygon对象的深拷贝对象
            // 注意:需要由调用者释放所返回的指针
            virtual Geometry *const Clone() const;

            // 返回Polygon对象的外壳
            LinearRing *const GetExteriorRing() const;

            // 返回Polygon对象内部洞的数目
            Base::SizeT GetNumInteriorRing() const;

            // 返回Polygon对象内部下标为index的洞
            // 注意:下标越界,会引发异常
            LinearRing *const GetInteriorRingN(Base::SizeT index) const;

            // 返回Polygon对象的周长(包括外壳和内部的洞)
            Base::Double GetLength() const;

            // 矩形的放缩，矩形四个角点和四个边的中点，按逆时针方向索引号分别是0到7。
            // 拉取角点，则除对角点以外的点的坐标改变；
            // 拉取边的中点，则对边的中点和对边的两个角点坐标不变，其它的点的坐标改变
            // 输入：index，点的索引号；pt放缩后鼠标UP的点的坐标
            // 返回：m_shell被改变，即矩形四个角点和四个边的中点更新后的矩形Polygon
            Base::Bool ZoomInOut(Base::UInt16 index, const Engine::Geometries::Coordinate &pt);

            // 求多边形的面积
            // 面积是不考虑z值，即多边形在 x-y平面投影的面积
            Base::Double GetArea() const;

            // 去重,Write By xueyufei
            Base::Bool Distinct();

            ///////

            bool Assing(const char *_sGeo);

        protected:
            void AddLinearRing(const string &_sGeo);

        protected:
            // 多边形外壳
            LinearRing *m_shell;

            // 多边形内部的洞
            Base::Array<LinearRing *> *m_holes;

            // 如果linearRings中至少有一个LinearRing不为空,返回true;否则返回false
            Base::Bool HasNoEmptyElement(Base::Array<LinearRing *> *linearRings) const;
        };
    }
}

#endif // ENGINE_GEOMETRIES_POLYGON_H_