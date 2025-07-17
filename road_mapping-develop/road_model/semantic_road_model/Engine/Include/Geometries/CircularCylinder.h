/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:CircularCylinder.h
简要描述:斜圆柱体
******************************************************************/

#ifndef ENGINE_GEOMETRIES_OBLIQUECIRCULARCYLINDER_H_
#define ENGINE_GEOMETRIES_OBLIQUECIRCULARCYLINDER_H_

#include "Geometry.h"

namespace Engine
{
    namespace Geometries
    {
        class Polygon;

        class Geometries_API CircularCylinder : public Geometry
        {
        public:
            CircularCylinder();

            // 构造函数，斜圆柱体的底面和顶面平行于xoy平面
            // 输入：pnt1、pnt2、pnt3，斜圆柱体的底面的三点；obliqueVector，斜圆柱体底面到顶面的向量；
            // height，斜圆柱体底面到顶面的高度
            CircularCylinder(const Engine::Geometries::Coordinate &pnt1,
                             const Engine::Geometries::Coordinate &pnt2,
                             const Engine::Geometries::Coordinate &pnt3,
                             const Engine::Geometries::Coordinate obliqueVector,
                             const Base::Double height);

            // 构造函数，斜圆柱体的底面和顶面平行于xoy平面
            // 输入：top_center、bottom_center，斜圆柱体的上下面中心；
            // topDiameter,上顶面直径；
            // bottomDiameter,上顶面直径；
            CircularCylinder(const Engine::Geometries::Coordinate &top_center,
                             const Engine::Geometries::Coordinate &bottom_center,
                             const double &topDiameter,
                             const double &bottomDiameter);

            Base::Bool Make(const Engine::Geometries::Coordinate &pnt1,
                            const Engine::Geometries::Coordinate &pnt2,
                            const Engine::Geometries::Coordinate &pnt3,
                            const Engine::Geometries::Coordinate obliqueVector,
                            const Base::Double height);

            // 析构函数
            ~CircularCylinder();

            // 复制构造函数
            CircularCylinder(const CircularCylinder &rhs);

            // 赋值操作符
            CircularCylinder &operator=(const CircularCylinder &rhs);

            // 返回GeometryType::OBLIQUECIRCULARCYLINDER
            GeometryType GetGeometryType() const;

            // 返回CircularCylinder对象的外包框
            Envelope *const GetEnvelope() const;

            // 返回CircularCylinder对象底面的4个控制点
            Base::Array<Coordinate> GetControlPoints();

            // 返回CircularCylinder对象的深拷贝对象
            // 注意：需要由调用者释放所返回的指针
            Geometry *const Clone() const;

            // 获得斜圆柱体底面的3个点，索引0到2是底面的3个点
            Base::Array<Coordinate> GetBottomsurfaceCoords();

            // 获得斜圆柱体底面到顶面的向量
            Coordinate GetObliqueVector();

            // 获得斜圆柱体底面到顶面的高度
            Base::Double GetHeight();

            // 获得斜圆柱体顶面的3个点，索引0到2是顶面的3个点
            Base::Array<Coordinate> GetTopsurfaceCoords();

            // 获得斜圆柱体底面的圆心
            Coordinate GetBottomsurfaceCenter();

            // 获得斜圆柱体顶面的圆心
            Coordinate GetTopsurfaceCenter();

            // 获得顶面的半径
            Base::Double GetTopRadius();

            // 获取底面半径
            Base::Double GetBottomRadius();

            // 斜圆柱体平移
            Base::Void Offset(Base::Double dx, Base::Double dy, Base::Double dz);

            // 斜圆柱体缩放
            Base::Void ResizeTop(Base::Double radius);

            // 斜圆柱体缩放
            Base::Void ResizeBottom(Base::Double radius);

            // 在设置顶面圆心的位置后，更新斜圆柱体底面到顶面的向量和斜圆柱体底面到顶面的向量的长度
            Base::Void SetTopsurfaceCenter(const Coordinate &pnt);

            // 在设置顶面圆心的位置后，更新斜圆柱体底面到顶面的向量和斜圆柱体底面到顶面的向量的长度
            Base::Void SetBottomsurfaceCenter(const Coordinate &pnt);

            // 根据分段数将斜圆柱体的底面圆分割，生成新的多边形
            Polygon *BottomsurfaceToPolygon(Base::UInt16 subsectionnum);

            // 根据分段数将斜圆柱体的顶面圆分割，生成新的多边形
            Polygon *TopsurfaceToPolygon(Base::UInt16 subsectionnum);

            // 获得斜圆柱体底面和顶面圆平面的法向量
            Coordinate GetNormalVector();

        private:
            // 斜圆柱体底面和顶面的6个点，索引0到2是底面的3个点，索引3到5是顶面的3个点
            Base::Array<Coordinate> m_coordinates;

            // 斜圆柱体底面和顶面的圆心
            Coordinate m_center1;
            Coordinate m_center2;

            // 斜圆柱体底面和顶面圆平面的法向量
            Coordinate m_normalVector;

            // 斜圆柱体底面和顶面的半径
            Base::Double m_radius1;
            Base::Double m_radius2;

            // 斜圆柱体底面到顶面的向量
            Coordinate m_obliqueVector;

            // 斜圆柱体底面到顶面的高度
            Base::Double m_height;

            // 斜圆柱体底面到顶面的向量的长度
            Base::Double m_obliqueVectorLength;

            // 计算斜圆柱体底面和顶面的圆心
            Base::Void ComputeCenters();

            // 计算斜圆柱顶面的圆心
            Base::Void ComputeTopCenters();

            // 计算斜圆柱体底面的圆心
            Base::Void ComputeBottomCenters();

            // 计算斜圆柱体底面和顶面圆平面的法向量
            Base::Void ComputeNormalVector();
        };
    }
}

#endif // ENGINE_GEOMETRIES_OBLIQUECIRCULARCYLINDER_H_
