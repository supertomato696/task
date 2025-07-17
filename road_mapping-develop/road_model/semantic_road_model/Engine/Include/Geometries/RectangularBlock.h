/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:RectangularBlock.h
简要描述:斜矩形块
******************************************************************/

#ifndef ENGINE_GEOMETRIES_INCLINEDRECTANGULARBLOCK_H_
#define ENGINE_GEOMETRIES_INCLINEDRECTANGULARBLOCK_H_

#include "Geometry.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API RectangularBlock : public Geometry
        {
        public:
            RectangularBlock();

            // 构造函数
            // 输入：pnt1、pnt2、pnt3，斜矩形块的底面的三点；obliqueVector，斜矩形块底面到顶面的向量；
            // height，斜矩形块底面到顶面的高度
            RectangularBlock(const Engine::Geometries::Coordinate &pntSegFrom,
                             const Engine::Geometries::Coordinate &pntSegTo,
                             const Engine::Geometries::Coordinate &pnt,
                             const Coordinate obliqueVector,
                             const Base::Double height);

            Base::Bool Make(const Engine::Geometries::Coordinate &pntSegFrom,
                            const Engine::Geometries::Coordinate &pntSegTo,
                            const Engine::Geometries::Coordinate &pnt,
                            const Coordinate obliqueVector,
                            const Base::Double height);

            // 析构函数
            ~RectangularBlock();

            // 复制构造函数
            RectangularBlock(const RectangularBlock &rhs);

            // 赋值操作符
            RectangularBlock &operator=(const RectangularBlock &rhs);

            // 返回GeometryType::INCLINEDRECTANGULARBLOCK
            GeometryType GetGeometryType() const;

            // 返回RectangularBlock对象的外包框
            Envelope *const GetEnvelope() const;

            // 返回RectangularBlock对象底面的8个控制点，4个角点的索引分别为0到3，4个边的中点的索引分别为4到7
            Base::Array<Coordinate> GetControlPoints();

            // 返回RectangularBlock对象的深拷贝对象
            // 注意：需要由调用者释放所返回的指针
            Geometry *const Clone() const;

            // 获得斜矩形块底面的4个点，索引0到3是底面的4个点
            Base::Array<Coordinate> GetBottomsurfaceCoords();

            // 获得斜矩形块底面到顶面的向量
            Coordinate GetObliqueVector();

            // 获得斜矩形块底面到顶面的高度
            Base::Double GetHeight();

            // 获得斜矩形块顶面的4个点，索引0到3是顶面的4个点
            Base::Array<Coordinate> GetTopsurfaceCoords();

            // 获得斜矩形块底面的中心
            Coordinate GetBottomsurfaceCenter();

            // 获得斜矩形块顶面的中心
            Coordinate GetTopsurfaceCenter();

            // 斜矩形块平移
            Base::Void Offset(Base::Double dx, Base::Double dy, Base::Double dz);

            // 斜矩形块缩放
            // 拉动斜矩形块底面的角点或边的中点，做斜矩形块底面的缩放。四个角点的索引依次为0到3，四个边的中点的索引依次为4到7。
            // 拉动斜矩形块底面的角点，中心点不变，长宽比例不变，斜矩形块底面的四个角点的位置更新；
            // 拉动斜矩形块底面的边的中点，除了对边的两端的角点外，斜矩形块底面的其它角点的位置更新，中心点更新。
            // 斜矩形块顶面的角点的位置相应的更新
            // 输入：index，点的索引号；pt，点的新的位置
            Base::Bool Resize(Base::UInt16 index, const Engine::Geometries::Coordinate &pt);

            // 斜矩形块旋转
            Base::Void Rotate(Base::Double angle);

            // 在设置顶面中心的位置后，更新斜矩形块底面到顶面的向量、斜矩形块底面到顶面的向量的长度和斜矩形块底面到顶面的高度
            Base::Void SetTopsurfaceCenter(const Coordinate &pnt);

            Base::Void SetBottomsurfaceCenter(const Coordinate &pnt);

            // 获得斜矩形块底面和顶面的法向量
            Coordinate GetNormalVector();

        private:
            // 斜矩形块底面和顶面的8个点，索引0到3是底面的4个点，索引4到7是顶面的4个点
            Base::Array<Coordinate> m_coordinates;

            // 斜矩形块底面和顶面的中心
            Coordinate m_center1;
            Coordinate m_center2;

            // 斜矩形块底面到顶面的向量
            Coordinate m_obliqueVector;

            // 斜矩形块底面到顶面的高度
            Base::Double m_height;

            // 斜矩形块底面到顶面的向量的长度
            Base::Double m_obliqueVectorLength;

            // 斜矩形块底面和顶面的法向量
            Coordinate m_normalVector;

            // 计算斜矩形块底面和顶面的中心
            Base::Void ComputeCenters();

            // 计算斜矩形块底面和顶面的法向量
            Base::Void ComputeNormalVector();
        };
    }
}

#endif // ENGINE_GEOMETRIES_INCLINEDRECTANGULARBLOCK_H_
