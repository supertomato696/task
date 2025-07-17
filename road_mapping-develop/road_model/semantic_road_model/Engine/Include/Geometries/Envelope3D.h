/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Envelope3D.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_ENVELOPE3D_H_
#define ENGINE_GEOMETRIES_ENVELOPE3D_H_

#include "Envelope.h"
#include "Base/Types.h"
#include "Base/Math.h"
namespace Engine
{
    namespace Geometries
    {
        class Geometries_API Envelope3D : public Envelope
        {
        public:
            // 默认构造函数
            Envelope3D();
            Envelope3D(Base::Double x1, Base::Double y1, Base::Double x2, Base::Double y2, Base::Double z1, Base::Double z2);
            // 虚析构函数
            virtual ~Envelope3D();

            // 复制构造函数
            Envelope3D(const Envelope3D &rhs);

            // 赋值操作符
            Envelope3D &operator=(const Envelope3D &rhs);

            virtual Envelope &operator=(const Envelope &rhs) override
            {
                // throw std::logic_error("The method or operation is not implemented.");
            }

            Base::Bool Equals(const Envelope3D *other) const;

            // 使用给定的deltaX,deltaY,deltaZ分别在x\y\z方向扩展Envelope3D对象
            Base::Void ExpandBy(Base::Double deltaX, Base::Double deltaY, Base::Double deltaZ);

            // 使用给定的distance在x,y,z方向扩展Envelope对象
            Base::Void ExpandBy(Base::Double distance);

            // 扩展Envelope3D对象,使其包含给定的Envelope3D对象
            Base::Void ExpandToInclude(const Envelope3D *other);

            // 如果Envelope3D对象包含(包括在边框上)Coordinate对象,返回true;否则返回false
            Base::Bool Intersects(const Coordinate &p) const;

            // 如果Envelope3D对象包含(包括在边框上)给定的x,y,z坐标,返回true;否则返回false
            Base::Bool Intersects(Base::Double x, Base::Double y, Base::Double z) const;

            // 如果Envelope3D对象与给定的Envelope3D对象相交，返回true，否则返回false
            Base::Bool Intersects(const Envelope3D *other) const;

            // 如果Envelope3D对象与给定的Envelope3D对象相交，返回true，否则返回false
            Base::Bool Intersects(const Envelope3D &other) const;

            // 判断是否与v1,v2构成的线段相交,write by xueyufei
            Base::Bool Intersects(const Coordinate &v1, const Coordinate &v2) const;

            // 计算Envelope3D对象与给定的Envelope3D对象的交集
            Base::Bool Intersection(const Envelope3D *other, Envelope3D &Intersec) const;

            // 计算Envelope3D对象与给定的Envelope3D对象的交集
            Base::Bool Intersection(const Envelope3D &other, Envelope3D &Intersec) const;

            // 如果Envelope3D对象包含给定的Envelope3D对象，返回true，否则返回false
            Base::Bool Contains(const Envelope3D *other) const;

            // 如果Envelope3D对象包含给定的Envelope3D对象，返回true，否则返回false
            Base::Bool Contains(const Envelope3D &other) const;

            // 如果Envelope3D对象被给定的Envelope3D对象包含，返回true，否则返回false
            Base::Bool Contained(const Envelope3D *other) const;

            // 如果Envelope3D对象被给定的Envelope3D对象包含，返回true，否则返回false
            Base::Bool Contained(const Envelope3D &other) const;

            // 判断点是否在长方体中
            Base::Bool PtInEnvelope3D(const Coordinate pt) const;

            virtual Base::Bool IsNull() const override;

            virtual Base::Void SetToNull() override;

            virtual Base::Void Normalize() override;

            // 设置z值最大值
            Base::Void SetMaxZ(Base::Double z);

            // 设置z值最小值
            Base::Void SetMinZ(Base::Double z);
            // 返回Envelope3D对象宽度,MaxZ-MinZ
            Base::Double GetDepth() const;

            // 返回Envelope对象的体积,Height*Width*Depth
            Base::Double GetVolum() const;

            // 返回Envelope对象的最大z
            Base::Double GetMaxZ() const;

            // 返回Envelope对象的最小z
            Base::Double GetMinZ() const;
            Base::Void Reset(Base::Double x1, Base::Double y1, Base::Double x2, Base::Double y2, Base::Double z1, Base::Double z2);

        protected:
            Base::Void Init(Base::Double x1, Base::Double x2, Base::Double y1, Base::Double y2, Base::Double z1, Base::Double z2);

        protected:
            // z坐标最小值
            Base::Double m_minz;

            // z坐标最大值
            Base::Double m_maxz;
        };
    }
}

#endif // ENGINE_GEOMETRIES_ENVELOPE3D_H_