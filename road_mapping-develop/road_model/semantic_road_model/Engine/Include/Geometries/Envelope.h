/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Envelope.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_ENVELOPE_H_
#define ENGINE_GEOMETRIES_ENVELOPE_H_

#include "Export.h"
#include "Base/Types.h"
#include "Coordinate.h"

namespace Engine
{
    namespace Geometries
    {
        class LineString;

        class Geometries_API Envelope
        {
        public:
            // 默认构造函数,使Envelope对象的maxx<minx且maxy<miny;
            Envelope();

            // 使用给定的两个点(x1,y1),(x2,y2)构造Envelope对象
            Envelope(Base::Double x1, Base::Double y1, Base::Double x2, Base::Double y2);

            // 使用给定的两个Coordinate对象p1,p2构造Envelope对象
            Envelope(const Coordinate &p1, const Coordinate &p2);

            // 析构函数
            virtual ~Envelope();

            // 复制构造函数,深拷贝
            Envelope(const Envelope &rhs);

            // 赋值操作符,深拷贝
            virtual Envelope &operator=(const Envelope &rhs);

            // 返回Envelope对象宽度,MaxX-MinX
            Base::Double GetWidth() const;

            // 返回Envelope对象宽度,MaxY-MinY
            Base::Double GetHeight() const;

            // 返回Envelope对象的面积,Height*Width
            Base::Double GetArea() const;

            // 返回Envelope对象的最大Y
            Base::Double GetMaxY() const;

            // 返回Envelope对象的最大X
            Base::Double GetMaxX() const;

            // 返回Envelope对象的最小Y
            Base::Double GetMinY() const;

            // 返回Envelope对象的最小X
            Base::Double GetMinX() const;

            // 使用给定的deltaX,deltaY分别在x和y方向扩展Envelope对象
            Base::Void ExpandBy(Base::Double deltaX, Base::Double deltaY);

            // 使用给定的distance在x和y方向扩展Envelope对象
            Base::Void ExpandBy(Base::Double distance);

            // 扩展Envelope对象,使其包含给定的Envelope对象
            Base::Void ExpandToInclude(const Envelope *other);

            // 如果Envelope对象与给定的Envelope对象相等(分别比较minx,miny,maxx,maxy),返回true;否则,返回false
            virtual Base::Bool Equals(const Envelope *other) const;

            // 如果Envelope对象包含(包括在边框上)Coordinate对象,返回true;否则返回false
            Base::Bool Intersects(const Coordinate &p) const;

            // 如果Envelope对象包含(包括在边框上)给定的x,y坐标,返回true;否则返回false
            Base::Bool Intersects(Base::Double x, Base::Double y) const;

            // 如果Envolope对象包含或者与LineString相交,返回true,否则返回false,write by xueyufei
            Base::Bool IntersectOrContain(LineString *lineString);

            // 如果Envelope对象与给定的Envelope对象相交，返回true，否则返回false
            Base::Bool Intersects(const Envelope *other) const;

            // 如果Envelope对象与给定的Envelope对象相交，返回true，否则返回false
            Base::Bool Intersects(const Envelope &other) const;

            // 计算Envelope对象与给定的Envelope对象的交集
            Base::Bool Intersection(const Envelope *other, Envelope &Intersec) const;

            // 计算Envelope对象与给定的Envelope对象的交集
            Base::Bool Intersection(const Envelope &other, Envelope &Intersec) const;

            // 如果Envelope对象包含给定的Envelope对象，返回true，否则返回false
            Base::Bool Contains(const Envelope *other) const;

            // 如果Envelope对象包含给定的Envelope对象，返回true，否则返回false
            Base::Bool Contains(const Envelope &other) const;

            // 如果Envelope对象被给定的Envelope对象包含，返回true，否则返回false
            Base::Bool Contained(const Envelope *other) const;

            // 如果Envelope对象被给定的Envelope对象包含，返回true，否则返回false
            Base::Bool Contained(const Envelope &other) const;

            // 判断点是否在矩形中
            Base::Bool PtInRect(Base::Double x, Base::Double y) const;

            // 如果maxx小于minx或者maxy小于miny,返回true;否则,返回false;
            virtual Base::Bool IsNull() const;

            // 使Envelope对象的maxx<minx且maxy<miny;
            virtual Base::Void SetToNull();

            // 设置X值最大值 added by yanchl 16/4/21
            Base::Void SetMaxX(Base::Double x);

            // 设置X值最小值 added by yanchl 16/4/21
            Base::Void SetMinX(Base::Double x);

            // 设置X值最大值 added by yanchl 16/4/21
            Base::Void SetMaxY(Base::Double y);

            // 设置Y值最小值 added by yanchl 16/4/21
            Base::Void SetMinY(Base::Double y);

            // 使用给定的两个点(x1,y1),(x2,y2) 重置外包框 added by yanchl 16/4/21
            Base::Void Reset(Base::Double x1, Base::Double y1, Base::Double x2, Base::Double y2);

            // 合法化四至范围 added by yanchl 16/4/21
            virtual Base::Void Normalize();

            Coordinate BottomLeft() const;

            Coordinate TopRight() const;

            // 合并矩形
            Base::Void Union(const Envelope &other);

        protected:
            // 使用给定的x1,x2,y1,y2初始化Envelope对象,保证minx<maxx且miny<maxy
            Base::Void Init(Base::Double x1, Base::Double x2, Base::Double y1, Base::Double y2);

        protected:
            // x坐标最小值
            Base::Double m_minx;

            // x坐标最大值
            Base::Double m_maxx;

            // y坐标最小值
            Base::Double m_miny;

            // y坐标最大值
            Base::Double m_maxy;
        };
    }
}

#endif // ENGINE_GEOMETRIES_ENVELOPE_H_