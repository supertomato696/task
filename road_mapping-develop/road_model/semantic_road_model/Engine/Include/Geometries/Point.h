/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Point.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRY_POINT_H_
#define ENGINE_GEOMETRY_POINT_H_

#include "Geometry.h"
#include "Base/Array.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API Point : public Geometry
        {
        public:
            // 默认构造函数,x,y,z都等于0
            Point();

            // 使用给定的x,y,z构造Point对象
            Point(Base::Double x, Base::Double y, Base::Double z);

            // 使用给定的Coordinate对象构造Point对象
            Point(Coordinate *coord);

            // write by xueyufei
            Point(Coordinate coord);

            // 析构函数
            ~Point();

            // 复制构造函数,深拷贝
            Point(const Point &rhs);

            // 赋值操作符,深拷贝
            Point &operator=(const Point &rhs);

            // 返回GeometryType::POINT
            GeometryType GetGeometryType() const;

            // 返回Point对象的二维外包框,只包含一个点
            Envelope *const GetEnvelope() const;

            // 返回Point对象的深拷贝对象
            // 注意:需要由调用者释放所返回的指针
            Geometry *const Clone() const;

            // 返回Point对象的X坐标
            Base::Double GetX() const;

            // 返回Point对象的Y坐标
            Base::Double GetY() const;

            // 返回Point对象的Z坐标
            Base::Double GetZ() const;

            // 设置Point对象的X坐标
            Base::Void SetX(Base::Double d);

            // 设置Point对象的Y坐标
            Base::Void SetY(Base::Double d);

            // 设置Point对象的Z坐标
            Base::Void SetZ(Base::Double d);

            // 输出为坐标形式
            Coordinate ToCoordinate() const;

        private:
            Coordinate *m_coordinate;
        };
    }
}

#endif // ENGINE_GEOMETRY_POINT_H_