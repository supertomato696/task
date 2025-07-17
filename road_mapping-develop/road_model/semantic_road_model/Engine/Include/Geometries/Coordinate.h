/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Coordinate.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_COORDINATE_H_
#define ENGINE_GEOMETRIES_COORDINATE_H_

#include "Export.h"
#include "Base/Types.h"
#include "Base/String.h"

#include <string>
using namespace std;

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API Coordinate
        {
        public:
            // 默认构造函数,x,y,z都等于0
            Coordinate();

            // 使用给定的x,y,z构造Coordinate对象
            Coordinate(Base::Double x, Base::Double y, Base::Double z);

            // 析构函数
            ~Coordinate();

            // 复制构造函数,深拷贝
            Coordinate(const Coordinate &rhs);

            // 赋值操作符,深拷贝
            Coordinate &operator=(const Coordinate &rhs);

            // 比较两个坐标是否相等,同时比较x,y,z
            Base::Bool Equals(const Coordinate &other) const;

            // 返回x y z格式字符串
            Base::String ToString() const;

            // 返回x y格式字符串
            Base::String ToString2D() const;

            // 计算Coordinate对象与给定的Coordinate对象之间的三维距离
            Base::Double Distance(const Coordinate &p) const;

            // 计算Coordinate对象与给定的Coordinate对象之间的三维距离
            Base::Double DistanceSquare(const Coordinate &p) const;

            /*!
             *\brief 计算Coordinate对象与给定的Coordinate对象之间的二维xy距离
             *\ param const Coordinate & p
             *\ Returns:   Base::Double
             */
            Base::Double DistanceXY(const Coordinate &p) const;

            // 重载运算符-
            Coordinate operator-(const Coordinate &rhs) const;

            // 重载运算符+
            Coordinate operator+(const Coordinate &rhs) const;

            // 重载运算符*,将坐标看做向量运算
            Base::Double operator*(const Coordinate &rhs) const;

            // 重载运算符*,x,y,z均放大N倍
            Coordinate operator*(Base::Double N) const;

            // 重载运算符/,x,y,z均缩小N倍
            Coordinate operator/(Base::Double N) const;

            Coordinate &operator+=(const Coordinate &rhs);

            Coordinate &operator*=(Base::Double N);

            // 将坐标看做向量,求向量长度
            Base::Double GetLength() const;

            // 将坐标看做向量进行单位化
            Base::Void Normalize();

            // 矢量叉积
            Coordinate CrossProduct(const Coordinate &other) const;

            // 点积
            Base::Double DotProduct(const Coordinate &pnt) const;

            // 是否是0向量
            Base::Bool IsZeroVector() const;

            // 与其他向量的夹角 弧度(规定：0向量与 任何向量夹角为0)
            Base::Double AngleWith(const Coordinate &v) const;
            // 具有误差容忍度Geometries_NEP的相等比较接口--wufz 20170901
            // output	true: 相等，false:不相等
            bool IsEqual(const Coordinate &t, Base::Double tolerance = 1.0E-5) const;

            bool operator==(Coordinate const &t) const { return t.x == this->x && t.y == this->y && t.z == this->z; }
            bool operator!=(Coordinate const &t) const { return t.x != this->x || t.y != this->y || t.z != this->z; }

            /*!
             *\brief 单目运算符 负号
             *\ Returns:   Navinfo::Engine::Geometries::Coordinate
             */
            Coordinate operator-() const { return Coordinate(-x, -y, -z); }

            /*
            格式化输出 add llj 2018-10-24
            */

            void FormatOne(string &_lineTxt, const char *_sStart = NULL) const;
            void FormatTwo(string &_lineTxt, const Coordinate &_other) const;

            ////////
            bool Assing(const char *_sGeo); // 获取形状
            void Assing(const int _nIndex, const Base::Double _fNum);

        public:
            Base::Double x;
            Base::Double y;
            Base::Double z;
        };

        typedef Coordinate Vector3d;
    }
}
#endif // ENGINE_GEOMETRIES_COORDINATE_H_