/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:LineString.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_LINESTRING_H_
#define ENGINE_GEOMETRIES_LINESTRING_H_

#include "Point.h"
#include "Envelope3D.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API LineString : public Geometry
        {
        public:
            // 默认构造函数,不包含任何坐标点
            LineString();

            // 使用给定的坐标数组构造LineString对象
            LineString(Base::Array<Coordinate *> *coordinates);

            // 构造两点连接的LineString对象
            LineString(Coordinate *coord1, Coordinate *coord2);

            // 析构函数,负责释放内存
            ~LineString();

            // 复制构造函数,深拷贝
            LineString(const LineString &rhs);

            // 赋值操作符,深拷贝
            LineString &operator=(const LineString &rhs);

            // 返回GeometryType::LINESTRING
            virtual GeometryType GetGeometryType() const;

            // 返回LineString对象的最小二维外包框
            virtual Envelope *const GetEnvelope() const;

            // 返回LineString对象的最小三维外包框, write by xueyufei
            virtual Envelope3D *const GetEnvelope3D() const;

            // 如果LineString对象不包含任何坐标点,返回true;否则,返回false
            virtual Base::Bool IsEmpty() const;

            // 返回LineString对象包含的坐标点数目
            virtual Base::SizeT GetNumPoints() const;

            // 返回LineString对象包含的坐标点数组指针
            virtual Base::Array<Coordinate *> *const GetCoordinates() const;

            // 返回LineString对象的深拷贝对象
            // 注意:需要由调用者释放所返回的指针
            virtual Geometry *const Clone() const;

            // 返回LineString对象的下标为index的坐标点
            // 注意:如果下标越界,会引发异常
            virtual Coordinate *const GetCoordinateN(Base::SizeT index) const;

            // 返回LineString对象的下标为index的Point指针
            // 注意:需要调用者释放内存,如果下标越界,会引发异常
            virtual Point *const GetPointN(Base::SizeT index) const;

            // 返回LineString对象的起点Point指针
            // 注意:需要调用者释放内存,如果下标越界,会引发异常
            virtual Point *const GetStartPoint() const;

            // 返回LineString对象的终点Point指针
            // 注意:需要调用者释放内存,如果下标越界,会引发异常
            virtual Point *const GetEndPoint() const;

            // 如果起点和重点坐标相同,返回true;否则,返回false;如果不包含任何坐标点,返回false;
            virtual Base::Bool IsClosed() const;

            // 返回LineString对象的三维长度
            virtual Base::Double GetLength() const;

            // 返回LineString对象的三维长度 [2/27/2018 wufuzheng]
            virtual Base::Double GetPartLength(Base::Int32 from, Base::Int32 to) const;

            // 返回LineString对象的XY长度
            virtual Base::Double GetLengthXY() const;

            // 返回LineString对象的坐标顺序相反的LineString对象
            // 注意:需要由调用者释放所返回的指针
            Geometry *const Reverse() const;

            /*******************************************************************?
             * @brief:裁剪线（从线的第一个点开始截取dLength米长度）.
             * @author:DouHang
             * @date:2018/05/10
             * @param[in]:dLength：裁剪长度（单位米）
             * @param[out]:
             * @return:裁剪后的线。
             * @other:如果线长度小于dLength，返回null
             ******************************************************/
            Geometry *CutByLength(double dLength);

            // 线的离散化，在LineString的每一段的首末点之间每隔distance插入一个点
            Base::Bool Discretization(Base::Double distance);
            Base::Bool Discretization3D(Base::Double distance);

            // 线的离散化，从LineString的起点开始每隔distance距离取一个点 add by douhang 19/6/2017
            Base::Bool Discretization3D2(Base::Double distance);

            // 线的重采样，距离小于阈值的剔除
            bool Resampling(double tolerance);

            // 橡皮筋功能
            bool Flexibility(Base::SizeT nIndex, const Coordinate *pNewPos);
            bool Flexibility3D(Base::SizeT nIndex, const Coordinate *pNewPos);

            Base::Bool GetMAtPoint(const Coordinate *pntProject, Base::Int32 nSegIndex, Base::Double &dMeasure);

            Base::Bool GetPointAtM(const Base::Double dMeasure, Coordinate &pntProject, Base::Int32 &nSegIndex);

            // 橡皮筋功能,带衰减旋转算法
            bool DeclineRotate3D(bool bIsStart, const Coordinate *pNewPos, unsigned int iDecCoeff = 0);

            // 获取线上两点间距离 [4/12/2018 guohaiqiang]
            Base::Double GetDistanceBetweenTowPoint(Base::SizeT nIndexS, Base::SizeT nIndexE);
            Base::Double GetDistanceBetweenTowPoint3D(Base::SizeT nIndexS, Base::SizeT nIndexE);

            // 去重,Write By xueyufei
            virtual Base::Bool Distinct();

            // 添加只考虑XY坐标的去重接口  [8/9/2017 guohaiqiang]
            virtual Base::Bool DistinctXY();

            // 去折角,如果形状点到前一个后一个点组成的线段的距离大于tolerance认为有折角,采用二分法,Write By xueyufei
            virtual Base::Bool Smooth(Base::Double tolerance = 0.1);

            // 判断与rhs是否在同一方向,write by xueyufei
            // 返回：true 代表同一方向，false代表垂直或者反向；
            Base::Bool IsSameDirection(const LineString *rhs) const;

            // 判断与rhs是否在同一方向,包括盘桥，获取最近两个端最后一段线段判断方向,write by qiuyanwei
            // 返回：true 代表同一方向，false代表垂直或者反向；
            Base::Bool IsSameDirection2(const LineString *rhs) const;

            // 新接口,理论上应该能代替上两种方法,add by xueyufei 2018.4.28
            // 目前用在了设置墙的中与轨迹线判断方向上
            Base::Bool IsSameDirectionEx(const LineString *rhs) const;

            // 线的各节点的x、y、z分别平移某固定距离
            // 输入：dx、dy、dz，x、y、z分别平移的距离数
            // 返回：原线各节点的x、y、z分别平移某固定距离后的新线
            virtual Base::Void Translation(Base::Double dx, Base::Double dy, Base::Double dz);

            Envelope GetBound() const;

            bool Assing(const char *_sGeo); // 获取形状

        protected:
            Base::Array<Coordinate *> *m_coordinates;
        };
    }
}

#endif // ENGINE_GEOMETRIES_LINESTRING_H_