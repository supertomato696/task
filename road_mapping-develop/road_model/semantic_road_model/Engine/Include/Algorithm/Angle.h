/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称: Angle.h
简要描述:
******************************************************************/
#ifndef _1556BB0A_3EB8_4676_8C3C_EC053C3E9728
#define _1556BB0A_3EB8_4676_8C3C_EC053C3E9728
#include "Algorithm/Algorithm.h"
#include "Base/Types.h"
#include "Base/Array.h"
#include "Base/String.h"
#include "Geometries/Coordinate.h"

namespace Engine
{
    namespace Algorithm
    {
        class Algorithm_API Angle
        {
        public:
            static Base::Double Degree2Radian(const Base::Double &degree);
            static Base::Double Radian2Degree(const Base::Double &radian);
            static Geometries::Vector3d Azimuth2Vector(Base::Double angle);
            // const static Base::Double PI;
        };
    }
}
#endif //_1556BB0A_3EB8_4676_8C3C_EC053C3E9728