/******************************************************************
Copyright(c) 2015-2020 Navinfo
All rights reserved.

作者:dongjian
日期:2016-06-20 15:24:14
文件名称:KPICaculatorByLeastSquareCicle.h
简要描述:
******************************************************************/
#ifndef ENGINE_GEOMETRIES_KPICACULATORBYLEASTSQUARECICLE_H_ 
#define ENGINE_GEOMETRIES_KPICACULATORBYLEASTSQUARECICLE_H_  
#include "Geometries/Export.h"
#include "Base/Types.h"
#include "Base/Array.h"
#include "Geometries/Coordinate.h"
#include "Geometries/SCHAlgorithm/NodeInfo.h"
#include <vector>

namespace Engine
{
    namespace Geometries
    {
        /*
        * Position 点生成 工具
        */
        class Geometries_API PositionlizeUtility
        {
        public:
            /*!
             *\brief position 点生成
            *\ param const Base::Array<Coordinate * > * vecCoorArray Coordinate 点序列
            *\ param const Base::Double dInterval position 点间隔
            *\ param Base::Array<NodeInfo> & vecPositionNodes position点结果
            *\ Returns:   Base::Void
            */
            static Base::Void GeneratePosition(const Base::Array<Coordinate*>* vecCoorArray, const Base::Double dInterval, std::vector<NodeInfo>& vecPositionNodes);
            //	static Base::Void Complete(Base::Array<NodeInfo>& vecPositionNodes);
        };

        class LineString;

        /*
         *	以最小二乘法方式拟合圆，求曲率
         */
        class Geometries_API CurvatureByLeastSquareCicle
        {
        public:
            CurvatureByLeastSquareCicle();

        private:
            /**带求linestring 数据 */
            const LineString* m_pLineString;

        public:
            /*!
             *\brief  设置数据，仅在 调用 CurvatureAtVertex 时调用
            *\ param const LineString * pLineString
            *\ Returns:   Base::Void
            */
            Base::Void SetData(const LineString* pLineString);
            /*!
             *\brief 求形状点处曲率值
            *\ param std::vector<Base::Double> & vecCurvatures   vecCurvatures[i]为i th节点处曲率值
            *\ Returns:   Base::Bool
            */
            Base::Bool SCHAtVertex(std::vector<Base::Double>& vecCurvatures);
            /*!
             *\brief position点处曲率值 ，可使用 PositionlizeUtility 生成position
            *\ param std::vector<NodeInfo> & vecNodeInfo
            *\ Returns:   Base::Bool
            */
            Base::Bool SCHAtPosition(std::vector<NodeInfo>& vecNodeInfo);
        };
    }
}

#endif //ENGINE_GEOMETRIES_KPICACULATORBYLEASTSQUARECICLE_H_