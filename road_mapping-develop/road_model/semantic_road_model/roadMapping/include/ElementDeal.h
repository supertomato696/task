//
//
//

#ifndef HDMAP_BUILD_ELEMENTDEAL_H
#define HDMAP_BUILD_ELEMENTDEAL_H

#include "Base/Types.h"
#include "Base/Array.h"
#include "Geometries/Coordinate.h"
#include <string>

#include "LineObj.h"
#include "RoadMark.h"
#include "TrafficSign.h"

#include "../data-access-engine/manager/road_geometry_mgr.h"

using namespace std;
using namespace Engine;
using namespace Engine::Base;
using namespace Engine::Geometries;

namespace hdmap_build
{
    class ElementDeal
    {
    public:
        ElementDeal();
        ~ElementDeal();

        // 初始化参数
        void InitParameter(string strTilePath, string down_road_branch, string up_road_branch, int tile_id, string editor, string address);

        // 提取所有的车道线并进行处理
        void ExtractLaneGroups();

        // 将LaneGroups函数组装为对应的proxy结构体
        void CreatLaneGroupProxy();

        // 提取RoadBoundarys内所有线型几何
        void ExtractRoadBoundarys(); // 目前废弃不用

        // 首先提取道路边界
        void ExtractRoadBoundarys2(); // 处理道路边界

        // 将RoadBoundary函数组装为对应的proxy结构体
        void CreatRoadBoundaryProxy();

        // 提取RoadMark的矩形框
        void ExtractRoadMark();

        // 将RoadMarkObj组装为对应的proxy结构体
        void CreatRoadMarkProxy();

        // 提取交通牌
        void ExtractTrafficSign();

        // 将TrafficSignObj组装为proxy结构体
        void CreatTrafficSignProxy();

        // 更新数据与原始数据进行去重，修偏
        void MerageData();

        // 数据的上传
        void UpLoadTiles(data_access_engine::TileInfoList &corrtiles);

    private:
        // 路径，分支等基本信息存储
        int m_TileId;
        std::string m_DownLoadBranch;
        std::string m_UpLoadBranch;
        std::string m_BasePath;
        std::string m_editor;
        std::string m_address;

        std::vector<std::vector<LineObj>> m_lineobjs;                   // 建图结果的所有车道线
        std::vector<std::vector<LineObj>> m_roadboundaryobjs;           // 建图结果的所有车道线
        Engine::Base::Array<hdmap_build::RoadMarkObj> m_roadmarks;      // 建图所有的地面标识
        Engine::Base::Array<hdmap_build::TrafficSignObj> m_trafficsign; // 建图标牌
        std::vector<int> tile_ids;
    };
}
#endif // HDMAP_BUILD_ELEMENTDEAL_H
