//
//
//

#ifndef HDMAP_BUILD_ROADMARK_H
#define HDMAP_BUILD_ROADMARK_H
#include "RoadTopoBuild.h"
#include "CloudAlgorithm.h"
#include "Base/Types.h"
#include "Base/Array.h"
#include "Geometries/Coordinate.h"
#include "Geometries/LineString.h"
#include <string>

#include "../data-access-engine/manager/road_geometry_mgr.h"

using namespace std;
using namespace Engine;
using namespace Engine::Base;
using namespace Engine::Geometries;

namespace hdmap_build
{
    struct NearestLankMark
    {
        Int32 leftLineIndex; // 左侧最近线
        Double leftDis;      // 左侧最近距离
        Coordinate sPtLeft;  // 左侧最近距离起点
        Coordinate ePtLeft;  // 左侧最近距离终点

        Int32 rightLineIndex; // 右侧最近线
        Double rightDis;      // 右侧最近距离
        Coordinate sPtRight;  // 右侧最近距离起点
        Coordinate ePtRight;  // 右侧最近距离终点
    };

    struct PointToLineInfo
    {
        Double dDistance;
        Double dMeasure;
        UInt32 index;
        Int32 nSide;
        Coordinate projectPnt;
    };

    struct RoadMarkObj
    {
        Array<Coordinate> arrPackPnts; // 外包络框点集
        Array<Coordinate> plygonPnts;  // 矩形框点集
        Coordinate centerPt;           // 中心点
        TLineTwoPoints axisTwoPts;     // 轴线
        int nearstLineIndex;
    };

    class RoadMark
    {
    public:
        RoadMark();
        ~RoadMark();

        // 初始化参数
        void InitParameter(string strTilePath, string road_branch, int tile_id);

        // 提取RoadMark的外轮廓函数
        void ExtractPackages(Array<Array<Coordinate>> &arrArrPackPoints);

        // 根据车道线确认地面标识的主轴
        void GetRoadMarkObjs(const Array<LineString *> &arrLaneMarks, Array<Array<Coordinate>> &arrArrPackPoints, Array<RoadMarkObj> &arrRoadMarkObj);

        // 计算需绑定的LaneSection并据此计算沿道路行驶方向的矩形框
        void BindRoadMarkToLane(Array<Array<Coordinate>> &arrArrPackPoints, data_access_engine::TileInfoList &corrtiles);

        // 输出上传前数据
        void WriteRoadMarkToObj(const data_access_engine::TileInfoList &tiles, string path, string name);

        std::shared_ptr<data_access_engine::PositionObjectProxy> createObject(const vector<Vec3> &arrPolygonPts, data_access_engine::TileInfoPtr &tile);

        // 对数据与原有数据进行去重
        void DealDuplicatePosition(Array<RoadMarkObj> &lineobjs, std::vector<std::shared_ptr<data_access_engine::PositionObjectProxy>> &vecPositions);

    private:
        void GetNearestLankMark(const Array<LineString *> &arrLaneMarks, Array<MatchLineInfo> &arrMatchLines, NearestLankMark &resLankMark);
        bool CalOriAxis(const Array<Coordinate> &arrCloudPts, TLineTwoPoints &resAxis, double &_dAvageThinness);

        void CalculaAxis(Coordinate centerPt, Array<Coordinate> &arrCloudPts, const Array<LineString *> &arrLaneMarks, NearestLankMark resLankMark, TLineTwoPoints &resAxis, int &nearstLine);
        void GetConerPnts(const Array<Coordinate> &arrCloudPts, TLineTwoPoints resAxis, Array<Coordinate> &conerPnts);
        void CalPtsToLineInfo(const Array<Coordinate> &arrCloudPts, const TLineTwoPoints &resAxis, Array<PointToLineInfo> &arrPointInfoSort);

    private:
        int m_TileId;
        std::string m_UpLoadBranch;
        std::string m_BasePath;
        Coordinate m_offset;
    };
}
#endif // HDMAP_BUILD_ROADMARK_H
