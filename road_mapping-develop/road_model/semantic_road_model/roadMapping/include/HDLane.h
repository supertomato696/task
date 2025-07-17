//
//
//
#ifndef HDMAP_BUILD_HDLANE_H
#define HDMAP_BUILD_HDLANE_H
#include "Vec.h"
#include "RoadTopoBuild.h"
#include <vector>
#include <string>
#include "Geometries/Coordinate.h"
#include "Base/Types.h"
#include "LineObj.h"

using namespace Engine;
using namespace Engine::Base;
using namespace Engine::Geometries;

namespace hdmap_build
{
    class HDLane
    {
    public:
        HDLane();
        ~HDLane();

        struct GroupLane
        {
            std::vector<Vec3> leftPoints;
            std::vector<Vec3> rightPoints;
        };

        // 根据逻辑连接ID将线完整组装
        void ConnectLineByID(std::vector<LineObj> &OriLaneMarks, std::vector<LineObj> &resLanes);

        // 区分左右
        void GroupLeftRight(std::vector<std::vector<LineObj>> &vecLanes, std::vector<std::vector<GroupLane>> &vecGroupLanes);

        // 删除短线
        void RemoveShortHDLanes(vector<vector<LineObj>> &tileHDLaneVecVec);

        // 进行节点对齐
        void NodeAline(vector<vector<LineObj>> &tileHDLaneVecVec); // 节点对齐整个流程

        void NodeAlineLaneGroup(vector<LineObj> &HDLaneVec); // 针对一个lanegroup节点对齐操作

        bool GetOutstandLane(vector<LineObj> &HDLaneVec, LineObj &resHDLane); // 计算位置最突出的线

        void NodeAlineLeftRight(vector<LineObj> &HDLaneVec, const LineObj &longestLane, bool isLeft); // 处理最左右侧线，太长删除，较长不管；短则拉齐

        void NodeAlineMiddle(vector<LineObj> &HDLaneVec); // 处理中间线，有车道变化，先不管；无车道变化，拉长

        void NodeAlineBreak(vector<LineObj> &HDLaneVec); // 用最左侧线起点再次进行打断平齐操作

        void ClipLane(LineObj &dealHDLane, const LineObj &compareHDLane); // 根据短线打断长线

        bool Flexibility(LineObj &dealHDLane, const LineObj &compareHDLane); // 根据长线拉长短线

        void ReverseLane(vector<LineObj> &HDLaneVec);

        void ResampleCHDLane(vector<vector<LineObj>> &tileHDLaneVecVec, double tolerance);

        void SmoothSTurnCHDLane(vector<vector<LineObj>> &tileHDLaneVecVec);

        void ConnectHDLanes(vector<vector<LineObj>> &tileHDLaneVecVec);

        void ClipByYellow(Array<Coordinate> &yellowPnts, vector<vector<LineObj>> &tileHDLaneVecVec);

        // 转换结构
        void HDLaneToLineObject(LineObj _hdlane, LineObject &_lineobject, int i, int j);

        void LineObjectsToHDLaneArr(Array<LineObject> &lineObjectArr, vector<vector<LineObj>> &oriTileHDLaneVecVec, vector<vector<LineObj>> &tileHDLaneVecVec);

        LineString *LineObjToLineString(LineObj _hdlane, bool PROJ = false);

        // 输出lanegroup相关
        void WriteToOBJ(std::vector<GroupLane> &vecLanes, std::string filePath, std::string frameid, bool utm = true);

        void WriteToOBJ(GroupLane &vecLanes, std::string filePath, std::string frameid, bool utm = true);

        void GroupPoints(std::vector<std::vector<LineObj>> &vecVecLanes, std::vector<GroupLane> &groupLanes);

    private:
        // 判断两条线是否进行连接
        bool IsConnect(const Array<Coordinate> &firstLine, const Array<Coordinate> &compareLine);

    public:
        std::string m_tilePath;
        std::string m_tileID;
    };
}
#endif // HDMAP_BUILD_HDLANE_H
