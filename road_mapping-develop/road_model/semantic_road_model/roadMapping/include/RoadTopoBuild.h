//
//
//

#ifndef HDMAP_BUILD_ROADTOPOBUILD_H
#define HDMAP_BUILD_ROADTOPOBUILD_H

#include "Base/Types.h"
#include "Geometries/Coordinate.h"
#include "Geometries/Envelope.h"
#include "Base/Array.h"
#include "Vec.h"
#include <map>

using namespace Engine;
using namespace Engine::Base;
using namespace Engine::Geometries;

namespace hdmap_build
{
    struct MatchLineInfo
    {
        Int32 nLinkIndex;
        Coordinate sPnt;
        Coordinate ePnt;
        Coordinate testPnt;
        Coordinate projPoint;
        Double projectDis;
        Int32 nSide; // 1��ࣻ2�Ҳ�
    };

    struct GridNumberTag
    {
        Int32 NumberTag;            // ���������
        Array<Int32> arrPointIndex; // �����ڵ������
    };

    struct LineObject
    {
        Int32 nSide;
        Int32 nObjType;
        Int32 nLineType;
        LineString *pLine;
        Int32 groupID;
        Int32 indexInGroup;
        Double length;
    };

    struct GroupLineObject
    {
        Array<LineObject> arrLineObject;
        double avgDis;
    };

    struct PointIndex_Tag
    {
        Int32 nPointIndex;
        Int32 nNumberTag;
    };

    class TGridSegment
    {
    public:
        Int32 nLinkIndex;    // λ��������
        Int32 nLinkSegIndex; // λ�������ߵ�����
        Int32 nPntIndex;     // �����ڵ㼯�е�����
    };

    class TopoGridUnit
    {
    public:
        Bool AddSegment(const TGridSegment &tSegment);
        TopoGridUnit();
        ~TopoGridUnit();

        Int32 nSegmentsCount;
        TGridSegment *pSegments;
    };

    class TopoGridEnv
    {
    public:
        TopoGridEnv();
        ~TopoGridEnv();

        Int32 IndexCols;
        Int32 IndexRows;
        Double dGridInterval;
        Envelope rcBounds;

        TopoGridUnit *pGridUnits;
    };

    class RoadTopoGrid
    {
    public:
        RoadTopoGrid();
        virtual ~RoadTopoGrid();

    public:
        //! \brief
        //! \param arrpGeoLines [in] �߶�������
        //! \param dGridInterval [in] ������ȣ�Ĭ��5��
        bool BuildTopoGrid(const Array<Engine::Geometries::LineString *> &arrpGeoLines, double dGridInterval = 5.0);

        //! \brief ������������
        //! \param arrPoints [in] �߶�������
        //! \param dGridInterval [in] ������ȣ�Ĭ��0.5��
        Bool BuildTopoGrid(const Array<Coordinate> &arrPoints, Double dGridInterval = 0.5);

        //! \brief ��ָ����Χ�ڹ����߶������
        //! \param bounds [in] ָ�������Χ
        //! \param arrPoints [in] �߶�������
        //! \param dGridInterval [in] ������ȣ�Ĭ��0.5��
        Bool BuildTopoGrid(Envelope &rcBounds, const Array<Coordinate> &arrPoints, Double dGridInterval = 0.5);

        //! \brief ��ָ����Χ�ڹ����߶������
        //! \param rcBounds [in] ָ�������Χ
        //! \param arrpGeoLines [in] �߶�������
        //! \param dGridInterval [in] ������ȣ�Ĭ��0.5��
        Bool BuildTopoGrid(Envelope &rcBounds, const Array<LineString *> &arrpGeoLines, Double dGridInterval = 0.5);

    public:
        //! \brief 检测函数
        //! \param arrpntTrack [in] 检测点集
        //! \param direction_type true 仅同向进行对比 false 仅反向对比
        Bool GetMatchInfos(Array<Coordinate *> &arrpntTrack, Array<Int32> &arrPointMatch, double dTolerance, bool direction_type = true);

        Bool GetMatchInfos(Array<Coordinate> &arrpntTrack, Array<Int32> &arrPointMatch, double dTolerance, bool direction_type = true);

        Bool GetMatchInfos(Array<Coordinate *> &arrpntTrack, Int32 nIndex, Array<Int32> &arrPointMatch, double dTolerance, bool direction_type = true);

        Bool GetMatchInfos(Array<Coordinate> &Points, Array<Coordinate *> &arrpntTrack, Array<Int32> &arrPointMatch);

        Bool GetMatchInfos(Array<Coordinate> &Points, Array<Coordinate *> &arrpntTrack, Array<Int32> &arrPointMatch, Double dTolerance);

        Bool GetMatchInfos(Coordinate pntTrack, Array<Coordinate> &arrPointMatch, Array<Int32> &arrMatchIndex, Double dMaxDistance);

        // 根据点距离进行网格聚类
        Void ConnectedComponent(Array<Coordinate> &arrPoints, Array<Int32> &arrMarked, Double dMaxDistance = 0.5);

        // 得到距离一点给定buffer的线
        Bool GetLinesInDis(Coordinate pntTrack, double dTolerance, Array<MatchLineInfo> &arrResultInfos);

    private:
        Bool IniTopGrid(const Envelope &bounds, Double dGridCount);

        Bool BuildTopoGridSegment(LineString *pGeoLine, Int32 nGeoIndex, Int32 &nTickCount);

        Bool BuildTopoGridPoints(const Array<Coordinate> &arrPoints);

        Double DistanceToSegment(const Coordinate &pntHitTest, const Coordinate &pntStart, const Coordinate &pntEnd, Coordinate &pntMatch);

        Double DistanceToSegment(const Coordinate &pntHitTest, const Coordinate &pntStart, const Coordinate &pntEnd);

        Bool IsIntersectLineSect(const Coordinate &pntStart1, const Coordinate &pntEnd1, const Coordinate &pntStart2, const Coordinate &pntEnd2);

        Bool GetNearPoints(Array<Coordinate> &arrPoints, Int32 nIndex, Int32 VertexRow, Int32 VertexCol, Array<Int32> &arrGridMarked, Array<Int32> &arrPointIndex, Double dMaxDistance);

        Bool GetGridSegments(const Array<Coordinate> &arrPoints, const Int32 gridIndex, const Array<Int32> &arrMarked, Array<GridNumberTag> &differNumberTag);

        Bool GetMindisBetweenSegmets(const Array<Coordinate> &arrPoints, const GridNumberTag &gridNumberTag, const GridNumberTag &neighborNumberTag, Double dMaxDistance);

        Bool Merge(Array<Int32> &arrMarked, Array<Int32> &arrRealMarked, Int32 nNumberTag, Array<Int32> &arrPointIndex);

        Bool MergeSameGrid(const Array<Coordinate> &arrPoints, Array<Int32> &arrMarked, Array<Int32> &arrRealMarked, Array<GridNumberTag> &differNumberTag, Double dMaxDistance = 0.5);

        Void MergeGridTag(const Array<Coordinate> &arrPoints, Int32 VertexRow, Int32 VertexCol, Array<Int32> &arrMarked, Array<Int32> &arrRealMarked, Array<GridNumberTag> &differNumberTag, Array<Int32> &arrGridMarked, Double dMaxDistance = 0.5);

        Bool IsSameValue(Int32 VertexRow, Int32 VertexCol, const Array<Int32> &arrMarked, Array<Int32> &arrGridMarked);

    private:
        TopoGridEnv m_topoGridEnv;
        Array<Coordinate> m_arrLinkPoints;
    };

    class RoadTopoBuild
    {
    public:
        RoadTopoBuild();
        ~RoadTopoBuild();

        //! \brief ȥ�ظ���
        //! \param arrLineObjects [in & out] �������߼���
        //! \param dTolerance [in] ������ֵ
        static Bool CleanDuplicateLines(Array<Array<LineObject>> &lineObjectArrArr, Array<LineObject> &arrLineObjects, Double dTolerance = 0.1, Bool joint = true, Double dLength = 10.0);

        static Bool DelDuplicateSegs(Array<Coordinate> &arrCroods, Array<Int32> &arrPointMatch, Array<Array<Coordinate>> &arrResCroods, Double dLength = 5.0);

        // 有锐角说明线有折回
        static Void SmoothSTurnSegments(Base::Array<Geometries::Coordinate> &vecInput);
        //! \brief ��ƥ��
        //! \param PrecisePoints [in] ���ȵ�
        //! \param arrArrCloudPoints [in] �����������
        //! \param max_distance_in_seg [in] ������
        static Bool MatchPoints(Array<Coordinate> &PrecisePoints, Array<LineString *> &MatchLines, Array<Array<Int32>> &arrMatchInfos);

        //! \brief ��ƥ��
        //! \param topoGridPoints [in] ���ȵ��ѹ���������Ҫ��ǰ������
        //! \param PrecisePoints [in] ���ȵ�
        //! \param MatchLines [in] ��ƥ����
        //! \param arrMatchInfos[out] ƥ����
        //! \param dTolerance[in] �����ƥ��ľ��ȣ�XYͶӰ�棩
        static Bool MatchPoints(RoadTopoGrid &topoGridPoints, Array<Coordinate> &PrecisePoints, LineString *MatchLines, Array<Int32> &arrMatchInfos, Double dTolerance);

        static Void LineSmooth3(Array<Coordinate> &inputPoints, Array<Coordinate> &outputPoints); // ������������ƽ�� https://blog.csdn.net/u014345526/article/details/43817309
        static Void LineSmooth5(Array<Coordinate> &inputPoints, Array<Coordinate> &outputPoints); // ����5������ƽ��
        static Void LineSmooth7(Array<Coordinate> &inputPoints, Array<Coordinate> &outputPoints); // ����7��ƽ��

        //! 点网格聚类
        //!  \param arrCloudPoints [in] ԭʼ����
        //!  \param arrArrCloudPoints [in] �����������
        //!  \param max_distance_in_seg [in] ������
        static Bool ConnectedComponent(Array<Coordinate> &arrCloudPoints, Array<Array<Int32>> &arrArrCloudPoints, Double dDistance);

    private:
        static Bool DelDuplicateSegs(Array<LineObject> &arrLineObjects, Int32 nIndex, Array<Int32> &arrPointMatch, Double dLength = 10.0);
        static Bool Joints(Array<LineObject> &arrLineObjects);
    };

}

#endif // HDMAP_BUILD_ROADTOPOBUILD_H
