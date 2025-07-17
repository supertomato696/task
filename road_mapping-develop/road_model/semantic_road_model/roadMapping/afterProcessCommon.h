//
//
//
/* 将后处理中一些公用函数从afterProcess中剥离出来*/
#ifndef ROADMAPPING_AFTERPROCESSCOMMON_H
#define ROADMAPPING_AFTERPROCESSCOMMON_H

#include "Base/Array.h"
#include "Geometries/Coordinate.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/LineString.h"
#include "Geometries/LinearRing.h"
#include "Geometries/Polygon.h"

namespace RoadMapping
{
#define RAD2DEG 57.29577951308232
    /* 在后处理过程中需要用到的结构体 */
    // Lane Boundary 车道边界线
    struct LB
    {
        Engine::Base::Array<Engine::Geometries::Coordinate> linePts;
        double len;
        double disToRef;  // 到参考线的距离，左侧为负，右侧为正
        double SToRef;    // 起点到参考线的垂直距离
        double EToRef;    // 终点到参考线的垂直距离
        double disToRefS; // 到参考线起点的距离
        bool isCutByRefS; // 记录线的起点是否经过切分，为节点对其准备
        bool isCutByRefE; // 记录线的终点是否经过切分，为节点对其准备

        int leftOrRight;  // 记录位于link的哪一侧
        std::string link; // 记录归属于哪一条link
        int type;
    };

    // 车道组
    struct LG
    {
        Engine::Base::Array<LB> laneGroupLBs;
        int refIndex; // 对应的参考线的segIndex
    };

    struct RM
    {
        Engine::Base::Array<Engine::Geometries::Coordinate> plygonPnts; // 矩形框点集
        int label;
        int ObjectType;
        int ArrowType = 0;
    };

    struct ImpassableArea
    {
        Engine::Base::Array<Engine::Geometries::Coordinate> plygonPnts;
        int subtype;
        Engine::Geometries::Coordinate midpt;
        bool isBind;
    };

    struct Junction
    {
        Engine::Base::Array<Engine::Geometries::Coordinate> lukou_polygon_pts; // polygon的顶点
        Engine::Base::Array<LB> lukoubds; // 路口信息
        Engine::Base::Array<ImpassableArea> areas_relation; // 不可同行区域
    };

    class afterProcessCommon
    {

    public:
        /*以下为一些在后处理中常用的函数*/
        /********************** 1、文件的读取  **********************/
        // 读取结果文件中后缀为"_trjCloud.pcd"的pcd文件
        static void ReadLB(std::string dataDir, Engine::Base::Array<LB> &arrLines);

        // 读取参考link
        static void ReadRfLine(std::string dataDir, Engine::Base::Array<Engine::Geometries::Coordinate> &refLine);

        // 将obj结果读入
        static void ReadObj(std::string filepath, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts);

        // 将txt结果读入
        static void Readtxt(std::string filepath, Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts);

        // 读取生成的地面标志
        static void ReadRM(std::string dataDir, int label, Array<RM> &arrRMs);

        // 读取生成的地面标志
        static void ReadRM_wq(std::string dataDir, int label, Array<RM> &arrRMs);

        // 读取生成的斑马线、停止线等
        static void ReadRMObj(std::string dataDir, Array<RM> &arrRMs);

        // 读取停止线
        static void ReadStopLine(std::string stopLineObj, Engine::Base::Array<RM> &arrRMs);

        // 读取斑马线
        static void ReadCrossWalk(std::string dataDir, Engine::Base::Array<RM> &arrRMs);

        // 根据json文件读取sub_crosses_center
        static void ReadCenterPts(std::string paraJson, Engine::Base::Array<Engine::Geometries::Coordinate> &centerPts);
        /********************** 2、线相关的处理  **********************/
        // 对线型进行抽稀
        static void ResampleLBs(Engine::Base::Array<LB> &arrLBs, double tolerance = 0.01);

        // 对线型按照3点平滑方法进行平滑处理
        static void SmoothLBs(Engine::Base::Array<LB> &arrLBs, double tolerance);

        // 对车道组内线进行抽稀
        static void ResampleLGs(Engine::Base::Array<LG> &arrLGs, double tolerance = 0.01);

        // 对内部线进行反转,仅改变单根线的方向
        static void ReverseLane(Array<LB> &arrLBs);

        // 对内部线进行反转,并反转其存储顺序
        static void ReverseLane2(Array<LB> &arrLBs);

        // 根据json文件读取polygon
        static void ReadPolygonPts(std::string dataDir, Engine::Base::Array<Engine::Geometries::Coordinate> &refLine);

        // 线根据polygon裁剪
        static void ClipByPolygon(Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts, const Engine::Base::Array<Engine::Geometries::Coordinate> &polyPts);

        // 判断一个polygon中点是否在另一个polygon内
        static bool IsInPolygon(const Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts, const Engine::Base::Array<Engine::Geometries::Coordinate> &polyPts);

        // 将线序列转化为polygon
        static void line_pts_to_geo_polygon(const Engine::Base::Array<Engine::Geometries::Coordinate> &plyCoords, Engine::Geometries::Polygon &resPolygon);
        /********************** 3、点相关处理  **********************/
        // 将utm坐标转换为wgs84坐标，需要知道坐标点所处度带
        static void utmToWgs84(Engine::Geometries::Coordinate &in_pt, int zone);

        static void Wgs84Toutm(Engine::Geometries::Coordinate &in_pt, int zone);

        /********************** 4、测试输出相关  **********************/
        // 测试输出所有车道组 keychar自定义名称
        static void WriteLGToObj(std::string dataDir, std::string keychar, const Engine::Base::Array<LG> &arrLGs);

        // 测试输出车道线 keychar自定义名称
        static void WriteLBToObj(std::string dataDir, std::string keychar, const Engine::Base::Array<LB> &arrLGs);

        // 测试输出地面标识 keychar自定义名称
        static void WriteRMToObj(std::string dataDir, std::string keychar, const Engine::Base::Array<RM> &arrRMs);
    };
}

#endif // ROADMAPPING_AFTERPROCESSCOMMON_H
