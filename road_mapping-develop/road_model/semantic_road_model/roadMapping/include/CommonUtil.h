//
//
//

#ifndef HDMAP_BUILD_COMMONUTIL_H
#define HDMAP_BUILD_COMMONUTIL_H
#include "LineObj.h"
#include "Vec.h"
#include "Geometries/LineString.h"
namespace hdmap_build
{
    class CommonUtil
    {
    public:
        CommonUtil();
        ~CommonUtil();

    public:
        /**********************************************测试相关*******************************************************/
        // 测试输出线型(utm 50N)
        static void WriteToOBJ(std::vector<LineObj> &vecLanes, std::string filePath, std::string frameid, bool utm = true);

        static void WriteToOBJ(Array<LineString *> &vecLanes, std::string filePath, std::string frameid, bool utm = true);

        static void WriteToOBJ(std::vector<std::vector<LineObj>> &vecLanes, std::string strTilePath, std::string tileid, bool utm = true);

        static void WriteToOBJ(Array<Array<Coordinate>> &vecLanes, std::string strTilePath, std::string tileid, bool utm = true);

        static void WriteToTxt(Array<Coordinate> &vecLanes, std::string filePath, std::string name, bool utm);
        /**********************************************点线转换相关*******************************************************/
        // Coordinate 点与Vec3 相互转换
        static vector<Vec3> EngineCroodToVec3(Array<Coordinate> arrPnts);
        static Array<Coordinate> Vec3CroodToEngine(vector<Vec3> arrPnts);

        // utm 50N度带转换为 wgs84相互转换
        static vector<Vec3> Utm2Wgs84(vector<Vec3> oriVecVec3);
        static void Utm2Wgs84(Array<Coordinate> &oriVecVec3);
        static void Wgs84toUtm(Array<Coordinate> &oriVecVec3);
        static vector<Vec3> Wgs84toUtm(vector<Vec3> oriVecVec3);
        // 点组织成线
        static LineString *CoordsToLineString(Array<Coordinate> &inputPoints); // ����7��ƽ��
        // 线转换为点
        static void EngineLinestringToVec3Points(const LineString *line, Array<Coordinate> &resCoords);

        /**********************************************点计算相关*******************************************************/
        static Coordinate GetAveragePt(const Array<Coordinate> &arrCloudPts);

        // 根据搜索半径和点密度过滤点 searchRadius搜索半径 filterDensity半径内点个数要求
        //  autoCalDensity = false则按照要求筛选，否则true自动计算平均个数，按照平均个数的30%进行筛选
        static void FilterPtByRadius(Array<Coordinate> &arrCloudPts, double searchRadius, double filterDensity, bool autoCalDensity = false);

        // 点位移
        static void TransGet(Array<Coordinate> &CloudPoints, Coordinate &pntOrign); // 得到位移参考点，并进行转换
        static void TransSub(const Coordinate &pntOrign, Array<Coordinate> &arrPoint);
        static void TransSub(const Coordinate &pntOrign, Array<Array<Coordinate>> &arrPoint);
        static void TransPlus(const Coordinate &pntOrign, Array<Coordinate> &arrPoint);
        static void TransPlus(const Coordinate &pntOrign, Array<Array<Coordinate>> &arrEntrePoints);
        /**********************************************线计算相关*******************************************************/
        // 计算线长
        static double GetLength(const Array<Coordinate> &arrPnts);
        // 对线进行抽稀
        static bool Resample(Array<Coordinate> &coordinates, double tolerance);
        // 去除线上重复点
        static void RemoveDuplicatePoints(Base::Array<Geometries::Coordinate> &vecInput, const Base::Double dTolerance);
        // 多段线中计算长度最长的线,并返回长度值
        static double LongestLine(Array<Array<Coordinate>> &arrLines, Array<Coordinate> &longest);

        static bool getInterpolationLine(Engine::Base::Array<Engine::Geometries::Coordinate> &coordinates, double interval = 1.0, int save_ori_pts = 0);

        // 计算任意多边形的面积，顶点按照顺时针或者逆时针方向排列
        static double ComputePolygonArea(const Engine::Base::Array<Engine::Geometries::Coordinate> &points)
        {
            int point_num = points.GetCount();
            if (point_num < 3)
                return 0.0;
            double s = 0;
            for (int i = 0; i < point_num; ++i)
                s += points[i].x * points[(i + 1) % point_num].y - points[i].y * points[(i + 1) % point_num].x;
            return fabs(s / 2.0);
        }
        /**********************************************Tile计算相关*******************************************************/
        static int GetBit(int v, int i)
        {
            return ((int)v >> i) & 1;
        }

        static int WGS_to_tile_ID(double x, double y, double z);
        static bool get_tile_center_WGS(int tile_id, double &wgsx, double &wgsy);
        static bool get_tile_box_WGS(int tile_id, double &minx, double &miny, double &maxx, double &maxy);
        static void Tile_box_obj(string tilePath, int tile_id);
    };
}
#endif // HDMAP_BUILD_COMMONUTIL_H
