//
//
//

#ifndef HDMAP_BUILD_DATARENDER_H
#define HDMAP_BUILD_DATARENDER_H
#include <string>
#include "Vec.h"
#include "LineObj.h"
#include "HDLane.h"
#include "TrafficSign.h"

using namespace std;

namespace hdmap_build
{
    class DataReader
    {
    public:
        DataReader();
        ~DataReader();

    public:
        static DataReader *instance()
        {
            static DataReader instance;
            return &instance;
        }

    public:
        // 读取gpkg文件
        //        bool LoadGpkg(std::string _filename, vector<LineObj>& VecHDLane);
        // 文件路径修复
        static void postfix(string &strDir);

        // 按照特殊字符分割字符串
        static std::vector<std::string> SplitString(const string &strSrc, const string &pattern);

        // 读取json文件
        static bool LoadJson(std::string _filename, vector<LineObj> &VecHDLane);

        // 读取RoadBoundary文件
        static bool LoadRoadBoundaryJson(std::string _filename, vector<LineObj> &VecHDLane);

        // 读取TMP_EDGE 的json文件
        static bool LoadTmpEdgeJson(std::string _filename, Array<Coordinate> &yellowPnts);

        // 读取所有的json文件
        static bool LoadAllLanes(std::string _filename, vector<vector<LineObj>> &tileHDLaneVecVec, Array<Coordinate> &yellowPnts);
        static bool LoadRoadBoundarys2(std::string strTilePath, vector<vector<LineObj>> &tileHDLaneVecVec);
        static bool LoadRoadBoundarys(std::string strTilePath, vector<vector<LineObj>> &tileHDLaneVecVec);
        // 解析tileid
        static int GetTileId(std::string _filename);

        // 读取SD文件
        static bool LoadSDJson(std::string _filename, std::string _tileID, std::vector<string> strLinkList, vector<LineObj> &VecHDLane);

        // 读取RoadMark的json文件
        static bool LoadRoadMarkJson(std::string _filename, Array<Coordinate> &RoadMarkPnts, Int32 &roadMarkCount);
        static bool LoadAllRoadMarkJson(std::string _filename, std::string _tileID, Array<Coordinate> &RoadMarkPnts, Int32 &allPtsCn);

        // 读取TrafficSign的json文件
        static bool LoadTrafficSignJson(std::string _filename, Array<TrafficSignObj> &arrTrafficSignObj);
        static bool LoadAllTrafficSignJson(std::string _filename, std::string _tileID, Array<TrafficSignObj> &arrTrafficSignObjs);
    };

    //    class CDataSqlite
    //    {
    //
    //    public:
    //        CDataSqlite();
    //        virtual ~CDataSqlite();
    //
    //    public:
    //        bool Open(const char *_filename);//打开SQL
    //        void Close();//关闭SQL
    //
    //        bool read(LineObj& _HDLane);
    //    private:
    //        bool execSql(const char *_sql);
    //
    //    public:
    //        //读取相关
    //        bool read_start(const char *_sql );
    //
    //        void  read(bool &_cValue, const int _nID);//sqlite3_column_int
    //        void  read(char &_cValue, const int _nID);//sqlite3_column_int
    //        void  read(int &_nValue, const int _nID);//sqlite3_column_int
    //        void  read(int64_t &_nValue, const int _nID);//sqlite3_column_int64
    //        void  read(float &_fValue, const int _nID);//sqlite3_column_double
    //        void  read(double &_fValue, const int _nID);//sqlite3_column_double
    //        void  read(char* &_sValue, const int _nID);//sqlite3_column_text
    //        void  read(string &_sValue, const int _nID);//sqlite3_column_text
    //        bool  read_geo(std::vector<Vec3> &_pGeo, const int _nID);
    //        bool read_next();//
    //        bool read_end();
    //
    //        int getFeatureCount();//
    //
    //        void clear_note();//
    //
    //    private:
    //        sqlite3_stmt *m_stmt;
    //        sqlite3 *m_pSqliteDB;
    //        void *m_pcache;
    //    };

}

#endif // HDMAP_BUILD_DATARENDER_H
