//
//
//
#include "DataReader.h"
#include "Log.h"
#include "HDLane.h"
#include "LineObj.h"
#include "TrafficSign.h"
#include "CommonUtil.h"

#include <jsoncpp/json.h>
#include <jsoncpp/config.h>
// #include <spatialite/gaiageo.h>
// #include <spatialite.h>
// #include <spatialite/gg_structs.h>
// #include <sqlite3.h>
#include <string>
#include <vector>
#include <iostream>
#include <dirent.h>

using namespace std;
using namespace hdmap_build;

DataReader::DataReader()
{
}

DataReader::~DataReader()
{
}

// bool DataReader::LoadGpkg(std::string _filename, vector<LineObj>& VecHDLane)
//{
//     VecHDLane.clear();
//     if(_filename.empty())
//     {
//         VULCAN_LOG_INFO("gpkg filePath has no data.gpkg file...");//为空输出log
//         return false;
//     }
//
//     //读取gpkg文件中HD_LANE_EDGE
//     CDataSqlite *pDSqlite = new CDataSqlite;
//     bool bok = false;
//
//     if (pDSqlite->Open(_filename.c_str()))//把gpkg文件作为SQL数据库处理
//     {
//         //筛选条件
//         std::string sql;
//         sql.append("SELECT fid, LANE_EDGE_ID, PREV_ID, NEXT_ID, LANE_EDGE_SEQUENCE, geom ");
//         sql.append("FROM HD_LANE_EDGE ");
//         sql.append("ORDER BY fid");
//         if (pDSqlite->read_start(sql.c_str()))
//         {
//             int ncount = pDSqlite->getFeatureCount();
//             VULCAN_LOG_INFO("select hd_lane_dege count: {}", std::to_string(ncount));
//             do{
//                 LineObj _HDLane;
//                 pDSqlite->read(_HDLane);
//                 VecHDLane.push_back(_HDLane);
//             }while(pDSqlite->read_next());
//
//             pDSqlite->read_end();
//         }
//         pDSqlite->Close();
//         bok = true;
//     }
//     delete pDSqlite;
//
//     if (!VecHDLane.empty())
//     {
//         bok = true;
//     }
//     return bok;
// }

void DataReader::postfix(string &strDir)
{
    if (strDir.back() == '/' || strDir.back() == '\\')
        return;
    strDir.push_back('/');
}

std::vector<std::string> DataReader::SplitString(const string &strSrc, const string &pattern)
{
    vector<string> resultstr;
    if (strSrc == "")
        return resultstr;

    // 添加在字符串最后，可以截取最后一段数据
    std::string strcom = strSrc + pattern;
    auto pos = strSrc.find(pattern);
    auto len = strcom.size();

    //
    while (pos != std::string::npos)
    {
        std::string coStr = strcom.substr(0, pos);
        resultstr.push_back(coStr);

        strcom = strcom.substr(pos + pattern.size(), len);
        pos = strcom.find(pattern);
    }

    return resultstr;
}

bool DataReader::LoadJson(std::string _filename, vector<LineObj> &VecHDLane)
{
    std::ifstream ifs(_filename);
    if (!ifs.is_open())
    {
        VULCAN_LOG_INFO("json filePath has no json file..."); // 为空输出log
        return false;
    }
    Json::Value root;
    JSONCPP_STRING errs;
    Json::CharReaderBuilder readerBuilder;
    if (!Json::parseFromStream(readerBuilder, ifs, &root, &errs))
    {
        VULCAN_LOG_INFO("Func<{}>: {} can't parse", "LoadJson", _filename); // 初始化错误输出log
        return false;
    }
    int ncount = root["features"].size();
    for (Json::Value::ArrayIndex i = 0; i < root["features"].size(); i++)
    {
        LineObj _hdlane;
        // 读取lane_edge_id prev_id next_id lane_edge_sequence
        _hdlane.m_id = i;

        string lane_edge_id = root["features"][i]["properties"]["LANE_EDGE_ID"].asString();
        _hdlane.m_lane_edge_id = atoll(lane_edge_id.c_str());

        string prev_id = root["features"][i]["properties"]["PREV_ID"].asString();
        if (prev_id != "null")
        {
            _hdlane.m_prev_id = atoll(prev_id.c_str());
        }

        string next_id = root["features"][i]["properties"]["NEXT_ID"].asString();
        if (next_id != "null")
        {
            _hdlane.m_next_id = atoll(next_id.c_str());
        }

        _hdlane.m_lane_edge_sequence = root["features"][i]["properties"]["LANE_EDGE_SEQUENCE"].asInt();

        // 读取点坐标
        int nCount = root["features"][i]["geometry"]["coordinates"].size();
        for (Json::Value::ArrayIndex j = 0; j < nCount; ++j)
        {
            Coordinate newVec3;
            newVec3.x = root["features"][i]["geometry"]["coordinates"][j][0].asDouble();
            newVec3.y = root["features"][i]["geometry"]["coordinates"][j][1].asDouble();
            newVec3.z = root["features"][i]["geometry"]["coordinates"][j][2].asDouble();
            _hdlane.m_lineCroods.Add(newVec3);
        }
        VecHDLane.push_back(_hdlane);
    }
    return true;
}

bool DataReader::LoadRoadBoundaryJson(std::string _filename, vector<LineObj> &VecHDLane)
{
    std::ifstream ifs(_filename);
    if (!ifs.is_open())
    {
        VULCAN_LOG_INFO("json filePath has no json file..."); // 为空输出log
        return false;
    }
    Json::Value root;
    JSONCPP_STRING errs;
    Json::CharReaderBuilder readerBuilder;
    if (!Json::parseFromStream(readerBuilder, ifs, &root, &errs))
    {
        VULCAN_LOG_INFO("Func<{}>: {} can't parse", "LoadJson", _filename); // 初始化错误输出log
        return false;
    }
    int ncount = root["features"].size();
    for (Json::Value::ArrayIndex i = 0; i < root["features"].size(); i++)
    {
        LineObj _hdlane;
        // 读取lane_edge_id prev_id next_id lane_edge_sequence
        _hdlane.m_id = i;

        // 读取点坐标
        int nCount = root["features"][i]["geometry"]["coordinates"].size();
        for (Json::Value::ArrayIndex j = 0; j < nCount; ++j)
        {
            Coordinate newVec3;
            newVec3.x = root["features"][i]["geometry"]["coordinates"][j][0].asDouble();
            newVec3.y = root["features"][i]["geometry"]["coordinates"][j][1].asDouble();
            newVec3.z = root["features"][i]["geometry"]["coordinates"][j][2].asDouble();
            _hdlane.m_lineCroods.Add(newVec3);
        }
        VecHDLane.push_back(_hdlane);
    }
    return true;
}

bool DataReader::LoadTmpEdgeJson(std::string _filename, Array<Coordinate> &yellowPnts)
{
    std::ifstream ifs(_filename);
    if (!ifs.is_open())
    {
        VULCAN_LOG_INFO("json filePath has no TMP_LANE_EDGE.json file..."); // 为空输出log
        return false;
    }
    Json::Value root;
    JSONCPP_STRING errs;
    Json::CharReaderBuilder readerBuilder;
    if (!Json::parseFromStream(readerBuilder, ifs, &root, &errs))
    {
        VULCAN_LOG_INFO("Func<{}>: {} can't parse", "LoadJson", _filename); // 初始化错误输出log
        return false;
    }
    int ncount = root["features"].size();
    for (Json::Value::ArrayIndex i = 0; i < root["features"].size(); i++)
    {
        string lane_edge_color = root["features"][i]["properties"]["LANE_EDGE_COLOR"].asString();
        // 判断线颜色是否为黄色
        if (lane_edge_color != "YELLOW")
        {
            continue;
        }

        // 读取点坐标
        int nCount = root["features"][i]["geometry"]["coordinates"].size();
        for (Json::Value::ArrayIndex j = 0; j < nCount; ++j)
        {
            Coordinate newVec3;
            newVec3.x = root["features"][i]["geometry"]["coordinates"][j][0].asDouble();
            newVec3.y = root["features"][i]["geometry"]["coordinates"][j][1].asDouble();
            newVec3.z = root["features"][i]["geometry"]["coordinates"][j][2].asDouble();
            yellowPnts.Add(newVec3);
        }
    }
    return true;
}

bool DataReader::LoadSDJson(std::string _filename, std::string _tileID, std::vector<string> strLinkList, vector<LineObj> &VecHDLane)
{
    // 读取路径中task_sd.json
    std::ifstream ifs(_filename);
    if (!ifs.is_open())
    {
        VULCAN_LOG_INFO("json filePath has no json file..."); // 为空输出log
        return false;
    }
    bool downLoadAllLinks = strLinkList.empty() ? true : false; // link序列为空，下载全部数据
    Json::Value root;
    JSONCPP_STRING errs;
    Json::CharReaderBuilder readerBuilder;
    if (!Json::parseFromStream(readerBuilder, ifs, &root, &errs))
    {
        VULCAN_LOG_INFO("Func<{}>: {} can't parse", "LoadJson", _filename); // 初始化错误输出log
        return false;
    }
    int ncount = root["tile_data"].size();
    Json::Value linkValues = root["tile_data"][_tileID];
    if (linkValues.isNull())
    {
        return false;
    }
    VULCAN_LOG_INFO("linkSize: {}", linkValues.size());
    int nid = 0;
    // 下载需要的linkID
    if (!downLoadAllLinks)
    {
        for (int i = 0; i < strLinkList.size(); ++i)
        {
            // 寻找特定的linkID
            LineObj _hdlane;
            VULCAN_LOG_INFO("link_id: {}", strLinkList[i]);
            Json::Value linkValue = linkValues[strLinkList[i]];
            VULCAN_LOG_INFO("link_id has size: {}", linkValue.size());
            if (linkValue.isNull())
            {
                VULCAN_LOG_INFO("link_id: {} not find......", strLinkList[i]);
                continue;
            }
            // 解析坐标
            string laneGeo = linkValue["geo"].asString();
            int bj = 0;
            int ej = 0;

            while (ej = laneGeo.find(";", bj))
            {
                string tempCCre = laneGeo.substr(bj, ej - bj);
                if (ej < 1)
                {
                    break;
                }

                int bbj = 0;
                int eej = 0;
                vector<double> coord;
                while (eej = tempCCre.find(",", bbj))
                {
                    string xyz = tempCCre.substr(bbj, eej - bbj);
                    if (eej < 1 && !xyz.empty())
                    {
                        coord.push_back(atof(xyz.c_str()));
                        break;
                    }

                    // 找到点
                    coord.push_back(atof(xyz.c_str()));
                    bbj = eej + 1;
                    eej = 0;
                }
                if (coord.size() == 3)
                {
                    Coordinate vec;
                    vec.x = coord[0];
                    vec.y = coord[1];
                    vec.z = coord[2];
                    VULCAN_LOG_INFO("v {}  {}  {}", vec.x, vec.y, vec.z);
                    _hdlane.m_lineCroods.Add(vec);
                }
                bj = ej + 1;
                ej = 0;
            }
            _hdlane.m_id = i + 1;
            _hdlane.m_lane_edge_id = atoll(strLinkList[i].c_str());
            nid++;
            VecHDLane.push_back(_hdlane);
        }
    }
    else
    {
        // 下载所有的linkID
        Json::Value::Members memberslink;
        memberslink = linkValues.getMemberNames();
        for (Json::Value::ArrayIndex i = 0; i < linkValues.size(); i++)
        {

            string linkID = memberslink[i];
            LineObj _hdlane;
            VULCAN_LOG_INFO("link_id: {}", linkID);
            // 解析坐标
            string laneGeo = linkValues[linkID]["geo"].asString();
            int bj = 0;
            int ej = 0;

            while (ej = laneGeo.find(";", bj))
            {
                string tempCCre = laneGeo.substr(bj, ej - bj);
                if (ej < 1)
                {
                    break;
                }

                int bbj = 0;
                int eej = 0;
                vector<double> coord;
                while (eej = tempCCre.find(",", bbj))
                {
                    string xyz = tempCCre.substr(bbj, eej - bbj);
                    if (eej < 1 && !xyz.empty())
                    {
                        coord.push_back(atof(xyz.c_str()));
                        break;
                    }

                    // 找到点
                    coord.push_back(atof(xyz.c_str()));
                    bbj = eej + 1;
                    eej = 0;
                }
                if (coord.size() == 3)
                {
                    Coordinate vec;
                    vec.x = coord[0];
                    vec.y = coord[1];
                    vec.z = coord[2];
                    VULCAN_LOG_INFO("v {}  {}  {}", vec.x, vec.y, vec.z);
                    _hdlane.m_lineCroods.Add(vec);
                }
                bj = ej + 1;
                ej = 0;
            }
            _hdlane.m_id = i + 1;
            _hdlane.m_lane_edge_id = atoll(linkID.c_str());
            nid++;
            VecHDLane.push_back(_hdlane);
        }
    }

    return true;
}

bool DataReader::LoadAllLanes(std::string strTilePath, vector<vector<LineObj>> &tileHDLaneVecVec, Array<Coordinate> &yellowPnts)
{
    // 读取子目录
    DIR *dir = opendir(strTilePath.c_str());
    if (dir == NULL)
    {
        VULCAN_LOG_INFO(" opendir tilepath <{}> is error......", strTilePath); // 为空输出log
        return false;
    }
    hdmap_build::HDLane _HDLane;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_DIR)
        { // It's dir
            cout << entry->d_name << endl;
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0 || strcmp(entry->d_name, "image_recog") == 0 || strcmp(entry->d_name, "camear") == 0 || strcmp(entry->d_name, "image_seg") == 0)
                continue;

            string dirLink = strTilePath + "/" + entry->d_name;
            postfix(dirLink);
            // VULCAN_LOG_INFO(" link path: <{}> ......", dirLink);
            string laneJsonPath = dirLink + "hd_map";
            postfix(laneJsonPath);

            string laneJsonFile = laneJsonPath + "HD_LANE_EDGE.json";
            VULCAN_LOG_INFO(" laneJsonFile path: <{}> ......", laneJsonFile);

            // 读取tmp中的黄色点信息
            string tmplaneJsonFile = laneJsonPath + "TMP_LANE_EDGE.json";
            if (!DataReader::LoadTmpEdgeJson(tmplaneJsonFile, yellowPnts))
            {
                VULCAN_LOG_INFO("TmpEdge json in <{}> load error", tmplaneJsonFile); // 为空输出log
            }

            vector<LineObj> VecLinkHDLane;
            // 解析json文件中数据
            if (!DataReader::LoadJson(laneJsonFile, VecLinkHDLane))
            {
                VULCAN_LOG_INFO("json in <{}> load error", laneJsonFile); // 为空输出log
                continue;
            }

            if (VecLinkHDLane.empty())
            {
                VULCAN_LOG_INFO("Link <{}> HD_LANE_EDGE size 0!!!!......", entry->d_name);
                continue;
            }

            VULCAN_LOG_INFO("Link <{}> HD_LANE_EDGE count: <{}> ......", entry->d_name, VecLinkHDLane.size());

            // 将线根据逻辑关系进行连接
            VULCAN_LOG_INFO("ConnectLineByID process start ...... ");
            std::vector<LineObj> resLinkLanes;

            _HDLane.ConnectLineByID(VecLinkHDLane, resLinkLanes);
            tileHDLaneVecVec.push_back(resLinkLanes);
            VULCAN_LOG_INFO("ConnectLineByID count <{}> ", resLinkLanes.size());
            // 输出线obj
            string output = laneJsonPath + "..//..//";
            CommonUtil::WriteToOBJ(resLinkLanes, output, entry->d_name, false);
            VULCAN_LOG_INFO("ConnectLineByID process finish ...... ");
        }
    }
    closedir(dir);
    return true;
}

bool DataReader::LoadRoadBoundarys(std::string strTilePath, vector<vector<LineObj>> &tileHDLaneVecVec)
{
    // 读取子目录
    DIR *dir = opendir(strTilePath.c_str());
    if (dir == NULL)
    {
        VULCAN_LOG_INFO(" opendir tilepath <{}> is error......", strTilePath); // 为空输出log
        return false;
    }
    hdmap_build::HDLane _HDLane;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_DIR)
        { // It's dir
            cout << entry->d_name << endl;
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0 || strcmp(entry->d_name, "image_recog") == 0 || strcmp(entry->d_name, "camear") == 0 || strcmp(entry->d_name, "image_seg") == 0)
                continue;

            string dirLink = strTilePath + "/" + entry->d_name;
            postfix(dirLink);
            // VULCAN_LOG_INFO(" link path: <{}> ......", dirLink);
            string laneJsonPath = dirLink + "hd_map";
            postfix(laneJsonPath);

            string laneJsonFile = laneJsonPath + "RoadBoundaryLayer.json";
            VULCAN_LOG_INFO(" laneJsonFile path: <{}> ......", laneJsonFile);

            vector<LineObj> VecLinkHDLane;
            // 解析json文件中数据
            if (!DataReader::LoadJson(laneJsonFile, VecLinkHDLane))
            {
                VULCAN_LOG_INFO("json in <{}> load error", laneJsonFile); // 为空输出log
                continue;
            }

            if (VecLinkHDLane.empty())
            {
                VULCAN_LOG_INFO("Link <{}> HD_LANE_EDGE size 0!!!!......", entry->d_name);
                continue;
            }

            VULCAN_LOG_INFO("Link <{}> HD_LANE_EDGE count: <{}> ......", entry->d_name, VecLinkHDLane.size());

            // 将线根据逻辑关系进行连接
            VULCAN_LOG_INFO("ConnectLineByID process start ...... ");
            std::vector<LineObj> resLinkLanes;

            _HDLane.ConnectLineByID(VecLinkHDLane, resLinkLanes);
            tileHDLaneVecVec.push_back(resLinkLanes);
            VULCAN_LOG_INFO("ConnectLineByID count <{}> ", resLinkLanes.size());
            // 输出线obj
            string output = laneJsonPath + "..//..//";
            CommonUtil::WriteToOBJ(resLinkLanes, output, entry->d_name, false);
            VULCAN_LOG_INFO("ConnectLineByID process finish ...... ");
        }
    }
    closedir(dir);
    return true;
}

bool DataReader::LoadRoadBoundarys2(std::string strTilePath, vector<vector<LineObj>> &tileHDLaneVecVec)
{
    // 读取子目录
    DIR *dir = opendir(strTilePath.c_str());
    if (dir == NULL)
    {
        VULCAN_LOG_INFO(" opendir tilepath <{}> is error......", strTilePath); // 为空输出log
        return false;
    }
    hdmap_build::HDLane _HDLane;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_DIR)
        { // It's dir
            cout << entry->d_name << endl;
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0 || strcmp(entry->d_name, "image_recog") == 0 || strcmp(entry->d_name, "camear") == 0 || strcmp(entry->d_name, "image_seg") == 0)
                continue;

            string dirLink = strTilePath + "/" + entry->d_name;
            postfix(dirLink);
            // VULCAN_LOG_INFO(" link path: <{}> ......", dirLink);
            string laneJsonPath = dirLink + "hd_map";
            postfix(laneJsonPath);

            string laneJsonFile = laneJsonPath + "HD_ROAD_BOUNDARY.json";
            VULCAN_LOG_INFO(" laneJsonFile path: <{}> ......", laneJsonFile);

            // 解析json文件中数据
            vector<LineObj> vecLane;
            if (!DataReader::LoadRoadBoundaryJson(laneJsonFile, vecLane))
            {
                VULCAN_LOG_INFO("json in <{}> load error", laneJsonFile); // 为空输出log
                continue;
            }

            if (vecLane.empty())
            {
                VULCAN_LOG_INFO("Link <{}> HD_LANE_EDGE size 0!!!!......", entry->d_name);
                continue;
            }

            VULCAN_LOG_INFO("Link <{}> HD_LANE_EDGE count: <{}> ......", entry->d_name, tileHDLaneVecVec.size());

            tileHDLaneVecVec.push_back(vecLane);
            // 输出线obj
            string output = laneJsonPath + "..//..//";
            string name = string(entry->d_name) + "_boundary";
            CommonUtil::WriteToOBJ(vecLane, output, name, false);
            VULCAN_LOG_INFO("ConnectLineByID process finish ...... ");
        }
    }
    closedir(dir);
    return true;
}

int DataReader::GetTileId(std::string strTilePath)
{
    int tile_id = -1;
    string tilepath = strTilePath;
    string::size_type nops1 = tilepath.find_last_of('/');
    string file1 = tilepath.substr(0, nops1);
    string::size_type nops2 = file1.find_last_of('/');
    string file2 = tilepath.substr(0, nops2);
    string::size_type nops3 = file2.find_last_of('/');
    string file3 = file2.substr(nops3 + 1, nops2);
    tile_id = atoi(file3.c_str());
    return tile_id;
}

bool DataReader::LoadRoadMarkJson(std::string _filename, Array<Coordinate> &RoadMarkPnts, Int32 &nCount)
{
    std::ifstream ifs(_filename);
    if (!ifs.is_open())
    {
        VULCAN_LOG_INFO("json filePath has no json file..."); // 为空输出log
        return false;
    }
    Json::Value root;
    JSONCPP_STRING errs;
    Json::CharReaderBuilder readerBuilder;
    if (!Json::parseFromStream(readerBuilder, ifs, &root, &errs))
    {
        VULCAN_LOG_INFO("Func<{}>: {} can't parse", "LoadJson", _filename); // 初始化错误输出log
        return false;
    }
    nCount = root["features"].size();
    for (Json::Value::ArrayIndex i = 0; i < root["features"].size(); i++)
    {
        // 读取点坐标
        int nCount = root["features"][i]["geometry"]["coordinates"].size();
        for (Json::Value::ArrayIndex j = 0; j < nCount; ++j)
        {
            Coordinate newVec3;
            newVec3.x = root["features"][i]["geometry"]["coordinates"][j][0].asDouble();
            newVec3.y = root["features"][i]["geometry"]["coordinates"][j][1].asDouble();
            newVec3.z = root["features"][i]["geometry"]["coordinates"][j][2].asDouble();
            RoadMarkPnts.Add(newVec3);
        }
    }
    return true;
}

bool DataReader::LoadAllRoadMarkJson(std::string strTilePath, std::string _tileID, Array<Coordinate> &RoadMarkPnts, Int32 &allSize)
{
    RoadMarkPnts.Clear();
    allSize = 0;
    // 读取子目录
    DIR *dir = opendir(strTilePath.c_str());
    if (dir == NULL)
    {
        VULCAN_LOG_INFO(" opendir tilepath <{}> is error......", strTilePath); // 为空输出log
        return false;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_DIR)
        { // It's dir
            cout << entry->d_name << endl;
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0 || strcmp(entry->d_name, "image_recog") == 0 || strcmp(entry->d_name, "camear") == 0 || strcmp(entry->d_name, "image_seg") == 0)
                continue;

            string dirLink = strTilePath + "/" + entry->d_name;
            postfix(dirLink);
            // VULCAN_LOG_INFO(" link path: <{}> ......", dirLink);
            string roadMarkJsonPath = dirLink + "hd_map";
            postfix(roadMarkJsonPath);

            string roadMarkJsonFile = roadMarkJsonPath + "HD_ROADMARK.json";
            VULCAN_LOG_INFO(" laneJsonFile path: <{}> ......", roadMarkJsonFile);

            // 解析json文件中数据
            Int32 roadMarkCount = 0;
            if (!DataReader::LoadRoadMarkJson(roadMarkJsonFile, RoadMarkPnts, roadMarkCount))
            {
                VULCAN_LOG_INFO("json in <{}> load error", roadMarkJsonFile); // 为空输出log
                continue;
            }
            allSize = allSize + roadMarkCount;
        }
    }
    closedir(dir);
    return true;
}

bool DataReader::LoadTrafficSignJson(std::string _filename, Array<TrafficSignObj> &arrTrafficSignObj)
{
    std::ifstream ifs(_filename);
    if (!ifs.is_open())
    {
        VULCAN_LOG_INFO("json filePath has no json file..."); // 为空输出log
        return false;
    }
    Json::Value root;
    JSONCPP_STRING errs;
    Json::CharReaderBuilder readerBuilder;
    if (!Json::parseFromStream(readerBuilder, ifs, &root, &errs))
    {
        VULCAN_LOG_INFO("Func<{}>: {} can't parse", "LoadJson", _filename); // 初始化错误输出log
        return false;
    }
    int nCount = root.size();
    for (Json::Value::ArrayIndex i = 0; i < root.size(); i++)
    {
        // 创建标牌对象
        TrafficSignObj newTrafficSignObj;
        newTrafficSignObj.entity_id = i;
        int nsize = root[i].size();
        if (nsize < 3)
            continue;
        for (int j = 0; j < nsize; ++j)
        {
            Json::Value tem = root[i][j];

            // 读取点坐标
            Coordinate newVec3;
            newVec3.x = tem[0].asFloat();
            newVec3.y = tem[1].asFloat();
            newVec3.z = tem[2].asFloat();

            newTrafficSignObj.plygonPnts.Add(newVec3);
        }
        newTrafficSignObj.plygonPnts.Add(newTrafficSignObj.plygonPnts[0]);
        arrTrafficSignObj.Add(newTrafficSignObj);
    }
    return true;
}

bool DataReader::LoadAllTrafficSignJson(std::string strTilePath, std::string _tileID, Array<TrafficSignObj> &arrTrafficSignObjs)
{
    // trafficsign文件夹路径
    string::size_type nops1 = strTilePath.find_last_of('/');
    string file1 = strTilePath.substr(0, nops1);
    string::size_type nops2 = file1.find_last_of('/');
    string file2 = file1.substr(0, nops2);
    postfix(file2);
    string trafficsign_path = file2 + "trafficsign";
    postfix(trafficsign_path);

    // 读取子目录
    DIR *dir = opendir(trafficsign_path.c_str());
    if (dir == NULL)
    {
        VULCAN_LOG_INFO(" opendir tilepath <{}> is error......", strTilePath); // 为空输出log
        return false;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_DIR)
            continue;

        // 只读文件
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        string JsonFile = trafficsign_path + entry->d_name;
        VULCAN_LOG_INFO(" laneJsonFile path: <{}> ......", JsonFile);

        // 解析json文件中数据
        if (!DataReader::LoadTrafficSignJson(JsonFile, arrTrafficSignObjs))
        {
            VULCAN_LOG_INFO("json in <{}> load error", JsonFile); // 为空输出log
            continue;
        }
    }
    closedir(dir);
    return true;
}
// CDataSqlite::CDataSqlite()
//         :m_pSqliteDB(NULL), m_pcache(NULL), m_stmt(NULL)
//{
// }
// CDataSqlite::~CDataSqlite()
//{
//
// }
//
// bool CDataSqlite::Open(const char *_filename)//���ļ�
//{
//     if (m_pSqliteDB != NULL)
//         Close();
//
////    int nRes = sqlite3_open(_filename, &m_pSqliteDB);
////    if (nRes != SQLITE_OK)
////    {
////        printf("Open database fail:\n ", sqlite3_errmsg(m_pSqliteDB));
////        return false;
////    }
//
//    if(sqlite3_open_v2(_filename, &m_pSqliteDB, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK)
//    {
//        printf("Open database fail:\n ", sqlite3_errmsg(m_pSqliteDB));
//        sqlite3_close(m_pSqliteDB);
//        return false;
//    }
//
//    m_pcache = spatialite_alloc_connection();
//    spatialite_init_ex(m_pSqliteDB, m_pcache, 0);
//    return true;
//}
//
// void CDataSqlite::Close()//�ر��ļ�
//{
//    if (m_pSqliteDB != NULL)
//    {
//        sqlite3_close(m_pSqliteDB);
//        m_pSqliteDB = NULL;
//    }
//    spatialite_cleanup_ex(m_pcache);
//}
//
// bool CDataSqlite::read(LineObj& _HDLane)
//{
//    read(_HDLane.m_id, 0);
//    read(_HDLane.m_lane_edge_id, 1);
//    read(_HDLane.m_prev_id, 2);
//    read(_HDLane.m_next_id, 3);
//    read(_HDLane.m_lane_edge_sequence, 4);
//    read_geo(_HDLane.m_linePnts, 5);
//    return true;
//}
//
// bool CDataSqlite::execSql(const char *_sql)
//{
//    char *pszErrMsg = NULL;
//    int rc = sqlite3_exec(m_pSqliteDB, _sql, NULL, NULL, &pszErrMsg);
//
//    if (rc != SQLITE_OK)
//    {
//        /*String errMsg = pszErrMsg;
//        Engine::Log::PrintRelateZMsg::PrintToFile(errMsg);*/
//        const char *errmsg = sqlite3_errmsg(m_pSqliteDB);
//        printf("sexecSql error: %s\n", errmsg);
//
//        sqlite3_free(pszErrMsg);
//        return false;
//    }
//
//    return true;
//}
//
// int CDataSqlite::getFeatureCount()
//{
//    return sqlite3_column_count(m_stmt);
//}
//
/////////////////
// bool CDataSqlite::read_start(const char *_sql)
//{
//     const char *errmsg = NULL;
////    int nRes = sqlite3_prepare(m_pSqliteDB, _sql, -1, &m_stmt, &errmsg);
//    int nRes = sqlite3_prepare_v2(m_pSqliteDB, _sql, -1, &m_stmt, nullptr);
//    if (nRes != SQLITE_OK)
//    {
//        //String errMsg = sqlite3_errmsg(hDB);
//        //	Engine::Log::PrintRelateZMsg::PrintToFile(errMsg);
//        printf(" run sql (%s)  fail:%s\n ", _sql, sqlite3_errmsg(m_pSqliteDB));
//        clear_note();
//        return false;
//    }
//    return read_next();
//
//}
// bool CDataSqlite::read_end()
//{
//    clear_note();
//    return true;
//}
// void  CDataSqlite::read(bool &_cValue, const int _nID)
//{
//    _cValue = sqlite3_column_int(m_stmt, _nID);
//}
// void  CDataSqlite::read(char &_cValue, const int _nID)//sqlite3_column_int
//{
//    _cValue = sqlite3_column_int(m_stmt, _nID);
//}
// void  CDataSqlite::read(int &_nValue, const int _nID)//sqlite3_column_int
//{
//    _nValue = sqlite3_column_int(m_stmt, _nID);
//}
// void  CDataSqlite::read(int64_t &_nValue, const int _nID)//sqlite3_column_int64
//{
//    _nValue = sqlite3_column_int64(m_stmt, _nID);
//}
// void  CDataSqlite::read(float &_fValue, const int _nID)//sqlite3_column_double
//{
//    _fValue = (float)sqlite3_column_double(m_stmt, _nID);
//}
// void  CDataSqlite::read(double &_fValue, const int _nID)//sqlite3_column_double
//{
//    _fValue = sqlite3_column_double(m_stmt, _nID);
//}
// void  CDataSqlite::read(char* &_sValue, const int _nID)//sqlite3_column_text
//{
//    int nBytes = sqlite3_column_bytes(m_stmt, _nID);
//
//    _sValue = new char[nBytes + 1];
//    memset(_sValue, 0, nBytes + 1);
//    char *pstr = (char *)sqlite3_column_text(m_stmt, _nID);
//    memcpy(_sValue, pstr, nBytes);
//}
// void  CDataSqlite::read(string &_sValue, const int _nID)//sqlite3_column_text
//{
//    char *pstr = NULL;
//    read(pstr, _nID);
//    if (pstr != NULL)
//    {
//        _sValue = pstr;
//        delete[] pstr;
//    }
//}
//
// bool  CDataSqlite::read_geo(std::vector<Vec3>& _pGeo, const int _nID)//��ȡ������Ϣ
//{
//    int nBytes = sqlite3_column_bytes(m_stmt, _nID);
//    unsigned char* wkb = NULL;
//    //wkb = sqlite3_column_text(m_stmt, _nID);
//    wkb = (unsigned char*)sqlite3_column_blob(m_stmt, _nID);
//
//
//    gaiaGeomCollStruct* geom = gaiaFromSpatiaLiteBlobWkb(wkb, nBytes);
//    SpatialiteHelper::geomToCoords(geom, _pGeo);
//    gaiaFreeGeomColl(geom);
//    if (_pGeo.empty())
//    {
//        return false;
//    }
//    return true;
//}
//
//
// bool CDataSqlite::read_next()
//{
//    return  (sqlite3_step(m_stmt) == SQLITE_ROW);
//}
//////////////////
//
// void CDataSqlite::clear_note()
//{
//    if (m_stmt != NULL)
//    {
//        sqlite3_finalize(m_stmt);
//        m_stmt = NULL;
//    }
//}