//
//
//
#include "CommonUtil.h"
#include "HDLane.h"
#include "LineObj.h"
#include "Log.h"
#include <map>
#include <list>
#include "proj_api.h"
#include "Geometries/Coordinate.h"
#include "Geometries/LineString.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Algorithm/Line.h"
// #include <pcl/features/normal_3d.h>
// #include <pcl/filters/project_inliers.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/register_point_struct.h>
// #include <pcl/kdtree/kdtree_flann.h>

using namespace hdmap_build;
using namespace std;
using namespace Engine::Algorithm;
using namespace Engine::Geometries;
using namespace Engine::Base;

CommonUtil::CommonUtil()
{
}
CommonUtil::~CommonUtil()
{
}

void CommonUtil::WriteToTxt(Array<Coordinate> &vecPnts, std::string filePath, std::string name, bool utm)
{
    string dirPath = filePath;
    if (dirPath.back() != '/' || dirPath.back() != '\\')
    {
        dirPath += "/";
    }

    string objFullPath = dirPath + name + ".txt";

    ofstream ofs(objFullPath);
    if (!ofs.is_open())
    {
        cout << "obj 保存不成功" << endl;
        return;
    }

    vector<Vec3> lineVec;
    for (auto j = 0; j < vecPnts.GetCount(); j++)
    {
        Coordinate curCrood = vecPnts[j];
        Vec3 coor;
        coor.x = curCrood.x;
        coor.y = curCrood.y;
        coor.z = curCrood.z;
        if (j == 0)
        {
            VULCAN_LOG_INFO("ori coordinate: <{}> <{}> <{}>......", coor.x, coor.y, coor.z);
        }

        if (!utm)
        {
            coor.x = coor.x * DEG_TO_RAD;
            coor.y = coor.y * DEG_TO_RAD;

            projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
            projPJ g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
            pj_transform(g_pWGS84, g_utm, 1, 1, &coor.x, &coor.y, &coor.z);
            if (j == 0)
            {
                VULCAN_LOG_INFO("trans coordinate: <{}> <{}> <{}>......", coor.x, coor.y, coor.z);
            }
        }
        ofs << std::fixed;
        ofs << coor.x << " " << coor.y << " " << coor.z << endl;
        lineVec.push_back(coor);
    }

    ofs.close();
    return;
}

void CommonUtil::WriteToOBJ(std::vector<LineObj> &vecLanes, std::string filePath, std::string frameid, bool utm)
{
    string dirPath = filePath;
    if (dirPath.back() != '/' || dirPath.back() != '\\')
    {
        dirPath += "/";
    }

    string objFullPath = dirPath + frameid + ".obj";

    ofstream ofs(objFullPath);
    if (!ofs.is_open())
    {
        cout << "obj 保存不成功" << endl;
        return;
    }

    vector<vector<Vec3>> lineVec;
    for (auto currentLane : vecLanes)
    {
        vector<Vec3> curVec;
        VULCAN_LOG_INFO("lane id <{}> ......", currentLane.m_id);
        VULCAN_LOG_INFO("lane lane_edge_id <{}> ......", currentLane.m_lane_edge_id);

        for (auto j = 0; j < currentLane.m_lineCroods.GetCount(); j++)
        {
            Coordinate curCrood = (currentLane.m_lineCroods)[j];
            Vec3 coor;
            coor.x = curCrood.x;
            coor.y = curCrood.y;
            coor.z = curCrood.z;

            if (j == 0)
            {
                VULCAN_LOG_INFO("start ori coordinate: <{}> <{}> <{}>......", coor.x, coor.y, coor.z);
            }

            if (!utm)
            {
                coor.x = coor.x * DEG_TO_RAD;
                coor.y = coor.y * DEG_TO_RAD;

                projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
                projPJ g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
                pj_transform(g_pWGS84, g_utm, 1, 1, &coor.x, &coor.y, &coor.z);

                if (j == 0)
                {
                    VULCAN_LOG_INFO("start trans coordinate: <{}> <{}> <{}>......", coor.x, coor.y, coor.z);
                }
            }
            ofs << std::fixed;
            ofs << "v " << coor.x << " " << coor.y << " " << coor.z << endl;
            curVec.push_back(coor);
        }
        lineVec.push_back(curVec);
    }

    int K = 0;
    for (auto i = 0; i < lineVec.size(); i++)
    {
        if (lineVec[i].size() <= 1)
        {
            continue;
        }
        ofs << "l ";
        for (auto j = 0; j < lineVec[i].size(); j++)
        {
            ofs << ++K << " ";
        }
        ofs << endl;
    }

    ofs.close();
    return;
}

void CommonUtil::WriteToOBJ(Array<LineString *> &vecLanes, std::string filePath, std::string frameid, bool utm)
{
    if (vecLanes.IsEmpty())
        return;
    Array<Array<Coordinate>> arrArrCoords;
    for (int i = 0; i < vecLanes.GetCount(); ++i)
    {
        Array<Coordinate> arrCroods;
        EngineLinestringToVec3Points(vecLanes[i], arrCroods);
        arrArrCoords.Add(arrCroods);
    }
    WriteToOBJ(arrArrCoords, filePath, frameid, utm);
}

void CommonUtil::WriteToOBJ(std::vector<std::vector<LineObj>> &vecLanes, std::string strTilePath, std::string tileid, bool utm)
{
    string dirPath = strTilePath;
    if (dirPath.back() != '/' || dirPath.back() != '\\')
    {
        dirPath += "/";
    }

    std::vector<LineObj> resCHDLaneVec;
    for (int i = 0; i < vecLanes.size(); ++i)
    {
        // 排序
        std::sort(vecLanes[i].begin(), vecLanes[i].end(), [](const LineObj &p1, const LineObj &p2)
                  { return p1.m_ave_sequence < p2.m_ave_sequence; });

        for (int j = 0; j < vecLanes[i].size(); ++j)
        {
            resCHDLaneVec.push_back(vecLanes[i][j]);
        }
    }
    WriteToOBJ(resCHDLaneVec, strTilePath, tileid, utm);
}

void CommonUtil::WriteToOBJ(Array<Array<Coordinate>> &vecLanes, std::string strTilePath, std::string name, bool utm)
{
    string dirPath = strTilePath;
    if (dirPath.back() != '/' || dirPath.back() != '\\')
    {
        dirPath += "/";
    }

    string objFullPath = dirPath + name + ".obj";

    ofstream ofs(objFullPath);
    if (!ofs.is_open())
    {
        cout << "obj 保存不成功" << endl;
        return;
    }

    vector<vector<Vec3>> lineVec;
    for (int i = 0; i < vecLanes.GetCount(); i++)
    {
        vector<Vec3> curVec;
        for (auto j = 0; j < vecLanes[i].GetCount(); j++)
        {
            Coordinate curCrood = vecLanes[i][j];
            Vec3 coor;
            coor.x = curCrood.x;
            coor.y = curCrood.y;
            coor.z = curCrood.z;

            if (!utm)
            {
                coor.x = coor.x * DEG_TO_RAD;
                coor.y = coor.y * DEG_TO_RAD;

                projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
                projPJ g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
                pj_transform(g_pWGS84, g_utm, 1, 1, &coor.x, &coor.y, &coor.z);

                if (j == 0)
                {
                    VULCAN_LOG_INFO("start trans coordinate: <{}> <{}> <{}>......", coor.x, coor.y, coor.z);
                }
            }
            ofs << std::fixed;
            ofs << "v " << coor.x << " " << coor.y << " " << coor.z << endl;
            curVec.push_back(coor);
        }
        lineVec.push_back(curVec);
    }

    int K = 0;
    for (auto i = 0; i < lineVec.size(); i++)
    {
        if (lineVec[i].size() <= 1)
        {
            continue;
        }
        ofs << "l ";
        for (auto j = 0; j < lineVec[i].size(); j++)
        {
            ofs << ++K << " ";
        }
        ofs << endl;
    }

    ofs.close();
    return;
}

double CommonUtil::GetLength(const Array<Coordinate> &arrPnts)
{
    Double dLength = 0.0;

    Int32 nPntCount = arrPnts.GetCount();
    nPntCount -= 1;

    for (Int32 i = 0; i < nPntCount; i++)
    {
        dLength += arrPnts[i].DistanceXY(arrPnts[i + 1]);
    }

    return dLength;
}

LineString *CommonUtil::CoordsToLineString(Array<Coordinate> &inputPoints)
{
    if (inputPoints.GetCount() < 2)
    {
        return nullptr;
    }

    Array<Coordinate *> *newCoords = new Array<Coordinate *>();
    for (int i = 0; i < inputPoints.GetCount(); ++i)
    {
        Coordinate *newPoint = new Coordinate();
        newPoint->x = inputPoints[i].x;
        newPoint->y = inputPoints[i].y;
        newPoint->z = inputPoints[i].z;
        newCoords->Add(newPoint);
    }
    LineString *newLine = new LineString(newCoords);
    return newLine;
}

void CommonUtil::EngineLinestringToVec3Points(const LineString *line, Array<Coordinate> &resCoords)
{
    resCoords.Clear();
    vector<Vec3> resPoints;
    if (line == NULL)
    {
        return;
    }

    for (int i = 0; i < line->GetCoordinates()->GetCount(); i++)
    {
        Coordinate temCrood;
        temCrood.x = line->GetCoordinateN(i)->x;
        temCrood.y = line->GetCoordinateN(i)->y;
        temCrood.z = line->GetCoordinateN(i)->z;
        resCoords.Add(temCrood);
    }
}

Coordinate CommonUtil::GetAveragePt(const Array<Coordinate> &arrCloudPts)
{
    Coordinate resultPt;
    if (arrCloudPts.IsEmpty())
    {
        return resultPt;
    }

    Double sumX = 0, sumY = 0, sumZ = 0;
    Int32 nCount = arrCloudPts.GetCount();

    for (Int32 i = 0; i < nCount; i++)
    {
        sumX += arrCloudPts[i].x;
        sumY += arrCloudPts[i].y;
        sumZ += arrCloudPts[i].z;
    }

    resultPt.x = sumX / nCount;
    resultPt.y = sumY / nCount;
    resultPt.z = sumZ / nCount;

    return resultPt;
}

void CommonUtil::TransGet(Array<Coordinate> &CloudPoints, Coordinate &pntOrign)
{
    int nPointCount = CloudPoints.GetCount();

    if (nPointCount == 0)
        return;

    pntOrign = CloudPoints[0];

    for (int i = 0; i < nPointCount; i++)
    {
        CloudPoints[i] = CloudPoints[i] - pntOrign;
    }
}

void CommonUtil::TransSub(const Coordinate &pntOrign, Array<Coordinate> &arrPoint)
{
    int nPointCount = arrPoint.GetCount();

    if (nPointCount == 0)
        return;

    for (int i = 0; i < nPointCount; i++)
    {
        arrPoint[i] = arrPoint[i] - pntOrign;
    }
}

void CommonUtil::TransSub(const Coordinate &pntOrign, Array<Array<Coordinate>> &arrPoint)
{
    for (int i = 0; i < arrPoint.GetCount(); i++)
    {
        TransSub(pntOrign, arrPoint[i]);
    }
}

void CommonUtil::TransPlus(const Coordinate &pntOrign, Array<Array<Coordinate>> &arrEntrePoints)
{
    Int32 nLineCount = arrEntrePoints.GetCount();
    if (nLineCount == 0)
    {
        return;
    }

    for (Int32 i = 0; i < nLineCount; i++)
    {
        Array<Coordinate> tempLine = arrEntrePoints[i];

        for (Int32 j = 0; j < arrEntrePoints[i].GetCount(); j++)
        {
            arrEntrePoints[i][j] = arrEntrePoints[i][j] + pntOrign;
        }
    }
}

void CommonUtil::TransPlus(const Coordinate &pntOrign, Array<Coordinate> &arrPoint)
{
    Int32 nPointCount = arrPoint.GetCount();

    if (nPointCount == 0)
        return;

    for (Int32 i = 0; i < nPointCount; i++)
    {
        arrPoint[i] += pntOrign;
    }
}

void CommonUtil::FilterPtByRadius(Array<Coordinate> &arrCloudPts, double searchRadius, double filterDensity, bool autoCalDensity)
{
    if (arrCloudPts.IsEmpty())
        return;

    if (searchRadius < 0)
        return;

    //    //包装pcl中的Cloud
    //    int nCount = arrCloudPts.GetCount();
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    cloud->resize(nCount);
    //
    //    for (size_t i = 0; i < nCount; ++i)
    //    {
    //        cloud->points[i].x = arrCloudPts[i].x;
    //        cloud->points[i].y = arrCloudPts[i].y;
    //        cloud->points[i].z = arrCloudPts[i].z;
    //    }
    //    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //    kdtree.setInputCloud(cloud);
    //
    //    #pragma omp parallel for
    //    for (Int32 i = nCount - 1; i >= 0; i--)
    //    {
    //        pcl::PointXYZ searchPoint;
    //        searchPoint.x = arrCloudPts[i].x;
    //        searchPoint.y = arrCloudPts[i].y;
    //        searchPoint.z = arrCloudPts[i].z;
    //        vector<int> pointIdxRadiusSearch;
    //        vector<float> pointRadiusSquaredDistance;
    //        float radiusSearch = searchRadius;
    //        bool isDelete = true;
    //        if (kdtree.radiusSearch(searchPoint, radiusSearch,
    //                                pointIdxRadiusSearch, pointRadiusSquaredDistance)>0)
    //        {
    //            if(pointIdxRadiusSearch.size() > filterDensity)
    //                isDelete = false;
    //        }
    //
    //        if(isDelete)
    //            arrCloudPts.Delete(i);
    //    }

    // 采用网格法提高效率
    RoadTopoGrid ptsRoadTopoGrid;
    ptsRoadTopoGrid.BuildTopoGrid(arrCloudPts, searchRadius);

    // 首先统计点领域内搜索到的点个数的平均值
    int nCount = arrCloudPts.GetCount();
    int allSearchCount = 0;
    for (Int32 i = nCount - 1; i >= 0; i--)
    {
        Array<Int32> pointIdxRadiusSearch;
        ptsRoadTopoGrid.GetMatchInfos(arrCloudPts[i], arrCloudPts, pointIdxRadiusSearch, searchRadius);
        allSearchCount = allSearchCount + pointIdxRadiusSearch.GetCount();
    }

    if (autoCalDensity)
    {
        double averageSize = allSearchCount / nCount;
        filterDensity = 0.1 * averageSize;
    }

    for (Int32 i = nCount - 1; i >= 0; i--)
    {
        Array<Int32> pointIdxRadiusSearch;
        ptsRoadTopoGrid.GetMatchInfos(arrCloudPts[i], arrCloudPts, pointIdxRadiusSearch, searchRadius);
        if (pointIdxRadiusSearch.GetCount() < filterDensity)
        {
            arrCloudPts.Delete(i);
        }
    }
}

vector<Vec3> CommonUtil::EngineCroodToVec3(Array<Coordinate> arrPnts)
{
    vector<Vec3> newVec;
    for (int i = 0; i < arrPnts.GetCount(); ++i)
    {
        Vec3 curVec3;
        curVec3.x = arrPnts[i].x;
        curVec3.y = arrPnts[i].y;
        curVec3.z = arrPnts[i].z;
        newVec.push_back(curVec3);
    }
    return newVec;
}

Array<Coordinate> CommonUtil::Vec3CroodToEngine(vector<Vec3> arrPnts)
{
    Array<Coordinate> newCroods;
    for (int i = 0; i < arrPnts.size(); ++i)
    {
        Coordinate curCrood;
        curCrood.x = arrPnts[i].x;
        curCrood.y = arrPnts[i].y;
        curCrood.z = arrPnts[i].z;
        newCroods.Add(curCrood);
    }
    return newCroods;
}

vector<Vec3> CommonUtil::Utm2Wgs84(vector<Vec3> oriVecVec3)
{
    vector<Vec3> resPoints;
    if (oriVecVec3.empty())
    {
        return resPoints;
    }

    for (int i = 0; i < oriVecVec3.size(); i++)
    {
        Vec3 tempVec3;
        tempVec3.x = oriVecVec3[i].x;
        tempVec3.y = oriVecVec3[i].y;
        tempVec3.z = oriVecVec3[i].z;

        // 转换坐标
        projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
        projPJ g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
        pj_transform(g_utm, g_pWGS84, 1, 1, &tempVec3.x, &tempVec3.y, &tempVec3.z);
        tempVec3.x = tempVec3.x * RAD_TO_DEG;
        tempVec3.y = tempVec3.y * RAD_TO_DEG;
        resPoints.push_back(tempVec3);
    }

    return resPoints;
}

void CommonUtil::Utm2Wgs84(Array<Coordinate> &oriVecVec3)
{
    if (oriVecVec3.IsEmpty())
    {
        return;
    }

    for (int i = 0; i < oriVecVec3.GetCount(); i++)
    {
        Coordinate tempVec3;
        tempVec3.x = oriVecVec3[i].x;
        tempVec3.y = oriVecVec3[i].y;
        tempVec3.z = oriVecVec3[i].z;

        // 转换坐标
        projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
        projPJ g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
        pj_transform(g_utm, g_pWGS84, 1, 1, &tempVec3.x, &tempVec3.y, &tempVec3.z);
        oriVecVec3[i].x = tempVec3.x * RAD_TO_DEG;
        oriVecVec3[i].y = tempVec3.y * RAD_TO_DEG;
    }
}

void CommonUtil::Wgs84toUtm(Array<Coordinate> &oriVecVec3)
{
    if (oriVecVec3.IsEmpty())
    {
        return;
    }

    for (int i = 0; i < oriVecVec3.GetCount(); i++)
    {
        Coordinate coor;
        coor.x = oriVecVec3[i].x;
        coor.y = oriVecVec3[i].y;
        coor.z = oriVecVec3[i].z;

        coor.x = coor.x * DEG_TO_RAD;
        coor.y = coor.y * DEG_TO_RAD;

        projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
        projPJ g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
        pj_transform(g_pWGS84, g_utm, 1, 1, &coor.x, &coor.y, &coor.z);
        oriVecVec3[i].x = coor.x;
        oriVecVec3[i].y = coor.y;
    }
}

vector<Vec3> CommonUtil::Wgs84toUtm(vector<Vec3> oriVecVec3)
{
    vector<Vec3> resVecVec3;
    Array<Coordinate> temp = Vec3CroodToEngine(std::move(oriVecVec3));
    Wgs84toUtm(temp);
    resVecVec3 = EngineCroodToVec3(temp);
    return resVecVec3;
}

bool CommonUtil::Resample(Array<Coordinate> &coordinates, Double tolerance)
{
    if ((coordinates.GetCount() <= 0) || (tolerance <= 0.0))
    {
        return false;
    }

    Int32 nCount = coordinates.GetCount();
    UChar *pID = new UChar[nCount];

    if (pID == NULL)
    {
        return false;
    }

    memset(pID, 0, nCount);

    // 2020.11.17�������β���غϣ���ɾ��β��
    if (nCount > 2 && coordinates[0].x == coordinates[nCount - 1].x && coordinates[0].y == coordinates[nCount - 1].y && coordinates[0].z == coordinates[nCount - 1].z)
    {
        // ɾ��β��
        coordinates.Delete(nCount - 1);

        // ������ȥ1
        nCount--;
    }

    pID[0] = 1;
    pID[nCount - 1] = 1;

    std::list<SizeT> lstActiveID;
    lstActiveID.push_back(0);
    lstActiveID.push_back(nCount - 1);

    Int32 nFromID, nToID, nMaxDistID;
    Double dMaxDist, dDistToLine;

    Int32 i = 0;
    list<SizeT>::iterator iterSE;
    list<SizeT>::iterator iterInsert;

    while (lstActiveID.size() > 1)
    {
        nMaxDistID = 0;
        dMaxDist = 0.0;
        iterSE = lstActiveID.begin();
        nFromID = (Int32)*iterSE;
        iterSE++;
        nToID = (Int32)*iterSE;

        for (i = nFromID + 1; i < nToID; i++)
        {
            dDistToLine = BaseAlgorithm::DistancePtToLine(&(coordinates[i]), &(coordinates[nFromID]), &(coordinates[nToID]));

            if (dMaxDist < dDistToLine)
            {
                dMaxDist = dDistToLine;
                nMaxDistID = i;
            }
        }

        if (dMaxDist > tolerance)
        {
            pID[nMaxDistID] = 1;
            iterInsert = lstActiveID.begin();
            iterInsert++;
            lstActiveID.insert(iterInsert, nMaxDistID);
        }
        else
        {
            lstActiveID.pop_front();
        }
    }

    // ȷ����β�㲻�ᱻɾ��
    pID[0] = 1;
    pID[nCount - 1] = 1;

    for (i = (int)nCount - 1; i >= 0; i--)
    {
        if (pID[i] == 0)
        {
            coordinates.Delete(i);
        }
    }

    if (pID != NULL)
    {
        delete[] pID;
        pID = NULL;
    }

    return true;
}

void CommonUtil::RemoveDuplicatePoints(Base::Array<Geometries::Coordinate> &vecInput, const Base::Double dTolerance)
{
    auto num = vecInput.GetCount();
    if (num < 2 || dTolerance < 0)
        return;

    for (int i = 0; i < num - 1;)
    {
        Double distance = vecInput[i].Distance(vecInput[i + 1]);
        if (distance < dTolerance)
        {
            // ��������̫�������󱻳�ϡ��1����
            if (vecInput.GetCount() > 2)
            {
                vecInput.Delete(i + 1);
                num--;
            }
            else
                break;
        }
        else
        {
            ++i;
        }
    }
}

bool CommonUtil::getInterpolationLine(Engine::Base::Array<Engine::Geometries::Coordinate> &coordinates, double interval, int save_ori_pts)
{
    if (coordinates.GetCount() < 2)
    {
        return false;
    }
    double totalLength = 0.0f;
    for (int i = 0; i < coordinates.GetCount() - 1; i++)
    {
        totalLength += coordinates[i].Distance(coordinates[i + 1]);
    }
    if (totalLength <= 0.0f)
    {
        return false;
    }
    double length;
    float thresholld = interval;
    Engine::Geometries::Coordinate startPoint, endPoint, point;
    Engine::Base::Array<Engine::Geometries::Coordinate> preLine = coordinates;
    Engine::Base::Array<Engine::Geometries::Coordinate> newLine;
    if (save_ori_pts)
    {
        newLine.Add(coordinates[0]);
        for (int i = 0; i < preLine.GetCount() - 1; i++)
        {
            startPoint = preLine[i];
            endPoint = preLine[i + 1];
            length = startPoint.Distance(endPoint);
            if (length < interval)
            {
                newLine.Add(endPoint);
                continue;
            }
            int radio = int(length / interval);
            double dis = 0.0;
            for (int j = 1; j <= radio; ++j)
            {
                dis = j * interval;
                if (fabs(length - dis) < 0.5)
                    continue;
                point.x = startPoint.x + (endPoint.x - startPoint.x) * (dis) / length;
                point.y = startPoint.y + (endPoint.y - startPoint.y) * (dis) / length;
                point.z = startPoint.z + (endPoint.z - startPoint.z) * (dis) / length;
                newLine.Add(point);
            }
            newLine.Add(endPoint);
        }
    }
    else
    {
        newLine.Add(coordinates[0]);
        for (int i = 0; i < preLine.GetCount() - 1;)
        {
            startPoint = preLine[i];
            endPoint = preLine[i + 1];
            length = startPoint.Distance(endPoint);
            if (length < thresholld)
            {
                thresholld -= length;
                preLine.Delete(i);
                continue;
            }
            point.x = startPoint.x + (endPoint.x - startPoint.x) * (thresholld) / length;
            point.y = startPoint.y + (endPoint.y - startPoint.y) * (thresholld) / length;
            point.z = startPoint.z + (endPoint.z - startPoint.z) * (thresholld) / length;
            newLine.Add(point);
            preLine[i] = point;
            thresholld = interval;
        }
    }
    if (newLine[newLine.GetCount() - 1].Distance(coordinates[coordinates.GetCount() - 1]) > 1.0E-5)
    {
        newLine.Add(coordinates[coordinates.GetCount() - 1]);
    }
    else
    {
        newLine[newLine.GetCount() - 1] = coordinates[coordinates.GetCount() - 1];
    }
    coordinates = newLine;
    return true;
}

double CommonUtil::LongestLine(Array<Array<Coordinate>> &arrLines, Array<Coordinate> &longest)
{
    double disLength = -1.0;
    for (int i = 0; i < arrLines.GetCount(); ++i)
    {
        if (arrLines[i].GetCount() < 2)
            continue;

        double temLen = GetLength(arrLines[i]);
        if (temLen > disLength)
        {
            temLen = disLength;
            longest = arrLines[i];
        }
    }
    return disLength;
}
const int kTileLevel = 15;

int CommonUtil::WGS_to_tile_ID(double x, double y, double z)
{
    int ix = (int)(x / (90. / (1 << 30)));
    int iy = (int)(y / (90. / (1 << 30)));
    int tile_id = GetBit(ix, 31);
    for (int i = 30; i > (30 - kTileLevel); --i)
    {
        tile_id <<= 2;
        tile_id |= GetBit(iy, i) << 1 | GetBit(ix, i);
    }
    return tile_id;
}

bool CommonUtil::get_tile_center_WGS(int tile_id, double &wgsx, double &wgsy)
{
    int lx = 0;
    int ly = 0;

    for (int i = (kTileLevel - 1); i >= 0; --i)
    {
        lx <<= 1;
        lx |= GetBit(tile_id, i * 2);
        ly <<= 1;
        ly |= GetBit(tile_id, i * 2 + 1);
    }

    wgsx = (lx + 0.5) * 180. / (1 << kTileLevel);
    wgsy = (ly + 0.5) * 180. / (1 << kTileLevel);
    return true;
}

bool CommonUtil::get_tile_box_WGS(int tile_id, double &minx, double &miny, double &maxx, double &maxy)
{
    double cx = 0;
    double cy = 0;
    if (!get_tile_center_WGS(tile_id, cx, cy))
    {
        return false;
    }
    minx = cx - 0.5 * 180. / (1 << kTileLevel);
    miny = cy - 0.5 * 180. / (1 << kTileLevel);
    maxx = cx + 0.5 * 180. / (1 << kTileLevel);
    maxy = cy + 0.5 * 180. / (1 << kTileLevel);
    return true;
}

void CommonUtil::Tile_box_obj(string tilePath, int tile_id)
{
    if (tilePath.empty())
    {
        return;
    }

    double minx = 0.0, miny = 0.0, maxx = 0.0, maxy = 0.0;
    get_tile_box_WGS(tile_id, minx, miny, maxx, maxy);
    LineObj boxLine;
    boxLine.m_lineCroods.Add(Coordinate(minx, miny, 0.0)); // 左下
    boxLine.m_lineCroods.Add(Coordinate(minx, maxy, 0.0)); // 左上
    boxLine.m_lineCroods.Add(Coordinate(maxx, maxy, 0.0)); // 右上
    boxLine.m_lineCroods.Add(Coordinate(maxx, miny, 0.0)); // 右下
    boxLine.m_lineCroods.Add(Coordinate(minx, miny, 0.0)); // 左下

    Wgs84toUtm(boxLine.m_lineCroods);
    vector<LineObj> arrLineObj;
    arrLineObj.push_back(boxLine);

    string name = to_string(tile_id) + "_box";
    WriteToOBJ(arrLineObj, tilePath, name, true);
}