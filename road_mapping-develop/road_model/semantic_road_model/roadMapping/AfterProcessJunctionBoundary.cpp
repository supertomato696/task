//
//
//

#include "Base/Types.h"
#include "afterProcess.h"
#include "json.hpp"
#include "Utils.h"
#include "pclPtType.h"
#include "Geometries/LineString.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/GeometryAlgorithm.h"

#include <list>
#include "./include/CommonUtil.h"
#include "./include/DataManager.h"
#include "./include/RoadLayer.h"
#include "./include/RoadMark.h"
#include "./include/RoadTopoBuild.h"
#include "./include/LinearObjDuplicateRemoval.h"
#include "../hdmap_server/data-access-engine/data_access_engine.h"
#include "AfterProcessJunctionBoundary.h"
#include "MathUtils.h"
#include "./include/CommonUtil.h"
AfterProcessJunctionBoundary::AfterProcessJunctionBoundary()
{
}

AfterProcessJunctionBoundary::~AfterProcessJunctionBoundary()
{
}

bool AfterProcessJunctionBoundary::process()
{
    generateByJunctionPt();
    generateByCrossWalk();
    return true;
}

void AfterProcessJunctionBoundary::generateByJunctionPt()
{
    getJunctionCrossWalk();
    int size = _junctionPtArray.GetCount();
    for (int i = 0; i < size; i++)
    {
        auto itor = _mapJunctionCrossWalk.find(i);
        if (itor == _mapJunctionCrossWalk.end())
        {
            continue;
        }
        Coordinate &junctionPt = _junctionPtArray[i];
        std::list<RoadMapping::RM> lstAntiClock;
        sortAntiClockByAngle(itor->second, lstAntiClock, junctionPt);
        generateJunctionBoundaryWithLeftUp(lstAntiClock);
    }
}

void AfterProcessJunctionBoundary::generateByCrossWalk()
{
    getGroupCrossWalk();
    int size = _filterCoordArray.size();
    for (int i = 0; i < size; i++)
    {
        auto itor = _mapGroupCrossWalk.find(i);
        if (itor == _mapGroupCrossWalk.end())
        {
            continue;
        }
        Coordinate junctionPt = getJunctionPt(itor->second);
        std::list<RoadMapping::RM> lstAntiClock;
        sortAntiClockByAngle(itor->second, lstAntiClock, junctionPt);
        generateJunctionBoundaryWithoutLeftUp(lstAntiClock, junctionPt);
    }
}

void AfterProcessJunctionBoundary::sortAntiClockByAngle(vector<RoadMapping::RM> &vecCrossWalk, std::list<RoadMapping::RM> &lstAntiClock,
                                                        Coordinate &junctionPt)
{
    vector<Coordinate> vecWalkPt;
    int walkSize = vecCrossWalk.size();
    vecWalkPt.resize(walkSize);
    for (int i = 0; i < walkSize; i++)
    {
        RoadMapping::RM &rm = vecCrossWalk[i];
        Coordinate coord = getFilterCoordinate(rm.plygonPnts);
        vecWalkPt[i] = coord;
    }
    std::list<double> lstAngle;
    for (int i = 0; i < vecCrossWalk.size(); i++)
    {
        if (vecCrossWalk[i].ObjectType != 10)
        {
            continue;
        }
        double angle = getClockAngle(&vecWalkPt[i], junctionPt);
        if (lstAngle.size() == 0)
        {
            lstAngle.push_back(angle);
            lstAntiClock.push_back(vecCrossWalk[i]);
            continue;
        }
        auto itor = lstAntiClock.begin();
        int index = 0;
        for (auto angleItor = lstAngle.begin(); angleItor != lstAngle.end(); angleItor++, itor++)
        {
            if (angle < *angleItor)
            {
                lstAntiClock.insert(itor, vecCrossWalk[i]);
                lstAngle.insert(angleItor, angle);
                break;
            }
            if (index == lstAntiClock.size() - 1)
            {
                lstAngle.push_back(angle);
                lstAntiClock.push_back(vecCrossWalk[i]);
                break;
            }
            index++;
        }
    }
}

void AfterProcessJunctionBoundary::generateJunctionBoundary(std::list<RoadMapping::RM> &lstAntiClock)
{

    RoadMapping::LB junctionBoundary;
    for (RoadMapping::RM &rm : lstAntiClock)
    {
        /*int size = junctionBoundary.linePts.GetCount();
        int rmPtSize = rm.plygonPnts.GetCount();
        bool addFirst = true;
        if( size > 2 && rmPtSize > 2){
            Coordinate coord1 = junctionBoundary.linePts[size - 3];
            Coordinate coord2 = junctionBoundary.linePts[size - 2];

            Coordinate coord3 = rm.plygonPnts[1];
            Coordinate coord4 = rm.plygonPnts[2];

            double angle = MathUtils::getTwoVectorAngle(&coord1,&coord2,&coord3,&coord4);
            if(angle < 20 ){
                junctionBoundary.linePts.Delete(size - 1);
                addFirst = false;
            }
        }*/
        /* if(addFirst){
             junctionBoundary.linePts.Add(rm.plygonPnts[0]);
         }*/

        int crossCount = 0;
        for (int i = 0; i < rm.plygonPnts.GetCount() - 2; i++)
        {
            Coordinate &coord1 = rm.plygonPnts[i];
            Coordinate &coord2 = rm.plygonPnts[i + 1];
            Coordinate &coord3 = rm.plygonPnts[i + 2];
            double angle = BaseAlgorithm::ComputeAngle(coord1, coord2, coord3);
            angle = angle * 180 / PI;
            junctionBoundary.linePts.Add(coord2);

            if (abs(angle - 90) < 10)
            {

                crossCount++;
            }
            /*if(i == rm.plygonPnts.GetCount() - 3){
                junctionBoundary.linePts.Add(coord3);
            }*/
            if (crossCount == 2)
            {
                break;
            }
        }
        /* vector<double> t_utm_world;
         t_utm_world.push_back(455964);
         t_utm_world.push_back(4418561);
         t_utm_world.push_back(0);
         RoadMapping::afterProcess::UtmToWgsAdd(t_utm_world,50,coordArray);*/
        int ss = 0;
    }
    /* int size = junctionBoundary.linePts.GetCount();
     bool addLast = true;
     if( size > 2 ){
         Coordinate coord1 = junctionBoundary.linePts[size - 3];
         Coordinate coord2 = junctionBoundary.linePts[size - 2];

         Coordinate coord3 = junctionBoundary.linePts[1];
         Coordinate coord4 = junctionBoundary.linePts[2];

         double angle = MathUtils::getTwoVectorAngle(&coord1,&coord2,&coord3,&coord4);
         if(angle < 20 ){
             junctionBoundary.linePts.Delete(size - 1);
             junctionBoundary.linePts.Delete(0);
         }
     }*/
    if (junctionBoundary.linePts.GetCount() != 0)
    {
        junctionBoundary.linePts.Add(junctionBoundary.linePts[0]);
    }
    // hdmap_build::CommonUtil::WriteToOBJ(junctionBoundary.linePts, _dataPath, "/Lukou/junctionboundary", true);
    _junctionBoundaryArray.Add(junctionBoundary);
}

void AfterProcessJunctionBoundary::generateJunctionBoundaryWithLeftUp(std::list<RoadMapping::RM> &lstAntiClock)
{
    for (RoadMapping::RM &rm : lstAntiClock)
    {
        getExpensionWalk(rm);
    }
    generateJunctionBoundary(lstAntiClock);
}
void AfterProcessJunctionBoundary::generateJunctionBoundaryWithoutLeftUp(std::list<RoadMapping::RM> &lstAntiClock,
                                                                         Coordinate &junctionPt)
{
    std::list<RoadMapping::RM> lstSorted;
    for (RoadMapping::RM &rm : lstAntiClock)
    {
        RoadMapping::RM sortedRm = sortCrossWalk(rm, junctionPt);
        getExpensionWalk(sortedRm);
        lstSorted.push_back(sortedRm);
    }
    generateJunctionBoundary(lstSorted);
}

RoadMapping::RM AfterProcessJunctionBoundary::sortCrossWalk(RoadMapping::RM &rm, Coordinate &junctionPt)
{
    Coordinate coord1 = rm.plygonPnts[0];
    Coordinate coord2 = rm.plygonPnts[1];
    Coordinate coord3 = rm.plygonPnts[2];
    Coordinate coord4 = rm.plygonPnts[3];

    Coordinate centerPt;
    centerPt.x = (coord1.x + coord3.x) / 2;
    centerPt.y = (coord1.y + coord3.y) / 2;
    centerPt.z = (coord1.z + coord3.z) / 2;
    RoadMapping::RM sortedRM;
    sortedRM.ObjectType = 10;
    sortedRM.label = rm.label;
    double dis1 = MathUtils::caculatePointDistance(&coord1, &coord2);
    double dis2 = MathUtils::caculatePointDistance(&coord2, &coord3);
    if (dis1 < dis2)
    {
        double angle = MathUtils::getTwoVectorAngle(&centerPt, &junctionPt, &coord1, &coord2);
        if (angle < 90)
        {
            sortedRM.plygonPnts.Add(coord3);
            sortedRM.plygonPnts.Add(coord4);
            sortedRM.plygonPnts.Add(coord1);
            sortedRM.plygonPnts.Add(coord2);
        }
        else
        {
            sortedRM.plygonPnts.Add(coord1);
            sortedRM.plygonPnts.Add(coord2);
            sortedRM.plygonPnts.Add(coord3);
            sortedRM.plygonPnts.Add(coord4);
        }
    }
    else
    {
        double angle = MathUtils::getTwoVectorAngle(&centerPt, &junctionPt, &coord2, &coord3);
        if (angle < 90)
        {
            sortedRM.plygonPnts.Add(coord4);
            sortedRM.plygonPnts.Add(coord1);
            sortedRM.plygonPnts.Add(coord2);
            sortedRM.plygonPnts.Add(coord3);
        }
        else
        {
            sortedRM.plygonPnts.Add(coord2);
            sortedRM.plygonPnts.Add(coord3);
            sortedRM.plygonPnts.Add(coord4);
            sortedRM.plygonPnts.Add(coord1);
        }
    }
    return sortedRM;
}
void AfterProcessJunctionBoundary::getJunctionCrossWalk()
{
    int size = _junctionPtArray.GetCount();
    for (int i = 0; i < size; i++)
    {
        Coordinate &junctionPt = _junctionPtArray[i];
        for (int j = 0; j < _crossWalkArray.GetCount(); j++)
        {
            RoadMapping::RM &rm = _crossWalkArray[j];
            if (rm.ObjectType != 10)
            {
                continue;
            }
            Coordinate &stopCoord = rm.plygonPnts[0];
            double dis = BaseAlgorithm::DistancePtToPt(&stopCoord, &junctionPt);
            if (dis < 50)
            {
                _mapJunctionCrossWalk[i].push_back(rm);
                checkedRMSet.insert(j);
            }
        }
    }
}

void AfterProcessJunctionBoundary::getGroupCrossWalk()
{
    int filterIndex = -1;
    for (int j = 0; j < _crossWalkArray.GetCount(); j++)
    {
        RoadMapping::RM &rm = _crossWalkArray[j];
        if (rm.ObjectType != 10)
        {
            continue;
        }
        if (checkedRMSet.find(j) != checkedRMSet.end())
        {
            continue;
        }
        if (rm.plygonPnts.GetCount() < 3)
        {
            continue;
        }
        Coordinate filterCoord = getFilterCoordinate(rm.plygonPnts);
        if (_filterCoordArray.size() == 0)
        {
            _filterCoordArray.push_back(filterCoord);
            _mapGroupCrossWalk[0].push_back(rm);
            continue;
        }

        bool contains = false;
        for (int i = 0; i < _filterCoordArray.size(); i++)
        {
            double dis = BaseAlgorithm::DistancePtToPt(&filterCoord, &_filterCoordArray[i]);
            if (dis < 50)
            {
                _mapGroupCrossWalk[i].push_back(rm);
                checkedRMSet.insert(j);
                contains = true;
                break;
            }
        }
        if (!contains)
        {
            _filterCoordArray.push_back(filterCoord);
            _mapGroupCrossWalk[_filterCoordArray.size() - 1].push_back(rm);
        }
    }
}
Coordinate AfterProcessJunctionBoundary::getJunctionPt(std::vector<RoadMapping::RM> &groupCrossWalk)
{
    vector<Coordinate> vecWalkPt;
    int walkSize = groupCrossWalk.size();
    vecWalkPt.resize(walkSize);
    for (int i = 0; i < walkSize; i++)
    {
        RoadMapping::RM &rm = groupCrossWalk[i];
        Coordinate coord = getFilterCoordinate(rm.plygonPnts);
        vecWalkPt[i] = coord;
    }

    Coordinate junctionPt;
    double maxDis = -1;
    for (int i = 0; i < walkSize; i++)
    {

        for (int j = 0; j < walkSize; j++)
        {
            if (i == j)
            {
                continue;
            }
            double dis = MathUtils::caculatePointDistance(&vecWalkPt[i], &vecWalkPt[j]);
            if (maxDis < dis)
            {
                junctionPt.x = (vecWalkPt[i].x + vecWalkPt[j].x) / 2;
                junctionPt.y = (vecWalkPt[i].y + vecWalkPt[j].y) / 2;
                junctionPt.z = (vecWalkPt[i].z + vecWalkPt[j].z) / 2;
                maxDis = dis;
            }
        }
    }
    return junctionPt;
}
Coordinate AfterProcessJunctionBoundary::getFilterCoordinate(Array<Coordinate> &coordArray)
{
    if (coordArray.GetCount() < 3)
    {
        double x = (coordArray[0].x + coordArray[1].x) / 2;
        double y = (coordArray[0].y + coordArray[1].y) / 2;
        return Coordinate(x, y, coordArray[0].z);
    }
    double x = (coordArray[0].x + coordArray[2].x) / 2;
    double y = (coordArray[0].y + coordArray[2].y) / 2;
    return Coordinate(x, y, coordArray[0].z);
}
double AfterProcessJunctionBoundary::getClockAngle(const Coordinate *coord, Coordinate &junctionPt)
{
    Coordinate xPix(junctionPt.x + 1, junctionPt.y, 0);
    double angle = BaseAlgorithm::ComputeAngle(xPix, junctionPt, *coord);
    angle = angle * 180 / PI;
    double detY = coord->y - junctionPt.y;

    if (detY > 0)
    {
        return angle;
    }
    return 360 - angle;
}

void AfterProcessJunctionBoundary::getExpensionWalk(RoadMapping::RM &rm)
{
    Coordinate coord1 = rm.plygonPnts[0];
    Coordinate coord2 = rm.plygonPnts[1];
    Coordinate coord3 = rm.plygonPnts[2];
    Coordinate coord4 = rm.plygonPnts[3];

    rm.plygonPnts.Clear();
    LineString *line1 = new LineString();
    line1->GetCoordinates()->Add(new Coordinate(coord1));
    line1->GetCoordinates()->Add(new Coordinate(coord2));

    LineString *offsetLine1 = GeometryAlgorithm::GenerateOffsetLine(line1, -0.5);

    Coordinate offsetCoord1 = MathUtils::calculatePointOnRay(offsetLine1->GetCoordinates()->ElementAt(0),
                                                             offsetLine1->GetCoordinates()->ElementAt(1),
                                                             0.5);
    rm.plygonPnts.Add(*offsetLine1->GetCoordinates()->ElementAt(0));
    rm.plygonPnts.Add(offsetCoord1);

    LineString *line2 = new LineString();
    line2->GetCoordinates()->Add(new Coordinate(coord3));
    line2->GetCoordinates()->Add(new Coordinate(coord4));
    LineString *offsetLine2 = GeometryAlgorithm::GenerateOffsetLine(line2, -0.5);

    Coordinate offsetCoord2 = MathUtils::calculatePointOnRay(offsetLine2->GetCoordinates()->ElementAt(1),
                                                             offsetLine2->GetCoordinates()->ElementAt(0),
                                                             0.5);

    rm.plygonPnts.Add(offsetCoord2);
    rm.plygonPnts.Add(*offsetLine2->GetCoordinates()->ElementAt(1));

    delete line1;
    delete line2;
    delete offsetLine1;
    delete offsetLine2;
}

void AfterProcessJunctionBoundary::getBufferCrosswalkPackage(const std::vector<RoadMapping::RM> &vecCrossWalk, RoadMapping::LB &junctionBoundary)
{
    Engine::Base::Array<Engine::Geometries::Coordinate> arrCrosswalkPts;
    for (int i = 0; i < vecCrossWalk.size(); i++)
    {
        arrCrosswalkPts.Add(vecCrossWalk[i].plygonPnts);
    }
    if (arrCrosswalkPts.IsEmpty())
    {
        return;
    }
    Engine::Base::Array<Engine::Geometries::Coordinate> arrPackgePoints;
    Engine::Geometries::BaseAlgorithm::PointsPackage(arrCrosswalkPts, arrPackgePoints);
    if (arrPackgePoints.GetCount() < 4)
    {
        return;
    }

    double offsetDis = 0.5;
    Engine::Base::Array<Engine::Geometries::Coordinate> OffsetPlyPts;
    Engine::Base::Array<Engine::Geometries::Coordinate *> *coords = new Base::Array<Geometries::Coordinate *>;
    for (int i = 0; i < arrPackgePoints.GetCount(); ++i)
    {
        coords->Add(new Engine::Geometries::Coordinate(arrPackgePoints[i]));
    }
    Engine::Geometries::LinearRing lr(coords);
    Engine::Geometries::Polygon pPolygon(new Engine::Geometries::LinearRing(lr));

    Engine::Geometries::Polygon *newOffsetPly = new Engine::Geometries::Polygon(pPolygon);
    Engine::Base::Double dis = offsetDis;
    newOffsetPly = Engine::Geometries::GeometryAlgorithm::GenerateOffsetPolygon(&pPolygon, dis);
    if (newOffsetPly != NULL)
    {
        Engine::Base::Array<Engine::Geometries::Coordinate *> *coords = newOffsetPly->GetExteriorRing()->GetCoordinates();
        for (int i = 0; i < coords->GetCount(); ++i)
            OffsetPlyPts.Add(Engine::Geometries::Coordinate(coords->GetAt(i)->x, coords->GetAt(i)->y, coords->GetAt(i)->z));
    }
}