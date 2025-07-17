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
#include "Geometries/IRCreatLaneAlgorithm.h"

#include "./include/CommonUtil.h"
#include "./include/DataManager.h"
#include "./include/RoadLayer.h"
#include "./include/RoadMark.h"
#include "./include/RoadTopoBuild.h"
#include "./include/LinearObjDuplicateRemoval.h"
#include "../hdmap_server/data-access-engine/data_access_engine.h"
#include "MathUtils.h"
#include "AfterProcessStopLine.h"
#include "afterProcessCommon.h"
#include "afterProcess.h"
AfterProcessStopLine::AfterProcessStopLine()
{
}

AfterProcessStopLine::~AfterProcessStopLine()
{
    /*for(LaneGroupPolygon* lgPolygon : _vecLgGroup) {
        delete lgPolygon;
        lgPolygon = NULL;
    }
    _vecLgGroup.clear();*/
}

bool AfterProcessStopLine::process()
{
    setFeatureIndex();
    filterByJunctionPt();
    filterByStopLine();

    return true;
}

void AfterProcessStopLine::filterByJunctionPt()
{
    getJunctionStopLine();

    for (int i = 0; i < _junctionPtArray.GetCount(); i++)
    {
        auto itor = _mapJunctionStop.find(i);
        if (itor == _mapJunctionStop.end())
        {
            continue;
        }

        Coordinate junctionPt = _junctionPtArray[i];
        vector<RefLink *> vecInLink;
        vector<RefLink *> vecOutLink;
        getJunctionLink(junctionPt, vecInLink, vecOutLink);

        vector<RoadMapping::RM> vecFilteredStopLine;
        getStopLineByLaneGroup(junctionPt, itor->second, vecFilteredStopLine, vecInLink, vecOutLink);
        getStopLineByLink(junctionPt, vecFilteredStopLine, vecInLink, vecOutLink);
        addResultStopLine();

        for (RefLink *line : vecInLink)
        {
            delete line;
        }
        for (RefLink *line : vecOutLink)
        {
            delete line;
        }
    }
}
void AfterProcessStopLine::filterByStopLine()
{
    getFilterStopLine();
    for (int i = 0; i < _filterCoordArray.size(); i++)
    {
        auto itor = _mapFilterStop.find(i);
        if (itor == _mapFilterStop.end())
        {
            continue;
        }
        Coordinate filterCoord = _filterCoordArray[i];
        vector<RefLink *> vecInLink;
        vector<RefLink *> vecOutLink;
        getJunctionLink(filterCoord, vecInLink, vecOutLink);

        vector<RoadMapping::RM> vecFilteredStopLine;
        getStopLineByLaneGroup(filterCoord, itor->second, vecFilteredStopLine, vecInLink, vecOutLink);
        getStopLineByLink(filterCoord, vecFilteredStopLine, vecInLink, vecOutLink);
        addResultStopLine();

        for (RefLink *line : vecInLink)
        {
            delete line;
        }
        for (RefLink *line : vecOutLink)
        {
            delete line;
        }
    }
}
void AfterProcessStopLine::getStopLineByLaneGroup(Coordinate &junctionPt, vector<RoadMapping::RM> &vecStopLine,
                                                  vector<RoadMapping::RM> &vecFilteredStopLine, vector<RefLink *> &vecInLink,
                                                  vector<RefLink *> &vecOutLink)
{
    vector<LaneGroupPolygon *> vecLaneGroup;
    getLaneGroupPolygon(junctionPt, vecLaneGroup);
    for (int i = 0; i < vecStopLine.size(); i++)
    {
        RoadMapping::RM rm = vecStopLine[i];
        if (rm.ObjectType != 8)
        {
            continue;
        }
        double length = MathUtils::caculatePointDistance(&rm.plygonPnts[0], &rm.plygonPnts[1]);
        if (length > 21)
        {
            continue;
        }
        if (!checkStopLineInLaneGroup(rm, vecLaneGroup))
        {
            vecFilteredStopLine.push_back(rm);
            continue;
        }
        bool allIntersect = false;
        RefLink *refLink = getStopLineIntersectLink(rm, vecInLink, vecOutLink, allIntersect);
        if (allIntersect == true)
        {
            vecFilteredStopLine.push_back(rm);
            continue;
        }
        if (refLink == NULL)
        {
            _vecNoLinkRm.push_back(rm);
            continue;
        }
        checkMutableStopLine(refLink, rm);
        //_resultStopLineArray.Add(rm);
    }
    for (LaneGroupPolygon *lgPolygon : vecLaneGroup)
    {
        delete lgPolygon;
        lgPolygon = NULL;
    }
    vecLaneGroup.clear();
}

void AfterProcessStopLine::getStopLineByLink(Coordinate &junctionPt, vector<RoadMapping::RM> &vecStopLine,
                                             vector<RefLink *> &vecInLink, vector<RefLink *> &vecOutLink)
{

    vector<RoadMapping::RM> vecFilterd;
    for (RoadMapping::RM &stopLine : vecStopLine)
    {
        if (!checkStopLineByLink(stopLine, vecInLink, vecOutLink, vecFilterd))
        {
            continue;
        }
        //_resultStopLineArray.Add(stopLine);
    }

    for (RoadMapping::RM &stopLine : vecFilterd)
    {
        checkExpendStopLineByLink(stopLine, vecInLink, vecOutLink);
    }
}

void AfterProcessStopLine::addResultStopLine()
{
    set<int> insertedSet;
    for (auto itor = _mapLinkStopLine.begin(); itor != _mapLinkStopLine.end(); itor++)
    {
        for (int i = 0; i < itor->second.GetCount(); i++)
        {
            if (insertedSet.find(itor->second[i].label) == insertedSet.end())
            {
                _resultStopLineArray.Add(itor->second[i]);
                insertedSet.insert(itor->second[i].label);
            }
        }
    }
    _mapLinkStopLine.clear();
    //  _mapRoadStopLine.clear();
    for (auto rm : _vecNoLinkRm)
    {
        if (insertedSet.find(rm.label) == insertedSet.end())
        {
            _resultStopLineArray.Add(rm);
            insertedSet.insert(rm.label);
        }
    }
    _vecNoLinkRm.clear();
}

void AfterProcessStopLine::getFilterStopLine()
{
    int filterIndex = -1;
    for (int j = 0; j < _stopLineArray.GetCount(); j++)
    {
        RoadMapping::RM &rm = _stopLineArray[j];
        if (rm.ObjectType != 8)
        {
            continue;
        }
        if (checkedRMSet.find(j) != checkedRMSet.end())
        {
            continue;
        }
        if (_filterCoordArray.size() == 0)
        {
            _filterCoordArray.push_back(rm.plygonPnts[0]);
            _mapFilterStop[0].push_back(rm);
            continue;
        }
        Coordinate &stopCoord = rm.plygonPnts[0];
        bool contains = false;
        for (int i = 0; i < _filterCoordArray.size(); i++)
        {
            double dis = BaseAlgorithm::DistancePtToPt(&stopCoord, &_filterCoordArray[i]);
            if (dis < 50)
            {
                _mapFilterStop[i].push_back(rm);
                checkedRMSet.insert(j);
                contains = true;
                break;
            }
        }
        if (!contains)
        {
            _filterCoordArray.push_back(rm.plygonPnts[0]);
            _mapFilterStop[_filterCoordArray.size() - 1].push_back(rm);
        }
    }
}

bool AfterProcessStopLine::checkStopLineInLaneGroup(RoadMapping::RM &stopLine, std::vector<LaneGroupPolygon *> &vecLaneGroup)
{
    LineString *line = new LineString();
    Base::Array<Coordinate *> *coordArray = line->GetCoordinates();
    LaneGroupPolygon *laneGroupPoly = NULL;
    for (int i = 0; i < stopLine.plygonPnts.GetCount(); i++)
    {
        Coordinate *coord = new Coordinate(stopLine.plygonPnts[i]);
        coordArray->Add(coord);
    }
    for (LaneGroupPolygon *lgPolygon : vecLaneGroup)
    {
        if (GeometryAlgorithm::Intersects(line, lgPolygon->polygon))
        {
            Array<LineString *> inPolygonLine;
            if (!IRCreatLaneAlgorithm::InterceptLinesInPolygon(lgPolygon->polygon, line, inPolygonLine))
            {
                continue;
            }
            if (inPolygonLine.GetCount() == 0)
            {
                continue;
            }
            double inLength = inPolygonLine[0]->GetLength();
            double allLength = line->GetLength();
            double percent = inLength / allLength;
            if (percent < 0.4)
            {
                continue;
            }
            laneGroupPoly = lgPolygon;
            break;
        }
    }
    if (laneGroupPoly == NULL)
    {
        return false;
    }

    Coordinate leftCoord;
    Coordinate rightCoord;
    int leftIndex;
    int rightIndex;
    if (!BaseAlgorithm::GetNearestPntToLineset(stopLine.plygonPnts[0], laneGroupPoly->leftBoundary.linePts, leftCoord, leftIndex) ||
        !BaseAlgorithm::GetNearestPntToLineset(stopLine.plygonPnts[0], laneGroupPoly->rightBoundary.linePts, rightCoord, rightIndex))
    {
        return false;
    }
    double leftDis = BaseAlgorithm::DistancePtToPt(&stopLine.plygonPnts[0], &leftCoord);
    double rightDis = BaseAlgorithm::DistancePtToPt(&stopLine.plygonPnts[0], &rightCoord);

    Engine::Base::Array<Engine::Geometries::Coordinate> &boundaryCoordArray = leftDis < rightDis ? laneGroupPoly->leftBoundary.linePts : laneGroupPoly->rightBoundary.linePts;
    int index = leftDis < rightDis ? leftIndex : rightIndex;

    double angle = MathUtils::getTwoVectorAngle(&boundaryCoordArray[index], &boundaryCoordArray[index + 1],
                                                &stopLine.plygonPnts[0], &stopLine.plygonPnts[1]);
    if (angle > 80 && angle < 100)
    {
        delete line;
        return true;
    }
    delete line;
    return false;
}

bool AfterProcessStopLine::checkStopLineByLink(RoadMapping::RM &stopLineRm, vector<RefLink *> &vecInLink,
                                               vector<RefLink *> &vecOutLink, vector<RoadMapping::RM> &vecFiltered)
{
    bool allIntersect = false;
    RefLink *interLine = getStopLineIntersectLink(stopLineRm, vecInLink, vecOutLink, allIntersect);
    if (interLine == NULL)
    {
        vecFiltered.push_back(stopLineRm);
        return false;
    }
    Coordinate coord;
    int index = 0;
    if (!BaseAlgorithm::GetNearestPntToLineset(&stopLineRm.plygonPnts[0], interLine->line->GetCoordinates(),
                                               coord, index))
    {
        return false;
    }
    double angle = MathUtils::getTwoVectorAngle(interLine->line->GetCoordinateN(index), interLine->line->GetCoordinateN(index + 1),
                                                &stopLineRm.plygonPnts[0], &stopLineRm.plygonPnts[1]);

    if (angle > 60 && angle < 120)
    {
        checkMutableStopLine(interLine, stopLineRm);
        return true;
    }

    return false;
}

bool AfterProcessStopLine::checkExpendStopLineByLink(RoadMapping::RM &stopLineRm, vector<RefLink *> &vecInLink,
                                                     vector<RefLink *> &vecOutLink)
{
    bool allIntersect = false;
    RefLink *interLine = getExpendStopLineIntersectLink(stopLineRm, vecInLink, vecOutLink, allIntersect);
    if (interLine == NULL)
    {

        return false;
    }
    Coordinate coord;
    int index = 0;
    if (!BaseAlgorithm::GetNearestPntToLineset(&stopLineRm.plygonPnts[0], interLine->line->GetCoordinates(),
                                               coord, index))
    {
        return false;
    }
    double angle = MathUtils::getTwoVectorAngle(interLine->line->GetCoordinateN(index), interLine->line->GetCoordinateN(index + 1),
                                                &stopLineRm.plygonPnts[0], &stopLineRm.plygonPnts[1]);

    Engine::Base::Array<Engine::Geometries::Coordinate> vecCoord;
    vecCoord.Add(stopLineRm.plygonPnts[0]);
    vecCoord.Add(stopLineRm.plygonPnts[1]);

    vector<double> t_utm_world;
    t_utm_world.push_back(334032);
    t_utm_world.push_back(3472256);
    t_utm_world.push_back(0);
    RoadMapping::afterProcess::UtmToWgsAdd(t_utm_world, 51, vecCoord);
    if (angle > 60 && angle < 120)
    {
        auto itor = _mapRoadStopLine.find(interLine->roadIndex);
        if (itor == _mapRoadStopLine.end())
        {
            _mapLinkStopLine[interLine->index].Add(stopLineRm);
            return true;
        }
        checkExpendMutableStopLine(interLine, stopLineRm);
        return true;
    }

    return false;
}

RefLink *AfterProcessStopLine::getStopLineIntersectLink(RoadMapping::RM &stopLineRm, vector<RefLink *> &vecInLink,
                                                        vector<RefLink *> &vecOutLink, bool &allIntersect)
{

    LineString *stopLine = createLineString(stopLineRm.plygonPnts);
    bool outIntersect = false;
    RefLink *interLine = NULL;
    int linkIndex = 0;
    for (RefLink *linkLine : vecOutLink)
    {
        if (GeometryAlgorithm::Intersects(stopLine, linkLine->line))
        {
            interLine = linkLine;
            outIntersect = true;
            linkIndex++;
            break;
        }
        linkIndex++;
    }

    Coordinate endCoord = MathUtils::calculatePointOnRay(&stopLineRm.plygonPnts[0], &stopLineRm.plygonPnts[1], 5);
    Coordinate startCoord = MathUtils::calculatePointOnRay(&stopLineRm.plygonPnts[1], &stopLineRm.plygonPnts[0], 5);
    Array<Coordinate> coordArray;
    coordArray.Add(startCoord);
    coordArray.Add(endCoord);
    LineString *expendStopLine = createLineString(coordArray);
    bool inIntersect = false;
    for (RefLink *linkLine : vecInLink)
    {
        if (GeometryAlgorithm::Intersects(expendStopLine, linkLine->line))
        {
            interLine = linkLine;
            inIntersect = true;
            break;
        }
        linkIndex++;
    }
    if (outIntersect && inIntersect)
    {
        allIntersect = true;
        delete stopLine;
        delete expendStopLine;
        return NULL;
    }
    delete stopLine;
    delete expendStopLine;
    return interLine;
}

RefLink *AfterProcessStopLine::getExpendStopLineIntersectLink(RoadMapping::RM &stopLineRm, vector<RefLink *> &vecInLink,
                                                              vector<RefLink *> &vecOutLink, bool &allIntersect)
{
    Coordinate endCoord = MathUtils::calculatePointOnRay(&stopLineRm.plygonPnts[0], &stopLineRm.plygonPnts[1], 5);
    Coordinate startCoord = MathUtils::calculatePointOnRay(&stopLineRm.plygonPnts[1], &stopLineRm.plygonPnts[0], 5);
    Array<Coordinate> coordArray;
    coordArray.Add(startCoord);
    coordArray.Add(endCoord);
    LineString *expendStopLine = createLineString(coordArray);

    bool outIntersect = false;
    RefLink *interLine = NULL;
    int linkIndex = 0;
    for (RefLink *linkLine : vecOutLink)
    {
        if (GeometryAlgorithm::Intersects(expendStopLine, linkLine->line))
        {
            interLine = linkLine;
            outIntersect = true;
            linkIndex++;
            break;
        }
        linkIndex++;
    }

    bool inIntersect = false;
    for (RefLink *linkLine : vecInLink)
    {
        if (GeometryAlgorithm::Intersects(expendStopLine, linkLine->line))
        {
            interLine = linkLine;
            inIntersect = true;
            break;
        }
        linkIndex++;
    }

    if (outIntersect && inIntersect)
    {
        allIntersect = true;
        delete expendStopLine;
        return interLine;
    }

    delete expendStopLine;
    return interLine;
}

void AfterProcessStopLine::checkMutableStopLine(RefLink *link, RoadMapping::RM &rm)
{
    if (_mapLinkStopLine.find(link->index) == _mapLinkStopLine.end())
    {
        _mapLinkStopLine[link->index].Add(rm);
        _mapRoadStopLine[link->roadIndex].Add(rm);
        return;
    }
    double rmLength = MathUtils::caculatePointDistance(&rm.plygonPnts[0], &rm.plygonPnts[1]);
    Array<RoadMapping::RM> &stopLineArray = _mapLinkStopLine[link->index];
    bool added = false;
    for (int i = 0; i < stopLineArray.GetCount(); i++)
    {
        RoadMapping::RM &exitRM = stopLineArray[i];
        double dis = MathUtils::caculatePoit2Line(&rm.plygonPnts[0], &exitRM.plygonPnts[0], &exitRM.plygonPnts[1]);
        double exitDis = MathUtils::caculatePoit2Line(&exitRM.plygonPnts[0], &rm.plygonPnts[0], &rm.plygonPnts[1]);
        if (dis > 8 || exitDis > 8)
        {
            continue;
        }
        double exitLength = MathUtils::caculatePointDistance(&exitRM.plygonPnts[0], &exitRM.plygonPnts[1]);
        Coordinate foot1 = MathUtils::caculateFootOnSegement(&rm.plygonPnts[0], &exitRM.plygonPnts[0],
                                                             &exitRM.plygonPnts[1]);
        Coordinate foot2 = MathUtils::caculateFootOnSegement(&rm.plygonPnts[1], &exitRM.plygonPnts[0],
                                                             &exitRM.plygonPnts[1]);
        double footLength = MathUtils::caculatePointDistance(&foot1, &foot2);
        double rmPercent = footLength / rmLength;
        double exitPercent = footLength / exitLength;

        if (rmPercent > 0.8 || exitPercent > 0.8)
        {
            if (exitLength > rmLength)
            {
                stopLineArray.Delete(i);
                stopLineArray.Add(rm);
                _mapRoadStopLine[link->roadIndex].Add(rm);
            }
            added = true;
            break;
        }
    }
    if (!added)
    {
        stopLineArray.Add(rm);
        _mapRoadStopLine[link->roadIndex].Add(rm);
    }
}

void AfterProcessStopLine::checkExpendMutableStopLine(RefLink *link, RoadMapping::RM &rm)
{
    if (_mapLinkStopLine.find(link->index) == _mapLinkStopLine.end())
    {
        _mapLinkStopLine[link->index].Add(rm);
        _mapRoadStopLine[link->roadIndex].Add(rm);
        return;
    }
    double rmLength = MathUtils::caculatePointDistance(&rm.plygonPnts[0], &rm.plygonPnts[1]);
    Array<RoadMapping::RM> &stopLineArray = _mapLinkStopLine[link->index];
    bool added = false;
    for (int i = 0; i < stopLineArray.GetCount(); i++)
    {
        RoadMapping::RM &exitRM = stopLineArray[i];
        double dis = MathUtils::caculatePoit2Line(&rm.plygonPnts[0], &exitRM.plygonPnts[0], &exitRM.plygonPnts[1]);
        double exitDis = MathUtils::caculatePoit2Line(&exitRM.plygonPnts[0], &rm.plygonPnts[0], &rm.plygonPnts[1]);
        if (dis > 10 && exitDis > 10)
        {
            continue;
        }
        added = true;
    }
    if (!added)
    {
        stopLineArray.Add(rm);
        _mapRoadStopLine[link->roadIndex].Add(rm);
    }
}

bool AfterProcessStopLine::checkStopLineInLink(RoadMapping::RM &stopLineRm, vector<LineString *> &vecInLink, bool outIntersect)
{

    Coordinate endCoord = MathUtils::calculatePointOnRay(&stopLineRm.plygonPnts[0], &stopLineRm.plygonPnts[1], 3);
    Coordinate startCoord = MathUtils::calculatePointOnRay(&stopLineRm.plygonPnts[1], &stopLineRm.plygonPnts[0], 3);
    Array<Coordinate> coordArray;
    coordArray.Add(startCoord);
    coordArray.Add(endCoord);
    LineString *stopLine = createLineString(coordArray);
    for (LineString *linkLine : vecInLink)
    {
        if (GeometryAlgorithm::Intersects(stopLine, linkLine))
        {
            if (outIntersect)
            {
                delete stopLine;
                return false;
            }
            Coordinate coord;
            int index = 0;
            if (!BaseAlgorithm::GetNearestPntToLineset(stopLine->GetCoordinateN(0), linkLine->GetCoordinates(), coord, index))
            {
                continue;
            }
            double angle = MathUtils::getTwoVectorAngle(linkLine->GetCoordinateN(index), linkLine->GetCoordinateN(index + 1),
                                                        stopLine->GetCoordinateN(0), stopLine->GetCoordinateN(1));
            if (angle > 70 && angle < 110)
            {
                delete stopLine;
                return true;
            }
        }
    }

    delete stopLine;
    return false;
}

void AfterProcessStopLine::getLaneGroupPolygon(Coordinate &junctionPt, std::vector<LaneGroupPolygon *> &vecLaneGroup)
{

    for (int i = 0; i < _laneGroupArray.GetCount(); i++)
    {
        RoadMapping::LG &laneGroup = _laneGroupArray[i];
        RoadMapping::LB &leftBoundary = laneGroup.laneGroupLBs[0];
        RoadMapping::LB &rightBoundary = laneGroup.laneGroupLBs[laneGroup.laneGroupLBs.GetCount() - 1];
        if (!checkBoundaryToJunction(leftBoundary, junctionPt))
        {
            continue;
        }
        // RoadMapping::RM rm;
        LinearRing *linearRing = new LinearRing();
        Base::Array<Coordinate *> *coordArray = linearRing->GetCoordinates();
        Coordinate leftStart = MathUtils::calculatePointOnRay(&leftBoundary.linePts[1], &leftBoundary.linePts[0], 3);
        coordArray->Add(new Coordinate(leftStart));
        // rm.plygonPnts.Add(leftStart);
        int leftSize = leftBoundary.linePts.GetCount();
        for (int i = 0; i < leftSize; i++)
        {
            Coordinate *coord = new Coordinate(leftBoundary.linePts[i]);
            coordArray->Add(coord);
            // rm.plygonPnts.Add(leftBoundary.linePts[i]);
        }
        Coordinate leftEnd = MathUtils::calculatePointOnRay(&leftBoundary.linePts[leftSize - 2], &leftBoundary.linePts[leftSize - 1], 3);
        coordArray->Add(new Coordinate(leftEnd));
        // rm.plygonPnts.Add(leftEnd);
        int rightSize = rightBoundary.linePts.GetCount();
        Coordinate rightEnd = MathUtils::calculatePointOnRay(&rightBoundary.linePts[rightSize - 2], &rightBoundary.linePts[rightSize - 1], 3);
        coordArray->Add(new Coordinate(rightEnd));
        //  rm.plygonPnts.Add(rightEnd);
        for (int i = rightSize - 1; i > -1; i--)
        {
            Coordinate *coord = new Coordinate(rightBoundary.linePts[i]);
            coordArray->Add(coord);
            //    rm.plygonPnts.Add(rightBoundary.linePts[i]);
        }
        Coordinate rightStart = MathUtils::calculatePointOnRay(&rightBoundary.linePts[1], &rightBoundary.linePts[0], 3);
        coordArray->Add(new Coordinate(rightStart));
        //   rm.plygonPnts.Add(rightStart);

        coordArray->Add(new Coordinate(leftStart));
        //    rm.plygonPnts.Add( leftStart);
        /*Engine::Base::Array<Engine::Geometries::Coordinate> vecCoord;
         vecCoord.Add(leftStart);
         vecCoord.Add(rightStart);
         vector<double> t_utm_world;
         t_utm_world.push_back(455964);
         t_utm_world.push_back(4418561);
         t_utm_world.push_back(0);
         RoadMapping::afterProcess::UtmToWgsAdd(t_utm_world,50,vecCoord);*/
        Polygon *polygon = new Polygon(linearRing);
        LaneGroupPolygon *lgPolygon = new LaneGroupPolygon();
        lgPolygon->polygon = polygon;
        lgPolygon->laneGroup = laneGroup;
        lgPolygon->leftBoundary = leftBoundary;
        lgPolygon->rightBoundary = rightBoundary;
        vecLaneGroup.push_back(lgPolygon);
        //_resultStopLineArray.Add(rm);
        //_vecLgGroup.push_back(lgPolygon);
    }
}

void AfterProcessStopLine::getJunctionStopLine()
{
    int size = _junctionPtArray.GetCount();
    for (int i = 0; i < size; i++)
    {
        Coordinate &junctionPt = _junctionPtArray[i];
        for (int j = 0; j < _stopLineArray.GetCount(); j++)
        {
            RoadMapping::RM &rm = _stopLineArray[j];
            if (rm.ObjectType != 8)
            {
                continue;
            }
            if (checkedRMSet.find(j) != checkedRMSet.end())
            {
                continue;
            }
            Coordinate &stopCoord = rm.plygonPnts[0];
            double dis = BaseAlgorithm::DistancePtToPt(&stopCoord, &junctionPt);
            if (dis < 50)
            {
                _mapJunctionStop[i].push_back(rm);
                checkedRMSet.insert(j);
            }
        }
    }
}
bool AfterProcessStopLine::checkBoundaryToJunction(RoadMapping::LB &boundary, Coordinate &junctionPt)
{
    Coordinate startCoord = boundary.linePts.GetAt(0);
    Coordinate endCoord = boundary.linePts.GetAt(boundary.linePts.GetCount() - 1);

    double startDis = BaseAlgorithm::DistancePtToPt(&startCoord, &junctionPt);
    double endDis = BaseAlgorithm::DistancePtToPt(&junctionPt, &endCoord);

    return /*startDis > endDis &&*/ startDis < 60 || endDis < 60;
}

void AfterProcessStopLine::getJunctionLink(Coordinate &junctionPt, std::vector<RefLink *> &vecInLink,
                                           std::vector<RefLink *> &vecOutLink)
{
    vector<RefLink *> vecJuncLink;
    for (int i = 0; i < _refLinkArray.GetCount(); i++)
    {
        RoadMapping::LB &link = _refLinkArray[i];
        Coordinate startCoord = link.linePts.GetAt(0);
        Coordinate endCoord = link.linePts.GetAt(link.linePts.GetCount() - 1);

        double startDis = BaseAlgorithm::DistancePtToPt(&startCoord, &junctionPt);
        double endDis = BaseAlgorithm::DistancePtToPt(&junctionPt, &endCoord);
        if (startDis > 60 && endDis > 60)
        {
            continue;
        }
        int ptSize = link.linePts.GetCount();
        Coordinate startEx = MathUtils::calculatePointOnRay(&link.linePts[1], &link.linePts[0], 5);
        Coordinate endEx = MathUtils::calculatePointOnRay(&link.linePts[ptSize - 2], &link.linePts[ptSize - 1], 5);
        Array<Coordinate> coordArray;
        coordArray.Add(startEx);
        coordArray.Add(link.linePts);
        coordArray.Add(endEx);
        if (startDis > endDis)
        {
            RefLink *refLink = new RefLink();
            LineString *line = createLineString(coordArray);
            refLink->line = line;
            refLink->index = i;
            refLink->inLink = true;
            vecInLink.push_back(refLink);
            vecJuncLink.push_back(refLink);
        }
        else
        {
            RefLink *refLink = new RefLink();
            LineString *line = createLineString(coordArray);
            refLink->line = line;
            refLink->index = i;
            vecOutLink.push_back(refLink);
            vecJuncLink.push_back(refLink);
        }
    }
    int index = 0;
    set<int> checkedSet;
    for (int i = 0; i < vecJuncLink.size(); i++)
    {
        if (checkedSet.find(i) != checkedSet.end())
        {
            continue;
        }
        RefLink *link = vecJuncLink[i];
        _mapJunctionLink[index].push_back(link);
        link->roadIndex = index;
        checkedSet.insert(i);
        for (int j = 0; j < vecJuncLink.size(); j++)
        {
            if (checkedSet.find(j) != checkedSet.end())
            {
                continue;
            }
            RefLink *otherLink = vecJuncLink[j];
            if (link->inLink == otherLink->inLink)
            {
                continue;
            }
            RefLink *inLink = NULL;
            RefLink *outLink = NULL;
            if (link->inLink)
            {
                inLink = link;
                outLink = otherLink;
            }
            else
            {
                inLink = otherLink;
                outLink = link;
            }

            int inSize = inLink->line->GetCoordinates()->GetCount();
            const Coordinate *coord1 = inLink->line->GetCoordinateN(inSize - 1);
            const Coordinate *coord2 = inLink->line->GetCoordinateN(inSize - 2);

            const Coordinate *coord3 = outLink->line->GetCoordinateN(0);
            const Coordinate *coord4 = outLink->line->GetCoordinateN(1);
            double dis = MathUtils::caculatePoit2Line(coord1, coord3, coord4);
            if (dis > 20)
            {
                continue;
            }
            double angle = MathUtils::getTwoVectorAngle(coord1, coord2, coord3, coord4);
            if (angle > 20)
            {
                continue;
            }
            _mapJunctionLink[index].push_back(otherLink);
            otherLink->roadIndex = index;
            checkedSet.insert(j);
        }
        index++;
    }
}

LineString *AfterProcessStopLine::createLineString(Array<Coordinate> &coordArray)
{
    LineString *line = new LineString();
    Base::Array<Coordinate *> *lineCoord = line->GetCoordinates();
    LaneGroupPolygon *laneGroupPoly = NULL;
    for (int i = 0; i < coordArray.GetCount(); i++)
    {
        Coordinate *coord = new Coordinate(coordArray[i]);
        lineCoord->Add(coord);
    }
    return line;
}

void AfterProcessStopLine::setFeatureIndex()
{
    for (int i = 0; i < _stopLineArray.GetCount(); i++)
    {
        _stopLineArray[i].label = i;
    }
    /*for(int i = 0; i < _refLinkArray.GetCount(); i++){
        _refLinkArray[i].index = i;
    }*/
}