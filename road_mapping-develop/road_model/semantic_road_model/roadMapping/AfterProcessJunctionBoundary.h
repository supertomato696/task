//
//
//

#ifndef LIDAR_MAPPING_AFTERPROCESSJUNCTIONBOUNDARY_H
#define LIDAR_MAPPING_AFTERPROCESSJUNCTIONBOUNDARY_H

#include <list>
#include <vector>
#include "afterProcessCommon.h"

class AfterProcessJunctionBoundary
{
public:
    AfterProcessJunctionBoundary();
    virtual ~AfterProcessJunctionBoundary();
    void setLaneGroupArray(Array<RoadMapping::LG> &lgArray)
    {
        _laneGroupArray = lgArray;
    }
    void setCrossWalkArray(Array<RoadMapping::RM> &crossWalkArray)
    {
        _crossWalkArray = crossWalkArray;
    }
    void setJunctionPt(Array<Coordinate> &coordArray)
    {
        _junctionPtArray = coordArray;
    }
    Array<RoadMapping::LB> &getJunctionBoundaryArray()
    {
        return _junctionBoundaryArray;
    }
    void setDataPath(const string &dataPath)
    {
        _dataPath = dataPath;
    }

public:
    virtual bool process();

private:
    double getClockAngle(const Coordinate *coord, Coordinate &junctionPt);
    void sortAntiClockByAngle(vector<RoadMapping::RM> &vecCrossWalk, std::list<RoadMapping::RM> &lstAntiClock,
                              Coordinate &junctionPt);
    void generateJunctionBoundary(std::list<RoadMapping::RM> &lstAntiClock);
    void getJunctionCrossWalk();

    void generateByJunctionPt();
    void generateByCrossWalk();
    void getGroupCrossWalk();
    Coordinate getFilterCoordinate(Array<Coordinate> &coordArray);
    Coordinate getJunctionPt(std::vector<RoadMapping::RM> &groupCrossWalk);
    void generateJunctionBoundaryWithoutLeftUp(std::list<RoadMapping::RM> &lstAntiClock, Coordinate &junctionPt);
    void generateJunctionBoundaryWithLeftUp(std::list<RoadMapping::RM> &lstAntiClock);
    RoadMapping::RM sortCrossWalk(RoadMapping::RM &rm, Coordinate &junctionPt);

    void getExpensionWalk(RoadMapping::RM &rm);
    void getBufferCrosswalkPackage(const std::vector<RoadMapping::RM> &vecCrossWalk, RoadMapping::LB &junctionBoundary);

private:
    Array<RoadMapping::LG> _laneGroupArray;
    Array<RoadMapping::RM> _crossWalkArray;
    Array<Coordinate> _junctionPtArray;
    Array<RoadMapping::RM> _resultStopLineArray;
    std::map<int, vector<RoadMapping::RM>> _mapJunctionCrossWalk;
    Array<RoadMapping::LB> _junctionBoundaryArray;
    string _dataPath;
    std::set<int> checkedRMSet;
    std::map<int, vector<RoadMapping::RM>> _mapGroupCrossWalk;
    std::vector<Coordinate> _filterCoordArray;
};
#endif // LIDAR_MAPPING_AFTERPROCESSJUNCTIONBOUNDARY_H
