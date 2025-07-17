//
//
//

#ifndef LIDAR_MAPPING_AFTERPROCESSSTOPLINE_H
#define LIDAR_MAPPING_AFTERPROCESSSTOPLINE_H

#include <map>
#include <vector>
#include <set>
struct LaneGroupPolygon
{
    LaneGroupPolygon()
    {
        polygon = NULL;
    }
    virtual ~LaneGroupPolygon()
    {
        delete polygon;
        polygon = NULL;
    }
    Polygon *polygon;
    RoadMapping::LG laneGroup;
    RoadMapping::LB leftBoundary;
    RoadMapping::LB rightBoundary;
};

struct RefLink
{
    RefLink()
    {
        line = NULL;
        inLink = false;
        index = -1;
        roadIndex = -1;
    }
    virtual ~RefLink()
    {
        delete line;
        line = NULL;
    }
    LineString *line;
    int index;
    bool inLink;
    int roadIndex;
};
class AfterProcessStopLine
{
public:
    AfterProcessStopLine();
    virtual ~AfterProcessStopLine();

public:
    virtual bool process();

    void setLaneGroupArray(Array<RoadMapping::LG> &lgArray)
    {
        _laneGroupArray = lgArray;
    }
    void setStopLineArray(Array<RoadMapping::RM> &stopLineArray)
    {
        _stopLineArray = stopLineArray;
    }
    void setJunctionPt(Array<Coordinate> &coordArray)
    {
        _junctionPtArray = coordArray;
    }
    void setRefLink(Array<RoadMapping::LB> &refLinkArray)
    {
        _refLinkArray = refLinkArray;
    }
    Array<RoadMapping::RM> &getResultStopLine()
    {
        return _resultStopLineArray;
    }

private:
    void getLaneGroupPolygon(Coordinate &junctionPt, std::vector<LaneGroupPolygon *> &vecLaneGroup);
    bool checkBoundaryToJunction(RoadMapping::LB &boundary, Coordinate &junctionPt);
    bool checkStopLineInLaneGroup(RoadMapping::RM &stopLine, std::vector<LaneGroupPolygon *> &vecLaneGroup);
    bool checkStopLineByLink(RoadMapping::RM &stopLine, vector<RefLink *> &vecInLink,
                             vector<RefLink *> &vecOutLink, vector<RoadMapping::RM> &vecFiltered);

    bool checkExpendStopLineByLink(RoadMapping::RM &stopLine, vector<RefLink *> &vecInLink,
                                   vector<RefLink *> &vecOutLink);

    bool checkStopLineInLink(RoadMapping::RM &stopLineRm, vector<LineString *> &vecInLink, bool outIntersect);
    void getStopLineByLaneGroup(Coordinate &junctionPt, vector<RoadMapping::RM> &vecStopLine,
                                vector<RoadMapping::RM> &vecFilteredStopLine, vector<RefLink *> &vecInLink,
                                vector<RefLink *> &vecOutLink);

    void checkMutableStopLine(RefLink *link, RoadMapping::RM &rm);
    void checkExpendMutableStopLine(RefLink *link, RoadMapping::RM &rm);
    void getStopLineByLink(Coordinate &junctionPt, vector<RoadMapping::RM> &vecStopLine, vector<RefLink *> &vecInLink,
                           vector<RefLink *> &vecOutLink);
    void getJunctionStopLine();
    void getFilterStopLine();
    void filterByJunctionPt();
    void filterByStopLine();

    void setFeatureIndex();

    RefLink *getStopLineIntersectLink(RoadMapping::RM &stopLine, vector<RefLink *> &vecInLink,
                                      vector<RefLink *> &vecOutLink, bool &allIntersect);

    RefLink *getExpendStopLineIntersectLink(RoadMapping::RM &stopLine, vector<RefLink *> &vecInLink,
                                            vector<RefLink *> &vecOutLink, bool &allIntersect);

    LineString *createLineString(Array<Coordinate> &_coordArray);
    void getJunctionLink(Coordinate &junctionPt, std::vector<RefLink *> &vecInLink,
                         std::vector<RefLink *> &vecOutLink);

    void addResultStopLine();

private:
    Array<RoadMapping::LB> _refLinkArray;
    Array<RoadMapping::LG> _laneGroupArray;
    Array<RoadMapping::RM> _stopLineArray;
    Array<Coordinate> _junctionPtArray;
    // std::vector<LaneGroupPolygon*> _vecLgGroup;
    Array<RoadMapping::RM> _resultStopLineArray;
    std::map<int, vector<RoadMapping::RM>> _mapJunctionStop;
    std::map<int, vector<RoadMapping::RM>> _mapFilterStop;
    std::set<int> checkedRMSet;
    std::vector<Coordinate> _filterCoordArray;
    map<int, Array<RoadMapping::RM>> _mapLinkStopLine;
    vector<RoadMapping::RM> _vecNoLinkRm;
    map<int, vector<RefLink *>> _mapJunctionLink;
    std::map<int, Array<RoadMapping::RM>> _mapRoadStopLine;
};
#endif // LIDAR_MAPPING_AFTERPROCESSSTOPLINE_H
