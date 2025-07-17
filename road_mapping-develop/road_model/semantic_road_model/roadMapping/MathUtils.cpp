//
//
//
#include "Base/Types.h"
#include "afterProcess.h"
#include "json.hpp"
#include "Utils.h"
#include "pclPtType.h"
#include "Geometries/Coordinate.h"
#include "Geometries/LineString.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/GeometryAlgorithm.h"
#include "Geometries/IRCreatLaneAlgorithm.h"
#include "MathUtils.h"

double MathUtils::getTwoVectorAngle(const Coordinate *coord1, const Coordinate *coord2,
                                    const Coordinate *coord3, const Coordinate *coord4)
{
    double x1 = coord2->x - coord1->x;
    double y1 = coord2->y - coord1->y;
    double x2 = coord4->x - coord3->x;
    double y2 = coord4->y - coord3->y;
    double dot = x1 * x2 + y1 * y2;
    double dis1 = sqrt(x1 * x1 + y1 * y1);
    double dis2 = sqrt(x2 * x2 + y2 * y2);
    double dis = dis2 * dis1;
    if (dot > dis && dot > 0)
    {
        return 0;
    }
    if (abs(dot) > dis && dot < 0)
    {
        return 180;
    }
    double radian_angle = acos(dot / (dis));
    double angle = radian_angle * 180 / PI;
    return angle;
}

Coordinate MathUtils::calculatePointOnRay(const Coordinate *startCoord, const Coordinate *endCoord,
                                          double dis)
{
    double segDis = caculatePointDistance(startCoord, endCoord);

    double x = startCoord->x + (endCoord->x - startCoord->x) * (dis + segDis) / segDis;
    double y = startCoord->y + (endCoord->y - startCoord->y) * (dis + segDis) / segDis;
    return Coordinate(x, y, 0);
}

double MathUtils::caculatePointDistance(const Coordinate *coord1, const Coordinate *coord2)
{
    return sqrt((coord1->x - coord2->x) * (coord1->x - coord2->x) + (coord1->y - coord2->y) * (coord1->y - coord2->y));
}

Coordinate MathUtils::caculateFootOnSegement(const Coordinate *coord, const Coordinate *coord1, const Coordinate *coord2)
{
    Coordinate v;
    v.x = coord2->x - coord1->x;
    v.y = coord2->y - coord1->y;

    Coordinate w;
    w.x = coord->x - coord1->x;
    w.y = coord->y - coord1->y;

    double c1 = vectorDot(&v, &w);
    if (c1 < 0)
    {
        return Coordinate(coord1->x, coord1->y, 0);
    }
    double c2 = vectorDot(&v, &v);
    if (c2 < c1)
    {
        return Coordinate(coord2->x, coord2->y, 0);
    }
    double b = c1 / c2;
    Coordinate p;
    p.x = coord1->x + b * v.x;
    p.y = coord1->y + b * v.y;

    return p;
}

double MathUtils::vectorDot(const Coordinate *vector1, const Coordinate *vector2)
{
    return (vector1->x * vector2->x + vector1->y * vector2->y);
}

double MathUtils::caculatePoit2Line(const Coordinate *point, const Coordinate *pnt1, const Coordinate *pnt2)
{
    double A = pnt2->y - pnt1->y;                     // y2-y1
    double B = pnt1->x - pnt2->x;                     // x1-x2;
    double C = pnt2->x * pnt1->y - pnt1->x * pnt2->y; // x2*y1-x1*y2
    if (A * A + B * B < 1e-13)
    {
        return caculatePointDistance(point, pnt1); // pnt1与pnt2重叠
    }
    else if (abs(A * point->x + B * point->y + C) < 1e-10)
    {
        return 0.0; // point在直线上(pnt1_pnt2)
    }
    else
    {
        double x = (B * B * point->x - A * B * point->y - A * C) / (A * A + B * B);
        double y = (-A * B * point->x + A * A * point->y - B * C) / (A * A + B * B);
        double z = pnt1->z;
        Coordinate footP(x, y, z);
        return caculatePointDistance(point, &footP);
    }
}