//
//
//

#ifndef ROADMAPPING_MATHUTILS_H
#define ROADMAPPING_MATHUTILS_H
using namespace Engine::Geometries;
class MathUtils
{
public:
    static double getTwoVectorAngle(const Coordinate *coord1, const Coordinate *coord2,
                                    const Coordinate *coord3, const Coordinate *coord4);
    static Coordinate calculatePointOnRay(const Coordinate *startCoord, const Coordinate *endCoord, double dis);
    static double caculatePointDistance(const Coordinate *coord1, const Coordinate *coord2);
    static Coordinate caculateFootOnSegement(const Coordinate *coord, const Coordinate *coord1, const Coordinate *coord2);
    static double vectorDot(const Coordinate *vector1, const Coordinate *vector2);
    static double caculatePoit2Line(const Coordinate *point, const Coordinate *pnt1, const Coordinate *pnt2);
};
#endif // ROADMAPPING_MATHUTILS_H
