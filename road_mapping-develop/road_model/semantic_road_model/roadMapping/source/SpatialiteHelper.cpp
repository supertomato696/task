#include "SpatialiteHelper.h"
//#include <spatialite/gg_structs.h>
//#include <spatialite/gg_const.h>
#include "Vec.h"

using namespace hdmap_build;
using namespace std;
void SpatialiteHelper::split(const std::string & s, std::vector<std::string>& v, const std::string & c)
{
    std::string::size_type pos1 = 0;
    std::string::size_type pos2 = s.find(c);
    while (std::string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
    {
        v.push_back(s.substr(pos1));
    }
}

//void SpatialiteHelper::geomToCoord(gaiaGeomCollPtr geom, Vec3& geometry)
//{
//    //SD数据保存经纬度，没有Z值
//    geometry.x = geom->FirstPoint->X;
//    geometry.y = geom->FirstPoint->Y;
//    geometry.z = 0.0f;
//}

//void SpatialiteHelper::geomToCoords(gaiaGeomCollStruct* geom, std::vector<Vec3>& geometry)
//{
//    for (int i = 0; i < geom->FirstLinestring->Points; ++i)
//    {
//        double x, y, z;
//        gaiaGetPointXYZ(geom->FirstLinestring->Coords, i, &x, &y, &z);
//        Vec3 newVec3;
//        newVec3.x = x;
//        newVec3.y = y;
//        newVec3.z = z;
//        geometry.push_back(newVec3);
//    }
//}

