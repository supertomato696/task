//
//
//

#pragma once
#include "Vec.h"

#include <string>
#include <vector>

// #include <sqlite3.h>
// #include <spatialite.h>
// #include <spatialite/gg_structs.h>

namespace hdmap_build
{
    class SpatialiteHelper
    {
    public:
        static void split(const std::string &s, std::vector<std::string> &v, const std::string &c);

        //        static void geomToCoord(gaiaGeomCollPtr geom, Vec3& geometry);
        //
        //        static void geomToCoords(gaiaGeomCollStruct* geom, std::vector<Vec3>& geometry);
    };
}