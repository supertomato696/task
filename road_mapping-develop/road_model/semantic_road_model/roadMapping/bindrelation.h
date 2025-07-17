//
//
//

#ifndef ROADMAPPING_BINDRELATION_H
#define ROADMAPPING_BINDRELATION_H
#include "afterProcessCommon.h"
#include "Geometries/Coordinate.h"
#include "Base/Array.h"

namespace RoadMapping
{

    class BindRelation
    {
    public:
        static void bind_impassablearea_lukoubd(Engine::Base::Array<ImpassableArea> &areas, Engine::Base::Array<Junction> &lukoubd);
    };

} // namespace RoadMapping
#endif // ROADMAPPING_BINDRELATION_H