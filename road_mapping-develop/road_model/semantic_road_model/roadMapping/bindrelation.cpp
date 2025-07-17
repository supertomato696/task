//
//
//
#include "bindrelation.h"

namespace RoadMapping
{
    void BindRelation::bind_impassablearea_lukoubd(Engine::Base::Array<ImpassableArea> &areas, Engine::Base::Array<Junction> &lukoubd)
    {
        if (areas.IsEmpty() || lukoubd.IsEmpty())
        {
            return;
        }

        for (int i = 0; i < areas.GetCount(); i++)
        {
            if (areas[i].isBind == true)
            {
                continue;
            }

            for (int j = 0; j < lukoubd.GetCount(); j++)
            {
                // 判断是否在路口内
                if (afterProcessCommon::IsInPolygon(areas[i].plygonPnts, lukoubd[j].lukou_polygon_pts))
                {
                    areas[i].isBind = true;
                    lukoubd[j].areas_relation.Add(areas[i]);
                }
            }
        }
    }
}
