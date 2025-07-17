//
//
//
#ifndef HDMAP_BUILD_LINEOBJ_H
#define HDMAP_BUILD_LINEOBJ_H
#include "Geometries/Coordinate.h"
#include "Base/Types.h"
using namespace Engine;
using namespace Engine::Base;
using namespace Engine::Geometries;
namespace hdmap_build
{
    class LineObj
    {
    public:
        int m_id;
        int64_t m_lane_edge_id;
        int64_t m_prev_id;
        int64_t m_next_id;
        int m_lane_edge_sequence;
        double m_ave_sequence;
        std::vector<int> m_vecSeq;
        Array<Coordinate> m_lineCroods;
        double length;

        LineObj()
        {
            m_id = -1;
            m_lane_edge_id = -1;
            m_prev_id = -1;
            m_next_id = -1;
            m_lane_edge_sequence = -1;
            m_ave_sequence = -1;
        }
        ~LineObj()
        {
        }

        static bool dCmp_Length(const LineObj &p0, const LineObj &p1)
        {
            return p0.length > p1.length;
        }

        static bool dCmp_aveSeq(const LineObj &p0, const LineObj &p1)
        {
            return p0.m_ave_sequence < p1.m_ave_sequence;
        }
    };
}
#endif // HDMAP_BUILD_LINEOBJ_H
