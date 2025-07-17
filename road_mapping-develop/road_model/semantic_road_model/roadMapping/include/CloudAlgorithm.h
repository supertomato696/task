//
//
//

#ifndef HDMAP_BUILD_CLOUDALGORITHM_H
#define HDMAP_BUILD_CLOUDALGORITHM_H
#include "Geometries/Coordinate.h"
#include "math.h"

using namespace Engine;
using namespace Engine::Geometries;
using namespace Engine::Base;

namespace hdmap_build
{
    typedef struct struct_TprojPoint
    {
        double project;  //
        double projectH; //
        Coordinate c;    //

        static bool projPointCmp(const struct_TprojPoint c0, const struct_TprojPoint c1)
        {
            return c0.project < c1.project;
        }

        static bool projPointAbsHeightCmp(const struct_TprojPoint c0, const struct_TprojPoint c1)
        {
            return fabs(c0.projectH) < fabs(c1.projectH);
        }

    } TprojPoint;

    typedef struct struct_TPointInfoTOLine_Sort
    {
        //
        Double dDistance;
        //
        Double dMeasure;
        //
        UInt32 index;

        static bool dMeasureCmp_dMeasure(const struct_TPointInfoTOLine_Sort &p0, const struct_TPointInfoTOLine_Sort &p1)
        {
            return p0.dMeasure < p1.dMeasure;
        }

        static bool dDistanceCmp_dDistance(const struct_TPointInfoTOLine_Sort &p0, const struct_TPointInfoTOLine_Sort &p1)
        {
            return p0.dDistance < p1.dDistance;
        }

        static bool dDistanceCmp_dDistance_Desc(const struct_TPointInfoTOLine_Sort &p0, const struct_TPointInfoTOLine_Sort &p1)
        {
            return p0.dDistance > p1.dDistance;
        }

        static Base::Bool dIndexCmpPointInfo(const struct_TPointInfoTOLine_Sort &p0, const struct_TPointInfoTOLine_Sort &p1)
        {
            return p0.index < p1.index;
        }

    } TPointInfoTOLine_Sort;

    typedef struct struct_TPointToLine2DInfo
    {
        Coordinate Point; //
        Double dDistance; //
        Double dMeasure;  //

        static bool dDistanceLineCmp_dDistanceLine(const struct_TPointToLine2DInfo &p0, const struct_TPointToLine2DInfo &p1)
        {
            return p0.dDistance < p1.dDistance;
        }

        static bool dMeasureLineCmp_dMeasureLine(const struct_TPointToLine2DInfo &p0, const struct_TPointToLine2DInfo &p1)
        {
            return p0.dMeasure < p1.dMeasure;
        }

    } TPointToLine2DInfo;

    typedef struct struct_TLineTwoPoints
    {
        Coordinate ptStart; // ��ʼ��
        Coordinate ptEnd;   // ��β��
    } TLineTwoPoints;

    class CCloudToGeoLine
    {
    public:
        static bool JudgePointLineLeftRight(Array<Coordinate> &linePoints, Coordinate &point, Bool &onLeft);

        static bool Compute2DPointsToLine(Array<Coordinate> &CloudPoints, const Coordinate &normalVector, Coordinate &_ptStart, Coordinate &_ptEnd);

        static bool ExactDashLine(Array<Coordinate> &vecCloudPoints, Coordinate &_ptStart, Coordinate &_ptEnd, double &_dAvageThinness);
        static bool ExactDashLine(Array<Coordinate> &vecCloudPoints, TLineTwoPoints &_linePoints, double &_dAvageThinness);
        static bool LeastSquareFit(const Array<Coordinate> &vecCloudPoints, double &A, double &B, double &C, double &thinness, vector<TprojPoint> &vecPrjPoints, Vector3d *pPrevDirectVector = NULL);
        static bool ProjectPointsToSegments(std::vector<TprojPoint> &vecSortedPoints, const Base::Double &A, const Base::Double &B, const Base::Double &C, Coordinate &ptStart, Coordinate &ptEnd);
        static void ExactLine(TLineTwoPoints &curblinexy, TLineTwoPoints &curblinez, Array<Coordinate> &xytemp, Array<Coordinate> &dztemp, double &curbwidthxy, double &curbwidthz);
        static void GetPointsInfoToLine(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointInfoTOLine_Sort> &arrPointInfoSort);

        static void GetPointsInfoToLine(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointInfoTOLine_Sort> &arrPointInfoSort, bool (*cmp)(const struct_TPointInfoTOLine_Sort &, const struct_TPointInfoTOLine_Sort &));

        static void GetPointsInfoToLine(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointInfoTOLine_Sort> &arrPointInfoSort, double &_min, double &_max, bool (*cmp)(const struct_TPointInfoTOLine_Sort &, const struct_TPointInfoTOLine_Sort &) = TPointInfoTOLine_Sort::dMeasureCmp_dMeasure);

        static void GetPointsInfoToLine(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, double &_min, double &_max, bool (*cmp)(const struct_TPointInfoTOLine_Sort &, const struct_TPointInfoTOLine_Sort &) = TPointInfoTOLine_Sort::dMeasureCmp_dMeasure);

        static double GetPointsInfoToLine_Len(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd);

        static double GetPointsInfoToLine_Dif(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd);

        static bool ComputePointToLine2DInfo(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointToLine2DInfo> &arrPointToLine2DInfo);
        static bool ComputePointToLine2DInfo(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointToLine2DInfo> &arrPointToLine2DInfo, bool (*cmp)(const struct_TPointToLine2DInfo &, const struct_TPointToLine2DInfo &));
        static bool SortedProjPointsToSegments(std::vector<TprojPoint> &vecProjectPoint, Double xProj,
                                               Double yProj, const Coordinate &vDirect, Coordinate &ptStart, Coordinate &ptEnd);
    };

    class CSpatialProjection
    {

    public:
        static bool Get2DPlaneFrom3D(Array<Coordinate> &CloudPoints, const Coordinate &pntOrign, Coordinate &normalVector);

        static bool Get2DPlaneFrom3D(Array<Coordinate> &CloudPoints, const Array<Double> &PlaneParameters);

        static bool Get3DPlaneFrom3D(Array<Coordinate> &CloudPoints, const Array<Double> &PlaneParameters);

        static bool Get3DPlaneFrom3D(const Array<Coordinate> &_inCloudPoints, Array<Coordinate> &_outCloudPoints, const Array<Double> &_PlaneParameters);

        static bool ComputePlaneFromPoints(const Array<Coordinate> &Points, Coordinate &pntOrg, Coordinate &normalVector);

        static bool ComputePointsToPlane(const Array<Coordinate> &CloudPoints, Array<Double> &PlaneParameters);

        static void Convert3DCoordTo2D(Array<Coordinate> &CloudPoints, const Coordinate &normalVector);

        static void Convert3DCoordTo2D(Array<Coordinate> &CloudPoints, const Coordinate &normalVector, Coordinate &_originPoint, Coordinate &_goalVector, Coordinate &_rotateVector, double &_rotateAngle);

        static bool ComputeProjDiagonal(const Array<Coordinate> &CloudPoints, const Coordinate &PoleDirection, Double &ProjDiagonal);

        static bool ComputeProjArea(const Array<Coordinate> &CloudPoints, const Coordinate &PoleDirection, int &ProjArea);

        static void GetVerticalSegment(const Coordinate &_ptStart, const Coordinate &_ptEnd, Coordinate &_ProVector, Coordinate &_RotateVector, Coordinate &_goalStart, Coordinate &_goalEnd);

        static void GetNormalVector(Array<Coordinate> &coors3D, Array<Coordinate> &coorsTrace3D, Coordinate &vectorTrace);

        static Void GetNormalVector(Array<Coordinate> &coors3D, Array<Coordinate> &coorsTrace3D,
                                    Coordinate &vectorTrace, Bool *paraSign);

        static Void CorrectVecDirection(const Array<Coordinate> &TraceCoordinates, const Coordinate &onePnt,
                                        Coordinate &normalVector, Bool *paraSign, Coordinate *gpsPoint = NULL);

        static void CorrectVecDirection(const Array<Coordinate> &TraceCoordinates, const Coordinate &onePnt, Coordinate &normalVector);

        static bool GetCastPoints(Array<Coordinate> &points, Array<Coordinate> &cloudpoints);
    };
}

#endif // HDMAP_BUILD_CLOUDALGORITHM_H
