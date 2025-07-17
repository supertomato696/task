//
//
//
#include "CloudAlgorithm.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Algorithm/GeometricFit.h"
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>

using namespace hdmap_build;
using namespace Engine::Algorithm;

bool CCloudToGeoLine::JudgePointLineLeftRight(Array<Coordinate> &linePoints, Coordinate &point, Bool &onLeft)
{
    Coordinate vectZ(0, 0, 1);

    Coordinate pntProject(0, 0, 0);
    Int32 nSegIndex = -1;
    if (!BaseAlgorithm::GetNearestPntToLineset(point, linePoints, pntProject, nSegIndex))
    {
        return false;
    }
    Coordinate linktn = point - linePoints[nSegIndex];
    Coordinate nerVec = linePoints[nSegIndex + 1] - linePoints[nSegIndex];
    Coordinate cross = linktn.CrossProduct(nerVec);
    onLeft = cross.DotProduct(vectZ) < 0 ? true : false;

    return true;
}

bool CCloudToGeoLine::LeastSquareFit(const Array<Coordinate> &vecCloudPoints, double &A, double &B, double &C, double &thinness, vector<TprojPoint> &vecPrjPoints, Vector3d *pPrevDirectVector)
{
    SizeT n = vecCloudPoints.GetCount();
    if (n < 2)
        return false;

    // caculate LINE Ax+By+C =0;
    Algorithm::GeometricFit::LeastSquareLineFit(vecCloudPoints, A, B, C);

    // Vector3d vDirect = TangentVector(A, B);
    Vector3d vDirect(B, -A, 0);
    vDirect.Normalize();

    if (nullptr != pPrevDirectVector && *pPrevDirectVector * vDirect < 0)
    {
        A = -A;
        B = -B;
        C = -C;
        vDirect = vDirect * -1;
    }

    const Double t = -(A * A + B * B);
    const Double xProj = A * C / t;
    const Double yProj = B * C / t;
    const Double root = sqrt(A * A + B * B);

    thinness = 0.0;

    vecPrjPoints.resize(n);
    for (int i = 0; i < n; i++)
    {
        Coordinate c = vecCloudPoints.ElementAt(i);

        Vector3d v(c.x - xProj, c.y - yProj, 0);

        TprojPoint projPoint;
        projPoint.project = v * vDirect;
        projPoint.projectH = (A * c.x + B * c.y + C) / root;
        projPoint.c = c;

        thinness += abs(projPoint.projectH);

        vecPrjPoints[i] = projPoint;
    }

    thinness = thinness / n;
    return true;
}

Bool CCloudToGeoLine::ProjectPointsToSegments(std::vector<TprojPoint> &vecSortedPoints,
                                              const Base::Double &A, const Base::Double &B, const Base::Double &C, Coordinate &ptStart, Coordinate &ptEnd)
{
    if (vecSortedPoints.size() < 2)
        return false;

    std::sort(vecSortedPoints.begin(), vecSortedPoints.end(), struct_TprojPoint::projPointCmp);

    const Double t = -(A * A + B * B);
    const Double xProj = A * C / t;
    const Double yProj = B * C / t;
    const Double Z_EXTRACT_LENGTH = 0.2;

    // directional vector
    Vector3d vDirect(B, -A, 0);
    vDirect.Normalize();

    const Double projStart = vecSortedPoints.front().project;
    const Double projEnd = vecSortedPoints.back().project;
    ptStart = Coordinate(xProj, yProj, 0) + vDirect * projStart;
    ptEnd = Coordinate(xProj, yProj, 0) + vDirect * projEnd;

    Double sz = 0;
    UInt32 nPoint = 0;
    for (vector<TprojPoint>::iterator _it = vecSortedPoints.begin();
         _it != vecSortedPoints.end(); _it++)
    {
        double distance = std::abs(_it->project - projStart);
        if (distance < Z_EXTRACT_LENGTH)
        {
            sz += _it->c.z;
            nPoint++;
        }
        else
            break;
    }

    sz = sz / nPoint;
    ptStart.z = sz;

    sz = 0;
    nPoint = 0;
    for (vector<TprojPoint>::reverse_iterator _it = vecSortedPoints.rbegin();
         _it != vecSortedPoints.rend(); _it++)
    {
        double distance = abs(_it->project - projEnd);
        if (distance < Z_EXTRACT_LENGTH)
        {
            sz += _it->c.z;
            nPoint++;
        }
        else
            break;
    }
    sz = sz / nPoint;
    ptEnd.z = sz;

    return true;
}

void CCloudToGeoLine::ExactLine(TLineTwoPoints &curblinexy, TLineTwoPoints &curblinez, Array<Coordinate> &xytemp, Array<Coordinate> &dztemp, double &curbwidthxy, double &curbwidthz)
{
    ExactDashLine(xytemp, curblinexy, curbwidthxy);
    ExactDashLine(dztemp, curblinez, curbwidthz);

    if (curblinexy.ptStart.z > curblinexy.ptEnd.z)
    {
        Coordinate tempxy = curblinexy.ptStart;
        curblinexy.ptStart = curblinexy.ptEnd;
        curblinexy.ptEnd = tempxy;
    }
    if (curblinez.ptStart.x < curblinez.ptEnd.x)
    {
        curblinexy.ptStart.z = curblinez.ptStart.y;
        curblinexy.ptEnd.z = curblinez.ptEnd.y;
    }
    else
    {
        curblinexy.ptEnd.z = curblinez.ptStart.y;
        curblinexy.ptStart.z = curblinez.ptEnd.y;
    }
}

bool CCloudToGeoLine::ExactDashLine(Array<Coordinate> &vecCloudPoints, TLineTwoPoints &_linePoints, double &_dAvageThinness)
{
    return ExactDashLine(vecCloudPoints, _linePoints.ptStart, _linePoints.ptEnd, _dAvageThinness);
}

Bool CCloudToGeoLine::ExactDashLine(Array<Coordinate> &_vecCloudPoints, Coordinate &_ptStart, Coordinate &_ptEnd, double &_dAvageThinness)
{
    if (_vecCloudPoints.GetCount() < 2)
        return false;

    const Double LENGTH_LIMIT = 0.05;

    int n = _vecCloudPoints.GetCount();

    const Coordinate HeadPoint = _vecCloudPoints[0];
    for (int j = 0; j < n; j++)
    {
        Coordinate &c = _vecCloudPoints[j];
        c.x = c.x - HeadPoint.x;
        c.y = c.y - HeadPoint.y;
    }

    Double A, B, C;
    Algorithm::GeometricFit::LeastSquareLineFit(_vecCloudPoints, A, B, C);

    Double t = -(A * A + B * B);
    Double xProj = A * C / t;
    Double yProj = B * C / t;
    Double x = sqrt(A * A + B * B + C * C);

    ////////////////////////////////////////////////////////////////////////////
    // Vector3d vDirect(B, -A, 0);
    Coordinate vDirect(B, -A, 0);
    vDirect.Normalize();

    double sum = 0;

    vector<TprojPoint> vecProjectPoint(n);
    for (int i = 0; i < n; i++)
    {
        Coordinate c = _vecCloudPoints.ElementAt(i);

        Vector3d v(c.x - xProj, c.y - yProj, 0);

        TprojPoint projPoint;
        projPoint.project = v * vDirect;
        projPoint.projectH = (A * c.x + B * c.y + C) / x;
        projPoint.c = c;

        sum += abs(projPoint.projectH);
        vecProjectPoint[i] = projPoint;
    }

    _dAvageThinness = sum / n; //
    std::sort(vecProjectPoint.begin(), vecProjectPoint.end(), struct_TprojPoint::projPointCmp);

    double projStart = vecProjectPoint.front().project;
    double projEnd = vecProjectPoint.back().project;

    double length = abs(projStart - projEnd);
    if (length < LENGTH_LIMIT) //
        return false;

    //////////////////////////////////////////////////////////////////////////
    bool bSuccess = SortedProjPointsToSegments(vecProjectPoint, xProj, yProj, vDirect, _ptStart, _ptEnd);

    Vector3d vTemp(HeadPoint.x, HeadPoint.y, 0);
    _ptStart = _ptStart + vTemp;
    _ptEnd = _ptEnd + vTemp;

    // return back
    for (int j = 0; j < n; j++)
    {
        Coordinate &c = _vecCloudPoints[j];
        c.x = c.x + HeadPoint.x;
        c.y = c.y + HeadPoint.y;
    }

    return bSuccess;
}

void CCloudToGeoLine::GetPointsInfoToLine(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointInfoTOLine_Sort> &arrPointInfoSort)
{
    Int32 nPointCount = CloudPoints.GetCount();

    if (nPointCount == 0)
        return;

    arrPointInfoSort.Clear();
    arrPointInfoSort.SetSize(nPointCount);

    TPointInfoTOLine_Sort PointInfoSort;

    Coordinate vDirect = _ptEnd - _ptStart;
    vDirect.Normalize();

    for (Int32 i = 0; i < nPointCount; i++)
    {
        PointInfoSort.dDistance = BaseAlgorithm3D::DisPtToLine(_ptStart, _ptEnd, CloudPoints[i]);
        Coordinate vec = CloudPoints[i] - _ptStart;
        PointInfoSort.dMeasure = vec.DotProduct(vDirect);
        PointInfoSort.index = i;

        arrPointInfoSort.SetAt(i, PointInfoSort);
    }
}

//////////////////////////////////
bool CCloudToGeoLine::ComputePointToLine2DInfo(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointToLine2DInfo> &arrPointToLine2DInfo)
{
    Int32 nPointCount = CloudPoints.GetCount();

    if (nPointCount == 0 || _ptStart.Equals(_ptEnd))
    {
        return false;
    }

    arrPointToLine2DInfo.Clear();
    arrPointToLine2DInfo.SetSize(nPointCount);

    TPointToLine2DInfo pntToLine2DInfo;

    Coordinate vDirect = _ptEnd - _ptStart;
    vDirect.z = 0.0;
    vDirect.Normalize();

    for (Int32 i = 0; i < nPointCount; i++)
    {
        pntToLine2DInfo.Point = CloudPoints[i];

        pntToLine2DInfo.dDistance = BaseAlgorithm::DistancePtToLine2D(&pntToLine2DInfo.Point, &_ptStart, &_ptEnd);
        Int16 Tag = BaseAlgorithm::PntMatchLine(_ptStart, _ptEnd, pntToLine2DInfo.Point);
        pntToLine2DInfo.dDistance = (Tag == 0) * 0 + (Tag == 1) * (-1) * pntToLine2DInfo.dDistance + (Tag == 2) * 1 * pntToLine2DInfo.dDistance;

        Coordinate vec = pntToLine2DInfo.Point - _ptStart;
        vec.z = 0.0;
        pntToLine2DInfo.dMeasure = vec.DotProduct(vDirect);

        arrPointToLine2DInfo[i] = pntToLine2DInfo;
    }

    return true;
}

bool CCloudToGeoLine::ComputePointToLine2DInfo(const Array<Coordinate> &_CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointToLine2DInfo> &_arrPointToLine2DInfo, bool (*cmp)(const struct_TPointToLine2DInfo &, const struct_TPointToLine2DInfo &))
{
    if (ComputePointToLine2DInfo(_CloudPoints, _ptStart, _ptEnd, _arrPointToLine2DInfo) == false)
        return false;

    sort(_arrPointToLine2DInfo.Begin(), _arrPointToLine2DInfo.End(), cmp);
    return true;
}
///////////////////////

void CCloudToGeoLine::GetPointsInfoToLine(const Array<Coordinate> &CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointInfoTOLine_Sort> &_arrPointInfoSort, bool (*cmp)(const struct_TPointInfoTOLine_Sort &, const struct_TPointInfoTOLine_Sort &))
{
    GetPointsInfoToLine(CloudPoints, _ptStart, _ptEnd, _arrPointInfoSort);

    sort(_arrPointInfoSort.Begin(), _arrPointInfoSort.End(), cmp);
}

void CCloudToGeoLine::GetPointsInfoToLine(const Array<Coordinate> &_CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, Array<TPointInfoTOLine_Sort> &_arrPointInfoSort, double &_min, double &_max, bool (*_cmp)(const struct_TPointInfoTOLine_Sort &, const struct_TPointInfoTOLine_Sort &))
{
    GetPointsInfoToLine(_CloudPoints, _ptStart, _ptEnd, _arrPointInfoSort, _cmp);
    int nSize = _arrPointInfoSort.GetCount();
    if (nSize < 2)
        return;
    _min = _arrPointInfoSort.GetAt(0).dMeasure;
    _max = _arrPointInfoSort.GetAt(nSize - 1).dMeasure;
}

void CCloudToGeoLine::GetPointsInfoToLine(const Array<Coordinate> &_CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd, double &_min, double &_max, bool (*cmp)(const struct_TPointInfoTOLine_Sort &, const struct_TPointInfoTOLine_Sort &))
{
    Array<TPointInfoTOLine_Sort> arrPointInfoSort;
    GetPointsInfoToLine(_CloudPoints, _ptStart, _ptEnd, arrPointInfoSort, _min, _max, cmp);
}

double CCloudToGeoLine::GetPointsInfoToLine_Len(const Array<Coordinate> &_CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd)
{
    double min = 0;
    double max = 0;
    GetPointsInfoToLine(_CloudPoints, _ptStart, _ptEnd, min, max);
    return max - min;
}

double CCloudToGeoLine::GetPointsInfoToLine_Dif(const Array<Coordinate> &_CloudPoints, const Coordinate &_ptStart, const Coordinate &_ptEnd)
{
    Array<TPointInfoTOLine_Sort> arrPointInfoSort;
    GetPointsInfoToLine(_CloudPoints, _ptStart, _ptEnd, arrPointInfoSort, TPointInfoTOLine_Sort::dMeasureCmp_dMeasure);
    int nSize = arrPointInfoSort.GetCount();
    if (nSize < 2)
        return 0;
    return arrPointInfoSort.GetAt(0).dMeasure - arrPointInfoSort.GetAt(1).dMeasure;
}

bool CCloudToGeoLine::SortedProjPointsToSegments(std::vector<TprojPoint> &vecProjectPoint, Double xProj,
                                                 Double yProj, const Coordinate &vDirect, Coordinate &ptStart, Coordinate &ptEnd)
{
    if (vecProjectPoint.empty())
        return false;

    const Double Z_EXTRACT_LENGTH = 0.2;

    Double projStart = vecProjectPoint.front().project;
    Double projEnd = vecProjectPoint.back().project;

    Double sz = 0;
    Int32 nPoint = 0;
    for (vector<TprojPoint>::iterator _it = vecProjectPoint.begin();
         _it != vecProjectPoint.end(); _it++)
    {
        Double distance = abs(_it->project - projStart);
        if (distance < Z_EXTRACT_LENGTH)
        {
            sz += _it->c.z;
            nPoint++;
        }
        else
            break;
    }

    sz = sz / nPoint;
    ptStart = Coordinate(xProj, yProj, sz) + vDirect * projStart;

    sz = 0;
    nPoint = 0;
    for (vector<TprojPoint>::reverse_iterator _it = vecProjectPoint.rbegin();
         _it != vecProjectPoint.rend(); _it++)
    {
        Double distance = abs(_it->project - projEnd);
        if (distance < Z_EXTRACT_LENGTH)
        {
            sz += _it->c.z;
            nPoint++;
        }
        else
            break;
    }
    sz = sz / nPoint;
    ptEnd = Coordinate(xProj, yProj, sz) + vDirect * projEnd;

    return true;
}

//***************************************************************************************************//
bool CSpatialProjection::Get2DPlaneFrom3D(Array<Coordinate> &CloudPoints, const Coordinate &pntOrign, Coordinate &normalVector)
{
    if (CloudPoints.IsEmpty())
        return false;

    for (Int32 i = 0, n = CloudPoints.GetCount(); i < n; i++)
    {
        Coordinate pntProject; //
        BaseAlgorithm3D::GetProjectpntToPlane(CloudPoints[i], pntOrign, normalVector, pntProject);

        CloudPoints[i] = pntProject;
    }

    return true;
}

bool CSpatialProjection::ComputePlaneFromPoints(const Array<Coordinate> &Points, Coordinate &pntOrg, Coordinate &normalVector)
{
    if (Points.GetCount() < 3)
    {
        return false;
    }

    Array<Double> PlaneParameters;
    PlaneParameters.SetSize(4);

    Eigen::Vector4f Parameters;
    Float curvature;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.resize(Points.GetCount());

    for (Int32 i = 0, n = Points.GetCount(); i < n; i++)
    {
        cloud.points[i].x = Points[i].x;
        cloud.points[i].y = Points[i].y;
        cloud.points[i].z = Points[i].z;
    }
    pcl::computePointNormal(cloud, Parameters, curvature);

    PlaneParameters[0] = Parameters[0];
    PlaneParameters[1] = Parameters[1];
    PlaneParameters[2] = Parameters[2];
    PlaneParameters[3] = Parameters[3];

    pntOrg = Coordinate(0, 0, 0);

    if (PlaneParameters[2] != 0)
    {
        pntOrg.z = (-PlaneParameters[3]) / PlaneParameters[2];
    }
    else if (PlaneParameters[1] != 0)
    {
        pntOrg.y = (-PlaneParameters[3]) / PlaneParameters[1];
    }
    else
    {
        pntOrg.x = (-PlaneParameters[3]) / PlaneParameters[0];
    }

    normalVector = Coordinate(PlaneParameters[0], PlaneParameters[1], PlaneParameters[2]); // ƽ��ķ�����
    normalVector.Normalize();

    return true;
}