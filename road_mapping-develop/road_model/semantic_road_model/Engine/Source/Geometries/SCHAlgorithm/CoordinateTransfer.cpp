//#include "stdafx.h"
#include "Geometries/SCHAlgorithm/CoordinateTransfer.h"
#include <math.h>

//double CoordinateTransfer::PI = 3.14159265358979323846;
double CoordinateTransfer::Earth_R_VALUE = 6378137.0;

CoordinateTransfer::CoordinateTransfer(void)
{
}

CoordinateTransfer::~CoordinateTransfer(void)
{
}

double CoordinateTransfer::calcP2PDistance(double X1, double Y1, double X2, double Y2)
{
	//////////////////////////////////////////////////////////////////////////
	//经纬度
	const double C_PI = 3.1415926;
	const double C_EQUATOR_RADIUS = 6378137.0;
	const double C_POLAR_RADIUS = 6356752.3142;
	double DEG2RAD = (C_PI / 180.0);
	double EQUATOR_CIRCUM = 2 * C_PI * C_EQUATOR_RADIUS;
	double POLAR_CIRCUM = 2 * C_PI * C_POLAR_RADIUS;
	double AVG_LAT = (Y1 + Y2) / 2.0;
	double WK_LON = ((X1 - X2) / 360.0) * EQUATOR_CIRCUM * cos(fabs(AVG_LAT) * DEG2RAD);
	double WK_LAT = ((Y1 - Y2) / 360.0) * POLAR_CIRCUM;
	double distance = sqrt(pow(WK_LON, 2) + pow(WK_LAT, 2));

	//////////////////////////////////////////////////////////////////////////
	//平面
	//double distance = sqrt((X1 - X2)*(X1 - X2) + (Y1 - Y2)*(Y1-Y2));

	return distance;
}

