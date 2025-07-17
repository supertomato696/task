#include "Algorithm/Angle.h"
#include "math.h"
using namespace Engine::Base;
using namespace Engine;
using namespace Engine::Algorithm;
using namespace Engine::Geometries;

//const Double Angle::PI = 3.141592654;

Base::Double Angle::Degree2Radian(const Base::Double& degree)
{
	return 0.01745329252*degree;
}

Base::Double Angle::Radian2Degree(const Base::Double& radian)
{
	return 57.295779513*radian;
}

Geometries::Vector3d Angle::Azimuth2Vector(Base::Double radian)
{
	return Vector3d(sin(radian), cos(radian), 0);
}