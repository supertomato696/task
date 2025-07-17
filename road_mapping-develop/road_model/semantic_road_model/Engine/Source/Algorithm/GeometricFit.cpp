#include "Algorithm/GeometricFit.h"
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Engine::Algorithm;
using namespace Engine::Base;
using namespace Engine::Geometries;
using namespace Engine;

Base::Bool GeometricFit::LeastSquareLineFit(const Base::Array<Coordinate>& vecPoints, Base::Int32 nStartIndex, Base::Int32 nEndIndex, Base::Double& dA, Base::Double& dB, Base::Double& dC)
{
	Int32 size = nEndIndex - nStartIndex;
	if (size < 2)
		return false;

	Double sx, sy;
	sx = sy = 0;

	//x y ƽ��ֵ
	for (Int32 j = nStartIndex; j < nEndIndex; j++)
	{
		Coordinate c = vecPoints[j];
		sx += c.x;
		sy += c.y;
	}

	double xAvg = sx / size;
	double yAvg = sy / size;

	/*
	*	��XTX������ֵ
	*/
	Eigen::MatrixXd  X = Eigen::MatrixXd::Zero(size, 2);
	for (Int32 i = nStartIndex; i < nEndIndex; i++)
	{
		Coordinate c = vecPoints.ElementAt(i);
		X(i, 0) = c.x - xAvg;
		X(i, 1) = c.y - yAvg;
	}

	Eigen::MatrixXd XTX = X.transpose()*X;

//	Eigen::EigenSolver<Eigen::MatrixXd> es(XTX);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
    es.compute(XTX);
    Eigen::MatrixXd eigenvectors = es.eigenvectors();
    Eigen::VectorXd eigenvalues = es.eigenvalues();
//    std::cout<<"es.eigenvalues:"<< eigenvalues <<std::endl;
//	std::complex<Double>  d0 = es.eigenvalues()[0];
//	std::complex<Double>  d1 = es.eigenvalues()[1];
    double d0 = eigenvalues[0];
    double d1 = eigenvalues[1];
//    std::cout<<"d0:"<<d0<<std::endl;
//    std::cout<<"d1:"<<d1<<std::endl;
//    std::cout<<"v0:"<<eigenvectors.col(0)<<std::endl;
//    std::cout<<"v1:"<<eigenvectors.col(1)<<std::endl;
//	Eigen::VectorXcd V0;
//	if (d0.real() < d1.real())
//		V0 = es.eigenvectors().col(0);
//	else
//		V0 = es.eigenvectors().col(1);
    Eigen::VectorXd V0;
    if (d0 < d1)
		V0 = eigenvectors.col(0);
	else
		V0 = eigenvectors.col(1);
//    std::cout<<"V0:"<<V0<<std::endl;

	dA = V0.x();
	dB = V0.y();
	dC = -dA*xAvg - dB*yAvg;

	return true;
}

Base::Bool GeometricFit::LeastSquareLineFit(const Base::Array<Coordinate>& vecPoints, Base::Double& dA, Base::Double& dB, Base::Double& dC)
{
	return LeastSquareLineFit(vecPoints, 0, vecPoints.GetCount(), dA, dB, dC);
}

Base::Bool GeometricFit::LeastSquareLineFit(const Base::Array<Coordinate>& vecPoints, Coordinate& pntStart, Coordinate& pntEnd)
{
	Double A, B, C;
	if (!LeastSquareLineFit(vecPoints, A, B, C))
		return false;

	//�� vecPoints[0]��ֱ���ϵ�ͶӰ��(x0,y0)
	//�ⷽ��Kx = D;
	Eigen::Matrix2d K;
	Eigen::Vector2d D;

	K << A, B, -B, A;
	D << -C, (A*vecPoints[0].y - B*vecPoints[0].x);

	Eigen::ColPivHouseholderQR<Eigen::Matrix2d> dec(K);
	Eigen::Vector2d x = dec.solve(D);

	Coordinate c0;
	c0.x = x[0];
	c0.y = x[1];
	c0.z = 0;

	//���� vecPoints[i] - ��x0,y0�� ��ֱ���ϵ�ͶӰ
	Vector3d V(-B, A, 0);
	V.Normalize();

	Array<Double> array_Projections(vecPoints.GetCount());
	for (SizeT i = 0; i < vecPoints.GetCount();i++)
	{
		const Coordinate& c = vecPoints[i];
		array_Projections[i] = (c - c0)*V;
	}

	const Double min_dis = *(std::min_element(array_Projections.Begin(), array_Projections.End()));
	const Double max_dis = *(std::max_element(array_Projections.Begin(), array_Projections.End()));

	pntStart = c0 + V*min_dis;
	pntEnd = c0 + V*max_dis;

	return true;
}

Bool GeometricFit::LeastSquareCircleFit_Algebraic(const Base::Array<Geometries::Coordinate>& vecPoints, Base::Double& cir_x, Base::Double& cir_y, Base::Double& cir_r)
{
	if (vecPoints.IsEmpty())
		return false;

	int i, iter, IterMAX = 99;

	double Xi, Yi, Zi;
	double Mz, Mxy, Mxx, Myy, Mxz, Myz, Mzz, Cov_xy, Var_z;
	double A0, A1, A2, A22, A3, A33;
	double Dy, xnew, x, ynew, y;
	double DET, Xcenter, Ycenter;

	double avg_x, avg_y,avg_z;
	AverageXYZ(vecPoints, avg_x, avg_y,avg_z);   // Compute x- and y- sample means (via a function in the class "data")

	//     computing moments 

	Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.;

	int n = vecPoints.GetCount();

	for (i = 0; i <n; i++)
	{
		Xi = vecPoints[i].x - avg_x;   //  centered x-coordinates
		Yi = vecPoints[i].y - avg_y;   //  centered y-coordinates
		Zi = Xi*Xi + Yi*Yi;

		Mxy += Xi*Yi;
		Mxx += Xi*Xi;
		Myy += Yi*Yi;
		Mxz += Xi*Zi;
		Myz += Yi*Zi;
		Mzz += Zi*Zi;
	}
	Mxx /= n;
	Myy /= n;
	Mxy /= n;
	Mxz /= n;
	Myz /= n;
	Mzz /= n;

	//      computing coefficients of the characteristic polynomial

	Mz = Mxx + Myy;
	Cov_xy = Mxx*Myy - Mxy*Mxy;
	Var_z = Mzz - Mz*Mz;
	A3 = 4 * Mz;
	A2 = -3 * Mz*Mz - Mzz;
	A1 = Var_z*Mz + 4 * Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
	A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
	A22 = A2 + A2;
	A33 = A3 + A3 + A3;

	//    finding the root of the characteristic polynomial
	//    using Newton's method starting at x=0  
	//     (it is guaranteed to converge to the right root)

	for (x = 0., y = A0, iter = 0; iter < IterMAX; iter++)  // usually, 4-6 iterations are enough
	{
		Dy = A1 + x*(A22 + A33*x);
		xnew = x - y / Dy;
		if ((xnew == x) || (!std::isfinite(xnew))) break;
		ynew = A0 + xnew*(A1 + xnew*(A2 + xnew*A3));
		if (fabs(ynew) >= fabs(y))  break;
		x = xnew;  y = ynew;
	}

	//       computing paramters of the fitting circle

	DET = x*x - x*Mz + Cov_xy;
	Xcenter = (Mxz*(Myy - x) - Myz*Mxy) / DET / 2;
	Ycenter = (Myz*(Mxx - x) - Mxz*Mxy) / DET / 2;

	//       assembling the output
	cir_x = Xcenter + avg_x;
	cir_y = Ycenter + avg_y;
	cir_r = sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz);

	return true;
}

Base::Bool GeometricFit::LeastSquareCircleFit_Geometric(const Base::Array<Geometries::Coordinate>& vecPoints_Original, Base::Double& cir_x, Base::Double& cir_y, Base::Double& cir_r)
{
	if (vecPoints_Original.IsEmpty())
		return false;

	//offset 
	Base::Array<Geometries::Coordinate> vecPoints = vecPoints_Original;
	Geometries::Coordinate ptFront = vecPoints[0];
	for (size_t i = 0; i < vecPoints.GetCount(); i++)
	{
		vecPoints[i] = vecPoints[i] - ptFront;
	}

	typedef struct {
		double x;
		double y;
		double r;
	}circle;

	circle circleIni;
	if (!LeastSquareCircleFit_Algebraic(vecPoints, circleIni.x, circleIni.y,circleIni.r))
		return false;

	int n = vecPoints.GetCount();

	double avg_x, avg_y, avg_z;
	AverageXYZ(vecPoints, avg_x, avg_y,avg_z);
	
	int code, i, iter, inner, IterMAX = 99;

	double factorUp = 10., factorDown = 0.04, lambda, ParLimit = 1.e+6;
	double dx, dy, ri, u, v;
	double Mu, Mv, Muu, Mvv, Muv, Mr, UUl, VVl, Nl, F1, F2, F3, dX, dY, dR;
	double epsilon = 3.e-8;
	double G11, G22, G33, G12, G13, G23, D1, D2, D3;

	circle Old, New;

	double oldCircleSd, newCircleSd;
	//       starting with the given initial circle (initial guess)

	New = circleIni;

	double LambdaIni = 0.01;

	//       compute the root-mean-square error via function Sigma; see Utilities.cpp
	newCircleSd = DeviationAnalysis(vecPoints, New.x,New.y,New.r);

	//       initializing lambda, iteration counters, and the exit code
	lambda = LambdaIni;
	iter = inner = code = 0;

NextIteration:

	Old = New;
	oldCircleSd = newCircleSd;

	if (++iter > IterMAX) { code = 1;  goto enough; }

	//       computing moments
	Mu = Mv = Muu = Mvv = Muv = Mr = 0.;

	for (i = 0; i <n; i++)
	{
		dx = vecPoints[i].x - Old.x;
		dy = vecPoints[i].y - Old.y;
		ri = sqrt(dx*dx + dy*dy);
		u = dx / ri;
		v = dy / ri;
		Mu += u;
		Mv += v;
		Muu += u*u;
		Mvv += v*v;
		Muv += u*v;
		Mr += ri;
	}
	Mu /= n;
	Mv /= n;
	Muu /= n;
	Mvv /= n;
	Muv /= n;
	Mr /= n;

	//       computing matrices
	F1 = Old.x + Old.r*Mu - avg_x;
	F2 = Old.y + Old.r*Mv - avg_y;
	F3 = Old.r - Mr;

try_again:

	UUl = Muu + lambda;
	VVl = Mvv + lambda;
	Nl = 1.0 + lambda;

	//  Cholesly decomposition
	G11 = sqrt(UUl);
	G12 = Muv / G11;
	G13 = Mu / G11;
	G22 = sqrt(VVl - G12*G12);
	G23 = (Mv - G12*G13) / G22;
	G33 = sqrt(Nl - G13*G13 - G23*G23);

	D1 = F1 / G11;
	D2 = (F2 - G12*D1) / G22;
	D3 = (F3 - G13*D1 - G23*D2) / G33;

	dR = D3 / G33;
	dY = (D2 - G23*dR) / G22;
	dX = (D1 - G12*dY - G13*dR) / G11;

	if ((fabs(dR) + fabs(dX) + fabs(dY)) / (1.0 + Old.r) < epsilon) goto enough;

	//       updating the parameters

	New.x = Old.x - dX;
	New.y = Old.y - dY;

	if (fabs(New.x) > ParLimit || fabs(New.y) > ParLimit) { code = 3; goto enough; }

	New.r = Old.r - dR;

	if (New.r <= 0.)
	{
		lambda *= factorUp;
		if (++inner > IterMAX) { code = 2;  goto enough; }
		goto try_again;
	}

	//       compute the root-mean-square error via function Sigma; see Utilities.cpp
	newCircleSd = DeviationAnalysis(vecPoints, New.x, New.y, New.r);

	//       check if improvement is gained
	if (newCircleSd < oldCircleSd)    //   yes, improvement
	{
		lambda *= factorDown;
		goto NextIteration;
	}
	else                       //   no improvement
	{
		if (++inner > IterMAX) { code = 2;  goto enough; }
		lambda *= factorUp;
		goto try_again;
	}

	//       exit

enough:
	cir_x = Old.x + ptFront.x;
	cir_y = Old.y + ptFront.y;
	cir_r = Old.r;

	return (code ==0);
}

Void GeometricFit::AverageXYZ(const Base::Array<Geometries::Coordinate>& vecPoints, Base::Double& x, Base::Double& y,Base::Double& z)
{
	x = 0;
	y = 0;
	z = 0;
	auto n = vecPoints.GetCount();
	for (auto i = 0; i < n;i++)
	{
		x += vecPoints[i].x / n;
		y += vecPoints[i].y / n;
		z += vecPoints[i].z / n;
	}
}

Double GeometricFit::DeviationAnalysis(const Base::Array<Geometries::Coordinate>& vecPoints, const Base::Double& x, const Base::Double& y, const Base::Double& r)
{
	Double sum = 0;
	auto n = vecPoints.GetCount();
	for (auto i = 0; i < n;i++)
	{
		const Geometries::Coordinate& p = vecPoints[i];
		auto d = sqrt(pow((p.x - x), 2) + pow((p.y - y), 2)) - r;
		sum +=  d*d/ n;
	}

	return sqrt(sum);
}

Base::Bool GeometricFit::LeastSquare3DLineFit(const Base::Array<Geometries::Coordinate>& vecPoints, Geometries::Vector3d& directVector, Geometries::Coordinate& pointOnLine)
{
	auto n = vecPoints.GetCount();
	if (n < 2)
		return false;

	AverageXYZ(vecPoints, pointOnLine.x, pointOnLine.y, pointOnLine.z);

	Double x2, y2, z2, xy, yz, xz;
	x2 = y2 = z2 = xy = yz = xz = 0;

	for (auto i = 0; i < n;i++)
	{
		const Coordinate c = vecPoints[i] - pointOnLine;
		x2 += c.x * c.x;
		y2 += c.y *c.y;
		z2 += c.z*c.z;
		xy += c.x*c.y;
		yz += c.y*c.z;
		xz += c.x*c.z;
	}

	Eigen::Matrix3d S;
	S << (y2 + z2), -xy, -xz, -xy, (x2 + z2), -yz, -xz, -yz, (x2 + y2);

	Eigen::EigenSolver<Eigen::MatrixXd> es(S);

	vector<Int32> eigen_values(3);
	for (auto i = 0; i < 3;i++)
	{
		eigen_values[i] = es.eigenvalues()[i].real();
	} 

	auto min_eigen_value_index = std::distance(eigen_values.begin(), std::min_element(eigen_values.begin(), eigen_values.end()));

	Eigen::VectorXcd V0 = es.eigenvectors().col(min_eigen_value_index);

	directVector.x = V0(0).real();
	directVector.y = V0(1).real();
	directVector.z = V0(2).real();

	return true;
}