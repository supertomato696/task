#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <vector>
#include <limits>
#include <iostream>
#include <cpl_port.h>
#include <geos/io/WKTReader.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <Eigen/SparseLU>
#include <Eigen/Dense>

using namespace geos::geom;
using namespace geos::io;
using namespace std;
using namespace Eigen;

typedef struct {
	double derivative;
	double dderivative;
	double value;
}interpolator;

typedef struct {
	double x;
	double y;
	double curvature;
}coordinate_ex;

class interpolation {
public:
	interpolation();
	~interpolation();

public:
	void multivariateInterpolation(string& wkt, vector<coordinate_ex*>& coordinate_list);

private:
	void univariateInterpolation(vector<Coordinate>* pCS, vector<interpolator*>& values);

	void chordLength(vector<Coordinate>* pCS, double* h);

	void univariateCoordinates(vector<Coordinate>* pCS, vector<Coordinate>* pCS_x, vector<Coordinate>* pCS_y);

	ArrayXd diff(const ArrayXd& vec);

	Array<DenseIndex,Dynamic,1> digitize(const ArrayXd& arr, const ArrayXd& bins);

	SparseMatrix<double,ColMajor, DenseIndex> makeSparseMatrix(const ArrayXXd& diags, const Array<DenseIndex,Dynamic, 1>& offsets, DenseIndex rows, DenseIndex cols);

	ArrayXd solve(const SparseMatrix<double, ColMajor, DenseIndex>& A, const ArrayXd& b);

	void makeCurve(ArrayXd& xdata, const ArrayXd& ydata, ArrayXd& weights, ArrayXXd& coeffs);

	Array<DenseIndex, Dynamic, 1> getIndex(ArrayXd& xdata, ArrayXd& xidata);

	double optimFunction(double a, double b, double c, double d, double x, double x0);

	double optimFunction(double sigma, double k1, double k2, double x1, double x2, double h, double x, double y1, double y2);

	double derivative(double a, double b, double c, double x, double x0);

	double dderivative(double a, double b, double x, double x0);

	void evaluate(ArrayXd& xdata, ArrayXXd& coeffs,vector<interpolator*>& values);

	void tensionInterpolation(vector<Coordinate>* pCS, double* h, double sigma, vector<interpolator*>& vecRefer, double* start_derivative, double* end_derivative);
};