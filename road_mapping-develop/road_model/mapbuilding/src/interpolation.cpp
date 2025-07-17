#include "interpolation.h"

interpolation::interpolation() = default;

interpolation::~interpolation() = default;

void interpolation::univariateInterpolation(vector<Coordinate>* pCS,vector<interpolator*>& values) {
	const size_t pcount = pCS->size();
	ArrayXd xdata(pcount);
	ArrayXd ydata(pcount);
	ArrayXd weights(pcount);
	ArrayXXd coeffs;

	for (size_t i = 0; i < pcount; ++i) {
		xdata[i] = pCS->at(i).x;
		ydata[i] = pCS->at(i).y;
		if (i == 0 || i == pcount - 1) {
			weights[i] = std::numeric_limits<double>::max();
		} else {
			weights[i] = 1;
		}
	}

	makeCurve(xdata,ydata,weights, coeffs);
	evaluate(xdata,coeffs, values);
}

void interpolation::multivariateInterpolation(string& wkt, vector<coordinate_ex*>& coordinate_list) {
	try {
		WKTReader reader;
		auto ls = reader.read<LineString>(wkt);
		int size = ls->getNumPoints();
		if (ls->getLength() < 5 || ls->getNumPoints() < 2) {
			return;
		}

		vector<Coordinate>* pCS = new vector<Coordinate>();;

		for (size_t i = 0; i < size; i++) {
			pCS->emplace_back(ls->getCoordinateN(i));
		}

		double* h = new double[size - 1];
		chordLength(pCS, h);

		vector<Coordinate>* pCS_x = new vector<Coordinate>();
		vector<Coordinate>* pCS_y = new vector<Coordinate>();

		univariateCoordinates(pCS, pCS_x, pCS_y);

		vector<interpolator*> values;
		univariateInterpolation(pCS_x, values);
		univariateInterpolation(pCS_y, values);

		size_t inter_size = values.size() / 2;

		for (size_t i = 0; i < inter_size; i++) {
			double x = values[i]->value;
			double y = values[i + inter_size]->value;
			double curvature = abs(values[i]->dderivative * values[i + inter_size]->derivative - values[i]->derivative * values[i + inter_size]->dderivative) / pow(values[i]->derivative * values[i]->derivative + values[i + inter_size]->derivative * values[i + inter_size]->derivative, 1.5);
			Coordinate pt(x, y);
			if (i == 0 || i == inter_size - 1) {
				coordinate_ex* coor = new coordinate_ex();
				coor->x = x;
				coor->y = y;
				coor->curvature = curvature;
				coordinate_list.emplace_back(coor);
			} else {
				if (curvature == 0) {
					coordinate_ex* coor = new coordinate_ex();
					coor->x = x;
					coor->y = y;
					coor->curvature = curvature;
					coordinate_list.emplace_back(coor);
				} else {
					double arc_length = 2 * M_PI / curvature / 360;
					Coordinate backPoint(coordinate_list.back()->x, coordinate_list.back()->y);
					if (backPoint.distance(pt) > arc_length) {
						coordinate_ex* coor = new coordinate_ex();
						coor->x = x;
						coor->y = y;
						coor->curvature = curvature;
						coordinate_list.emplace_back(coor);
					}
				}
			}
			delete values[i];
			delete values[i + inter_size];
		}

		delete pCS_x;
		delete pCS_y;
		delete[]h;
		delete pCS;
	} catch (...) {
	}
}

void interpolation::chordLength(vector<Coordinate>* pCS, double* h) {
	size_t n = pCS->size();
	for (size_t i = 0; i < n - 1; i++) {
		Coordinate pt1 = pCS->at(i);
		Coordinate pt2 = pCS->at(i + 1);
		h[i] = pt1.distance(pt2);
	}
}

void interpolation::univariateCoordinates(vector<Coordinate>* pCS, vector<Coordinate>* pCS_x, vector<Coordinate>* pCS_y) {
	size_t n = pCS->size();
	for (size_t i = 0; i < n; i++) {
		double x = pCS->at(i).x;
		double y = pCS->at(i).y;
		double sum = 0;
		for (size_t j = 0; j < i; j++) {
			Coordinate pt1 = pCS->at(j);
			Coordinate pt2 = pCS->at(j + 1);
			sum += pt1.distance(pt2);
		}
		pCS_x->emplace_back(Coordinate(sum, x));
		pCS_y->emplace_back(Coordinate(sum, y));
	}
}

ArrayXd interpolation::diff(const ArrayXd& vec) {
	const DenseIndex n = vec.size() - 1;
	return vec.tail(n) - vec.head(n);
}

Array<Index,Dynamic,1> interpolation::digitize(const ArrayXd& arr, const ArrayXd& bins) {
	Array<Index, Dynamic, 1> indexes = Array<Index, Dynamic, 1>::Ones(arr.size());

	auto IsInsideBin = [arr, bins](Index item, Index index) {
		const double prc = 1.e-8;

		double a = arr(item);
		double bl = bins(index - 1);
		double br = bins(index);
		return (a > bl || std::abs(a - bl) < std::abs(std::min(a, bl)) * prc) && a < br;
	};

	DenseIndex kstart = 1;

	for (DenseIndex i = 0; i < arr.size(); ++i) {
		for (DenseIndex k = kstart; k < bins.size(); ++k) {
			if (IsInsideBin(i, k)) {
				indexes(i) = k;
				kstart = k;
				break;
			}
		}
	}

	return indexes;
}

SparseMatrix<double, ColMajor, DenseIndex> interpolation::makeSparseMatrix(const ArrayXXd& diags, const Array<Index,Dynamic, 1>& offsets, DenseIndex rows, DenseIndex cols) {
	auto getNumElemsAndIndex = [rows, cols](DenseIndex offset, DenseIndex& i, DenseIndex& j) {
		if (offset < 0) {
			i = -offset;
			j = 0;
		} else {
			i = 0;
			j = offset;
		}

		return std::min(rows - i, cols - j);
	};

	SparseMatrix<double,ColMajor, DenseIndex> m(rows, cols);

	for (Index k = 0; k < offsets.size(); ++k) {
		DenseIndex offset = offsets(k);
		DenseIndex i, j;

		DenseIndex n = getNumElemsAndIndex(offset, i, j);
		ArrayXd diag(n);

		if (offset < 0) {
			if (rows >= cols) {
				diag = diags.row(k).head(n);
			} else {
				diag = diags.row(k).tail(n);
			}
		} else {
			if (rows >= cols) {
				diag = diags.row(k).tail(n);
			} else {
				diag = diags.row(k).head(n);
			}
		}

		for (DenseIndex l = 0; l < n; ++l) {
			m.insert(i + l, j + l) = diag(l);
		}
	}

	return m;
}

ArrayXd interpolation::solve(const SparseMatrix<double, ColMajor, DenseIndex>& A, const ArrayXd& b) {
	SparseLU<SparseMatrix<double, ColMajor, DenseIndex>> solver;

	solver.analyzePattern(A);
	solver.factorize(A);
	return solver.solve(b.matrix()).array();
}

void interpolation::makeCurve(ArrayXd& xdata, const ArrayXd& ydata, ArrayXd& weights, ArrayXXd& coeffs) {
	const DenseIndex pcount = xdata.size();
	const DenseIndex pcount_m1 = pcount - 1;
	const DenseIndex pcount_m2 = pcount - 2;

	ArrayXd dx = diff(xdata);
	ArrayXd dy = diff(ydata);
	ArrayXd divdydx = dy / dx;

	double p = 0.0006;

	const DenseIndex n = dx.size() - 1;

	ArrayXXd diags(3, n);

	ArrayXd head_r = dx.head(n);
	ArrayXd tail_r = dx.tail(n);

	diags.row(0) = tail_r;
	diags.row(1) = 2 * (tail_r + head_r);
	diags.row(2) = head_r;

	Array<DenseIndex,Dynamic,1> offsets(3);
	offsets << -1, 0, 1;
	SparseMatrix<double, ColMajor, DenseIndex> r = makeSparseMatrix(diags, offsets, pcount_m2, pcount_m2);

	ArrayXd odx = 1. / dx;
	ArrayXd head_qt = odx.head(n);
	ArrayXd tail_qt = odx.tail(n);

	diags.row(0) = head_qt;
	diags.row(1) = -(tail_qt + head_qt);
	diags.row(2) = tail_qt;

	offsets << 0, 1, 2;
	SparseMatrix<double,ColMajor, DenseIndex> qt = makeSparseMatrix(diags, offsets, pcount_m2, pcount);

	ArrayXd ow = 1. / weights;
	ArrayXd osqw = 1. / weights.sqrt();

	offsets.resize(1);
	offsets << 0;

	SparseMatrix<double, ColMajor, DenseIndex> w = makeSparseMatrix(ow.transpose(), offsets, pcount, pcount);
	SparseMatrix<double, ColMajor, DenseIndex> qw = makeSparseMatrix(osqw.transpose(), offsets, pcount, pcount);

	SparseMatrix<double, ColMajor, DenseIndex> qtw = qt * qw;
	SparseMatrix<double, ColMajor, DenseIndex> qtwq = qtw * qtw.transpose();

	auto Trace = [](const SparseMatrix<double, ColMajor, DenseIndex>& m) {
		return m.diagonal().sum();
		};

	if (p < 0) {
		p = 1. / (1. + Trace(r) / (6. * Trace(qtwq)));
	}

	SparseMatrix<double, ColMajor, DenseIndex> A = ((6. * (1. - p)) * qtwq) + (p * r);
	A.makeCompressed();

	ArrayXd b = diff(divdydx);

	//Ab = u
	ArrayXd u = solve(A, b);

	ArrayXd d1 = ArrayXd::Zero(u.size() + 2);
	d1.segment(1, u.size()) = u; d1 = diff(d1) / dx;

	ArrayXd d2 = ArrayXd::Zero(d1.size() + 2);
	d2.segment(1, d1.size()) = d1; d2 = diff(d2);

	ArrayXd yi = ydata - ((6. * (1. - p)) * w * d2.matrix()).array();

	ArrayXd c3 = ArrayXd::Zero(u.size() + 2);
	c3.segment(1, u.size()) = p * u;

	ArrayXd c2 = diff(yi) / dx - dx * (2. * c3.head(pcount_m1) + c3.tail(pcount_m1));

	coeffs.resize(pcount_m1, 4);

	coeffs.col(0) = diff(c3) / dx;
	coeffs.col(1) = 3. * c3.head(pcount_m1);
	coeffs.col(2) = c2;
	coeffs.col(3) = yi.head(pcount_m1);
}

Array<Index, Dynamic, 1> interpolation::getIndex(ArrayXd& xdata, ArrayXd& xidata) {
	const DenseIndex x_size = xdata.size();

	ArrayXd mesh = xdata.segment(1, x_size - 2);
	ArrayXd edges(x_size);

	edges(0) = -std::numeric_limits<double>::infinity();
	edges.segment(1, x_size - 2) = mesh;
	edges(x_size - 1) = std::numeric_limits<double>::infinity();

	return digitize(xidata, edges) - 1;
}
double interpolation::optimFunction(double sigma, double k1, double k2, double x1, double x2, double h, double x, double y1, double y2) {
	return k1 * sinh(sigma * (x2 - x)) / sinh(sigma * h) + k2 * sinh(sigma * (x - x1)) / sinh(sigma * h) + (y1 - k1) * (x2 - x) / h + (y2 - k2) * (x - x1) / h;
}
double interpolation::optimFunction(double a, double b, double c, double d, double x, double x0) {
	return a * pow(x - x0, 3) + b * pow(x - x0, 2) + c * (x - x0) + d;
}

double interpolation::derivative(double a, double b, double c, double x, double x0) {
	return 3 * a * pow(x - x0, 2) + 2 * b * (x - x0) + c;
}

double interpolation::dderivative(double a, double b, double x, double x0) {
	return 6 * a * (x - x0) + 2 * b;
}

void interpolation::evaluate(ArrayXd& xdata, ArrayXXd& coeffs, vector<interpolator*>& values) {
	size_t xidata_size = xdata[xdata.size() - 1] / 0.5;

	ArrayXd xidata;
	xidata.resize(xidata_size);
	xidata << ArrayXd::LinSpaced(xidata_size, xdata(0), xdata(xdata.size() - 1));

	Array<Index,Dynamic, 1> indexes = getIndex(xdata, xidata);

	for (Index i = 0; i < xidata_size; ++i) {
		Index index = indexes(i);
		double x0 = xdata(index);
		double a = coeffs(index, 0);
		double b = coeffs(index, 1);
		double c = coeffs(index, 2);
		double d = coeffs(index, 3);
		double x = xidata[i];
		double y = optimFunction(a, b, c, d, x, x0);
		double first_d = derivative(a, b, c, x, x0);
		double second_d = dderivative(a, b, x, x0);
		interpolator* inter_value = new interpolator();
		inter_value->value = y;
		inter_value->derivative = first_d;
		inter_value->dderivative = second_d;
		values.emplace_back(inter_value);
	}
}
void interpolation::tensionInterpolation(vector<Coordinate>* pCS, double* h, double sigma, vector<interpolator*>& vecRefer, double* start_derivative, double* end_derivative) {
	size_t n = pCS->size();
	double* a = new double[n];
	double* b = new double[n];
	double* c = new double[n];
	double* d = new double[n];

	double start_k = 0;
	double end_k = 0;
	if (start_derivative == NULL) {
		start_k = (pCS->at(1).y - pCS->at(0).y) / h[0];
	} else {
		start_k = *start_derivative;
	}

	if (end_derivative == NULL) {
		end_k = (pCS->at(n - 1).y - pCS->at(n - 2).y) / h[n - 2];
	} else {
		end_k = *end_derivative;
	}

	b[0] = sigma * cosh(sigma * h[0]) / sinh(sigma * h[0]) - 1 / h[0];
	c[0] = 1 / h[0] - sigma / sinh(sigma * h[0]);
	d[0] = (pCS->at(1).y - pCS->at(0).y) / h[0] - start_k;

	for (size_t i = 1; i < n - 1; i++) {
		a[i] = 1 / h[i - 1] - sigma / sinh(sigma * h[i - 1]);
		b[i] = sigma * cosh(sigma * h[i - 1]) / sinh(sigma * h[i - 1]) - 1 / h[i - 1] + sigma * cosh(sigma * h[i]) / sinh(sigma * h[i]) - 1 / h[i];
		c[i] = 1 / h[i] - sigma / sinh(sigma * h[i]);
		d[i] = (pCS->at(i + 1).y - pCS->at(i).y) / h[i] - (pCS->at(i).y - pCS->at(i - 1).y) / h[i - 1];
	}

	a[n - 1] = 1 / h[n - 2] - sigma / sinh(sigma * h[n - 2]);
	b[n - 1] = sigma * cosh(sigma * h[n - 2]) / sinh(sigma * h[n - 2]) - 1 / h[n - 2];
	d[n - 1] = end_k - (pCS->at(n - 1).y - pCS->at(n - 2).y) / h[n - 2];

	VectorXd B(n);
	MatrixXd A(n, n);

	A = MatrixXd::Zero(n, n);
	B = VectorXd::Zero(n);

	A(0, 0) = b[0];
	A(0, 1) = c[0];

	for (size_t i = 1; i < n - 1; i++) {
		A(i, i - 1) = a[i];
		A(i, i) = b[i];
		A(i, i + 1) = c[i];
	}

	A(n - 1, n - 2) = a[n - 1];
	A(n - 1, n - 1) = b[n - 1];

	for (size_t i = 0; i < n; i++) {
		B(i, 0) = d[i];
	}

	VectorXd solution = A.lu().solve(B);

	double step = 0.06;
	double offset = 0;
	for (size_t i = 0; i < n - 1; i++) {
		double x1 = pCS->at(i).x;
		double y1 = pCS->at(i).y;

		double x2 = pCS->at(i + 1).x;
		double y2 = pCS->at(i + 1).y;

		double cur_x = 0;
		if (i == 0) {
			cur_x = x1;
		} else {
			cur_x = x1 + offset;
		}

		bool flag = false;
		bool equal = false;

		while (true) {
			if (cur_x >= x2) {
				if (cur_x == x2) {
					equal = true;
					offset = step;
				} else {
					offset = cur_x - x2;
				}
				cur_x = x2;
				flag = true;
			}

			double linear_value = 0;
			double tension_value = optimFunction(sigma, solution[i], solution[i + 1], x1, x2, h[i], cur_x, y1, y2);
			double first_derivative = 0;
			double second_derivative = 0;
			interpolator* pValue = new interpolator();
			pValue->value = tension_value;
			pValue->derivative = first_derivative;
			pValue->dderivative = second_derivative;

			size_t beforeSize = vecRefer.size();
			if (flag) {
				if (equal) {
					vecRefer.push_back(pValue);
				} else {
					if (i == n - 2) {
						vecRefer.push_back(pValue);
					}
				}
			} else {
				vecRefer.push_back(pValue);
			}

			if (beforeSize == vecRefer.size()) {
				delete pValue;
			}

			cur_x += step;
			if (flag) {
				break;
			}
		}
	}

	delete[]a;
	delete[]b;
	delete[]c;
	delete[]d;
}