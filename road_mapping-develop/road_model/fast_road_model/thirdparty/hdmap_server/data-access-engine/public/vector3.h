#pragma once
#include <cmath>
#include <algorithm>
#include <limits>
#include <ostream>
#undef min
#undef max

namespace data_access_engine {

const static double ZERO_TOLERANCE = 1e-8;

template <int DIMENSION, typename TYPE>
class Tuple
{
public:
    // Construction and destruction.  The default constructor does not
    // initialize the tuple elements for native elements.  The tuple elements
    // are initialized for class data whenever TYPE initializes during its
    // default construction.
    Tuple();
    Tuple(const Tuple& tuple);
    ~Tuple();

    // Coordinate access.
    inline operator const TYPE* () const;
    inline operator TYPE* ();
    inline const TYPE& operator[] (int i) const;
    inline TYPE& operator[] (int i);

    // Assignment.
    Tuple& operator= (const Tuple& tuple);

    // Comparison.
    bool operator== (const Tuple& tuple) const;
    bool operator!= (const Tuple& tuple) const;
    bool operator<  (const Tuple& tuple) const;
    bool operator<= (const Tuple& tuple) const;
    bool operator>  (const Tuple& tuple) const;
    bool operator>= (const Tuple& tuple) const;

protected:
    TYPE mTuple[DIMENSION];
};

template <typename Real>
class Vector3 : public Tuple<3, Real>
{
public:
    // Construction.
    Vector3();  // uninitialized
    Vector3(const Vector3& vec);
    Vector3(const Tuple<3, Real>& tuple);
    Vector3(Real x, Real y, Real z);

    // Assignment.
    Vector3& operator= (const Vector3& vec);
    Vector3& operator= (const Tuple<3, Real>& tuple);

    // Coordinate access.
    inline Real X() const;
    inline Real& X();
    inline Real Y() const;
    inline Real& Y();
    inline Real Z() const;
    inline Real& Z();

    // Arithmetic operations.
    inline Vector3 operator+ (const Vector3& vec) const;
    inline Vector3 operator- (const Vector3& vec) const;
    inline Vector3 operator* (Real scalar) const;
    inline Vector3 operator/ (Real scalar) const;
    inline Vector3 operator- () const;

    // Arithmetic updates.
    inline Vector3& operator+= (const Vector3& vec);
    inline Vector3& operator-= (const Vector3& vec);
    inline Vector3& operator*= (Real scalar);
    inline Vector3& operator/= (Real scalar);

    // Vector3 operations.
    inline Real Length() const;
    inline Real SquaredLength() const;
    inline Real Dot(const Vector3& vec) const;
    inline Real Normalize(const Real epsilon = ZERO_TOLERANCE);

    // Compute the axis-aligned bounding box of the points.
    static void ComputeExtremes(int numVectors, const Vector3* vectors,
        Vector3& vmin, Vector3& vmax);

    // The cross products are computed using the right-handed rule.  Be aware
    // that some graphics APIs use a left-handed rule.  If you have to compute
    // a cross product with these functions and send the result to the API
    // that expects left-handed, you will need to change sign on the vector
    // (replace each component value c by -c).
    Vector3 Cross(const Vector3& vec) const;
    Vector3 UnitCross(const Vector3& vec) const;

    // Special vectors.
    static const Vector3 ZERO;    // (0,0,0)
    static const Vector3 UNIT_X;  // (1,0,0)
    static const Vector3 UNIT_Y;  // (0,1,0)
    static const Vector3 UNIT_Z;  // (0,0,1)
    static const Vector3 ONE;     // (1,1,1)

protected:
    using Tuple<3, Real>::mTuple;
};

// Arithmetic operations.
template <typename Real>
inline Vector3<Real> operator* (Real scalar, const Vector3<Real>& vec);

// Debugging output.
template <typename Real>
std::ostream& operator<< (std::ostream& outFile, const Vector3<Real>& vec);

#include "vector3.hpp"

typedef Vector3<float> Vector3F;
typedef Vector3<double> Vector3D;

}; // data_access_engine
