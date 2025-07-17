//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
Tuple<DIMENSION, TYPE>::Tuple()
{
    for (int i = 0; i < DIMENSION; ++i) {
        mTuple[i] = std::numeric_limits<TYPE>::quiet_NaN();
    }
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
Tuple<DIMENSION, TYPE>::Tuple(const Tuple& tuple)
{
    for (int i = 0; i < DIMENSION; ++i)
    {
        mTuple[i] = tuple.mTuple[i];
    }
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
Tuple<DIMENSION, TYPE>::~Tuple()
{
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
inline Tuple<DIMENSION, TYPE>::operator const TYPE* () const
{
    return mTuple;
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
inline Tuple<DIMENSION, TYPE>::operator TYPE* ()
{
    return mTuple;
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
inline const TYPE& Tuple<DIMENSION, TYPE>::operator[] (int i) const
{
    return mTuple[i];
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
inline TYPE& Tuple<DIMENSION, TYPE>::operator[] (int i)
{
    return mTuple[i];
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
Tuple<DIMENSION, TYPE>& Tuple<DIMENSION, TYPE>::operator= (const Tuple& tuple)
{
    for (int i = 0; i < DIMENSION; ++i)
    {
        mTuple[i] = tuple.mTuple[i];
    }
    return *this;
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
bool Tuple<DIMENSION, TYPE>::operator== (const Tuple& tuple) const
{
    for (int i = 0; i < DIMENSION; ++i)
    {
        if (mTuple[i] != tuple.mTuple[i])
        {
            return false;
        }
    }
    return true;
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
bool Tuple<DIMENSION, TYPE>::operator!= (const Tuple& tuple) const
{
    for (int i = 0; i < DIMENSION; ++i)
    {
        if (mTuple[i] != tuple.mTuple[i])
        {
            return true;
        }
    }
    return false;
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
bool Tuple<DIMENSION, TYPE>::operator< (const Tuple& tuple) const
{
    for (int i = 0; i < DIMENSION; ++i)
    {
        if (mTuple[i] < tuple.mTuple[i])
        {
            return true;
        }

        if (mTuple[i] > tuple.mTuple[i])
        {
            return false;
        }
    }
    return false;
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
bool Tuple<DIMENSION, TYPE>::operator<= (const Tuple& tuple) const
{
    for (int i = 0; i < DIMENSION; ++i)
    {
        if (mTuple[i] < tuple.mTuple[i])
        {
            return true;
        }

        if (mTuple[i] > tuple.mTuple[i])
        {
            return false;
        }
    }
    return true;
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
bool Tuple<DIMENSION, TYPE>::operator> (const Tuple& tuple) const
{
    for (int i = 0; i < DIMENSION; ++i)
    {
        if (mTuple[i] > tuple.mTuple[i])
        {
            return true;
        }

        if (mTuple[i] < tuple.mTuple[i])
        {
            return false;
        }
    }
    return false;
}
//----------------------------------------------------------------------------
template <int DIMENSION, typename TYPE>
bool Tuple<DIMENSION, TYPE>::operator>= (const Tuple& tuple) const
{
    for (int i = 0; i < DIMENSION; ++i)
    {
        if (mTuple[i] > tuple.mTuple[i])
        {
            return true;
        }

        if (mTuple[i] < tuple.mTuple[i])
        {
            return false;
        }
    }
    return true;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
template <typename Real>
Vector3<Real>::Vector3()
{
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3<Real>::Vector3(const Vector3& vec)
{

    mTuple[0] = vec.mTuple[0];
    mTuple[1] = vec.mTuple[1];
    mTuple[2] = vec.mTuple[2];
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3<Real>::Vector3(const Tuple<3, Real>& tuple)
{
    mTuple[0] = tuple[0];
    mTuple[1] = tuple[1];
    mTuple[2] = tuple[2];
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3<Real>::Vector3(Real x, Real y, Real z)
{
    mTuple[0] = x;
    mTuple[1] = y;
    mTuple[2] = z;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3<Real>& Vector3<Real>::operator= (const Vector3& vec)
{
    mTuple[0] = vec.mTuple[0];
    mTuple[1] = vec.mTuple[1];
    mTuple[2] = vec.mTuple[2];
    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3<Real>& Vector3<Real>::operator= (const Tuple<3, Real>& tuple)
{
    mTuple[0] = tuple[0];
    mTuple[1] = tuple[1];
    mTuple[2] = tuple[2];
    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real Vector3<Real>::X() const
{
    return mTuple[0];
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real& Vector3<Real>::X()
{
    return mTuple[0];
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real Vector3<Real>::Y() const
{
    return mTuple[1];
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real& Vector3<Real>::Y()
{
    return mTuple[1];
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real Vector3<Real>::Z() const
{
    return mTuple[2];
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real& Vector3<Real>::Z()
{
    return mTuple[2];
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real> Vector3<Real>::operator+ (const Vector3& vec) const
{
    return Vector3
        (
        mTuple[0] + vec.mTuple[0],
        mTuple[1] + vec.mTuple[1],
        mTuple[2] + vec.mTuple[2]
        );
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real> Vector3<Real>::operator- (const Vector3& vec) const
{
    return Vector3
        (
        mTuple[0] - vec.mTuple[0],
        mTuple[1] - vec.mTuple[1],
        mTuple[2] - vec.mTuple[2]
        );
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real> Vector3<Real>::operator* (Real scalar) const
{
    return Vector3
        (
        scalar*mTuple[0],
        scalar*mTuple[1],
        scalar*mTuple[2]
        );
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real> Vector3<Real>::operator/ (Real scalar) const
{
    Vector3 result;

    if (scalar != (Real)0)
    {
        Real invScalar = ((Real)1) / scalar;
        result.mTuple[0] = invScalar*mTuple[0];
        result.mTuple[1] = invScalar*mTuple[1];
        result.mTuple[2] = invScalar*mTuple[2];
    }
    else
    {
        result.mTuple[0] = std::numeric_limits<Real>::max();
        result.mTuple[1] = std::numeric_limits<Real>::max();
        result.mTuple[2] = std::numeric_limits<Real>::max();
    }

    return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real> Vector3<Real>::operator- () const
{
    return Vector3
        (
        -mTuple[0],
        -mTuple[1],
        -mTuple[2]
        );
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real>& Vector3<Real>::operator+= (const Vector3& vec)
{
    mTuple[0] += vec.mTuple[0];
    mTuple[1] += vec.mTuple[1];
    mTuple[2] += vec.mTuple[2];
    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real>& Vector3<Real>::operator-= (const Vector3& vec)
{
    mTuple[0] -= vec.mTuple[0];
    mTuple[1] -= vec.mTuple[1];
    mTuple[2] -= vec.mTuple[2];
    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real>& Vector3<Real>::operator*= (Real scalar)
{
    mTuple[0] *= scalar;
    mTuple[1] *= scalar;
    mTuple[2] *= scalar;
    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real>& Vector3<Real>::operator/= (Real scalar)
{
    if (scalar != (Real)0)
    {
        Real invScalar = ((Real)1) / scalar;
        mTuple[0] *= invScalar;
        mTuple[1] *= invScalar;
        mTuple[2] *= invScalar;
    }
    else
    {
        mTuple[0] = std::numeric_limits<Real>::max();
        mTuple[1] = std::numeric_limits<Real>::max();
        mTuple[2] = std::numeric_limits<Real>::max();
    }

    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real Vector3<Real>::Length() const
{
    return std::sqrt
        (
        mTuple[0] * mTuple[0] +
        mTuple[1] * mTuple[1] +
        mTuple[2] * mTuple[2]
        );
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real Vector3<Real>::SquaredLength() const
{
    return
        mTuple[0] * mTuple[0] +
        mTuple[1] * mTuple[1] +
        mTuple[2] * mTuple[2];
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real Vector3<Real>::Dot(const Vector3& vec) const
{
    return
        mTuple[0] * vec.mTuple[0] +
        mTuple[1] * vec.mTuple[1] +
        mTuple[2] * vec.mTuple[2];
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real Vector3<Real>::Normalize(const Real epsilon)
{
    Real length = Length();

    if (length > epsilon)
    {
        Real invLength = ((Real)1) / length;
        mTuple[0] *= invLength;
        mTuple[1] *= invLength;
        mTuple[2] *= invLength;
    }
    else
    {
        length = (Real)0;
        mTuple[0] = (Real)0;
        mTuple[1] = (Real)0;
        mTuple[2] = (Real)0;
    }

    return length;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3<Real> Vector3<Real>::Cross(const Vector3& vec) const
{
    return Vector3
        (
        mTuple[1] * vec.mTuple[2] - mTuple[2] * vec.mTuple[1],
        mTuple[2] * vec.mTuple[0] - mTuple[0] * vec.mTuple[2],
        mTuple[0] * vec.mTuple[1] - mTuple[1] * vec.mTuple[0]
        );
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3<Real> Vector3<Real>::UnitCross(const Vector3& vec) const
{
    Vector3 cross
        (
        mTuple[1] * vec.mTuple[2] - mTuple[2] * vec.mTuple[1],
        mTuple[2] * vec.mTuple[0] - mTuple[0] * vec.mTuple[2],
        mTuple[0] * vec.mTuple[1] - mTuple[1] * vec.mTuple[0]
        );
    cross.Normalize();
    return cross;
}
//----------------------------------------------------------------------------
template <typename Real>
void Vector3<Real>::ComputeExtremes(int numVectors, const Vector3* vectors,
    Vector3& vmin, Vector3& vmax)
{
    if (numVectors <= 0 || !vectors) {
        return;
    }
    vmin = vectors[0];
    vmax = vmin;
    for (int j = 1; j < numVectors; ++j)
    {
        const Vector3& vec = vectors[j];
        for (int i = 0; i < 3; ++i)
        {
            if (vec[i] < vmin[i])
            {
                vmin[i] = vec[i];
            }
            else if (vec[i] > vmax[i])
            {
                vmax[i] = vec[i];
            }
        }
    }
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector3<Real> operator* (Real scalar, const Vector3<Real>& vec)
{
    return Vector3<Real>
        (
        scalar*vec[0],
        scalar*vec[1],
        scalar*vec[2]
        );
}
//----------------------------------------------------------------------------
template <typename Real>
std::ostream& operator<< (std::ostream& outFile, const Vector3<Real>& vec)
{
    outFile.precision(10);
    return outFile << vec.X() << ' ' << vec.Y() << ' ' << vec.Z();
}
//----------------------------------------------------------------------------
