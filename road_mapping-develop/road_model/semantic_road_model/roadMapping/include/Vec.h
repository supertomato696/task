//
//
//

#pragma once
#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <iomanip>
#include <cmath>
#include <cfloat>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

namespace hdmap_build
{
    class Vec2
    {
        friend class boost::serialization::access;
        template <typename Archive>
        void serialize(Archive &archive, const unsigned int version)
        {
            archive & x & y;
        }

    public:
        double x, y;

        Vec2(const double &n = 0) : x(n), y(n) {}
        Vec2(const double &x, const double &y) : x(x), y(y) {}

        bool operator==(const Vec2 &vec) const
        {
            return (fabs(x - vec.x) <= DBL_EPSILON) && (fabs(y - vec.y) <= DBL_EPSILON);
        }
        bool operator!=(const Vec2 &vec) const
        {
            return !(*this == vec);
        }

        // Vec + Vec
        Vec2 operator+(const Vec2 &vec) const
        {
            return Vec2(x + vec.x, y + vec.y);
        }
        Vec2 operator-(const Vec2 &vec) const
        {
            return Vec2(x - vec.x, y - vec.y);
        }
        Vec2 operator*(const Vec2 &vec) const
        {
            return Vec2(x * vec.x, y * vec.y);
        }
        Vec2 operator/(const Vec2 &vec) const
        {
            return Vec2(x / vec.x, y / vec.y);
        }
        Vec2 operator+=(const Vec2 &vec)
        {
            x += vec.x;
            y += vec.y;
            return *this;
        }
        Vec2 operator-=(const Vec2 &vec)
        {
            x -= vec.x;
            y -= vec.y;
            return *this;
        }
        Vec2 operator*=(const Vec2 &vec)
        {
            x *= vec.x;
            y *= vec.y;
            return *this;
        }
        Vec2 operator/=(const Vec2 &vec)
        {
            x /= vec.x;
            y /= vec.y;
            return *this;
        }

        // Vec + double
        Vec2 operator+(const double &value) const
        {
            return Vec2(x + value, y + value);
        }
        Vec2 operator-(const double &value) const
        {
            return Vec2(x - value, y - value);
        }
        Vec2 operator*(const double &value) const
        {
            return Vec2(x * value, y * value);
        }
        Vec2 operator/(const double &value) const
        {
            return Vec2(x / value, y / value);
        }
        Vec2 operator+=(const double &value)
        {
            x += value;
            y += value;
            return *this;
        }
        Vec2 operator-=(const double &value)
        {
            x -= value;
            y -= value;
            return *this;
        }
        Vec2 operator*=(const double &value)
        {
            x *= value;
            y *= value;
            return *this;
        }
        Vec2 operator/=(const double &value)
        {
            x /= value;
            y /= value;
            return *this;
        }

        double dist(Vec2 b) const
        {
            return sqrt(pow(x - b.x, 2) + pow(y - b.y, 2));
        }

        double dot(Vec2 b) const
        {
            return x * b.x + y * b.y;
        }

        double norm() const
        {
            return sqrt(pow(x, 2) + pow(y, 2));
        }

        void normalize()
        {
            double length = this->norm();
            if (length <= 1e-5)
            {
                return;
            }
            if (length > 0.0)
            {
                x = x / length;
                y = y / length;
            }
        }

        friend std::ostream &operator<<(std::ostream &os, const Vec2 &vec)
        {
            os << vec.x << ", " << vec.y;
            return os;
        }
    };

    class Vec3
    {
        friend class boost::serialization::access;
        template <typename Archive>
        void serialize(Archive &archive, const unsigned int version)
        {
            archive & x & y & z;
        }

    public:
        double x, y, z;

        Vec3(const double &n = 0) : x(n), y(n), z(n) {};
        Vec3(const double &x, const double &y, const double &z) : x(x), y(y), z(z) {};

        bool operator==(const Vec3 &vec) const
        {
            return (fabs(x - vec.x) <= DBL_EPSILON) && (fabs(y - vec.y) <= DBL_EPSILON) && (fabs(z - vec.z) <= DBL_EPSILON);
        }
        bool operator!=(const Vec3 &vec) const
        {
            return !(*this == vec);
        }

        // Vec3 + Vec3
        Vec3 operator+(const Vec3 &vec) const
        {
            return Vec3(x + vec.x, y + vec.y, z + vec.z);
        }

        Vec3 operator-(const Vec3 &vec) const
        {
            return Vec3(x - vec.x, y - vec.y, z - vec.z);
        }

        Vec3 operator*(const Vec3 &vec) const
        {
            return Vec3(x * vec.x, y * vec.y, z * vec.z);
        }

        Vec3 operator/(const Vec3 &vec) const
        {
            return Vec3(x / vec.x, y / vec.y, z / vec.z);
        }

        Vec3 operator+=(const Vec3 &vec)
        {
            x += vec.x;
            y += vec.y;
            z += vec.z;
            return *this;
        }

        Vec3 operator-=(const Vec3 &vec)
        {
            x -= vec.x;
            y -= vec.y;
            z -= vec.z;
            return *this;
        }

        Vec3 operator*=(const Vec3 &vec)
        {
            x *= vec.x;
            y *= vec.y;
            z *= vec.z;
            return *this;
        }

        Vec3 operator/=(const Vec3 &vec)
        {
            x /= vec.x;
            y /= vec.y;
            z /= vec.z;
            return *this;
        }

        // Vec + double
        Vec3 operator+(const double &value) const
        {
            return Vec3(x + value, y + value, z + value);
        }

        Vec3 operator-(const double &value) const
        {
            return Vec3(x - value, y - value, z - value);
        }

        Vec3 operator*(const double &value) const
        {
            return Vec3(x * value, y * value, z * value);
        }

        Vec3 operator/(const double &value) const
        {
            return Vec3(x / value, y / value, z / value);
        }

        Vec3 operator+=(const double &value)
        {
            x += value;
            y += value;
            z += value;
            return *this;
        }

        Vec3 operator-=(const double &value)
        {
            x -= value;
            y -= value;
            z -= value;
            return *this;
        }

        Vec3 operator*=(const double &value)
        {
            x *= value;
            y *= value;
            z *= value;
            return *this;
        }

        Vec3 operator/=(const double &value)
        {
            x /= value;
            y /= value;
            z /= value;
            return *this;
        }

        Vec3 cross(Vec3 b) const
        {
            return Vec3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
        }

        double dot(Vec3 b) const
        {
            return x * b.x + y * b.y + z * b.z;
        }

        double norm() const
        {
            return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
        }

        double angleTo(Vec3 vec, Vec3 perp) const
        {
            // 避免出现nan
            if (this->norm() <= 1e-5 || vec.norm() <= 1e-5)
            {
                return 0;
            }
            double d = this->dot(vec) / (this->norm() * vec.norm());
            if (d > 1)
            {
                d = 1.0;
            }
            if (d < -1)
            {
                d = -1.0;
            }
            double angle = acos(d);
            if (vec.dot(perp) < 0)
            {
                angle = 2.0f * std::acos(-1.0f) - angle;
            }
            return angle;
        }

        // 简化上述函数
        double angleTo(Vec3 vec) const
        {
            // 避免出现nan
            if (this->norm() <= 1e-5 || vec.norm() <= 1e-5)
            {
                return 0;
            }
            double d = this->dot(vec) / (this->norm() * vec.norm());
            if (d > 1)
            {
                d = 1.0;
            }
            if (d < -1)
            {
                d = -1.0;
            }
            return acos(d);
        }

        // 正则化
        void Normalization()
        {
            double length = this->norm();
            if (length <= 1e-5)
            {
                return;
            }
            if (length > 0.0)
            {
                x = x / length;
                y = y / length;
                z = z / length;
            }
        }

        double dist(Vec3 b) const
        {
            return sqrt(pow(x - b.x, 2) + pow(y - b.y, 2) + pow(z - b.z, 2));
        }
        double distXY(Vec3 b) const
        {
            return sqrt(pow(x - b.x, 2) + pow(y - b.y, 2));
        }
        Vec2 toVec2() const
        {
            return Vec2(x, y);
        }

        friend std::ostream &operator<<(std::ostream &os, const Vec3 &vec)
        {
            os << vec.x << ", " << vec.y << ", " << vec.z;
            return os;
        }

        static std::vector<Vec2> toVec2(const std::vector<Vec3> vec3s)
        {
            std::vector<Vec2> vec2s;
            vec2s.reserve(vec3s.size());
            for (auto vec : vec3s)
            {
                vec2s.push_back(vec.toVec2());
            }
            return vec2s;
        }
    };

    class Vec4
    {
        friend class boost::serialization::access;
        template <typename Archive>
        void serialize(Archive &archive, const unsigned int version)
        {
            archive & x & y & z & w;
        }

    public:
        double x, y, z, w;

        Vec4(const double &n = 0) : x(n), y(n), z(n), w(n) {};
        Vec4(const double &x, const double &y, const double &z, const double &w) : x(x), y(y), z(z), w(w) {};

        bool operator==(const Vec4 &vec) const
        {
            return (fabs(x - vec.x) <= DBL_EPSILON) && (fabs(y - vec.y) <= DBL_EPSILON) && (fabs(z - vec.z) <= DBL_EPSILON) && (fabs(w - vec.w) <= DBL_EPSILON);
        }
        bool operator!=(const Vec4 &vec) const
        {
            return !(*this == vec);
        }

        // Vec4 + Vec4
        Vec4 operator+(const Vec4 &vec) const
        {
            return Vec4(x + vec.x, y + vec.y, z + vec.z, w + vec.w);
        }

        Vec4 operator-(const Vec4 &vec) const
        {
            return Vec4(x - vec.x, y - vec.y, z - vec.z, w - vec.w);
        }

        Vec4 operator*(const Vec4 &vec) const
        {
            return Vec4(x * vec.x, y * vec.y, z * vec.z, w * vec.w);
        }

        Vec4 operator/(const Vec4 &vec) const
        {
            return Vec4(x / vec.x, y / vec.y, z / vec.z, w / vec.w);
        }

        // Vec4 += Vec4
        Vec4 operator+=(const Vec4 &vec)
        {
            x += vec.x;
            y += vec.y;
            z += vec.z;
            w += vec.w;
            return *this;
        }

        Vec4 operator-=(const Vec4 &vec)
        {
            x -= vec.x;
            y -= vec.y;
            z -= vec.z;
            w -= vec.w;
            return *this;
        }

        Vec4 operator*=(const Vec4 &vec)
        {
            x *= vec.x;
            y *= vec.y;
            z *= vec.z;
            w *= vec.w;
            return *this;
        }

        Vec4 operator/=(const Vec4 &vec)
        {
            x /= vec.x;
            y /= vec.y;
            z /= vec.z;
            w /= vec.w;
            return *this;
        }

        // Vec4 + double
        Vec4 operator+(const double &value) const
        {
            return Vec4(x + value, y + value, z + value, w + value);
        }

        Vec4 operator-(const double &value) const
        {
            return Vec4(x - value, y - value, z - value, w - value);
        }

        Vec4 operator*(const double &value) const
        {
            return Vec4(x * value, y * value, z * value, w * value);
        }

        Vec4 operator/(const double &value) const
        {
            return Vec4(x / value, y / value, z / value, w / value);
        }

        // Vec4 += double
        Vec4 operator+=(const double &value)
        {
            x += value;
            y += value;
            z += value;
            w += value;
            return *this;
        }

        Vec4 operator-=(const double &value)
        {
            x -= value;
            y -= value;
            z -= value;
            w -= value;
            return *this;
        }

        Vec4 operator*=(const double &value)
        {
            x *= value;
            y *= value;
            z *= value;
            w *= value;
            return *this;
        }

        Vec4 operator/=(const double &value)
        {
            x /= value;
            y /= value;
            z /= value;
            w /= value;
            return *this;
        }

        friend std::ostream &operator<<(std::ostream &os, const Vec4 &vec)
        {
            os << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w;
            return os;
        }
    };

    class Vec4i
    {
        friend class boost::serialization::access;
        template <typename Archive>
        void serialize(Archive &archive, const unsigned int version)
        {
            archive & x & y & z & w;
        }

    public:
        std::int32_t x, y, z, w;

        Vec4i(const std::int32_t &n = 0) : x(n), y(n), z(n), w(n) {};
        Vec4i(const std::int32_t &x, const std::int32_t &y, const std::int32_t &z, const std::int32_t &w) : x(x), y(y), z(z), w(w) {};

        bool operator==(const Vec4i &vec) const
        {
            return x == vec.x && y == vec.y && z == vec.z && w == vec.w;
        }
        bool operator!=(const Vec4i &vec) const
        {
            return !(*this == vec);
        }

        // Vec4i + Vec4i
        Vec4i operator+(const Vec4i &vec) const
        {
            return Vec4i(x + vec.x, y + vec.y, z + vec.z, w + vec.w);
        }

        Vec4i operator-(const Vec4i &vec) const
        {
            return Vec4i(x - vec.x, y - vec.y, z - vec.z, w - vec.w);
        }

        Vec4i operator*(const Vec4i &vec) const
        {
            return Vec4i(x * vec.x, y * vec.y, z * vec.z, w * vec.w);
        }

        Vec4i operator/(const Vec4i &vec) const
        {
            return Vec4i(x / vec.x, y / vec.y, z / vec.z, w / vec.w);
        }

        // Vec4i += Vec4i
        Vec4i operator+=(const Vec4i &vec)
        {
            x += vec.x;
            y += vec.y;
            z += vec.z;
            w += vec.w;
            return *this;
        }

        Vec4i operator-=(const Vec4i &vec)
        {
            x -= vec.x;
            y -= vec.y;
            z -= vec.z;
            w -= vec.w;
            return *this;
        }

        Vec4i operator*=(const Vec4i &vec)
        {
            x *= vec.x;
            y *= vec.y;
            z *= vec.z;
            w *= vec.w;
            return *this;
        }

        Vec4i operator/=(const Vec4i &vec)
        {
            x /= vec.x;
            y /= vec.y;
            z /= vec.z;
            w /= vec.w;
            return *this;
        }

        // Vec4i + std::int32_t
        Vec4i operator+(const std::int32_t &value) const
        {
            return Vec4i(x + value, y + value, z + value, w + value);
        }

        Vec4i operator-(const std::int32_t &value) const
        {
            return Vec4i(x - value, y - value, z - value, w - value);
        }

        Vec4i operator*(const std::int32_t &value) const
        {
            return Vec4i(x * value, y * value, z * value, w * value);
        }

        Vec4i operator/(const std::int32_t &value) const
        {
            return Vec4i(x / value, y / value, z / value, w / value);
        }

        // Vec4i += std::int32_t
        Vec4i operator+=(const std::int32_t &value)
        {
            x += value;
            y += value;
            z += value;
            w += value;
            return *this;
        }

        Vec4i operator-=(const std::int32_t &value)
        {
            x -= value;
            y -= value;
            z -= value;
            w -= value;
            return *this;
        }

        Vec4i operator*=(const std::int32_t &value)
        {
            x *= value;
            y *= value;
            z *= value;
            w *= value;
            return *this;
        }

        Vec4i operator/=(const std::int32_t &value)
        {
            x /= value;
            y /= value;
            z /= value;
            w /= value;
            return *this;
        }

        friend std::ostream &operator<<(std::ostream &os, const Vec4i &vec)
        {
            os << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w;
            return os;
        }
    };
}
