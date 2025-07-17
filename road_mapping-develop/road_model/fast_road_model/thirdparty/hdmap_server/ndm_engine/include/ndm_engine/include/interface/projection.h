/**
 * @file projection.h
 * @author Fei Han (fei.han@horizon.ai)
 * @brief
 * @version 0.1
 * @date 2019-09-10
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_PROJECTION_H_
#define INTERFACE_PROJECTION_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <string>
#include <algorithm> 

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace map_interface {
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef TWO_PI
#define TWO_PI 6.28318530717958647693
#endif

#ifndef M_TWO_PI
#define M_TWO_PI 6.28318530717958647693
#endif

#ifndef FOUR_PI
#define FOUR_PI 12.56637061435917295385
#endif

#ifndef HALF_PI
#define HALF_PI 1.57079632679489661923
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / M_PI)
#endif

// Grid granularity for rounding UTM coordinates to generate MapXY.
const double grid_size = 100000.0;  /// < 100 km grid

// WGS84 Parameters
#define WGS84_A 6378137.0         /// < major axis
#define WGS84_B 6356752.31424518  ///< minor axis
#define WGS84_F 0.0033528107      ///< ellipsoid flattening
#define WGS84_E 0.0818191908      ///< first eccentricity
#define WGS84_EP 0.0820944379     ///< second eccentricity

// UTM Parameters
#define UTM_K0 0.9996               ///< scale factor
#define UTM_FE 500000.0             ///< false easting
#define UTM_FN_N 0.0                ///< false northing, northern hemisphere
#define UTM_FN_S 10000000.0         ///< false northing, southern hemisphere
#define UTM_E2 (WGS84_E * WGS84_E)  ///< e^2
#define UTM_E4 (UTM_E2 * UTM_E2)    ///< e^4
#define UTM_E6 (UTM_E4 * UTM_E2)    ///< e^6
#define UTM_EP2 (UTM_E2 / (1 - UTM_E2))  ///< e'^2

#define BJ54_A 6378245.0  /// 北京54坐标系长半轴

/**
 * Determine the correct UTM letter designator for the
 * given latitude
 *
 * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
static inline char UTMLetterDesignator(double Lat) {
  char LetterDesignator;

  if ((84 >= Lat) && (Lat >= 72))
    LetterDesignator = 'X';
  else if ((72 > Lat) && (Lat >= 64))
    LetterDesignator = 'W';
  else if ((64 > Lat) && (Lat >= 56))
    LetterDesignator = 'V';
  else if ((56 > Lat) && (Lat >= 48))
    LetterDesignator = 'U';
  else if ((48 > Lat) && (Lat >= 40))
    LetterDesignator = 'T';
  else if ((40 > Lat) && (Lat >= 32))
    LetterDesignator = 'S';
  else if ((32 > Lat) && (Lat >= 24))
    LetterDesignator = 'R';
  else if ((24 > Lat) && (Lat >= 16))
    LetterDesignator = 'Q';
  else if ((16 > Lat) && (Lat >= 8))
    LetterDesignator = 'P';
  else if ((8 > Lat) && (Lat >= 0))
    LetterDesignator = 'N';
  else if ((0 > Lat) && (Lat >= -8))
    LetterDesignator = 'M';
  else if ((-8 > Lat) && (Lat >= -16))
    LetterDesignator = 'L';
  else if ((-16 > Lat) && (Lat >= -24))
    LetterDesignator = 'K';
  else if ((-24 > Lat) && (Lat >= -32))
    LetterDesignator = 'J';
  else if ((-32 > Lat) && (Lat >= -40))
    LetterDesignator = 'H';
  else if ((-40 > Lat) && (Lat >= -48))
    LetterDesignator = 'G';
  else if ((-48 > Lat) && (Lat >= -56))
    LetterDesignator = 'F';
  else if ((-56 > Lat) && (Lat >= -64))
    LetterDesignator = 'E';
  else if ((-64 > Lat) && (Lat >= -72))
    LetterDesignator = 'D';
  else if ((-72 > Lat) && (Lat >= -80))
    LetterDesignator = 'C';
  // 'Z' is an error flag, the Latitude is outside the UTM limits
  else
    LetterDesignator = 'Z';
  return LetterDesignator;
}

static inline int zoneNum(const char* s_zone) {
  if (strlen(s_zone) != 3) {
    return -1;
  }

  char tmp[12] = {0};
  tmp[0] = s_zone[0];
  tmp[1] = s_zone[1];

  return atoi(tmp);
}

static inline int zoneNum(std::string s_zone) {
  return zoneNum(s_zone.c_str());
}

/**
 * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
static inline void LLtoUTMFixZone(const double Lat, const double Long,
                                  double& UTMNorthing, double& UTMEasting,
                                  int ZoneNumber) {
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  // Make sure the longitude is between -180.00 .. 179.9
  double LongTemp =
      (Long + 180) - static_cast<int>((Long + 180) / 360) * 360 - 180;

  double LatRad = Lat * DEG_TO_RAD;
  double LongRad = LongTemp * DEG_TO_RAD;
  double LongOriginRad;

  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
  LongOriginRad = LongOrigin * DEG_TO_RAD;

  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad - LongOriginRad);

  M = a *
      ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 -
        5 * eccSquared * eccSquared * eccSquared / 256) *
           LatRad -
       (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 +
        45 * eccSquared * eccSquared * eccSquared / 1024) *
           sin(2 * LatRad) +
       (15 * eccSquared * eccSquared / 256 +
        45 * eccSquared * eccSquared * eccSquared / 1024) *
           sin(4 * LatRad) -
       (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

  UTMEasting = static_cast<double>(
      k0 * N * (A + (1 - T + C) * A * A * A / 6 +
                (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A *
                    A * A * A / 120) +
      500000.0);

  UTMNorthing = static_cast<double>(
      k0 * (M +
            N * tan(LatRad) *
                (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                 (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A *
                     A * A * A * A * A / 720)));

  if (Lat < 0) {
    // 10000000 meter offset for southern hemisphere
    UTMNorthing += 10000000.0;
  }
}

/**
 * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
static inline void LLtoUTM(const double Lat, const double Long,
                           double& UTMNorthing, double& UTMEasting,
                           char* UTMZone) {
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  // Make sure the longitude is between -180.00 .. 179.9
  double LongTemp =
      (Long + 180) - static_cast<int>((Long + 180) / 360) * 360 - 180;

  double LatRad = Lat * DEG_TO_RAD;
  double LongRad = LongTemp * DEG_TO_RAD;
  double LongOriginRad;
  int ZoneNumber;

  ZoneNumber = static_cast<int>((LongTemp + 180) / 6) + 1;

  if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
    ZoneNumber = 32;

  // Special zones for Svalbard
  if (Lat >= 72.0 && Lat < 84.0) {
    if (LongTemp >= 0.0 && LongTemp < 9.0)
      ZoneNumber = 31;
    else if (LongTemp >= 9.0 && LongTemp < 21.0)
      ZoneNumber = 33;
    else if (LongTemp >= 21.0 && LongTemp < 33.0)
      ZoneNumber = 35;
    else if (LongTemp >= 33.0 && LongTemp < 42.0)
      ZoneNumber = 37;
  }
  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
  LongOriginRad = LongOrigin * DEG_TO_RAD;

  // compute the UTM Zone from the latitude and longitude
  snprintf(UTMZone, sizeof(UTMZone), "%d%c", ZoneNumber,
           UTMLetterDesignator(Lat));

  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad - LongOriginRad);

  M = a *
      ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 -
        5 * eccSquared * eccSquared * eccSquared / 256) *
           LatRad -
       (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 +
        45 * eccSquared * eccSquared * eccSquared / 1024) *
           sin(2 * LatRad) +
       (15 * eccSquared * eccSquared / 256 +
        45 * eccSquared * eccSquared * eccSquared / 1024) *
           sin(4 * LatRad) -
       (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

  UTMEasting = static_cast<double>(
      k0 * N * (A + (1 - T + C) * A * A * A / 6 +
                (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A *
                    A * A * A / 120) +
      500000.0);

  UTMNorthing = static_cast<double>(
      k0 * (M +
            N * tan(LatRad) *
                (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                 (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A *
                     A * A * A * A * A / 720)));

  if (Lat < 0) {
    // 10000000 meter offset for southern hemisphere
    UTMNorthing += 10000000.0;
  }
}

/**
 * Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees.
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
static inline void UTMtoLL(const double UTMNorthing, const double UTMEasting,
                           const char* UTMZone, double& Lat, double& Long) {
  double k0 = UTM_K0;
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double eccPrimeSquared;
  double e1 = (1 - sqrt(1 - eccSquared)) / (1 + sqrt(1 - eccSquared));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1Rad;
  double x, y;
  int ZoneNumber;
  char* ZoneLetter;

  x = UTMEasting - 500000.0;  // remove 500,000 meter offset for longitude
  y = UTMNorthing;

  ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
  if ((*ZoneLetter - 'N') < 0) {
    // remove 10,000,000 meter offset used for southern hemisphere
    y -= 10000000.0;
  }

  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  M = y / k0;
  mu = M / (a * (1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 -
                 5 * eccSquared * eccSquared * eccSquared / 256));

  phi1Rad =
      mu + ((3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu) +
            (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu) +
            (151 * e1 * e1 * e1 / 96) * sin(6 * mu));

  N1 = a / sqrt(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad));
  T1 = tan(phi1Rad) * tan(phi1Rad);
  C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  R1 = a * (1 - eccSquared) /
       pow(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad), 1.5);
  D = x / (N1 * k0);

  Lat = phi1Rad - ((N1 * tan(phi1Rad) / R1) *
                   (D * D / 2 -
                    (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) *
                        D * D * D * D / 24 +
                    (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 -
                     252 * eccPrimeSquared - 3 * C1 * C1) *
                        D * D * D * D * D * D / 720));

  Lat = Lat * RAD_TO_DEG;

  Long = ((D - (1 + 2 * T1 + C1) * D * D * D / 6 +
           (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * eccPrimeSquared +
            24 * T1 * T1) *
               D * D * D * D * D / 120) /
          cos(phi1Rad));
  Long = LongOrigin + Long * RAD_TO_DEG;
}

/**
 * @brief WGS84(LLA) convert to ECEF
 *
 * @param latitude (degree)
 * @param longitude (degree)
 * @param height (meters)
 * @param ECEF_X (meters)
 * @param ECEF_Y (meters)
 * @param ECEF_Z (meters)
 * solution see: https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf
 */
static inline void WGS84toECEF(const double& latitude, const double& longitude,
                               const double& height, double& ECEF_X,
                               double ECEF_Y, double& ECEF_Z) {
  double a = 6378137;                    // earth's semi-major axis
  double b = 6356752.314245;             // earth's semi-minor axis
  double E = (a * a - b * b) / (a * a);  // earth's first eccentricity

  double cos_lat = cos(latitude * M_PI / 180.0);
  double sin_lat = sin(latitude * M_PI / 180.0);

  double cos_long = cos(longitude * M_PI / 180.0);
  double sin_long = sin(longitude * M_PI / 180.0);

  double N =
      a / (sqrt(1 - E * sin_lat * sin_lat));  // Radius of Curvature(meters)
  double NH = N + height;

  ECEF_X = NH * cos_lat * cos_long;
  ECEF_Y = NH * cos_lat * sin_long;
  ECEF_Z = (b * b * N / (a * a) + height) * sin_lat;
}

/**
 * @brief ECEF convert to WGS84
 *
 * @param ECEF_X (meters)
 * @param ECEF_Y (meters)
 * @param ECEF_Z (meters)
 * @param latitude (degree)
 * @param longitude (degree)
 * @param height (meters)
 * solution see: https://microem.ru/files/2012/08/GPS.G1-X-00006.pdf
 */
static inline void ECEFtoWGS84(const double& ECEF_X, const double& ECEF_Y,
                               const double& ECEF_Z, double& latitude,
                               double& longitude, double& height) {
  double a = 6378137;         // earth's semi-major axis
  double b = 6356752.314245;  // earth's semi-minor axis

  double e = sqrt(((a * a) - (b * b)) / (a * a));  // earth first eccentricity
  double e_1 =
      sqrt(((a * a) - (b * b)) / (b * b));  // earth's second eccentricity

  double p = sqrt((ECEF_X * ECEF_X) + (ECEF_Y * ECEF_Y));
  double q = atan2((ECEF_Z * a), (p * b));

  longitude = atan2(ECEF_Y, ECEF_X);
  latitude = atan2((ECEF_Z + (e_1 * e_1) * b * pow(sin(q), 3)),
                   (p - (e * e) * a * pow(cos(q), 3)));

  double N = a / sqrt(1 - ((e * e) * pow(sin(latitude), 2)));

  height = (p / cos(latitude)) - N;
  longitude = longitude * 180.0 / M_PI;
  latitude = latitude * 180.0 / M_PI;
}

/**
 * @brief 判断是否在国内，不在国内经纬度不做偏转
 *
 * @param longitude 经度
 * @param latitude  纬度
 * @return true     不在国内
 * @return false    在国内
 */
static inline bool OutOfChina(const double& longitude, const double& latitude) {
  if (longitude > 73.66 && longitude < 135.05 && latitude > 3.86 &&
      latitude < 53.55) {
    return false;
  } else {
    return true;
  }
}

static inline double TransformLat(const double& longitude,
                                  const double& latitude) {
  double result = 0.0;
  result = -100.0 + 2.0 * longitude + 3.0 * latitude +
           0.2 * latitude * latitude + 0.1 * longitude * latitude +
           0.2 * sqrt(fabs(longitude));
  result += (20.0 * sin(6.0 * longitude * M_PI) +
             20.0 * sin(2.0 * longitude * M_PI)) *
            2.0 / 3.0;
  result += (20.0 * sin(latitude * M_PI) + 40.0 * sin(latitude / 3.0 * M_PI)) *
            2.0 / 3.0;
  result += (160.0 * sin(latitude / 12.0 * M_PI) +
             320 * sin(latitude * M_PI / 30.0)) *
            2.0 / 3.0;
  return result;
}

static inline double TransformLong(const double& longitude,
                                   const double& latitude) {
  double result = 0.0;
  result = 300.0 + longitude + 2.0 * latitude + 0.1 * longitude * longitude +
           0.1 * longitude * latitude + 0.1 * sqrt(fabs(longitude));
  result += (20.0 * sin(6.0 * longitude * M_PI) +
             20.0 * sin(2.0 * longitude * M_PI)) *
            2.0 / 3.0;
  result +=
      (20.0 * sin(longitude * M_PI) + 40.0 * sin(longitude / 3.0 * M_PI)) *
      2.0 / 3.0;
  result += (150.0 * sin(longitude / 12.0 * M_PI) +
             300.0 * sin(longitude / 30.0 * M_PI)) *
            2.0 / 3.0;
  return result;
}

/**
 * @brief WGS84坐标系转火星坐标系(国测局坐标系)
 *
 * @param longitude     WGS84 经度
 * @param latitude      WGS84 纬度
 * @param gcj_longitude 火星坐标系 经度
 * @param gcj_latitude  火星坐标系 纬度
 */
static inline void WGS84toGCJ02(const double& longitude, const double& latitude,
                                double& gcj_longitude, double& gcj_latitude) {
  if (OutOfChina(longitude, latitude)) {
    // 不在国内，则不做偏转
    gcj_longitude = longitude;
    gcj_latitude = latitude;
    return;
  }
  double delta_lat = TransformLat(longitude - 105.0, latitude - 35.0);
  double delta_long = TransformLong(longitude - 105.0, latitude - 35.0);
  double rad_lat = latitude / 180.0 * M_PI;
  double magic = sin(rad_lat);
  magic = 1 - UTM_E2 * magic * magic;
  double sqrt_magic = sqrt(magic);
  delta_lat = (delta_lat * 180.0) /
              ((BJ54_A * (1 - UTM_E2)) / (magic * sqrt_magic) * M_PI);
  delta_long =
      (delta_long * 180.0) / (BJ54_A / sqrt_magic * cos(rad_lat) * M_PI);

  gcj_longitude = longitude + delta_long;
  gcj_latitude = latitude + delta_lat;
}

/**
 * @brief 火星坐标系转WGS84坐标系
 *
 * @param gcj_longitude 火星坐标系 经度
 * @param gcj_latitude  火星坐标系 纬度
 * @param longitude     WGS84 经度
 * @param latitude      WGS84 纬度
 */
static inline void GCJ02toWGS84(const double& gcj_longitude,
                                const double& gcj_latitude, double& longitude,
                                double& latitude) {
  // 不在国内，则没有偏转
  if (OutOfChina(gcj_longitude, gcj_latitude)) {
    longitude = gcj_longitude;
    latitude = gcj_latitude;
    return;
  }
  double delta_lat = TransformLat(gcj_longitude - 105.0, gcj_latitude - 35.0);
  double delta_long = TransformLong(gcj_longitude - 105.0, gcj_latitude - 35.0);
  double rad_lat = gcj_latitude / 180.0 * M_PI;
  double magic = sin(rad_lat);
  magic = 1 - UTM_E2 * magic * magic;
  double sqrt_magic = sqrt(magic);
  delta_lat = (delta_lat * 180.0) /
              ((BJ54_A * (1 - UTM_E2)) / (magic * sqrt_magic) * M_PI);
  delta_long =
      (delta_long * 180.0) / (BJ54_A / sqrt_magic * cos(rad_lat) * M_PI);

  double magic_long = gcj_longitude + delta_long;
  double magic_lat = gcj_latitude + delta_lat;

  longitude = gcj_longitude * 2 - magic_long;
  latitude = gcj_latitude * 2 - magic_lat;
}

static inline void LLToUtmMatrix(double lat, double lon, double height,
                                 double yaw, Eigen::Matrix4d& transform,
                                 char* utm_zone) {
  transform = Eigen::Matrix4d::Zero();

  double utm_northing, utm_easting;

  map_interface::LLtoUTM(lat, lon, utm_northing, utm_easting, utm_zone);
  Eigen::Matrix3d gnss_r =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

  transform.block<3, 3>(0, 0) = gnss_r;
  transform(0, 3) = utm_easting;
  transform(1, 3) = utm_northing;
  transform(2, 3) = height;

  transform(3, 3) = 1;
}

static inline void LLToUtmVector(double lat, double lon, double height,
                                 double yaw, Eigen::Vector3d& transform,
                                 char* utm_zone) {
  double utm_northing, utm_easting;

  map_interface::LLtoUTM(lat, lon, transform[1], transform[0], utm_zone);
  transform[2] = yaw;
}

static inline void LLToUtmMatrixFixZone(double lat, double lon, double height,
                                        double yaw, Eigen::Matrix4d& transform,
                                        int zone_num) {
  transform = Eigen::Matrix4d::Zero();

  double utm_northing, utm_easting;

  map_interface::LLtoUTMFixZone(lat, lon, utm_northing, utm_easting, zone_num);
  Eigen::Matrix3d gnss_r =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

  transform.block<3, 3>(0, 0) = gnss_r;
  transform(0, 3) = utm_easting;
  transform(1, 3) = utm_northing;
  transform(2, 3) = height;

  transform(3, 3) = 1;
}

}  // namespace map_interface
#endif  // INTERFACE_PROJECTION_H_
