/*
 * @Description: smooth 3D control points of lane lines
 * @Author: ming01.jian@horizon.ai
 * @Date: 2019-06-26 20:39:48
 * @LastEditTime: 2019-07-17 11:56:00
 * @LastEditors: jinglin.zhang@horizon.ai
 * @copyright Copyright (c) 2020
 */

#ifndef INTERFACE_SMOOTH_PT_SMOOTH_CONTROL_POINT_H_
#define INTERFACE_SMOOTH_PT_SMOOTH_CONTROL_POINT_H_
#include "kalman_estimation.h"
// c++
#include <list>
#include <mutex>
#include <queue>
#include <vector>
// eigen 3
#include <Eigen/Core>
#include <Eigen/Dense>

namespace lane_smooth {

struct LanePoint {
  explicit LanePoint(Eigen::Matrix<float, 3, 1> pt_xyz) { xyz_ = pt_xyz; }
  Eigen::Matrix<float, 3, 1> xyz_;
  bool pre_solid_ = false;
  bool suc_solid_ = false;
};

struct PolynomialCurve2 {
  Eigen::Matrix<float, 3, 1> start_pt_,
      end_pt_;  // updated in first parametrization
  Eigen::Matrix<float, 3, 1> coeffs_x_, coeffs_y_,
      coeffs_z_;  // updated in first parametrization and re-parametrization
  float max_arc_length_;  // updated in re-parametrization　[0, max_arc_length_]
  std::vector<float> all_pt_arc_;
  // std::list<float> dashed_pt_arc_; // maybe used in saving the arc lenght of
  // dashed point after re-parametrization
};

class SingleLaneLine2 {
 public:
  SingleLaneLine2() {}
  ~SingleLaneLine2() {}

  void getPiecewiseCurves(std::vector<PolynomialCurve2> &curves) {
    if (!piecewise_curves_.empty())
      curves.insert(curves.begin(), piecewise_curves_.begin(),
                    piecewise_curves_.end());

    std::cout << " piecewise_curves_.size() " << piecewise_curves_.size()
              << std::endl;
    if (!pt_arc_.empty()) {
      PolynomialCurve2 poly_curve;
      poly_curve.start_pt_ = start_pt_;
      poly_curve.end_pt_ =
          end_pt_;  // buffer中第一个点是最先开始不适用方程系数的点
      poly_curve.coeffs_x_ = coeffs_x_;
      poly_curve.coeffs_y_ = coeffs_y_;
      poly_curve.coeffs_z_ = coeffs_z_;
      poly_curve.max_arc_length_ = pt_arc_.back();
      poly_curve.all_pt_arc_.reserve(pt_arc_.size());
      poly_curve.all_pt_arc_ = pt_arc_;
      curves.push_back(poly_curve);
    }
  }

  // for kalman filter
  bool initKalmanEstimator(Eigen::Matrix<float, 3, 1> &pt_start,
                           Eigen::Matrix<float, 3, 1> &pt_end);
  inline float getAccumulativeArcLength() { return accumulative_arc_; }
  inline float addAccumulativeArcLength(float arc_length) {
    accumulative_arc_ += arc_length;
  }
  inline void setCurveCoeffs(cv::Mat &estimated_status) {
    coeffs_x_ << estimated_status.at<float>(0), estimated_status.at<float>(1),
        estimated_status.at<float>(2);
    coeffs_y_ << estimated_status.at<float>(3), estimated_status.at<float>(4),
        estimated_status.at<float>(5);
    coeffs_z_ << estimated_status.at<float>(6), estimated_status.at<float>(7),
        estimated_status.at<float>(8);
  }
  inline void setCurveEndPoint(Eigen::Matrix<float, 3, 1> &pt_end) {
    end_pt_ = pt_end;
  }
  inline Eigen::Matrix<float, 3, 1> getCurveStartPoint() { return start_pt_; }
  inline Eigen::Matrix<float, 3, 1> calculateCurveSamplePoint(float arc) {
    Eigen::Matrix<float, 3, 1> sample_pt;
    float arc2 = arc * arc;
    float arc3 = arc2 * arc;
    sample_pt(0) = coeffs_x_(0) * arc3 + coeffs_x_(1) * arc2 +
                   coeffs_x_(2) * arc + start_pt_(0);
    sample_pt(1) = coeffs_y_(0) * arc3 + coeffs_y_(1) * arc2 +
                   coeffs_y_(2) * arc + start_pt_(1);
    sample_pt(2) = coeffs_z_(0) * arc3 + coeffs_z_(1) * arc2 +
                   coeffs_z_(2) * arc + start_pt_(2);
    return sample_pt;
  }

  // for curve transition trigger
  // (1) distance trigger
  const int nBUFFER_ = 10;
  int error_counter_ = 0;
  const float xy_THRE_ = 0.2;             // 30 cm
  const float z_THRE_ = 0.3;              // 30 cm
  std::queue<float> nbuffer_arc_length_;  // the arc length set of each point in
                                          // nbuffer_points_.
  std::queue<Eigen::Matrix<float, 3, 1>> nbuffer_points_;  // size = buffer;
  std::queue<Eigen::Matrix<float, 3, 3>> nbuffer_curve_coeffs_;  // size =
                                                                 // buffer;
                                                                 // [coeffs_x_,
                                                                 // coeffs_y_,
                                                                 // coeffs_z_]
  bool curveTransitionTrigger(const Eigen::Matrix<float, 3, 1> &pt,
                              const float arc_length,
                              Eigen::Matrix<float, 3, 1> &control_start_pt,
                              Eigen::Matrix<float, 3, 1> &control_end_pt);
  void saveLastCurve();
  void clearVariables();
  Eigen::Matrix<float, 3, 1> computeVerticalPointToCubicCurve(
      Eigen::Matrix<float, 3, 1> &pt);
  Eigen::Matrix<float, 3, 1> computeNearestPoint(
      Eigen::Matrix<float, 3, 1> &pt);

  // (3) the distance of neighborhood 3D points
  const float neighborhood_dis_THRE_ = 10.0;  // 10 m

  lane_smooth::KalmanEstimation kalman_est_;

  // for kalman filter visualization
  std::vector<float> pt_arc_;  // all approximate arc length of points
  std::list<Eigen::Matrix<float, 3, 1>>
      pt_measurement_;  // corresponding pt_arc_;
  int curve_test_num_ = 0;

 private:
  std::mutex mMutexControlPts;

  std::list<PolynomialCurve2> piecewise_curves_;  // all piecwise polynomial
                                                  // curves belong to a lane
                                                  // line.

  //
  float accumulative_arc_ = 0.0;
  float arc_length_max_;
  /** curve parameters
   * 0.0 <= s <= arc_length_max_
   * X = coeffs_x_(0)*s^3 + coeffs_x_(1)*s^2 + coeffs_x_(2)*s + start_pt_(0);
   * Y = coeffs_y_(0)*s^3 + coeffs_y_(1)*s^2 + coeffs_y_(2)*s + start_pt_(1);
   * Z = coeffs_z_(0)*s^3 + coeffs_z_(1)*s^2 + coeffs_z_(2)*s + start_pt_(2);
  */
  Eigen::Matrix<float, 3, 1> start_pt_, end_pt_;
  Eigen::Matrix<float, 3, 1> coeffs_x_, coeffs_y_, coeffs_z_;
};  // class SingleLaneLine

class LaneLineSmooth {
 public:
  LaneLineSmooth() {}
  ~LaneLineSmooth() {}
  bool smoothControlPoints(
      std::vector<Eigen::Matrix<float, 3, 1>> &control_points);

  bool smoothLaneParam(std::vector<Eigen::Vector3f> &lane_points);

  void getSmoothResults(std::vector<PolynomialCurve2> &curves);

  SingleLaneLine2 lane_line_;
};

}  // end namespace lane_smooth
#endif  // INTERFACE_SMOOTH_PT_SMOOTH_CONTROL_POINT_H_
