/*
 * @Description: lane line tracking in 3D
 * @Author: ming01.jian@horizon.ai
 * @Date: 2019-06-14 16:57:56
 * @LastEditTime: 2019-07-17 11:56:00
 * @LastEditors: jinglin.zhang@horizon.ai
 * @copyright Copyright (c) 2020
 */
#ifndef INTERFACE_SMOOTH_PT_KALMAN_ESTIMATION_H_
#define INTERFACE_SMOOTH_PT_KALMAN_ESTIMATION_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

namespace lane_smooth {

class KalmanEstimation {
 public:
  KalmanEstimation()
      : state_size_(9),
        control_size_(0),
        measurement_size_(3),
        type_(CV_32F),
        init_status_(false) {}
  explicit KalmanEstimation(Eigen::Matrix<float, 9, 1> &init_state)
      : state_size_(9),
        control_size_(0),
        measurement_size_(3),
        type_(CV_32F),
        init_status_(false) {
    initKalman(init_state);
  }
  ~KalmanEstimation() {}
  void initKalman(Eigen::Matrix<float, 9, 1> &init_state);
  void kalmanPredict();
  cv::Mat kalmanCorrect(Eigen::Matrix<float, 3, 1> &point,
                        const Eigen::Matrix<float, 3, 1> &curve_start_pt,
                        float arc_length);
  inline bool getInitStatus() { return init_status_; }
  void resetInitStatus() { init_status_ = false; }
  bool checkMeasurement(Eigen::Matrix<float, 3, 1> &point,
                        const Eigen::Matrix<float, 3, 1> &curve_start_pt,
                        float arc_length);

 private:
  int state_size_;
  int control_size_;
  int measurement_size_;
  unsigned int type_;

  bool init_status_;

  cv::Mat state_;
  cv::Mat measurement_;

  cv::KalmanFilter kf_;
};

}  // namespace lane_smooth

#endif  // INTERFACE_SMOOTH_PT_KALMAN_ESTIMATION_H_
