/**
 * @file data_type.h
 * @author Fei Han (fei.han@horizon.ai)
 * @brief 
 * @version 0.1
 * @date 2019-11-15
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef INTERFACE_DATA_TYPE_H_
#define INTERFACE_DATA_TYPE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace map_interface {
struct Odometry {
  uint64_t stamp;
  uint32_t seq_id;
  std::string frame_id;
  std::string child_frame_id;

  Eigen::Vector3d position;  // x, y, z
  Eigen::Quaterniond quaternion;
  Eigen::Matrix<double, 6, 6> pose_cov;  // (x, y, z, rotation about X axis,
                                         // rotation about Y axis, rotation
                                         // about Z axis)

  Eigen::Vector3d linear_velocity;        // in m/s
  Eigen::Vector3d angular_velocity;       // in rad/s
  Eigen::Matrix<double, 6, 6> twist_cov;  // (x, y, z, rotation about X axis,
                                          // rotation about Y axis, rotation
                                          // about Z axis)
  std::string utm_zone;                   // UTM zone
  Eigen::Vector3d map_offset;             // map offset in UTM
};

struct Image {
  Image() {
    name = "default";
    width = 0;
    height = 0;
    channel = 0;
    data_len = 0;
    is_encoded = true;
    data = NULL;
  }
  ~Image() {
    if (data != NULL) {
      delete data;
    }
  }

  std::string name;
  int width;
  int height;
  int channel;
  int data_len;
  bool is_encoded;
  char *data;
};

struct TextInfo
{
  TextInfo() {
    name_ = "default";
    id_ = "0";
    data_ = "";
  }
  TextInfo(std::string name, std::string data, std::string id) {
    name_ = name;
    data_ = data;
    id_ = id;
  }
  std::string name_;
  std::string data_;
  std::string id_;
};
}  // namespace map_interface
#endif  // INTERFACE_DATA_TYPE_H_
