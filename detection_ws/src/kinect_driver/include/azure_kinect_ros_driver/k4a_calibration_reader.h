#ifndef K4A_CALIBRATION_READER_H
#define K4A_CALIBRATION_READER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace azure_kinect_ros_driver
{

class K4ACalibrationReader
{
public:
  static bool readCalibrationFile(const std::string& calibration_file, sensor_msgs::msg::CameraInfo& camera_info, rclcpp::Logger logger);
};

} // namespace azure_kinect_ros_driver

#endif // K4A_CALIBRATION_READER_H