#include "azure_kinect_ros_driver/k4a_calibration_reader.h"

namespace azure_kinect_ros_driver
{

bool K4ACalibrationReader::readCalibrationFile(const std::string& calibration_file, sensor_msgs::msg::CameraInfo& camera_info, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Attempting to read calibration file: %s", calibration_file.c_str());
  
  if (calibration_file.empty())
  {
    RCLCPP_WARN(logger, "Calibration file path is empty");
    return false;
  }

  try
  {
    RCLCPP_INFO(logger, "Reading YAML file...");
    YAML::Node config = YAML::LoadFile(calibration_file);
    RCLCPP_INFO(logger, "YAML file loaded successfully");
    
    // Check if this is a valid calibration file
    if (!config["camera_matrix"] || !config["distortion_coefficients"])
    {
      RCLCPP_ERROR(logger, "Invalid calibration file format - missing camera_matrix or distortion_coefficients");
      return false;
    }
    
    // Set the distortion model
    RCLCPP_INFO(logger, "Setting distortion model to plumb_bob");
    camera_info.distortion_model = "plumb_bob";
    
    // Clear existing distortion coefficients
    camera_info.d.clear();
    
    // Read distortion coefficients
    RCLCPP_INFO(logger, "Reading distortion coefficients");
    auto distortion = config["distortion_coefficients"]["data"];
    for (std::size_t i = 0; i < distortion.size(); i++)
    {
      camera_info.d.push_back(distortion[i].as<double>());
    }
    
    // Read camera matrix
    RCLCPP_INFO(logger, "Reading camera matrix");
    auto camera_matrix = config["camera_matrix"]["data"];
    camera_info.k[0] = camera_matrix[0].as<double>(); // fx
    camera_info.k[2] = camera_matrix[2].as<double>(); // cx
    camera_info.k[4] = camera_matrix[4].as<double>(); // fy
    camera_info.k[5] = camera_matrix[5].as<double>(); // cy
    camera_info.k[8] = 1.0;
    
    // Read projection matrix if available
    if (config["projection_matrix"])
    {
      RCLCPP_INFO(logger, "Reading projection matrix");
      auto projection = config["projection_matrix"]["data"];
      camera_info.p[0] = projection[0].as<double>(); // fx
      camera_info.p[2] = projection[2].as<double>(); // cx
      camera_info.p[5] = projection[5].as<double>(); // fy
      camera_info.p[6] = projection[6].as<double>(); // cy
      camera_info.p[10] = 1.0;
    }
    else
    {
      RCLCPP_INFO(logger, "No projection matrix found, copying from camera matrix");
      // If no projection matrix, copy from camera matrix
      camera_info.p[0] = camera_info.k[0]; // fx
      camera_info.p[2] = camera_info.k[2]; // cx
      camera_info.p[5] = camera_info.k[4]; // fy
      camera_info.p[6] = camera_info.k[5]; // cy
      camera_info.p[10] = 1.0;
    }
    
    // Set the rectification matrix to identity
    camera_info.r[0] = 1.0;
    camera_info.r[4] = 1.0;
    camera_info.r[8] = 1.0;
    
    RCLCPP_INFO(logger, "Successfully loaded calibration from %s", calibration_file.c_str());
    RCLCPP_INFO(logger, "Camera info now has distortion_model: %s", camera_info.distortion_model.c_str());
    RCLCPP_INFO(logger, "Camera info now has %zu distortion coefficients", camera_info.d.size());
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Failed to read calibration file: %s", e.what());
    return false;
  }
}

} // namespace azure_kinect_ros_driver