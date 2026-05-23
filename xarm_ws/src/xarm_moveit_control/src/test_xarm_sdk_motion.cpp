#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/move_joint.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class XArmSDKMotionTest : public rclcpp::Node
{
public:
  XArmSDKMotionTest() : Node("xarm_sdk_motion_test")
  {
    // === Create clients for SDK services ===
    joint_client_ = this->create_client<xarm_msgs::srv::MoveJoint>("/xarm/set_servo_angle");
    cart_client_  = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");

    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for xArm SDK services...");
    joint_client_->wait_for_service();
    cart_client_->wait_for_service();
    RCLCPP_INFO(this->get_logger(), "✅ Services available!");

    // === Initialize MoveIt interface ===
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "xarm7");
    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(this->get_logger(), "✅ MoveGroupInterface initialized for xarm7");

    // === TEST 1: MoveJoint to a known configuration ===
    auto joint_req = std::make_shared<xarm_msgs::srv::MoveJoint::Request>();
    joint_req->angles = { 0.0f, 0.2081f, 0.1690f, 0.5495f, -0.0417f, -1.2699f, 0.0f };
    joint_req->speed  = 0.25f;   // Slow and smooth
    joint_req->acc    = 8.0f;
    joint_req->mvtime = 0.0f;
    joint_req->wait   = true;

    RCLCPP_INFO(this->get_logger(), "▶️ Sending MoveJoint command...");
    auto joint_future = joint_client_->async_send_request(joint_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), joint_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "❌ MoveJoint call failed");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "✅ MoveJoint completed — reading current pose from MoveIt...");
    rclcpp::sleep_for(1s);

    // === Get current pose directly from MoveIt ===
    geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
    RCLCPP_INFO(this->get_logger(),
                "📍 Current Pose:\n  x=%.3f m, y=%.3f m, z=%.3f m",
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z);

    // === Compute forward offset (5 cm in X direction) ===
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += 0.05; // move forward 5 cm

    RCLCPP_INFO(this->get_logger(),
                "🎯 Target Pose:\n  x=%.3f m, y=%.3f m, z=%.3f m (forward +5 cm)",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    // === TEST 2: Send MoveCartesian (absolute mm coordinates) ===
    auto cart_req = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
    cart_req->pose = {
      static_cast<float>(target_pose.position.x * 1000.0),
      static_cast<float>(target_pose.position.y * 1000.0),
      static_cast<float>(target_pose.position.z * 1000.0),
      0.0f, 0.0f, 0.0f // roll, pitch, yaw (optional)
    };
    cart_req->speed  = 50.0f;   // mm/s
    cart_req->acc    = 200.0f;  // mm/s²
    cart_req->mvtime = 0.0f;
    cart_req->wait   = true;

    RCLCPP_INFO(this->get_logger(), "▶️ Sending MoveCartesian command (forward +50 mm)...");
    auto cart_future = cart_client_->async_send_request(cart_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cart_future)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto resp = cart_future.get();
      if (resp->ret == 0)
        RCLCPP_INFO(this->get_logger(), "✅ Cartesian motion finished successfully.");
      else
        RCLCPP_WARN(this->get_logger(), "⚠️ Cartesian motion returned: %s", resp->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ MoveCartesian call failed");
    }

    RCLCPP_INFO(this->get_logger(), "✅ Test completed — you can stop the node now.");
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<xarm_msgs::srv::MoveJoint>::SharedPtr joint_client_;
  rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr cart_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XArmSDKMotionTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
