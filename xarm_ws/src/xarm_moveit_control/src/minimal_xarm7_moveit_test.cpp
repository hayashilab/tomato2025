#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MinimalXarm7MoveItTest : public rclcpp::Node
{
public:
  MinimalXarm7MoveItTest()
  : Node("minimal_xarm7_moveit_test")
  {
    RCLCPP_INFO(this->get_logger(), "📡 Waiting for /move_group parameters...");

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/tomato/target_pose", 10,
        std::bind(&MinimalXarm7MoveItTest::onTarget, this, std::placeholders::_1));
  }

  void init()
  {
    auto param_client =
        std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");

    while (!param_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Waiting for /move_group service...");
    }

    std::string urdf, srdf;
    while (urdf.empty() || srdf.empty()) {
      try {
        urdf = param_client->get_parameter<std::string>("robot_description");
        srdf = param_client->get_parameter<std::string>("robot_description_semantic");
      } catch (...) {
        urdf.clear();
        srdf.clear();
      }
      if (urdf.empty() || srdf.empty()) {
        RCLCPP_INFO(this->get_logger(), "⏳ Parameters not ready yet...");
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }

    this->declare_parameter<std::string>("robot_description", urdf);
    this->set_parameter(rclcpp::Parameter("robot_description", urdf));
    this->declare_parameter<std::string>("robot_description_semantic", srdf);
    this->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf));

    RCLCPP_INFO(this->get_logger(), "✅ URDF & SRDF copied locally.");

    // ✅ Create MoveGroupInterface AFTER node is fully constructed
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "xarm7");
    move_group_->setEndEffectorLink("link_eef");
    move_group_->setPlanningTime(5.0);

    RCLCPP_INFO(this->get_logger(),
                "✅ MoveGroupInterface initialized (EEF: %s)",
                move_group_->getEndEffectorLink().c_str());
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;

  void onTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_WARN(this->get_logger(), "⚠️ MoveGroupInterface not initialized yet.");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "🎯 Target received: x=%.3f y=%.3f z=%.3f",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    move_group_->setPoseTarget(msg->pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (ok) {
      move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Planning failed.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalXarm7MoveItTest>();
  node->init();   // ✅ Initialize MoveGroupInterface after shared_ptr exists
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
