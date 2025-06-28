#include <memory>
#include <vector>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class WaypointNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  WaypointNavigator()
  : Node("waypoint_navigator")
  {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // 等待 action server 準備好
    while (!action_client_->wait_for_action_server(2s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    // 初始化 waypoints
    geometry_msgs::msg::PoseStamped pose1;
    pose1.header.frame_id = "map";
    pose1.pose.position.x = -4.07;
    pose1.pose.position.y = -3.65;
    pose1.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped pose2;
    pose2.header.frame_id = "map";
    pose2.pose.position.x = -1.72;
    pose2.pose.position.y = -2.17;
    pose2.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped pose3;
    pose3.header.frame_id = "map";
    pose3.pose.position.x = 2.16;
    pose3.pose.position.y = -2.51;
    pose3.pose.orientation.w = 1.0;

    waypoints_.push_back(pose1);
    waypoints_.push_back(pose2);
    waypoints_.push_back(pose3);

    // 開始導航第一個點
    sendNextGoal();
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t current_index_ = 0;

  void sendNextGoal()
  {
    if (current_index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints completed.");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = waypoints_[current_index_];
    goal_msg.behavior_tree = "";

    RCLCPP_INFO(this->get_logger(), "Sending goal %ld: (%.2f, %.2f)",
                current_index_,
                goal_msg.pose.pose.position.x,
                goal_msg.pose.pose.position.y);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&WaypointNavigator::goalResponseCallback, this, std::placeholders::_1);
    options.feedback_callback =
      std::bind(&WaypointNavigator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
      std::bind(&WaypointNavigator::resultCallback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, options);
  }

  void goalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal %ld rejected.", current_index_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal %ld accepted.", current_index_);
    }
  }

  void feedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Remaining distance: %.2f", feedback->distance_remaining);
  }

  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal %ld succeeded.", current_index_);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal %ld aborted.", current_index_);
        return; // 結束整個任務
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal %ld canceled.", current_index_);
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Goal %ld unknown result code.", current_index_);
        return;
    }

    current_index_++;
    sendNextGoal();  // 自動遞送下一個 goal
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
