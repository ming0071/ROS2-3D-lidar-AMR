#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class WaypointNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  WaypointNavigator() : Node("waypoint_navigator")
  {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    clear_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_timer_ = this->create_wall_timer(100ms, std::bind(&WaypointNavigator::recordBaseLinkPosition, this));

    while (!action_client_->wait_for_action_server(2s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    initializeWaypoints();
    sendNextGoal();
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_client_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t current_index_ = 0;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  std::vector<std::pair<double, double>> trajectory_xy_;
  rclcpp::Time start_time_, end_time_;
  double total_distance_ = 0.0;
  geometry_msgs::msg::Pose final_goal_pose_;
  geometry_msgs::msg::Point last_recorded_point_;
  bool first_odom_received_ = false;

  const std::string kCsvPath = "/home/scl/ros2_ws/src/scl_amr/data/multiPath/goal.csv";

  void initializeWaypoints()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    // 1
    pose.pose.position.x = 2.38;
    pose.pose.position.y = -2.59;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    waypoints_.push_back(pose);
    // 2
    pose.pose.position.x = 13.86;
    pose.pose.position.y = 8.92;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    waypoints_.push_back(pose);
    // 3
    pose.pose.position.x = 45.23;
    pose.pose.position.y = 3.76;
    pose.pose.orientation.z = -0.68;
    pose.pose.orientation.w = 0.73;
    waypoints_.push_back(pose);
    // 4
    pose.pose.position.x = 12.97;
    pose.pose.position.y = -2.63;
    pose.pose.orientation.z = 1.0;
    pose.pose.orientation.w = 0.0;
    waypoints_.push_back(pose);
    // 5
    pose.pose.position.x = 1.47;
    pose.pose.position.y = 8.30;
    pose.pose.orientation.z = 1.0;
    pose.pose.orientation.w = 0.0;
    waypoints_.push_back(pose);
    // 6
    pose.pose.position.x = -27.93;
    pose.pose.position.y = -0.91;
    pose.pose.orientation.z = -0.68;
    pose.pose.orientation.w = 0.72;
    waypoints_.push_back(pose);
  }

  void sendNextGoal()
  {
    if (current_index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "All waypoints completed.");
      end_time_ = this->now();
      final_goal_pose_ = waypoints_.back().pose;
      writeTrajectoryToCsv();
      clearCostmap();
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = waypoints_[current_index_];
    goal_msg.behavior_tree = "";

    RCLCPP_INFO(this->get_logger(), "Sending goal %ld: (%.2f, %.2f)",
                current_index_, goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

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
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal %ld canceled.", current_index_);
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Goal %ld unknown result code.", current_index_);
        return;
    }

    current_index_++;
    sendNextGoal();
  }

  void clearCostmap()
  {
    if (!clear_costmap_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Clear costmap service not available.");
      return;
    }

    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    clear_costmap_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Global costmap cleared.");
  }

  void recordBaseLinkPosition()
  {
    geometry_msgs::msg::TransformStamped transform;

    try {
      transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;

      if (!first_odom_received_) {
        start_time_ = this->now();
        last_recorded_point_.x = x;
        last_recorded_point_.y = y;
        first_odom_received_ = true;
      }

      double dx = x - last_recorded_point_.x;
      double dy = y - last_recorded_point_.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance > 0.01) {
        total_distance_ += distance;
        last_recorded_point_.x = x;
        last_recorded_point_.y = y;
        trajectory_xy_.emplace_back(x, y);
      }
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Could not transform map -> base_link: %s", ex.what());
    }
  }

  void writeTrajectoryToCsv()
  {
    std::ofstream file(kCsvPath);

    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", kCsvPath.c_str());
      return;
    }

    file << "x,y,goal_x,goal_y,distance,spend_time\n";

    double elapsed = (end_time_ - start_time_).seconds();
    double goal_x = final_goal_pose_.position.x;
    double goal_y = final_goal_pose_.position.y;

    for (size_t i = 0; i < trajectory_xy_.size(); ++i) {
      const auto & [x, y] = trajectory_xy_[i];
      file << std::fixed << std::setprecision(4) << x << "," << y;
      if (i == 0) {
        file << "," << goal_x << "," << goal_y << "," << total_distance_ << "," << elapsed;
      }
      file << "\n";
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Trajectory saved to: %s", kCsvPath.c_str());
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
