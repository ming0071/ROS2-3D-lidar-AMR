#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class MultiWaypointNavigator : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;

  MultiWaypointNavigator()
  : Node("multi_waypoint_navigator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    declare_parameter<std::vector<std::vector<double>>>("waypoints", {});
    declare_parameter<std::string>("log_csv", "path_log.csv");

    get_parameter("waypoints", waypoint_list_);
    get_parameter("log_csv", log_file_);
    log_path_ = "/home/scl/ros2_ws/src/scl_amr/data/Path/" + log_file_;

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    costmap_client_ = create_client<ClearCostmap>("/global_costmap/clear_entirely_global_costmap");
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    wait_timer_ = create_wall_timer(1s, std::bind(&MultiWaypointNavigator::waitForServer, this));
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr costmap_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::TimerBase::SharedPtr wait_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;
  rclcpp::TimerBase::SharedPtr stop_timer_;

  std::vector<std::vector<double>> waypoint_list_;
  std::vector<std::pair<double, double>> full_path_;

  std::string log_file_;
  std::string log_path_;

  size_t current_index_ = 0;
  int stop_countdown_ = 0;

  std::chrono::steady_clock::time_point start_time_;
  double total_distance_ = 0.0;
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;

  void waitForServer() {
    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(get_logger(), "Waiting for /navigate_to_pose...");
      return;
    }
    wait_timer_->cancel();
    start_time_ = std::chrono::steady_clock::now();
    sendNextGoal();
    log_timer_ = create_wall_timer(200ms, std::bind(&MultiWaypointNavigator::logPose, this));
  }

  void sendNextGoal() {
    if (current_index_ >= waypoint_list_.size()) {
      RCLCPP_INFO(get_logger(), "All waypoints completed.");
      log_timer_->cancel();
      stopRobot();
      printFinalPose();
      writeLogToFile();
      clearCostmap();
      rclcpp::shutdown();
      return;
    }

    const auto &wp = waypoint_list_[current_index_];
    if (wp.size() < 3) {
      RCLCPP_WARN(get_logger(), "Waypoint[%zu] is invalid.", current_index_);
      ++current_index_;
      sendNextGoal();
      return;
    }

    goal_x_ = wp[0];
    goal_y_ = wp[1];

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.position.x = wp[0];
    pose.pose.position.y = wp[1];
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, wp[2]);
    pose.pose.orientation = tf2::toMsg(q);

    NavigateToPose::Goal goal;
    goal.pose = pose;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback = std::bind(&MultiWaypointNavigator::onResult, this, std::placeholders::_1);

    nav_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "Sent waypoint %zu: (%.2f, %.2f, %.2f)", current_index_+1, wp[0], wp[1], wp[2]);
  }

  void onResult(const GoalHandle::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Waypoint %zu reached.", current_index_+1);
    } else {
      RCLCPP_WARN(get_logger(), "Failed to reach waypoint %zu.", current_index_+1);
    }
    ++current_index_;
    sendNextGoal();
  }

  void logPose() {
    try {
      auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      double x = tf.transform.translation.x;
      double y = tf.transform.translation.y;

      if (!full_path_.empty()) {
        double dx = x - full_path_.back().first;
        double dy = y - full_path_.back().second;
        total_distance_ += std::hypot(dx, dy);
      }

      full_path_.emplace_back(x, y);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF Error: %s", ex.what());
    }
  }

  void writeLogToFile() {
    std::ofstream file(log_path_, std::ios::out);
    file << "x,y,goal_x,goal_y,distance,spend_time\n";
    auto end_time = std::chrono::steady_clock::now();
    double duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time_).count();

    for (size_t i = 0; i < full_path_.size(); ++i) {
      const auto & pt = full_path_[i];
      file << std::fixed << std::setprecision(2)
           << pt.first << "," << pt.second;
      if (i == 0) {
        file << "," << goal_x_ << "," << goal_y_ << ","
             << total_distance_ << "," << duration;
      }
      file << "\n";
    }
    file.close();
    RCLCPP_INFO(get_logger(), "Log saved to %s", log_path_.c_str());
  }

  void stopRobot() {
    stop_countdown_ = 5;
    stop_timer_ = create_wall_timer(100ms, [this]() {
      if (stop_countdown_-- > 0) {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;
        cmd_vel_pub_->publish(stop);
      } else {
        stop_timer_->cancel();
      }
    });
  }

  void printFinalPose() {
    try {
      auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      double x = tf.transform.translation.x;
      double y = tf.transform.translation.y;

      double qx = tf.transform.rotation.x;
      double qy = tf.transform.rotation.y;
      double qz = tf.transform.rotation.z;
      double qw = tf.transform.rotation.w;

      double siny = 2.0 * (qw * qz + qx * qy);
      double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
      double yaw = std::atan2(siny, cosy);
      double yaw_deg = yaw * 180.0 / M_PI;

      RCLCPP_INFO(get_logger(), "\033[1;34mFinal pose: x = %.3f, y = %.3f, θ = %.2f°\033[0m", x, y, yaw_deg);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF Error (final pose): %s", ex.what());
    }
  }

  void clearCostmap() {
    if (!costmap_client_->wait_for_service(2s)) {
      RCLCPP_WARN(get_logger(), "Costmap service not available.");
      return;
    }
    auto request = std::make_shared<ClearCostmap::Request>();
    auto future = costmap_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, 2s)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Clear costmap request failed.");
    } else {
      RCLCPP_INFO(get_logger(), "Successfully cleared global costmap.");
    }
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiWaypointNavigator>());
  rclcpp::shutdown();
  return 0;
}
