#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class NavigateAndLogNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigateAndLogNode()
    : Node("navigate_and_log_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        declare_parameter("x", 0.0);
        declare_parameter("y", 0.0);
        declare_parameter("z", 0.0);
        declare_parameter("qx", 0.0);
        declare_parameter("qy", 0.0);
        declare_parameter("qz", 0.0);
        declare_parameter("qw", 1.0);

        log_file_path_ = "/home/scl/ros2_ws/src/scl_amr/data/Path/log.txt";
        log_active_ = false;

        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        wait_timer_ = this->create_wall_timer(1s, std::bind(&NavigateAndLogNode::wait_for_server, this));
    }

private:
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Action Client
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::TimerBase::SharedPtr wait_timer_;
    rclcpp::TimerBase::SharedPtr log_timer_;

    // Log
    bool log_active_;
    std::string log_file_path_;
    std::vector<std::string> log_data_;

    void wait_for_server()
    {
        if (!nav_action_client_->wait_for_action_server(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /navigate_to_pose action server...");
            return;
        }

        wait_timer_->cancel();
        send_goal();
    }

    void send_goal()
    {
        double x = get_parameter("x").as_double();
        double y = get_parameter("y").as_double();
        double z = get_parameter("z").as_double();
        double qx = get_parameter("qx").as_double();
        double qy = get_parameter("qy").as_double();
        double qz = get_parameter("qz").as_double();
        double qw = get_parameter("qw").as_double();

        NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.position.z = z;
        goal_msg.pose.pose.orientation.x = qx;
        goal_msg.pose.pose.orientation.y = qy;
        goal_msg.pose.pose.orientation.z = qz;
        goal_msg.pose.pose.orientation.w = qw;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.result_callback = std::bind(&NavigateAndLogNode::result_callback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(goal_msg, options);
        RCLCPP_INFO(this->get_logger(), "Goal sent: [%.2f, %.2f, %.2f]", x, y, z);

        log_active_ = true;
        log_timer_ = this->create_wall_timer(200ms, std::bind(&NavigateAndLogNode::log_pose, this));
    }

    void log_pose()
    {
        if (!log_active_) return;

        try {
            geometry_msgs::msg::TransformStamped tf_msg =
                tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            std::ostringstream ss;
            ss <<  tf_msg.transform.translation.x << ", "
               << tf_msg.transform.translation.y ;

            log_data_.push_back(ss.str());
            RCLCPP_INFO(this->get_logger(), "Logged: %s", ss.str().c_str());

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }

    void result_callback(const GoalHandle::WrappedResult & result)
    {
        log_active_ = false;
        log_timer_->cancel();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded. Saving log...");
            write_log_to_file();
        } else {
            RCLCPP_WARN(this->get_logger(), "Navigation failed. Discarding log.");
        }

        rclcpp::shutdown();
    }

    void write_log_to_file()
    {
        std::ofstream file(log_file_path_, std::ios::out);
        for (const auto & line : log_data_) {
            file << line << std::endl;
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Log saved to %s", log_file_path_.c_str());
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateAndLogNode>());
    rclcpp::shutdown();
    return 0;
}
