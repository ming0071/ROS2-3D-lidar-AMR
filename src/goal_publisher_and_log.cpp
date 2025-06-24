#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

using namespace std::chrono_literals;

class NavigateAndLogNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;

    NavigateAndLogNode()
    : Node("navigate_and_log_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        declare_parameter("mode", 1);   // 0:start , 1: goal

        declare_parameter("start.x", -1.76);
        declare_parameter("start.y", -0.83);
        declare_parameter("start.z", 0.0);
        declare_parameter("start.qx", 0.0);
        declare_parameter("start.qy", 0.0);
        declare_parameter("start.qz", 1.0);
        declare_parameter("start.qw", 0.0);

        declare_parameter("goal.x", 6.71);
        declare_parameter("goal.y", -0.59);
        declare_parameter("goal.z", 0.0);
        declare_parameter("goal.qx", 0.0);
        declare_parameter("goal.qy", 0.0);
        declare_parameter("goal.qz", 0.0);
        declare_parameter("goal.qw", 1.0);

        declare_parameter("log_csv", "default_log.csv");

        std::string log_filename;
        get_parameter("log_csv", log_filename);
        log_file_path_ = "/home/scl/ros2_ws/src/scl_amr/data/Path/" + log_filename;

        log_active_ = false;

        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
        costmap_client_ = this->create_client<ClearCostmap>("/global_costmap/clear_entirely_global_costmap");
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        wait_timer_ = this->create_wall_timer(1s, std::bind(&NavigateAndLogNode::wait_for_server, this));
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::Client<ClearCostmap>::SharedPtr costmap_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::TimerBase::SharedPtr wait_timer_;
    rclcpp::TimerBase::SharedPtr log_timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    int stop_countdown_ = 0;

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
        int mode = get_parameter("mode").as_int();
        std::string base = (mode == 0) ? "start" : "goal";

        NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = now();

        goal_msg.pose.pose.position.x = get_parameter(base + ".x").as_double();
        goal_msg.pose.pose.position.y = get_parameter(base + ".y").as_double();
        goal_msg.pose.pose.position.z = get_parameter(base + ".z").as_double();
        goal_msg.pose.pose.orientation.x = get_parameter(base + ".qx").as_double();
        goal_msg.pose.pose.orientation.y = get_parameter(base + ".qy").as_double();
        goal_msg.pose.pose.orientation.z = get_parameter(base + ".qz").as_double();
        goal_msg.pose.pose.orientation.w = get_parameter(base + ".qw").as_double();

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.result_callback = std::bind(&NavigateAndLogNode::result_callback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(goal_msg, options);
        RCLCPP_INFO(this->get_logger(), "Goal sent");

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
            ss << tf_msg.transform.translation.x << ","
               << tf_msg.transform.translation.y;

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

            // Stop robot by repeatedly publishing zero velocity 
            scheduleStopMotion();
            // Print final pose (x, y, yaw) from tf
            printFinalPose();            
            // Write trajectory log to CSV file
            write_log_to_file();
            clear_costmap();
        } else {
            RCLCPP_WARN(this->get_logger(), "Navigation failed. Discarding log.");
        }

        rclcpp::shutdown();
    }

    void scheduleStopMotion(){
        stop_countdown_ = 5;  // 100ms * 5 = 500ms
        stop_timer_ = this->create_wall_timer(100ms, [this]() {
            if (stop_countdown_-- > 0) {
                geometry_msgs::msg::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
                cmd_vel_pub_->publish(stop_msg);
                RCLCPP_INFO(this->get_logger(), "Re-publishing stop cmd_vel (remaining: %d)", stop_countdown_);
            } else {
                stop_timer_->cancel();
                RCLCPP_INFO(this->get_logger(), "Final stop command complete");
            }
        });
    }
    void printFinalPose(){
        try {
            auto tf_msg = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            double x = tf_msg.transform.translation.x;
            double y = tf_msg.transform.translation.y;

            double qx = tf_msg.transform.rotation.x;
            double qy = tf_msg.transform.rotation.y;
            double qz = tf_msg.transform.rotation.z;
            double qw = tf_msg.transform.rotation.w;

            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
            double yaw_deg = yaw_rad * 180.0 / M_PI;

            RCLCPP_INFO(this->get_logger(),
                "\033[1;34mFinal pose: x = %.3f, y = %.3f, θ = %.2f°\033[0m",
                x, y, yaw_deg);

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error on final pose: %s", ex.what());
        }
    }

    void clear_costmap()
    {
        if (!costmap_client_->wait_for_service(2s)) {
            RCLCPP_WARN(this->get_logger(), "Costmap service not available.");
        } else {
            auto request = std::make_shared<ClearCostmap::Request>();
            auto future = costmap_client_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 2s)
                != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to call clear costmap service");
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully cleared global costmap.");
            }
        }
    }

    void write_log_to_file()
    {
        std::ofstream file(log_file_path_, std::ios::out);
        file << "x,y\n";
        for (const auto & line : log_data_) {
            file << line << "\n";
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
