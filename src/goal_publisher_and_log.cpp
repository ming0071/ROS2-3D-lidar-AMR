#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>

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
        declare_parameters();
        initialize_interfaces();
        wait_timer_ = create_wall_timer(1s, std::bind(&NavigateAndLogNode::waitForNavServer, this));
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

    std::string log_file_path_;
    std::vector<std::pair<double, double>> log_data_;

    bool log_active_ = false;
    int stop_countdown_ = 0;
    double goal_x_ = 0.0, goal_y_ = 0.0, total_distance_ = 0.0;
    std::chrono::steady_clock::time_point start_time_;

    constexpr static int STOP_RETRY_COUNT = 5;

    // Declare and read all required parameters
    void declare_parameters()
    {
        declare_parameter("mode", 1);

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

        std::string filename;
        get_parameter("log_csv", filename);
        log_file_path_ = "/home/scl/ros2_ws/src/scl_amr/data/Path/" + filename;
    }

    // Initialize clients, publishers
    void initialize_interfaces()
    {
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
        costmap_client_ = create_client<ClearCostmap>("/global_costmap/clear_entirely_global_costmap");
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    // Wait until navigation action server is ready
    void waitForNavServer()
    {
        if (!nav_action_client_->wait_for_action_server(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for /navigate_to_pose server...");
            return;
        }
        wait_timer_->cancel();
        sendNavigationGoal();
    }

    // Construct and send navigation goal
    void sendNavigationGoal()
    {
        int mode = get_parameter("mode").as_int();
        std::string prefix = (mode == 0) ? "start" : "goal";

        auto goal_msg = createGoalMessage(prefix);
        goal_x_ = goal_msg.pose.pose.position.x;
        goal_y_ = goal_msg.pose.pose.position.y;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.result_callback = std::bind(&NavigateAndLogNode::onNavResult, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(goal_msg, options);
        RCLCPP_INFO(get_logger(), "Goal sent.");

        startLoggingPose();
    }

    // Create navigation goal from parameter prefix
    NavigateToPose::Goal createGoalMessage(const std::string &prefix)
    {
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();

        goal.pose.pose.position.x = get_parameter(prefix + ".x").as_double();
        goal.pose.pose.position.y = get_parameter(prefix + ".y").as_double();
        goal.pose.pose.position.z = get_parameter(prefix + ".z").as_double();
        goal.pose.pose.orientation.x = get_parameter(prefix + ".qx").as_double();
        goal.pose.pose.orientation.y = get_parameter(prefix + ".qy").as_double();
        goal.pose.pose.orientation.z = get_parameter(prefix + ".qz").as_double();
        goal.pose.pose.orientation.w = get_parameter(prefix + ".qw").as_double();

        return goal;
    }

    // Start logging current TF pose at 5Hz
    void startLoggingPose()
    {
        start_time_ = std::chrono::steady_clock::now();
        log_active_ = true;

        log_timer_ = create_wall_timer(200ms, std::bind(&NavigateAndLogNode::logPose, this));
    }

    // Log robot pose from TF and compute total distance
    void logPose()
    {
        if (!log_active_) return;

        try {
            auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            double x = tf.transform.translation.x;
            double y = tf.transform.translation.y;

            if (!log_data_.empty()) {
                double dx = x - log_data_.back().first;
                double dy = y - log_data_.back().second;
                total_distance_ += std::hypot(dx, dy);
            }

            log_data_.emplace_back(x, y);
            RCLCPP_INFO(get_logger(), "Logged: %.3f, %.3f", x, y);

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
        }
    }

    // Navigation result callback
    void onNavResult(const GoalHandle::WrappedResult &result)
    {
        log_active_ = false;
        log_timer_->cancel();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {

            // Stop robot by repeatedly publishing zero velocity 
            scheduleStop();
            // Print final pose (x, y, yaw) from tf
            printFinalPose();
            // Write trajectory log to CSV file
            writeLogToCSV();
            clearCostmap();
        } else {
            RCLCPP_WARN(get_logger(), "Navigation failed.");
        }

        rclcpp::shutdown();
    }

    // Stop robot by sending zero velocity several times
    void scheduleStop()
    {
        stop_countdown_ = STOP_RETRY_COUNT;
        stop_timer_ = create_wall_timer(100ms, [this]() {
            if (stop_countdown_-- > 0) {
                geometry_msgs::msg::Twist stop_msg;
                stop_msg.linear.x = 0.0;
                stop_msg.angular.z = 0.0;
                cmd_vel_pub_->publish(stop_msg);
                RCLCPP_INFO(this->get_logger(), "Re-publishing stop cmd_vel (remaining: %d)", stop_countdown_);
            } else {
                stop_timer_->cancel();
                RCLCPP_INFO(get_logger(), "Stop command completed.");
            }
        });
    }

    // Print final robot pose in map frame with heading
    void printFinalPose()
    {
        try {
            auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            double x = tf.transform.translation.x;
            double y = tf.transform.translation.y;

            double qx = tf.transform.rotation.x;
            double qy = tf.transform.rotation.y;
            double qz = tf.transform.rotation.z;
            double qw = tf.transform.rotation.w;

            double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                    1.0 - 2.0 * (qy * qy + qz * qz));

            RCLCPP_INFO(get_logger(), "\033[1;34mFinal pose: x = %.3f, y = %.3f, θ = %.2f°\033[0m",
                        x, y, yaw * 180.0 / M_PI);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
        }
    }

    // Clear global costmap via service
    void clearCostmap()
    {
        if (!costmap_client_->wait_for_service(2s)) {
            RCLCPP_WARN(get_logger(), "Costmap service unavailable.");
            return;
        }

        auto req = std::make_shared<ClearCostmap::Request>();
        auto future = costmap_client_->async_send_request(req);

        if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, 2s) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(get_logger(), "Failed to call costmap clear service.");
        } else {
            RCLCPP_INFO(get_logger(), "Global costmap cleared.");
        }
    }

    // Write CSV file of pose trajectory
    void writeLogToCSV()
    {
        std::ofstream file(log_file_path_);
        file << "x,y,goal_x,goal_y,distance,spend_time\n";

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        for (size_t i = 0; i < log_data_.size(); ++i) {
            const auto &[x, y] = log_data_[i];
            file << std::fixed << std::setprecision(2) << x << "," << y;
            if (i == 0) {
                file << "," << goal_x_ << "," << goal_y_ << ","
                     << total_distance_ << "," << elapsed;
            }
            file << "\n";
        }

        RCLCPP_INFO(get_logger(), "Log saved: %s", log_file_path_.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateAndLogNode>());
    rclcpp::shutdown();
    return 0;
}
