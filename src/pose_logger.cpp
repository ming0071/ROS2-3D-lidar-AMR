// #include <chrono>
// #include <fstream>
// #include <memory>
// #include <string>
// #include <vector>
// #include <sstream>

// #include "rclcpp/rclcpp.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"

// using namespace std::chrono_literals;

// class PoseLogger : public rclcpp::Node
// {
// public:
//     using NavigateToPose = nav2_msgs::action::NavigateToPose;
//     using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

//     PoseLogger()
//     : Node("pose_logger"),
//       tf_buffer_(this->get_clock()),
//       tf_listener_(tf_buffer_)
//     {
//         log_active_ = false;
//         log_file_path_ = "/home/scl/ros2_ws/src/scl_amr/data/Path/base_link_log.txt";

//         nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

//         // 等 action server
//         timer_ = this->create_wall_timer(1s, std::bind(&PoseLogger::check_action_server, this));
//     }

// private:
//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;
//     rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::TimerBase::SharedPtr log_timer_;

//     std::vector<std::string> log_data_;
//     bool log_active_;
//     std::string log_file_path_;

//     void check_action_server()
//     {
//         if (!nav_action_client_->wait_for_action_server(1s)) {
//             RCLCPP_INFO(this->get_logger(), "Waiting for navigate_to_pose action server...");
//             return;
//         }
//         // 等候導航指令後開始記錄
//         log_active_ = true;
//         log_timer_ = this->create_wall_timer(200ms, std::bind(&PoseLogger::log_pose, this));
//         timer_->cancel();

//         // 監聽導航結果
//         auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//         options.result_callback = std::bind(&PoseLogger::result_callback, this, std::placeholders::_1);

//         // 注意：不送 goal，只訂閱結果。用空 goal 嘗試取得目標 handle 可能無效。
//         // 這裡改用訂閱 action 狀態，實務上你可用 lifecycle 或 topic 監控，或改由其他 node 送出 goal
//     }

//     void log_pose()
//     {
//         if (!log_active_) return;

//         try {
//             geometry_msgs::msg::TransformStamped transform =
//                 tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

//             std::ostringstream ss;
//             double time_sec = now().seconds();
//             ss << time_sec << ", "
//                << transform.transform.translation.x << ", "
//                << transform.transform.translation.y << ", "
//                << transform.transform.translation.z;

//             log_data_.push_back(ss.str());
//             RCLCPP_INFO(this->get_logger(), "Logged: %s", ss.str().c_str());

//         } catch (const tf2::TransformException & ex) {
//             RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
//         }
//     }

//     void result_callback(const GoalHandle::WrappedResult & result)
//     {
//         log_active_ = false;
//         log_timer_->cancel();

//         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//             RCLCPP_INFO(this->get_logger(), "Navigation succeeded. Saving log...");
//             write_log_to_file();
//         } else {
//             RCLCPP_WARN(this->get_logger(), "Navigation failed or canceled. Discarding log.");
//         }

//         rclcpp::shutdown();
//     }

//     void write_log_to_file()
//     {
//         std::ofstream out_file(log_file_path_, std::ios::out);
//         for (const auto & line : log_data_) {
//             out_file << line << std::endl;
//         }
//         out_file.close();
//         RCLCPP_INFO(this->get_logger(), "Log saved to %s", log_file_path_.c_str());
//     }
// };

// int main(int argc, char ** argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PoseLogger>());
//     rclcpp::shutdown();
//     return 0;
// }
