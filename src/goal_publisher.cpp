#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class GoalPublisher : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoalPublisher() : Node("goal_publisher")
    {
        declare_parameter("goal_x", 1.0);
        declare_parameter("goal_y", 2.0);
        declare_parameter("goal_yaw", 0.0);

        client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
        timer_ = this->create_wall_timer(1s, std::bind(&GoalPublisher::try_send_goal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void try_send_goal()
    {
        if (!client_->wait_for_action_server(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
            return;
        }

        double x = get_parameter("goal_x").as_double();
        double y = get_parameter("goal_y").as_double();
        double yaw = get_parameter("goal_yaw").as_double();

        RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

        geometry_msgs::msg::Quaternion orientation;
        orientation.w = cos(yaw / 2.0);
        orientation.z = sin(yaw / 2.0);

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.orientation = orientation;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        client_->async_send_goal(goal, options);

        timer_->cancel(); // 送一次就好
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}
