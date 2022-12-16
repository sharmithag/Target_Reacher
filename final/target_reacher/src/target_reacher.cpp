#include <rclcpp/rclcpp.hpp>
#include <target_reacher.h>

TargetReacher::TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {
        // Declaring the parameters.
        this->declare_parameter<float>("aruco_target.x");
        this->declare_parameter<float>("aruco_target.y");
        this->declare_parameter<std::string>("final_destination.frame_id");
        this->declare_parameter<float>("final_destination.aruco_0.x");
        this->declare_parameter<float>("final_destination.aruco_0.y");
        this->declare_parameter<float>("final_destination.aruco_1.x");
        this->declare_parameter<float>("final_destination.aruco_1.y");
        this->declare_parameter<float>("final_destination.aruco_2.x");
        this->declare_parameter<float>("final_destination.aruco_2.y");
        this->declare_parameter<float>("final_destination.aruco_3.x");
        this->declare_parameter<float>("final_destination.aruco_3.y");
        m_bot_controller = bot_controller;
        // goal_subscription_ = this->create_subscription<std_msgs::msg::Bool>("goal_reached", 10, std::bind(&TargetReacher::goal_check_callback, this, std::placeholders::_1));

        control_loop();
    }

void TargetReacher::control_loop()
{
    // Sending the bot to goal
    std::this_thread::sleep_for(std::chrono::seconds(5));
    m_bot_controller->set_goal(this->get_parameter("aruco_target.x").as_double(), this->get_parameter("aruco_target.y").as_double());
}

// void TargetReacher::goal_check_callback(const std_msgs::msg::Bool::SharedPtr &msg)
// {
//     goal_reached_ = msg->data;
// }