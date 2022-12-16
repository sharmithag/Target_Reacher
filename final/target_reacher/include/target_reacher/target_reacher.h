#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
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

    }

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller;
};