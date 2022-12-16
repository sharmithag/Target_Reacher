#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <memory>
#include "bot_controller/bot_controller.h"
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/bool.hpp>

// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller); 

protected:

    // void goal_check_callback(const std_msgs::msg::Bool::SharedPtr &msg);
    void control_loop();

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_subscription_; //!< A subcriber to the /goal_reached topic.
    bool goal_reached_; //!< To store the goal reached state.
};