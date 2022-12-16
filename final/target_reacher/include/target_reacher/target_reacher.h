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

    void goal_check_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
    void control_loop();

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_subscription_; //!< A subcriber to the /goal_reached topic.
    rclcpp::TimerBase::SharedPtr control_loop_; //!< The control loop for the whole system.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;  //!< The pointer to the publisher.
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_marker_; //!< Subscription to check the aruco marker
    
    // States of teh control loop
    bool aruco_reached_; //!< To store the goal reached state.
    bool aruco_goal_sent_; //!< To keep track of the goal send status.
    bool finding_aruco_; //!< To keep track of finding the aruco
    bool aruco_found_; //!< To check if aruco is found.
};