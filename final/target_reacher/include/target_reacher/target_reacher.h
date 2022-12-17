#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <memory>
#include "bot_controller/bot_controller.h"
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

// timer
class TargetReacher : public rclcpp::Node
{
public:
    TargetReacher(std::shared_ptr<BotController> const &bot_controller); 

protected:

    void goal_check_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
    void final_transform();
    void control_loop();
    void transform_and_send_goal();

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_subscription_; //!< A subcriber to the /goal_reached topic.
    rclcpp::TimerBase::SharedPtr control_loop_; //!< The control loop for the whole system.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;  //!< The pointer to the publisher.
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_marker_; //!< Subscription to check the aruco marker
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> final_goal_transform_; //!< The final goal's tranformation.
    
    // States of the control loop
    bool goal_reached_; //!< To store the goal reached state.
    bool aruco_goal_sent_; //!< To keep track of the goal send status.
    bool finding_aruco_; //!< To keep track of finding the aruco
    bool aruco_found_; //!< To check if aruco is found.
    bool transform_created_; //!< To create the static transform.
    bool final_goal_decoded_; //!< To check if the final goal is decoded.
    bool final_goal_reached_; //!< To check if the final goal is reached.

    int64_t marker_id_; //!< To store the marker ID read from the aruco marker.
    std::string frame_id_; //!< To store the frame ID of the aruco marker.

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; //!< Buffer for the final goal transform.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; //!< Tf listener for the final transform.
};