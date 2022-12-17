/**
 * @file target_reacher.h
 * @author Aneesh Chodisetty (aneeshch@umd.edu)
 * @author Orlandis Devon Smith (osmith15@umd.edu)
 * @author Sharmitha Ganesan (sganesa3@umd.edu)
 * @brief The main control loop that controls the robot and makes it reach the target.
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
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

class TargetReacher : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Target Reacher object.
     * 
     * @param bot_controller This is control the robot.
     */
    TargetReacher(std::shared_ptr<BotController> const &bot_controller); 

protected:
    /**
     * @brief Checks the /goal_reached topic to check if the set goal is reached.
     * 
     * @param msg The message read from /goal_reached.
     */
    void goal_check_callback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Checks the /aruco_markers topic to get the marker ID.
     * 
     * @param msg The message read from /aruco_markers topic.
     */
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief Create a static transfrom between the frame ID in the .yaml file and final_destination. This final destination is read from the aruco marker.
     * 
     */
    void final_transform();

    /**
     * @brief This is the main control loop that does the following.
     *          1. Moves the robot towards the aruco marker.
     *          2. Seeks the aruco marker.
     *          3. Decodes the aruco marker for the marker id.
     *          4. Uses the marker ID to create a static transform between the frame_id defined in the .yaml file and the aruco destination.
     *          5. Transforms the final destination to robot1/odom frame.
     *          6. Moves the robot towards the final destination.
     * 
     */
    void control_loop();

    /**
     * @brief The method that transforms final_destination to the robot1/odom frame and then sends the robot to the final destination.
     * 
     */
    void transform_and_send_goal();

private:
    // attributes
    std::shared_ptr<BotController> m_bot_controller; //!< The pointer the BotController.
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

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; //!< Buffer for the final goal transform.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; //!< Tf listener for the final transform.
};