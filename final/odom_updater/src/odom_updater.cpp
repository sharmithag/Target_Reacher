/**
 * @file odom_updater.cpp
 * @author Aneesh Chodisetty (aneeshch@umd.edu)
 * @author Orlandis Devon Smith (osmith15@umd.edu)
 * @author Sharmitha Ganesan (sganesa3@umd.edu)
 * @brief The definition for the BroadcastPose class.
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <odom_updater.h>

BroadcastPose::BroadcastPose(const std::string &node_name)
    : Node(node_name)
{
    // Setting up the tf broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Setting up the subscriber
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("robot1/odom", 10, std::bind(&BroadcastPose::odom_callback, this, std::placeholders::_1));
}

void BroadcastPose::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // To store the current message
    geometry_msgs::msg::TransformStamped current_msg; 

    // Creating the header
    current_msg.header.stamp = this->get_clock()->now();
    current_msg.header.frame_id = "robot1/odom";
    current_msg.child_frame_id = "robot1/base_footprint";

    // Storing the translation
    current_msg.transform.translation.x = msg->pose.pose.position.x;
    current_msg.transform.translation.y = msg->pose.pose.position.y;
    current_msg.transform.translation.z = msg->pose.pose.position.z;

    // Storing the orientation
    current_msg.transform.rotation.w = msg->pose.pose.orientation.w;
    current_msg.transform.rotation.x = msg->pose.pose.orientation.x;
    current_msg.transform.rotation.y = msg->pose.pose.orientation.y;
    current_msg.transform.rotation.z = msg->pose.pose.orientation.z;

    // Sending the transform
    send_transform(current_msg);
}

void BroadcastPose::send_transform(geometry_msgs::msg::TransformStamped t_msg)
{
    tf_broadcaster_->sendTransform(t_msg);
}
