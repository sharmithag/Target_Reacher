/**
 * @file odom_updater.h
 * @author Aneesh Chodisetty (aneeshch@umd.edu)
 * @author Orlandis Devon Smith (osmith15@umd.edu)
 * @author Sharmitha Ganesan (sganesa3@umd.edu)
 * @brief The odom updates that joins the TF trees present.
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __ODOM_UPDATER_H__
#define __ODOM_UPDATER_H__

#include <memory>
#include <string>
#include <sstream>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class BroadcastPose : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Broadcast Pose object.
         * 
         * @param node_name The name of the node.
         */
        BroadcastPose(const std::string &node_name = "odom_updater");

    protected:
        /**
         * @brief The callback that send a transfrom between robot1/odom and robot1/base_footprint frames.
         * 
         * @param msg The message read from the robot1/odom topic.
         */
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        
        /**
         * @brief The Method to send the created transform.
         * 
         * @param t_msg The transform that was created in the callback
         */
        void send_transform(geometry_msgs::msg::TransformStamped t_msg);

    private:
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; //!< The frame broadcaster.
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; //!< The subscriber to the robot1/odom topic.

}; //BroadcastPose

#endif //__ODOM_UPDATER_H__