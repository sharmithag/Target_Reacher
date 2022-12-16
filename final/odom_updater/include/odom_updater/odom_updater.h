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

// robot1/odom
// robot1/base_footprint
class BroadcastPose : public rclcpp::Node
{
    public:
        BroadcastPose (const std::string &node_name = "odom_updater");

    protected:
        void odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg);
        void send_transform (geometry_msgs::msg::TransformStamped t_msg);

    private:
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; //!< The frame broadcaster.
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; //!< The subscriber to the robot1/odom topic.

}; //BroadcastPose

#endif //__ODOM_UPDATER_H__