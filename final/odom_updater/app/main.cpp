/**
 * @file main.cpp
 * @author Aneesh Chodisetty (aneeshch@umd.edu)
 * @author Orlandis Devon Smith (osmith15@umd.edu)
 * @author Sharmitha Ganesan (sganesa3@umd.edu)
 * @brief The main function that spins the created BroadcastPose node.
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <memory>

#include <odom_updater.h>

int main(int argc, char** argv)
{
    // Initializing rclcpp.
    rclcpp::init(argc, argv);

    // Starting the Node.
    rclcpp::spin(std::make_shared<BroadcastPose>());

    // Shutting down the node.
    rclcpp::shutdown();

}