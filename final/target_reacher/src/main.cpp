/**
 * @file main.cpp
 * @author Aneesh Chodisetty (aneeshch@umd.edu)
 * @author Orlandis Devon Smith (osmith15@umd.edu)
 * @author Sharmitha Ganesan (sganesa3@umd.edu)
 * @brief The main file that spins the launches the BotController and TargetReacher nodes.
 * @version 0.1
 * @date 2022-12-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <target_reacher.h>
#include "bot_controller/bot_controller.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto bot_controller = std::make_shared<BotController>("bot_controller_robot", "robot1");
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<TargetReacher>(bot_controller);
    exec.add_node(node);
    exec.add_node(bot_controller);
    exec.spin();
    rclcpp::shutdown();
}