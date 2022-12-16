#include <target_reacher.h>

TargetReacher::TargetReacher(std::shared_ptr<BotController> const &bot_controller)
    : Node("target_reacher")
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

    // Initializing attributes and states
    m_bot_controller = bot_controller;
    aruco_goal_sent_ = false;
    aruco_reached_ = false;
    finding_aruco_ = false;
    aruco_found_ = false;

    // Creating a scbscriber to check if goal reached
    goal_subscription_ = this->create_subscription<std_msgs::msg::Bool>("goal_reached", 10, std::bind(&TargetReacher::goal_check_callback, this, std::placeholders::_1));

    // Creating a scbscriber to check the aruco marker
    aruco_marker_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1));

    // The publisher to send the twist messages
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

    // Creating the control loop
    control_loop_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TargetReacher::control_loop, this));
}

void TargetReacher::control_loop()
{
    if (!aruco_goal_sent_ && !aruco_found_)
    {
        // ROBOT STATE: Sending the aruco marker position.
        std::this_thread::sleep_for(std::chrono::seconds(5));
        m_bot_controller->set_goal(this->get_parameter("aruco_target.x").as_double(), this->get_parameter("aruco_target.y").as_double());
        RCLCPP_INFO(this->get_logger(), "ARUCO Goal set");
        aruco_goal_sent_ = true;
    }
    else if (!finding_aruco_)
    {
        // ROBOT STATE: Moving towards aruco marker.
        if (aruco_reached_)
        {
            RCLCPP_INFO(this->get_logger(), "ARUCO Reached");
            finding_aruco_ = true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Travelling towards ARUCO!");
        }
    }
    else if (!aruco_found_)
    {
        // ROBOT STATE: Rotating to find aruco marker.
        geometry_msgs::msg::Twist temp;
        temp.angular.z = 0.2;
        twist_publisher_->publish(temp);
    }
    else
    {
        geometry_msgs::msg::Twist temp;
        temp.angular.z = 0.0;
        twist_publisher_->publish(temp);
    }
}

void TargetReacher::goal_check_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    aruco_reached_ = msg->data;
}

void TargetReacher::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    aruco_found_ = true;
}