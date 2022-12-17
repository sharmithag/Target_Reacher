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
    goal_reached_ = false;
    finding_aruco_ = false;
    aruco_found_ = false;
    transform_created_ = false;
    final_goal_decoded_ = false;
    final_goal_reached_ = false;

    // Creating a scbscriber to check if goal reached
    goal_subscription_ = this->create_subscription<std_msgs::msg::Bool>("goal_reached", 10, std::bind(&TargetReacher::goal_check_callback, this, std::placeholders::_1));

    // Creating a scbscriber to check the aruco marker
    aruco_marker_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1));

    // The publisher to send the twist messages
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

    // The final transformer
    final_goal_transform_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Creating the control loop
    control_loop_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TargetReacher::control_loop, this));
}

void TargetReacher::control_loop()
{
    if (!final_goal_decoded_)
    {
        if (!aruco_goal_sent_)
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
            if (goal_reached_)
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
            RCLCPP_INFO(this->get_logger(), "Finsing the aruco marker.");
        }
        else if (!transform_created_)
        {
            // ROBOT STATE: Decoding the aruco marker.
            geometry_msgs::msg::Twist temp;
            temp.angular.z = 0.0;
            twist_publisher_->publish(temp);
            RCLCPP_INFO(this->get_logger(), "ARUCO Decoded!");

            
            RCLCPP_INFO(this->get_logger(), "Creating a static transform between %s and final_destination.", frame_id_.c_str());
            this->final_transform();
            // Listening to the transform
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            RCLCPP_INFO(this->get_logger(), "Transform created.");
            transform_created_ = true;
        }
        else
        {
            // ROBOT STATE: Transform the final destination to odom and send goal
            this->transform_and_send_goal();           
        }
    }
    else
    {
        if (!goal_reached_)
        {
            RCLCPP_INFO(this->get_logger(), "Travelling towards Final goal!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal Reached.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Exiting.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Bye, take care!");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "But before we leave....");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Thank you Prof. Zeid, TA and grader for this course.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "Now press ctrl+C to get done with this! Duh....-\\_(o_O)_/-");
            exit(0);
        }
    }
}

void TargetReacher::goal_check_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    goal_reached_ = msg->data;
}

void TargetReacher::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    aruco_found_ = true;
    marker_id_ = msg->marker_ids.at(0);
    frame_id_ = msg->header.frame_id;

}

void TargetReacher::final_transform()
{   
    int64_t marker_id = marker_id_;
    std::string aruco = frame_id_;
    std::string goal_x = "final_destination.aruco_" + std::to_string(marker_id) + ".x";
    std::string goal_y = "final_destination.aruco_" + std::to_string(marker_id) + ".y";

    geometry_msgs::msg::TransformStamped final_transform;

    final_transform.header.stamp = this->get_clock()->now();
    final_transform.header.frame_id = this->get_parameter("final_destination.frame_id").as_string();
    final_transform.child_frame_id = "final_destination";

    // Storing the translation
    final_transform.transform.translation.x = this->get_parameter(goal_x).as_double();
    final_transform.transform.translation.y = this->get_parameter(goal_y).as_double();
    final_transform.transform.translation.z = 0.0;

    // Storing the rotation
    final_transform.transform.rotation.w = 1.0;
    final_transform.transform.rotation.x = 0;
    final_transform.transform.rotation.y = 0;
    final_transform.transform.rotation.z = 0;

    // Sending the transform
    final_goal_transform_->sendTransform(final_transform);

    
}

void TargetReacher::transform_and_send_goal()
{
    geometry_msgs::msg::TransformStamped odom_transform;

    // Transforming the /final_destination to /robot1/odom
    
    try
    {
        odom_transform = tf_buffer_->lookupTransform("robot1/odom", "final_destination", tf2::TimePointZero);
        RCLCPP_INFO(this->get_logger(), "Found transform.");
        m_bot_controller->set_goal(odom_transform.transform.translation.x, odom_transform.transform.translation.y);
        final_goal_decoded_ = true;
        goal_reached_ = false;
    }
    catch(const tf2::TransformException & ex)
    {
        RCLCPP_WARN(this->get_logger(), "Couldn't find transform.");
    }
}