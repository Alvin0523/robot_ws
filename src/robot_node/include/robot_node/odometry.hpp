#pragma once

#include <rclcpp/rclcpp.hpp> // Core ROS2 C++ API: Node, Publisher, Subscriber, etc.
#include <nav_msgs/msg/odometry.hpp> // For nav_msgs/Odometry message type
#include <geometry_msgs/msg/pose_stamped.hpp> // For geometry_msgs/PoseStamped
#include <geometry_msgs/msg/twist_stamped.hpp> // For geometry_msgs/TwistStamped
#include <tf2_ros/transform_broadcaster.h> // For publishing TF transforms
#include <string> // For using std::string
#include <memory> // For using std::shared_ptr and std::unique_ptr

class OdomPublisher : public rclcpp::Node //Inherit from rcl.cpp::Node to create a ROS2 node
{
    
public:
    OdomPublisher(); // Constructor to initialize the node

