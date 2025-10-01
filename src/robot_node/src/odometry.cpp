#include "odom_publisher.hpp"

OdomPublisher::OdomPublisher()
    : Node("odom_publisher")
{
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("publish_tf", true);

    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("publish_tf", publish_tf_);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ap/pose/filtered", 10, std::bind(&OdomPublisher::pose_callback, this, std::placeholders::_1));
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/ap/twist/filtered", 10, std::bind(&OdomPublisher::twist_callback, this, std::placeholders::_1));
}

void OdomPublisher::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_pose_ = msg->pose;
    publish_odometry(msg->header.stamp);
}

void OdomPublisher::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    current_twist_ = msg->twist;
}

void OdomPublisher::publish_odometry(const rclcpp::Time& stamp)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = current_pose_;
    odom.twist.twist = current_twist_;

    odom_pub_->publish(odom);

    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = current_pose_.position.x;
        tf.transform.translation.y = current_pose_.position.y;
        tf.transform.translation.z = current_pose_.position.z;
        tf.transform.rotation = current_pose_.orientation;
        tf_broadcaster_->sendTransform(tf);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}