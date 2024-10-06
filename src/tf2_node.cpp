#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "turtlesim/msg/pose.hpp"
#include <functional>
#include <rclcpp/subscription.hpp>
#include <string>
#include <turtlesim/msg/detail/pose__struct.hpp>
#include <vector>

class node: public rclcpp::Node
{
private:
    tf2_ros::TransformBroadcaster tfb;
    // std::string turtleName;
    // rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> subscriptions_;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg, const std::string turtle_name)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtle_name;
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tfb.sendTransform(t);
    }
public:
    node(std::vector<std::string> turtle_names): Node("turtles_boardcaster"), tfb(this)
    {
        for (const auto & turtle_name : turtle_names){
            subscriptions_.push_back(this->create_subscription<turtlesim::msg::Pose>(
            turtle_name+"/pose", 10, [this, turtle_name](const turtlesim::msg::Pose::SharedPtr msg) {this->pose_callback(msg, turtle_name);}));
            // turtle_name+"/pose", 10, std::bind(&node::pose_callback, this, std::placeholders::_1, turtle_name)));
        }
        
        RCLCPP_INFO(this->get_logger(), "Hello, world");
    }
    ~node()
    {
        RCLCPP_INFO(this->get_logger(), "Goodbye, world");
    }
};

class ROS_EVENT_LOOP
{
public:
    ROS_EVENT_LOOP(int argc, char *argv[])
    {
        rclcpp::init(argc, argv);
        std::vector<std::string> turtle_names;
        for (int i = 1; i < argc; i++){
            turtle_names.push_back(argv[i]);
        }
        rclcpp::spin(std::make_shared<node>(turtle_names));
    }
    ~ROS_EVENT_LOOP()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[])
{
    if (argc < 2 )
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: tf turtle_name");
        return 1;
    }
    ROS_EVENT_LOOP(argc, argv);
    return 0;
}