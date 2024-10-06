#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

class PIDController
{
public:
    PIDController(double Kp, double Ki, double Kd)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double error, double dt)
    {
        // 计算积分项
        integral_ += error * dt;

        // 计算微分项
        double derivative = (error - prev_error_) / dt;

        // 计算控制输出
        double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;

        // 更新上一次的误差
        prev_error_ = error;

        return output;
    }

private:
    double Kp_, Ki_, Kd_;  // PID增益
    double prev_error_;    // 上一次的误差
    double integral_;      // 积分项
};


class node: public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target,catcher;
    bool reach_flag;

    PIDController linearPID_;
    PIDController angularPID_;

    // Well, these two variables are not used in the code, but they may help in the task
    bool inital_flag;
    geometry_msgs::msg::TransformStamped tf_inital;

    void timer_callback()
    {

        geometry_msgs::msg::TransformStamped tf;     
        try
        {
            tf = tf_buffer_->lookupTransform(catcher, target, tf2::TimePointZero);

        } catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", catcher.c_str(), target.c_str(), ex.what());
            return;
        }

        

        auto t = tf.transform;
        auto message = geometry_msgs::msg::Twist();



        if (hypot(t.translation.x, t.translation.y) < 0.1)
        {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
            if (!reach_flag) RCLCPP_INFO(this->get_logger(), "Reached target");
            reach_flag = true;
            return;
        }
        else
        {
            reach_flag = false;
        }

        // message.linear.x = 0.5 * hypot(t.translation.x, t.translation.y);
        message.linear.x = linearPID_.compute(hypot(t.translation.x, t.translation.y), 0.1);
        // message.angular.z = 1.0 * atan2(t.translation.y, t.translation.x);
        message.angular.z = angularPID_.compute(atan2(t.translation.y, t.translation.x), 0.1);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: '%f', angular.z: '%f'", message.linear.x, message.angular.z);
        publisher_->publish(message);

    }
public: 
    node(std::string target, std::string catcher): Node("follower"), target(target), catcher(catcher),
        linearPID_(1.0, 0.0, 0.0), angularPID_(1.0, 0.0, 0.0)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&node::timer_callback, this));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(catcher+"/cmd_vel", 10);
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
    ROS_EVENT_LOOP(int argc, char *argv[], std::string target,std::string catcher)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<node>(target,catcher));
    }
    ~ROS_EVENT_LOOP()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[])
{
    if (argc != 3 )
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: follow target catcher");
        return 1;
    }
    ROS_EVENT_LOOP(argc, argv, argv[1], argv[2]);
    return 0;
}