#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream> 

using namespace std;

class DiabloSquarePathNode : public rclcpp::Node
{
public:
    DiabloSquarePathNode() : Node("diablo_simulation")
    {
        RCLCPP_INFO(this->get_logger(), "Node Started!");

        motion_subscriber_ = this->create_subscription<motion_msgs::msg::MotionCtrl>(
            "/simulation/velocity", 1000, std::bind(&DiabloSquarePathNode::MotionCallback, this, std::placeholders::_1));

        diablo_pose_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/simulation/odom", 1000);

        step = 0.04;
        x = 0.0; y = 0.0; yaw = 0;
        vx = 0;
        vy = 0;
        x_next = 0;
        y_next = 0;

    }
    
private:

    void MotionCallback(const motion_msgs::msg::MotionCtrl::SharedPtr cmd)
    {
        double v = cmd->value.forward;
        double w = cmd->value.left;

        //Update positions
        x = x_next;
        y = y_next;
        x_next = x + (v * cos(yaw) * step);
        y_next = y + (v * sin(yaw) * step);
        yaw = yaw + (w * step);

        //Update velocities
        vx = (x_next-x)/step;
        vy = (y_next-y)/step;

        nav_msgs::msg::Odometry msg;
        msg.pose.pose.orientation.x = yaw;
        msg.pose.pose.position.x = x_next;
        msg.pose.pose.position.y = y_next;

        msg.twist.twist.linear.x = vx;
        msg.twist.twist.linear.y = vy;

        diablo_pose_publisher_->publish(msg);
    }

    rclcpp::Subscription<motion_msgs::msg::MotionCtrl>::SharedPtr motion_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr diablo_pose_publisher_;
    double x, y, x_next, y_next, yaw, step;
    double vx, vy;
    ofstream data_file_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiabloSquarePathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

