// turtle_control_node.h

#ifndef TURTLE_CONTROL_NODE_H
#define TURTLE_CONTROL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "ros2_custom_interface/msg/hw3.hpp"

class TurtleControlNode : public rclcpp::Node
{
public:
    TurtleControlNode() : Node("turtle_control_node")
    {

        publisher_ = this->create_publisher<ros2_custom_interface::msg::Hw3>("hw3_cmd_vel", 10);
    }


    void move_forward()
    {
        auto msg = ros2_custom_interface::msg::Hw3();
        msg.linear_x = 2.0; 
        msg.angular_z = 0.0; 
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
    }


    void move_backward()
    {
        auto msg = ros2_custom_interface::msg::Hw3();
        msg.linear_x = -2.0;
        msg.angular_z = 0.0;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving backward");
    }


    void turn_left()
    {
        auto msg = ros2_custom_interface::msg::Hw3();
        msg.linear_x = 0.0; 
        msg.angular_z = 1.0; 
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Turning left");
    }

    void turn_right()
    {
        auto msg = ros2_custom_interface::msg::Hw3();
        msg.linear_x = 0.0; 
        msg.angular_z = -1.0;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Turning right");
    }

private:
    rclcpp::Publisher<ros2_custom_interface::msg::Hw3>::SharedPtr publisher_;
};

#endif 

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
