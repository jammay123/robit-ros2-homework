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
        // 퍼블리셔 초기화: hw3_cmd_vel 토픽으로 Hw3 메시지 발행
        publisher_ = this->create_publisher<ros2_custom_interface::msg::Hw3>("hw3_cmd_vel", 10);
    }

    // 거북이를 앞으로 움직임
    void move_forward()
    {
        auto msg = ros2_custom_interface::msg::Hw3();
        msg.linear_x = 2.0;  // 앞으로 이동
        msg.angular_z = 0.0; // 회전 없음
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving forward");
    }

    // 거북이를 뒤로 움직임
    void move_backward()
    {
        auto msg = ros2_custom_interface::msg::Hw3();
        msg.linear_x = -2.0; // 뒤로 이동
        msg.angular_z = 0.0; // 회전 없음
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Moving backward");
    }

    // 거북이를 좌회전
    void turn_left()
    {
        auto msg = ros2_custom_interface::msg::Hw3();
        msg.linear_x = 0.0; // 이동 없음
        msg.angular_z = 1.0; // 좌회전
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Turning left");
    }

    // 거북이를 우회전
    void turn_right()
    {
        auto msg = ros2_custom_interface::msg::Hw3();
        msg.linear_x = 0.0; // 이동 없음
        msg.angular_z = -1.0; // 우회전
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Turning right");
    }

private:
    rclcpp::Publisher<ros2_custom_interface::msg::Hw3>::SharedPtr publisher_;
};

#endif // TURTLE_CONTROL_NODE_H

int main(int argc, char *argv[])
{
    // ROS2 초기화 및 노드 실행
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlNode>();

    // spin은 Qt와 통합하는 경우 필요하지 않을 수 있지만, CLI로 테스트 시 필요
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
