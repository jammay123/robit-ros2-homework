#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_interface/msg/hw2.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
  public:
    Publisher()
    : Node("hw2_Publisher")
    {
      publisher_ = this->create_publisher<ros2_custom_interface::msg::Hw2>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Publisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = ros2_custom_interface::msg::Hw2();

      std::string text_input;
      int32_t int_input;
      float float_input;

      std::cout << "Enter string: ";
      std::cin >> text_input;
      std::cout << "Enter num: ";
      std::cin >> int_input;
      std::cout << "Enter float: ";
      std::cin >> float_input;

      message.message = text_input;
      message.num = int_input;
      message.value = float_input;

      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2_custom_interface::msg::Hw2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
