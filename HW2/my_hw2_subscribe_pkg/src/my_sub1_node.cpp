#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_interface/msg/hw2.hpp"

class Subscriber : public rclcpp::Node
{
  public:
    Subscriber()
    : Node("hw2_Subscriber")
    {
      subscription_ = this->create_subscription<ros2_custom_interface::msg::Hw2>(
        "topic", 10, std::bind(&Subscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const ros2_custom_interface::msg::Hw2::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Received: text='%s', int=%d, float=%.2f",
                  msg->message.c_str(), msg->num, msg->value);
    }

    rclcpp::Subscription<ros2_custom_interface::msg::Hw2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}
