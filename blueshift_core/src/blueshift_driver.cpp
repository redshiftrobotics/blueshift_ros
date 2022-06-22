#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic2", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "yes '%s'", std::to_string(msg.linear.x).c_str());
    auto message = std_msgs::msg::String();
    message.data = "hi";
    publisher_->publish(message);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}