#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "holonomic.hpp"
#include "blueshift_interfaces/msg/motors.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    publisher_ = this->create_publisher<blueshift_interfaces::msg::Motors>("motor_speeds", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "input", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I recieved Linear x:'%s'", std::to_string(msg.linear.x).c_str());
    auto message = blueshift_interfaces::msg::Motors();
    Motors motor = holonomic_math(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);

    message.top_front_left = motor.top_front_left;
    message.top_front_right = motor.top_front_right;
    message.top_back_left = motor.top_back_left;
    message.top_back_right = motor.top_back_right;
    message.bottom_front_left = motor.bottom_front_left;
    message.bottom_front_right = motor.bottom_front_right;
    message.bottom_back_left = motor.bottom_back_left;
    message.bottom_back_right = motor.bottom_back_right;

    publisher_->publish(message);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<blueshift_interfaces::msg::Motors>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}