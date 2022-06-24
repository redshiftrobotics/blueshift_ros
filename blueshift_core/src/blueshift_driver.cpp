#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "holonomic.hpp"
#include "blueshift_interfaces/msg/motors.hpp"

class Control : public rclcpp::Node
{
public:
  Control()
      : Node("minimal_subscriber")
  {
    this->declare_parameter("holonomic_speed_limiter", 0);
    publisher_ = this->create_publisher<blueshift_interfaces::msg::Motors>("motor_speeds", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "input", 10, std::bind(&Control::topic_callback, this, std::placeholders::_1));
  }

  void respond(){
    this->get_parameter("holonomic_speed_limiter",parameter_value_);
  }



private:
  void topic_callback(const geometry_msgs::msg::Twist &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I recieved Linear x:'%s'", std::to_string(msg.linear.x).c_str());
    auto message = blueshift_interfaces::msg::Motors();
    
    this->get_parameter("holonomic_speed_limiter",parameter_value_);
    Motors motor = holonomic_math(
    msg.linear.x, msg.linear.y, msg.linear.z, 
    msg.angular.x, msg.angular.y, msg.angular.z, 
    parameter_value_);

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

  int parameter_value_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<blueshift_interfaces::msg::Motors>::SharedPtr publisher_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}