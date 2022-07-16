#include "rclcpp/rclcpp.hpp"
#include "blueshift_interfaces/msg/motors.hpp"
#include "i2c/i2c.hpp"

class Control : public rclcpp::Node
{
public:
  // node name
  Control()
      : Node("minimal_subscriber"), bus("/dev/i2c-8"), device(bus,0x32)
  {
    
    subscription_ = this->create_subscription<blueshift_interfaces::msg::Motors>(
        "motor_speeds", 10, std::bind(&Control::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const blueshift_interfaces::msg::Motors &msg)
  {
    RCLCPP_INFO(this->get_logger(), "I received '%f'", msg.top_front_left);
    for (char c: std::to_string(msg.top_front_left)){
      device.writeByte(0x00,c);
    }
    
  }

  Bus bus;
  Device device;
  rclcpp::Subscription<blueshift_interfaces::msg::Motors>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}