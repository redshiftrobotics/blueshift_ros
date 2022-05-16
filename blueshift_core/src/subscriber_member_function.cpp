#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "blueshift_interfaces/msg/num.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    // node name
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      
      subscription_ = this->create_subscription<blueshift_interfaces::msg::Num>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    // recieving the information
    // void topic_callback(const blueshift_interfaces::msg::Num::SharedPtr msg) const
    void topic_callback(const blueshift_interfaces::msg::Num::SharedPtr ) const
    {
      // print the info out
      // RCLCPP_INFO(this->get_logger(), std::to_string(msg->linear.x).c_str());
      RCLCPP_INFO(this->get_logger(), "<< msg->num << ");
    }
    rclcpp::Subscription<blueshift_interfaces::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}