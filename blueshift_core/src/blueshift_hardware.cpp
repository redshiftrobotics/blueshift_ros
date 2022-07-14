#include "rclcpp/rclcpp.hpp"
#include "customi2clib/i2c.h"
#include "blueshift_interfaces/msg/motors.hpp"

class Control : public rclcpp::Node
{
  public:
    // node name
    Control()
    : Node("minimal_subscriber")
    {
      
      subscription_ = this->create_subscription<blueshift_interfaces::msg::Motors>(
      "motor_speeds", 10, std::bind(&Control::topic_callback, this, std::placeholders::_1));

        /* Open i2c bus /dev/i2c-0 */
        if ((bus = i2c_open("/dev/i2c-8")) == -1) {

            /* Error process */
        }

        memset(&device, 0, sizeof(device));

        /* 24C04 */
        device.bus = bus;	/* Bus 0 */
        device.addr = 0x42;	/* Target address is 0x42, 7-bit */
        device.iaddr_bytes = 1;	/* Device internal address is 1 byte */
        device.page_bytes = 16; /* Device are capable of 16 bytes per page */
    }
    ~Control()
    {
      i2c_close(bus);
    }

  private:
    // recieving the information
    // void topic_callback(const blueshift_interfaces::msg::Num::SharedPtr msg) const
    void topic_callback(const blueshift_interfaces::msg::Motors &msg) const
    {
        unsigned char newBuffer[3] = "hi";
        ssize_t newSize = sizeof(newBuffer);

        ssize_t test = i2c_write(&device, 0x0, newBuffer, newSize);
    }

    

    rclcpp::Subscription<blueshift_interfaces::msg::Motors>::SharedPtr subscription_;
    int bus;
    I2CDevice device;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Control>());
    rclcpp::shutdown();
    return 0;
}