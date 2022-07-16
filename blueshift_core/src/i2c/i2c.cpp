extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include "i2c.hpp"


//g++ i2c.cpp -li2c -o test

Bus::Bus(std::string busPath) : path(busPath) {}


Device::Device(Bus &bus, int address) : bus(this->bus), address(this->address)
{
    file = open(bus.path.c_str(), I2C_RDWR);
    if (ioctl(file, I2C_SLAVE, address) < 0)
    {
        initFail = true;
        // ERROR
    }
    initFail = false;
}
Device::~Device()
{
    close(file);
}

uint16_t Device::readWord(uint8_t reg)
{
    int res = i2c_smbus_read_word_data(file, reg);
    if (res < 0)
    {
        // Error
    }
    return res;
}

// uint16_t Device::readWordSwapped(uint8_t reg)
// {
//     int res = i2c_smbus_read_word_data(file, reg);
//     if (res < 0)
//     {
//         // Error
//     }
//     return res;
// }

uint8_t Device::readByte(uint8_t reg)
{
    int res = i2c_smbus_read_byte_data(file, reg);
    if (res < 0)
    {
        // Error
    }
    return res;
}

void Device::writeWord(uint8_t reg, uint8_t dataHigh, uint8_t dataLow)
{
    int res = i2c_smbus_write_word_data(file, reg, ((uint16_t)dataLow << 8) | dataHigh);
    if (res < 0)
    {
        // Error
    }
}

// void Device::writeWordSwapped(uint8_t reg, uint8_t dataLow, uint8_t dataHigh)
// {
//     int res = i2c_smbus_write_word_data_swapped(file, reg, dataLow, dataHigh);
//     if (res < 0)
//     {
//         // Error
//     }
// }

void Device::writeByte(uint8_t reg, uint8_t data)
{
    int res = i2c_smbus_write_byte_data(file, reg, data);
    if (res < 0)
    {
        // Error
    }
}

int Device::getAddress()
{
    return address;
}

// int main(){
//     Bus b("/dev/i2c-8");
//     Device d(b,0x32);
//     d.writeWord(0x00,'H','i');

//     return 0;
// }