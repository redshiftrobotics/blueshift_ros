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
#include <iostream>


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

int Device::getAddress()
{
    return address;
}

uint8_t Device::readByte(uint8_t reg)
{
    int res = i2c_smbus_read_byte_data(file, reg);
    if (res < 0)
    {
        // Error
    }
    return res;
}

void Device::writeByte(uint8_t reg, uint8_t data)
{
    int res = i2c_smbus_write_byte_data(file, reg, data);
    if (res < 0)
    {
        // Error
    }
}


uint16_t Device::readWord(uint8_t reg)
{
    int res = i2c_smbus_read_byte_data(file, reg);

    if (res < 0)
    {
        std::cout << res << std::endl;
    }

    int res2 = i2c_smbus_read_byte_data(file,reg+1);
    return ((uint16_t)res2<<8)+res;
}


void Device::writeWord(uint8_t reg, uint8_t dataHigh, uint8_t dataLow)
{
    int res = i2c_smbus_write_byte_data(file, reg, dataLow);
    if (res < 0)
    {
        // Error
    }

    int res2 = i2c_smbus_write_byte_data(file,reg+1,dataHigh);
    if (res2 < 0)
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

// uint16_t Device::readWordSwapped(uint8_t reg)
// {
//     int res = i2c_smbus_read_word_data(file, reg);
//     if (res < 0)
//     {
//         // Error
//     }
//     return res;
// }


// int main(){
//     Bus b("/dev/i2c-8");
//     Device d(b,0x2a);

//     d.writeWord(4,0x01,0x00);s
//     std::cout << (int) d.readWord(4) << std::endl;
    
//     return 0;
// }