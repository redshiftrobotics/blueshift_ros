#include <string>

class Bus{
    public:
        Bus(std::string busPath);
        std::string path;
};

class Device{
    public:
        Device(Bus &_bus,int _address);
        ~Device();
        uint16_t readWord(uint8_t reg);
        //uint16_t readWordSwapped(uint8_t reg);
        uint8_t readByte(uint8_t reg);
        void writeWord(uint8_t reg, uint8_t dataLow, uint8_t dataHigh);
        //void writeWordSwapped(uint8_t reg, uint8_t dataLow, uint8_t dataHigh);
        void writeByte(uint8_t reg, uint8_t data);
        int getAddress();
        uint16_t encode(float num);
        float decode(uint16_t num);
    private:
        int address;
        Bus &bus;
        int file;
        bool initFail;
};