#ifndef SERIAL_READ_HPP
#define SERIAL_READ_HPP

#include "mbed.h"
#include <string>

class serial_unit
{
public:
    serial_unit(BufferedSerial &serial);
    std::string read_serial();

private:
    std::string str;
    BufferedSerial &men_serial;
};

inline serial_unit::serial_unit(BufferedSerial &serial) : men_serial(serial) {}

#endif