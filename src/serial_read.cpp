#include "serial_read.hpp"
#include "mbed.h"
#include <string>

std::string serial_unit::read_serial()
{
    std::string buff_str = "";
    char buf[1];

    while (men_serial.readable())
    {
        men_serial.read(buf, 1);
        if (buf[0] == '|')
        {
            buff_str = str;
            str.clear();
            return buff_str;
        }
        else
        {
            str += buf[0];
        }
    }
    return "";
}