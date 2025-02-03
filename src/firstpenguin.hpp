#ifndef FIRSTPENGUIN_HPP
#define FIRSTPENGUIN_HPP

#include "mbed.h"

extern BufferedSerial pc;
extern CAN can;

class FirstPenguin {
public:
    static constexpr int max = INT16_MAX;
    FirstPenguin(uint32_t id, CAN& can_interface);
    bool send();
    void read(const CANMessage& msg);
    int16_t pwm[4] = {};
    struct {
        int32_t enc;
        uint32_t adc;
        void set(const uint8_t data[8]) {
            memcpy(this, data, sizeof(*this));
        }
    } receive[4] = {};
private:
    uint32_t send_id;
    CAN& can_interface;
};

#endif // FIRSTPENGUIN_HPP