#include "firstpenguin.hpp"

FirstPenguin::FirstPenguin(uint32_t id, CAN& can_interface) : send_id(id), can_interface(can_interface) {}

bool FirstPenguin::send() {
    return can_interface.write(CANMessage{send_id, reinterpret_cast<const uint8_t*>(pwm), 8});
}

void FirstPenguin::read(const CANMessage &msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == sizeof(receive[0]) && send_id < msg.id &&
       msg.id <= send_id + 5) {
        receive[msg.id - send_id - 1].set(msg.data);
    }
}