#include "PID.hpp"
#include "firstpenguin.hpp"
#include "mbed.h"
#include <array>
#include <cstring>
#include <string>

CAN can1(PA_11, PA_12, 1e6);
CAN can2(PB_12, PB_13, 1e6);
// FirstPenguin penguin(40, can2);
FirstPenguin penguin(35, can2);
PID pid_controller(0.4, 0.5, 0.001, 0.02);
// PIDコントローラ
PID p_move0(0.2, 0.0, 0.01, 0.02);
PID p_move1(0.2, 0.0, 0.01, 0.02);
PID p_move2(0.2, 0.0, 0.01, 0.02);
PID p_move3(0.2, 0.0, 0.01, 0.02);

int maxspeed = 10000;
BufferedSerial pc(USBTX, USBRX, 250000);
uint8_t servo[8] = {};
uint8_t DATA_move[8] = {};
uint8_t DATA[8] = {};
int leftJoystickX = 0, leftJoystickY = 0, rightJoystickX = 0;
int latemode = 0, Takamatsu = 0, UnderUp = 0;
bool triangle = false;
int sabo = 0, sabo2 = 0;
bool sabohoukou = false;
int l1_direction = 0, R1_direction = 0, square_direction = 0,
    triangle_direction = 0, circle_direction = 0;
int bashibashi = 0;
int guwa = 0;
int16_t move0_speed = 0, move1_speed = 0, move2_speed = 0, move3_speed = 0;
int servovo = 150;

typedef struct {
    signed char LX;
    signed char LY;
    signed char RX;
    signed char RY;

    unsigned char Circle : 1;
    unsigned char Cross : 1;
    unsigned char Square : 1;
    unsigned char Triangle : 1;

    unsigned char Up : 1;
    unsigned char Right : 1;
    unsigned char Down : 1;
    unsigned char Left : 1;

    unsigned char L2 : 1;
    unsigned char R2 : 1;
    unsigned char L1 : 1;
    unsigned char R1 : 1;

    unsigned char SHARE : 1;
    unsigned char OPTION : 1;
    unsigned char PS : 1;
} PS2Con;

PS2Con ps4;

void controller_read(const char *buffer) {
    if (strncmp(buffer, "n:", 2) == 0) {
        int lx, ly, rx, ry;
        if (sscanf(buffer, "n:%d:%d:%d:%d|", &lx, &ly, &rx, &ry) == 4) {
            ps4.LX = static_cast<signed char>(lx);
            ps4.LY = static_cast<signed char>(ly);
            ps4.RX = static_cast<signed char>(rx);
            ps4.RY = static_cast<signed char>(ry);

            printf("LX: %d, LY: %d, RX: %d, RY: %d\n", ps4.LX, ps4.LY, ps4.RX,
                   ps4.RY);
        } else {
            printf("Error parsing joystick data: %s\n", buffer);
        }
    }
    if (strstr(buffer, "Circle:pressing |"))
        ps4.Circle = 1;
    else if (strstr(buffer, "Circle:no_pressing|"))
        ps4.Circle = 0;

    if (strstr(buffer, "Cross:pressing|"))
        ps4.Cross = 1;
    else if (strstr(buffer, "Cross:no_pressing|"))
        ps4.Cross = 0;

    if (strstr(buffer, "Square:pressing|"))
        ps4.Square = 1;
    else if (strstr(buffer, "Square no_pressing|"))
        ps4.Square = 0;

    if (strstr(buffer, "Triangle:pressing|"))
        ps4.Triangle = 1;
    else if (strstr(buffer, "Triangle:no_pressing|"))
        ps4.Triangle = 0;

    if (strstr(buffer, "L1:pressing|"))
        ps4.L1 = 1;
    else if (strstr(buffer, "L1:no_pressing|"))
        ps4.L1 = 0;

    if (strstr(buffer, "R1:pressing|"))
        ps4.R1 = 1;
    else if (strstr(buffer, "R1:no_pressing|"))
        ps4.R1 = 0;

    if (strstr(buffer, "L2:pressing|"))
        ps4.L2 = 1;
    else if (strstr(buffer, "L2:no_pressing|"))
        ps4.L2 = 0;

    if (strstr(buffer, "R2:pressing|"))
        ps4.R2 = 1;
    else if (strstr(buffer, "R2:no_pressing|"))
        ps4.R2 = 0;

    if (strstr(buffer, "SHARE:pressing|"))
        ps4.SHARE = 1;
    else if (strstr(buffer, "SHARE:no_pressing|"))
        ps4.SHARE = 0;

    if (strstr(buffer, "OPTION:pressing|"))
        ps4.OPTION = 1;
    else if (strstr(buffer, "OPTION:no_pressing|"))
        ps4.OPTION = 0;

    if (strstr(buffer, "PS:pressing|"))
        ps4.PS = 1;
    else if (strstr(buffer, "PS:no_pressing|"))
        ps4.PS = 0;
}

void serial_read() {
    char buffer[128] = {};
    while (1) {
        int length = pc.read(buffer, sizeof(buffer) - 1);
        if (length > 0) {
            buffer[length] = '\0'; // Null 終端
            printf("Received: %s\n", buffer);
            controller_read(buffer);
        }
        ThisThread::sleep_for(10ms);
    }
}

void CANReceive() {
    while (1) {
        CANMessage msg;
        if (can1.read(msg)) {
            // printf("kitayo\n");、
            switch (msg.id) {
            case 0x201:
                move0_speed = (msg.data[2] << 8) | msg.data[3];
                break;
            case 0x202:
                move1_speed = (msg.data[2] << 8) | msg.data[3];
                break;
            case 0x203:
                move2_speed = (msg.data[2] << 8) | msg.data[3];
                break;
            case 0x204:
                move3_speed = (msg.data[2] << 8) | msg.data[3];
                // printf("picAngle_d = %d\n", picAngle_d);
                break;
            default:
                break;
            }
        }
    }
}

bool Square_flag = false;
bool Square_state = 0;

void CANSend() {
    while (1) {
        // バシバシ前後
        if (ps4.R2 == 1) {
            penguin.pwm[0] = -8000;
            penguin.pwm[1] = 8000;
        }
        if (ps4.R2 == 0 && ps4.L2 == 0) {
            penguin.pwm[0] = 0;
            penguin.pwm[1] = 0;
        }
        if (ps4.L2 == 1) {
            penguin.pwm[0] = -8000;
            penguin.pwm[1] = 8000;
        }
        // 床
        if (ps4.Square == 1) {
            if (Square_flag) {
                UnderUp = 4000;
            } else {
                UnderUp = -2000;
            }
        } else if (ps4.Square == 0) {
            UnderUp = 0;
        }
        if (ps4.Square == 1 && Square_state == 0) {
            Square_flag = !Square_flag;
        }
        Square_state = ps4.Square;
        // 高松
        if (ps4.R1 == 1) {
            Takamatsu = 10000;
        }
        if (ps4.R1 == 0 && ps4.L1 == 0) {
            Takamatsu = 0;
        }
        if (ps4.L1 == 1) {
            Takamatsu = -10000;
        }
        // 発射
        if (ps4.Triangle == 1) {
            penguin.pwm[2] = std::min(5000, penguin.pwm[2] + 300);
            penguin.pwm[3] = std::max(-5000, penguin.pwm[3] - 300);
        } else if (ps4.Triangle == 0) {
            penguin.pwm[2] = std::max(0, penguin.pwm[2] - 300);
            penguin.pwm[3] = std::min(0, penguin.pwm[3] + 300);
        }
        // バシバシ
        if (ps4.Circle == 1) {
            bashibashi = 5000;
        }
        if (ps4.Circle == 0 && ps4.Circle == 0) {
            bashibashi = 0;
        }
        if (ps4.Circle == 1) {
            bashibashi = -5000;
        }
        // サーボ
        else if (ps4.Cross == 1) {
            servovo = (servovo == 0) ? 150 : (servovo == 150 ? 90 : 0);
        }

        // 移動
        int move0 = ps4.LX * 0.6 - ps4.LY * 0.6 + ps4.RX * 0.8;
        int move1 = -ps4.LX * 0.6 - ps4.LY * 0.6 + ps4.RX * 0.8;
        int move2 = -ps4.LX * 0.6 + ps4.LY * 0.6 + ps4.RX * 0.8;
        int move3 = ps4.LY * 0.6 + ps4.LX * 0.6 + ps4.RX * 0.8;
        move0 = std::min(move0, maxspeed);
        move1 = std::min(move1, maxspeed);
        move2 = std::min(move2, maxspeed);
        move3 = std::min(move3, maxspeed);
        if (move0 > maxspeed)
            move0 = maxspeed;
        if (move1 > maxspeed)
            move1 = maxspeed;
        if (move2 > maxspeed)
            move2 = maxspeed;
        if (move3 > maxspeed)
            move3 = maxspeed;
        int move0_pid = p_move0.calculate(move0, move0_speed);
        int move1_pid = p_move1.calculate(move1, move1_speed);
        int move2_pid = p_move2.calculate(move2, move2_speed);
        int move3_pid = p_move3.calculate(move3, move3_speed);

        printf("PID Output: move0_pid = %d, move1_pid = %d, move2_pid "
               "= %d, "
               "move3_pid = %d\n",
               move0_pid, move1_pid, move2_pid, move3_pid);

        // 足回り
        int16_t move0416 = static_cast<int16_t>(move0_pid);
        int16_t move1416 = static_cast<int16_t>(move1_pid);
        int16_t move2416 = static_cast<int16_t>(move2_pid);
        int16_t move3416 = static_cast<int16_t>(move3_pid);
        DATA_move[0] = move0416 >> 8;   // MSB
        DATA_move[1] = move0416 & 0xFF; // LSB

        DATA_move[2] = move1416 >> 8;   // MSB
        DATA_move[3] = move1416 & 0xFF; // LSB

        DATA_move[4] = move2416 >> 8;   // MSB
        DATA_move[5] = move2416 & 0xFF; // LSB

        DATA_move[6] = move3416 >> 8;   // MSB
        DATA_move[7] = move3416 & 0xFF; // LSB
        // 機構
        int16_t Takamatsu416 = static_cast<int16_t>(Takamatsu);
        int16_t UnderUp416 = static_cast<int16_t>(UnderUp);
        int16_t bashibashiInt16 = static_cast<int16_t>(bashibashi);
        DATA[0] = Takamatsu416 >> 8;      // MSB
        DATA[1] = Takamatsu416 & 0xFF;    // LSB
        DATA[2] = bashibashiInt16 >> 8;   // MSB
        DATA[3] = bashibashiInt16 & 0xFF; // LSB

        DATA[4] = 0 >> 8;            // MSB
        DATA[5] = 0 & 0xFF;          // LSB
        DATA[6] = UnderUp416 >> 8;   // MSB
        DATA[7] = UnderUp416 & 0xFF; // LSB

        servo[7] = servovo;

        CANMessage msg_move(0x200, DATA_move, 8);
        CANMessage msg0(0x1ff, DATA, 8);
        CANMessage servo_msg(140, servo, 8);

        if (can2.write(servo_msg)) {
            // printf("[servo]:can");
        } else {
            printf("[servo]:can not");
        }
        if (can1.write(msg_move)) {
            // printf("[move]:can");
        } else {
            printf("[move]:can");
        }
        if (can1.write(msg0)) {
            // printf("[msg0]:can");
        } else {
            printf("[msg0]:can not");
        }
        if (penguin.send()) {
            // printf("[FP]:can\n");
        } else {
            printf("[FP]:can not \n");
        }

        ThisThread::sleep_for(10ms);
    }
}

int main() {
    printf("start\n");

    pc.set_baud(115200);
    pc.set_blocking(false);

    servo[7] = servovo;
    CANMessage servo_msg(140, servo, 8);
    can2.write(servo_msg);

    Thread thread1; // コントローラーの状態を受け取る
    thread1.start(serial_read);
    Thread thread2; // 足回りのセンサ情報を受け取る
    thread2.start(CANReceive);
    Thread thread3; // 機械制御とmsgを送る
    thread3.start(CANSend);

    while (1) {
        ThisThread::sleep_for(10ms);
    }
}
