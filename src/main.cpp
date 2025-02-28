#include "PID.hpp"
#include "firstpenguin.hpp"
#include "mbed.h"
#include "serial_read.hpp"
#include <array>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

PwmOut MINIMA(D5); // D6
CAN can1(PA_11, PA_12, 1e6);
CAN can2(PB_12, PB_13, 1e6);
FirstPenguin penguin(35, can1);
BufferedSerial pc(USBTX, USBRX, 250000);
PID pid_controller(0.4, 0.5, 0.001, 0.02);
PID p_move0(0.5, 0.0, 0.01, 0.02);
PID p_move1(0.5, 0.0, 0.01, 0.02);
PID p_move2(0.5, 0.0, 0.01, 0.02);
PID p_move3(0.5, 0.0, 0.01, 0.02);
uint8_t servo[8] = {};
uint8_t DATA_move[8] = {};
uint8_t DATA[8] = {};
int16_t move0_speed = 0, move1_speed = 0, move2_speed = 0, move3_speed = 0;
int Takamatsu = 0, UnderUp = 0, bashibashi = 0;
int servovo = 130;
int Kodai = 70;
int maxspeed = 10000;
int Kodaiho = 1000;
auto pre_PC_1 = HighResClock::time_point();
auto pre_PC_2 = HighResClock::time_point();

DigitalIn Userbutton(BUTTON1);
DigitalIn SW_10(PC_10);
bool PC_10_flag = false;
DigitalIn SW_1(PC_1);
bool PC_1_flag = false;
DigitalIn SW_2(PC_2);
bool PC_2_flag = false;

DigitalIn SWPH_0(PH_0);
bool PH_0_flag = false;
DigitalIn SWPH_1(PH_1);
bool PH_1_flag = false;

typedef struct {
    double LX;
    double LY;
    double RX;
    double RY;

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

std::vector<double> to_numbers(const std::string &input) {
    std::vector<double> numbers;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, ':')) { // ':'で区切る
        if (token.back() == '|') {         // 最後の '|' を削除
            token.pop_back();
        }
        numbers.push_back(std::stod(token)); // 文字列をdoubleに変換
    }
    return numbers;
}

void controller_read(const std::string buffer) {
    if (buffer == "circle:pressing")
        ps4.Circle = 1;
    else if (buffer == "circle:no_pressing")
        ps4.Circle = 0;
    if (buffer == "cross:pressing")
        ps4.Cross = 1;
    else if (buffer == "cross:no_pressing")
        ps4.Cross = 0;
    if (buffer == "square:pressing")
        ps4.Square = 1;
    else if (buffer == "square:no_pressing")
        ps4.Square = 0;
    if (buffer == "triangle:pressing")
        ps4.Triangle = 1;
    else if (buffer == "triangle:no_pressing")
        ps4.Triangle = 0;
    if (buffer == "L1:pressing")
        ps4.L1 = 1;
    else if (buffer == "L1:no_pressing")
        ps4.L1 = 0;
    if (buffer == "R1:pressing")
        ps4.R1 = 1;
    else if (buffer == "R1:no_pressing")
        ps4.R1 = 0;
    if (buffer == "L2:pressing")
        ps4.L2 = 1;
    else if (buffer == "L2:no_pressing")
        ps4.L2 = 0;
    if (buffer == "R2:pressing")
        ps4.R2 = 1;
    else if (buffer == "R2:no_pressing")
        ps4.R2 = 0;
    if (buffer == "SHARE:pressing")
        ps4.SHARE = 1;
    else if (buffer == "SHARE:no_pressing")
        ps4.SHARE = 0;
    if (buffer == "OPTION:pressing")
        ps4.OPTION = 1;
    else if (buffer == "OPTION:no_pressing")
        ps4.OPTION = 0;
    if (buffer == "PS:pressing")
        ps4.PS = 1;
    else if (buffer == "PS:no_pressing")
        ps4.PS = 0;
    if (buffer == "up:pressing")
        ps4.Up = 1;
    else if (buffer == "up:no_pressing")
        ps4.Up = 0;
    if (buffer == "dowm:pressing")
        ps4.Down = 1;
    else if (buffer == "dowm:no_pressing")
        ps4.Down = 0;
    if (buffer == "left:pressing")
        ps4.Left = 1;
    else if (buffer == "left:no_pressing")
        ps4.Left = 0;
    if (buffer == "right:pressing")
        ps4.Right = 1;
    else if (buffer == "right:no_pressing")
        ps4.Right = 0;
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
bool toggle_flag = false;
auto pre_servo = HighResClock::time_point();

bool angle_flag = false;
auto pre_Kodaiho = HighResClock::time_point();
void CANSend() {
    while (1) {
        PC_10_flag = SW_10.read();
        PC_1_flag = SW_1.read();
        PC_2_flag = SW_2.read();
        PH_0_flag = SWPH_0.read();
        PH_1_flag = SWPH_1.read();

        // バシバシ前後
        if (ps4.Right == 1) {
            penguin.pwm[0] = -15000;
            penguin.pwm[1] = 15000;
        }
        if (ps4.Left == 1) {
            penguin.pwm[0] = 15000;
            penguin.pwm[1] = -15000;
        }
        if (ps4.Left == 0 && ps4.Right == 0) {
            penguin.pwm[0] = 0;
            penguin.pwm[1] = 0;
        }
        if (PC_1_flag == 0) {
            auto now_PC_1 = HighResClock::now();
            if (pre_PC_1 == HighResClock::time_point()) {
                pre_PC_1 = now_PC_1;
            }
            if (now_PC_1 - pre_PC_1 <= 1000ms) {
                penguin.pwm[0] = 0;
            }
        } else {
            pre_PC_1 = HighResClock::time_point();
        }

        if (PC_2_flag == 0) {
            auto now_PC_2 = HighResClock::now();
            if (pre_PC_2 == HighResClock::time_point()) {
                pre_PC_2 = now_PC_2;
            }
            if (now_PC_2 - pre_PC_2 <= 1000ms) {
                penguin.pwm[1] = 0;
            }
        } else {
            pre_PC_2 = HighResClock::time_point();
        }
        // 床
        if (ps4.R2 == 1) {
            if (PH_0_flag == 0) {
                int16_t UnderUp416 = static_cast<int16_t>(0);
                DATA[6] = UnderUp416 >> 8;
                DATA[7] = UnderUp416 & 0xFF;
            } else if (PH_0_flag == 1) {
                int16_t UnderUp416 = static_cast<int16_t>(1500);
                DATA[6] = UnderUp416 >> 8;
                DATA[7] = UnderUp416 & 0xFF;
            }
        }
        if (ps4.L2 == 1) {
            if (PH_1_flag == 0) {
                int16_t UnderUp416 = static_cast<int16_t>(0);
                DATA[6] = UnderUp416 >> 8;
                DATA[7] = UnderUp416 & 0xFF;
            } else if (PH_1_flag == 1) {
                int16_t UnderUp416 = static_cast<int16_t>(1500);
                DATA[6] = -UnderUp416 >> 8;
                DATA[7] = -UnderUp416 & 0xFF;
            }
        }
        if (ps4.R2 == 0 && ps4.L2 == 0) {
            int16_t UnderUp416 = static_cast<int16_t>(0);
            DATA[6] = UnderUp416 >> 8;
            DATA[7] = UnderUp416 & 0xFF;
        }

        // 高松
        if (ps4.R1 == 1 && PC_10_flag == 1) {
            int16_t Takamatsu416 = static_cast<int16_t>(3500);
            DATA[0] = Takamatsu416 >> 8;
            DATA[1] = Takamatsu416 & 0xFF;
        } else if (ps4.R1 == 1 && PC_10_flag == 0) {
            int16_t Takamatsu416 = static_cast<int16_t>(1700);
            DATA[0] = Takamatsu416 >> 8;
            DATA[1] = Takamatsu416 & 0xFF;
        } else if (ps4.R1 == 0 && ps4.L1 == 0) {
            int16_t Takamatsu416 = static_cast<int16_t>(0);
            DATA[0] = Takamatsu416 >> 8;
            DATA[1] = Takamatsu416 & 0xFF;
        } else if (ps4.L1 == 1) {
            int16_t Takamatsu416 = static_cast<int16_t>(1500);
            DATA[0] = -Takamatsu416 >> 8;
            DATA[1] = -Takamatsu416 & 0xFF;
        }
        // 発射
        if (ps4.Triangle == 1) {
            penguin.pwm[2] = std::min(3700, penguin.pwm[2] + 600);
            penguin.pwm[3] = std::max(-3700, penguin.pwm[3] - 600);
        } else if (ps4.Triangle == 0) {
            penguin.pwm[2] = std::max(0, penguin.pwm[2] - 600);
            penguin.pwm[3] = std::min(0, penguin.pwm[3] + 600);
        }
        // バシバシ
        // if (ps4.Circle == 1) {
        //     bashibashi = 7000;
        // } else if (ps4.Circle == 0) {
        //     bashibashi = 0;
        // }

        // サーボ
        if (ps4.Cross == 1) {
            Kodai += 1;
        }
        if (ps4.Square == 1) {
            Kodai -= 1;
        }

        if (ps4.Circle == 1) {
            Kodaiho = std::min(1600, Kodaiho + 6);
            MINIMA.pulsewidth_us(Kodaiho);
        } else if (ps4.Circle == 0) {
            Kodaiho = std::max(1000, Kodaiho - 6);
            MINIMA.pulsewidth_us(Kodaiho);
        }

        auto now_Kodaiho = HighResClock::now();
        if (pre_Kodaiho == HighResClock::time_point()) {
            pre_Kodaiho = now_Kodaiho;
        }
        if (now_Kodaiho - pre_Kodaiho <= 100ms) {
            return;
        }
        if (ps4.SHARE == 1) {
            angle_flag = !angle_flag;
            Kodai += angle_flag ? 1 : -1;
            pre_Kodaiho = now_Kodaiho;
        }
        // 移動
        int move0 = (ps4.LX - ps4.LY - ps4.RX * 0.8) * 10000;
        int move1 = -(ps4.LX - ps4.LY + ps4.RX * 0.8) * 10000;
        int move2 = -(ps4.LX + ps4.LY + ps4.RX * 0.8) * 10000;
        int move3 = (ps4.LY + ps4.LX - ps4.RX * 0.8) * 10000;
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

        // printf("PID Output: move0_pid = %d, move1_pid = %d, move2_pid "
        //        "= %d, "
        //        "move3_pid = %d\n",
        //        move0_pid, move1_pid, move2_pid, move3_pid);

        // 足回り
        int16_t move0416 = static_cast<int16_t>(move0_pid);
        int16_t move1416 = static_cast<int16_t>(move1_pid);
        int16_t move2416 = static_cast<int16_t>(move2_pid);
        int16_t move3416 = static_cast<int16_t>(move3_pid);
        DATA_move[0] = move0416 >> 8;
        DATA_move[1] = move0416 & 0xFF;

        DATA_move[2] = move1416 >> 8;
        DATA_move[3] = move1416 & 0xFF;

        DATA_move[4] = move2416 >> 8;
        DATA_move[5] = move2416 & 0xFF;

        DATA_move[6] = move3416 >> 8;
        DATA_move[7] = move3416 & 0xFF;

        // 機構
        int16_t bashibashiInt16 = static_cast<int16_t>(bashibashi);

        DATA[2] = bashibashiInt16 >> 8;
        DATA[3] = bashibashiInt16 & 0xFF;
        DATA[4] = 0 >> 8;
        DATA[5] = 0 & 0xFF;

        servo[0] = Kodai;
        servo[1] = 0;
        servo[2] = 0;
        servo[3] = 0;
        servo[4] = 0;
        servo[5] = 0;
        servo[6] = 0;
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
    std::vector<double> joy_nums;
    pc.set_baud(115200);
    pc.set_blocking(false);
    servo[0] = Kodai;
    servo[7] = servovo;
    CANMessage servo_msg(140, servo, 8);
    can2.write(servo_msg);
    serial_unit serial(pc);
    Thread thread2;
    Thread thread3;
    thread2.start(CANReceive);
    thread3.start(CANSend);

    printf("[controller_tester]setup!!\n");

    SW_10.mode(PullUp);
    SW_1.mode(PullUp);
    SW_2.mode(PullUp);
    SWPH_0.mode(PullUp);
    SWPH_1.mode(PullUp);

    while (true) {
        std::string msg = serial.read_serial();
        if (msg != "") {
            if (msg[0] == 'n') {
                msg.erase(0, 2);
                joy_nums = to_numbers(msg);
                ps4.LX = joy_nums[0];
                ps4.LY = joy_nums[1];
                ps4.RX = joy_nums[2];
                ps4.RY = joy_nums[3];
            } else {
                controller_read(msg);
            }
        }
    }
}
