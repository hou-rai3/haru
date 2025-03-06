#include "PID.hpp"
#include "QEI.h"
#include "firstpenguin.hpp"
#include "mbed.h"
#include "serial_read.hpp"
#include <array>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#define PPR 512  // エンコーダの1回転あたりのパルス数
Ticker rpm_ticker;
volatile int rpm = 0;

BufferedSerial uart(PB_6, PA_10, 9600);
PwmOut MINIMA(D5); // D6
CAN can1(PA_11, PA_12, 1e6);
CAN can2(PB_12, PB_13, 1e6);
FirstPenguin penguin(35, can1);
BufferedSerial pc(USBTX, USBRX, 115200);
PID pid_controller(0.4, 0.5, 0.001, 0.02);
PID p_move0(0.5, 0.0, 0.01, 0.02);
PID p_move1(0.5, 0.0, 0.01, 0.02);
PID p_move2(0.5, 0.0, 0.01, 0.02);
PID p_move3(0.5, 0.0, 0.01, 0.02);
PID ball(0.5, 0.0, 0.01, 0.02);
uint8_t servo[8] = {};
uint8_t DATA_move[8] = {};
uint8_t DATA[8] = {};
int16_t move0_speed = 0, move1_speed = 0, move2_speed = 0, move3_speed = 0,
        bashibashi_speed = 0, Takamatsu_speed = 0, UnderUp_speed = 0;

int Takamatsu = 0, UnderUp = 0, bashibashi = 0;
int servovo = 130;
int Kodai = 70;
int maxspeed = 10000;
int Kodaiho = 1000;
auto pre_PC_3 = HighResClock::time_point();
auto pre_PC_2 = HighResClock::time_point();
auto pre_Kodaiho = HighResClock::time_point();
auto pre_servo = HighResClock::time_point();
auto pre_RYUGU = HighResClock::time_point();

QEI encoder(PC_6, PC_7, NC, PPR, QEI::X4_ENCODING);  // A相, B相, インデックス, PPR, エンコーディングモード

DigitalOut led(LED1);

DigitalIn Userbutton(BUTTON1);

DigitalIn SW_0(PC_0); // マンジロード上
DigitalIn SW_1(PC_1); // マンジロード下
DigitalIn SW_2(PC_2); // 前後右
DigitalIn SW_3(PC_3); // 前後左
DigitalIn SW_4(PC_4); // 高松上
DigitalIn SW_5(PC_5); // 高松下
bool PC_0_flag = false;
bool PC_1_flag = false;
bool PC_2_flag = false;
bool PC_3_flag = false;
bool PC_4_flag = false;
bool PC_5_flag = false;

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

void calculate_rpm() {
    static int last_pulses = 0;
    int current_pulses = encoder.getPulses();
    int delta_pulses = current_pulses - last_pulses;
    last_pulses = current_pulses;

    // 0.01秒間に取得したパルス数からRPMを計算
    rpm = (delta_pulses * 6000) / (PPR * 4);  // 4逓倍の場合
}

void controller_read(const std::string buffer) {
    if (buffer == "ci:p")
        ps4.Circle = 1;
    else if (buffer == "ci:no_p")
        ps4.Circle = 0;
    if (buffer == "cr:p")
        ps4.Cross = 1;
    else if (buffer == "cr:no_p")
        ps4.Cross = 0;
    if (buffer == "sq:p")
        ps4.Square = 1;
    else if (buffer == "sq:no_p")
        ps4.Square = 0;
    if (buffer == "tri:p")
        ps4.Triangle = 1;
    else if (buffer == "tri:no_p")
        ps4.Triangle = 0;
    if (buffer == "L1:p")
        ps4.L1 = 1;
    else if (buffer == "L1:no_p")
        ps4.L1 = 0;
    if (buffer == "R1:p")
        ps4.R1 = 1;
    else if (buffer == "R1:no_p")
        ps4.R1 = 0;
    if (buffer == "L2:p")
        ps4.L2 = 1;
    else if (buffer == "L2:no_p")
        ps4.L2 = 0;
    if (buffer == "R2:p")
        ps4.R2 = 1;
    else if (buffer == "R2:no_p")
        ps4.R2 = 0;
    if (buffer == "SH:p")
        ps4.SHARE = 1;
    else if (buffer == "SH:no_p")
        ps4.SHARE = 0;
    if (buffer == "OP:p")
        ps4.OPTION = 1;
    else if (buffer == "OP:no_p")
        ps4.OPTION = 0;
    if (buffer == "PS:p")
        ps4.PS = 1;
    else if (buffer == "PS:no_p")
        ps4.PS = 0;
    if (buffer == "u:p")
        ps4.Up = 1;
    else if (buffer == "u:no_p")
        ps4.Up = 0;
    if (buffer == "d:p")
        ps4.Down = 1;
    else if (buffer == "d:no_p")
        ps4.Down = 0;
    if (buffer == "l:p")
        ps4.Left = 1;
    else if (buffer == "l:no_p")
        ps4.Left = 0;
    if (buffer == "r:p")
        ps4.Right = 1;
    else if (buffer == "r:no_p")
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
            case 0x205:
                Takamatsu_speed = (msg.data[2] << 8) | msg.data[3];
                break;
            case 0x206:
                move0_speed = (msg.data[2] << 8) | msg.data[3];
                break;
            case 0x207:
                UnderUp_speed = (msg.data[2] << 8) | msg.data[3];
                break;
            case 0x208:
                bashibashi_speed = (msg.data[2] << 8) | msg.data[3];
                break;
            default:
                break;
            }
        }
    }
}
bool toggle_flag = false;
bool angle_flag = false;

void updateTarget(int &move, int &move_mokuhyou) {
    int increment = (move_mokuhyou < 2000)
                        ? 250
                        : 1000; // Choose increment based on target value

    if (move > move_mokuhyou) {
        move_mokuhyou += increment;
    } else if (move < move_mokuhyou) {
        move_mokuhyou -= increment;
    }
}
void CANSend() {
    // int move0_mokuhyou = 0;
    // int move1_mokuhyou = 0;
    // int move2_mokuhyou = 0;
    // int move3_mokuhyou = 0;

    while (1) {
        PC_0_flag = SW_0.read();
        PC_1_flag = SW_1.read();
        PC_2_flag = SW_2.read();
        PC_3_flag = SW_3.read();
        PC_4_flag = SW_4.read();
        PC_5_flag = SW_5.read();

        if (ps4.SHARE == 1) {
            auto now_RYUGU = HighResClock::now();
            if (now_RYUGU - pre_RYUGU > 300ms) {
                uart.write("ryugu", 5);
                pre_RYUGU = now_RYUGU;
            }
        }
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
        if (PC_2_flag == 0) {
            auto now_PC_2 = HighResClock::now();
            if (pre_PC_2 == HighResClock::time_point()) {
                pre_PC_2 = now_PC_2;
            }
            if (now_PC_2 - pre_PC_2 <= 1000ms) {
                penguin.pwm[0] = 0;
            }
        } else {
            pre_PC_2 = HighResClock::time_point();
        }

        if (PC_3_flag == 0) {
            auto now_PC_3 = HighResClock::now();
            if (pre_PC_3 == HighResClock::time_point()) {
                pre_PC_3 = now_PC_3;
            }
            if (now_PC_3 - pre_PC_3 <= 1000ms) {
                penguin.pwm[1] = 0;
            }
        } else {
            pre_PC_3 = HighResClock::time_point();
        }
        // マンジロード
        if (ps4.R2 == 1 && PC_0_flag == 0) {
            int16_t UnderUp416 = static_cast<int16_t>(0);
            DATA[6] = UnderUp416 >> 8;
            DATA[7] = UnderUp416 & 0xFF;
        } else if (ps4.R2 == 1) {
            int16_t UnderUp416 = static_cast<int16_t>(1500);
            DATA[6] = UnderUp416 >> 8;
            DATA[7] = UnderUp416 & 0xFF;
        }
        if (ps4.L2 == 1 && PC_1_flag == 0) {
            int16_t UnderUp416 = static_cast<int16_t>(0);
            DATA[6] = UnderUp416 >> 8;
            DATA[7] = UnderUp416 & 0xFF;
        } else if (ps4.L2 == 1) {
            int16_t UnderUp416 = static_cast<int16_t>(1500);
            DATA[6] = -UnderUp416 >> 8;
            DATA[7] = -UnderUp416 & 0xFF;
        }

        if (ps4.R2 == 0 && ps4.L2 == 0) {
            int16_t UnderUp416 = static_cast<int16_t>(0);
            DATA[6] = UnderUp416 >> 8;
            DATA[7] = UnderUp416 & 0xFF;
        }

        // 高松
        if (ps4.R1 == 1 && PC_3_flag == 0) {
            int Takamatsu_target = 3000;
            int Takamatsu_pw =
                pid_controller.calculate(Takamatsu_target, Takamatsu_speed);
            int16_t Takamatsu416 = static_cast<int16_t>(Takamatsu_pw);
            DATA[0] = -Takamatsu416 >> 8;
            DATA[1] = -Takamatsu416 & 0xFF;
        } else if (ps4.R1 == 1) {
            int Takamatsu_target = 3400;
            int Takamatsu_pw =
                pid_controller.calculate(Takamatsu_target, Takamatsu_speed);
            int16_t Takamatsu416 = static_cast<int16_t>(Takamatsu_pw);
            DATA[0] = -Takamatsu416 >> 8;
            DATA[1] = -Takamatsu416 & 0xFF;
        }
        if (ps4.L1 == 1 && PC_4_flag == 0) {
            int Takamatsu_target = 0;
            int Takamatsu_pw =
                pid_controller.calculate(Takamatsu_target, Takamatsu_speed);
            int16_t Takamatsu416 = static_cast<int16_t>(Takamatsu_pw);
            DATA[0] = -Takamatsu416 >> 8;
            DATA[1] = -Takamatsu416 & 0xFF;
        } else if (ps4.L1 == 1) {
            int Takamatsu_target = 1500;
            int Takamatsu_pw =
                pid_controller.calculate(Takamatsu_target, Takamatsu_speed);
            int16_t Takamatsu416 = static_cast<int16_t>(Takamatsu_pw);
            DATA[0] = -Takamatsu416 >> 8;
            DATA[1] = -Takamatsu416 & 0xFF;
        }
        if (ps4.R1 == 0 && ps4.L1 == 0) {
            int Takamatsu_target = 0;
            int Takamatsu_pw =
                pid_controller.calculate(Takamatsu_target, Takamatsu_speed);
            int16_t Takamatsu416 = static_cast<int16_t>(Takamatsu_pw);
            DATA[0] = -Takamatsu416 >> 8;
            DATA[1] = -Takamatsu416 & 0xFF;
        }
        // 発射
        if (ps4.Triangle == 1) {
            penguin.pwm[2] = ball.calculate(2000, rpm);
        } else if (ps4.Triangle == 0) {
            penguin.pwm[2] = 0;
        }
        //  バシバシ
        if (ps4.Circle == 1) {
            int bashibashi_target = 6000;
            int bashibashi_pw =
                pid_controller.calculate(bashibashi_target, bashibashi_speed);
            int16_t bashibashiInt16 = static_cast<int16_t>(bashibashi_pw);
            DATA[2] = bashibashiInt16 >> 8;
            DATA[3] = bashibashiInt16 & 0xFF;
        } else if (ps4.Circle == 0 && ps4.Down == 0) {
            int bashibashi_target = 0;
            int bashibashi_pw =
                pid_controller.calculate(bashibashi_target, bashibashi_speed);
            int16_t bashibashiInt16 = static_cast<int16_t>(bashibashi_pw);
            DATA[2] = bashibashiInt16 >> 8;
            DATA[3] = bashibashiInt16 & 0xFF;
        }
        if (ps4.Down == 1) {
            int16_t bashibashiInt16 = static_cast<int16_t>(3000);
            DATA[2] = -bashibashiInt16 >> 8;
            DATA[3] = -bashibashiInt16 & 0xFF;
        }

        // サーボ
        if (ps4.Cross == 1) {
            auto now_servo = HighResClock::now();
            if (now_servo - pre_servo > 200ms) {
                toggle_flag = !toggle_flag;
                servovo = toggle_flag ? 150 : 0;
                pre_servo = now_servo;
            }
        }
        if (ps4.Square == 1) {
            auto now_servo = HighResClock::now();
            if (now_servo - pre_servo > 200ms) {
                toggle_flag = !toggle_flag;
                Kodai = toggle_flag ? 150 : 0;
                pre_servo = now_servo;
            }
        }

        if (ps4.OPTION == 1) {
            Kodaiho = std::min(1650, Kodaiho + 6);
            MINIMA.pulsewidth_us(Kodaiho);
        } else if (ps4.OPTION == 0) {
            Kodaiho = std::max(1000, Kodaiho - 6);
            MINIMA.pulsewidth_us(Kodaiho);
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

        // updateTarget(move0, move0_mokuhyou);
        // updateTarget(move1, move1_mokuhyou);
        // updateTarget(move2, move2_mokuhyou);
        // updateTarget(move3, move3_mokuhyou);
        int move0_pid = p_move0.calculate(move0, move0_speed);
        int move1_pid = p_move1.calculate(move1, move1_speed);
        int move2_pid = p_move2.calculate(move2, move2_speed);
        int move3_pid = p_move3.calculate(move3, move3_speed);

        printf("PID Output: move0_pid = %d, move1_pid = %d, move2_pid "
               "= %d, "
               "move3_pid = %d\n",
               move0_pid, move1_pid, move2_pid, move3_pid);

        //  足回り
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

        DATA[4] = 0 >> 8;
        DATA[5] = 0 & 0xFF;

        servo[0] = 0;
        servo[1] = Kodai;
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
    rpm_ticker.attach(&calculate_rpm, 0.01);
    printf("[controller_tester]setup!!\n");
    std::vector<double> joy_nums;
    pc.set_baud(115200);
    pc.set_blocking(false);
    servo[1] = Kodai;
    servo[7] = servovo;
    CANMessage servo_msg(140, servo, 8);
    can2.write(servo_msg);

    serial_unit serial(pc);
    Thread thread1;
    Thread thread2;
    thread1.start(CANReceive);
    thread2.start(CANSend);

    SW_0.mode(PullUp);
    SW_1.mode(PullUp);
    SW_2.mode(PullUp);
    SW_3.mode(PullUp);
    SW_4.mode(PullUp);
    SW_5.mode(PullUp);

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
