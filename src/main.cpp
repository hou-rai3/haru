#include "FP.hpp"
#include "QEI.hpp"
#include "c610.hpp"
#include "mbed.h"
#include "pid.hpp"
#include "serial_read.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>
//------------powers----------
#define TAKAMATU_GOAL_RPM 4000
#define UNDER_GOAL_RPM 10000
#define BASI2_ZEN_GOAL_RPM 13000
#define BASI2_GO_GOAL_RPM 3500
#define CANNON_GOAL_RPM 800 // 発射
#define BASI2_ZENGO_POWER 10000
#define SERVOVO_MODE0 225
#define SERVOVO_MODE1 0
#define KODAI_MODE0 65
#define KODAI_MODE1 30
//---------------buttons------------
bool Circle = false;
bool Cross = false;
bool Square = false;
bool Triangle = false;
bool Up = false;
bool Right = false;
bool Down = false;
bool Left = false;
bool L2 = false;
bool R2 = false;
bool L1 = false;
bool R1 = false;
bool SHARE = false;
bool OPTION = false;
bool PS = false;
bool L3 = false;
bool R3 = false;
//---------------------------

#define PPR 512 // エンコーダの1回転あたりのパルス数

PwmOut MINIMA(D5); // D6
CAN can1(PA_11, PA_12, 1e6);
CAN can2(PB_12, PB_13, 1e6);

BufferedSerial pc(USBTX, USBRX, 115200);
BufferedSerial arudino(PB_6, PA_10, 9600);
serial_unit serial(pc);

C610 c610(can1);
FP fp(35, can2);
uint8_t servo[8] = {};

double zoukaryou = 0.003;

double move_pid_Tilt_p = 1.0;
double move_pid_Tilt_i = 1.0; // 使ってない
double move_pid_Tilt_d = 1.0; // 使ってない

// PID move_pid[4] = {PID(1.00, 0.015, 30.0), PID(1.0, 0.015, 30.0), PID(1.0,
// 0.015, 30.0), PID(1.0, 0.015, 30.0)};
PID move_pid[4] = {PID(1.2 * move_pid_Tilt_p, 0.015, 30.0),
                   PID(1.5 * move_pid_Tilt_p, 0.015, 30.0),
                   PID(1.65 * move_pid_Tilt_p, 0.015, 30.0),
                   PID(0.8 * move_pid_Tilt_p, 0.015, 30.0)};
PID underup_pid(1.0, 0.0, 30.0);
PID takamatu_pid(1.8, 0.0, 5.0);
PID bashi2_rpm_pid(1.5, 0.0, 0.5);
PID cannon_pid(4.0, 0.1, 0.4);

double move_Tilt = 6000.0;

auto pre_PC_1 = HighResClock::time_point();
auto pre_PC_2 = HighResClock::time_point();

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

DigitalIn Userbutton(BUTTON1);

QEI encoder(
    PC_6, PC_7,
    QEI::X4_ENCODING); // A相, B相, インデックス, PPR, エンコーディングモード
Ticker rpm_ticker;
volatile int cannon_rpm = 0;

bool manzi_flag = false;
bool manzi_state = false;

bool ryugu_flag = false;
bool servovo_flag = false;
bool kodai_flag = false;
bool takamatu_flag = false;

bool c610_no_pid_8 = false;
int cannon_power = 0;
int bashi_power = 0;
double Kodaiho = 1000.0;

int servovo = 0;
int Kodai = 70;

double increment_value = 0.0;

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
    cannon_rpm = (delta_pulses * 6000) / (PPR * 4); // 4逓倍の場合
}

void move(std::string msg) {
    msg.erase(0, 2);
    std::vector<double> joys = to_numbers(msg);
    double move[4] = {(joys[0] - joys[1] - joys[2] * 0.8) * move_Tilt,
                      -(joys[0] - joys[1] + joys[2] * 0.8) * move_Tilt,
                      -(joys[0] + joys[1] + joys[2] * 0.8) * move_Tilt,
                      (joys[1] + joys[0] - joys[2] * 0.8) * move_Tilt};
    for (size_t i = 0; i < joys.size(); i++) {
        move_pid[i].set_goal(move[i]);
    }
}

void key_puress(std::string &msg) {
    if (msg == "ci:p")
        Circle = true;
    else if (msg == "ci:no_p")
        Circle = false;

    if (msg == "cr:p")
        Cross = true;
    else if (msg == "cr:no_p")
        Cross = false;

    if (msg == "sq:p")
        Square = true;
    else if (msg == "sq:no_p")
        Square = false;

    if (msg == "tri:p")
        Triangle = true;
    else if (msg == "tri:no_p")
        Triangle = false;

    if (msg == "L1:p")
        L1 = true;
    else if (msg == "L1:no_p")
        L1 = false;

    if (msg == "R1:p")
        R1 = true;
    else if (msg == "R1:no_p")
        R1 = false;

    if (msg == "L2:p")
        L2 = true;
    else if (msg == "L2:no_p")
        L2 = false;

    if (msg == "R2:p")
        R2 = true;
    else if (msg == "R2:no_p")
        R2 = false;

    if (msg == "SH:p")
        SHARE = true;
    else if (msg == "SH:no_p")
        SHARE = false;

    if (msg == "OP:p")
        OPTION = true;
    else if (msg == "OP:no_p")
        OPTION = false;

    if (msg == "PS:p")
        PS = true;
    else if (msg == "PS:no_p")
        PS = false;

    if (msg == "u:p")
        Up = true;
    else if (msg == "u:no_p")
        Up = false;

    if (msg == "d:p")
        Down = true;
    else if (msg == "d:no_p")
        Down = false;

    if (msg == "l:p")
        Left = true;
    else if (msg == "l:no_p")
        Left = false;

    if (msg == "r:p")
        Right = true;
    else if (msg == "r:no_p")
        Right = false;

    if (msg == "L3:p")
        L3 = true;
    else if (msg == "L3:no_p")
        L3 = false;

    if (msg == "R3:p")
        R3 = true;
    else if (msg == "R3:no_p")
        R3 = false;
}

void key_binding() {

    PC_0_flag = SW_0.read();
    PC_1_flag = SW_1.read();
    PC_2_flag = SW_2.read();
    PC_3_flag = SW_3.read();
    PC_4_flag = SW_4.read();
    PC_5_flag = SW_5.read();

    // if (!PC_0_flag)
    //     printf("PC_0:");
    // if (!PC_1_flag)
    //     printf("PC_1:");
    // if (!PC_2_flag)
    //     printf("PC_2:");
    // if (!PC_3_flag)
    //     printf("PC_3:");
    // if (!PC_4_flag)
    //     printf("PC_4:");
    // if (!PC_5_flag)
    //     printf("PC_5:");
    // printf("\n");

    // バシバシ前後
    if (Up && !R3) {
        fp.pwm[0] = -BASI2_ZENGO_POWER;
        fp.pwm[1] = BASI2_ZENGO_POWER;
    } else if (Down && !R3) {
        fp.pwm[0] = BASI2_ZENGO_POWER;
        fp.pwm[1] = -BASI2_ZENGO_POWER;
    } else if (!Up && !Down) {
        fp.pwm[0] = 0;
        fp.pwm[1] = 0;
    }
    if (PC_2_flag == 0) {
        auto now_PC_1 = HighResClock::now();
        if (pre_PC_1 == HighResClock::time_point()) {
            pre_PC_1 = now_PC_1;
        }
        if (now_PC_1 - pre_PC_1 <= 1000ms) {
            fp.pwm[0] = 0;
        }
    } else {
        pre_PC_1 = HighResClock::time_point();
    }

    if (PC_3_flag == 0) {
        auto now_PC_2 = HighResClock::now();
        if (pre_PC_2 == HighResClock::time_point()) {
            pre_PC_2 = now_PC_2;
        }
        if (now_PC_2 - pre_PC_2 <= 1000ms) {
            fp.pwm[1] = 0;
        }
    } else {
        pre_PC_2 = HighResClock::time_point();
    }
    // 卍ろーど
    if (L2) {
        c610_no_pid_8 = false;
        if (PC_1_flag) {
            underup_pid.set_goal(-UNDER_GOAL_RPM);
        } else if (!PC_1_flag) {
            underup_pid.set_goal(0);
        }
    } else if (R2) {
        c610_no_pid_8 = false;
        if (PC_0_flag) {
            underup_pid.set_goal(UNDER_GOAL_RPM);
        } else if (!PC_0_flag) {
            underup_pid.set_goal(0);
        }
    } else if (!R2 && !L2) {
        underup_pid.set_goal(0);
    }
    // 高松
    if (L1 && PC_5_flag) {
        takamatu_flag = false;
        takamatu_pid.set_goal(TAKAMATU_GOAL_RPM);
    } else if (L1 && !PC_5_flag) {
        c610_no_pid_8 = true;
        takamatu_pid.set_goal(0);
        c610.set_power(5, 0);
    } else if (!L1 && !R1) {
        takamatu_pid.set_goal(0);
    } else if (R1 && PC_4_flag) {
        takamatu_pid.set_goal(-TAKAMATU_GOAL_RPM);
    } else if (R1 && !PC_4_flag) {
        takamatu_flag = true;
    }
    if (takamatu_flag) {
        c610_no_pid_8 = true;
        takamatu_pid.set_goal(0);
        c610.set_power(5, -2400);
    }
    // 発射
    if (Triangle) {
        cannon_pid.set_goal(CANNON_GOAL_RPM);
        // cannon_power = cannon_pid.do_pid(c610.get_rpm(5));
    } else if (Triangle == false) {
        cannon_pid.set_goal(0);
        // cannon_power = cannon_pid.do_pid(c610.get_rpm(5));
    }
    // bashi2
    if (Right && !R3) {
        bashi2_rpm_pid.set_goal(-BASI2_GO_GOAL_RPM);
        bashi_power = bashi2_rpm_pid.do_pid(c610.get_rpm(6));
    } else if (Circle) {
        bashi2_rpm_pid.set_goal(BASI2_ZEN_GOAL_RPM);
        bashi_power = bashi2_rpm_pid.do_pid(c610.get_rpm(6));
    } else {
        bashi2_rpm_pid.set_goal(0);
        bashi_power = bashi2_rpm_pid.do_pid(c610.get_rpm(6));
    }

    // sa-bo
    if (Cross) {
        if (!servovo_flag) {
            // サーボ角度をトグル
            servovo =
                (servovo == SERVOVO_MODE0) ? SERVOVO_MODE1 : SERVOVO_MODE0;
            servovo_flag = true; // トグルしたのでフラグを設定
        }
    } else {
        servovo_flag = false; // ボタンが押されていない場合、フラグをリセット
    }
    // if (Cross) {
    //         fp.pwm[3] = SER_FP_POWER;
    // }
    // else if(Square)
    // {
    //     fp.pwm[3] = -SER_FP_POWER;
    // }
    // else{
    //     fp.pwm[3] = 0;
    // }
    // Koudai砲モータ
    if (OPTION) {
        Kodaiho = std::min(1600.0, Kodaiho + 0.7);
        MINIMA.pulsewidth_us(Kodaiho);
    } else if (!OPTION) {
        Kodaiho = std::max(1000.0, Kodaiho - 0.7);
        MINIMA.pulsewidth_us(Kodaiho);
    }
    // Koudai砲サーボ
    if (Square) {
        if (!kodai_flag) {
            Kodai = (Kodai == KODAI_MODE0) ? KODAI_MODE1 : KODAI_MODE0;
            kodai_flag = true;
        }
    } else {
        kodai_flag = false;
    }
    // 竜宮宣言
    if (!Left && ryugu_flag) {
        ryugu_flag = false;
    } else if (Left && !ryugu_flag) {
        arudino.write("ryugu", 5);
        ryugu_flag = true;
    }
}

void serial_read() {
    while (1) {

        std::string msg = serial.read_serial();
        if (msg != "") {
            if (msg[0] == 'n') {
                if (R3 && Right) {
                    msg = "n:-0.500000:0.000000:0.000000:0.000000|";
                    printf("Updated msg: %s\n",
                           msg.c_str()); // 増加した値を表示
                } else if (R3 && Left) {
                    msg = "n:0.500000:0.000000:0.000000:0.000000|";
                    printf("Updated msg: %s\n",
                           msg.c_str()); // 増加した値を表示
                } else if (R3 && Up) {
                    msg = "n:0.000000:0.500000:0.000000:0.000000|";
                    printf("Updated msg: %s\n",
                           msg.c_str()); // 増加した値を表示
                } else if (R3 && Down) {
                    msg = "n:0.000000:-0.500000:0.000000:0.000000|";
                    printf("Updated msg: %s\n",
                           msg.c_str()); // 増加した値を表示
                }
                // 前だけに進むやつ
                if (PS) {
                    increment_value += zoukaryou;
                    // msgの中の値を動的に変更
                    char buffer[100];
                    snprintf(buffer, sizeof(buffer),
                             "n:0.000000:%.6f:0.000000:0.000000|",
                             increment_value);
                    msg = buffer; // msgに新しい値を設定
                    printf("Updated msg: %s\n",
                           msg.c_str()); // 増加した値を表示
                }
                move(msg);
            } else {
                key_puress(msg);
            }
        }
        key_binding();
        // ThisThread::sleep_for(10ms);
    }
}

void PID_calculation() {
    auto pre_time = HighResClock::now();
    while (1) {
        auto now_time = HighResClock::now();
        c610.param_update();

        double dt = std::chrono::duration_cast<std::chrono::microseconds>(
                        now_time - pre_time)
                        .count() /
                    1000000.0;
        for (int i = 0; i < 4; i++)
            move_pid[i].set_dt(dt);
        takamatu_pid.set_dt(dt);
        underup_pid.set_dt(dt);
        cannon_pid.set_dt(dt);
        bashi2_rpm_pid.set_dt(dt);
        c610.set_power(1, move_pid[0].do_pid(c610.get_rpm(1)));
        c610.set_power(2, move_pid[1].do_pid(c610.get_rpm(2)));
        c610.set_power(3, move_pid[2].do_pid(c610.get_rpm(3)));
        c610.set_power(4, move_pid[3].do_pid(c610.get_rpm(4)));
        if (!c610_no_pid_8)
            c610.set_power(5, takamatu_pid.do_pid(c610.get_rpm(5)));
        c610.set_power(6, bashi_power);
        // c610.set_power(7, cannon_power);
        c610.set_power(8, underup_pid.do_pid(c610.get_rpm(8)));
        // printf("cannon_power: %d\n", c610.get_rpm(7));
        fp.pwm[3] = cannon_pid.do_pid(cannon_rpm);
        printf("RPM: %d:", cannon_rpm);

        pre_time = now_time;
        // ThisThread::sleep_for(10ms);
    }
}

int main() {
    Thread thread;
    thread.start(serial_read);
    Thread PID_thread;
    PID_thread.start(PID_calculation);
    std::vector<double> joys;
    pc.set_baud(115200);
    pc.set_blocking(false);
    servo[1] = Kodai;
    servo[7] = servovo;
    CANMessage servo_msg(140, servo, 8);
    can2.write(servo_msg);
    // rpm_ticker.attach(&calculate_rpm, 10ms);

    SW_0.mode(PullUp);
    SW_1.mode(PullUp);
    SW_2.mode(PullUp);
    SW_3.mode(PullUp);
    SW_4.mode(PullUp);
    SW_5.mode(PullUp);

    // bashi2_rpm_pid.set_goal(4000.0);
    printf("[robot]setup\n");
    while (1) {
        // printf("Cannon: %d\n", fp.pwm[2]);

        servo[1] = Kodai;
        servo[7] = servovo;
        CANMessage servo_msg(140, servo, 8);

        c610.send_message();
        fp.send();
        can2.write(servo_msg);
        c610_no_pid_8 = false;
        // ThisThread::sleep_for(10ms);
    }
}
