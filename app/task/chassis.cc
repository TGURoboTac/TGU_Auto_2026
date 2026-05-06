//
// Created by JJ on 2026/4/1.
//

#include "bsp/time.h"
#include "bsp/uart.h"
#include "controller/pid.h"
#include "ins/ins.h"
#include "motor/gyj.h"
#include "utils/os.h"
#include "utils/vofa.h"
#include "def.h"
#include "bsp/buzzer.h"
#include "rc/dr16.h"
// 控制红蓝方的开关
#define SIDE_RED

typedef enum {
    E_TURN_LEFT,
    E_TURN_RIGHT,
    E_TURN_NONE
} dir_t;
// 每个路口de任务
dir_t route[] = {
    E_TURN_NONE,    // 0（起点）
    E_TURN_LEFT,    // 1 进入主路
    E_TURN_RIGHT,   // 2 路口一进
    E_TURN_LEFT,    // 3 路口一出
    E_TURN_NONE,    // 4 路口二跳过
    E_TURN_NONE,    // 5 路口三跳过
    E_TURN_LEFT,    // 6 路口四进
    E_TURN_LEFT,    // 7 路口四出->反
    E_TURN_LEFT,    // 8 路口三进
    E_TURN_RIGHT,   // 9 路口三出
    E_TURN_LEFT,    // 10 路口二进
    E_TURN_LEFT     // 11 路口二出 ->正
};
mode_t mode = E_MODE_TRAIL_F;

motor::gyj m0("m0", {.id = 0x00,.port = E_CAN_1,.mode = motor::gyj::SPEED_LOOP,.have_feedback = true}, 1);
motor::gyj m1("m1", {.id = 0x01,.port = E_CAN_1,.mode = motor::gyj::SPEED_LOOP,.have_feedback = false}, 1);
motor::gyj m2("m2", {.id = 0x02,.port = E_CAN_1,.mode = motor::gyj::SPEED_LOOP,.have_feedback = false}, 1);
motor::gyj m3("m3", {.id = 0x03,.port = E_CAN_1,.mode = motor::gyj::SPEED_LOOP,.have_feedback = false}, 1);

controller::pid angle_pid(20, 1, 0, 5, 20);    // 转弯角速度环
controller::pid trail_pid(2.5, 0, 0.1, 0, 8);  // 循迹纠偏

constexpr float fpi = M_PI;
constexpr float sqrt2f = M_SQRT2;
constexpr float weight[8] = {-4.2,-3.0,-1.8,-0.6,0.6,1.8,3.0,4.2};

static act_stage_t stage = ACT_READY;
static uint32_t act_start_time = 0;
static uint8_t trail_data_front, trail_data_back;
static uint32_t trail_timestamp_f, trail_timestamp_b, state_time;
static uint16_t cross_count = 0, last_cross_count = 0;
static uint32_t last_cross_time = 0;
static bool turn_finished = false, last_mode = false;
static float aim_angle;

void set_speed(const float &vx, const float &vy, const float &rotate) {
    m0.update(rotate - vy * sqrt2f - vx * sqrt2f);
    m2.update(rotate + vy * sqrt2f + vx * sqrt2f);
    m1.update(rotate - vy * sqrt2f + vx * sqrt2f);
    m3.update(rotate + vy * sqrt2f - vx * sqrt2f);
}
void trail_mode(float &vx, float &vy, float &rotate) {
    float sum = 0; uint8_t cnt = 0; static uint8_t data_g;

    if (mode == E_MODE_TRAIL_F) data_g = trail_data_front;
    if (mode == E_MODE_TRAIL_B) data_g = trail_data_back;

    for (int i = 0; i < 8; i++) {
        if (data_g >> (7 - i) & 1) {
            sum += weight[i];
            cnt++;
        }
    }
    float trail_err = 0;
    if (cnt) trail_err = sum / static_cast <float> (cnt);

    vx = 0;
    if (mode == E_MODE_TRAIL_F) {
        vy = 13; rotate = trail_pid.update(trail_err, 0.0f);
    }
    if (mode == E_MODE_TRAIL_B) {
        vy = -15; rotate = -trail_pid.update(trail_err, 0.0f);
    }
}

void turn_mode(float &vx, float &vy, float &rotate) {
    float turn_speed = angle_pid.update(ins::data()->yaw_total_angle, aim_angle);
    vx = 0;
    vy = 0;
    rotate = turn_speed;
    if (fabs(ins::data()->yaw_total_angle - aim_angle) < 0.05f) {
        rotate = 0;
        turn_finished = true;
    }
}

void turn_dir(dir_t t) {
#ifdef SIDE_RED
    if (t == E_TURN_LEFT)
        aim_angle = ins::data()->yaw_total_angle + fpi / 2;
    if (t == E_TURN_RIGHT)
        aim_angle = ins::data()->yaw_total_angle - fpi / 2;
#else
    if (t == E_TURN_LEFT)
        aim_angle = ins::data()->yaw_total_angle - fpi / 2;
    if (t == E_TURN_RIGHT)
        aim_angle = ins::data()->yaw_total_angle + fpi / 2;
#endif
    turn_finished = false;
    mode = E_MODE_TURN;
}

static int count_l, count_r, count_l_lst, count_r_lst, count_b;
static uint8_t cross_confirm = 0;
void check_cross() {
    count_l_lst = count_l, count_r_lst = count_r;

    count_r = __builtin_popcount(trail_data_front & 0b00001111);
    count_l = __builtin_popcount(trail_data_front & 0b11110000);

    count_b = __builtin_popcount(trail_data_back & 0b11111111);

    bool is_cross = false;
    if (mode == E_MODE_TRAIL_F) {
        is_cross = (count_l - count_l_lst >= 2 || count_r - count_r_lst >= 3 || count_l == 4 || count_r == 4 );
    }
    if (mode == E_MODE_TRAIL_B) {
        is_cross = (count_b >= 6);
    }

    if (is_cross) {
        if (bsp_time_get_ms() - last_cross_time >1500 && ++cross_confirm > 3) {
            cross_beep_req = true;
            last_cross_time = bsp_time_get_ms();
            cross_count ++; //路口书更新处
            cross_confirm = 0;
        }
    }else cross_confirm = 0;
}
bool gimbal_action(lift_t target_lift, float target_servo1, float target_servo2) {
    static uint32_t lift_done_time = 0;
    switch (stage) {
    case ACT_READY:
        lift_finished = false;
        lift_mode = target_lift;
        act_start_time = bsp_time_get_ms();
        stage = ACT_LIFT;
        return false;

    case ACT_LIFT:
        // 等待升降到位
        if (lift_finished) {
            if (lift_done_time == 0) {
                lift_done_time = bsp_time_get_ms();
            }
            if (bsp_time_get_ms() - lift_done_time > 100) {
                servo2_angle = target_servo2;
                servo1_angle = target_servo1;
                act_start_time = bsp_time_get_ms();
                lift_done_time = 0;
                stage = ACT_SERVO;
            }
        }
        return false;

    case ACT_SERVO:
        // 舵机没有反馈
        if (bsp_time_get_ms() - act_start_time > 1000) {
            stage = ACT_READY; // 为下一次动作重置状态
            return true;       // 真正完成
        }
        return false;

    default:
        lift_done_time = 0;
        return false;
    }
}

[[noreturn]] void chassis_task(void *args) {
    bsp_uart_set_callback(E_UART_10, [](bsp_uart_e device, const uint8_t *data, size_t len) {
        if (len == 2 and data[0] == 0xAA) {
            trail_data_front = data[1];
            trail_timestamp_f = bsp_time_get_ms();
        }
    });
    bsp_uart_set_callback(E_UART_7, [](bsp_uart_e device, const uint8_t *data, size_t len) {
        if (len == 2 and data[0] == 0xBB) {
            trail_data_back = data[1];
            trail_timestamp_b = bsp_time_get_ms();
        }
    });
    os::task::sleep_seconds(1);

    m0.init(); m1.init(); m2.init(); m3.init();

    static float vx = 0, vy = 0, rotate = 0;
    dir_t current_turn = E_TURN_NONE;
    mode = E_MODE_TRAIL_F;

    while (true) {

        //强制停止
        if (cross_count >= 12) mode = E_MODE_DIED;

        bool front_ok = bsp_time_get_ms() - trail_timestamp_f < 100;
        bool back_ok  = bsp_time_get_ms() - trail_timestamp_b < 100;

        switch (mode) {
        case E_MODE_TRAIL_F:
            trail_mode(vx, vy, rotate);

            if (front_ok) {
                check_cross();

                if (cross_count > last_cross_count) {
                    last_cross_count = cross_count;

                    current_turn = route[cross_count];
                    state_time = bsp_time_get_ms();
                    last_mode = (mode == E_MODE_TRAIL_F);
                    mode = E_MODE_FUCK_CROSS;         // 此状态中会判断并更新至 TURN 状态
                }
            }
            // 进入分叉路口后前进的距离，由时间限制。进路口用
            if (cross_count == 2 || cross_count == 6 || cross_count == 8 || cross_count == 10)
                if (bsp_time_get_ms() - state_time > BRANCH_TIME) {
                    stage = ACT_READY;
                    mode = E_MODE_ACTION;
                }
            // 在此添加在路口完成任务之前的高度和夹爪，只能在出路口时调用。
            if (cross_count == 7) {
                // lift_mode = E_HIGH; servo2_angle = SERVO_2_ROTATE;
                gimbal_action(E_HIGH, SERVO_1_CLOSE, SERVO_2_ROTATE);
            }

            break;

        case E_MODE_TRAIL_B:
            trail_mode(vx, vy, rotate);

            if (back_ok) {
                check_cross();

                if (cross_count > last_cross_count) {
                    last_cross_count = cross_count;

                    current_turn = route[cross_count];
                    state_time = bsp_time_get_ms();
                    last_mode = false;
                    mode = E_MODE_FUCK_CROSS;
                }
            }
            break;

        case E_MODE_ACTION:
            vx = 0; vy = 0; rotate = 0;

            if (cross_count == 2) {
                if (gimbal_action(E_LOW, SERVO_1_OPEN, SERVO_2_NORMAL)) {
                    mode = E_MODE_TRAIL_B;        // 动作完直接切倒车
                }
            }
            else if (cross_count == 6) {
                if (gimbal_action(E_LOW, SERVO_1_CLOSE, SERVO_2_NORMAL)) {
                    mode = E_MODE_TRAIL_B;
                }
            }
            else if (cross_count == 8) {
                if (gimbal_action(E_HIGH, SERVO_1_OPEN, SERVO_2_ROTATE)) {
                    mode = E_MODE_TRAIL_B;
                }
            }
            else if (cross_count == 10) {
                if (gimbal_action(E_LOW, SERVO_1_OPEN, SERVO_2_NORMAL)) {
                    mode = E_MODE_TRAIL_B;
                }
            }
            break;

        case E_MODE_FUCK_CROSS:
            vx = 0;
            vy = (last_mode) ? 10 : -7;     // 使车身对准路口
            rotate = 0;

            if (bsp_time_get_ms() - state_time >= 300) {
                if (current_turn != E_TURN_NONE) {
                    turn_dir(current_turn);
                } else {
                    mode = E_MODE_TRAIL_F;
                }
            }
            break;

        case E_MODE_TURN:
            turn_mode(vx, vy, rotate);

            if (turn_finished) {
                turn_finished = false;
                state_time = bsp_time_get_ms();
                mode = E_MODE_WAIT;                   // WAIT 模式中也会更新至 TRAiL 模式
            }
            break;

        case E_MODE_WAIT:
            vx = 0; vy = 0; rotate = 0;

            if (bsp_time_get_ms() - state_time >= 200) {
                state_time = bsp_time_get_ms();
                mode = E_MODE_TRAIL_F;
            }
            break;

        case E_MODE_DIED:
            vx = 0; vy = 0; rotate = 0;
            break;
        }
        set_speed(vx, vy, rotate);

        vofa::send(E_UART_1, servo1_angle, servo2_angle, cross_count);

        os::task::sleep(1);
    }
}

[[noreturn]] void manual_chassis_task(void *args) {
    m0.init(); m1.init(); m2.init(); m3.init();
    float vx = 0, vy = 0, rotate = 0;
    auto rc = rc::dr16::data();
    while (true) {
        // 离线保护
        if (bsp_time_get_ms() - rc->timestamp > 100) {
            vx = 0, vy = 0, rotate = 0;
        } else {
            // 底盘
            vy = static_cast<float>(rc->rc_l[1]) / 10.f;
            vx = static_cast<float>(rc->rc_l[0]) / 10.f;
            rotate = static_cast<float>(rc->reserved) / 33.f;
            // 遥控器死区
            vy = (vy < 0.05f) ? 0.0f : vy;
            vx = (vx < 0.05f) ? 0.0f : vx;

        }
        // vofa::send(E_UART_1, rc->mouse_l, rc->mouse_r, rc->mouse_x, rc->mouse_y, rc->mouse_z);

        set_speed(vx, vy, rotate);
        os::task::sleep(1);
    }
}
