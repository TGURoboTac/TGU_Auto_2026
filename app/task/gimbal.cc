//
// Created by JJ on 2026/4/1.
//
#include <cstdio>
#include <algorithm>

#include "bsp/tim.h"
#include "bsp/time.h"
#include "controller/pid.h"
#include "motor/dji.h"
#include "utils/os.h"
#include "def.h"
#include "bsp/uart.h"
#include "rc/dr16.h"
#include "rc/ht10.h"
#include "utils/vofa.h"
extern TIM_HandleTypeDef htim2;

using namespace motor;
controller::pid spd_pid(200, 0, 3, 3000, 10000);
controller::pid po_pid(30, 0, 600, 0, 20);
dji lift("m", dji::M2006,dji::param_t{.id = 1,.port = E_CAN_2,.mode = dji::CURRENT});
constexpr float fpi = M_PI;
static float measure_speed;

typedef enum {
    SERVO_1,
    SERVO_2
} servo_id_t;

struct {
    float zero = 0;              // 软零点
    uint8_t zero_flag = 0;       // 回零完成
    uint8_t reaching_flag = 0;   // 到位
    uint32_t stall_ms = 0;       // 堵转计时
    uint32_t reach_ms = 0;       //到位计时
    float zero_speed = 0;
} lift_state;

void lift_soft_init() {
    lift_state.zero_speed = spd_pid.update(lift.feedback.speed,-10);
    lift.update(lift_state.zero_speed);

    // 堵转检测
    if (lift.output < -1800 && fabsf(lift.feedback.current) > 1.8 && fabsf(lift.feedback.speed) < 1) {
        if (lift_state.stall_ms == 0)
            lift_state.stall_ms = bsp_time_get_ms();
        // 可以修改阈值（堵转时间）
        if (bsp_time_get_ms() - lift_state.stall_ms > 50) {
            //判断到限位
            lift.update(0);
            spd_pid.clear();
            po_pid.clear();
            lift_state.zero = static_cast<float>(lift.feedback.round) * 2 * fpi + lift.feedback.angle;
            lift_state.zero_flag = 1;
            lift_state.stall_ms = 0;

            spd_pid.clear();
            po_pid.clear();
        }
    }else lift_state.stall_ms = 0;
}

float lift_get_soft_pos() {
    // if 软零点为零 返回值即电机total_angle
    return static_cast<float>(lift.feedback.round) * 2 * fpi  + lift.feedback.angle - lift_state.zero;
}

void lift_move_to(float target_pos) {
    float pos = lift_get_soft_pos();
    float err = target_pos - pos;

    float _speed = po_pid.update(pos,target_pos);
    float speed_output_ = spd_pid.update(measure_speed,_speed);
    lift.update(speed_output_);

    if(fabsf(err) < 3.0f) {
        lift_state.reaching_flag = 1;
    }else {
        lift_state.reaching_flag = 0;
    }
}
// auto mode
[[noreturn]] void lift_task(void *args) {
    lift.init();
    os::task::sleep_seconds(1);

    while (lift_state.zero_flag != 1) {
        lift_soft_init();
        os::task::sleep(10);
    }
    lift_mode = E_HIGH; // 初始高度就是low
    while (true) {
        constexpr  float alpha = 0.09;
        measure_speed = (1 - alpha) * measure_speed + alpha * lift.feedback.speed;

        if (bsp_time_get_ms() - lift.feedback.timestamp > 3) {
            spd_pid.clear();
            lift.update(38);
        }
        else {
            if (lift_mode == E_HIGH) {
                lift_move_to(38);
            }
            if (lift_mode == E_LOW) {
                lift_move_to(12);
            }
            lift_finished = (lift_state.reaching_flag == 1);
        }
        os::task::sleep(1);
    }
}

void servo_init() {
    bsp_tim_set(&htim2, 20000 - 1, 240 - 1);
    bsp_tim_pwm_enable(&htim2,TIM_CHANNEL_1);              // 靠近接收器的
    bsp_tim_pwm_enable(&htim2,TIM_CHANNEL_3);              // 远离接收器的
}

void servo_angle_set(servo_id_t id, float angle) {
    angle = std::clamp(angle, 0.0f, 180.0f);

    float duty = 0.025f + (angle / 180.0f) * 0.1f;

    switch (id) {
    case SERVO_1:
        bsp_tim_set_duty(&htim2,TIM_CHANNEL_3,duty);
        break;
    case SERVO_2:
        bsp_tim_set_duty(&htim2,TIM_CHANNEL_1,duty);
        break;
    }
}
[[noreturn]] void servo_task(void *args) {
    servo_init();
    os::task::sleep(100);
    servo_angle_set(SERVO_1, 68);// 63
    servo_angle_set(SERVO_2, 50);// 50

    // servo1 的夹取角度60以下，滚动角度110
    // servo2 的夹取角度50，滚动角度110
    while (true) {
        servo_angle_set(SERVO_1, servo1_angle);
        servo_angle_set(SERVO_2, servo2_angle);
        os::task::sleep(20);
    }
}
