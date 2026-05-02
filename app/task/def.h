//
// Created by double_J on 2026/4/2.
//

#ifndef TROBOT_DEF_H
#define TROBOT_DEF_H

// 一些量的宏定义
#define BRANCH_TIME 750       // 进入支路停止的时间

#define SERVO_1_OPEN 110      // 放置
#define SERVO_1_CLOSE 63      //夹取
#define SERVO_2_NORMAL 50
#define SERVO_2_ROTATE 110

// 蜂鸣器请求
inline bool cross_beep_req = false;

inline bool lift_finished = false;
inline float servo1_angle = 63.0f, servo2_angle = 50.0f;
// chassis
typedef enum {
    E_MODE_WAIT,              //
    E_MODE_DIED,
    E_MODE_TRAIL_F,           // 循迹
    E_MODE_TRAIL_B,
    E_MODE_TURN,              // 原地旋转
    E_MODE_FUCK_CROSS,         // 过路口后一段时间不考虑循迹往前走
    E_MODE_ACTION
} mode_t;
typedef enum {
    ACT_READY,
    ACT_LIFT,
    ACT_SERVO,
    ACT_RESET
} act_stage_t;
// gimbal
typedef enum {
    E_IDLE,
    E_HIGH,
    E_LOW,
} lift_t;
inline lift_t lift_mode = E_IDLE;

#endif //TROBOT_DEF_H