# Full Table-Driven Chassis Reference

This file is a reference-only full rewrite of `chassis.cc`.

It is stored as Markdown so it will not be compiled by the current `app/CMakeLists.txt`, which glob-loads files under `app/`. The code below keeps the current behavior as closely as possible, but moves route decisions and gimbal action parameters into `route_plan[]`.

```cpp
//
// Table-driven chassis.cc reference.
// This is not compiled while it lives inside this Markdown file.
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
#include "robomaster/robomaster.h"

// Switch red/blue route direction.
#define SIDE_RED

typedef enum {
    E_TURN_LEFT,
    E_TURN_RIGHT,
    E_TURN_NONE
} dir_t;

typedef enum {
    E_ROUTE_ACTION_NONE,

    // Stop after entering branch for action_delay_ms, then run gimbal_action().
    E_ROUTE_ACTION_AFTER_BRANCH_DELAY,

    // Keep trying a prepare posture while passing this cross.
    // This matches the current cross_count == 7 early lift behavior.
    E_ROUTE_ACTION_KEEP_PREPARE
} route_action_t;

typedef struct {
    dir_t turn;
    route_action_t action_type;
    uint32_t action_delay_ms;
    lift_t lift;
    float servo1;
    float servo2;
    mode_t action_done_mode;
} route_step_t;

static constexpr route_step_t route_plan[] = {
    // 0: start
    {E_TURN_NONE,  E_ROUTE_ACTION_NONE,               0,           E_IDLE, SERVO_1_CLOSE, SERVO_2_NORMAL, E_MODE_TRAIL_F},

    // 1: enter main road
    {E_TURN_LEFT,  E_ROUTE_ACTION_NONE,               0,           E_IDLE, SERVO_1_CLOSE, SERVO_2_NORMAL, E_MODE_TRAIL_F},

    // 2: branch 1 in, low place, then reverse trail
    {E_TURN_RIGHT, E_ROUTE_ACTION_AFTER_BRANCH_DELAY, BRANCH_TIME, E_LOW,  SERVO_1_OPEN,  SERVO_2_NORMAL, E_MODE_TRAIL_B},

    // 3: branch 1 out
    {E_TURN_LEFT,  E_ROUTE_ACTION_NONE,               0,           E_IDLE, SERVO_1_CLOSE, SERVO_2_NORMAL, E_MODE_TRAIL_F},

    // 4: skip branch 2
    {E_TURN_NONE,  E_ROUTE_ACTION_NONE,               0,           E_IDLE, SERVO_1_CLOSE, SERVO_2_NORMAL, E_MODE_TRAIL_F},

    // 5: skip branch 3
    {E_TURN_NONE,  E_ROUTE_ACTION_NONE,               0,           E_IDLE, SERVO_1_CLOSE, SERVO_2_NORMAL, E_MODE_TRAIL_F},

    // 6: branch 4 in, low grab, then reverse trail
    {E_TURN_LEFT,  E_ROUTE_ACTION_AFTER_BRANCH_DELAY, BRANCH_TIME, E_LOW,  SERVO_1_CLOSE, SERVO_2_NORMAL, E_MODE_TRAIL_B},

    // 7: branch 4 out, early lift/rotate to avoid collision with placing platform
    {E_TURN_LEFT,  E_ROUTE_ACTION_KEEP_PREPARE,       0,           E_HIGH, SERVO_1_CLOSE, SERVO_2_ROTATE, E_MODE_TRAIL_F},

    // 8: branch 3 in, high place, then reverse trail
    {E_TURN_LEFT,  E_ROUTE_ACTION_AFTER_BRANCH_DELAY, BRANCH_TIME, E_HIGH, SERVO_1_OPEN,  SERVO_2_ROTATE, E_MODE_TRAIL_B},

    // 9: branch 3 out
    {E_TURN_RIGHT, E_ROUTE_ACTION_NONE,               0,           E_IDLE, SERVO_1_CLOSE, SERVO_2_NORMAL, E_MODE_TRAIL_F},

    // 10: branch 2 in, low place, then reverse trail
    {E_TURN_LEFT,  E_ROUTE_ACTION_AFTER_BRANCH_DELAY, BRANCH_TIME, E_LOW,  SERVO_1_OPEN,  SERVO_2_NORMAL, E_MODE_TRAIL_B},

    // 11: branch 2 out. Current code stops after cross_count reaches 12.
    {E_TURN_LEFT,  E_ROUTE_ACTION_NONE,               0,           E_IDLE, SERVO_1_CLOSE, SERVO_2_NORMAL, E_MODE_TRAIL_F},
};

static constexpr uint16_t route_plan_size =
    sizeof(route_plan) / sizeof(route_plan[0]);

mode_t mode = E_MODE_TRAIL_F;

motor::gyj m0("m0", {.id = 0x00,.port = E_CAN_1,.mode = motor::gyj::SPEED_LOOP,.have_feedback = true}, 1);
motor::gyj m1("m1", {.id = 0x01,.port = E_CAN_1,.mode = motor::gyj::SPEED_LOOP,.have_feedback = true}, 1);
motor::gyj m2("m2", {.id = 0x02,.port = E_CAN_1,.mode = motor::gyj::SPEED_LOOP,.have_feedback = false}, 1);
motor::gyj m3("m3", {.id = 0x03,.port = E_CAN_1,.mode = motor::gyj::SPEED_LOOP,.have_feedback = false}, 1);

controller::pid angle_pid(20, 1, 0, 5, 20);
controller::pid trail_pid(2.5, 0, 0.1, 0, 8);

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

static inline const route_step_t &current_route_step() {
    if (cross_count < route_plan_size) {
        return route_plan[cross_count];
    }
    return route_plan[route_plan_size - 1];
}

static inline bool route_step_need_branch_action(const route_step_t &step) {
    return step.action_type == E_ROUTE_ACTION_AFTER_BRANCH_DELAY;
}

static inline bool route_step_need_prepare_action(const route_step_t &step) {
    return step.action_type == E_ROUTE_ACTION_KEEP_PREPARE;
}

void set_speed(const float &vx, const float &vy, const float &rotate) {
    m0.update(rotate - vy * sqrt2f - vx * sqrt2f);
    m2.update(rotate + vy * sqrt2f + vx * sqrt2f);
    m1.update(rotate - vy * sqrt2f + vx * sqrt2f);
    m3.update(rotate + vy * sqrt2f - vx * sqrt2f);
}

void trail_mode(float &vx, float &vy, float &rotate) {
    float sum = 0;
    uint8_t cnt = 0;
    static uint8_t data_g;

    if (mode == E_MODE_TRAIL_F) data_g = trail_data_front;
    if (mode == E_MODE_TRAIL_B) data_g = trail_data_back;

    for (int i = 0; i < 8; i++) {
        if (data_g >> (7 - i) & 1) {
            sum += weight[i];
            cnt++;
        }
    }

    float trail_err = 0;
    if (cnt) trail_err = sum / static_cast<float>(cnt);

    vx = 0;
    if (mode == E_MODE_TRAIL_F) {
        vy = 13;
        rotate = trail_pid.update(trail_err, 0.0f);
    }
    if (mode == E_MODE_TRAIL_B) {
        vy = -15;
        rotate = -trail_pid.update(trail_err, 0.0f);
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
    count_l_lst = count_l;
    count_r_lst = count_r;

    count_r = __builtin_popcount(trail_data_front & 0b00001111);
    count_l = __builtin_popcount(trail_data_front & 0b11110000);
    count_b = __builtin_popcount(trail_data_back & 0b11111111);

    bool is_cross = false;
    if (mode == E_MODE_TRAIL_F) {
        is_cross = (count_l - count_l_lst >= 2 || count_r - count_r_lst >= 3 || count_l == 4 || count_r == 4);
    }
    if (mode == E_MODE_TRAIL_B) {
        is_cross = (count_b >= 6);
    }

    if (is_cross) {
        if (bsp_time_get_ms() - last_cross_time > 1500 && ++cross_confirm > 3) {
            cross_beep_req = true;
            last_cross_time = bsp_time_get_ms();
            cross_count++;
            cross_confirm = 0;
        }
    } else {
        cross_confirm = 0;
    }
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
        if (bsp_time_get_ms() - act_start_time > 1000) {
            stage = ACT_READY;
            return true;
        }
        return false;

    default:
        lift_done_time = 0;
        return false;
    }
}

static void update_cross_event(dir_t &current_turn) {
    if (cross_count > last_cross_count) {
        last_cross_count = cross_count;

        const route_step_t &step = current_route_step();
        current_turn = step.turn;
        state_time = bsp_time_get_ms();
        last_mode = (mode == E_MODE_TRAIL_F);
        mode = E_MODE_FUCK_CROSS;
    }
}

static void update_front_trail_extra_action() {
    const route_step_t &step = current_route_step();

    if (route_step_need_branch_action(step)) {
        if (bsp_time_get_ms() - state_time > step.action_delay_ms) {
            stage = ACT_READY;
            mode = E_MODE_ACTION;
        }
    }

    if (route_step_need_prepare_action(step)) {
        gimbal_action(step.lift, step.servo1, step.servo2);
    }
}

static void update_route_action() {
    const route_step_t &step = current_route_step();

    if (!route_step_need_branch_action(step)) {
        mode = E_MODE_TRAIL_F;
        return;
    }

    if (gimbal_action(step.lift, step.servo1, step.servo2)) {
        mode = step.action_done_mode;
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

    m0.init();
    m1.init();
    m2.init();
    m3.init();

    static float vx = 0, vy = 0, rotate = 0;
    dir_t current_turn = E_TURN_NONE;
    mode = E_MODE_TRAIL_F;

    while (true) {
        if (cross_count >= route_plan_size) {
            mode = E_MODE_DIED;
        }

        bool front_ok = bsp_time_get_ms() - trail_timestamp_f < 100;
        bool back_ok  = bsp_time_get_ms() - trail_timestamp_b < 100;

        switch (mode) {
        case E_MODE_TRAIL_F:
            trail_mode(vx, vy, rotate);

            if (front_ok) {
                check_cross();
                update_cross_event(current_turn);
            }

            update_front_trail_extra_action();
            break;

        case E_MODE_TRAIL_B:
            trail_mode(vx, vy, rotate);

            if (back_ok) {
                check_cross();
                update_cross_event(current_turn);
            }
            break;

        case E_MODE_ACTION:
            vx = 0;
            vy = 0;
            rotate = 0;
            update_route_action();
            break;

        case E_MODE_FUCK_CROSS:
            vx = 0;
            vy = (last_mode) ? 10 : -7;
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
                mode = E_MODE_WAIT;
            }
            break;

        case E_MODE_WAIT:
            vx = 0;
            vy = 0;
            rotate = 0;

            if (bsp_time_get_ms() - state_time >= 200) {
                state_time = bsp_time_get_ms();
                mode = E_MODE_TRAIL_F;
            }
            break;

        case E_MODE_DIED:
            vx = 0;
            vy = 0;
            rotate = 0;
            break;
        }

        set_speed(vx, vy, rotate);
        vofa::send(E_UART_1, servo1_angle, servo2_angle, cross_count);
        os::task::sleep(1);
    }
}

[[noreturn]] void manual_chassis_task(void *args) {
    m0.init();
    m1.init();
    m2.init();
    m3.init();

    float vx = 0, vy = 0, rotate = 0;
    auto rc = rc::dr16::data();

    bsp_uart_set_baudrate(E_UART_1, 115200);
    auto vision_set = [](const uint8_t &x) {
        robomaster::transmit(E_UART_1, 0x0f01, &x, 1);
    };

    vision_set(1);

    while (true) {
        if (rc->s_r == 1) {
            bsp_sys_reset();
        }

        if (bsp_time_get_ms() - rc->timestamp > 100) {
            vx = 0;
            vy = 0;
            rotate = 0;
        } else {
            vy = static_cast<float>(rc->rc_l[1]) / 20.f;
            vx = static_cast<float>(rc->rc_l[0]) / 20.f;
            rotate = -static_cast<float>(rc->reserved) / 33.f;

            vy = (fabsf(vy) < 0.1f) ? 0.0f : vy;
            vx = (fabsf(vx) < 0.1f) ? 0.0f : vx;
        }

        set_speed(vx, vy, rotate);
        os::task::sleep(1);
    }
}
```

## Notes

- `cross_count == 7` is preserved as `E_ROUTE_ACTION_KEEP_PREPARE`.
- The action table keeps the same action parameters as current `chassis.cc`.
- The stop condition uses `cross_count >= route_plan_size`, equivalent to the current `cross_count >= 12`.
- `current_route_step()` includes a simple clamp to avoid table overflow if the route counter glitches. If you want the exact current behavior, remove that helper clamp and index directly.
