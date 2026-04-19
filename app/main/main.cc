//
// Created by fish on 2025/9/13.
//

#include "bsp/bsp.h"
#include "bsp/led.h"
#include "bsp/time.h"

#include "bsp/buzzer.h"
#include "ins/ins.h"
#include "rc/dr16.h"
#include "rc/ht10.h"
#include "utils/os.h"

extern void chassis_task(void *args);
extern void lift_task(void *args);
extern void servo_task(void *args);

extern "C" [[noreturn]] void app_entrance(void *args) {
    bsp_hw_init();

    bsp_led_set(0, 50, 0);
    bsp_buzzer_flash(4500, 0.2f, 100);
    bsp_time_delay(100);

    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);
    bsp_uart_set_baudrate(E_UART_10, 115200);
    bsp_uart_set_baudrate(E_UART_7, 115200);
    // Init Basic Components'

    // logger::init(E_UART_1, logger::INFO);
    // terminal::init(E_UART_1, 921600);
    // rc::dr16::init(E_UART_5);
    // rc::ht10::init(E_UART_5);

    ins::init();
    while (!ins::inited) os::task::sleep(5), bsp_iwdg_refresh();

    bsp_buzzer_flash(4500, 0.2f, 75);
    bsp_time_delay(50);
    bsp_buzzer_flash(4500, 0.2f, 75);

    // Init Application Tasks
    os::task::static_create(chassis_task, nullptr, "chassis", 1024, os::task::Priority::HIGH);
    os::task::static_create(lift_task, nullptr, "lift", 1024, os::task::Priority::HIGH);
    os::task::static_create(servo_task, nullptr, "servo", 256, os::task::Priority::HIGH);

    for (;;) {
        bsp_led_set_hsv(static_cast<float>(bsp_time_get_ms() % 3000) / 3000.0f, 1.0f, 0.3f);
        bsp_iwdg_refresh();
        os::task::sleep(5);
    }
}
