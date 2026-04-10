//
// Created by fish on 2026/1/8.
//

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 判断是否在中断中
 * @return 若在中断中调用则返回 1, 否则返回 0
 */
uint8_t bsp_sys_in_isr();

/**
 * 软件 reset, 效果几乎等效于按 reset 键
 */
void bsp_sys_reset();

/**
 * 进入临界区
 * @return state, 在 exit 时需传入
 */
unsigned long bsp_sys_enter_critical();

/**
 * 退出临界区
 * @param state 进入时获取的 state
 */
void bsp_sys_exit_critical(unsigned long state);

#ifdef __cplusplus
}
#endif