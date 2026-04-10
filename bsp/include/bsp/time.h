//
// Created by fish on 2025/9/13.
//

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 获取当前系统时间戳 (ms)
 * @return 当前系统时间戳 (ms)
 */
uint32_t bsp_time_get_ms();

/**
 * 获取当前系统时间戳 (us)
 * @return 当前系统时间戳 (us)
 */
uint64_t bsp_time_get_us();

/**
 * 系统级阻塞 delay (ms)
 * @param ms 延时时间 (ms)
 */
void bsp_time_delay(uint32_t ms);

/**
 * 系统级阻塞 delay (us)
 * @param us 延时时间 (us)
 */
void bsp_time_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif