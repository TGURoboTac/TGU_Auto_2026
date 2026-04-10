//
// Created by fish on 2025/9/22.

#pragma once

#include "bsp/def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 设置定时器基本参数
 * @param device 定时器外设指针
 * @param period
 * @param prescaler
 */
void bsp_tim_set(bsp_timer_handle_t *device, uint32_t period, uint32_t prescaler);

/**
 * 设置定时器频率到指定值, 会自动计算基本参数
 * @param device 定时器外设指针
 * @param p 频率 (Hz)
 */
void bsp_tim_config(bsp_timer_handle_t *device, float p);

/**
 * 设置定时器占空比
 * @param device 定时器外设指针
 * @param channel 定时器通道
 * @param duty 占空比
 */
void bsp_tim_set_duty(bsp_timer_handle_t *device, uint32_t channel, float duty);

/**
 * 使能 pwm 输出
 * @param device 定时器外设指针
 * @param channel 定时器通道
 */
void bsp_tim_pwm_enable(bsp_timer_handle_t *device, uint32_t channel);

/**
 * 失能 pwm 输出
 * @param device 定时器外设指针
 * @param channel 定时器通道
 */
void bsp_tim_pwm_disable(bsp_timer_handle_t *device, uint32_t channel);

/**
 * 设置定时中断回调
 * @param device 定时器外设指针
 * @param callback 回调函数
 */
void bsp_tim_set_callback(bsp_timer_handle_t *device, void (*callback)(bsp_timer_handle_t *htim));

#ifdef __cplusplus
}
#endif
