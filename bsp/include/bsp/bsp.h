//
// Created by fish on 2025/9/24.
//

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * bsp 硬件初始化
 */
void bsp_hw_init();

/**
 * 喂狗
 */
void bsp_iwdg_refresh();

#ifdef __cplusplus
}
#endif