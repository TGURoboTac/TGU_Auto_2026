//
// Created by fish on 2026/1/6.
//

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化板载 flash
 * @return 失败返回 0, 成功返回 1
 */
uint8_t bsp_flash_init();

/**
 * 从 flash 中读取指定字段的数据到指定地址
 * @param s 字段名
 * @param buf 内存指针
 * @param len 内存长度
 */
void bsp_flash_read(const char *s, void *buf, size_t len);

/**
 * 将指定地址的数据写入 flash 中指定字段
 * @param s 字段名
 * @param buf 内存指针
 * @param len 内存长度
 * @return 失败返回 0, 成功返回 1
 */
uint8_t bsp_flash_write(const char *s, void *buf, size_t len);

#ifdef __cplusplus
}
#endif