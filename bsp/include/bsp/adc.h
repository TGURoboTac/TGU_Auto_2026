//
// Created by fish on 2026/1/25.
//

#ifndef TROBOT_VBUS_H
#define TROBOT_VBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化 ADC
 */
void bsp_adc_init(void);

/**
 * 获取 vbus 电压
 * @return vbus 电压 (V)
 */
float bsp_adc_vbus(void);

#ifdef __cplusplus
}
#endif

#endif //TROBOT_VBUS_H