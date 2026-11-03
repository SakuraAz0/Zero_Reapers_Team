//
// led.h —— 档位指示 LED（3 个，对应慢/中/快三档）
//
// 接线（高电平点亮）：PA8 -- 330~470Ω -- LED -- GND，PA11、PA12 同理
//
#ifndef __LED_H
#define __LED_H

#include "main.h"

#define LED_PIN_SLOW   GPIO_PIN_8    /* PA8  ：慢速档 */
#define LED_PIN_MID    GPIO_PIN_11   /* PA11 ：中速档 */
#define LED_PIN_FAST   GPIO_PIN_12   /* PA12 ：快速档 */
#define LED_PORT       GPIOA

void LED_Init(void);
void LED_SetGear(uint8_t gear);      /* 0=慢 1=中 2=快；只点亮对应那一个 */
void LED_AllOff(void);

/* 诊断用：用慢速 LED(PA8) 闪 count 次（约 150ms 亮 / 250ms 灭），用于开机显示复位原因 */
void LED_BlinkCode(uint8_t count);

#endif
