// MCU-B 的 UART：向 MCU-A 发送心跳和姿态帧
// 帧格式：AA 55 类型 长度 载荷 校验和（头/类型/长度/载荷的异或）

#ifndef __UART_B_H
#define __UART_B_H

#include <stdint.h>

void UART_Init(void);
void UART_SendHeartbeat(void);
void UART_SendAttitude(float roll, float pitch, float yaw);

// 每 10ms 调用一次（可在 TIM1 100Hz 中断里调用）
void UART_Task10ms(void);

#endif
