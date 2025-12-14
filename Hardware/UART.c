#include "UART.h"
#include "stm32f10x.h" // Device header
#include <string.h>

// 帧格式: AA 55
#define FRAME_HEAD1 0xAA
#define FRAME_HEAD2 0x55
#define FRAME_TYPE_HEARTBEAT 0x01
#define FRAME_TYPE_ATTITUDE 0x02

// 心跳周期: 100ms (10 * 10ms tick)
#define HEARTBEAT_TICKS 10U

static uint8_t hb_tick = 0;

/**
 * @brief 发送一个字节数据
 * @param data 要发送的字节
 */
static void UART_SendByte(uint8_t data)
{
    USART_SendData(USART1, data);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        ;
}

/**
 * @brief 发送自定义协议帧
 * @param type 帧类型 (心跳或姿态)
 * @param payload 数据载荷指针
 * @param len 数据载荷长度
 *
 * 帧结构: [HEAD1][HEAD2][TYPE][LEN][PAYLOAD...][CHECKSUM]
 * CHECKSUM = HEAD1 ^ HEAD2 ^ TYPE ^ LEN ^ PAYLOAD...
 */
static void UART_SendFrame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t cs = FRAME_HEAD1 ^ FRAME_HEAD2 ^ type ^ len;
    UART_SendByte(FRAME_HEAD1);
    UART_SendByte(FRAME_HEAD2);
    UART_SendByte(type);
    UART_SendByte(len);
    for (uint8_t i = 0; i < len; i++) {
        UART_SendByte(payload[i]);
        cs ^= payload[i];
    }
    UART_SendByte(cs);
}

/**
 * @brief 初始化 UART1
 *
 * 配置参数:
 * - 波特率: 115200
 * - 数据位: 8
 * - 停止位: 1
 * - 校验位: 无
 * - 流控制: 无
 * - 引脚: TX=PA9, RX=PA10
 */
void UART_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief 发送心跳包
 *
 * 用于告知接收端（MCU-A）本设备（MCU-B）在线。
 */
void UART_SendHeartbeat(void)
{
    UART_SendFrame(FRAME_TYPE_HEARTBEAT, NULL, 0);
}

/**
 * @brief 发送姿态角数据
 * @param roll 横滚角 (float)
 * @param pitch 俯仰角 (float)
 * @param yaw 偏航角 (float)
 *
 * 将三个浮点数打包发送。
 */
void UART_SendAttitude(float roll, float pitch, float yaw)
{
    uint8_t payload[12];
    memcpy(payload + 0, &roll, 4);
    memcpy(payload + 4, &pitch, 4);
    memcpy(payload + 8, &yaw, 4);
    UART_SendFrame(FRAME_TYPE_ATTITUDE, payload, sizeof(payload));
}

/**
 * @brief UART 周期性任务
 *
 * 应在定时器中断中每 10ms 调用一次。
 * 功能：
 * - 维护心跳发送计时器，每 100ms 发送一次心跳包。
 */
void UART_Task10ms(void)
{
    if (++hb_tick >= HEARTBEAT_TICKS) {
        hb_tick = 0;
        UART_SendHeartbeat();
    }
}
