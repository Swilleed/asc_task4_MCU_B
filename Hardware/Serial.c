#include "stm32f10x.h" // Device header
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

char Serial_RxPacket[100]; //"@MSG\r\n"
uint8_t Serial_RxFlag;

#define JUSTFLOAT_MAX_CHANNELS 8

static uint8_t Serial_JustFloatBuffer[JUSTFLOAT_MAX_CHANNELS * 4 + 4];
static void Serial_PackJustFloat(const float *dataArray, uint8_t dataCount, uint8_t *txBuffer);

void Serial_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        ;
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte(Array[i]);
    }
}

void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        Serial_SendByte(String[i]);
    }
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

static void Serial_PackJustFloat(const float *dataArray, uint8_t dataCount, uint8_t *txBuffer)
{
    uint8_t idx = 0;
    for (uint8_t i = 0; i < dataCount; i++) {
        memcpy(&txBuffer[idx], &dataArray[i], sizeof(float));
        idx += sizeof(float);
    }
    txBuffer[idx++] = 0x00;
    txBuffer[idx++] = 0x00;
    txBuffer[idx++] = 0x80;
    txBuffer[idx] = 0x7F;
}

void Serial_SendJustFloat(const float *dataArray, uint8_t dataCount)
{
    if (dataArray == NULL || dataCount == 0 || dataCount > JUSTFLOAT_MAX_CHANNELS) {
        return;
    }

    uint16_t frameLength = (uint16_t)dataCount * sizeof(float) + 4;
    Serial_PackJustFloat(dataArray, dataCount, Serial_JustFloatBuffer);
    Serial_SendArray(Serial_JustFloatBuffer, frameLength);
}

int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

uint8_t Serial_TryParseTarget(int16_t *target)
{
    if (Serial_RxFlag == 0) {
        return 0;
    }

    char packet[sizeof(Serial_RxPacket)];

    __disable_irq();
    strncpy(packet, Serial_RxPacket, sizeof(packet) - 1);
    packet[sizeof(packet) - 1] = '\0';
    Serial_RxFlag = 0;
    __enable_irq();

    char *ptr = packet;
    while (*ptr == ' ') {
        ptr++;
    }

    if ((strncmp(ptr, "SPD", 3) == 0) || (strncmp(ptr, "spd", 3) == 0)) {
        ptr += 3;
        while (*ptr == ' ' || *ptr == ':' || *ptr == '=') {
            ptr++;
        }
    }

    char *endptr;
    long value = strtol(ptr, &endptr, 10);
    if (endptr == ptr) {
        return 0;
    }

    if (value > 99) {
        value = 99;
    }
    else if (value < -99) {
        value = -99;
    }

    *target = (int16_t)value;
    return 1;
}

void USART2_IRQHandler(void)
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
        uint8_t RxData = USART_ReceiveData(USART2);

        if (RxState == 0) {
            if (RxData == '@' && Serial_RxFlag == 0) {
                RxState = 1;
                pRxPacket = 0;
            }
        }
        else if (RxState == 1) {
            if (RxData == '\r') {
                RxState = 2;
            }
            else {
                Serial_RxPacket[pRxPacket] = RxData;
                pRxPacket++;
            }
        }
        else if (RxState == 2) {
            if (RxData == '\n') {
                RxState = 0;
                Serial_RxPacket[pRxPacket] = '\0';
                Serial_RxFlag = 1;
            }
        }

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
