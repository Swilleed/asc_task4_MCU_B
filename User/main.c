#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "MahonyAHRS.h"
#include <math.h>
#include "MPU6050.h"

static int16_t AccX, AccY, AccZ;
static int16_t GyroX, GyroY, GyroZ;
static float GyroBiasX, GyroBiasY, GyroBiasZ;
static float rowl, pitch, yaw;

// Simple on-boot gyro bias calibration; keep the board still while it runs
static void CalibrateGyro(void)
{
    const uint16_t samples = 500; // ~2.5s at 5ms per sample
    int32_t sumX = 0, sumY = 0, sumZ = 0;

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_GetData(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);
        sumX += GyroX;
        sumY += GyroY;
        sumZ += GyroZ;
        Delay_ms(5);
    }

    GyroBiasX = sumX / (float)samples;
    GyroBiasY = sumY / (float)samples;
    GyroBiasZ = sumZ / (float)samples;
}

int main(void)
{
    OLED_Init();
    MPU6050_Init();
    CalibrateGyro();
    Timer_Init();

    while (1) {
        MPU6050_GetData(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);
        OLED_ShowString(1, 1, "Roll:");
        OLED_ShowSignedNum(1, 7 * 6, (int)(rowl * 100), 6);
        OLED_ShowString(2, 1, "Pitch:");
        OLED_ShowSignedNum(2, 7 * 6, (int)(pitch * 100), 6);
        OLED_ShowString(3, 1, "Yaw:");
        OLED_ShowSignedNum(3, 7 * 6, (int)(yaw * 100), 6);
    }
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        // Subtract calibrated biases before converting to rad/s
        MahonyAHRSupdateIMU((GyroX - GyroBiasX) / 131.0f * 3.1415926f / 180.0f,
                            (GyroY - GyroBiasY) / 131.0f * 3.1415926f / 180.0f,
                            (GyroZ - GyroBiasZ) / 131.0f * 3.1415926f / 180.0f,
                            AccX / 16384.0f,
                            AccY / 16384.0f,
                            AccZ / 16384.0f);
        rowl = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
        pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.29578f;
        yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}