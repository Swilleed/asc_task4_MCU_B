#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "MahonyAHRS.h"
#include <math.h>
#include "MPU6050.h"
#include "UART.h"
#include "Serial.h"

static int16_t AccX, AccY, AccZ;
static int16_t GyroX, GyroY, GyroZ;
static float GyroBiasX, GyroBiasY, GyroBiasZ;
static float rowl, pitch, yaw;
static uint8_t att_tx_div = 0;
static uint16_t static_count = 0; // 静止状态计数器
/**
 * @brief 简单的上电陀螺仪零偏校准函数
 *
 * 注意事项：
 * 1. 在执行此函数期间，必须保持开发板绝对静止。
 * 2. 采集一定数量的样本求平均值，作为陀螺仪的零偏（Bias）。
 * 3. 该零偏值将在后续的姿态解算中被减去，以消除静态漂移。
 */
static void CalibrateGyro(void)
{
    const uint16_t samples = 500; // 采样次数，500次 * 5ms = 约2.5秒
    int32_t sumX = 0, sumY = 0, sumZ = 0;

    // 循环采集数据
    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_GetData(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);
        sumX += GyroX;
        sumY += GyroY;
        sumZ += GyroZ;
        Delay_ms(5); // 采样间隔 5ms
    }

    // 计算平均值得到零偏
    GyroBiasX = sumX / (float)samples;
    GyroBiasY = sumY / (float)samples;
    GyroBiasZ = sumZ / (float)samples;
}

int main(void)
{
    OLED_Init();
    MPU6050_Init();
    UART_Init();
    Serial_Init();
    CalibrateGyro();
    Timer_Init();

    while (1) {
        MPU6050_GetData(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);
    }
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {

        float gx = GyroX - GyroBiasX;
        float gy = GyroY - GyroBiasY;
        float gz = GyroZ - GyroBiasZ;

        // --- 动态零偏校准逻辑 ---
        // 阈值设为约 1.5 deg/s (200 LSB)，如果在此范围内则认为可能静止
        if (fabs(gx) < 200.0f && fabs(gy) < 200.0f && fabs(gz) < 200.0f) {
            static_count++;
            // 持续静止超过 1秒 (100 * 10ms = 1s)
            if (static_count > 100) {
                // 缓慢更新零偏，跟随温漂
                GyroBiasX += gx * 0.005f;
                GyroBiasY += gy * 0.005f;
                GyroBiasZ += gz * 0.005f;

                // 防止计数器溢出
                if (static_count > 1000)
                    static_count = 1000;
            }
        }
        else {
            static_count = 0; // 检测到运动，重置计数器
        }

        // --- 死区处理 ---
        // 滤除微小底噪，保持绝对静止
        if (fabs(gx) < 5.0f)
            gx = 0.0f;
        if (fabs(gy) < 5.0f)
            gy = 0.0f;
        if (fabs(gz) < 5.0f)
            gz = 0.0f;

        // MPU6050 初始化为 +/- 2000 dps (LSB/dps = 16.4) 和 +/- 16g (LSB/g = 2048)
        MahonyAHRSupdateIMU(gx / 16.4f * 3.1415926f / 180.0f,
                            gy / 16.4f * 3.1415926f / 180.0f,
                            gz / 16.4f * 3.1415926f / 180.0f,
                            AccX / 2048.0f,
                            AccY / 2048.0f,
                            AccZ / 2048.0f);
        rowl = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
        pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.29578f;
        yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;

        if (++att_tx_div >= 5) {
            att_tx_div = 0;
            UART_SendAttitude(rowl, pitch, yaw);

            float data[3];
            data[0] = pitch;
            data[1] = rowl;
            data[2] = yaw;
            Serial_SendJustFloat(data, 3);
        }

        UART_Task10ms();

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}