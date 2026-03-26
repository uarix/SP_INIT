#ifndef __PWM_CTRL_H
#define __PWM_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

/**
 * @brief 设置A/B两路PWM的占空比（高电平有效）
 * @param a_high  A路占空比，范围 0.0 ~ 1.0，对应Timer A CMP1/CMP3输出
 * @param b_high  B路占空比，范围 0.0 ~ 1.0，对应Timer B CMP1/CMP3输出
 * @note  占空比被限制在[0,1]区间内，CMP1固定为CTRL_PWM_START_TICKS，CMP3 = CMP1 + Ton
 *        当占空比为0时，CMP3 == CMP1，输出始终为低电平（无有效高脉冲）
 */
void HRTIM_UpdateHighDuty(float a_high, float b_high);

/**
 * @brief Buck模式占空比分配：A路输出给定duty，B路常开（占空比1.0）
 * @param duty     目标占空比（0.0~1.0），将被限幅
 * @param a_high   输出指针，用于接收A路占空比
 * @param b_high   输出指针，用于接收B路占空比（恒为1.0）
 * @note  若传入指针为NULL，则不会进行赋值，并会将内部a/b置0（安全保护）
 */
void Charge_buck(float duty, float *a_high, float *b_high);

/**
 * @brief Boost模式占空比分配：A路常开（占空比1.0），B路输出给定duty
 * @param duty     目标占空比（0.0~1.0），将被限幅
 * @param a_high   输出指针，用于接收A路占空比（恒为1.0）
 * @param b_high   输出指针，用于接收B路占空比
 * @note  若传入指针为NULL，则不会进行赋值，并会将内部a/b置0（安全保护）
 */
void Output_boost(float duty, float *a_high, float *b_high);

#ifdef __cplusplus
}
#endif

#endif /* __PWM_CTRL_H */
