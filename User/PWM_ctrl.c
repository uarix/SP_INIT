#include "stm32f3xx_hal.h"

#include "main.h"
#include "hrtim.h"
#include "adc.h"
#include "gpio.h"
#include "stm32f3xx_hal_hrtim.h"
#include "ADC_FUNCTION.h"

/* 
 * 兜底宏：部分工程/索引器环境下可能无法解析 HRTIM_TIMERINDEX_* / HRTIM_COMPAREUNIT_*。
 * 这里仅在未定义时补齐，避免再次退回“裸常量”导致通道/比较单元写错。
 * 数值取自本工程此前可工作的写法：
 *   TIMER_A=0x0, TIMER_B=0x1, CMP1=0x1, CMP3=0x4
 */
#ifndef HRTIM_TIMERINDEX_TIMER_A
#define HRTIM_TIMERINDEX_TIMER_A   (0x0U)
#endif
#ifndef HRTIM_TIMERINDEX_TIMER_B
#define HRTIM_TIMERINDEX_TIMER_B   (0x1U)
#endif
#ifndef HRTIM_COMPAREUNIT_1
#define HRTIM_COMPAREUNIT_1        (0x00000001U)
#endif
#ifndef HRTIM_COMPAREUNIT_3
#define HRTIM_COMPAREUNIT_3        (0x00000004U)
#endif


//工作流程：buck 获得当前电流，小于目标，送进pid，加占空比，输出duty（可能需要归一化），
//送给Charge_buck，实际上只管a，得到a_high,送到HRTIM_UpdateHighDuty
//			boost 获得当前电容输出电压，小于24，送进pid，取负，减占空比
//送给Output_boost，实际上只管b，得到b_high，送到HRTIM_UpdateHighDuty


// 设置占空比，转换为比较事件数值，负责a上b上开关占空比
void HRTIM_UpdateHighDuty(float a_high, float b_high)//param：占空比
{
    /* 输入限幅，防止负数转 uint32_t 下溢 */
    if (a_high < 0.0f) a_high = 0.0f;  if (a_high > 1.0f) a_high = 1.0f;
    if (b_high < 0.0f) b_high = 0.0f;  if (b_high > 1.0f) b_high = 1.0f;

    const uint32_t period = TIMERA_PERIOD;
    const uint32_t t0 = (CTRL_PWM_START_TICKS < (period - 1U)) ? CTRL_PWM_START_TICKS : 1U;

    uint32_t ton_a = (uint32_t)(a_high * (float)period);
    uint32_t ton_b = (uint32_t)(b_high * (float)period);

    /* Ton 限幅：0..period-1 */
    if (ton_a >= period) ton_a = period - 1U;
    if (ton_b >= period) ton_b = period - 1U;

    /* 置位点：CMP1 = t0（非 0） */
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, t0);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, t0);

    /* 复位点：CMP3 = t0 + Ton
     * - Ton=0 时，让 CMP3==CMP1（不产生有效高电平脉冲，等价全关）
     * - 避免越界：最大 period-1
     */
    uint32_t cmp3_a = t0 + ton_a;
    uint32_t cmp3_b = t0 + ton_b;
    if (cmp3_a >= period) cmp3_a = period - 1U;
    if (cmp3_b >= period) cmp3_b = period - 1U;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, cmp3_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, cmp3_b);
}

/* duty 限幅工具 */
static inline float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

void Charge_buck(float duty ,float *a_high ,float *b_high)
{
	float d = clamp01(duty);
	
	float a = 0;
	float b = 1;//b_high常开
	
	a = d;
	
	if(a_high != NULL && b_high != NULL)
	{
		*a_high = a;
		*b_high = b;
	}
	else
	{
		*a_high = 0;
		*b_high = 0;
	}
}

void Output_boost(float duty ,float *a_high ,float *b_high)
{
	float d = clamp01(duty);
	
	float a = 1;//a_high常开
	float b = 0;
	
	b = d;
	
	if(a_high != NULL && b_high != NULL)
   {
    	*a_high = a;
    	*b_high = b;
    }
    else
    {
    	*a_high = 0;
    	*b_high = 0;
    }
}
