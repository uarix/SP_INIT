#ifndef __ADC_FUNCTION_H__
#define __ADC_FUNCTION_H__

#include "main.h"

/*
 * 通道映射
 *   ADC1 → DMA1_Ch1:  [0] V_CHASSIS  [1] I_REFEREE  [2] I_A
 *   ADC2 → DMA1_Ch2:  [3] CAP_I      [4] TEMP        [5] V_CAP
 *   采样率：200kHz 原始 → 每8组平均 → 25kHz 输出
 */
#define ADCF_CH_COUNT       6U
#define ADCF_AVG_COUNT      8U
#define ADCF_CH_V_CHASSIS   0U   /* 底盘电压，PA0，91k+10k 分压 */
#define ADCF_CH_I_REFEREE   1U   /* 裁判系统电流，PA1，INA241 */
#define ADCF_CH_I_A         2U   /* A侧电流，PA2，INA241 */
#define ADCF_CH_CAP_I       3U   /* 电容电流，PC1，INA241 */
#define ADCF_CH_TEMP        4U   /* 温度，PC2，待换算 */
#define ADCF_CH_V_CAP       5U   /* 电容电压，PC3，分压 */

/*
 * 采样状态结构体
 *   buf[][]  : 环形缓冲，存最近 ADCF_AVG_COUNT 组原始 ADC 码值
 *   cnt      : 当前已写入组数（0 ~ AVG_COUNT-1）
 *   ready    : 满8组后由 PushSample 置1，Poll 处理完后清0
 *   out[]    : 最终输出，单位：原始 ADC 码值（0 ~ 4095），
 *              物理量换算在 ADC_Process 中完成，结果存全局变量
 */
typedef struct
{
    uint16_t         buf[ADCF_AVG_COUNT][ADCF_CH_COUNT];
    volatile uint8_t cnt;
    volatile uint8_t ready;
    float            out[ADCF_CH_COUNT];
} ADCF_State_t;

/*
 * 全局实例（定义于 ADC_FUNCTION.c）
 */
extern volatile ADCF_State_t g_adcf;

/*
 *   调用顺序（main.c）：
 *     1. ADCF_Init()      — 清零状态
 *     3. ADCF_Start()     — 校准 ADC + 启动 DMA + 禁 DMA IRQ
 *     4. while(1) 中持续调用 ADC_Process()
 */
void ADCF_Init(void);       // 清零全部状态，启动前必须调用一次
void ADCF_Start(void);      // 校准 ADC1/2，启动 DMA，禁用 DMA 中断
uint8_t ADC_Process(void);  // 主循环轮询入口：返回1表示新一次滤波和物理量换算完成，返回0表示未完成

// 以下两个函数供 ADC_Process 内部使用，外部一般不直接调用
void ADCF_PushSample(const uint16_t raw[ADCF_CH_COUNT]);  /* 写入一组原始码值到环形缓冲 */
void ADC_Average(void);    // 8组均值

/*
 * 物理量输出（定义于 ADC_FUNCTION.c）
 *   这些变量在 ADC_Process() 中按 25kHz 更新，供控制/应用层直接读取。
 */
extern volatile float RE_V_CHASSIS;   // 底盘电压
extern volatile float RE_V_CAP;       // 电容电压
extern volatile float RE_I_REFEREE;   // 裁判系统电流
extern volatile float RE_I_CHASSIS;   // 底盘电流
extern volatile float RE_CAP_I;       // 电容电流
extern volatile float TEMP;           // 温度通道

#endif /* __ADC_FUNCTION_H__ */
