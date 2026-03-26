/* ================================================================
 * ADC_FUNCTION.c
 *
 * ADC 采样 / 滤波 / 物理量换算模块
 *
 * 架构概述：
 *   STM32F334 双 ADC（ADC1/ADC2）均由 HRTIM 触发，200kHz 采样率。
 *   DMA 中断已禁用，主循环轮询 DMA1->ISR TC 标志（ADC_Process）。
 *   每累积 8 组原始码值做均值 + 一阶 IIR 滤波，输出 25kHz 物理量。
 *
 * 数据流：
 *   HRTIM触发 → ADC转换 → DMA搬运到 adc1_raw_data/adc2_raw_data
 *     → ADC_Process 检测TC标志 → ADCF_PushSample 写入环形缓冲
 *     → ADC_Average 均值+IIR → 物理量换算 → RE_xxx 全局变量
 *
 * 调用顺序（main.c USER CODE 2）：
 *   ADCF_Init() → ADCF_Start()
 *   while(1) { ADC_Process(); }
 * ================================================================ */
#include "ADC_FUNCTION.h"
#include "adc.h"    /* hadc1, hadc2 句柄 */
#include "dma.h"    /* DMA1 寄存器基地址 */

volatile ADCF_State_t g_adcf;        /* ADC 采样/滤波状态机 */

/*
 * ADCF_Init
 *   清零全部状态，上电/复位后、ADCF_Start 前调用一次
 */
void ADCF_Init(void)
{
    uint8_t i, j;
    for (j = 0U; j < ADCF_AVG_COUNT; j++)
        for (i = 0U; i < ADCF_CH_COUNT; i++)
            g_adcf.buf[j][i] = 0U;
    for (i = 0U; i < ADCF_CH_COUNT; i++)
    {
        g_adcf.out[i] = 0.0f;
    }
    g_adcf.cnt   = 0U;
    g_adcf.ready = 0U;
}

/*
 * ADCF_PushSample（内部使用，由 ADC_Process 调用）
 *   将一组 6 路原始码值写入环形缓冲 buf[][]
 *   只做数据搬运，无浮点运算，每写满 ADCF_AVG_COUNT 组置 ready=1
 */
void ADCF_PushSample(const uint16_t raw[ADCF_CH_COUNT])
{
    uint8_t i;
    uint8_t idx = g_adcf.cnt;

    for (i = 0U; i < ADCF_CH_COUNT; i++)
        g_adcf.buf[idx][i] = raw[i];

    idx++;
    if (idx >= ADCF_AVG_COUNT)
    {
        idx          = 0;
        g_adcf.ready = 1;   // 置标志位，通知 Process：8 组已满，可以处理
    }
    g_adcf.cnt = idx;
}

/*
 * ADC_Average（内部使用，由 ADC_Process 调用）
 *   对环形缓冲中 8 组数据做算术均值，结果直接写入 g_adcf.out[]
 *   （已移除一阶 IIR，均值滤波已足够，且延迟更低）
 *   单位：原始 ADC 码值（0 ~ 4095）
 */
void ADC_Average(void)
{
    uint8_t i, j;
    for (i = 0U; i < ADCF_CH_COUNT; i++)
    {
        uint32_t sum = 0U;
        for (j = 0U; j < ADCF_AVG_COUNT; j++)
            sum += (uint32_t)g_adcf.buf[j][i];

        g_adcf.out[i] = (float)sum * (1.0f / (float)ADCF_AVG_COUNT);
    }
}

/*
 * ADCF_Start（main.c USER CODE 2 调用，仅调用一次）
 *   1. 对 ADC1/ADC2 执行硬件自校准
 *   2. 以 DMA 循环模式启动双 ADC 连续采集
 *   3. 禁用 DMA1_Ch1/Ch2 中断，改由主循环轮询 TC 标志
 */
void ADCF_Start(void)
{
    extern uint16_t adc1_raw_data[3];   // 原数据缓冲区，并且在数据处理函数里面直接处理
    extern uint16_t adc2_raw_data[3];   

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) { Error_Handler(); }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) { Error_Handler(); }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_raw_data, 3)   != HAL_OK) { Error_Handler(); }
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_raw_data, 3)   != HAL_OK) { Error_Handler(); }

    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);   // 禁 IRQ，转轮询
    HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
}
/*
 * 物理量换算参数，公式就这么给的，就是这么算的，就这个超电上能用
 *
 * 电压通道（两点线性拟合，y = k*adc + b）：
 *   V_CHASSIS: 标定点 (224, 1.85V) (2917, 24.00V) → k=0.008225 b=0.006
 *   V_CAP:     标定点 (225, 1.86V) (1424, 11.70V) → k=0.008207 b=0.014
 *
 * 电流通道（INA241，GAIN=20，RSHUNT=0.002Ω，VZERO=VREF/2）：
 *   I = (adc / 4095 * 3.3 - 1.65) / 0.04    单位：A
 */
#define POLL_K_V_CHASSIS   0.008225f
#define POLL_B_V_CHASSIS   0.006f
#define POLL_K_V_CAP       0.008207f
#define POLL_B_V_CAP       0.014f

#define POLL_ADC_MAX       4095.0f   // 12位ADC满量程
#define POLL_VREF          3.3f      // ADC参考电压 (V)
#define POLL_VZERO         1.65f     // 零电流对应输出电压 = VREF/2 (V)
#define POLL_SENS          0.04f     // INA241灵敏度 (V/A)

volatile float RE_V_CHASSIS;   // 底盘电压
volatile float RE_V_CAP;       // 电容电压
volatile float RE_I_REFEREE;   // 裁判系统电流
volatile float RE_I_CHASSIS;   // 底盘电流
volatile float RE_CAP_I;       // 电容电流
volatile float TEMP;           // 温度通道
/*
 * ADC_Process
 *   轮询 DMA1->ISR TC 标志，双 ADC 均完成后才处理：
 *     1. 清 TC 标志
 *     2. 快照原始码值到 raw[]
 *     3. 推入环形缓冲（ADCF_PushSample）
 *     4. 满8组则执行均值+IIR（ADC_Average）
 *     5. 将 g_adcf.out[] 换算为物理量，写入 RE_xxx / TEMP
 *   若任一 ADC 尚未完成本轮 DMA，直接 return 等待下次调用
 *   直接读取了寄存器，因此没有在dma.c里面使用库函数搬运数据
 */
uint8_t ADC_Process(void)
{
    extern uint16_t adc1_raw_data[3];
    extern uint16_t adc2_raw_data[3];

    /* --- 检查 DMA 传输完成标志 --- */
    uint8_t adc1_done = (DMA1->ISR & DMA_ISR_TCIF1) ? 1U : 0U;
    uint8_t adc2_done = (DMA1->ISR & DMA_ISR_TCIF2) ? 1U : 0U;

    if (!adc1_done) { return 0; }   /* ADC1 未就绪，下次再试 */

    /* --- 清 ADC1 TC 标志，快照 ADC1 数据 --- */
    DMA1->IFCR = DMA_IFCR_CTCIF1;
    uint16_t raw[ADCF_CH_COUNT];
    raw[ADCF_CH_V_CHASSIS] = adc1_raw_data[0];
    raw[ADCF_CH_I_REFEREE] = adc1_raw_data[1];
    raw[ADCF_CH_I_A]       = adc1_raw_data[2];

    if (!adc2_done) { return 0; }   /* ADC2 未就绪，下次再试 */

    /* --- 清 ADC2 TC 标志，快照 ADC2 数据 --- */
    DMA1->IFCR = DMA_IFCR_CTCIF2;
    raw[ADCF_CH_CAP_I] = adc2_raw_data[0];
    raw[ADCF_CH_TEMP]  = adc2_raw_data[1];
    raw[ADCF_CH_V_CAP] = adc2_raw_data[2];

    /* --- 推入环形缓冲 --- */
    ADCF_PushSample(raw);

    if (!g_adcf.ready) { return 0; }   /* 未满8组，继续积累 */

    /* --- 8组已满：均值 + IIR 滤波 --- */
    g_adcf.ready = 0U;
    ADC_Average();

    /* --- 物理量换算 --- */
    TEMP         = g_adcf.out[ADCF_CH_TEMP];

    RE_V_CHASSIS = POLL_K_V_CHASSIS * g_adcf.out[ADCF_CH_V_CHASSIS] + POLL_B_V_CHASSIS;
    RE_V_CAP     = POLL_K_V_CAP     * g_adcf.out[ADCF_CH_V_CAP]     + POLL_B_V_CAP;

    RE_I_REFEREE = (g_adcf.out[ADCF_CH_I_REFEREE] / POLL_ADC_MAX * POLL_VREF - POLL_VZERO) / POLL_SENS;
    RE_I_CHASSIS = (g_adcf.out[ADCF_CH_I_A]       / POLL_ADC_MAX * POLL_VREF - POLL_VZERO) / POLL_SENS;
    RE_CAP_I     = (g_adcf.out[ADCF_CH_CAP_I]     / POLL_ADC_MAX * POLL_VREF - POLL_VZERO) / POLL_SENS;

    return 1;
}
