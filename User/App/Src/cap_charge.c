#include "cap_charge.h"

#include <stdint.h>

#include "ADC_FUNCTION.h" /* RE_V_CAP, RE_V_CHASSIS, RE_CAP_I */

/* 硬阈值保护
 * 这些阈值用于“系统级安全兜底”，不依赖 PID
 */
#define CAP_HARD_OVERVOLT_V     (35.0f)    // 硬限压：超过立即故障
#define CAP_HARD_OVERCURR_A      (20.0f)   // 硬限流：|Icap| 超过立即故障

// 达到该电压后自动停止充电
#define CAP_AUTO_STOP_V 24.0f

static inline float cap_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}
//防出事
static inline uint8_t CAP_FaultDetected(float vcap, float icap)
{
    /* 过压硬保护（独立于配置的 v_cap_max） */
    if (vcap >= CAP_HARD_OVERVOLT_V) return 1;

    /* 过流硬保护 */
    if (cap_absf(icap) >= CAP_HARD_OVERCURR_A) return 1;

    return 0;
}

static volatile CapChargeConfig_t s_cfg = {
    .v_cap_max    = 24.0f,
    .i_charge_ref = 0.5f,
    .v_cap_min    = 0.0f,
};

// 目标输出（供 control 层读取）
static volatile uint8_t ctrl_flag = 0;
static volatile float   i_ref_out = 0.0f;//裁判系统输出给超电充电的参数，恒定电流pid
static volatile float   v_cap_out = 0.0f;//超电输出电压，恒定电压pid
// 用于刷新一下控制限幅，初始化
void CAP_Init(void)//包含了允许控制的指令
{
    v_cap_out = s_cfg.v_cap_max;
    i_ref_out = s_cfg.i_charge_ref;

    ctrl_flag = 1;//允许控制，是否允许运行控制环
}
// 不知道杨一写这个有什么意义，必须要重新执行init才能刷新到超电的pid参数
void CAP_SetChargeCurrent(float i_ref)
{
    s_cfg.i_charge_ref = i_ref;
}

void CAP_SetCapVoltageMax(float v_max)
{
    s_cfg.v_cap_max = v_max;
}
