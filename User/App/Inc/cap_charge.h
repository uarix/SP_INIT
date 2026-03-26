#ifndef CAP_CHARGE__H
#define CAP_CHARGE__H

#include <stdint.h>

typedef struct {
    float v_cap_max;      // 电容电压上限 (V)
    float i_charge_ref;   // 充电目标电流 (A)，正值代表充电
    float v_cap_min;      // 允许充电的最低输入电压 (V)
} CapChargeConfig_t;

void CAP_Init(void);

/* 应用层 25kHz Tick：建议由 scheduler 在同一个 HRTIM 节拍中调用（可选） */
void CAP_Tick(void);

/* 控制命令：由上层（遥控/裁判/策略）设置 */
void CAP_SetChargeCurrent(float i_ref);
void CAP_SetCapVoltageMax(float v_max);

#endif
