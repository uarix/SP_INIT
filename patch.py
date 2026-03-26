with open('/workspaces/SP_INIT/Core/Src/main.c', 'r') as f:
    content = f.read()

import re

pattern = r"PID_InitAll\(\);\s*while\(1\)\s*\{\s*//\s*// 第二个循环：恒压输出放电.*?}\n"
replacement = """PID_InitAll();

\t\tuint32_t last_uart_tick_dis = 0;
\t\tstatic float current_duty_dis = 1.0f; // 初始占空比 1.0 (不起压，直通状态)

\t\twhile(1)
\t\t{
\t\t\t// 第二个循环：恒压输出放电，25kHz同步控制
\t\t\tif(ADC_Process())
\t\t\t{
\t\t\t\t// 依旧保留致命硬件级保护：
\t\t\t\tif((RE_V_CAP >= 26.0f) || (RE_CAP_I >= 25.0f)  || (RE_CAP_I <= -25.0f))
\t\t\t\t{
\t\t\t\t\t//锁存，本次放弃使用超电
\t\t\t\t\tHAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | 
\t\t\t\t\t\t\t\t\t\t\t\t\t\t   HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
\t\t\t\t\tError_Handler();
\t\t\t\t}
\t\t\t\t
\t\t\t\t// 业务保护：放电电容电压过低 或 电流绝对值超过15A
\t\t\t\tfloat abs_cap_i = (RE_CAP_I > 0) ? RE_CAP_I : -RE_CAP_I;
\t\t\t\tif (RE_V_CAP <= 10.0f || abs_cap_i >= 15.0f)
\t\t\t\t{
\t\t\t\t\t// 退出放电循环，回到重新充电阶段
\t\t\t\t\tbreak;
\t\t\t\t}
\t\t\t\t
\t\t\t\t// 这一行写pid控制恒定电压输出，boost模式。
\t\t\t\t// 目标是 OUTPUT_VOLTAGE (24V), 当前测量为 RE_V_CHASSIS
\t\t\t\tfloat pid_out = PID_Calculate(&pid_discharge, RE_V_CHASSIS, OUTPUT_VOLTAGE);
\t\t\t\t
\t\t\t\t// 根据 Boost 逻辑：B端占空比d越大，升压越小。最大升压幅度受到限制。
\t\t\t\t// 如果欠压了（Measure < Target），误差Err > 0，pid_out增大
\t\t\t\t// 我们需要它增大的时候，占空比d反而减小进而提升电压。
\t\t\t\tfloat duty_reduction = PID_Output_To_Duty(&pid_discharge, pid_out);
\t\t\t\tcurrent_duty_dis = 1.0f - duty_reduction;
\t\t\t\t
\t\t\t\t// 底线安全占空比限制：不要让它降得太低（例如低于0.2等同于极端的暴力升压，非常危险）
\t\t\t\tif(current_duty_dis < 0.2f) current_duty_dis = 0.2f;
\t\t\t\tif(current_duty_dis > 1.0f) current_duty_dis = 1.0f;

\t\t\t\tfloat a_high, b_high;
\t\t\t\tOutput_boost(current_duty_dis, &a_high, &b_high);
\t\t\t\tHRTIM_UpdateHighDuty(a_high, b_high);
\t\t\t}
\t\t\t
\t\t\t// 非阻塞定时发送串口数据
\t\t\tuint32_t current_tick = HAL_GetTick();
\t\t\tif (current_tick - last_uart_tick_dis >= 10)
\t\t\t{
\t\t\t\tlast_uart_tick_dis = current_tick;
\t\t\t\t
\t\t\t\tfloat cap_i = RE_CAP_I;
\t\t\t\t// 放电时主看恒压输出能力，传出 chassis_v
\t\t\t\tfloat chassis_v = RE_V_CHASSIS;
\t\t\t\tuint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

\t\t\t\tmemcpy(&send_data[0], &current_duty_dis, 4);
\t\t\t\tmemcpy(&send_data[4], &cap_i, 4);
\t\t\t\tmemcpy(&send_data[8], &chassis_v, 4);
\t\t\t\tmemcpy(&send_data[12], tail, 4);
\t\t\t\t
\t\t\t\tif(huart3.gState == HAL_UART_STATE_READY)
\t\t\t\t{
\t\t\t\t\tHAL_UART_Transmit_IT(&huart3, send_data, sizeof(send_data));
\t\t\t\t}
\t\t\t}
\t\t}
"""
import sys
res = re.sub(pattern, replacement, content, flags=re.DOTALL)
if res == content:
    print("Replace failed")
    sys.exit(1)
with open('/workspaces/SP_INIT/Core/Src/main.c', 'w') as f:
    f.write(res)
print("Replace success")
