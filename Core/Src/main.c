/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "hrtim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADC_FUNCTION.h"
#include "PWM_ctrl.h"
#include "CAN_bsp.h"
#include "pid.h"
#include "cap_charge.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define TIMERA_PERIOD   360U     //pwm周期tick值

#define CHARGE_CURRENT 0.5f      //3.0f
#define OUTPUT_VOLTAGE 24.0f	   //两个是pid目标，对应充放电的时候

#define CTRL_PWM_START_TICKS   (2U)  // 可按示波器观测调整：1~10 都可以，防止0u的时候打不开pwm输出，加上偏移

extern uint16_t adc1_raw_data[3];   // ADC1: [0]=V_CHASSIS [1]=I_REFEREE [2]=I_CHASSIS  //裁判系统视为恒压源24V
extern uint16_t adc2_raw_data[3];   // ADC2: [0]=CAP_I     [1]=TEMP      [2]=V_CAP      //温度用不到，不管

extern volatile float RE_V_CHASSIS;   // 底盘电压
extern volatile float RE_V_CAP;       // 电容电压
extern volatile float RE_I_REFEREE;   // 裁判系统电流
extern volatile float RE_I_CHASSIS;   // 底盘电流
extern volatile float RE_CAP_I;       // 电容电流
extern volatile float TEMP;           // 温度通道

volatile unsigned long g_dbg_main_heartbeat = 0;
volatile unsigned long g_dbg_hrtim_irq_seen = 0;
volatile unsigned char g_dbg_pwm_outputs_enabled = 0;
volatile unsigned long g_dbg_hrtim_master_cnt = 0;

extern CAN_HandleTypeDef hcan;
extern HRTIM_HandleTypeDef hhrtim1;

PID_TypeDef pid_charge;      // 充电恒流控制
PID_TypeDef pid_discharge;   // 放电恒压控制
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();
  
  ADCF_Init();
  ADCF_Start();

  CAP_Init();// 初始化安全值

  PID_InitAll();	

  HRTIM_UpdateHighDuty(0.0f, 0.0f);

  if (HAL_HRTIM_WaveformCounterStart_IT(&hhrtim1,
      HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
		PID_InitAll();	
		
		while(1)
		{
			ADC_Process();	//第一个循环里面不断处理数据（dma是不会停的）
			
			//这一行写数据侦测，也就是保护
			if((RE_V_CAP >= 26.0f) || (RE_CAP_I >= 25.0f) || (RE_CAP_I <= -25.0f))
			{
				//锁存，本次放弃使用超电
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | 
													   HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
				Error_Handler();
			}
		
			//这一行写pid控制恒定电流充电
			HRTIM_UpdateHighDuty(0.0f, 0.0f);//先保护一手
			
			// while(RE_V_CAP <= 1.8f)
			// {
			// 	ADC_Process();
			// 	HRTIM_UpdateHighDuty(0.1f, 1.0f);
			// 	uint8_t send_data[16];
			// 	float cap_i = RE_CAP_I;
			// 	float cap_v = RE_V_CAP;
			// 	float duty = 0.1f;
			// 	uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
			// 	memcpy(&send_data[0], &duty, 4);
			// 	memcpy(&send_data[4], &cap_i, 4);
			// 	memcpy(&send_data[8], &cap_v, 4);
			// 	memcpy(&send_data[12], tail, 4);
			// 	HAL_UART_Transmit(&huart3, send_data, sizeof(send_data), 100);
			// }
			
			float pid_out = PID_Calculate(&pid_charge, -RE_CAP_I, CHARGE_CURRENT);
			float duty = PID_Output_To_Duty(&pid_charge, pid_out);

			float a_high, b_high;
			Charge_buck(duty, &a_high, &b_high);
			HRTIM_UpdateHighDuty(a_high, b_high);
			
			//这一行集中书写所有保护(真的要写吗)
			uint8_t send_data[16];
			float cap_i = RE_CAP_I;
			float cap_v = RE_V_CAP;
			uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
			memcpy(&send_data[0], &duty, 4);
			memcpy(&send_data[4], &cap_i, 4);
			memcpy(&send_data[8], &cap_v, 4);
			memcpy(&send_data[12], tail, 4);
			HAL_UART_Transmit(&huart3, send_data, sizeof(send_data), 100);
			
			//这一行数据侦测，但是是电容组的电压，到24v停止充电，break进入轮询（等待冲刺模式下发）
			if (RE_V_CAP >= 24.0f)
			{
				break;
			}
			HAL_Delay(1);
		}
		
		//等待can的消息控制
		
		PID_InitAll();	
		
		while(1)
		{
//			ADC_Process();	//第二个循环里面不断处理数据
//			
//			//这一行写数据侦测，给pid
//			if((RE_V_CAP >= 26.0f) || (RE_CAP_I >= 25.0f)  || (RE_CAP_I <= -25.0f))
//			{
//				//锁存，本次放弃使用超电
//				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | 
//													   HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
//				Error_Handler();
//			}
//			
//			//这一行写pid控制恒定电压输出，boost模式，把电容组小于24v的电压升到24v输出
//			float pid_out = -PID_Calculate(&pid_discharge, RE_V_CAP, OUTPUT_VOLTAGE);
//			float duty = PID_Output_To_Duty(&pid_discharge, pid_out);

//			float a_high, b_high;
//			Output_boost(duty, &a_high, &b_high);
//			HRTIM_UpdateHighDuty(a_high, b_high);
//			
//			//数据侦测，电容组电压不得低于12v，否则会过流烧毁mos，恒定功率时，电压减少，电流增大			
//			//如果cap小于等于12v，break，放电结束
//			if (RE_V_CAP <= 12.0f)
//			{
//				break;
//			}
HAL_Delay(10);
		}
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case 0x009:
		{
			
		}
		break;
    }
	
	ADC_Process();
	
	// 计算功率
    float chassis_power = RE_V_CHASSIS * RE_I_CHASSIS;      // 底盘功率 (W)
    float referee_power = 24.0f * RE_I_REFEREE;             // 裁判系统功率，按恒压24V计算
	
	uint8_t can_data[8] = {0};
	
	memcpy(can_data, &chassis_power, 4);
    memcpy(can_data + 4, &referee_power, 4);
	
	CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = 0x010;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;        // 标准帧
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
	
	uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(hcan, &tx_header, can_data, &tx_mailbox);
}

void PID_InitAll(void)
{
    // 充电：电流控制，目标电流 2.0A，反馈范围 0~3A
    // max_out 设为 1000，则 PID 输出范围 -1000~1000，映射后 duty = (out+1000)/2000
    PID_Init(&pid_charge,
             100.0f,      // kp
             0.0f,      // ki
             0.01f,     // kd
             1000,      // max_out   (绝对值)
             100,       // integral_limit (积分限幅)
             0.01f,     // deadband  (死区)
             10.0f,     // A (变积分系数A)
             5.0f,      // B (变积分系数B)
             0.9f,      // output_filtering_coefficient (输出滤波)
             0.8f,      // derivative_filtering_coefficient (微分滤波)
             Integral_Limit | Trapezoid_Intergral | OutputFilter);   // 开启的改进项

    // 放电：电压控制，目标电压 24.0V，反馈范围 0~30V
    PID_Init(&pid_discharge,
             0.2f,      // kp
             0.05f,     // ki
             0.005f,    // kd
             1000,      // max_out
             800,       // integral_limit
             0.2f,      // deadband
             10.0f,     // A
             5.0f,      // B
             0.9f,      // output_filtering_coefficient
             0.8f,      // derivative_filtering_coefficient
             Integral_Limit | Trapezoid_Intergral | OutputFilter);
}
/**
 * @brief 将 PID 输出（范围 -max_out ~ max_out）映射到占空比（0~1）
 * @param pid     PID 实例指针
 * @param output  PID_Calculate 的返回值
 * @return 占空比（0.0 ~ 1.0）
 */
static inline float PID_Output_To_Duty(PID_TypeDef *pid, float output)
{
    // 修改原先的半桥映射(0输出对应50%占空比)，改为单极Buck/Boost映射(0输出对应0%占空比)。
    float duty = output / pid->MaxOut;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    return duty;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
