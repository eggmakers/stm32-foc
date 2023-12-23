/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "MagneticSensor.h"
#include "FOCMotor.h"
#include "BLDCmotor.h"
#include "FlashStorage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// ���ڷ���������
enum
{
	// ����
	T_None = 0,
	// �Ͱ˶�
	T_L1 = 3822,
	T_L2 = 3405,
	T_L3 = 3034,
	T_L4 = 2863,
	T_L5 = 2551,
	T_L6 = 2272,
	T_L7 = 2052,
	// �а˶�
	T_M1 = 1911,
	T_M2 = 1703,
	T_M3 = 1517,
	T_M4 = 1432,
	T_M5 = 1276,
	T_M6 = 1136,
	T_M7 = 1012,
	// �߰˶�
	T_H1 = 956,
	T_H2 = 851,
	T_H3 = 758,
	T_H4 = 716,
	T_H5 = 638,
	T_H6 = 568,
	T_H7 = 506
};

// �������˲��ṹ��
typedef struct
{
	float X_last;
	float X_mid;
	float X_now;
	float P_mid;
	float P_now;
	float P_last;
	float kg;
	float A;
	float Q;
	float R;
	float H;
} Kalman;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VOLT_SUPPLY 12 // ����ĸ�ߵ�ѹ
#define MAX_VOLT 6.9f	   // ���ƹ����ѹ(ƽ��ֵ)�����ΪVOLT_SUPPLY/��3; 4010���:6.9V, 2804���:4V
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float targetVotage = 0;	   // ��ǰĿ���ѹ
uint8_t beepPlaying = 0;   // ��ǰ�Ƿ��ڷ���״̬
uint8_t motorID = 1;	   // ��ʼ���ID
uint8_t ledBlink = 1;	   // led�Ƿ���������˸ģʽ
float speed = 0;		   // ������������������ת��
float filteredAngle = 0;   // ���˲��õ���ת�ӽǶ�
uint32_t lastRecvTime = 0; // �ϴ��յ�CAN����֡��ʱ��
Kalman angleFilter;		   // �������˲��ṹ��
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f) { return ch; }

// ��Ƭ�������λ
void System_Reset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

/*************�������˲�*************/

void Kalman_Init(Kalman *p, float T_Q, float T_R)
{
	p->X_last = (float)0;
	p->P_last = 0;
	p->Q = T_Q;
	p->R = T_R;
	p->A = 1;
	p->H = 1;
	p->X_mid = p->X_last;
}

float Kalman_Filter(Kalman *p, float dat)
{
	if (!p)
		return 0;
	p->X_mid = p->A * p->X_last;					// x(k|k-1) = AX(k-1|k-1)+BU(k)
	p->P_mid = p->A * p->P_last + p->Q;				// p(k|k-1) = Ap(k-1|k-1)A'+Q
	p->kg = p->P_mid / (p->P_mid + p->R);			// kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
	p->X_now = p->X_mid + p->kg * (dat - p->X_mid); // x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
	p->P_now = (1 - p->kg) * p->P_mid;				// p(k|k) = (I-kg(k)H)P(k|k-1)
	p->P_last = p->P_now;
	p->X_last = p->X_now;
	return p->X_now;
}

/*************CANͨ��*************/

// CAN�����ʼ��
void CAN_Init()
{
	CAN_FilterTypeDef filter;
	filter.FilterActivation = ENABLE;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterBank = 0;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan, &filter);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// ����һ����������֡
void CAN_SendState(float angle, float speed)
{
	CAN_TxHeaderTypeDef header;
	header.StdId = motorID + 0x100;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = 8;

	uint8_t data[8];
	memcpy(data, &(int32_t){angle * 1000}, 4);	 // �Ƕ����ݷ���ǰ�ĸ��ֽ�
	memcpy(&data[4], &(int16_t){speed * 10}, 2); // ת�����ݷ��ڵ�5-6�ֽ�

	uint32_t mailbox;
	HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
}

// CAN�յ����ݵ��жϻص�
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef header;
	uint8_t rxData[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rxData) != HAL_OK)
		return;

	if (header.StdId == 0x100 && motorID <= 4) // ID=1~4����0x100����֡
	{
		targetVotage = *(int16_t *)&rxData[(motorID - 1) * 2] / 1000.0f;
		lastRecvTime = HAL_GetTick();
	}
	else if (header.StdId == 0x200 && motorID > 4) // ID=5~8����0x200����֡
	{
		targetVotage = *(int16_t *)&rxData[(motorID - 5) * 2] / 1000.0f;
		lastRecvTime = HAL_GetTick();
	}
}

/*************FOC*************/

void SimpleFOC_Init()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // ��������PWMͨ�����
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	MagneticSensor_Init(); // ��ʼ���Ŵ�����

	voltage_power_supply = VOLT_SUPPLY; // �趨FOC�������
	voltage_limit = MAX_VOLT;
	voltage_sensor_align = voltage_limit;
	targetVotage = 0;

	Motor_init();	 // ��ʼ�������Ϣ
	Motor_initFOC(); // ��ʼ��FOC����

	motorID = Flash_ReadMotorID(); // ��Flash��ȡ���ID
	motorID = motorID ? motorID : 1;
}

/*************����*************/

// ���ݷ����������ò�������ʱ��
void Beep_Play(uint16_t period)
{
	if (period != 0)
	{
		__HAL_TIM_SetAutoreload(&htim1, period / 2);
		HAL_TIM_Base_Start_IT(&htim1);
		beepPlaying = 1;
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim1);
		beepPlaying = 0;
	}
}

// ����һ������
void Beep_PlayNotes(uint8_t num, uint16_t notes[][2])
{
	for (uint8_t i = 0; i < num; i++)
	{
		Beep_Play(notes[i][0]);
		HAL_Delay(notes[i][1]);
	}
	Beep_Play(0);
}

// �����жϴ����ڶ�ʱ���жϻص��е���
void Beep_IRQHandler()
{
	static uint8_t flipFlag = 0;
	if (targetVotage == 0)
	{
		setPhaseVoltage(voltage_limit / 2, 0, _PI / 3 * flipFlag); // ʹ�ų�������0-PI/3����
		flipFlag = !flipFlag;
	}
}

/*************������ʱ����*************/

// ��������������
void Key_Process()
{
	static uint32_t downTime = 0;
	static uint8_t lastKeyState = 0;
	uint8_t keyState = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1 : 0;
	if (keyState && !lastKeyState) // ����
	{
		downTime = HAL_GetTick();
		ledBlink = 0;
	}
	else if (keyState && lastKeyState) // ��ס
	{
		uint32_t pressTime = HAL_GetTick() - downTime;
		if (pressTime < 500 * 8) // ��8�£�ÿ��500ms
		{
			if (pressTime % 500 < 100)
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		else if (pressTime < 500 * 12)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ��8��֮����
	}
	else if (!keyState && lastKeyState) // �ɿ�
	{
		uint32_t pressTime = HAL_GetTick() - downTime;
		if (pressTime > 50 && pressTime < 500 * 8) // ��8�������ɿ�������ID
		{
			motorID = pressTime / 500 + 1;
			Flash_SaveMotorID(motorID);
		}
		else if (pressTime >= 500 * 8 && pressTime < 500 * 12) // ��8�º��ɿ������Flash��λ����У׼
		{
			Flash_EraseMotorParam();
			System_Reset();
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		ledBlink = 1;
	}
	lastKeyState = keyState;
}

// ������LED��ʱ������˸������ʾ���ID
void Led_Process()
{
	if (!ledBlink)
		return;
	uint32_t period = 1000 + (100 + 200) * motorID;
	uint32_t mod = HAL_GetTick() % period;
	if (mod < (100 + 200) * motorID && mod % (100 + 200) < 100)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

// ���ת�ټ���
void Motor_SpeedCalcProcess()
{
	const uint8_t speedLpfLen = 5;
	static float speedLpfBuf[speedLpfLen] = {0}; // ���5��filteredAngle

	float angle = filteredAngle;
	float curSpeed = (angle - speedLpfBuf[0]) * 1000 / speedLpfLen / _2PI * 60;
	speed = curSpeed;

	for (uint8_t i = 0; i < speedLpfLen - 1; i++)
		speedLpfBuf[i] = speedLpfBuf[i + 1];
	speedLpfBuf[speedLpfLen - 1] = angle;
}

// CAN���߼�⣬500msû�յ�CAN�ź���ͣ��
void Motor_OfflineCheckProcess()
{
	static uint8_t isOffline = 0;
	if (HAL_GetTick() - lastRecvTime > 500)
	{
		if (!isOffline)
		{
			targetVotage = 0;
			isOffline = 1;
		}
	}
	else
		isOffline = 0;
}

// �ڵδ�ʱ���е��ã�1ms���ڣ��������ʱ����
void SysTick_UserExec()
{
	Key_Process();
	Led_Process();
	Motor_SpeedCalcProcess();
	Motor_OfflineCheckProcess();
	if (HAL_GetTick() % 2)
		CAN_SendState(shaft_angle, speed);
	filteredAngle = Kalman_Filter(&angleFilter, shaft_angle);
}
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
	MX_CAN_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	CAN_Init();																   // CAN�����ʼ��
	SimpleFOC_Init();														   // FOC��ʼ��
	Beep_PlayNotes(3, (uint16_t[][2]){{T_H1, 200}, {T_H3, 200}, {T_H5, 500}}); // ���ſ�����Ч

	Kalman_Init(&angleFilter, 0.0005f, 0.1f); // �趨�������˲�ϵ��

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		setTargetVotage(6); // �趨FOC��ѹ
		loopFOC();			// ����FOC�㷨
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

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
