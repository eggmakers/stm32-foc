/************************************************
���ļ���Ҫ����CSDN����loop222��SimpleFOC��Ŀ����ֲ
ԭ�̳̣�https://blog.csdn.net/loop222/article/details/120471390
����Ŀɾ���˲��ִ��룬������Flash���ݶ�ȡ�ͶԷ������ܵļ���
************************************************/

#include "BLDCmotor.h"
#include "FOCMotor.h"
#include "foc_utils.h"
#include "gpio.h"
#include <stdio.h>
#include "MagneticSensor.h"
#include "tim.h"
#include "FlashStorage.h"

#define M1_Enable HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define M1_Disable HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define PWM_Period 1280

extern float target;

long sensor_direction;
float voltage_power_supply;
float voltage_limit;
float voltage_sensor_align;
int pole_pairs;
unsigned long open_loop_timestamp;
float velocity_limit;

int alignSensor(void);
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);
/******************************************************************************/
void Motor_init(void)
{
	printf("MOT: Init\r\n");

	if (voltage_sensor_align > voltage_limit)
		voltage_sensor_align = voltage_limit;

	pole_pairs = 11;
	sensor_direction = UNKNOWN;
	M1_Enable;
	printf("MOT: Enable driver.\r\n");
}
/******************************************************************************/
void Motor_initFOC(void)
{
	if (Flash_ReadMotorParam(&pole_pairs, &zero_electric_angle, (int *)&sensor_direction) == -1) // ���Զ�ȡFlash����
	{
		if (alignSensor())															 // ��ȡʶ������е��У׼��������ƫ�����ͼ�����
			Flash_SaveMotorParam(pole_pairs, zero_electric_angle, sensor_direction); // У׼���ݴ���Flash
	}

	// added the shaft_angle update
	angle_prev = getAngle(); // getVelocity(),make sure velocity=0 after power on
	HAL_Delay(5);
	shaft_angle = shaftAngle(); // shaft angle

	HAL_Delay(200);
}
/******************************************************************************/
int alignSensor(void)
{
	long i;
	float angle;
	float mid_angle, end_angle;
	float moved;

	printf("MOT: Align sensor.\r\n");

	// find natural direction
	// move one electrical revolution forward
	for (i = 0; i <= 500; i++)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(voltage_sensor_align, 0, angle);
		HAL_Delay(2);
	}
	mid_angle = getAngle();

	for (i = 500; i >= 0; i--)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(voltage_sensor_align, 0, angle);
		HAL_Delay(2);
	}
	end_angle = getAngle();
	setPhaseVoltage(0, 0, 0);
	HAL_Delay(200);

	printf("mid_angle=%.4f\r\n", mid_angle);
	printf("end_angle=%.4f\r\n", end_angle);

	moved = fabs(mid_angle - end_angle);
	if ((mid_angle == end_angle) || (moved < 0.02)) // ��Ȼ��߼���û�ж�
	{
		printf("MOT: Failed to notice movement loop222.\r\n");
		M1_Disable; // �����ⲻ�������ر�����
		return 0;
	}
	else if (mid_angle < end_angle)
	{
		printf("MOT: sensor_direction==CCW\r\n");
		sensor_direction = CCW;
	}
	else
	{
		printf("MOT: sensor_direction==CW\r\n");
		sensor_direction = CW;
	}

	printf("MOT: PP check: ");				   // ����Pole_Pairs
	if (fabs(moved * pole_pairs - _2PI) > 0.5) // 0.5 is arbitrary number it can be lower or higher!
	{
		printf("fail - estimated pp:");
		pole_pairs = _2PI / moved + 0.5; // ������ת���Σ���������
		printf("%d\r\n", pole_pairs);
	}
	else
		printf("OK!\r\n");

	setPhaseVoltage(voltage_sensor_align, 0, _3PI_2); // �������ƫ�ƽǶ�
	HAL_Delay(700);
	zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction * getAngle(), pole_pairs));
	HAL_Delay(20);
	printf("MOT: Zero elec. angle:");
	printf("%.4f\r\n", zero_electric_angle);

	setPhaseVoltage(0, 0, 0);
	HAL_Delay(200);

	return 1;
}
/******************************************************************************/
void loopFOC(void)
{
	if (controller == Type_angle_openloop || controller == Type_velocity_openloop)
		return;

	shaft_angle = shaftAngle();			  // shaft angle
	electrical_angle = electricalAngle(); // electrical angle - need shaftAngle to be called first

	switch (torque_controller)
	{
	case Type_voltage: // no need to do anything really
		break;
	case Type_dc_current:
		break;
	case Type_foc_current:
		break;
	default:
		printf("MOT: no torque control selected!");
		break;
	}
	// set the phase voltage - FOC heart function :)
	extern uint8_t beepPlaying;
	if (beepPlaying == 0) // û�ڷ���״̬�����������ѹ
		setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}
/******************************************************************************/
void setTargetVotage(float new_target)
{
	voltage.q = new_target;
}
/******************************************************************************/
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Uout;
	uint32_t sector;
	float T0, T1, T2;
	float Ta, Tb, Tc;

	if (Ud) // only if Ud and Uq set
	{		// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	}
	else
	{ // only Uq available - no need for atan2 and sqrt
		Uout = Uq / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	if (Uout > 0.577)
		Uout = 0.577;
	if (Uout < -0.577)
		Uout = -0.577;

	sector = (angle_el / _PI_3) + 1;
	T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
	T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uout;
	T0 = 1 - T1 - T2;

	// calculate the duty cycles(times)
	switch (sector)
	{
	case 1:
		Ta = T1 + T2 + T0 / 2;
		Tb = T2 + T0 / 2;
		Tc = T0 / 2;
		break;
	case 2:
		Ta = T1 + T0 / 2;
		Tb = T1 + T2 + T0 / 2;
		Tc = T0 / 2;
		break;
	case 3:
		Ta = T0 / 2;
		Tb = T1 + T2 + T0 / 2;
		Tc = T2 + T0 / 2;
		break;
	case 4:
		Ta = T0 / 2;
		Tb = T1 + T0 / 2;
		Tc = T1 + T2 + T0 / 2;
		break;
	case 5:
		Ta = T2 + T0 / 2;
		Tb = T0 / 2;
		Tc = T1 + T2 + T0 / 2;
		break;
	case 6:
		Ta = T1 + T2 + T0 / 2;
		Tb = T0 / 2;
		Tc = T1 + T0 / 2;
		break;
	default: // possible error state
		Ta = 0;
		Tb = 0;
		Tc = 0;
	}

	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, Ta * PWM_Period);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, Tb * PWM_Period);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, Tc * PWM_Period);
}
