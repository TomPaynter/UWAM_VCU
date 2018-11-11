// Inverter Settings
#define MAX_TORQUE 140

// CAN Receive IDs
#define PEDALBOX_ID 0x3C

// CAN Transmit IDs
#define INVERTER_ID 0x0C0
#define PDM_ID 0x28C

// NU_TALK_VALUE - debug please not to be released if present
#define NU_TALK_VALUE 0

// ****** End User Settings ************************************

extern "C" {
#include "main.h"
#include "stm32f0xx_hal.h"
#include "can.h"
//#include "adc.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
int __io_putchar(int ch) {
	uint8_t character = (uint8_t) ch;
	HAL_UART_Transmit(&huart1, &character, 1, 1000);
	return ch;
}
}

#include "Can_Bus.h"
#include "linear_scale.h"


void SystemClock_Config(void);

Can_Bus *Can_Bus::instance = 0;
bool stop = false;

int main(void) {

	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_CAN_Init();

	Can_Bus *can_bus = can_bus->getInstance();

	linear_scale torque_scale = linear_scale(0, 100, MAX_TORQUE, 0);

//	Set Safety Line High
	HAL_GPIO_WritePin(SAFETY_CONTROL_GPIO_Port, SAFETY_CONTROL_Pin,
			GPIO_PIN_SET);

	while (1) {
// 		Now in "Standby" State
//		Check if Brake Pressure is high enough
		bool bp_ok = (can_bus->get_brake_pressure() > 5);

//		Check Start Switch to go Low
		bool start_ok = (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin)
				== GPIO_PIN_RESET);

//		Check Safety Line to go High
		bool safety_ok = (HAL_GPIO_ReadPin(SAFETY_GPIO_Port, SAFETY_Pin)
				== GPIO_PIN_SET);

		if (bp_ok && start_ok && safety_ok) {
//			Now in "RTD Sound" State
// 			Sound RTD Horn
			HAL_GPIO_WritePin(RTD_HORN_GPIO_Port, RTD_HORN_Pin, GPIO_PIN_SET);
			HAL_Delay(2000);
			HAL_GPIO_WritePin(RTD_HORN_GPIO_Port, RTD_HORN_Pin, GPIO_PIN_RESET);

//			Arm the Inverter!
			can_bus->arm_inverter();

			while (!stop) {
				uint16_t torque_command = (uint16_t) torque_scale.int_scale(NU_TALK_VALUE);
				can_bus->set_torque(torque_command);
			}

		}
	}
}

void emergency_stop() {

	stop = true;

//	Forever Stationary
	while(1);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *candle) {
	HAL_CAN_Receive_IT(candle, CAN_FIFO0);

	Can_Bus *can_bus = can_bus->getInstance();
	can_bus->rx_update();

}

void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void _Error_Handler(char *file, int line) {
	while (1) {
	}
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{

}
#endif
