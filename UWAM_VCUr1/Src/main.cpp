// Inverter Settings
#define MAX_TORQUE 140

// CAN Receive IDs
#define PEDALBOX_ID 0x3C
#define INVERTER_FAUL_ID 0x0AB
// CAN Transmit IDs
#define INVERTER_ID 0x0C0
#define PDM_ID 0x28C
#define VCU_ID 0x320

// Coolant Temp
#define MAX_COOLANT_TEMP 4000
#define THERMOSTAT_ON 30
#define THERMOSTAT_OFF 25

//Min Coolant Pressure
#define MIN_COOLANT_PRESSURE 2000

// NU_TALK_VALUE - debug please not to be released if present
#define NU_TALK_VALUE 0

// ****** End User Settings ************************************

extern "C" {
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
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
#include "thermostat.h"
#include "sensors/bosch_NTC_M12/bosch_NTC_M12.h"
#include "sensors/variohm_EPT2100/variohm_EPT2100.h"

void SystemClock_Config(void);
void _Error_Handler(char *file, int line);

Can_Bus *Can_Bus::instance = 0;

volatile uint8_t bp_ok = 0;
volatile uint8_t start_ok = 0;
volatile uint8_t safety_ok = 0;
volatile uint8_t inverter_happy = 0;

volatile uint8_t state = 0;

uint32_t ADC_RAW[200];

void emergency_stop(void);
void RTD_Sound(void);

int main(void) {

	HAL_Init();

	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_DMA_Init();
	MX_CAN_Init();
	MX_USART1_UART_Init();

	Can_Bus *can_bus = can_bus->getInstance();

	HAL_GPIO_WritePin(SAFETY_CONTROL_GPIO_Port, SAFETY_CONTROL_Pin,
			GPIO_PIN_RESET);

//	Set Safety Line High
	HAL_GPIO_WritePin(SAFETY_CONTROL_GPIO_Port, SAFETY_CONTROL_Pin,
			GPIO_PIN_SET);

	HAL_TIM_Base_Start_IT(&htim3);

	while (1) {
// 		Now in "Standby" State
		state = 1;

//		Check if Brake Pressure is high enough
		bp_ok = (can_bus->get_brake_pressure() > 5);

//		DEBUG TEST!!!!!!
		bp_ok = true;

//		Check if Inverter is happy
		inverter_happy = !(can_bus->get_inverter_fault());

//		DEBUG TEST!!!!!!
		inverter_happy = true;

//		Check Start Switch to go Low
		start_ok = (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin)
				== GPIO_PIN_RESET);

//		Check Safety Line to go High
		safety_ok = (HAL_GPIO_ReadPin(SAFETY_GPIO_Port, SAFETY_Pin)
				== GPIO_PIN_SET);

		if (bp_ok && start_ok && safety_ok && inverter_happy) {
//			Now in "RTD Sound" State
			state = 2;

// 			Sound RTD Horn
			RTD_Sound();

//			Arm the Inverter!
			can_bus->arm_inverter();

			while (1) {
//				Now in "Drive" State
				state = 3;

				if (can_bus->recievedPedalBox()) {
					can_bus->clearPedalBox();

					if (can_bus->get_implausibility_event())
						emergency_stop();

					if (can_bus->get_inverter_fault())
						emergency_stop();

					uint16_t torque_command = (uint16_t) torque_scale.int_scale(
							can_bus->get_torque_request());
					can_bus->set_inverter_torque(torque_command);
				}

			}
		}
	}
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *candle) {
	HAL_CAN_Receive_IT(candle, CAN_FIFO0);

	Can_Bus *can_bus = can_bus->getInstance();
	can_bus->rx_update();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	Can_Bus *can_bus = can_bus->getInstance();

//	Transmit Cooling info on can bus
//	can_bus->transmit_vcu_data(can_bus->get_inverter_fault,
//			coolant_temp.getTemp(), bp_ok, start_ok, safety_ok);

}

void RTD_Sound(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = RTD_HORN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(RTD_HORN_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(RTD_HORN_GPIO_Port, RTD_HORN_Pin, GPIO_PIN_SET);

	HAL_Delay(3000);

	HAL_GPIO_WritePin(RTD_HORN_GPIO_Port, RTD_HORN_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = RTD_HORN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(RTD_HORN_GPIO_Port, &GPIO_InitStruct);
}

void emergency_stop() {
//	Now in "Stop" State
	state = 4;

	Can_Bus *can_bus = can_bus->getInstance();
	can_bus->disable_inverter();

	//	Set Safety Line Low
	HAL_GPIO_WritePin(SAFETY_CONTROL_GPIO_Port, SAFETY_CONTROL_Pin,
			GPIO_PIN_RESET);

//	Forever Stationary
	while (1)
		;
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14
			| RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

void Error_Handler(void) {

}

#ifdef  USE_FULL_ASSERT

void assert_failed(char *file, uint32_t line)
{

}
#endif
