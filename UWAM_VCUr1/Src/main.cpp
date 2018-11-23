// Inverter Settings
#define MAX_TORQUE 140

// CAN Receive IDs
#define PEDALBOX_ID 0x3C

// CAN Transmit IDs
#define INVERTER_ID 0x0C0
#define PDM_ID 0x28C

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
bool stop = false;

linear_scale torque_scale = linear_scale(0, 100, MAX_TORQUE, 0);
thermostat coolant_thermostat = thermostat(THERMOSTAT_ON, THERMOSTAT_OFF);

bosch_NTC_M12 coolant_temp(10e3);
variohm_EPT2100 coolant_pressure;

volatile uint8_t coolant_ok = 0;
volatile uint8_t pressure_ok = 0;
volatile uint8_t bp_ok = 0;
volatile uint8_t start_ok = 0;
volatile uint8_t safety_ok = 0;

uint32_t ADC_RAW[200];

void emergency_stop() {
	stop = true;
	Can_Bus *can_bus = can_bus->getInstance();
	can_bus->disable_inverter();

//	Forever Stationary
	while (1)
		;
}

int main(void) {

	HAL_Init();

	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_DMA_Init();
	MX_CAN_Init();
	MX_USART1_UART_Init();
	MX_ADC_Init();

	HAL_ADCEx_Calibration_Start(&hadc);

	Can_Bus *can_bus = can_bus->getInstance();

	HAL_GPIO_WritePin(RTD_HORN_GPIO_Port, RTD_HORN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SAFETY_CONTROL_GPIO_Port, SAFETY_CONTROL_Pin,
			GPIO_PIN_RESET);

//	Set Safety Line High
	HAL_GPIO_WritePin(SAFETY_CONTROL_GPIO_Port, SAFETY_CONTROL_Pin,
			GPIO_PIN_SET);

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start_DMA(&hadc, ADC_RAW, 200);

	while (1) {
// 		Now in "Standby" State
//		Check if Brake Pressure is high enough
		bp_ok = (can_bus->get_brake_pressure() > 5);

//		DEBUG TEST!!!!!!
		bp_ok = true;

//		Check Start Switch to go Low
		start_ok = (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin)
				== GPIO_PIN_RESET);

//		Check Safety Line to go High
		safety_ok = (HAL_GPIO_ReadPin(SAFETY_GPIO_Port, SAFETY_Pin)
				== GPIO_PIN_SET);

		coolant_ok = coolant_temp.getTemp() < MAX_COOLANT_TEMP;

		if (bp_ok && start_ok && safety_ok && coolant_ok) {
//			Now in "RTD Sound" State
// 			Sound RTD Horn

			HAL_GPIO_WritePin(RTD_HORN_GPIO_Port, RTD_HORN_Pin, GPIO_PIN_SET);
			HAL_Delay(2000);
			HAL_GPIO_WritePin(RTD_HORN_GPIO_Port, RTD_HORN_Pin, GPIO_PIN_RESET);

//			Arm the Inverter!
			can_bus->arm_inverter();

			while (1) {

				if (!stop) {
					if (can_bus->recievedPedalBox()) {
						can_bus->clearPedalBox();

						if (can_bus->get_implausibility_event())
							emergency_stop();

						uint16_t torque_command =
								(uint16_t) torque_scale.int_scale(
										can_bus->get_torque_request());
						can_bus->set_inverter_torque(torque_command);
					}
				} else
					emergency_stop();
			}
		}
	}
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *candle) {
	HAL_CAN_Receive_IT(candle, CAN_FIFO0);

	Can_Bus *can_bus = can_bus->getInstance();
	can_bus->rx_update();

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	float adc0raw = 0;
	float adc1raw = 0;

	for (int i = 0; i < 200; i = i + 2)
		adc0raw = adc0raw + ADC_RAW[i];

	for (int i = 1; i < 200; i = i + 2)
		adc1raw = adc1raw + ADC_RAW[i];

	adc0raw = adc0raw / 100;
	adc1raw = adc1raw / 100;
//
//	//	Yea, do some fancy calibrations and shit for the adc to pressure here
//	coolant_pressure.calcPressure(adc0raw / 4096 * 3.3);
//
//	coolant_temp.calcTemp(adc1raw);
//	Can_Bus *can_bus = can_bus->getInstance();
//
//	coolant_ok = coolant_temp.getTemp() < MAX_COOLANT_TEMP;
//	pressure_ok = coolant_pressure.getPressure() > MIN_COOLANT_PRESSURE;
//
//	if (!(pressure_ok && coolant_ok)) {
//		emergency_stop();
//	}
//
//	if (coolant_thermostat.checkToggle(coolant_temp.getTemp())) {
//		if (coolant_thermostat.getStatus())
//			can_bus->cooling_on();
//		else
//			can_bus->cooling_off();
//	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	Can_Bus *can_bus = can_bus->getInstance();

//	Transmit Cooling info on can bus
//	can_bus->transmit_vcu_data(coolant_pressure.getPressure(),
//			coolant_temp.getTemp(), bp_ok, start_ok, safety_ok);

}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(char *file, uint32_t line)
{

}
#endif
