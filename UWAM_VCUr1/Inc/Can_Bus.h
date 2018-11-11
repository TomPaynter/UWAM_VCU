// Inverter Settings
#define MAX_TORQUE 140

// CAN Receive IDs
#define PEDALBOX_ID 0x3C

// CAN Transmit IDs
#define INVERTER_ID 0x0C0
#define PDM_ID 0x28C

// ****** End User Settings ************************************

#include <string>

extern "C" {
#include "stm32f0xx_hal.h"
#include "can.h"
#include "gpio.h"
}

class Can_Bus {
	static Can_Bus* instance;

public:
	static Can_Bus *getInstance() {
		if (!instance)
			instance = new Can_Bus;
		return instance;
	}

private:
	CanRxMsgTypeDef RxMessage;
	CanTxMsgTypeDef TxMessage;

	uint8_t torque_request;
	uint8_t brake_pressure;
	uint8_t implausibility;
	uint8_t cooling_state;
	uint16_t current_torque;

	uint8_t recievedpedalbox;

	Can_Bus(void) {
		torque_request = 0;
		brake_pressure = 0;
		implausibility = 0;
		cooling_state = 0;
		current_torque = 0;
		recievedpedalbox = 0;
	}

	void init(void) {
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		hcan.pTxMsg = &TxMessage;
		hcan.pRxMsg = &RxMessage;

		CAN_FilterConfTypeDef sFilterConfig;

		sFilterConfig.FilterNumber = 0;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
		sFilterConfig.FilterIdHigh = PEDALBOX_ID;
		sFilterConfig.FilterIdLow = PEDALBOX_ID << 5;

		sFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
		sFilterConfig.FilterActivation = ENABLE;
		HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

//		sFilterConfig.FilterNumber = 1;
//		sFilterConfig.FilterIdHigh = CANID_2;
//		sFilterConfig.FilterIdLow = CANID_2 << 5;
//		HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

		HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

	}

public:
	void rx_update(void) {
		switch (RxMessage.StdId) {
		case PEDALBOX_ID:
			torque_request = RxMessage.Data[0];
			brake_pressure = RxMessage.Data[2];
			implausibility = RxMessage.Data[3];

			recievedpedalbox = 1;
			break;

		}
		HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	}

	uint8_t recievedPedalBox() {
		return recievedpedalbox;
	}

	void clearPedalBox() {
		recievedpedalbox = 0;
	}

	void cooling_on() {
		TxMessage.StdId = PDM_ID;
		TxMessage.DLC = 1;
		TxMessage.Data[0] = 0b10000000;
		cooling_state = 1;
		HAL_CAN_Transmit_IT(&hcan);

	}

	void cooling_off() {
		TxMessage.StdId = PDM_ID;
		TxMessage.DLC = 1;
		TxMessage.Data[0] = 0b00000000;
		cooling_state = 0;
		HAL_CAN_Transmit_IT(&hcan);

	}

	uint8_t get_cooling_state() {
		return cooling_state;
	}

	void set_torque(uint16_t torque_command) {
		TxMessage.StdId = INVERTER_ID;
		TxMessage.DLC = 8;

		if (torque_command > MAX_TORQUE)
			torque_command = MAX_TORQUE;

//		Inverter Torque Settings
		TxMessage.Data[0] = torque_command % 256;
		TxMessage.Data[1] = torque_command / 256;

//		Inverter Speed Settings (Unused Set 0)
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;

//		Motor Direction - Forward
		TxMessage.Data[4] = 1;

//		Inverter Run - Run
		TxMessage.Data[5] = 1;

//		Unknown set 0
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;

		HAL_CAN_Transmit_IT(&hcan);

	}

	uint16_t get_current_torque() {
		return current_torque;
	}

	void arm_inverter() {
		TxMessage.StdId = INVERTER_ID;
		TxMessage.DLC = 8;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;

		HAL_CAN_Transmit_IT(&hcan);

	}

	void rx_print() {
		printf("Receiving CAN:  ID: %d DLC: %d Data: ", (int) RxMessage.StdId,
				(int) RxMessage.DLC);

		for (uint8_t i = 0; i < RxMessage.DLC; i++)
			printf("%d ", (int) RxMessage.Data[i]);

		printf("\n\r");

	}

	void tx_print() {

		printf("\tTransmitting CAN:  ID: %d DLC: %d Data: ",
				(int) TxMessage.StdId, (int) TxMessage.DLC);

		for (uint8_t i = 0; i < TxMessage.DLC; i++)
			printf("%d ", (int) TxMessage.Data[i]);

		printf("\n\r");
	}

}
;
