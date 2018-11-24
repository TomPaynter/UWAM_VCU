#include <string>

extern "C" {
#include "main.h"
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

	bool recievedpedalbox;
	bool inverter_fault;

	Can_Bus(void) {
		torque_request = 0;
		brake_pressure = 0;
		implausibility = 0;
		cooling_state = 0;
		current_torque = 0;
		recievedpedalbox = 0;
		inverter_fault = true;
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

		case INVERTER_FAUL_ID:
			bool byte0 = 0 == RxMessage.Data[0];
			bool byte1 = 0 == RxMessage.Data[1];
			bool byte2 = 0 == RxMessage.Data[2];
			bool byte3 = 0 == RxMessage.Data[3];
			bool byte4 = 0 == RxMessage.Data[4];
			bool byte5 = 0 == RxMessage.Data[5];
			bool byte6 = 0 == RxMessage.Data[6];
			bool byte7 = 0 == (RxMessage.Data[7] & 0b01111111);

			brake_pressure = RxMessage.Data[2];
			implausibility = RxMessage.Data[3];

			inverter_fault = byte0 && byte1 && byte2 && byte3 && byte4 && byte5
					&& byte6 && byte7;
			break;

		}
		HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	}

	bool get_inverter_fault(void) {
		return inverter_fault;
	}
	uint8_t get_brake_pressure() {
		return brake_pressure;
	}

	uint8_t get_torque_request() {
		return torque_request;
	}

	bool get_implausibility_event() {
		return implausibility == 1;
	}

	bool recievedPedalBox() {
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

	void set_inverter_torque(uint16_t torque_command) {
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

	void disable_inverter() {
		TxMessage.StdId = INVERTER_ID;
		TxMessage.DLC = 8;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;

//		Motor Direction - Dunno what direction that is, but Luke said so! :p
		TxMessage.Data[4] = 0xFF;

//		Inverter Run - Dunno what direction that is, but Luke said so! :p
		TxMessage.Data[5] = 0x40;

		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;

		HAL_CAN_Transmit_IT(&hcan);

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

	void transmit_vcu_data(uint8_t state, uint8_t bp_ok, uint8_t start_ok,
			uint8_t safety_ok, uint8_t inverter_happy) {

		TxMessage.StdId = VCU_ID;
		TxMessage.DLC = 5;
		TxMessage.Data[0] = state;
		TxMessage.Data[1] = bp_ok;
		TxMessage.Data[2] = start_ok;
		TxMessage.Data[3] = safety_ok;
		TxMessage.Data[4] = inverter_happy;


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
