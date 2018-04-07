/*
 * BalanceFSM.h
 *
 *  Created on: Mar 18, 2018
 *      Author: Ben
 */

#ifndef BALANCEFSM_H_
#define BALANCEFSM_H_

#include "main.h"
#include "stm32f4xx_hal.h"

typedef enum {Initializing, WaitForAndroidAndWeight, ReadSensorData,
	ProcessData, SendBluetoothData, ProgramTermination} FSMState;

typedef enum {false,true} boolean;

void FSMStateController();
void FSMInitializing();
void FSMWaitForAndroidAndWeight();
void FSMReadSensorData();
void FSMProcessData();
void FSMSendBluetoothData();
void FSMProgramTermination();

void RunADCs();

#define BLE_INIT_MSG				"BoboStart"
#define SIZE_OF_BLE_INIT_MSG	 	strlen(BLE_INIT_MSG)
#define SIZE_OF_BLE_RX_MSG			30
#define ADC_SAMPLES_LIMIT			10
#define UNEVEN_WEIGHT_DIST			35
#define THRESHOLD_WEIGHT				200
#define MAX_NUM_LOW_WEIGHT_SAMPLES	500
#define MINIMUM_WEIGHT_OF_PERSON		750
#define	BLE_END_OF_TX				"End"
#define SIZE_OF_BLE_END_OF_TX		strlen(BLE_END_OF_TX)

#endif /* BALANCEFSM_H_ */
