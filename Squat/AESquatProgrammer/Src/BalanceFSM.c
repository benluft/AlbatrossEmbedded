/*
 * BalanceFSM.c
 *
 *  Created on: Mar 18, 2018
 *      Author: Albatross Electronics
 */

#include "BalanceFSM.h"

// Controls the state of the MCU. Will always start at the initialization state
FSMState currentState = Initializing;

//Controls what the FSM controller will switch the current state to next
FSMState nextState = Initializing;

//The most recent values read from each of the ADC channels
static volatile uint32_t u32ADC1Channel1Value = 1;
static volatile uint32_t u32ADC1Channel2Value = 1;
static volatile uint32_t u32ADC2Value = 1;
static volatile uint32_t u32ADC3Value = 1;

//5 times the weight of just a piece of plywood sitting on the sensors.  This requires
//that no weight is on the scale at startup
static volatile uint32_t u32ADC1Channel1ValueNoWeightThreshold;
static volatile uint32_t u32ADC1Channel2ValueNoWeightThreshold;
static volatile uint32_t u32ADC2ValueNoWeightThreshold;
static volatile uint32_t u32ADC3ValueNoWeightThreshold;

//These values are for when no weight is on the scales
static volatile uint32_t u32ADC1Channel1Zero;
static volatile uint32_t u32ADC1Channel2Zero;
static volatile uint32_t u32ADC2Zero;
static volatile uint32_t u32ADC3Zero;

//Handles for the ADCs
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

//Handles for the uarts.  Uart1 is connected to the BLE module
extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart3;

//Since ADC1 must read two channels, this boolean helps tell if the ADC has run through both
//of them
static volatile boolean bIsThisFirstChannelADC1;

//Will become true when the BLE module is connected to an android device
static volatile boolean bIsBLEConnected = true;

//Contains the number of samples specified by ADC_SAMPLES_LIMIT
//These values are averaged to determine the average ADC value over this time
static volatile uint32_t au32ADC1Channel1Samples[ADC_SAMPLES_LIMIT];
static volatile uint32_t au32ADC1Channel2Samples[ADC_SAMPLES_LIMIT];
static volatile uint32_t au32ADC2Samples[ADC_SAMPLES_LIMIT];
static volatile uint32_t au32ADC3Samples[ADC_SAMPLES_LIMIT];

//The average value of the ADC over the number of samples specified by ADC_SAMPLES_LIMIT
static volatile uint32_t u32AvgADC1Channel1 = 0;
static volatile uint32_t u32AvgADC1Channel2 = 0;
static volatile uint32_t u32AvgADC2 = 0;
static volatile uint32_t u32AvgADC3 = 0;
static volatile uint32_t u32AvgOverall = 0;

//These determine the percentage of the user's weight placed on each sensor
volatile uint8_t u8LeftFrontPercent = 25;
volatile uint8_t u8LeftBackPercent = 25;
volatile uint8_t u8RightFrontPercent = 25;
volatile uint8_t u8RightBackPercent = 25;

//This contains the message sent from the Android device to the BLE module
static volatile uint8_t* au8UartReceivedMessage;
static volatile uint8_t u8UartReceiveIndex;
//static volatile uint8_t* u8UartCurrentPosition;

static uint16_t u16numberOfADCLowWeightSamples = 0;

static boolean bFirstRun = true;


////Start the UART interrupt
//__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);

//Controls the current state of the FSM
void FSMStateController()
{
	switch(nextState)
	{
		case Initializing:

			HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

			currentState = Initializing;

		break;

		case WaitForAndroidAndWeight:

			//In initializing, the values with no weight are read into the the ADC values
			//so they need to be moved to the correct variables, and are muliplied by
			//1.1 so the device will only start sending data once the user puts significant
			//weight on the sensor.  Also these zero values are stored for later comparison
			if(currentState == Initializing)
			{
					u32ADC1Channel1Zero = u32ADC1Channel1Value;
					u32ADC1Channel2Zero = u32ADC1Channel2Value;
					u32ADC2Zero = u32ADC2Value;
					u32ADC3Zero = u32ADC3Value;
					u32ADC1Channel1ValueNoWeightThreshold = u32ADC1Channel1Value + THRESHOLD_WEIGHT;
					u32ADC1Channel2ValueNoWeightThreshold = u32ADC1Channel2Value + THRESHOLD_WEIGHT;
					u32ADC2ValueNoWeightThreshold = u32ADC2Value + THRESHOLD_WEIGHT;
					u32ADC3ValueNoWeightThreshold = u32ADC3Value + THRESHOLD_WEIGHT;
			}

			HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

			currentState = WaitForAndroidAndWeight;
		break;

		case ReadSensorData:

			HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

			currentState = ReadSensorData;
		break;

		case ProcessData:
			HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

			currentState = ProcessData;
		break;

		case SendBluetoothData:

			HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

			currentState = SendBluetoothData;
		break;

		case ProgramTermination:

			HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

			currentState = ProgramTermination;
		break;
	}

}

void FSMInitializing()
{
	if(currentState == Initializing)
	{

		//Set up UART received message interrupt
		bIsBLEConnected = true;

		//Wait one second to make sure everything is initialized properly
		HAL_Delay(5000);

		//Transmit 3 $ chars to the board with 200 ms between
		if(bFirstRun == true)
		{
			bFirstRun = false;
			uint8_t au8dollar[] = {'$'};

			HAL_UART_Transmit(&huart1, au8dollar, 1, 100);
			HAL_Delay(1000);
			HAL_UART_Transmit(&huart1, au8dollar, 1, 100);
			HAL_UART_Transmit(&huart1, au8dollar, 1, 100);
			HAL_Delay(1000);
		}
		//Runs the ADCs to get the value that each sensor experiences with no weight
		RunADCs();

		//Allow the ADCs to run
		HAL_Delay(2);

		//The ADCs only need to run once for a baseline
		nextState = WaitForAndroidAndWeight;

	}
}
void FSMWaitForAndroidAndWeight()
{
	if(currentState == WaitForAndroidAndWeight)
	{
		//TALK TO NATHAN AND BOBO ABOUT CONNECTING TO ANDROID

		//Delay for 2 ms to ensure that the ADCs can get a sample
		RunADCs();
		HAL_Delay(2);

		//If the latest values read from the ADCs are above the threshold
		//go to the read sensor data state
		if((u32ADC1Channel1Value > u32ADC1Channel1ValueNoWeightThreshold)
				|| (u32ADC1Channel2Value > u32ADC1Channel2ValueNoWeightThreshold)
				|| (u32ADC2Value > u32ADC2ValueNoWeightThreshold)
				|| (u32ADC3Value > u32ADC3ValueNoWeightThreshold))
		{
			nextState = ReadSensorData;
		}

	}

}

void FSMReadSensorData()
{
	if(currentState == ReadSensorData )
	{
		//Tells how many ADC values have been collected by the MCU
		static volatile uint8_t u8numberOfADCSamples = 0;

		//Delay to ensure that ADCs have run to completion
		HAL_Delay(2);

		//fill the arrays containing all the samples from this set of samples with the current
		//ADC values, minus the value that they experienced with no weight. Also prevent negative
		//overflow by setting to zero if the current value is less than the zero value
		if(u32ADC1Channel1Value > u32ADC1Channel1Zero)
		{
			au32ADC1Channel1Samples[u8numberOfADCSamples] = u32ADC1Channel1Value - u32ADC1Channel1Zero;
		}
		else
		{
			au32ADC1Channel1Samples[u8numberOfADCSamples] = 0;
		}

		if(u32ADC1Channel2Value > u32ADC1Channel2Zero)
		{
			au32ADC1Channel2Samples[u8numberOfADCSamples] = u32ADC1Channel2Value - u32ADC1Channel2Zero;
		}
		else
		{
			au32ADC1Channel2Samples[u8numberOfADCSamples] = 0;
		}

		if(u32ADC2Value > u32ADC2Zero)
		{
			au32ADC2Samples[u8numberOfADCSamples] = u32ADC2Value - u32ADC2Zero;
		}
		else
		{
			au32ADC2Samples[u8numberOfADCSamples] = 0;
		}

		if(u32ADC3Value > u32ADC3Zero)
		{
			au32ADC3Samples[u8numberOfADCSamples] = u32ADC3Value - u32ADC3Zero;
		}
		else
		{
			au32ADC3Samples[u8numberOfADCSamples] = 0;
		}

		//Increment the sample counter
		u8numberOfADCSamples++;

		//If not enough samples have been taken, continue to take samples.  Otherwise, move to the
		//state that processes the data
		if(u8numberOfADCSamples < ADC_SAMPLES_LIMIT)
		{
			RunADCs();
			HAL_Delay(2);
		}
		else
		{
			u8numberOfADCSamples = 0;
			nextState = ProcessData;
		}

	}

}

void FSMProcessData()
{
	if(currentState == ProcessData)
	{
		//The low weight samples are below the threshold set in the initialization state.
		//If enough low weight samples are encountered, the user has stepped off of the device,
		//and we should move to the terminating program state


		//we are making new averages, so set the previous ones to zero
		u32AvgADC1Channel1 = 0;
		u32AvgADC1Channel2 = 0;
		u32AvgADC2 = 0;

		u32AvgADC3 = 0;
		u32AvgOverall = 0;

		//We are taking the average of the last ADC_SAMPLES_LIMIT samples, so we first add all of the
		//values in the array together
		for(int i = 0; i < ADC_SAMPLES_LIMIT; i++)
		{
			u32AvgADC1Channel1 += au32ADC1Channel1Samples[i];
			u32AvgADC1Channel2 += au32ADC1Channel2Samples[i];
			u32AvgADC2 += au32ADC2Samples[i];
			u32AvgADC3 += au32ADC3Samples[i];
		}
		//Then we divide by the number of samples to get the average
		u32AvgADC1Channel1 /= ADC_SAMPLES_LIMIT;
		u32AvgADC1Channel2 /= ADC_SAMPLES_LIMIT;
		u32AvgADC2 /= ADC_SAMPLES_LIMIT;
		u32AvgADC3 /= ADC_SAMPLES_LIMIT;

		//This value is the total weight on the board
		u32AvgOverall = u32AvgADC1Channel1 + u32AvgADC1Channel2 + u32AvgADC2 + u32AvgADC3;

		//THIS IS ASSUMING THAT ALL OF THE SENSORS WILL HAVE THE SAME VOLTAGE WEIGHT
		//SLOPE WHICH IS WHAT WE TRIED TO CREATE IN OUR CALI}BRATION

		//As this will give the percentage of the user's weights on each quadrant
		if((u32AvgADC1Channel1 + u32AvgADC1Channel2 + u32AvgADC2 + u32AvgADC3) > MINIMUM_WEIGHT_OF_PERSON)
		{
				u8LeftBackPercent = ((u32AvgADC1Channel1 * 100) / u32AvgOverall); //LeftBack
				u8RightBackPercent = ((u32AvgADC1Channel2 * 100) / u32AvgOverall); //BackRight
				u8RightFrontPercent = ((u32AvgADC2 * 100) / u32AvgOverall); //RightFront
				u8LeftFrontPercent = ((u32AvgADC3 * 100) / u32AvgOverall); //LeftFront
				u16numberOfADCLowWeightSamples = 0;
		}
		else
		{
				u16numberOfADCLowWeightSamples++;
				u8LeftBackPercent = 25; //LeftFront
				u8RightBackPercent = 25; //LEFTBACK
				u8LeftFrontPercent = 25; //RIGHTFRONT
				u8RightFrontPercent = 25; //BACKRIGHT
		}

		if(u8LeftBackPercent >= 100)
		{
			u8LeftBackPercent = 99;
		}

		if(u8LeftFrontPercent >= 100)
		{
			u8LeftFrontPercent = 99;
		}

		if(u8RightBackPercent >= 100)
		{
			u8RightBackPercent = 99;
		}

		if(u8RightFrontPercent >= 100)
		{
			u8RightFrontPercent = 99;
		}

		//These if statements set the logic for the buzzers.  If a quadrant has more than
		//UNEVEN_WEIGHT_DIST percent of the user's weight, the buzzer in that quadrant will be
		//activated
		if(u8RightBackPercent > UNEVEN_WEIGHT_DIST)
		{
			HAL_GPIO_WritePin(BackRightBuzz_GPIO_Port, BackRightBuzz_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(BackRightBuzz_GPIO_Port, BackRightBuzz_Pin, GPIO_PIN_RESET);
		}

		if(u8LeftFrontPercent > UNEVEN_WEIGHT_DIST)
		{
			HAL_GPIO_WritePin(FrontLeftBuzz_GPIO_Port, FrontLeftBuzz_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(FrontLeftBuzz_GPIO_Port, FrontLeftBuzz_Pin, GPIO_PIN_RESET);
		}

		if(u8LeftBackPercent > UNEVEN_WEIGHT_DIST)
		{
			HAL_GPIO_WritePin(BackLeftBuzz_GPIO_Port, BackLeftBuzz_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(BackLeftBuzz_GPIO_Port, BackLeftBuzz_Pin, GPIO_PIN_RESET);
		}
		if(u8RightFrontPercent > UNEVEN_WEIGHT_DIST)
		{
			HAL_GPIO_WritePin(FrontRightBuzz_GPIO_Port, FrontRightBuzz_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(FrontRightBuzz_GPIO_Port, FrontRightBuzz_Pin, GPIO_PIN_RESET);
		}



		//This logic allows us to determine how many times no weight has been detected on the
		//scale, and if we have WEIGHTED long enough to inform the Android device that the workout
		//is complete
		// if(u32ADC1Channel1Value <= u32ADC1Channel1ValueNoWeightThreshold &&
		// 		u32ADC1Channel2Value <= u32ADC1Channel2ValueNoWeightThreshold &&
		// 		u32ADC2Value <= u32ADC2ValueNoWeightThreshold &&
		// 		u32ADC3Value <= u32ADC3ValueNoWeightThreshold)
		// {
		//
		// 	HAL_GPIO_WritePin(BackLeftBuzz_GPIO_Port, BackLeftBuzz_Pin, GPIO_PIN_RESET);
		// 	HAL_GPIO_WritePin(BackRighttBuzz_GPIO_Port, BackRightBuzz_Pin, GPIO_PIN_RESET);
		// 	HAL_GPIO_WritePin(FrontLeftBuzz_GPIO_Port, FrontLeftBuzz_Pin, GPIO_PIN_RESET);
		// 	HAL_GPIO_WritePin(FrontRightBuzz_GPIO_Port, FrontRightBuzz_Pin, GPIO_PIN_RESET);
		// }
		// else
		// {
		// 	u16numberOfADCLowWeightSamples = 0;
		// }

		//If we have enough low weight samples, terminate the program
		//If we don't have enough low weight samples, and BLE is connected, begin sending the data
		//Otherwise, just read the sensors again
		if(u16numberOfADCLowWeightSamples >= MAX_NUM_LOW_WEIGHT_SAMPLES)
		{
			nextState = ProgramTermination;
		}
		else if(bIsBLEConnected)
		{
			nextState = SendBluetoothData;
		}
		else

		{
			nextState = ReadSensorData;
		}

	}

}

void FSMSendBluetoothData()
{
	if(currentState == SendBluetoothData)
	{
		//Clear the received message
		au8UartReceivedMessage = NULL;

		//Make an array of uintu16numberOfADCLowWeightSamples8s that can be sent via UART.  These values are the percentages of
		//the user's weight on each quadrant, starting with the front left value and moving clockwise

		nextState = ReadSensorData;

		//If the correct message was received, begin reading ADC values again.  If the Android device
		//sent the wrong message, or no message at all, try to send the message again and set an error
		//LED
	}

}

void FSMProgramTermination()
{
	if(currentState == ProgramTermination)
	{
		u16numberOfADCLowWeightSamples = 0;
		//Make sure to turn off all buzzers once the program is complete
		HAL_GPIO_WritePin(FrontRightBuzz_GPIO_Port, FrontRightBuzz_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(FrontLeftBuzz_GPIO_Port, FrontLeftBuzz_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BackRightBuzz_GPIO_Port, BackRightBuzz_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BackLeftBuzz_GPIO_Port, BackLeftBuzz_Pin, GPIO_PIN_RESET);

		//Go back to the initializing state
		nextState = Initializing;
	}

}

void RunADCs()
{
	//Runs all ADCs in single sample mode
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc3);
	HAL_ADC_Start_IT(&hadc1);

}

//Called when ADC has completed both conversions
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//Make sure ADC1 has been the one causing the interrupt
	if(hadc -> Instance == ADC1)
	{
		//The system will interrupt once each channel is converted, so read the first
		//one
		if(bIsThisFirstChannelADC1)
		{
			//Get the value and set the semaphore
			u32ADC1Channel1Value = HAL_ADC_GetValue(&hadc1);
			bIsThisFirstChannelADC1 = false;

			//Start the second conversion
			HAL_ADC_Start_IT(&hadc1);
		}
		else
		{
			//Get the second value and change the semaphore
			u32ADC1Channel2Value = HAL_ADC_GetValue(&hadc1);
			bIsThisFirstChannelADC1 = true;

			//ADC2 and ADC3 should be done converting by this point
			u32ADC2Value = HAL_ADC_GetValue(&hadc2);
			u32ADC3Value = HAL_ADC_GetValue(&hadc3);
		}
	}

}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	//If a message has been received, read it
//	HAL_UART_Receive_IT(huart, au8UartReceivedMessage, SIZE_OF_BLE_RX_MSG);
//}
