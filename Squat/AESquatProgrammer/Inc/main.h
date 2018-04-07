/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define FrontLeftBuzz_Pin GPIO_PIN_2
#define FrontLeftBuzz_GPIO_Port GPIOE
#define BackLeftBuzz_Pin GPIO_PIN_3
#define BackLeftBuzz_GPIO_Port GPIOE
#define FrontLeftSense_Pin GPIO_PIN_1
#define FrontLeftSense_GPIO_Port GPIOA
#define BackLeftSense_Pin GPIO_PIN_2
#define BackLeftSense_GPIO_Port GPIOA
#define BackRightSense_Pin GPIO_PIN_5
#define BackRightSense_GPIO_Port GPIOA
#define FrontRightSense_Pin GPIO_PIN_6
#define FrontRightSense_GPIO_Port GPIOA
#define FrontRightBuzz_Pin GPIO_PIN_4
#define FrontRightBuzz_GPIO_Port GPIOC
#define BackRightBuzz_Pin GPIO_PIN_5
#define BackRightBuzz_GPIO_Port GPIOC
#define CONFIG_MCU_Pin GPIO_PIN_15
#define CONFIG_MCU_GPIO_Port GPIOE
#define Debug_Tx_Pin GPIO_PIN_10
#define Debug_Tx_GPIO_Port GPIOB
#define Debug_RX_Pin GPIO_PIN_11
#define Debug_RX_GPIO_Port GPIOB
#define P3_6_Pin GPIO_PIN_13
#define P3_6_GPIO_Port GPIOB
#define P0_0_Pin GPIO_PIN_14
#define P0_0_GPIO_Port GPIOB
#define P2_7_Pin GPIO_PIN_15
#define P2_7_GPIO_Port GPIOB
#define P1_2_Pin GPIO_PIN_6
#define P1_2_GPIO_Port GPIOC
#define P1_3_Pin GPIO_PIN_7
#define P1_3_GPIO_Port GPIOC
#define P1_6_Pin GPIO_PIN_8
#define P1_6_GPIO_Port GPIOC
#define P1_7_Pin GPIO_PIN_9
#define P1_7_GPIO_Port GPIOC
#define MCU_TX_Pin GPIO_PIN_9
#define MCU_TX_GPIO_Port GPIOA
#define MCU_RX_Pin GPIO_PIN_10
#define MCU_RX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWDCLK_Pin GPIO_PIN_14
#define SWDCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_8
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_9
#define LED_2_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_0
#define LED_3_GPIO_Port GPIOE
#define LED_4_Pin GPIO_PIN_1
#define LED_4_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
