//
// Created by Koro on 2024/3/11.
//
#include <Arduino.h>
#ifndef HMG070PLCBOARD_FLEXIPLCLITE_H
#define HMG070PLCBOARD_FLEXIPLCLITE_H
#endif

#define OUT0 PA4
#define OUT1 PA5
#define OUT2 PA6
#define OUT3 PA7
#define OUT4 PB0
#define OUT5 PB1
#define OUT6 PB2
#define OUT7 PB12
#define OUT8 PA11
#define OUT9 PA12
#define OUT10 PA15
#define OUT11 PD0

#define sw1 PB15
#define sw2 PA8

#define IN0 PB3
#define IN1 PD3
#define IN2 PD2
#define IN3 PD1

#define BTN_UP PB9
#define BTN_DOWN PB8
#define BTN_OK PB5
#define BTN_CANCEL PB4

#define TX1 PB6
#define RX1 PB7

#define TX2 PA2
#define RX2 PA3

#define TX3 PB10
#define RX3 PB11

#define TX4 PA0
#define RX4 PA1

#define SCL1 PA9
#define SDA1 PA10

#define SCL2 PB13
#define SDA2 PB14

extern "C" void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}