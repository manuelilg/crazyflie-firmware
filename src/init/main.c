/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * main.c - Containing the main function.
 */

/* Personal configs */
#include "FreeRTOSConfig.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/* Project includes */
#include "config.h"
#include "platform.h"
#include "system.h"
#include "usec_time.h"

#include "led.h"

#include "dshot/dshot_main.h"

/* ST includes */
#include "stm32fxxx.h"

/*
typedef enum
{
    HAL_TICK_FREQ_10HZ         = 100U,
    HAL_TICK_FREQ_100HZ        = 10U,
    HAL_TICK_FREQ_1KHZ         = 1U,
    HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;

__IO uint32_t uwTick = 0;
HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;

#define NVIC_PRIORITYGROUP_4         0x00000003U

#define  TICK_INT_PRIORITY            0U

uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb)
{
  return SysTick_Config(TicksNumb);
}

void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t prioritygroup = 0x00U;

  assert_param(IS_NVIC_SUB_PRIORITY(SubPriority));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));

  prioritygroup = NVIC_GetPriorityGrouping();

  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}

void HAL_InitTick(uint32_t TickPriority)
{
  if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
  {
    return;
  }

  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
    //uwTickPrio = TickPriority;
  }
  else
  {
    return;
  }

  return;
}

uint32_t HAL_GetTick(void)
{
  return uwTick;
}

void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
}

//void __attribute__((used)) SysTick_Handler(void)
//{
//  HAL_IncTick();
//}

#define HAL_MAX_DELAY      0xFFFFFFFFU

void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}

typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
} dshotCommands_e;
*/
int main() 
{
  /*
  //NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_InitTick(TICK_INT_PRIORITY);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef gpioInit;
  GPIO_StructInit(&gpioInit);
  gpioInit.GPIO_Pin = GPIO_Pin_0;
  gpioInit.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &gpioInit);

  ADC_InitTypeDef adcInit;
  ADC_StructInit(&adcInit);
  //adcInit.ADC_ContinuousConvMode = ENABLE;
  ADC_Init(ADC1, &adcInit);
  ADC_Cmd(ADC1, ENABLE);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_56Cycles);
  //ADC_SoftwareStartConv(ADC1);
  //ADC_ContinuousModeCmd(ADC1, ENABLE);
  HAL_Delay(2);

  //uint16_t adcValue = ADC_GetConversionValue(ADC1);
  //HAL_Delay(adcValue);

  init_dshot();

  HAL_Delay(5000);

  for (int i = 0; i < 10; ++i) {
    pwmWriteDshotInt(2, 48);
    pwmCompleteDshotMotorUpdate();
    HAL_Delay(2);
  }

  pwmWriteDshotInt(2, DSHOT_CMD_SPIN_DIRECTION_REVERSED);
  pwmCompleteDshotMotorUpdate();
  HAL_Delay(2);

//  pwmWriteDshotInt(2, 0);
//  pwmCompleteDshotMotorUpdate();
//  HAL_Delay(2);

  pwmWriteDshotInt(2, 48);
  pwmCompleteDshotMotorUpdate();
  HAL_Delay(2);

  pwmWriteDshotInt(2, 1000);
  pwmCompleteDshotMotorUpdate();
  HAL_Delay(2);

  pwmWriteDshotInt(2, 2407);
  pwmCompleteDshotMotorUpdate();
  HAL_Delay(2);

  pwmWriteDshotInt(2, 1000);
  pwmCompleteDshotMotorUpdate();
  HAL_Delay(2);

  pwmWriteDshotInt(2, 48);
  pwmCompleteDshotMotorUpdate();
  HAL_Delay(2);

  for (int i = 0; i < 500; ++i) {
    pwmWriteDshotInt(2, 48);
    pwmCompleteDshotMotorUpdate();
    HAL_Delay(2);
  }

  pwmWriteDshotInt(2, DSHOT_CMD_SPIN_DIRECTION_REVERSED);
  pwmCompleteDshotMotorUpdate();
  HAL_Delay(2);

//  for (int i = 0; i < 1000; ++i) {
//    pwmWriteDshotInt(2, 1);
//    pwmCompleteDshotMotorUpdate();
//    HAL_Delay(2);
//  }
//
//  for (int i = 0; i < 1000; ++i) {
//    pwmWriteDshotInt(2, 2);
//    pwmCompleteDshotMotorUpdate();
//    HAL_Delay(2);
//  }
//
//  for (int i = 0; i < 1000; ++i) {
//    pwmWriteDshotInt(2, 3);
//    pwmCompleteDshotMotorUpdate();
//    HAL_Delay(2);
//  }

  while (true) {
    ADC_SoftwareStartConv(ADC1);
    HAL_Delay(2);
    uint32_t adcValue = ADC_GetConversionValue(ADC1);

    //map from 0 - 4096 to 48 - 2047
    adcValue = adcValue * (2407-48) / 4096 + 48;


    //pwmWriteDshotInt(0, 1000);
    //pwmWriteDshotInt(1, 1000);
    pwmWriteDshotInt(2, adcValue);
    //pwmWriteDshotInt(3, 1000);
    pwmCompleteDshotMotorUpdate();

    HAL_Delay(188);
  }
  //*/
  //while (true) {
  //    pwmWriteDshotInt(0, 1000);
  //    pwmWriteDshotInt(1, 1000);
  //    pwmCompleteDshotMotorUpdate();

  //    waitDmaComplete();
  //    //motor_DMA_IRQHandler();
  //    //while (true) ;
  //}

  //Initialize the platform.
  int err = platformInit();
  if (err != 0) {
    // The firmware is running on the wrong hardware. Halt
    while(1);
  }

  //Launch the system task that will initialize and start everything
  systemLaunch();

  //Start the FreeRTOS scheduler
  vTaskStartScheduler();

  //TODO: Move to platform launch failed
  ledInit();
  ledSet(0, 1);
  ledSet(1, 1);

  //Should never reach this point!
  while(1);

  return 0;
}

