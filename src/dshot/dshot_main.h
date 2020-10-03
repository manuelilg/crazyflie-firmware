//
// Created by manuel on 23.07.20.
//

#ifndef CRAZYFLIE_FIRMWARE_DSHOT_MAIN_H
#define CRAZYFLIE_FIRMWARE_DSHOT_MAIN_H

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_misc.h"

void init_dshot();

void dshotWrite(uint8_t index, float value);
void dshotWriteInt(uint8_t index, uint16_t value);
void pwmWriteDshotInt(uint8_t index, uint16_t value);
void pwmCompleteDshotMotorUpdate(void);

uint32_t timerClock(TIM_TypeDef *tim);


#endif //CRAZYFLIE_FIRMWARE_DSHOT_MAIN_H
