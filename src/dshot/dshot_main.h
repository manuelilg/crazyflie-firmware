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

#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */
#define DSHOT_DMA_BUFFER_UNIT uint32_t

DSHOT_DMA_BUFFER_UNIT dshotBurstDmaBuffer[2][DSHOT_DMA_BUFFER_SIZE * 4];

void init_dshot();

void dshotWrite(uint8_t index, float value);
void dshotWriteInt(uint8_t index, uint16_t value);
void pwmWriteDshotInt(uint8_t index, uint16_t value);
//uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb);
uint16_t prepareDshotPacket(uint16_t value);
uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet);
void pwmCompleteDshotMotorUpdate(void);
void motor_DMA_IRQHandler();
uint32_t timerClock(TIM_TypeDef *tim);


void waitDmaComplete();

#endif //CRAZYFLIE_FIRMWARE_DSHOT_MAIN_H
