//
// Created by manuel on 23.07.20.
//

#include "dshot/dshot_main.h"

#include <math.h>


void init_dshot() {
    /// GPIO
    // TIM2                                     -> DMA1[ S1-CH3: TIM2_UP , S7-CH3: TIM2_UP  ]
    // TIM2,  CH3, PA2, , PWM2 - OUT2 (Motor 2) -> DMA1[ S1-CH3: TIM2_CH3                   ]
    // TIM2,  CH4, PA3, , PWM4 - OUT4 (Motor 4) -> DMA1[ S6-CH3: TIM2_CH4, S7-CH3: TIM2_CH4 ]

    // TIM3                                     -> DMA1[ S2-CH5: TIM3_UP , S4-CH5: TIM3_TRIG]
    // TIM3,  CH1, PB4, , PWM1 - OUT1 (Motor 1) -> DMA1[ S4-CH5: TIM3_CH1                   ]
    // TIM3,  CH2, PB5, , PWM3 - OUT3 (Motor 3) -> DMA1[ S5-CH5: TIM3_CH2                   ]

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

    GPIO_InitTypeDef gpioInit;
    GPIO_StructInit(&gpioInit);

    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_Fast_Speed;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP; // TODO check this (see notes)
    // PA2
    gpioInit.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &gpioInit);
    // PA3
    gpioInit.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &gpioInit);
    // PB4
    gpioInit.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOB, &gpioInit);
    // PB5
    gpioInit.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOB, &gpioInit);

    /// Timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_Cmd(TIM2, DISABLE);
    TIM_Cmd(TIM3, DISABLE);
    //TIM_DeInit(TIM2);
    //TIM_DeInit(TIM3);

    TIM_TimeBaseInitTypeDef timInit;
    TIM_TimeBaseStructInit(&timInit);

    timInit.TIM_CounterMode = TIM_CounterMode_Up;
    timInit.TIM_Period = 19;
    timInit.TIM_ClockDivision = 0;
    timInit.TIM_RepetitionCounter = 0;

    // TIM2
    timInit.TIM_Prescaler = (uint16_t)(lrintf((float) timerClock(TIM2) / 12000000 + 0.01f) - 1);
    TIM_TimeBaseInit(TIM2, &timInit);

    // TIM3
    timInit.TIM_Prescaler = (uint16_t)(lrintf((float) timerClock(TIM3) / 12000000 + 0.01f) - 1);
    TIM_TimeBaseInit(TIM3, &timInit);

    TIM_OCInitTypeDef ocInitStruct;
    TIM_OCStructInit(&ocInitStruct);
    ocInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    ocInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    ocInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    ocInitStruct.TIM_OCPolarity = TIM_OCPolarity_High; // TODO check this changes if output is inverted
    ocInitStruct.TIM_Pulse = 0;

    /// DMA
    // dmaBurstRef = dmaRef
    // S1-CH3: TIM2_UP, S7-CH3: TIM2_UP
    DMA_Stream_TypeDef* dmaTim2 = DMA1_Stream7; //TODO check this
    // S2-CH5: TIM3_UP
    DMA_Stream_TypeDef* dmaTim3 = DMA1_Stream2; //TODO check this

    DMA_Cmd(dmaTim2, DISABLE);
    DMA_Cmd(dmaTim3, DISABLE);
    DMA_DeInit(dmaTim2);
    DMA_DeInit(dmaTim3);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(dmaTim2); // is redundant too in betaflight
    DMA_DeInit(dmaTim3); // is redundant too in betaflight

    DMA_InitTypeDef dmaInitStruct;
    DMA_StructInit(&dmaInitStruct);

    dmaInitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dmaInitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
    dmaInitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    dmaInitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dmaInitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dmaInitStruct.DMA_BufferSize = 18; // DSHOT_DMA_BUFFER_SIZE
    dmaInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dmaInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    dmaInitStruct.DMA_Mode = DMA_Mode_Normal;
    dmaInitStruct.DMA_Priority = DMA_Priority_High;

    //timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Disable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

    //timerOCInit(timer, timerHardware->channel, pOcInit);
    TIM_OC3Init(TIM2, &ocInitStruct);
    TIM_OC4Init(TIM2, &ocInitStruct);
    TIM_OC1Init(TIM3, &ocInitStruct);
    TIM_OC2Init(TIM3, &ocInitStruct);

    //timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // TIM2 Stream1 Channel 3
    dmaInitStruct.DMA_Channel = DMA_Channel_3;
    //dmaInitStruct.DMA_Memory0BaseAddr = 0; // TODO (uint32_t)motor->timer->dmaBurstBuffer;
    dmaInitStruct.DMA_Memory0BaseAddr = 0xABABABAB;
    dmaInitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM2->DMAR));
    DMA_Init(dmaTim2, &dmaInitStruct);

    // TIM3 Stream2 Channel 5
    dmaInitStruct.DMA_Channel = DMA_Channel_5;
    //dmaInitStruct.DMA_Memory0BaseAddr = 0; // TODO (uint32_t)motor->timer->dmaBurstBuffer;
    dmaInitStruct.DMA_Memory0BaseAddr = 0xCDCDCDCD;
    dmaInitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM3->DMAR));
    DMA_Init(dmaTim3, &dmaInitStruct);

    DMA_ITConfig(dmaTim2, DMA_IT_TC, ENABLE);
    DMA_ITConfig(dmaTim3, DMA_IT_TC, ENABLE);

    /// NVIC
    NVIC_InitTypeDef nvicInitStruct;
    nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    nvicInitStruct.NVIC_IRQChannelSubPriority = 0;
    nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;

    nvicInitStruct.NVIC_IRQChannel = DMA1_Stream1_IRQn;
    NVIC_Init(&nvicInitStruct);

    nvicInitStruct.NVIC_IRQChannel = DMA1_Stream2_IRQn;
    NVIC_Init(&nvicInitStruct);

    /// Enable
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    TIM_CCxCmd(TIM2, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);
    TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

/*
static bool dshotPwmEnableMotors(void)
{
    for (int i = 0; i < dshotPwmDevice.count; i++) {
        motorDmaOutput_t *motor = getMotorDmaOutput(i);
        const IO_t motorIO = IOGetByTag(motor->timerHardware->tag);
        IOConfigGPIOAF(motorIO, motor->iocfg, motor->timerHardware->alternateFunction);
    }

    // No special processing required
    return true;
}
*/

void dshotWrite(uint8_t index, float value) {
    pwmWriteDshotInt(index, lrintf(value));
}

void dshotWriteInt(uint8_t index, uint16_t value) {
    pwmWriteDshotInt(index, value);
}

void pwmWriteDshotInt(uint8_t index, uint16_t value) {
    motorDmaOutput_t *const motor = &dmaMotors[index];

    /*If there is a command ready to go overwrite the value and send that instead*/
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(index);
        if (value) {
            motor->protocolControl.requestTelemetry = true;
        }
    }

    motor->protocolControl.value = value;
    uint16_t packet = prepareDshotPacket(&motor->protocolControl);
    uint8_t bufferSize;

    if (useBurstDshot) {
        bufferSize = loadDmaBuffer(&motor->timer->dmaBurstBuffer[timerLookupChannelIndex(motor->timerHardware->channel)], 4, packet);
        motor->timer->dmaBurstLength = bufferSize * 4;
    }
    else {
        bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);
        motor->timer->timerDmaSources |= motor->timerDmaSource;
        DMA_SetCurrDataCounter((DMA_Stream_TypeDef *)(motor->dmaRef), bufferSize);
        DMA_Cmd((DMA_Stream_TypeDef *)(motor->dmaRef), ENABLE);
    }
}

void pwmCompleteDshotMotorUpdate(void) {
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
            return;
        }
    }

    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (useBurstDshot) {
            DMA_SetCurrDataCounter((DMA_Stream_TypeDef *)(dmaMotorTimers[i].dmaBurstRef), dmaMotorTimers[i].dmaBurstLength);
            DMA_Cmd((DMA_Stream_TypeDef *)(dmaMotorTimers[i].dmaBurstRef), ENABLE);
            TIM_DMAConfig(dmaMotorTimers[i].timer, ((uint16_t)0x000D), ((uint16_t)0x0300));
            TIM_DMACmd(dmaMotorTimers[i].timer, ((uint16_t)0x0100), ENABLE);
        }
        else {
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, DISABLE);
            dmaMotorTimers[i].timer->ARR = dmaMotorTimers[i].outputPeriod;
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, ENABLE);
            TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}

/*
void dshotPwmDisableMotors(void) {
    // No special processing required
    return;
}
*/

uint32_t timerClock(TIM_TypeDef *tim) {
    if (tim == TIM8 || tim == TIM1 || tim == TIM9 || tim == TIM10 || tim == TIM11) {
        return SystemCoreClock;
    } else {
        return SystemCoreClock / 2;
    }
}