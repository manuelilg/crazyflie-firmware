//
// Created by manuel on 23.07.20.
//

#include "dshot/dshot_main.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "i2c_drv.h"

#include <math.h>

// forward declaration
void setupDma1Stream2Channel5();

DMA_Stream_TypeDef* const dmaTim2 = DMA1_Stream1; //TODO check this
//uint16_t dmaBurstLengthStream7;
DMA_Stream_TypeDef* const dmaTim3 = DMA1_Stream2; //TODO check this
//uint16_t dmaBurstLengthStream2;

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
    TIM_DeInit(TIM2);
    TIM_DeInit(TIM3);

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
    //DMA_Stream_TypeDef* dmaTim2 = DMA1_Stream7; //TODO check this
    // S2-CH5: TIM3_UP
    //DMA_Stream_TypeDef* dmaTim3 = DMA1_Stream2; //TODO check this

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
    dmaInitStruct.DMA_Memory0BaseAddr = (uint32_t)dshotBurstDmaBuffer[0]; // TODO (uint32_t)motor->timer->dmaBurstBuffer;
    //dmaInitStruct.DMA_Memory0BaseAddr = 0xABABABAB;
    dmaInitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM2->DMAR));
    DMA_Init(dmaTim2, &dmaInitStruct);

    // TIM3 Stream2 Channel 5
    dmaInitStruct.DMA_Channel = DMA_Channel_5;
    dmaInitStruct.DMA_Memory0BaseAddr = (uint32_t)dshotBurstDmaBuffer[1]; // TODO (uint32_t)motor->timer->dmaBurstBuffer;
    //dmaInitStruct.DMA_Memory0BaseAddr = 0xCDCDCDCD;
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
    pwmWriteDshotInt(index, (uint16_t) lrintf(value));
}

void dshotWriteInt(uint8_t index, uint16_t value) {
    pwmWriteDshotInt(index, value);
}

//value range 48 - 2047
void pwmWriteDshotInt(uint8_t index, uint16_t value) {
    //motorDmaOutput_t *const motor = &dmaMotors[index];

    /*If there is a command ready to go overwrite the value and send that instead*/
//    if (dshotCommandIsProcessing()) {
//        value = dshotCommandGetCurrent(index);
//        if (value) {
//            motor->protocolControl.requestTelemetry = true;
//        }
//    }

    //motor->protocolControl.value = value;
    //uint16_t packet = prepareDshotPacket(&motor->protocolControl);
    uint16_t packet = prepareDshotPacket(value);
    //uint8_t bufferSize;

    //if (useBurstDshot) {
        //bufferSize = loadDmaBuffer(&motor->timer->dmaBurstBuffer[timerLookupChannelIndex(motor->timerHardware->channel)], 4, packet);
        //bufferSize = loadDmaBufferDshot(&dshotBurstDmaBuffer[1][index], 4, packet);

        uint32_t* dmaBuffer;
        switch (index) {
          case 0:
            // M1 -> TX2, PA2 (TIM2_CH3)
            dmaBuffer = &dshotBurstDmaBuffer[0][2];
            break;
          case 1:
            // M2 -> IO3, PB4 (TIM3_CH1)
            dmaBuffer = &dshotBurstDmaBuffer[1][0];
            break;
          case 2:
            // M3 -> IO2, PB5 (TIM3_CH2)
            dmaBuffer = &dshotBurstDmaBuffer[1][1];
            break;
          case 3:
            // M4 -> RX2, PA3 (TIM2_CH4)
            dmaBuffer = &dshotBurstDmaBuffer[0][3];
            break;
          default:
            break;
        }
        loadDmaBufferDshot(dmaBuffer, 4, packet);

        //motor->timer->dmaBurstLength = bufferSize * 4;
        //dmaBurstLengthStream2 = bufferSize * 4;
        //TODO where is dmaBurstLength needed
    //}
    //else {
    //    bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);
    //    motor->timer->timerDmaSources |= motor->timerDmaSource;
    //    DMA_SetCurrDataCounter((DMA_Stream_TypeDef *)(motor->dmaRef), bufferSize);
    //    DMA_Cmd((DMA_Stream_TypeDef *)(motor->dmaRef), ENABLE);
    //}
}

#define NVIC_PRIO_DSHOT_DMA                NVIC_BUILD_PRIORITY(2, 1)

uint16_t prepareDshotPacket(uint16_t value) {
//uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb) {
    uint16_t packet;

    //ATOMIC_BLOCK(NVIC_PRIO_DSHOT_DMA) {
            //packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);
            packet = (uint16_t) (value << 1);
            //pcb->requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row
    //}

    // compute checksum
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    // append checksum
    const uint16_t csum_masked = csum & 0x0f;
    packet = (uint16_t) (packet << 4) | csum_masked;

    return packet;
}

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       20

uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet) {
    int i;
    for (i = 0; i < 16; i++) {
        dmaBuffer[i * stride] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
        packet <<= 1;
    }
    dmaBuffer[i++ * stride] = 0;
    dmaBuffer[i++ * stride] = 0;

    return DSHOT_DMA_BUFFER_SIZE;
}

static DMA_Stream_TypeDef* dma[2] = {dmaTim2, dmaTim3};
static TIM_TypeDef* timers[2] = {TIM2, TIM3};

bool dmaIrqNotCleared;
bool dmaTimerIrqNotCleared;
#define I2C_MESSAGE_TIMEOUT     M2T(1000)

void pwmCompleteDshotMotorUpdate(void) {
//    if (!dshotCommandQueueEmpty()) {
//        if (!dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
//            return;
//        }
//    }


  xSemaphoreTake(sensorsBus.isBusFreeSemaphore, I2C_MESSAGE_TIMEOUT);

  setupDma1Stream2Channel5();

  dmaIrqNotCleared = true;
  dmaTimerIrqNotCleared = true;

  //dshotBurstDmaBuffer[0][70] = 3;
  //dshotBurstDmaBuffer[0][71] = 5;
  //dshotBurstDmaBuffer[1][68] = 7;
  //dshotBurstDmaBuffer[1][69] = 11;

  const int dmaMotorTimerCount = 2;
    for (int i = 0; i < dmaMotorTimerCount; i++) {
        //if (useBurstDshot) {
            //DMA_SetCurrDataCounter((DMA_Stream_TypeDef *)(dmaMotorTimers[i].dmaBurstRef), dmaMotorTimers[i].dmaBurstLength);
            //DMA_SetCurrDataCounter(dmaTim3, DSHOT_DMA_BUFFER_SIZE * 4);
            DMA_SetCurrDataCounter(dma[i], DSHOT_DMA_BUFFER_SIZE * 4);

            //DMA_Cmd((DMA_Stream_TypeDef *)(dmaMotorTimers[i].dmaBurstRef), ENABLE);
            //DMA_Cmd(dmaTim3, ENABLE);
            DMA_Cmd(dma[i], ENABLE);

            //TIM_DMAConfig(dmaMotorTimers[i].timer, ((uint16_t)0x000D), ((uint16_t)0x0300));
            //TIM_DMAConfig(TIM3, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
            TIM_DMAConfig(timers[i], TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);

            //TIM_DMACmd(dmaMotorTimers[i].timer, ((uint16_t)0x0100), ENABLE);
            //TIM_DMACmd(TIM3, TIM_DMA_Update, ENABLE);
            TIM_DMACmd(timers[i], TIM_DMA_Update, ENABLE);

        //TIM_SetCompare1(TIM3, 7);
        //TIM_SetCompare2(TIM3, 7);

        //}
        //else {
        //    TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, DISABLE);
        //    dmaMotorTimers[i].timer->ARR = dmaMotorTimers[i].outputPeriod;
        //    TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, ENABLE);
        //    TIM_SetCounter(dmaMotorTimers[i].timer, 0);
        //    TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
        //    dmaMotorTimers[i].timerDmaSources = 0;
        //}
    }
}

void setupDma1Stream2Channel5() {
  /// DMA
  // S2-CH5: TIM3_UP
  //DMA_Stream_TypeDef* dmaTim3 = DMA1_Stream2; //TODO check this

  DMA_Cmd(dmaTim3, DISABLE);
  DMA_DeInit(dmaTim3);

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

  // TIM3 Stream2 Channel 5
  dmaInitStruct.DMA_Channel = DMA_Channel_5;
  dmaInitStruct.DMA_Memory0BaseAddr = (uint32_t)dshotBurstDmaBuffer[1]; // TODO (uint32_t)motor->timer->dmaBurstBuffer;
  //dmaInitStruct.DMA_Memory0BaseAddr = 0xCDCDCDCD;
  dmaInitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM3->DMAR));
  DMA_Init(dmaTim3, &dmaInitStruct);

  DMA_ITConfig(dmaTim3, DMA_IT_TC, ENABLE);
}

void motor_DMA_IRQHandler() {
    if (DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2)) {

        //if (useBurstDshot) {
            //motorDmaTimer_t *burstDmaTimer = &dmaMotorTimers[descriptor->userParam];
            DMA_Cmd(DMA1_Stream2, DISABLE);
            TIM_DMACmd(TIM3, TIM_DMA_Update, DISABLE);
        //} else
        //{
        //    motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
        //    pwmChannelDMAStop(&motor->TimHandle,motor->timerHardware->channel);
        //    HAL_DMA_IRQHandler(motor->TimHandle.hdma[motor->timerDmaIndex]);
        //}

        DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
    }
}

void __attribute__((used)) DMA1_Stream1_IRQHandler(void) {
  if (DMA_GetFlagStatus(DMA1_Stream1, DMA_FLAG_TCIF1)) {
    DMA_Cmd(DMA1_Stream1, DISABLE);
    TIM_DMACmd(TIM2, TIM_DMA_Update, DISABLE);
    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
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