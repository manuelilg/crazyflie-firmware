//
// Created by manuel on 22.01.20.
//
//#define USABLE_TIMER_CHANNEL_COUNT 12

#include "dshot_main.h"



#include "motor.h"

//#include "pg/motor.h"
#include "dshot_bitbang.h" // has only enum active

#include "dshot/drivers/timer.h"
//#include "dshot/drivers/timer_def.h"
//#include "dshot/drivers/io_def.h"

#define BRUSHLESS_MOTORS_PWM_RATE 480
#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON

const timerHardware_t timerHardware[4 /*USABLE_TIMER_CHANNEL_COUNT*/] = {
//        // TIM2_UP  (DMA1_ST1_CH3, DMA1_ST7_CH3)
//        DEF_TIM(TIM2,  CH3,  PA2,  TIM_USE_MOTOR,               0, 0), // M1
//        DEF_TIM(TIM2,  CH4,  PA3,  TIM_USE_MOTOR,               0, 0), // M4
//        // TIM3_UP (DMA1_ST2_CH5)
//        DEF_TIM(TIM3,  CH1,  PB4,  TIM_USE_MOTOR,               0, 0), // M2
//        DEF_TIM(TIM3,  CH2,  PB5,  TIM_USE_MOTOR,               0, 0), // M3

        { ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000)), ((ioTag_t)((((0) + 1) << 4) | (2))), ((uint16_t)0x0008), TIM_USE_MOTOR, (TIMER_OUTPUT_NONE | 0), ((uint8_t)0x01) , (dmaResource_t *)((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6000) + 0x028)), ((uint32_t)0x06000000) , (dmaResource_t *)((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6000) + 0x0B8)), ((uint32_t)0x06000000), DMA1_ST7_HANDLER },
        { ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)), ((ioTag_t)((((1) + 1) << 4) | (4))), ((uint16_t)0x0000), TIM_USE_MOTOR, (TIMER_OUTPUT_NONE | 0), ((uint8_t)0x02) , (dmaResource_t *)((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6000) + 0x070)), ((uint32_t)0x0A000000) , (dmaResource_t *)((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6000) + 0x040)), ((uint32_t)0x0A000000), DMA1_ST2_HANDLER },
        { ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0400)), ((ioTag_t)((((1) + 1) << 4) | (5))), ((uint16_t)0x0004), TIM_USE_MOTOR, (TIMER_OUTPUT_NONE | 0), ((uint8_t)0x02) , (dmaResource_t *)((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6000) + 0x088)), ((uint32_t)0x0A000000) , (dmaResource_t *)((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6000) + 0x040)), ((uint32_t)0x0A000000), DMA1_ST2_HANDLER },
        { ((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000)), ((ioTag_t)((((0) + 1) << 4) | (3))), ((uint16_t)0x000C), TIM_USE_MOTOR, (TIMER_OUTPUT_NONE | 0), ((uint8_t)0x01) , (dmaResource_t *)((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6000) + 0x0B8)), ((uint32_t)0x06000000) , (dmaResource_t *)((DMA_Stream_TypeDef *) (((((uint32_t)0x40000000) + 0x00020000) + 0x6000) + 0x0B8)), ((uint32_t)0x06000000), DMA1_ST7_HANDLER },
};


//motorConfig_t motorConfig;

void init_dshot() {

// from pg/motor.c
    motorConfig_t* motorConfig = motorConfigMutable();

    motorConfig->minthrottle = 1070;
    motorConfig->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
//motorConfig->dev.motorPwmProtocol = PWM_TYPE_ONESHOT125;

    motorConfig->maxthrottle = 2000;
    motorConfig->mincommand = 1000;
    motorConfig->digitalIdleOffsetValue = 550;

    motorConfig->dev.useBurstDshot = ENABLE_DSHOT_DMAR;

    //for (int motorIndex = 0; motorIndex < 4; motorIndex++) {
    //    motorConfig->dev.ioTags[motorIndex] = timerioTagGetByUsage(TIM_USE_MOTOR, motorIndex);
    //}
    //ioTag_t is a uint8_t
    // !!!!! different Order for Motors between crazyflie and beatflight
    motorConfig->dev.ioTags[0] = ((ioTag_t)((((0) + 1) << 4) | (2))); // PA2, TIM2 CH3
    motorConfig->dev.ioTags[1] = ((ioTag_t)((((1) + 1) << 4) | (4))); // PB4, TIM3 CH1
    motorConfig->dev.ioTags[2] = ((ioTag_t)((((1) + 1) << 4) | (5))); // PB5, TIM3 CH2
    motorConfig->dev.ioTags[3] = ((ioTag_t)((((0) + 1) << 4) | (3))); // PA3, TIM2 CH4

    motorConfig->motorPoleCount = 14;   // Most brushes motors that we use are 14 poles


    //motorDevConfig_t dev;
    //motorConfig->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
    motorConfig->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    motorConfig->dev.motorPwmInversion = 0;
    motorConfig->dev.useUnsyncedPwm = false;
    //motorConfig.dev.useBurstDshot = DSHOT_DMAR_ON;
    motorConfig->dev.useDshotTelemetry = false;

    motorConfig->dev.motorTransportProtocol = 0; // not used anywhere

    motorConfig->dev.useDshotBitbang = DSHOT_BITBANG_OFF;
    motorConfig->dev.useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_AUTO;

    uint16_t idlePulse = motorConfig->mincommand;

    motorDevInit(&motorConfig->dev, idlePulse, 4);
    //systemState |= SYSTEM_STATE_MOTORS_READY;

    motorPostInit();
    motorEnable();




}
