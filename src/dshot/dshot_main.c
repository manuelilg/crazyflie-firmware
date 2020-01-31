//
// Created by manuel on 22.01.20.
//

#include <stdint.h>
#include <stdbool.h>

#include "motor.h"

#include "pg/motor.h"
#include "dshot_bitbang.h"


#define BRUSHLESS_MOTORS_PWM_RATE 480
#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON

motorConfig_t motorConfig;

void init_dshot() {

// from pg/motor.c
    motorConfig.minthrottle = 1070;
    motorConfig.dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
//motorConfig->dev.motorPwmProtocol = PWM_TYPE_ONESHOT125;

    motorConfig.maxthrottle = 2000;
    motorConfig.mincommand = 1000;
    motorConfig.digitalIdleOffsetValue = 550;

    motorConfig.dev.useBurstDshot = ENABLE_DSHOT_DMAR;

    for (int motorIndex = 0; motorIndex < 4; motorIndex++) {
        motorConfig.dev.ioTags[motorIndex] = timerioTagGetByUsage(TIM_USE_MOTOR, motorIndex);
    }

    motorConfig.motorPoleCount = 14;   // Most brushes motors that we use are 14 poles


//motorDevConfig_t dev;
//motorConfig->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
    motorConfig.dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    motorConfig.dev.motorPwmInversion = 0;
    motorConfig.dev.useUnsyncedPwm = false;
//motorConfig.dev.useBurstDshot = DSHOT_DMAR_ON;
    motorConfig.dev.useDshotTelemetry = false;

    motorConfig.dev.motorTransportProtocol = 0; // not used anywhere

    motorConfig.dev.useDshotBitbang = DSHOT_BITBANG_OFF;
    motorConfig.dev.useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_AUTO;

    uint16_t idlePulse = motorConfig.mincommand;

    motorDevInit(&motorConfig.dev, idlePulse, 4);
//systemState |= SYSTEM_STATE_MOTORS_READY;

motorPostInit();
motorEnable();




}
