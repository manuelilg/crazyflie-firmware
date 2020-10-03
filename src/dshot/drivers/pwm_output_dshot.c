/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"


//#include "build/debug.h"

#include "drivers/dma.h"
//#include "drivers/dma_reqmap.h"
//#include "drivers/io.h"

//#include "drivers/nvic.h"
//#include "drivers/rcc.h"
enum rcc_reg {
    RCC_EMPTY = 0,
    RCC_AHB,
    RCC_APB2,
    RCC_APB1,
    RCC_AHB1,
};

//#include "drivers/time.h"
#include "dshot/drivers/timer.h"
//#include "drivers/system.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"


//#include "pwm_output.h"
#include "dshot/dshot.h"
#include "dshot/dshot_dpwm.h"
#include "dshot/dshot_command.h"

#include "dshot/drivers/pwm_output_dshot_shared.h"


#include "cfassert.h" //MI

/*
FAST_CODE void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
)
{
    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;

    dmaResource_t *dmaRef = motor->dmaRef;

    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
    }

    xDMA_DeInit(dmaRef);

    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Disable);
    timerOCInit(timer, timerHardware->channel, pOcInit);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);

    if (useBurstDshot) {
        pDmaInit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    } else
    {
        pDmaInit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    }

    xDMA_Init(dmaRef, pDmaInit);
    xDMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);
}//*/

/*
void pwmCompleteDshotMotorUpdate(void)
{
    // If there is a dshot command loaded up, time it correctly with motor update
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
            return;
        }
    }

    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (useBurstDshot) {
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, ENABLE);
            TIM_DMAConfig(dmaMotorTimers[i].timer, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
            TIM_DMACmd(dmaMotorTimers[i].timer, TIM_DMA_Update, ENABLE);
        } else
        {
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, DISABLE);
            dmaMotorTimers[i].timer->ARR = dmaMotorTimers[i].outputPeriod;
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, ENABLE);
            TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}//*/

/*
FAST_CODE
static void motor_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];
        if (useBurstDshot) {
            xDMA_Cmd(motor->timerHardware->dmaTimUPRef, DISABLE);
            TIM_DMACmd(motor->timerHardware->tim, TIM_DMA_Update, DISABLE);
        } else
        {
            xDMA_Cmd(motor->dmaRef, DISABLE);
            TIM_DMACmd(motor->timerHardware->tim, motor->timerDmaSource, DISABLE);
        }

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}//*/

//* used by dshot_dpwm::dshotPwmDevInit
bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    TIM_OCInitTypeDef ocInitStruct;
    DMA_InitTypeDef   dmaInitStruct;
#define OCINIT ocInitStruct
#define DMAINIT dmaInitStruct

    dmaResource_t *dmaRef = NULL;
    uint32_t dmaChannel = 0;
#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec != NULL) {
        dmaRef = dmaSpec->ref;
        dmaChannel = dmaSpec->channel;
    }
#else
    dmaRef = timerHardware->dmaRef;
    dmaChannel = timerHardware->dmaChannel;
#endif

    if (useBurstDshot) {
        dmaRef = timerHardware->dmaTimUPRef;
        dmaChannel = timerHardware->dmaTimUPChannel;
    }

    if (dmaRef == NULL) {
        return false;
    }

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    TIM_TypeDef *timer = timerHardware->tim;

    // Boolean configureTimer is always true when different channels of the same timer are processed in sequence,
    // causing the timer and the associated DMA initialized more than once.
    // To fix this, getTimerIndex must be expanded to return if a new timer has been requested.
    // However, since the initialization is idempotent, it is left as is in a favor of flash space (for now).
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount-1);

    motor->timer = &dmaMotorTimers[timerIndex];
    motor->index = motorIndex;
    motor->timerHardware = timerHardware;

    //const IO_t motorIO = IOGetByTag(timerHardware->tag);
    // pointer to ioRec_t entry (filled by IOInit(..) earlier)
    //const void* motorIO;
    /* typedef struct ioRec_s {
        GPIO_TypeDef *gpio;
        uint16_t pin;
        resourceOwner_e owner;
        uint8_t index;
    } ioRec_t; */

    uint8_t pupMode = 0;
    pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP;

    //motor->iocfg = IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, pupMode);
    motor->iocfg = ((GPIO_Mode_AF) | ((GPIO_Fast_Speed) << 2) | ((GPIO_OType_PP) << 4) | ((pupMode) << 5));

    //IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);
    //void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
    ioConfig_t cfg = motor->iocfg;
    {
//        if (!io) {
//            return;
//        }

        //const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
        //RCC_ClockCmd(rcc, ENABLE);
        {
            switch(motorIndex) {
                case 0: case 3:
                    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); break;
                case 1: case 2:
                    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); break;
            }

            //RCC_AHB1PeriphClockCmd(mask, ENABLE);
        }

        uint16_t gpio_pin = 0;
        uint16_t pin_source = 0;
        GPIO_TypeDef* gpio;
        switch(motorIndex) {
            case 0:
                gpio_pin = GPIO_Pin_2; // PA2
                pin_source = GPIO_PinSource2;
                gpio = GPIOA;
                break;
            case 3:
                gpio_pin = GPIO_Pin_3; // PA3
                pin_source = GPIO_PinSource3;
                gpio = GPIOA;
                break;
            case 1:
                gpio_pin = GPIO_Pin_4; // PB4
                pin_source = GPIO_PinSource4;
                gpio = GPIOB;
                break;
            case 2:
                gpio_pin = GPIO_Pin_5; // PB5
                pin_source = GPIO_PinSource5;
                gpio = GPIOB;
                break;
        }

        GPIO_PinAFConfig(gpio, pin_source, timerHardware->alternateFunction); //GPIO_PinAFConfig(IO_GPIO(io), IO_GPIO_PinSource(io), af);

        GPIO_InitTypeDef init = {
                .GPIO_Pin = gpio_pin, //.GPIO_Pin = IO_Pin(io),
                .GPIO_Mode = (cfg >> 0) & 0x03,
                .GPIO_Speed = (cfg >> 2) & 0x03,
                .GPIO_OType = (cfg >> 4) & 0x01,
                .GPIO_PuPd = (cfg >> 5) & 0x03,
        };
        GPIO_Init(gpio, &init);//GPIO_Init(IO_GPIO(io), &init);
    }

    if (configureTimer) {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

        //RCC_ClockCmd(timerRCC(timer), ENABLE);
//        rccPeriphTag_t timerRCC(TIM_TypeDef *tim) {
//            for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
//                if (timerDefinitions[i].TIMx == tim) {
//                    return timerDefinitions[i].rcc;
//                }
//            }
//            return 0;
//        }

        rccPeriphTag_t periphTag;
        for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
            if (timerDefinitions[i].TIMx == timer) {
                periphTag = timerDefinitions[i].rcc;
            }
        }

        //void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
        {
            int tag = periphTag >> 5;
            uint32_t mask = 1 << (periphTag & 0x1f);
                switch (tag) {
                case RCC_APB2:
                    RCC_APB2PeriphClockCmd(mask, ENABLE);
                    break;
                case RCC_APB1:
                    RCC_APB1PeriphClockCmd(mask, ENABLE);
                    break;
                case RCC_AHB1:
                    RCC_AHB1PeriphClockCmd(mask, ENABLE);
                    break;
                }
        }


        TIM_Cmd(timer, DISABLE);

        //TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        //uint32_t timerClock(TIM_TypeDef *tim)
        {
            TIM_TypeDef* const tim = timer;
            uint32_t timer_clock;
            if (tim == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x00010000) + 0x0400)) || tim == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x00010000) + 0x0000)) || tim == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x00010000) + 0x4000)) || tim == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x00010000) + 0x4400)) || tim == ((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x00010000) + 0x4800))) {
                timer_clock = SystemCoreClock;
            } else {
                timer_clock = SystemCoreClock / 2;
            }
            TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(lrintf((float) timer_clock / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        }

        TIM_TimeBaseStructure.TIM_Period = (pwmProtocolType == PWM_TYPE_PROSHOT1000 ? (MOTOR_NIBBLE_LENGTH_PROSHOT) : MOTOR_BITLENGTH) - 1;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
    }

    TIM_OCStructInit(&OCINIT);
    OCINIT.TIM_OCMode = TIM_OCMode_PWM1;
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        OCINIT.TIM_OutputNState = TIM_OutputNState_Enable;
        OCINIT.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        OCINIT.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        OCINIT.TIM_OutputState = TIM_OutputState_Enable;
        OCINIT.TIM_OCIdleState = TIM_OCIdleState_Set;
        OCINIT.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    OCINIT.TIM_Pulse = 0;

    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;
    } else {
        ASSERT(false);
        //motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        //motor->timer->timerDmaSources &= ~motor->timerDmaSource;
    }

    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);
    DMA_StructInit(&DMAINIT);

    if (useBurstDshot) {
        dmaInit(timerHardware->dmaTimUPIrqHandler, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim));

        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];

        DMAINIT.DMA_Channel = timerHardware->dmaTimUPChannel;
        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->timer->dmaBurstBuffer;
        DMAINIT.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMAINIT.DMA_FIFOMode = DMA_FIFOMode_Enable;
        DMAINIT.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        DMAINIT.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMAINIT.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)&timerHardware->tim->DMAR;
        DMAINIT.DMA_BufferSize = (pwmProtocolType == PWM_TYPE_PROSHOT1000) ? PROSHOT_DMA_BUFFER_SIZE : DSHOT_DMA_BUFFER_SIZE; // XXX
        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMAINIT.DMA_Mode = DMA_Mode_Normal;
        DMAINIT.DMA_Priority = DMA_Priority_High;
    } else {
        ASSERT(false);
//        dmaInit(dmaGetIdentifier(dmaRef), OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
//
//        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];
//
//        DMAINIT.DMA_Channel = dmaChannel;
//        DMAINIT.DMA_Memory0BaseAddr = (uint32_t)motor->dmaBuffer;
//        DMAINIT.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//        DMAINIT.DMA_FIFOMode = DMA_FIFOMode_Enable;
//        DMAINIT.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//        DMAINIT.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//        DMAINIT.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//        DMAINIT.DMA_PeripheralBaseAddr = (uint32_t)timerChCCR(timerHardware);
//        DMAINIT.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//        DMAINIT.DMA_MemoryInc = DMA_MemoryInc_Enable;
//        DMAINIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
//        DMAINIT.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
//        DMAINIT.DMA_Mode = DMA_Mode_Normal;
//        DMAINIT.DMA_Priority = DMA_Priority_High;
    }

    // XXX Consolidate common settings in the next refactor

    motor->dmaRef = dmaRef;
/*

    pwmDshotSetDirectionOutput(motor, &OCINIT, &DMAINIT);

    if (useBurstDshot) {
        dmaSetHandler(timerHardware->dmaTimUPIrqHandler, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    } else
    {
        dmaSetHandler(dmaGetIdentifier(dmaRef), motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motor->index);
    }

    TIM_Cmd(timer, ENABLE);
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_CCxNCmd(timer, timerHardware->channel, TIM_CCxN_Enable);
    } else {
        TIM_CCxCmd(timer, timerHardware->channel, TIM_CCx_Enable);
    }
    if (configureTimer) {
        TIM_ARRPreloadConfig(timer, ENABLE);
        TIM_CtrlPWMOutputs(timer, ENABLE);
        TIM_Cmd(timer, ENABLE);
    }

    motor->configured = true;
    */
    return true;
}


