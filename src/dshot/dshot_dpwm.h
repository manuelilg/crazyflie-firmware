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
 *
 * Author: jflyper
 */

#pragma once

#include "drivers/motor.h"

#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       20

#define MOTOR_PROSHOT1000_HZ         MHZ_TO_HZ(24)
#define PROSHOT_BASE_SYMBOL          24 // 1uS
#define PROSHOT_BIT_WIDTH            3
#define MOTOR_NIBBLE_LENGTH_PROSHOT  (PROSHOT_BASE_SYMBOL * 4) // 4uS

#define DSHOT_TELEMETRY_DEADTIME_US   (30 + 5) // 30 to switch lines and 5 to switch lines back


typedef uint8_t loadDmaBufferFn(uint32_t *dmaBuffer, int stride, uint16_t packet);  // function pointer used to encode a digital motor value into the DMA buffer representation
extern FAST_RAM_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;
uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet);
uint8_t loadDmaBufferProshot(uint32_t *dmaBuffer, int stride, uint16_t packet);

uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType);

struct motorDevConfig_s;
motorDevice_t *dshotPwmDevInit(const struct motorDevConfig_s *motorConfig, uint16_t idlePulse, uint8_t motorCount, bool useUnsyncedPwm);

/* Motor DMA related, moved from pwm_output.h */

#define MAX_DMA_TIMERS        8

#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */
#define PROSHOT_DMA_BUFFER_SIZE 6  /* resolution + frame reset (2us) */

#define GCR_TELEMETRY_INPUT_LEN MAX_GCR_EDGES

#define DSHOT_DMA_BUFFER_ATTRIBUTE

#define DSHOT_DMA_BUFFER_UNIT uint32_t

#define DSHOT_DMA_BUFFER_ALLOC_SIZE DSHOT_DMA_BUFFER_SIZE

extern DSHOT_DMA_BUFFER_UNIT dshotDmaBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
extern DSHOT_DMA_BUFFER_UNIT dshotDmaInputBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];

extern DSHOT_DMA_BUFFER_UNIT dshotBurstDmaBuffer[MAX_DMA_TIMERS][DSHOT_DMA_BUFFER_SIZE * 4];

typedef struct {
    TIM_TypeDef *timer;
    uint16_t outputPeriod;
    dmaResource_t *dmaBurstRef;
    uint16_t dmaBurstLength;
    uint32_t *dmaBurstBuffer;
    uint16_t timerDmaSources;
} motorDmaTimer_t;

typedef struct motorDmaOutput_s {
    dshotProtocolControl_t protocolControl;
    ioTag_t ioTag;
    const timerHardware_t *timerHardware;
    uint16_t timerDmaSource;
    uint8_t timerDmaIndex;
    bool configured;
    uint8_t output;
    uint8_t index;
    uint32_t iocfg;
    DMA_InitTypeDef   dmaInitStruct;

    dmaResource_t *dmaRef;

    motorDmaTimer_t *timer;
    DSHOT_DMA_BUFFER_UNIT *dmaBuffer;
} motorDmaOutput_t;

motorDmaOutput_t *getMotorDmaOutput(uint8_t index);

bool isMotorProtocolDshot(void);

void pwmWriteDshotInt(uint8_t index, uint16_t value);
bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output);

void pwmCompleteDshotMotorUpdate(void);

extern bool useBurstDshot;

extern motorDevice_t dshotPwmDevice;
