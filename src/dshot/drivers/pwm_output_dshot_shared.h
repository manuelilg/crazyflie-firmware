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


typedef DMA_Stream_TypeDef dmaStream_t;

extern FAST_RAM_ZERO_INIT uint8_t dmaMotorTimerCount;
extern motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
extern motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

extern uint32_t readDoneCount;

typedef struct dshotDMAHandlerCycleCounters_s {
    uint32_t irqAt;
    uint32_t changeDirectionCompletedAt;
} dshotDMAHandlerCycleCounters_t;

FAST_RAM_ZERO_INIT extern dshotDMAHandlerCycleCounters_t dshotDMAHandlerCycleCounters;


uint8_t getTimerIndex(TIM_TypeDef *timer);
motorDmaOutput_t *getMotorDmaOutput(uint8_t index);
void dshotEnableChannels(uint8_t motorCount);

void pwmDshotSetDirectionOutput(motorDmaOutput_t * const motor, TIM_OCInitTypeDef *pOcInit, DMA_InitTypeDef* pDmaInit);

bool pwmStartDshotMotorUpdate(void);
