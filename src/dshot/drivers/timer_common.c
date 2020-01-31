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

#include "platform.h"

#include "drivers/dshot_bitbang.h"
#include "drivers/io.h"
#include "timer.h"

const timerHardware_t *timerGetByTag(ioTag_t ioTag)
{
    for (unsigned i = 0; i < TIMER_CHANNEL_COUNT; i++) {
        if (TIMER_HARDWARE[i].tag == ioTag) {
            return &TIMER_HARDWARE[i];
        }
    }
    return NULL;
}

const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex)
{
    UNUSED(owner);
    UNUSED(resourceIndex);

    return timerGetByTag(ioTag);
}

ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index)
{
#if !defined(USE_UNIFIED_TARGET) && USABLE_TIMER_CHANNEL_COUNT > 0
    uint8_t currentIndex = 0;
    for (unsigned i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if ((timerHardware[i].usageFlags & usageFlag) == usageFlag) {
            if (currentIndex == index) {
                return timerHardware[i].tag;
            }
            currentIndex++;
        }
    }
#else
    UNUSED(usageFlag);
    UNUSED(index);
#endif
    return IO_TAG_NONE;
}
