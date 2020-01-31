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

#pragma once

#include "common/time.h"

#define DSHOT_MIN_THROTTLE       48
#define DSHOT_MAX_THROTTLE     2047
#define DSHOT_3D_FORWARD_MIN_THROTTLE 1048

#define MIN_GCR_EDGES         7
#define MAX_GCR_EDGES         22

// comment out to see frame dump of corrupted frames in dshot_telemetry_info
//#define DEBUG_BBDECODE

typedef struct dshotProtocolControl_s {
    uint16_t value;
    bool requestTelemetry;
} dshotProtocolControl_t;

void dshotInitEndpoints(float outputLimit, float *outputLow, float *outputHigh, float *disarm, float *deadbandMotor3dHigh, float *deadbandMotor3dLow);
float dshotConvertFromExternal(uint16_t externalValue);
uint16_t dshotConvertToExternal(float motorValue);

uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb);

uint16_t getDshotTelemetry(uint8_t index);
bool isDshotMotorTelemetryActive(uint8_t motorIndex);
bool isDshotTelemetryActive(void);

int16_t getDshotTelemetryMotorInvalidPercent(uint8_t motorIndex);
