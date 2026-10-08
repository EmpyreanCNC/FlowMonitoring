/*

modbus_flow.h - plugin for grblHAL: spindle flow monitor using a MODBUS digital input

This plugin is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This plugin is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this plugin.  If not, see <http://www.gnu.org/licenses/>.

*/
#ifndef _MBFLOW_H_
#define _MBFLOW_H_

#include <stdint.h>
#include <stdbool.h>

#define MBFLOW_RETRIES          3   // modbus retries per read
#define MBFLOW_RETRY_DELAY      100 // ms between retries
#define MBFLOW_NOFLOW_READS     1   // consecutive "no flow" reads required before halting
#define MBFLOW_COMM_FAIL_READS  3   // consecutive failed reads (timeout/exception) before halting, 0 = never halt on comms failure
#define MBFLOW_RESPONSE_GUARD   2000 // ms, give up waiting for a read callback after this long

typedef union {
    uint8_t value;
    struct {
        uint8_t enable :1,  // plugin enabled
                invert :1,  // 0: input high = flow OK, 1: input low = flow OK
                unused :6;
    };
} mbflow_flags_t;

typedef struct {
    mbflow_flags_t flags;
    uint16_t device_address;    // modbus device (server) address, 1 - 250
    uint16_t input_address;     // digital input (discrete input) address on the device
    uint16_t poll_period;       // seconds between reads while monitoring
    uint16_t start_delay;       // seconds after spindle start before monitoring begins
} mbflow_settings_t;

void mbflow_init (void);

#endif
