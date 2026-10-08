/*

modbus_flow.c - plugin for grblHAL: spindle flow monitor using a MODBUS digital input

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

Behaviour:
  - While the spindle is commanded on (and the controller is not in alarm/e-stop/
    check mode/door/sleep) a timer runs from the moment the spindle was started.
  - Once the configured start delay has elapsed, the configured modbus discrete
    input is read every "poll period" seconds.
  - If the input reports no flow, a message is sent, the spindle and coolant are
    switched off and a system alarm is raised.

*/

#if MBFLOW_ENABLE

#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/system.h"
#include "grbl/alarms.h"
#include "grbl/gcode.h"
#include "grbl/nuts_bolts.h"
#include "grbl/modbus.h"
#include "grbl/report.h"
#include "grbl/settings.h"
#include "grbl/nvs_buffer.h"
#include "modbus_flow.h"

// Alarm raised when flow is lost. Change to any alarm_code_t that suits your build.
#ifndef MBFLOW_ALARM_CODE
#define MBFLOW_ALARM_CODE Alarm_AbortCycle
#endif

// States in which monitoring is suspended
#define MBFLOW_IGNORE_STATES (STATE_ALARM|STATE_ESTOP|STATE_CHECK_MODE|STATE_SAFETY_DOOR|STATE_SLEEP)

/* Setting IDs - plain numbers like the ATCi plugin (683 - 687).
   Change these if they clash with another plugin in your build. */
#define SETTING_MBFLOW_FLAGS          690
#define SETTING_MBFLOW_DEVICE_ADDR    691
#define SETTING_MBFLOW_INPUT_ADDR     692
#define SETTING_MBFLOW_POLL_PERIOD    693
#define SETTING_MBFLOW_START_DELAY    694

typedef enum {
    MBFLOW_Result_None = 0,
    MBFLOW_Result_Flow,
    MBFLOW_Result_NoFlow,
    MBFLOW_Result_CommError
} mbflow_result_t;

static on_report_options_ptr on_report_options = NULL;
static on_execute_realtime_ptr on_execute_realtime = NULL;
static bool hooks_installed = false;

static nvs_address_t nvs_addr;

static mbflow_settings_t config;

// Written by modbus callbacks, consumed in the realtime hook
static volatile mbflow_result_t rx_result = MBFLOW_Result_None;

// Monitoring state (realtime context only)
static bool spindle_was_on = false;
static bool monitoring = false;
static bool read_pending = false;
static uint32_t spindle_start_ms = 0;
static uint32_t next_poll_ms = 0;
static uint32_t read_sent_ms = 0;
static uint16_t noflow_count = 0;
static uint16_t comm_fail_count = 0;

/*
 * Settings (same mechanism as the ATCi plugin)
 */

static const setting_detail_t plugin_settings[] = {
    { SETTING_MBFLOW_FLAGS,        Group_Coolant, "Flow Monitor", NULL, Format_XBitfield, "Enable,Invert Input", NULL, NULL, Setting_NonCore, &config.flags.value },
    { SETTING_MBFLOW_DEVICE_ADDR,  Group_Coolant, "Flow Monitor Modbus Device Address", NULL, Format_Int16, "##0", "1", "250", Setting_NonCore, &config.device_address },
    { SETTING_MBFLOW_INPUT_ADDR,   Group_Coolant, "Flow Monitor Modbus Input Address", NULL, Format_Int16, "####0", "0", "65535", Setting_NonCore, &config.input_address },
    { SETTING_MBFLOW_POLL_PERIOD,  Group_Coolant, "Flow Monitor Poll Period", "seconds", Format_Int16, "###0", "1", "3600", Setting_NonCore, &config.poll_period },
    { SETTING_MBFLOW_START_DELAY,  Group_Coolant, "Flow Monitor Start Delay", "seconds", Format_Int16, "###0", "0", "3600", Setting_NonCore, &config.start_delay },
};

/*
 * Modbus read of the digital input
 *
 * Function 0x02 (read discrete inputs), quantity 1:
 *   request  = addr, func, start MSB, start LSB, qty MSB, qty LSB, CRC x2  (8 bytes)
 *   response = addr, func, byte count, data byte, CRC x2                   (6 bytes)
 *
 * The callbacks only store the result, the decision is made in the realtime hook.
 */

static void mbflow_rx_packet (modbus_message_t *msg)
{
    if(msg->adu[1] == ModBus_ReadDiscreteInputs && msg->adu[2] >= 1) {
        bool input_high = (msg->adu[3] & 0x01) != 0;
        bool flow_ok = config.flags.invert ? !input_high : input_high;
        rx_result = flow_ok ? MBFLOW_Result_Flow : MBFLOW_Result_NoFlow;
    } else
        rx_result = MBFLOW_Result_CommError;
}

static void mbflow_rx_exception (uint8_t code, void *context)
{
    rx_result = MBFLOW_Result_CommError;
}

static void mbflow_rx_timeout (uint8_t code, void *context)
{
    rx_result = MBFLOW_Result_CommError;
}

static const modbus_callbacks_t callbacks = {
    .retries = MBFLOW_RETRIES,
    .retry_delay = MBFLOW_RETRY_DELAY,
    .on_rx_packet = mbflow_rx_packet,
    .on_rx_exception = mbflow_rx_exception,
    .on_rx_timeout = mbflow_rx_timeout
};

static bool mbflow_send_read (void)
{
    modbus_message_t cmd = {
        .context = NULL,
        .crc_check = true,
        .adu[0] = (uint8_t)config.device_address,
        .adu[1] = ModBus_ReadDiscreteInputs,
        .adu[2] = MODBUS_SET_MSB16(config.input_address),
        .adu[3] = MODBUS_SET_LSB16(config.input_address),
        .adu[4] = MODBUS_SET_MSB16(1),  // quantity MSB
        .adu[5] = MODBUS_SET_LSB16(1),  // quantity LSB
        .tx_length = 8,
        .rx_length = 6
    };

    return modbus_send(&cmd, &callbacks, false); // non-blocking, queued
}

/*
 * Halt
 */

static void mbflow_halt (const char *reason)
{
    report_message(reason, Message_Warning);

    // Make sure everything that depends on the flow is switched off, then raise the alarm
    spindle_all_off(false);
    hal.coolant.set_state((coolant_state_t){0});

    system_raise_alarm(Alarm_Spindle);
}

static void mbflow_reset_monitor (void)
{
    spindle_was_on = false;
    monitoring = false;
    read_pending = false;
    noflow_count = 0;
    comm_fail_count = 0;
    rx_result = MBFLOW_Result_None;
}

/*
 * Realtime hook: runs the whole state machine of the monitor
 */

static void mbflow_execute_realtime (sys_state_t state)
{
    if(on_execute_realtime)
        on_execute_realtime(state);

    if(!config.flags.enable || !modbus_enabled())
        return;

    uint32_t now = hal.get_elapsed_ticks(); // milliseconds

    // Only active while the spindle is running and the controller is in a state where that is meaningful
    if(gc_state.modal.spindle == NULL || !gc_state.modal.spindle->state.on || (state & MBFLOW_IGNORE_STATES)) {
        if(spindle_was_on || monitoring || read_pending)
            mbflow_reset_monitor();
        return;
    }

    // Spindle just started: begin the start delay
    if(!spindle_was_on) {
        mbflow_reset_monitor();
        spindle_was_on = true;
        spindle_start_ms = now;
        next_poll_ms = now + (uint32_t)config.start_delay * 1000UL;
    }

    // Start delay elapsed?
    if(!monitoring) {
        if((int32_t)(now - next_poll_ms) >= 0) {
            monitoring = true;
            read_pending = false;
            rx_result = MBFLOW_Result_None; // discard anything left over from before the delay
        } else
            return;
    }

    // Consume a result from the last read
    mbflow_result_t result = rx_result;

    if(read_pending && result == MBFLOW_Result_None && (now - read_sent_ms) > MBFLOW_RESPONSE_GUARD)
        result = MBFLOW_Result_CommError; // callback never arrived

    if(result != MBFLOW_Result_None) {

        rx_result = MBFLOW_Result_None;
        read_pending = false;

        switch(result) {

            case MBFLOW_Result_Flow:
                noflow_count = 0;
                comm_fail_count = 0;
                break;

            case MBFLOW_Result_NoFlow:
                comm_fail_count = 0;
                if(++noflow_count >= MBFLOW_NOFLOW_READS) {
                    
                    system_raise_alarm(Alarm_Spindle);
                    return;
                }
                break;

            case MBFLOW_Result_CommError:
#if MBFLOW_COMM_FAIL_READS > 0
                if(++comm_fail_count >= MBFLOW_COMM_FAIL_READS) {
                    mbflow_halt("Flow monitor modbus comms failed - halting");
                    return;
                }
#else
                comm_fail_count++;
#endif
                break;

            default:
                break;
        }
    }

    // Time for the next read?
    if(!read_pending && (int32_t)(now - next_poll_ms) >= 0 && !modbus_isbusy()) {

        next_poll_ms = now + (uint32_t)config.poll_period * 1000UL;

        if(mbflow_send_read()) {
            read_pending = true;
            read_sent_ms = now;
        } else
            rx_result = MBFLOW_Result_CommError; // could not queue the message
    }
}

static void mbflow_report_options (bool newopt)
{
    if(on_report_options)
        on_report_options(newopt);

    if(!newopt)
        report_plugin("Modbus Flow Monitor", "1.00");
}

/* --- Settings save / restore / load --- */

static void mbflow_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void mbflow_restore (void)
{
    config.flags.value = 0;     // disabled by default, user must enable
    config.device_address = 1;
    config.input_address = 0;
    config.poll_period = 1;
    config.start_delay = 10;

    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void mbflow_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        mbflow_restore();

    // Guard against out of range values in NVS
    if(config.poll_period == 0)
        config.poll_period = 1;

    spindle_was_on = false;
    monitoring = false;
    read_pending = false;

    /* Hook handlers only once */
    if(!hooks_installed) {
        hooks_installed = true;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = mbflow_report_options;

        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = mbflow_execute_realtime;
    }
}

/* --- Init --- */

void mbflow_init (void)
{
    static setting_details_t settings = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .load = mbflow_load,
        .save = mbflow_save,
        .restore = mbflow_restore
    };

    // Not gated on modbus_enabled() here: the modbus driver may not be registered yet at this point.
    // modbus_enabled() is checked at run time in the realtime hook instead.
    if((nvs_addr = nvs_alloc(sizeof(config)))) {
        settings_register(&settings);
        report_message("Modbus flow monitor plugin v1.00 initialized", Message_Info);
    } else
        report_message("Modbus flow monitor plugin: NVS allocation failed, settings not available", Message_Warning);
}

#endif // MBFLOW_ENABLE
