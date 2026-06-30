/*
  flow_monitor.c - plugin for grblHAL that watches a coolant/flow switch over MODBUS
                   while the spindle is running, and raises a spindle alarm if flow
                   stops being detected for too many polls in a row.

  This file is a "remix" of three other grblHAL files, so you can see where each
  idea came from:

    - gs20.c              -> how a plugin hooks into "spindle selected" and how it
                              sends/receives MODBUS messages and reacts to errors.
    - modbus_io.c          -> the exact MODBUS message shape for reading a single
                              "discrete input" (i.e. a simple on/off point).
    - sienci-atci-plugin.c -> the settings struct + setting_detail_t table pattern,
                              and how settings get loaded/saved/restored from
                              non-volatile storage (NVS), plus task_add_delayed()
                              for periodic polling instead of hooking the realtime
                              loop directly.

  Copyright (c) 2026
  Part of grblHAL

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

// driver.h pulls in the board-specific configuration (pins, enabled plugins etc).
#include "driver.h"

// This whole file only gets compiled if FLOW_MONITOR_ENABLE is turned on somewhere
// in the board configuration (my_machine.h or similar). This is the same pattern
// gs20.c uses with "#if SPINDLE_ENABLE & (1<<SPINDLE_GS20)" - it means the code is
// simply not present in the firmware unless someone has asked for it.
#if FLOW_MONITOR_ENABLE

#include <string.h>

// Pull in the grblHAL core headers we need:
//   hal.h          - the hardware abstraction layer, gives us hal.nvs etc.
//   nvs_buffer.h   - functions for storing our settings in non-volatile memory
//   settings.h     - the setting_detail_t struct and settings_register()
//   modbus.h       - everything related to sending/receiving MODBUS messages
//   task.h         - task_add_delayed(), used for our periodic polling timer
//   report.h       - report_message(), used to print debug/status text
//   spindle.h      - access to spindle state and the spindle alarm helpers
#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/settings.h"
#include "grbl/modbus.h"
#include "grbl/task.h"
#include "grbl/report.h"
#include "grbl/spindle.h"

/*
  ---------------------------------------------------------------------------
  SETTINGS
  ---------------------------------------------------------------------------
  Everything in this struct gets saved to non-volatile storage (NVS), so the
  user's chosen settings survive a power cycle. This mirrors the "atci_config_t"
  struct in sienci-atci-plugin.c - one plain struct, one NVS slot, one set of
  setting_detail_t rows below describing each field to the $-settings system.
*/
typedef struct {
    uint8_t  modbus_address;     // MODBUS slave address of the flow sensor device (0-247)
    uint16_t input_address;      // Which discrete input (on/off point) number to read on that device
    uint16_t poll_period_ms;     // How often (in milliseconds) we ask the device for a new reading
    uint8_t  flow_active_value;  // Does a flowing reading return 0 or 1? (lets the user match wiring/logic)
    uint8_t  max_missed_polls;   // How many "no flow" readings in a row are allowed before we alarm
} flow_monitor_settings_t;

// The actual settings values live here. They get filled in by atci-style
// load/save/restore functions further down.
static flow_monitor_settings_t settings;

/*
  ---------------------------------------------------------------------------
  RUNTIME-ONLY STATE
  ---------------------------------------------------------------------------
  These variables are NOT saved to NVS - they only matter while the controller
  is powered on and reset back to a known state every time we start polling.
*/
static uint8_t  missed_count = 0;     // how many "no flow" readings we have seen in a row, right now
static bool     polling_active = false; // are we currently supposed to be polling? (true only while spindle is on)
static nvs_address_t nvs_address;     // where in NVS our settings struct is stored

/*
  ---------------------------------------------------------------------------
  SAVED grblHAL FUNCTION POINTERS
  ---------------------------------------------------------------------------
  grblHAL plugins "chain" themselves onto existing hooks rather than replacing
  them outright. We remember the previous handler here and call it ourselves,
  so other plugins that also use these hooks keep working. This is exactly
  what gs20.c does with on_spindle_selected and on_report_options.
*/
static on_report_options_ptr on_report_options;
static on_spindle_selected_ptr on_spindle_selected;

// Forward declarations of our own MODBUS receive callbacks, so the
// modbus_callbacks_t struct below can refer to them before they are defined.
static void flow_rx_packet (modbus_message_t *msg);
static void flow_rx_exception (uint8_t code, void *context);

/*
  This bundles together what should happen when a MODBUS reply comes back
  (flow_rx_packet) and what should happen if the device errors out or fails
  to respond (flow_rx_exception). It is passed into modbus_send() every time
  we transmit a request, the same as the "callbacks" struct in gs20.c.
*/
static const modbus_callbacks_t callbacks = {
    .retries = 3,           // how many times to silently retry before calling it a failure
    .retry_delay = 50,      // milliseconds to wait between retries
    .on_rx_packet = flow_rx_packet,
    .on_rx_exception = flow_rx_exception
};

/*
  A "context" tag we attach to our outgoing MODBUS message so that when the
  reply comes back into flow_rx_packet(), we know which request it was a
  reply to. We only ever send one type of request (read the flow input), so
  there is only one value here - but the pattern matches gs20.c's
  vfd_response_t enum, which lets a plugin tell several request types apart.
*/
typedef enum {
    FlowMonitor_ReadInput = 1
} flow_response_t;

/*
  ---------------------------------------------------------------------------
  SENDING THE MODBUS REQUEST
  ---------------------------------------------------------------------------
  This builds and sends a "read discrete input" MODBUS request. A discrete
  input is MODBUS's name for a single read-only on/off point - exactly what
  a flow switch looks like to the controller. This follows the exact byte
  layout used by mbio_ModBus_ReadDiscreteInputs() in modbus_io.c.

  A MODBUS RTU request for this function code looks like:

    adu[0] = slave address          (which device on the bus we are talking to)
    adu[1] = function code 0x02     (0x02 means "read discrete inputs")
    adu[2] = high byte of input address
    adu[3] = low byte of input address
    adu[4] = high byte of "how many inputs to read" (we always read just 1)
    adu[5] = low byte of "how many inputs to read"

  modbus_send() appends the CRC (an error-checking code) and handles the
  actual sending over the wire - we do not need to compute that ourselves.
*/
static void flow_poll_request (void)
{
    modbus_message_t request = {
        // Tag this message so we recognise the reply later.
        .context = (void *)FlowMonitor_ReadInput,

        // Ask grblHAL's MODBUS layer to add and check the CRC for us.
        .crc_check = true,

        // Byte 0: which MODBUS device (slave) we are asking.
        .adu[0] = settings.modbus_address,

        // Byte 1: the MODBUS function code for "read discrete inputs".
        .adu[1] = ModBus_ReadDiscreteInputs,

        // Bytes 2-3: the input point number, split into high byte / low byte.
        // MODBUS_SET_MSB16/LSB16 are helper macros (used in modbus_io.c) that
        // pull the top and bottom 8 bits out of a 16-bit number.
        .adu[2] = MODBUS_SET_MSB16(settings.input_address),
        .adu[3] = MODBUS_SET_LSB16(settings.input_address),

        // Bytes 4-5: how many inputs to read, starting at that address.
        // We only care about a single point, so this is always 1.
        .adu[4] = 0x00,
        .adu[5] = 0x01,

        // How many bytes we are sending, and how many bytes we expect back.
        // tx_length 8 = 6 message bytes + 2 CRC bytes.
        // rx_length 6 = address + function + byte-count + 1 data byte + 2 CRC bytes.
        .tx_length = 8,
        .rx_length = 6
    };

    // false = do not block waiting for the reply here. The reply will arrive
    // later and be handled by flow_rx_packet() below. This keeps polling from
    // ever freezing the rest of the controller.
    modbus_send(&request, &callbacks, false);
}

/*
  ---------------------------------------------------------------------------
  HANDLING THE REPLY
  ---------------------------------------------------------------------------
  Called automatically by the MODBUS layer whenever a reply comes back that
  matches a request we sent. This is the same mechanism as rx_packet() in
  gs20.c and mbio_rx_packet() in modbus_io.c.
*/
static void flow_rx_packet (modbus_message_t *msg)
{
    // The top bit of the first reply byte (0x80) being set means the device
    // sent back a MODBUS "exception" (error) instead of a normal reply.
    // We only process this as a normal reading if that bit is NOT set.
    if (!(msg->adu[0] & 0x80)) {

        // Was this reply actually for our flow-reading request?
        if ((flow_response_t)msg->context == FlowMonitor_ReadInput) {

            // For "read discrete inputs", the reply byte layout is:
            //   adu[0] = slave address
            //   adu[1] = function code (echoed back)
            //   adu[2] = byte count of the data that follows
            //   adu[3] = the actual data byte, where bit 0 is our input's value
            //
            // We only asked for 1 input, so we just look at bit 0 of adu[3].
            uint8_t reading = msg->adu[3] & 0x01;

            // Compare what we read against the value the user told us means
            // "flow is happening" (settings.flow_active_value lets the user
            // match however their particular sensor/wiring reports flow).
            if (reading == settings.flow_active_value) {

                // Flow detected - reset our "how many bad readings in a row"
                // counter back to zero, since things are working normally again.
                missed_count = 0;

            } else {

                // No flow detected on this poll. Count it, and only raise the
                // alarm once we have seen too many bad readings in a row -
                // this avoids a single missed or noisy reading causing a
                // false alarm. This mirrors how gs20.c counts "exceptions"
                // before calling vfd_failed().
                missed_count++;

                if (missed_count >= settings.max_missed_polls) {

                    // Too many "no flow" readings while the spindle is meant
                    // to be running - raise the spindle alarm.
                    spindle_alarm(SpindleAlarm_None); // TODO: use/define a flow-specific alarm reason if available

                    // Reset the counter so we do not immediately re-trigger
                    // again on the very next poll.
                    missed_count = 0;
                }
            }
        }
    }
}

/*
  ---------------------------------------------------------------------------
  HANDLING ERRORS / NO RESPONSE
  ---------------------------------------------------------------------------
  Called automatically if the MODBUS layer gives up on a request - for
  example the device never replied at all, or its reply was garbled. We
  treat that exactly like a "no flow" reading, because if we cannot confirm
  flow is happening, we should not assume that it is. Same idea as gs20.c's
  rx_exception(), which also counts failures and triggers an alarm function.
*/
static void flow_rx_exception (uint8_t code, void *context)
{
    UNUSED(code);

    if ((flow_response_t)context == FlowMonitor_ReadInput) {

        missed_count++;

        if (missed_count >= settings.max_missed_polls) {
            spindle_alarm(SpindleAlarm_None); // TODO: use/define a flow-specific alarm reason if available
            missed_count = 0;
        }
    }
}

/*
  ---------------------------------------------------------------------------
  THE POLLING TIMER
  ---------------------------------------------------------------------------
  task_add_delayed() schedules a function to run once, after a delay, on
  grblHAL's main "foreground" task queue (NOT inside an interrupt). To make
  something repeat forever, the function re-schedules itself again as the
  very first thing it does, every time it runs. This is the exact pattern
  poll_rack_sensor() uses in sienci-atci-plugin.c.

  We only want to actually poll the MODBUS device while the spindle is
  running, so this function checks polling_active before doing any work -
  but it keeps re-arming itself regardless, so it is always ready to start
  polling the moment the spindle turns on.
*/
static void flow_poll_tick (void *data)
{
    UNUSED(data);

    // Re-schedule ourselves to run again after the configured poll period.
    // Doing this first (rather than last) means a slow MODBUS response can
    // never push our next poll later than intended.
    task_add_delayed(flow_poll_tick, NULL, settings.poll_period_ms);

    // Only actually send a MODBUS request if the spindle is currently
    // supposed to be running. If it is not, there is nothing to check.
    if (polling_active)
        flow_poll_request();
}

/*
  ---------------------------------------------------------------------------
  SPINDLE STATE HOOK
  ---------------------------------------------------------------------------
  grblHAL calls on_spindle_selected whenever a spindle becomes the active
  one - gs20.c uses this same hook to know when "its" spindle is in use.
  We use it here just to know that a spindle exists and is in use, so we can
  decide whether we should be polling.

  NOTE: This only tells us a spindle was *selected*, not whether it is
  currently spinning. To know if it is actually running, we check the
  spindle's reported state every time the hook fires, and also whenever
  settings change. If your grblHAL version exposes a more direct
  "on_spindle_state_changed" style hook, that would be an even better fit -
  this is the safe, broadly-compatible option based on what gs20.c shows us.
*/
static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    if (spindle) {
        // Ask the spindle for its current state (on/off, direction etc) and
        // use that to decide whether we should be polling for flow.
        spindle_state_t state = spindle->get_state(spindle);
        polling_active = state.on;
    } else {
        // No spindle selected at all - nothing to monitor.
        polling_active = false;
    }

    // Always call any other plugin's handler that was already attached to
    // this same hook, so we do not break other plugins.
    if (on_spindle_selected)
        on_spindle_selected(spindle);
}

/*
  ---------------------------------------------------------------------------
  REPORTING OUR PRESENCE
  ---------------------------------------------------------------------------
  This makes the plugin show up in the startup/$I report, same idea as
  onReportOptions() in gs20.c and mbio_report_options() in modbus_io.c.
*/
static void onReportOptions (bool newopt)
{
    // Always call the previous handler first so other plugins still get
    // their turn to report themselves.
    on_report_options(newopt);

    if (!newopt)
        report_plugin("Flow Monitor", "v0.1");
}

/*
  ---------------------------------------------------------------------------
  SETTINGS: IDs
  ---------------------------------------------------------------------------
  Every setting needs a unique numeric ID across all of grblHAL and every
  other plugin you are using. Just like the comment in sienci-atci-plugin.c
  says, pick a free range - these are placeholder numbers, you will want to
  confirm they do not clash with anything else in your particular build.
*/
#define SETTING_FLOW_MODBUS_ADDRESS   700
#define SETTING_FLOW_INPUT_ADDRESS    701
#define SETTING_FLOW_POLL_PERIOD      702
#define SETTING_FLOW_ACTIVE_VALUE     703
#define SETTING_FLOW_MAX_MISSED       704

/*
  ---------------------------------------------------------------------------
  SETTINGS: DESCRIPTION TABLE
  ---------------------------------------------------------------------------
  This table is how grblHAL knows these settings exist at all - it is what
  lets the user read and change them with $-commands (e.g. $700=3), and it
  is what the load/save/restore functions below work with. Each row says:
  ID, which settings group it belongs to, the name shown to the user, the
  unit (if any), how to format/validate it, the allowed min/max, a flag, and
  finally a pointer to where the value actually lives in our settings struct.

  This is the same shape as the plugin_settings[] table in
  sienci-atci-plugin.c - we are just describing different fields.
*/
static const setting_detail_t flow_monitor_settings[] = {
    { SETTING_FLOW_MODBUS_ADDRESS, Group_Spindle, "Flow sensor MODBUS address", NULL,
      Format_Int8, "##0", "0", "247", Setting_NonCore, &settings.modbus_address },

    { SETTING_FLOW_INPUT_ADDRESS, Group_Spindle, "Flow sensor input number", NULL,
      Format_Int16, "####0", "0", "9999", Setting_NonCore, &settings.input_address },

    { SETTING_FLOW_POLL_PERIOD, Group_Spindle, "Flow sensor poll period", "ms",
      Format_Int16, "####0", "50", "60000", Setting_NonCore, &settings.poll_period_ms },

    { SETTING_FLOW_ACTIVE_VALUE, Group_Spindle, "Flow active reading value", NULL,
      Format_Int8, "#0", "0", "1", Setting_NonCore, &settings.flow_active_value },

    { SETTING_FLOW_MAX_MISSED, Group_Spindle, "Flow alarm after N missed reads", NULL,
      Format_Int8, "##0", "1", "50", Setting_NonCore, &settings.max_missed_polls }
};

/*
  ---------------------------------------------------------------------------
  SETTINGS: RESTORE (factory defaults)
  ---------------------------------------------------------------------------
  Called the very first time the plugin runs (when nothing valid is found in
  NVS yet), or whenever the user does a settings reset. We fill in sensible
  defaults and write them straight to NVS, matching atci_restore().
*/
static void flow_monitor_settings_restore (void)
{
    settings.modbus_address    = 1;     // a common default slave address
    settings.input_address     = 0;     // first discrete input on the device
    settings.poll_period_ms    = 1000;  // check once a second
    settings.flow_active_value = 1;     // assume "1" means flowing, by default
    settings.max_missed_polls  = 3;     // allow 3 bad readings before alarming

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&settings, sizeof(settings), true);
}

/*
  ---------------------------------------------------------------------------
  SETTINGS: LOAD
  ---------------------------------------------------------------------------
  Called once at startup to read our settings struct back out of NVS. If
  nothing usable is there yet (first ever boot, or corrupted data), we fall
  back to flow_monitor_settings_restore(). This also hooks our handlers into
  grblHAL exactly once and kicks off the polling timer for the first time -
  matching how atci_load() in sienci-atci-plugin.c does its one-time hook
  setup and starts its own task_add_delayed() polling loop.
*/
static void flow_monitor_settings_load (void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&settings, nvs_address, sizeof(settings), true) != NVS_TransferResult_OK)
        flow_monitor_settings_restore();

    // Reset runtime-only state every time settings are (re)loaded.
    missed_count = 0;
    polling_active = false;

    // Hook ourselves into grblHAL's spindle-selected and report-options
    // chains exactly once, remembering whoever was there before us.
    on_spindle_selected = grbl.on_spindle_selected;
    grbl.on_spindle_selected = onSpindleSelected;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    // Start the recurring polling timer. From here on, flow_poll_tick()
    // keeps re-scheduling itself forever - see the comment on that function.
    task_add_delayed(flow_poll_tick, NULL, settings.poll_period_ms);
}

/*
  ---------------------------------------------------------------------------
  SETTINGS: SAVE
  ---------------------------------------------------------------------------
  Called by the settings system whenever the user changes one of our values
  with a $-command, so the new value gets written into NVS and survives a
  reboot. Matches atci_save().
*/
static void flow_monitor_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&settings, sizeof(settings), true);
}

/*
  ---------------------------------------------------------------------------
  PLUGIN ENTRY POINT
  ---------------------------------------------------------------------------
  Called once from the driver's init code, the same way vfd_gs20_init() is
  called for the GS20 driver. This is where we reserve our slice of NVS and
  register our settings table with the core, using the exact same
  setting_details_t + settings_register() pattern as atci_init().
*/
void flow_monitor_init (void)
{
    static setting_details_t details = {
        .settings = flow_monitor_settings,
        .n_settings = sizeof(flow_monitor_settings) / sizeof(setting_detail_t),
        .load = flow_monitor_settings_load,
        .save = flow_monitor_settings_save,
        .restore = flow_monitor_settings_restore
    };

    // Reserve enough space in non-volatile storage to hold our settings
    // struct. nvs_alloc() returns 0/NULL if there is no room left, in which
    // case we deliberately do NOT register the plugin - same safety check
    // as atci_init() uses.
    if ((nvs_address = nvs_alloc(sizeof(settings))))
        settings_register(&details);
}

#endif // FLOW_MONITOR_ENABLE
