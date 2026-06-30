/*
  flow_monitor.h - header for the grblHAL flow monitor plugin

  This declares the one function other code needs to call (flow_monitor_init)
  and the settings struct shape, in case another file ever needs to look at
  the current settings. Everything else in flow_monitor.c is kept private to
  that file (declared "static"), since nothing outside it needs to see it.

  Copyright (c) 2026
  Part of grblHAL

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

// An "include guard" - this stops the file being read twice into the same
// build, which would cause "already defined" errors. The #ifndef/#define
// pair only lets the contents through the first time this header is
// included; any later #include of this same file is skipped because
// _FLOW_MONITOR_H_ is already defined by then.
#ifndef _FLOW_MONITOR_H_
#define _FLOW_MONITOR_H_

// Pull in driver.h here too, so that anyone who includes only this header
// (without having included driver.h themselves first) still gets the
// FLOW_MONITOR_ENABLE definition and basic types like uint8_t/uint16_t/bool
// that we use below.
#include "driver.h"

// Just like in flow_monitor.c, everything in this header only matters if
// the plugin has actually been turned on for this build. Wrapping the
// declarations in this #if means that on a build where the plugin is
// disabled, this header effectively declares nothing at all.
#if FLOW_MONITOR_ENABLE

/*
  ---------------------------------------------------------------------------
  SETTINGS STRUCT (shared shape)
  ---------------------------------------------------------------------------
  This describes the shape of the settings used by the flow monitor plugin,
  so that other source files can know what fields exist (for example, to
  read a poll_period_ms value for a status display) without needing to see
  the plugin's internals.

  NOTE: this header is kept separate/standalone. If you wire it up to
  flow_monitor.c later (by adding #include "flow_monitor.h" there), make
  sure flow_monitor.c's own copy of this struct matches this one exactly,
  field for field - the compiler will not check this for you unless the two
  files actually share this same definition.
*/
typedef struct {
    uint8_t  modbus_address;     // MODBUS slave address of the flow sensor device (0-247)
    uint16_t input_address;      // Which discrete input (on/off point) number to read on that device
    uint16_t poll_period_ms;     // How often (in milliseconds) we ask the device for a new reading
    uint8_t  flow_active_value;  // Does a flowing reading return 0 or 1? (lets the user match wiring/logic)
    uint8_t  max_missed_polls;   // How many "no flow" readings in a row are allowed before we alarm
} flow_monitor_settings_t;

/*
  ---------------------------------------------------------------------------
  PUBLIC FUNCTION
  ---------------------------------------------------------------------------
  flow_monitor_init() is the only function from this plugin that the rest of
  grblHAL needs to know about. It is declared here so that whichever file
  starts up all the plugins (often something like my_machine_map.c or a
  driver's init routine) can call it, the same way vfd_gs20_init() gets
  called from elsewhere to start up the GS20 spindle driver.

  Calling this function once at startup is what:
    - reserves space for our settings in non-volatile storage,
    - registers our settings with the $-settings system,
    - and (once settings are loaded) hooks us into spindle selection and
      starts the periodic MODBUS polling timer.

  Everything else this plugin does happens automatically after that single
  call - nothing else needs to call into flow_monitor.c directly.
*/
void flow_monitor_init (void);

#endif // FLOW_MONITOR_ENABLE

#endif // _FLOW_MONITOR_H_
