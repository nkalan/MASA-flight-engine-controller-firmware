// Autogenerated by firmware-libraries/SerialComms/python/cmd_template_parser.py on Sat Jul 24 17:00:26 2021

#ifndef PACK_CMD_DEFINES_H
#define PACK_CMD_DEFINES_H
#define NUM_CMD_ITEMS 42
#define COMMAND_MAP_SZ 50
#include <stdint.h>

void set_vlv(uint8_t* data, uint8_t* status);

void set_kp(uint8_t* data, uint8_t* status);

void set_ki(uint8_t* data, uint8_t* status);

void set_kd(uint8_t* data, uint8_t* status);

void set_state(uint8_t* data, uint8_t* status);

void download_flash(uint8_t* data, uint8_t* status);

void wipe_flash(uint8_t* data, uint8_t* status);

void start_logging(uint8_t* data, uint8_t* status);

void stop_logging(uint8_t* data, uint8_t* status);

void set_stepper_pos(uint8_t* data, uint8_t* status);

void set_stepper_zero(uint8_t* data, uint8_t* status);

void set_control_target_pressure(uint8_t* data, uint8_t* status);

void ambientize_pressure_transducers(uint8_t* data, uint8_t* status);

void set_low_toggle_percent(uint8_t* data, uint8_t* status);

void set_high_toggle_percent(uint8_t* data, uint8_t* status);

void set_stepper_speed(uint8_t* data, uint8_t* status);

void set_telem(uint8_t* data, uint8_t* status);

void set_presstank_status(uint8_t* data, uint8_t* status);

void ambientize_pot(uint8_t* data, uint8_t* status);

void led_write(uint8_t* data, uint8_t* status);

void set_system_clock(uint8_t* data, uint8_t* status);

void remove_pressure_ambients(uint8_t* data, uint8_t* status);

void set_autosequence_fuel_first(uint8_t* data, uint8_t* status);

void set_autosequence_lox_first(uint8_t* data, uint8_t* status);

void set_autosequence_mpv_delay(uint8_t* data, uint8_t* status);

void set_engine_test_duration(uint8_t* data, uint8_t* status);

void set_engine_purge_duration(uint8_t* data, uint8_t* status);

void clear_status_flags(uint8_t* data, uint8_t* status);

void start_simulation(uint8_t* data, uint8_t* status);

void advance_simulation(uint8_t* data, uint8_t* status);

void stop_simulation(uint8_t* data, uint8_t* status);

void set_pt_lower_voltage(uint8_t* data, uint8_t* status);

void set_pt_upper_voltage(uint8_t* data, uint8_t* status);

void set_pt_upper_pressure(uint8_t* data, uint8_t* status);

typedef void (*Cmd_Pointer)(uint8_t* x, uint8_t* y);

int16_t command_map[COMMAND_MAP_SZ];

int16_t command_sz[COMMAND_MAP_SZ];

Cmd_Pointer cmds_ptr[NUM_CMD_ITEMS];

// Note: to call a function do
/**
* (*cmds_ptr[0])(array ptr here)
*
* The actual cmd functions will be defined in a separate c file by the firwmare
* developer for each board. They simply need to include this header file
* in the cfile in which they define the function. This allows the developer
* to import/use any variables or typedefs from the hal library. This file is
* simply the jumptable that gives the comms library an easy way to call
* custom functions without additional knowledge of where this file is defined
*
*/


#endif
