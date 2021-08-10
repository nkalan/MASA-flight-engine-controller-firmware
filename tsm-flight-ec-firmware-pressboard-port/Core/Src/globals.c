/// BEGIN AUTOGENERATED SECTION - MODIFICATIONS TO THIS CODE WILL BE OVERWRITTEN

/// globals.c
/// Autogenerated by firmware-libraries/SerialComms/python/telem_file_generator.py on Tue Aug 10 17:14:38 2021
#include "globals.h"

enum States STATE = 0;
float e_batt = 0;
float i_batt = 0;
uint32_t valve_states = 0;
float e3v = 0;
float e5v = 0;
float i5v = 0;
float i3v = 0;
uint8_t last_command_id = 0;
uint32_t status_flags = 0;
float lox_control_pressure = 0;
float fuel_control_pressure = 0;
float copv_control_pressure = 0;
float init_motor_pos_deg_correction_factor = 0;
int32_t state_rem_duration = 0;
uint8_t telem_rate = 0;
uint8_t adc_rate = 0;
uint32_t flash_mem = 0;
uint8_t LOGGING_ACTIVE = 0;
float pressure[6] = {0};
float ivlv[9] = {0};
float evlv[9] = {0};
float tc[5] = {0};
float mtr_pos[2] = {0};
int16_t mtr_vel[2] = {0};
float i_mtr_ab[4] = {0};
float i_mtr[2] = {0};
float mtr_set[2] = {0};
volatile float epot[2] = {0};

/// END AUTOGENERATED SECTION - USER CODE GOES BELOW THIS LINE
