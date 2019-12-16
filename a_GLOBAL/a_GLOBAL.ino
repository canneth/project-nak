
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EnableInterrupt.h>

#define I2C_Add_1 0x40 // I2C addresses to assign to the PCA9685 modules
#define I2C_Add_2 0x41 // I'm using 2 modules, so I need 2 addresses

// GLOBAL CONSTANTS //

const boolean LEFT_LEG = true;
const boolean RIGHT_LEG = false;

const uint8_t OLD_SIGNAL_ARRAY = 1;
const uint8_t NEW_SIGNAL_ARRAY = 2;

// LEG ID ASSIGNMENT //

const uint8_t servo_L_11 = 0; // L_XY = Xth pair from front, Yth link from body
const uint8_t servo_L_12 = 1;
const uint8_t servo_L_13 = 2;
const uint8_t servo_L_21 = 4;
const uint8_t servo_L_22 = 5;
const uint8_t servo_L_23 = 6;
const uint8_t servo_L_31 = 12;
const uint8_t servo_L_32 = 13;
const uint8_t servo_L_33 = 14;

const uint8_t servo_R_11 = 12;
const uint8_t servo_R_12 = 13;
const uint8_t servo_R_13 = 14;
const uint8_t servo_R_21 = 4;
const uint8_t servo_R_22 = 5;
const uint8_t servo_R_23 = 6;
const uint8_t servo_R_31 = 0;
const uint8_t servo_R_32 = 1;
const uint8_t servo_R_33 = 2;

// LEG PULSE WIDTH OFFSETS //

const int8_t L_11_offset = 0;
const int8_t L_12_offset = 0;
const int8_t L_13_offset = 20;

const int8_t L_21_offset = 0;
const int8_t L_22_offset = 0;
const int8_t L_23_offset = 20;

const int8_t L_31_offset = 0;
const int8_t L_32_offset = 30;
const int8_t L_33_offset = 40;

const int8_t R_11_offset = -10;
const int8_t R_12_offset = 30;
const int8_t R_13_offset = 50;

const int8_t R_21_offset = 0;
const int8_t R_22_offset = 30;
const int8_t R_23_offset = 30;

const int8_t R_31_offset = 10;
const int8_t R_32_offset = 10;
const int8_t R_33_offset = 10;

// LEG ANGLE OFFSETS (RADIANS) //

const float L_1_coxa_rad_offset = radians(-45);
const float L_2_coxa_rad_offset = radians(0);
const float L_3_coxa_rad_offset = radians(45);

const float R_1_coxa_rad_offset = radians(45);
const float R_2_coxa_rad_offset = radians(0);
const float R_3_coxa_rad_offset = radians(-45);

// PHYSICAL DIMENSIONS //

const float coxa_length = 41.92; // Measured in mm
const float femur_length = 45.00;
const float tibia_length = 112.68;

const float body_width = 200; // coxa pivot to coxa pivot

// COMMAND LIMITS //

const long z_height_min = 65;
const long z_height_max = 150;
const long y_lean_min = -50;
const long y_lean_max = 50;
const long x_lean_min = -50;
const long x_lean_max = 50;
const long x_tilt_min = -10;
const long x_tilt_max = 10;
const long y_tilt_min = -10;
const long y_tilt_max = 10;
const long leg_flare_min = 40;
const long leg_flare_max = 150;

const long gait_delta_z_max = 50;
const long gait_delta_z_min = 0;

// FOR RC CONTROL //

const uint8_t num_of_ch = 10;

const uint8_t ch_1 = 0; // Just for convenience and to reduce confusion
const uint8_t ch_2 = 1;
const uint8_t ch_3 = 2;
const uint8_t ch_4 = 3;
const uint8_t ch_5 = 4;
const uint8_t ch_6 = 5;
const uint8_t ch_7 = 6;
const uint8_t ch_8 = 7;
const uint8_t ch_9 = 8;
const uint8_t ch_10 = 9;

const long signal_min = 980;
const long signal_max = 2000;
const long signal_mid = (signal_max + signal_min)/2;
const long signal_error = 100;

const uint8_t ch_1_pin = 52; // Lowest: 992, Neutral: 1488, Highest: 1984
const uint8_t ch_2_pin = 53; // Lowest: 984, Neutral: 1492, Highest: 1976
const uint8_t ch_3_pin = A13; // Lowest: 992, Highest: 1984
const uint8_t ch_4_pin = A12;
const uint8_t ch_5_pin = 50;
const uint8_t ch_6_pin = A15;
const uint8_t ch_7_pin = 0;
const uint8_t ch_8_pin = 51;
const uint8_t ch_9_pin = A14;
const uint8_t ch_10_pin = A11;

uint16_t new_signal[num_of_ch]; // Values memcpy'd from signal_buffer; For direct use
uint16_t old_signal[num_of_ch];
uint32_t rising_edge_time[num_of_ch];
volatile uint16_t signal_buffer[num_of_ch]; // For ISR use only; NOT for direct use

