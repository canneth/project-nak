
//#define DEBUG_MOVE_TO_BODY_XYZ
//#define DEBUG_MOVE_TO_STANCE_POS

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EnableInterrupt.h>

#define I2C_Add_1 0x40 // I2C addresses to assign to the PCA9685 modules
#define I2C_Add_2 0x41 // I'm using 2 modules, so I need 2 addresses

// GLOBAL CONSTANTS //

const boolean LEFT_LEG = true;
const boolean RIGHT_LEG = false;

const uint8_t ACTIVE_SIGNAL_ARRAY = 1;
const uint8_t DEBUG_SIGNAL_ARRAY = 2;

// HEXAPOD GLOBALS //

float default_stance_diameter = 320; // in mm
float default_stance_height = 100; // in mm

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

const long signal_min = 928;
const long signal_max = 1920;
const long signal_mid = (signal_max + signal_min)/2;

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

uint16_t new_signal[num_of_ch]; // Values memcpy'd from volatile_signals[] for manipulation.
uint16_t debug_signal[num_of_ch];
uint32_t rising_edge_time[num_of_ch];
volatile uint16_t volatile_signals[num_of_ch]; // For ISR access only.
