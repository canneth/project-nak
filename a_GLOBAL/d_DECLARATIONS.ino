
// UTILITY FUNCTIONS //

float floatMap(float x, float in_min, float in_max, float out_min, float out_max);

// HEXAPOD MOVEMENT //

void init_drivers();
void reset_drivers();
void set_driver_pwm_freqs(uint8_t freq);
void home_all_coxae();
void move_all_legs(float z_height, float y_lean, float x_lean, float tilt_rad, float tilt_heading_rad, float leg_flare);

// FOR RC CONTROL //

void enable_all_ch_interrupts();
void pin_mode_all_input();
void catch_signal(uint8_t ch_num, uint8_t ch_input_pin);
void catch_ch_1();
void catch_ch_2();
void catch_ch_3();
void catch_ch_4();
void catch_ch_5();
void catch_ch_6();
void catch_ch_7();
void catch_ch_8();
void catch_ch_9();
void catch_ch_10();
void copy_vals_to(uint8_t signal_array_id);

// FOR RC CONTROL - HEXAPOD SPECIFIC //

float copyVolatileSignalsToArray(uint8_t signal_array_id, uint8_t ch_num);
float getSignalFromArray(uint8_t signal_array_id, uint8_t ch_num);
