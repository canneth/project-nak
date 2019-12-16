
// FOR HEXAPOD MOTION //

void init_drivers() {
  L_driver.begin();
  R_driver.begin();
}

void reset_drivers() {
  L_driver.reset();
  R_driver.reset();
}

void set_driver_pwm_freqs(uint8_t freq) {
  L_driver.setPWMFreq(freq);
  R_driver.setPWMFreq(freq);
}

void home_all_coxae() {
  L_1.home_coxa();
  L_2.home_coxa();
  L_3.home_coxa();
  R_1.home_coxa();
  R_2.home_coxa();
  R_3.home_coxa();
}

void move_all_legs(float z_height, float y_lean = 0, float x_lean = 0, float tilt_deg = 0, float tilt_heading_deg = 0, float leg_flare = 60) {
  float leg_lag = PI/3;
  float dist_long = 118; // Distance from body origin to front and back coxae
  float dist_short = 98; // Distance from body origin to middle coxae
  float tilt_rad = radians(tilt_deg);
  float tilt_heading_rad = radians(tilt_heading_deg);
  
  L_1.move_leg(z_height - dist_long*tan(sin(tilt_heading_rad + leg_lag/2)*tilt_rad), y_lean, x_lean, leg_flare);
  R_1.move_leg(z_height - dist_long*tan(sin(tilt_heading_rad + leg_lag/2 + leg_lag)*tilt_rad), y_lean, x_lean, leg_flare);
  R_2.move_leg(z_height - dist_short*tan(sin(tilt_heading_rad + leg_lag/2 + leg_lag*2)*tilt_rad), y_lean, x_lean, leg_flare);
  R_3.move_leg(z_height - dist_long*tan(sin(tilt_heading_rad + leg_lag/2 + leg_lag*3)*tilt_rad), y_lean, x_lean, leg_flare);
  L_3.move_leg(z_height - dist_long*tan(sin(tilt_heading_rad + leg_lag/2 + leg_lag*4)*tilt_rad), y_lean, x_lean, leg_flare);
  L_2.move_leg(z_height - dist_short*tan(sin(tilt_heading_rad + leg_lag/2 + leg_lag*5)*tilt_rad), y_lean, x_lean, leg_flare);
}


