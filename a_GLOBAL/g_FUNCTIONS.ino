
// UTILITY FUNCTIONS //

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float floatMod(float dividend, float divisor) {
  float quotient = dividend / divisor;
  uint16_t quotient_trunc = (uint16_t)quotient;
  float remainder = divisor * (quotient - quotient_trunc);
  return remainder;
}

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
