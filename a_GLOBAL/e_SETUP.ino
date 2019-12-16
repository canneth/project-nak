
void setup() {
  Serial.begin(57600);

  // FOR HEXAPOD //
  init_drivers();
  set_driver_pwm_freqs(50);
  delay(20);

  // FOR RC CONTROL //
  pin_mode_all_input();
  enable_all_ch_interrupts();
}
