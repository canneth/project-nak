
void loop() {
//  copyVolatileSignalsToArray(ACTIVE_SIGNAL_ARRAY);
//  float command_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_3);

  hexapod.updateLegsStancePos();
  Leg leg_to_test = hexapod.R_3;
  leg_to_test.moveToStancePos();
  /*
   * R_1: NOT OKAY; Suspect limits are flipped
   * R_2: OKAY
   * R_3: OKAY
   * L_1: OKAY
   * L_2: OKAY
   * L_3: NOT OKAY; Suspects are also the limits
   * 
   */
}
