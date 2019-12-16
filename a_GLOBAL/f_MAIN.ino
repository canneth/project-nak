
void loop() {
  copyVolatileSignalsToArray(DEBUG_SIGNAL_ARRAY);
  float command_signal = getSignalFromArray(DEBUG_SIGNAL_ARRAY, ch_3);
  
  float command_signal_offset = -signal_min;
  float adjusted_command_signal = command_signal + command_signal_offset;
  float adjusted_signal_min = signal_min + command_signal_offset;
  float adjusted_signal_max = signal_max + command_signal_offset;

  // FOR FINDING SIGNAL LIMITS //
  R_1.moveCoxaToSignal(adjusted_command_signal);
  Serial.println(adjusted_command_signal);

  // FOR TESTING SIGNAL LIMITS //
//  float command_angle = floatMap(command_signal, signal_min, signal_max, L_1.getTibiaRadMin(), L_2.getTibiaRadMax());
//  L_1.moveTibiaToAngle(command_angle);
//  Serial.println(command_angle);
}
