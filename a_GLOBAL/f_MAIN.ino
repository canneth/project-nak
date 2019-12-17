
void loop() {
  copyVolatileSignalsToArray(ACTIVE_SIGNAL_ARRAY);
  float roll_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_1);
  float pitch_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_2);
  float yaw_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_4);
  float height_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_5);

  float roll_min = radians(-10);
  float roll_max = radians(10);
  float pitch_min = radians(-10);
  float pitch_max = radians(10);
  float yaw_min = radians(-10);
  float yaw_max = radians(10);
  float height_min = 80;
  float height_max = 150;
  
  float roll_angle = constrain(floatMap(roll_signal, signal_min, signal_max, roll_min, roll_max), roll_min, roll_max);
  float pitch_angle = constrain(floatMap(pitch_signal, signal_min, signal_max, pitch_min, pitch_max), pitch_min, pitch_max);
  float yaw_angle = constrain(floatMap(yaw_signal, signal_min, signal_max, yaw_min, yaw_max), yaw_min, yaw_max);
  float height = constrain(floatMap(yaw_signal, signal_min, signal_max, height_min, height_max), height_min, height_max);
  
  hexapod.dynamicStance(roll_angle, pitch_angle, yaw_angle, height);
}
