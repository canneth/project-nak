
void loop() {

  float phase = 0;
  
  while (true) {
    // Get signals
    copyVolatileSignalsToArray(ACTIVE_SIGNAL_ARRAY);
    float mode_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_6);
    float roll_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_1);
    float pitch_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_2);
    float yaw_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_9);
    float height_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_5);
    float diameter_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_10);
    float x_direction_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_4);
    float y_direction_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_3);
    
    // Set limits
    float roll_min = radians(-10);
    float roll_max = radians(10);
    float pitch_min = radians(-10);
    float pitch_max = radians(10);
    float yaw_min = radians(-10);
    float yaw_max = radians(10);
    float height_min = 80.0;
    float height_max = 150.0;
    float diameter_min = 200.0;
    float diameter_max = 500.0;
    float phase_step_scalar = 0.2;
    float x_direction_component_min = -1.0;
    float x_direction_component_max = 1.0;
    float y_direction_component_min = -1.0;
    float y_direction_component_max = 1.0;
    
    // Map signals into values
    float roll_angle = constrain(floatMap(roll_signal, signal_min, signal_max, roll_min, roll_max), roll_min, roll_max);
    float pitch_angle = constrain(floatMap(pitch_signal, signal_min, signal_max, pitch_min, pitch_max), pitch_min, pitch_max);
    float yaw_angle = constrain(floatMap(yaw_signal, signal_min, signal_max, yaw_min, yaw_max), yaw_min, yaw_max);
    float height = constrain(floatMap(height_signal, signal_min, signal_max, height_min, height_max), height_min, height_max);
    float diameter = constrain(floatMap(diameter_signal, signal_min, signal_max, diameter_min, diameter_max), diameter_min, diameter_max);
    float x_direction_component = constrain(floatMap(x_direction_signal, signal_min, signal_max, x_direction_component_min, x_direction_component_max), x_direction_component_min, x_direction_component_max);
    float y_direction_component = constrain(floatMap(y_direction_signal, signal_min, signal_max, y_direction_component_min, y_direction_component_max), y_direction_component_min, y_direction_component_max);
    x_direction_component = (abs(x_direction_component) > 0.005) ? x_direction_component : 0; // Neutral deadband
    y_direction_component = (abs(y_direction_component) > 0.005) ? y_direction_component : 0; // Neutral deadband
    
    uint8_t mode = (mode_signal < signal_mid) ? DYNAMIC_STANCE_MODE : DYNAMIC_GAIT_MODE;

    switch (mode) {
      case DYNAMIC_STANCE_MODE:
        hexapod.dynamicStance(height, diameter, roll_angle, pitch_angle, yaw_angle);
        break;
      case DYNAMIC_GAIT_MODE:
        float swing_diameter = 80.0;
        phase = phase + phase_step_scalar*sqrt(pow(x_direction_component, 2) + pow(y_direction_component, 2));
        
        foot_phase_diffs_t phase_diffs {
          .leg_1 = 0,
          .leg_2 = PI,
          .leg_3 = 0,
          .leg_4 = PI,
          .leg_5 = 0,
          .leg_6 = PI
        };
        hexapod.dynamicGait(height, diameter, roll_angle, pitch_angle, yaw_angle, swing_diameter, phase, x_direction_component, y_direction_component, phase_diffs);
        break;
      default:
        break;
    }
  }
}
