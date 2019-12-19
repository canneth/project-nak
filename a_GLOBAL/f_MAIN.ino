
void loop() {

  float phase = 0;
  float swing_diameter = 50.0;
  foot_phase_diffs_t tripod_gait {
          .leg_1 = 0.0,
          .leg_2 = PI,
          .leg_3 = 0.0,
          .leg_4 = PI,
          .leg_5 = 0.0,
          .leg_6 = PI
        };
  foot_phase_diffs_t ripple_gait {
    .leg_1 = 0.0,
    .leg_2 = 4.0*PI/3.0,
    .leg_3 = 2.0*PI/3.0,
    .leg_4 = PI,
    .leg_5 = PI/3.0,
    .leg_6 = 5.0*PI/3.0
  };
  
  while (true) {
    // Get signals
    copyVolatileSignalsToArray(ACTIVE_SIGNAL_ARRAY);
    float mode_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_10);
    float right_stick_mode_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_7);
    float roll_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_1);
    float pitch_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_2);
    float yaw_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_9);
    float translate_x_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_1); // Shares channel with roll_signal
    float translate_y_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_2); // Shares channel with pitch_signal
    float swing_speed_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_9); // Shares channel with yaw_signal
    float height_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_5);
    float diameter_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_8);
    float x_direction_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_4);
    float y_direction_signal = getSignalFromArray(ACTIVE_SIGNAL_ARRAY, ch_3);
    
    // Set limits
    float roll_min = radians(-15);
    float roll_max = radians(15);
    float pitch_min = radians(-15);
    float pitch_max = radians(15);
    float yaw_min = radians(-30);
    float yaw_max = radians(30);
    float translate_min = -50.0;
    float translate_max = 50.0;
    float swing_speed_min = 0;
    float swing_speed_max = 0.6;
    float height_min = 50.0;
    float height_max = 150.0;
    float diameter_min = 200.0;
    float diameter_max = 500.0;
    float phase_step_scalar = 0.2;
    float x_direction_component_min = -1.0;
    float x_direction_component_max = 1.0;
    float y_direction_component_min = -1.0;
    float y_direction_component_max = 1.0;

    // Map signals into values
    float roll_angle = constrain(floatMap(roll_signal - 40, signal_min, signal_max, roll_min, roll_max), roll_min, roll_max);
    float pitch_angle = constrain(floatMap(pitch_signal, signal_min, signal_max, pitch_min, pitch_max), pitch_min, pitch_max);
    float yaw_angle = constrain(floatMap(yaw_signal, signal_min, signal_max, yaw_min, yaw_max), yaw_min, yaw_max);
    float translate_x = constrain(floatMap(translate_x_signal, signal_min, signal_max, translate_min, translate_max), translate_min, translate_max);
    float translate_y = constrain(floatMap(translate_y_signal, signal_min, signal_max, translate_min, translate_max), translate_min, translate_max);
    float swing_speed = constrain(floatMap(swing_speed_signal, signal_min, signal_max, swing_speed_min, swing_speed_max), swing_speed_min, swing_speed_max);
    float height = constrain(floatMap(height_signal, signal_min, signal_max, height_min, height_max), height_min, height_max);
    float diameter = constrain(floatMap(diameter_signal, signal_min, signal_max, diameter_min, diameter_max), diameter_min, diameter_max);
    float x_direction_component = constrain(floatMap(x_direction_signal, signal_min, signal_max, x_direction_component_min, x_direction_component_max), x_direction_component_min, x_direction_component_max);
    float y_direction_component = constrain(floatMap(y_direction_signal, signal_min, signal_max, y_direction_component_min, y_direction_component_max), y_direction_component_min, y_direction_component_max);
    x_direction_component = (abs(x_direction_component) > 0.01) ? x_direction_component : 0; // Neutral deadband
    y_direction_component = (abs(y_direction_component) > 0.01) ? y_direction_component : 0; // Neutral deadband
    
    uint8_t right_stick_mode = (right_stick_mode_signal < signal_mid) ? RPY_MODE : TRANSLATION_MODE;
    
    switch (right_stick_mode) {
      case RPY_MODE:
      translate_x = 0;
      translate_y = 0;
      swing_speed = 0;
        break;
      case TRANSLATION_MODE:
        roll_angle = 0;
        pitch_angle = 0;
        yaw_angle = 0;
        break;
      default:
        break;
    }

    uint8_t mode = DYNAMIC_STANCE_MODE;
    
    if (mode_signal < signal_mid) {
//      Serial.println("TRIPOD_GAIT_MODE");
      mode = TRIPOD_GAIT_MODE;
    } else if (mode_signal > signal_mid - 100 && mode_signal < signal_mid + 100) {
//      Serial.println("DYNAMIC_STANCE_MODE");
      mode = DYNAMIC_STANCE_MODE;
    } else if (mode_signal > signal_mid) {
//      Serial.println("RIPPLE_GAIT_MODE");
      mode = RIPPLE_GAIT_MODE;
    }
    
    phase = floatMod(phase + phase_step_scalar*sqrt(pow(x_direction_component, 2) + pow(y_direction_component, 2)), 2.0*PI);

    switch (mode) {
      case DYNAMIC_STANCE_MODE:
        hexapod.dynamicStance(height, diameter, roll_angle, pitch_angle, yaw_angle, translate_x, translate_y);
        break;
      case TRIPOD_GAIT_MODE:
        hexapod.dynamicGait(height, diameter, roll_angle, pitch_angle, yaw_angle, translate_x, translate_y, swing_diameter, phase, x_direction_component, y_direction_component, tripod_gait, swing_speed);
        break;
      case RIPPLE_GAIT_MODE:
        hexapod.dynamicGait(height, diameter, roll_angle, pitch_angle, yaw_angle, translate_x, translate_y, swing_diameter, phase, x_direction_component, y_direction_component, ripple_gait, swing_speed);
      default:
        break;
    }
  }
}
