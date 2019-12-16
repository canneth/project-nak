
// FOR RC CONTROL //

void enable_all_ch_interrupts() {
  enableInterrupt(ch_1_pin, catch_ch_1, CHANGE); // enableInterrupt function only accepts the name of the function, not the call to it
  enableInterrupt(ch_2_pin, catch_ch_2, CHANGE);
  enableInterrupt(ch_3_pin, catch_ch_3, CHANGE);
  enableInterrupt(ch_4_pin, catch_ch_4, CHANGE);
  enableInterrupt(ch_5_pin, catch_ch_5, CHANGE);
  enableInterrupt(ch_6_pin, catch_ch_6, CHANGE);
  enableInterrupt(ch_7_pin, catch_ch_7, CHANGE);
  enableInterrupt(ch_8_pin, catch_ch_8, CHANGE);
  enableInterrupt(ch_9_pin, catch_ch_9, CHANGE);
  enableInterrupt(ch_10_pin, catch_ch_10, CHANGE);
}

void pin_mode_all_input() {
  pinMode(ch_1_pin, INPUT);
  pinMode(ch_2_pin, INPUT);
  pinMode(ch_3_pin, INPUT);
  pinMode(ch_4_pin, INPUT);
  pinMode(ch_5_pin, INPUT);
  pinMode(ch_6_pin, INPUT);
  pinMode(ch_7_pin, INPUT);
  pinMode(ch_8_pin, INPUT);
  pinMode(ch_9_pin, INPUT);
  pinMode(ch_10_pin, INPUT);
}

void catch_signal(uint8_t ch_num, uint8_t ch_input_pin) {
  if (digitalRead(ch_input_pin) == HIGH) {
    rising_edge_time[ch_num] = micros();
  } else {
    uint16_t duration = (uint16_t)(micros() - rising_edge_time[ch_num]);
    signal_buffer[ch_num] = duration;
  }
}

void catch_ch_1() { catch_signal(ch_1, ch_1_pin); }
void catch_ch_2() { catch_signal(ch_2, ch_2_pin); }
void catch_ch_3() { catch_signal(ch_3, ch_3_pin); }
void catch_ch_4() { catch_signal(ch_4, ch_4_pin); }
void catch_ch_5() { catch_signal(ch_5, ch_5_pin); }
void catch_ch_6() { catch_signal(ch_6, ch_6_pin); }
void catch_ch_7() { catch_signal(ch_7, ch_7_pin); }
void catch_ch_8() { catch_signal(ch_8, ch_8_pin); }
void catch_ch_9() { catch_signal(ch_9, ch_9_pin); }
void catch_ch_10() { catch_signal(ch_10, ch_10_pin); }

void copy_vals_to(uint8_t signal_array_id) {
  if (signal_array_id == NEW_SIGNAL_ARRAY) {
    noInterrupts();
    memcpy(new_signal, (const void *)signal_buffer, sizeof(signal_buffer)); // Copy data from volatile new_signal array
    interrupts();
  } else if (signal_array_id == OLD_SIGNAL_ARRAY) {
    noInterrupts();
    memcpy(old_signal, (const void *)signal_buffer, sizeof(signal_buffer)); // Copy data from volatile to old_signal array
    interrupts();
  }
}


// FOR RC CONTROL - HEXAPOD-SPECIFIC //

float get_cmd_val(uint8_t signal_array_id, uint8_t ch_num) {
  if (signal_array_id == NEW_SIGNAL_ARRAY) {
    switch (ch_num) {
      case ch_1: return float(map(new_signal[ch_num], signal_min, signal_max, y_lean_min, y_lean_max));
      case ch_2: return float(map(new_signal[ch_num], signal_min, signal_max, x_lean_min, x_lean_max));
      case ch_3: return float(map(new_signal[ch_num], signal_min, signal_max, x_tilt_min, x_tilt_max));
      case ch_4: return float(map(new_signal[ch_num], signal_min, signal_max, y_tilt_min, y_tilt_max));
      case ch_5: return float(map(new_signal[ch_num], signal_min, signal_max, z_height_min, z_height_max));
      case ch_6: return float(new_signal[ch_num]); // UNUSED SWITCH! (correct as at 1/10/2018)
      case ch_7: break;
      case ch_8: return float(map(new_signal[ch_num], signal_min, signal_max, leg_flare_min, leg_flare_max));
      case ch_9: return float(new_signal[ch_num]);
      case ch_10: return float(map(new_signal[ch_num], signal_min, signal_max, gait_delta_z_min, gait_delta_z_max));
      default: return 0;
    }
  } else if (signal_array_id == OLD_SIGNAL_ARRAY) {
    switch (ch_num) {
      case ch_1: return float(map(old_signal[ch_num], signal_min, signal_max, y_lean_min, y_lean_max));
      case ch_2: return float(map(old_signal[ch_num], signal_min, signal_max, x_lean_min, x_lean_max));
      case ch_3: return float(map(old_signal[ch_num], signal_min, signal_max, x_tilt_min, x_tilt_max));
      case ch_4: return float(map(old_signal[ch_num], signal_min, signal_max, y_tilt_min, y_tilt_max));
      case ch_5: return float(map(old_signal[ch_num], signal_min, signal_max, z_height_min, z_height_max));
      case ch_6: return float(old_signal[ch_num]);
      case ch_7: break;
      case ch_8: return float(map(old_signal[ch_num], signal_min, signal_max, leg_flare_min, leg_flare_max));
      case ch_9: return float(old_signal[ch_num]);
      case ch_10: return float(map(old_signal[ch_num], signal_min, signal_max, gait_delta_z_min, gait_delta_z_max));
      default: return 0;
    }
  }
    
}

void adjust_protocol() {
  copy_vals_to(NEW_SIGNAL_ARRAY);

  float z_height = get_cmd_val(NEW_SIGNAL_ARRAY, ch_5);
  float y_lean = get_cmd_val(NEW_SIGNAL_ARRAY, ch_1);
  float x_lean = get_cmd_val(NEW_SIGNAL_ARRAY, ch_2);
  float leg_flare = get_cmd_val(NEW_SIGNAL_ARRAY, ch_8);

  float tilt_rad = 0; // Calculate from the 2 channel input values from joystick
  float tilt_heading_rad = 0; // Calculate from the 2 channel input values from joystick

  move_all_legs(z_height, y_lean, x_lean, tilt_rad, tilt_heading_rad, leg_flare);
}

void enter_change_leg_flare_routine(float z_offset) {
  noInterrupts();
  
  copy_vals_to(OLD_SIGNAL_ARRAY);
  
  float old_z_height = get_cmd_val(OLD_SIGNAL_ARRAY, ch_5);
  float old_y_lean = get_cmd_val(OLD_SIGNAL_ARRAY, ch_1);
  float old_x_lean = get_cmd_val(OLD_SIGNAL_ARRAY, ch_2);
  float old_leg_flare = get_cmd_val(OLD_SIGNAL_ARRAY, ch_8);
  
  float up_set_z_height = old_z_height - z_offset;
  float down_set_z_height = old_z_height + z_offset;
  
  L_1.move_leg(down_set_z_height, old_y_lean, old_x_lean, old_leg_flare);
  L_2.move_leg(up_set_z_height, old_y_lean, old_x_lean, old_leg_flare);
  L_3.move_leg(down_set_z_height, old_y_lean, old_x_lean, old_leg_flare);
  R_1.move_leg(up_set_z_height, old_y_lean, old_x_lean, old_leg_flare);
  R_2.move_leg(down_set_z_height, old_y_lean, old_x_lean, old_leg_flare);
  R_3.move_leg(up_set_z_height, old_y_lean, old_x_lean, old_leg_flare);
    
  interrupts();
}

void change_leg_flare_routine(float z_offset) {
  
  copy_vals_to(NEW_SIGNAL_ARRAY);
  
  float down_set_z_height = get_cmd_val(NEW_SIGNAL_ARRAY, ch_5) + z_offset;
  float down_set_y_lean = get_cmd_val(NEW_SIGNAL_ARRAY, ch_1);
  float down_set_x_lean = get_cmd_val(NEW_SIGNAL_ARRAY, ch_2);
  float down_set_leg_flare = get_cmd_val(OLD_SIGNAL_ARRAY, ch_8);
  
  float up_set_z_height = get_cmd_val(NEW_SIGNAL_ARRAY, ch_5) - z_offset;
  float up_set_y_lean = get_cmd_val(NEW_SIGNAL_ARRAY, ch_1);
  float up_set_x_lean = get_cmd_val(NEW_SIGNAL_ARRAY, ch_2);
  float up_set_leg_flare = get_cmd_val(NEW_SIGNAL_ARRAY, ch_8);

  L_1.move_leg(down_set_z_height, down_set_y_lean, down_set_x_lean, down_set_leg_flare);
  L_2.move_leg(up_set_z_height, up_set_y_lean, up_set_x_lean, up_set_leg_flare);
  L_3.move_leg(down_set_z_height, down_set_y_lean, down_set_x_lean, down_set_leg_flare);
  R_1.move_leg(up_set_z_height, up_set_y_lean, up_set_x_lean, up_set_leg_flare);
  R_2.move_leg(down_set_z_height, down_set_y_lean, down_set_x_lean, down_set_leg_flare);
  R_3.move_leg(up_set_z_height, up_set_y_lean, up_set_x_lean, up_set_leg_flare);
}

void exit_change_leg_flare_routine(float z_offset) { // Call noInterrupts() before, and interrupts() after
  noInterrupts();
  
  copy_vals_to(NEW_SIGNAL_ARRAY);

  float down_z_height = get_cmd_val(NEW_SIGNAL_ARRAY, ch_5);
  float up_z_height = get_cmd_val(NEW_SIGNAL_ARRAY, ch_5) - z_offset;
  float y_lean = get_cmd_val(NEW_SIGNAL_ARRAY, ch_1);
  float x_lean = get_cmd_val(NEW_SIGNAL_ARRAY, ch_2);
  float old_leg_flare = get_cmd_val(OLD_SIGNAL_ARRAY, ch_8);  
  float new_leg_flare = get_cmd_val(NEW_SIGNAL_ARRAY, ch_8);
  
  L_1.move_leg(down_z_height, y_lean, x_lean, old_leg_flare); // Bring new legs down to same level
  L_2.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  L_3.move_leg(down_z_height, y_lean, x_lean, old_leg_flare);
  R_1.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  R_2.move_leg(down_z_height, y_lean, x_lean, old_leg_flare);
  R_3.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  delay(500);
  L_1.move_leg(up_z_height, y_lean, x_lean, new_leg_flare); // Bring old legs up and update leg flares
  L_2.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  L_3.move_leg(up_z_height, y_lean, x_lean, new_leg_flare);
  R_1.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  R_2.move_leg(up_z_height, y_lean, x_lean, new_leg_flare);
  R_3.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  delay(500);
  L_1.move_leg(down_z_height, y_lean, x_lean, new_leg_flare); // Bring all legs down to same level
  L_2.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  L_3.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  R_1.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  R_2.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);
  R_3.move_leg(down_z_height, y_lean, x_lean, new_leg_flare);

  interrupts();
}

void change_leg_flare_protocol() {
  enter_change_leg_flare_routine(15);
  while (get_cmd_val(NEW_SIGNAL_ARRAY, ch_9) > signal_mid) {
    change_leg_flare_routine(15);
  }
  exit_change_leg_flare_routine(15);
}

void walk_protocol() {
  // Use sin(i) and cos(i) to make z and x cycles
  // Throttle up: i will increment
  // Throttle down: i will decrement
  // i between 0 and 360
  // Switch to enter gait mode

  float i = 0;
  copy_vals_to(OLD_SIGNAL_ARRAY);
  
  while (get_cmd_val(NEW_SIGNAL_ARRAY, ch_9) > signal_mid) {
    
    copy_vals_to(NEW_SIGNAL_ARRAY);
    
    float z_leg_origin = get_cmd_val(NEW_SIGNAL_ARRAY, ch_5);
    float y_leg_origin = get_cmd_val(OLD_SIGNAL_ARRAY, ch_1);
    float x_leg_origin = get_cmd_val(OLD_SIGNAL_ARRAY, ch_2);
    float leg_flare_origin = get_cmd_val(OLD_SIGNAL_ARRAY, ch_8);

    float z_leg_displacement = get_cmd_val(NEW_SIGNAL_ARRAY, ch_10);
    
    float z_leg_offset_set_1 = cos(radians(i))*z_leg_displacement;
    float z_leg_offset_set_2 = cos(radians(i - 180))*z_leg_displacement;

    float z_leg_set_1 = ((z_leg_origin + z_leg_offset_set_1) < z_leg_origin) ? (z_leg_origin + z_leg_offset_set_1) : z_leg_origin;
    float z_leg_set_2 = ((z_leg_origin + z_leg_offset_set_2) < z_leg_origin) ? (z_leg_origin + z_leg_offset_set_2) : z_leg_origin;

    float x_rate = get_cmd_val(NEW_SIGNAL_ARRAY, ch_3);
    float y_rate = get_cmd_val(NEW_SIGNAL_ARRAY, ch_4);

    float stride_multiplier = 5;
    
    float x_stride = x_rate * stride_multiplier;
    float y_stride = y_rate * stride_multiplier;

    i = i + sqrt(pow(x_rate, 2) + pow(y_rate, 2));

    float x_leg_offset = x_stride*sin(radians(i));
    float y_leg_offset = y_stride*sin(radians(i));

    float x_leg_set_1 = x_leg_origin + x_leg_offset;
    float x_leg_set_2 = x_leg_origin - x_leg_offset;
    float y_leg_set_1 = y_leg_origin + y_leg_offset;
    float y_leg_set_2 = y_leg_origin - y_leg_offset;

    // SET 1 //    
    
    L_1.move_leg(z_leg_set_1, y_leg_set_1, x_leg_set_1, leg_flare_origin);
    L_3.move_leg(z_leg_set_1, y_leg_set_1, x_leg_set_1, leg_flare_origin);
    R_2.move_leg(z_leg_set_1, y_leg_set_1, x_leg_set_1, leg_flare_origin);

    // SET 2 //
    
    L_2.move_leg(z_leg_set_2, y_leg_set_2, x_leg_set_2, leg_flare_origin);
    R_1.move_leg(z_leg_set_2, y_leg_set_2, x_leg_set_2, leg_flare_origin);
    R_3.move_leg(z_leg_set_2, y_leg_set_2, x_leg_set_2, leg_flare_origin);
  }
}
