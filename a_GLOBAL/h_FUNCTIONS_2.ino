
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
  /*
   * This function is an ISR that calculates signal pulse width and
   * stores it into volatile_signals[];
   */
  if (digitalRead(ch_input_pin) == HIGH) {
    rising_edge_time[ch_num] = micros();
  } else {
    uint16_t duration = (uint16_t)(micros() - rising_edge_time[ch_num]);
    volatile_signals[ch_num] = duration;
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

void copyVolatileSignalsToArray(uint8_t signal_array_id) {
  switch (signal_array_id) {
    case ACTIVE_SIGNAL_ARRAY:
      noInterrupts();
      memcpy(active_signal, (const void *)volatile_signals, sizeof(volatile_signals)); // Copy data from volatile to new_signal array
      interrupts();
    case DEBUG_SIGNAL_ARRAY:
      noInterrupts();
      memcpy(debug_signal, (const void *)volatile_signals, sizeof(volatile_signals)); // Copy data from volatile to old_signal array
      interrupts();
    default:
      break;
  }
}

// FOR RC CONTROL - HEXAPOD-SPECIFIC //

float getSignalFromArray(uint8_t signal_array_id, uint8_t ch_num) {
  /*
   * Returns the value stored in the array of pulse widths identified by signal_array_id,
   * mapped to the range of expected signal limits.
   * 
   */
  switch (signal_array_id) {
    case ACTIVE_SIGNAL_ARRAY:
      switch (ch_num) {
        case ch_1: return float(active_signal[ch_num]);
        case ch_2: return float(active_signal[ch_num]);
        case ch_3: return float(active_signal[ch_num]);
        case ch_4: return float(active_signal[ch_num]);
        case ch_5: return float(active_signal[ch_num]);
        case ch_6: return float(active_signal[ch_num]);
        case ch_7: return float(active_signal[ch_num]);
        case ch_8: return float(active_signal[ch_num]);
        case ch_9: return float(active_signal[ch_num]);
        case ch_10: return float(active_signal[ch_num]);
        default: return 0;
      }
    case DEBUG_SIGNAL_ARRAY:
      switch (ch_num) {
        case ch_1: return float(debug_signal[ch_num]);
        case ch_2: return float(debug_signal[ch_num]);
        case ch_3: return float(debug_signal[ch_num]);
        case ch_4: return float(debug_signal[ch_num]);
        case ch_5: return float(debug_signal[ch_num]);
        case ch_6: return float(debug_signal[ch_num]);
        case ch_7: return float(debug_signal[ch_num]);
        case ch_8: return float(debug_signal[ch_num]);
        case ch_9: return float(debug_signal[ch_num]);
        case ch_10: return float(debug_signal[ch_num]);
        default: return 0;
      }
    default: return 0;
  }
}
