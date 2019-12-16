
void loop() {
//  reset_drivers();

  if (get_cmd_val(NEW_SIGNAL_ARRAY, ch_9) > signal_mid) {
    walk_protocol();
  } else {
    adjust_protocol();
  }
}
