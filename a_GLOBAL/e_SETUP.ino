
void setup() {
  Serial.begin(115200);

  // FOR HEXAPOD //
  init_drivers();
  set_driver_pwm_freqs(50);
  delay(20);

  // FOR RC CONTROL //
  pin_mode_all_input();
  enable_all_ch_interrupts();

  // Setting signal reference points //
  
  L_1.setCoxaSignalMin(0);
  L_1.setCoxaSignalMax(0);
  L_1.setFemurSignalMin(120);
  L_1.setFemurSignalMax(296);
  L_1.setTibiaSignalMin(304);
  L_1.setTibiaSignalMax(124);
  
  L_2.setCoxaSignalMin(0);
  L_2.setCoxaSignalMax(0);
  L_2.setFemurSignalMin(120);
  L_2.setFemurSignalMax(300);
  L_2.setTibiaSignalMin(320);
  L_2.setTibiaSignalMax(152);
  
  L_3.setCoxaSignalMin(0);
  L_3.setCoxaSignalMax(0);
  L_3.setFemurSignalMin(156);
  L_3.setFemurSignalMax(340);
  L_3.setTibiaSignalMin(332);
  L_3.setTibiaSignalMax(148);

  R_1.setCoxaSignalMin(0);
  R_1.setCoxaSignalMax(0);
  R_1.setFemurSignalMin(150);
  R_1.setFemurSignalMax(332);
  R_1.setTibiaSignalMin(344);
  R_1.setTibiaSignalMax(148);
  
  R_2.setCoxaSignalMin(0);
  R_2.setCoxaSignalMax(0);
  R_2.setFemurSignalMin(144);
  R_2.setFemurSignalMax(336);
  R_2.setTibiaSignalMin(328);
  R_2.setTibiaSignalMax(120);
  
  R_3.setCoxaSignalMin(0);
  R_3.setCoxaSignalMax(0);
  R_3.setFemurSignalMin(140);
  R_3.setFemurSignalMax(320);
  R_3.setTibiaSignalMin(300);
  R_3.setTibiaSignalMax(116);

  // Setting joint reference points // 
  
  L_1.setCoxaRadMin(0);
  L_1.setCoxaRadMax(0);
  L_1.setFemurRadMin(radians(-90));
  L_1.setFemurRadMax(radians(0));
  L_1.setTibiaRadMin(radians(-90));
  L_1.setTibiaRadMax(radians(0));
  
  L_2.setCoxaRadMin(0);
  L_2.setCoxaRadMax(0);
  L_2.setFemurRadMin(radians(-90));
  L_2.setFemurRadMax(radians(0));
  L_2.setTibiaRadMin(radians(-90));
  L_2.setTibiaRadMax(radians(0));
 
  L_3.setCoxaRadMin(0);
  L_3.setCoxaRadMax(0);
  L_3.setFemurRadMin(radians(-90));
  L_3.setFemurRadMax(radians(0));
  L_3.setTibiaRadMin(radians(-90));
  L_3.setTibiaRadMax(radians(0));
  
  R_1.setCoxaRadMin(radians(0));
  R_1.setCoxaRadMax(radians(90));
  R_1.setFemurRadMin(radians(-90));
  R_1.setFemurRadMax(radians(0));
  R_1.setTibiaRadMin(radians(-90));
  R_1.setTibiaRadMax(radians(0));
  
  R_2.setCoxaRadMin(0);
  R_2.setCoxaRadMax(0);
  R_2.setFemurRadMin(radians(-90));
  R_2.setFemurRadMax(radians(0));
  R_2.setTibiaRadMin(radians(-90));
  R_2.setTibiaRadMax(radians(0));
 
  R_3.setCoxaRadMin(0);
  R_3.setCoxaRadMax(0);
  R_3.setFemurRadMin(radians(-90));
  R_3.setFemurRadMax(radians(0));
  R_3.setTibiaRadMin(radians(-90));
  R_3.setTibiaRadMax(radians(0));
  
}
