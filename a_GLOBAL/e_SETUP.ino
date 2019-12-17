
void setup() {
  Serial.begin(115200);

  // FOR HEXAPOD //
  init_drivers();
  set_driver_pwm_freqs(50);
  delay(20);

  // FOR RC CONTROL //
  pin_mode_all_input();
  enable_all_ch_interrupts();

  // SETTING JOINT SIGNAL LIMITS //
  
  hexapod.L_1.setCoxaSignalMin(420);
  hexapod.L_1.setCoxaSignalMax(228);
  hexapod.L_1.setFemurSignalMin(120);
  hexapod.L_1.setFemurSignalMax(296);
  hexapod.L_1.setTibiaSignalMin(304);
  hexapod.L_1.setTibiaSignalMax(124);
  
  hexapod.L_2.setCoxaSignalMin(388);
  hexapod.L_2.setCoxaSignalMax(248);
  hexapod.L_2.setFemurSignalMin(120);
  hexapod.L_2.setFemurSignalMax(300);
  hexapod.L_2.setTibiaSignalMin(320);
  hexapod.L_2.setTibiaSignalMax(152);
  
  hexapod.L_3.setCoxaSignalMin(404);
  hexapod.L_3.setCoxaSignalMax(212);
  hexapod.L_3.setFemurSignalMin(156);
  hexapod.L_3.setFemurSignalMax(340);
  hexapod.L_3.setTibiaSignalMin(332);
  hexapod.L_3.setTibiaSignalMax(148);

  hexapod.R_1.setCoxaSignalMin(412);
  hexapod.R_1.setCoxaSignalMax(216);
  hexapod.R_1.setFemurSignalMin(150);
  hexapod.R_1.setFemurSignalMax(332);
  hexapod.R_1.setTibiaSignalMin(344);
  hexapod.R_1.setTibiaSignalMax(148);
  
  hexapod.R_2.setCoxaSignalMin(388);
  hexapod.R_2.setCoxaSignalMax(240);
  hexapod.R_2.setFemurSignalMin(144);
  hexapod.R_2.setFemurSignalMax(336);
  hexapod.R_2.setTibiaSignalMin(328);
  hexapod.R_2.setTibiaSignalMax(120);
  
  hexapod.R_3.setCoxaSignalMin(400);
  hexapod.R_3.setCoxaSignalMax(212);
  hexapod.R_3.setFemurSignalMin(140);
  hexapod.R_3.setFemurSignalMax(320);
  hexapod.R_3.setTibiaSignalMin(300);
  hexapod.R_3.setTibiaSignalMax(116);

  // SETTING JOINT ANGLE LIMITS // 
  
  hexapod.L_1.setCoxaRadMin(radians(180));
  hexapod.L_1.setCoxaRadMax(radians(90));
  hexapod.L_1.setFemurRadMin(radians(-90));
  hexapod.L_1.setFemurRadMax(radians(0));
  hexapod.L_1.setTibiaRadMin(radians(-90));
  hexapod.L_1.setTibiaRadMax(radians(0));
  
  hexapod.L_2.setCoxaRadMin(radians(213.5));
  hexapod.L_2.setCoxaRadMax(radians(146.5));
  hexapod.L_2.setFemurRadMin(radians(-90));
  hexapod.L_2.setFemurRadMax(radians(0));
  hexapod.L_2.setTibiaRadMin(radians(-90));
  hexapod.L_2.setTibiaRadMax(radians(0));
 
  hexapod.L_3.setCoxaRadMin(radians(180));
  hexapod.L_3.setCoxaRadMax(radians(270));
  hexapod.L_3.setFemurRadMin(radians(-90));
  hexapod.L_3.setFemurRadMax(radians(0));
  hexapod.L_3.setTibiaRadMin(radians(-90));
  hexapod.L_3.setTibiaRadMax(radians(0));
  
  hexapod.R_1.setCoxaRadMin(radians(0));
  hexapod.R_1.setCoxaRadMax(radians(90));
  hexapod.R_1.setFemurRadMin(radians(-90));
  hexapod.R_1.setFemurRadMax(radians(0));
  hexapod.R_1.setTibiaRadMin(radians(-90));
  hexapod.R_1.setTibiaRadMax(radians(0));
  
  hexapod.R_2.setCoxaRadMin(radians(-33.5));
  hexapod.R_2.setCoxaRadMax(radians(33.5));
  hexapod.R_2.setFemurRadMin(radians(-90));
  hexapod.R_2.setFemurRadMax(radians(0));
  hexapod.R_2.setTibiaRadMin(radians(-90));
  hexapod.R_2.setTibiaRadMax(radians(0));
 
  hexapod.R_3.setCoxaRadMin(radians(-90));
  hexapod.R_3.setCoxaRadMax(radians(0));
  hexapod.R_3.setFemurRadMin(radians(-90));
  hexapod.R_3.setFemurRadMax(radians(0));
  hexapod.R_3.setTibiaRadMin(radians(-90));
  hexapod.R_3.setTibiaRadMax(radians(0));
  
}
