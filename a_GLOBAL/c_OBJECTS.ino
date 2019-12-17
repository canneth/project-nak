
Adafruit_PWMServoDriver L_driver = Adafruit_PWMServoDriver(I2C_Add_1);
Adafruit_PWMServoDriver R_driver = Adafruit_PWMServoDriver(I2C_Add_2);

Leg L_1(L_driver, servo_L_11, servo_L_12, servo_L_13, -39.5, 66.0, radians(120), LEFT_LEG);
Leg L_2(L_driver, servo_L_21, servo_L_22, servo_L_23, -58.0, 0.0, radians(180), LEFT_LEG);
Leg L_3(L_driver, servo_L_31, servo_L_32, servo_L_33, -39.5, -66.0, radians(240), LEFT_LEG);
Leg R_1(R_driver, servo_R_11, servo_R_12, servo_R_13, 39.5, 66.0, radians(60), RIGHT_LEG);
Leg R_2(R_driver, servo_R_21, servo_R_22, servo_R_23, 58.0, 0.0, radians(0), RIGHT_LEG);
Leg R_3(R_driver, servo_R_31, servo_R_32, servo_R_33, 39.5, -66.0, radians(-60), RIGHT_LEG);

Hexapod hexapod(L_1, L_2, L_3, R_1, R_2, R_3, default_stance_diameter, default_stance_height);
