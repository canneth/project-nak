
Adafruit_PWMServoDriver L_driver = Adafruit_PWMServoDriver(I2C_Add_1);
Adafruit_PWMServoDriver R_driver = Adafruit_PWMServoDriver(I2C_Add_2);

Leg L_1(L_driver, servo_L_11, servo_L_12, servo_L_13, L_11_offset, L_12_offset, L_13_offset, -39.5, 66.0, LEFT_LEG);
Leg L_2(L_driver, servo_L_21, servo_L_22, servo_L_23, L_21_offset, L_22_offset, L_23_offset, -58.0, 0.0, LEFT_LEG);
Leg L_3(L_driver, servo_L_31, servo_L_32, servo_L_33, L_31_offset, L_32_offset, L_33_offset, -39.5, -66.0, LEFT_LEG);
Leg R_1(R_driver, servo_R_11, servo_R_12, servo_R_13, R_11_offset, R_12_offset, R_13_offset, 39.5, 66.0, RIGHT_LEG);
Leg R_2(R_driver, servo_R_21, servo_R_22, servo_R_23, R_21_offset, R_22_offset, R_23_offset, 58.0, 0.0, RIGHT_LEG);
Leg R_3(R_driver, servo_R_31, servo_R_32, servo_R_33, R_31_offset, R_32_offset, R_33_offset, 39.5, -66.0, RIGHT_LEG);
