
class Leg {
  private:
    float coxa_length = 41.92; // Measured in mm
    float femur_length = 45.00;
    float tibia_length = 112.68;
    
    Adafruit_PWMServoDriver driver = 0x40;
    
    uint8_t coxa_pin = 0;
    uint8_t femur_pin = 0;
    uint8_t tibia_pin = 0;
    uint16_t coxa_offset = 0;
    uint16_t femur_offset = 0;
    uint16_t tibia_offset = 0;
    float coxa_rad_min = 0;
    float coxa_rad_max = 0;
    float femur_rad_min = 0;
    float femur_rad_max = 0;
    float tibia_rad_min = 0;
    float tibia_rad_max = 0;
    float coxa_signal_min = 0;
    float coxa_signal_max = 0;
    float femur_signal_min = 0;
    float femur_signal_max = 0;
    float tibia_signal_min = 0;
    float tibia_signal_max = 0;
    float B_x_L = 0;
    float B_y_L = 0;
    float z_orientation_wrt_body = 0;
    boolean is_left_leg = true;
    
    float stance_pos[3] = {0, 0, 0};
    
  public:
    // STRUCTS //

    struct leg_angles_rad_t {
      float coxa_rad;
      float femur_rad;
      float tibia_rad;
    };
    struct foot_pos_t {
      float x = 0;
      float y = 0;
      float z = 0;
    };

    // CONSTRUCTORS //

    Leg() {
      uint8_t coxa_pin_def = 0;
      uint8_t femur_pin_def = 0;
      uint8_t tibia_pin_def = 0;
      uint16_t coxa_offset_def = 0;
      uint16_t femur_offset_def = 0;
      uint16_t tibia_offset_def = 0;
      float B_x_L_def = 0;
      float B_y_L_def = 0;
      float z_orientation_wrt_body_def = 0;
      boolean is_left_leg = true;
    }
    
    Leg(
      Adafruit_PWMServoDriver drive_def,
      uint8_t coxa_pin_def,
      uint8_t femur_pin_def,
      uint8_t tibia_pin_def,
      float B_x_L_def,
      float B_y_L_def,
      float z_orientation_wrt_body_def,
      boolean is_left_leg_def)
    {
      driver = drive_def;
      coxa_pin = coxa_pin_def;
      femur_pin = femur_pin_def;
      tibia_pin = tibia_pin_def;
      B_x_L = B_x_L_def;
      B_y_L = B_y_L_def;
      z_orientation_wrt_body = z_orientation_wrt_body_def;
      is_left_leg = is_left_leg_def;
    }

    // SETTER FUNCTIONS //
    
    void setDriver(Adafruit_PWMServoDriver driver_object) {
      driver = driver_object;
    }
    void setPins(uint8_t coxa_pin_num, uint8_t femur_pin_num, uint8_t tibia_pin_num) {
      coxa_pin = coxa_pin_num;
      femur_pin = femur_pin_num;
      tibia_pin = tibia_pin_num;
    }
    void setOffsets(uint16_t coxa, uint16_t femur, uint16_t tibia) {
      coxa_offset = coxa;
      femur_offset = femur;
      tibia_offset = tibia;
    }
    void setLengths(float coxa, float femur, float tibia) {
      coxa_length = coxa;
      femur_length = femur;
      tibia_length = tibia;
    }
    void set_B_p_L(float B_x_L_arg, float B_y_L_arg) {
      B_x_L = B_x_L_arg;
      B_y_L = B_y_L_arg;
    }
    void setZOrientationWrtBody(float z_orientation_wrt_body_arg) {
      z_orientation_wrt_body = z_orientation_wrt_body;
    }
    void setCoxaRadMin(float coxa_rad_min_arg) {
      coxa_rad_min = coxa_rad_min_arg;
    }
    void setCoxaRadMax(float coxa_rad_max_arg) {
      coxa_rad_max = coxa_rad_max_arg;
    }
    void setFemurRadMin(float femur_rad_min_arg) {
      femur_rad_min = femur_rad_min_arg;
    }
    void setFemurRadMax(float femur_rad_max_arg) {
      femur_rad_max = femur_rad_max_arg;
    }
    void setTibiaRadMin(float tibia_rad_min_arg) {
      tibia_rad_min = tibia_rad_min_arg;
    }
    void setTibiaRadMax(float tibia_rad_max_arg) {
      tibia_rad_max = tibia_rad_max_arg;
    }
    void setCoxaSignalMin(uint16_t coxa_signal_min_arg) {
      coxa_signal_min = coxa_signal_min_arg;
    }
    void setCoxaSignalMax(uint16_t coxa_signal_max_arg) {
      coxa_signal_max = coxa_signal_max_arg;
    }
    void setFemurSignalMin(uint16_t femur_signal_min_arg) {
      femur_signal_min = femur_signal_min_arg;
    }
    void setFemurSignalMax(uint16_t femur_signal_max_arg) {
      femur_signal_max = femur_signal_max_arg;
    }
    void setTibiaSignalMin(uint16_t tibia_signal_min_arg) {
      tibia_signal_min = tibia_signal_min_arg;
    }
    void setTibiaSignalMax(uint16_t tibia_signal_max_arg) {
      tibia_signal_max = tibia_signal_max_arg;
    }
    void setStancePos(float x, float y, float z) {
      // stance_pos is the foot location defined with respect to body frame.
      stance_pos[0] = x;
      stance_pos[1] = y;
      stance_pos[2] = z;
    }
    
    // GETTER FUNCTIONS //

    float getCoxaLength() {
      return coxa_length;
    }
    float getFemurLength() {
      return femur_length;
    }
    float getTibiaLength() {
      return tibia_length;
    }
    float getZOrientationWrtBody() {
      return z_orientation_wrt_body;
    }
    float getCoxaRadMin() {
      return coxa_rad_min;
    }
    float getCoxaRadMax() {
      return coxa_rad_max;
    }
    float getFemurRadMin() {
      return femur_rad_min;
    }
    float getFemurRadMax() {
      return femur_rad_max;
    }
    float getTibiaRadMin() {
      return tibia_rad_min;
    }
    float getTibiaRadMax() {
      return tibia_rad_max;
    }

    float getCoxaSignalMin() {
      return coxa_signal_min;
    }
    float getCoxaSignalMax() {
      return coxa_signal_max;
    }
    float getFemurSignalMin() {
      return femur_signal_min;
    }
    float getFemurSignalMax() {
      return femur_signal_max;
    }
    float getTibiaSignalMin() {
      return tibia_signal_min;
    }
    float getTibiaSignalMax() {
      return tibia_signal_max;
    }
    float getStancePosX() {
      return stance_pos[0];
    }
    float getStancePosY() {
      return stance_pos[1];
    }
    float getStancePosZ() {
      return stance_pos[2];
    }
      
    // ACTION FUNCTIONS //
    
    void moveCoxaToAngle(float coxa_rad_cmd) {
      uint16_t pulse_width_cmd = float_map(coxa_rad_cmd, coxa_rad_min, coxa_rad_max, coxa_signal_min, coxa_signal_max);
      driver.setPWM(coxa_pin, 0, pulse_width_cmd);
    }
    void moveFemurToAngle(float femur_rad_cmd) {
      uint16_t pulse_width_cmd = float_map(femur_rad_cmd, femur_rad_min, femur_rad_max, femur_signal_min, femur_signal_max);
      driver.setPWM(femur_pin, 0, pulse_width_cmd);
    }
    void moveTibiaToAngle(float tibia_rad_cmd) {
      uint16_t pulse_width_cmd = float_map(tibia_rad_cmd, tibia_rad_min, tibia_rad_max, tibia_signal_min, tibia_signal_max);
      driver.setPWM(tibia_pin, 0, pulse_width_cmd);
    }
    void moveCoxaToSignal(float coxa_signal_cmd) {
      driver.setPWM(coxa_pin, 0, coxa_signal_cmd);
    }
    void moveFemurToSignal(float femur_signal_cmd) {
      driver.setPWM(femur_pin, 0, femur_signal_cmd);
    }
    void moveTibiaToSignal(float tibia_signal_cmd) {
      driver.setPWM(tibia_pin, 0, tibia_signal_cmd);
    }
    void moveToStancePos() {
      #ifdef DEBUG_MOVE_TO_STANCE_POS
      Serial.print("leg stance_pos: ");
      Serial.print(stance_pos[0]);
      Serial.print(", ");
      Serial.print(stance_pos[1]);
      Serial.print(", ");
      Serial.println(stance_pos[2]);
      #endif
      moveToBodyXYZ(stance_pos[0], stance_pos[1], stance_pos[2]);
    }
    void moveToBodyXYZ(float B_x, float B_y, float B_z) {
      leg_angles_rad_t vals = bodyIK(B_x, B_y, B_z);
      #ifdef DEBUG_MOVE_TO_BODY_XYZ
      Serial.print("target leg angles: ");
      Serial.print(vals.coxa_rad);
      Serial.print(" (");
      Serial.print(degrees(vals.coxa_rad));
      Serial.print(")");
      Serial.print(", ");
      Serial.print(vals.femur_rad);
      Serial.print(" (");
      Serial.print(degrees(vals.femur_rad));
      Serial.print(")");
      Serial.print(", ");
      Serial.print(vals.tibia_rad);
      Serial.print(" (");
      Serial.print(degrees(vals.tibia_rad));
      Serial.println(")");
      #endif
      moveCoxaToAngle(vals.coxa_rad);
      moveFemurToAngle(vals.femur_rad);
      moveTibiaToAngle(vals.tibia_rad);
    }
    void updateStancePos(float stance_diameter, float stance_height) {
      stance_pos[0] = (stance_diameter/2.0)*cos(z_orientation_wrt_body);
      stance_pos[1] = (stance_diameter/2.0)*sin(z_orientation_wrt_body);
      stance_pos[2] = -stance_height;
    }
    foot_pos_t rollFoot(foot_pos_t foot_pos, float roll_angle) {
      /*
       * Calculates foot_pos after a roll transform (extrinsic rotation about the body Y-axis).
       * The transform of the foot_pos is by the negative of the desired roll_angle.
       * 
       * ARGUMENTS:
       * + roll_angle: A float representing the desired roll angle of the body frame.
       * 
       * RETURNS:
       * + new_foot_pos: A foot_pos_t struct containing the x, y, z coordinates of the new foot_pos.
       */
      // Elementary rotation matrix about Y-axis for roll.
      float R_y[3][3] =
        { 
          {cos(-roll_angle), 0, sin(-roll_angle)},
          {0, 1, 0},
          {-sin(-roll_angle), 0, cos(-roll_angle)}
        };
      // Apply roll rotation transform
      foot_pos_t new_foot_pos;
      new_foot_pos.x = foot_pos.x*R_y[0][0] + foot_pos.y*R_y[0][1] + foot_pos.z*R_y[0][2];
      new_foot_pos.y = foot_pos.x*R_y[1][0] + foot_pos.y*R_y[1][1] + foot_pos.z*R_y[1][2];
      new_foot_pos.z = foot_pos.x*R_y[2][0] + foot_pos.y*R_y[2][1] + foot_pos.z*R_y[2][2];
      // Return new_foot_pos
      return new_foot_pos;
    }
    foot_pos_t pitchFoot(foot_pos_t foot_pos, float pitch_angle) {
      /*
       * Calculates foot_pos after a pitch transform (extrinsic rotation about the body X-axis).
       * The transform of the foot_pos is by the negative of the desired pitch angle.
       * 
       * ARGUMENTS:
       * + pitch_angle: A float representing the desired pitch angle of the body frame.
       * 
       * RETURNS:
       * + new_foot_pos: A foot_pos_t struct containing the x, y, z coordinates of the new foot_pos.
       */
      // Elementary rotation matrix about X-axis for pitch.
      float R_x[3][3] =
        { 
          {1, 0, 0},
          {0, cos(-pitch_angle), -sin(-pitch_angle)},
          {0, sin(-pitch_angle), cos(-pitch_angle)}
        };
      // Apply pitch rotation transform
      foot_pos_t new_foot_pos;
      new_foot_pos.x = foot_pos.x*R_x[0][0] + foot_pos.y*R_x[0][1] + foot_pos.z*R_x[0][2];
      new_foot_pos.y = foot_pos.x*R_x[1][0] + foot_pos.y*R_x[1][1] + foot_pos.z*R_x[1][2];
      new_foot_pos.z = foot_pos.x*R_x[2][0] + foot_pos.y*R_x[2][1] + foot_pos.z*R_x[2][2];
      // Return new_foot_pos
      return new_foot_pos;
    }
    foot_pos_t yawFoot(foot_pos_t foot_pos, float yaw_angle) {
       /*
       * Calculates foot_pos after a yaw transform (extrinsic rotation about the body Z-axis).
       * The transform of the foot_pos is by the negative of the desired pitch angle.
       * 
       * ARGUMENTS:
       * + yaw_angle: A float representing the desired pitch angle of the body frame.
       * 
       * RETURNS:
       * + new_foot_pos: A foot_pos_t struct containing the x, y, z coordinates of the new foot_pos.
       */
      
      // Elementary rotation matrix about X-axis for pitch.
      float R_z[3][3] =
        { 
          {cos(-yaw_angle), -sin(-yaw_angle), 0},
          {sin(-yaw_angle), cos(-yaw_angle), 0},
          {0, 0, 1}
        };
      // Apply pitch rotation transform
      foot_pos_t new_foot_pos;
      new_foot_pos.x = foot_pos.x*R_z[0][0] + foot_pos.y*R_z[0][1] + foot_pos.z*R_z[0][2];
      new_foot_pos.y = foot_pos.x*R_z[1][0] + foot_pos.y*R_z[1][1] + foot_pos.z*R_z[1][2];
      new_foot_pos.z = foot_pos.x*R_z[2][0] + foot_pos.y*R_z[2][1] + foot_pos.z*R_z[2][2];
      // Return new_foot_pos
      return new_foot_pos;
    }
    void moveFootByBodyCommand(
      float roll_angle = 0,
      float pitch_angle = 0,
      float yaw_angle = 0)
    {
      /*
       * Calculates foot_pos from stance_pos after all transformations.
       * 
       * ARGUMENTS:
       * + roll_angle: The desired roll angle of the body frame.
       * + pitch_angle: The desired pitch angle of the body frame.
       */

      // Calculate final_foot_pos
      foot_pos_t final_foot_pos;
      final_foot_pos.x = stance_pos[0];
      final_foot_pos.y = stance_pos[1];
      final_foot_pos.z = stance_pos[2];
      
      final_foot_pos = rollFoot(final_foot_pos, roll_angle);
      final_foot_pos = pitchFoot(final_foot_pos, pitch_angle);
      final_foot_pos = yawFoot(final_foot_pos, yaw_angle);

      // Actuate final_foot_pos
      moveToBodyXYZ(final_foot_pos.x, final_foot_pos.y, final_foot_pos.z);
    }

    // INVERSE KINEMATIC FUNCTIONS //

    leg_angles_rad_t bodyIK(float B_x_dest = 0, float B_y_dest = 0, float B_z_dest = 0) {
      /*
       *  ARGUMENTS:
       *  + B_x_dest: The x-coordinate, defined in frame B, of the destination foot position
       *  + B_y_dest: The y-coordinate, defined in frame B, of the destination foot position
       *  + B_z_dest: The z-coordinate, defined in frame B, of the destination foot position
       *  
       *  RETURNS:
       *  + leg_angles_rad_t: A struct containing the calculated coxa, femur and tibia angles.
       */

      float B_x_dest_from_L = B_x_dest - B_x_L;
      float B_y_dest_from_L = B_y_dest - B_y_L;
      
      float B_p_dest_from_L[2];
      B_p_dest_from_L[0] = B_x_dest_from_L;
      B_p_dest_from_L[1] = B_y_dest_from_L;
      
      float theta = 0.0;
      if (B_p_dest_from_L[0] < 0 && B_p_dest_from_L[1] < 0) {
        theta = 2.0*PI + atan2(B_p_dest_from_L[1], B_p_dest_from_L[0]);
      } else {
        theta = atan2(B_p_dest_from_L[1], B_p_dest_from_L[0]);
      }
      
      float L_p_dest[3];
      // TODO: Figure out the problem with the legs!!
      // Realised the rotation matrix was not inverted (and fixed it),
      // but then why did all other legs except R_1 and L_3 work though??
      L_p_dest[0] = B_p_dest_from_L[0]*cos(theta) + B_p_dest_from_L[1]*sin(theta);
      L_p_dest[1] = B_p_dest_from_L[0]*(-sin(theta)) + B_p_dest_from_L[1]*cos(theta);
      L_p_dest[2] = B_z_dest;

      leg_angles_rad_t solved_leg_angles;
      solved_leg_angles = localIK(L_p_dest[0], L_p_dest[2]);
      solved_leg_angles.coxa_rad = theta;

      return solved_leg_angles;
    }

    leg_angles_rad_t localIK(float foot_local_x = 0, float foot_local_z = 0) {      
      /*  
       *  Calculates phi and rho (femur and tibia angles) from the x and z coordinates B_p_dest_from_L,
       *  where L is the frame attached to the coxa joint, with Z pointing up and X pointing
       *  along the coxa, and B_p_dest_from_L is the displacement of the commanded foot location from the
       *  origin of frame L, defined in frame B, the body frame.
       *  
       *  Returns a leg_angles_rad_t struct containing the calculated femur and tibia angles. Coxa
       *  angle is not calculated here, and is set to 0.
       */
      float eff_leg = float(sqrt(pow(foot_local_x - coxa_length, 2) + pow(foot_local_z, 2))); // effective leg length from femur joint to tibia joint
      float beta = acos((pow((eff_leg), 2) - pow(femur_length, 2) - pow(tibia_length, 2))/(-2*femur_length*tibia_length)); // angle between femur and tibia
      float alpha = acos((pow(eff_leg, 2) + pow(femur_length, 2) - pow(tibia_length, 2))/(2*eff_leg*femur_length));
      float phi = alpha + atan2(foot_local_z, foot_local_x - coxa_length);
      float rho = -PI + beta;
      
      leg_angles_rad_t calc_values;
      
      calc_values.coxa_rad = 0;
      calc_values.tibia_rad = rho;
      calc_values.femur_rad = phi;
      
      return calc_values;
    }
    
    // MISC FUNCTIONS //
    
    float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
      return (((x - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min);
    }
};

class Hexapod {
  private:
    float stance_diameter = 400;
    float stance_height = 80;

  public:
    Leg L_1;
    Leg L_2;
    Leg L_3;
    Leg R_1;
    Leg R_2;
    Leg R_3;

    // STRUCTS //

    struct foot_pos_t {
      float x = 0;
      float y = 0;
      float z = 0;
    };
    
    // CONSTRUCTORS //
    
    Hexapod() {
      Leg L_1();
      Leg L_2();
      Leg L_3();
      Leg R_1();
      Leg R_2();
      Leg R_3();
      stance_diameter = 400;
      stance_height = 80;
    }
    Hexapod(
      Leg L_1_arg,
      Leg L_2_arg,
      Leg L_3_arg,
      Leg R_1_arg,
      Leg R_2_arg,
      Leg R_3_arg,
      float stance_diameter_arg,
      float stance_height_arg)
    {
      L_1 = L_1_arg;
      L_2 = L_2_arg;
      L_3 = L_3_arg;
      R_1 = R_1_arg;
      R_2 = R_2_arg;
      R_3 = R_3_arg;
      stance_diameter = stance_diameter_arg;
      stance_height = stance_height_arg;
    }
    // SETTERS //

    void setStanceDiameter(float stance_diameter_arg) {
      stance_diameter = stance_diameter_arg;
    }
    void setStanceHeight(float stance_height_arg) {
      stance_height = stance_height_arg;
    }
    
    // GETTERS //

    float getStanceDiameter() {
      return stance_diameter;
    }
    float getStanceHeight() {
      return stance_height;
    }

    // ACTION FUNCTIONS //
    void updateLegsStancePos() {
      L_1.updateStancePos(stance_diameter, stance_height);
      L_2.updateStancePos(stance_diameter, stance_height);
      L_3.updateStancePos(stance_diameter, stance_height);
      R_1.updateStancePos(stance_diameter, stance_height);
      R_2.updateStancePos(stance_diameter, stance_height);
      R_3.updateStancePos(stance_diameter, stance_height);
    }
    void staticStance() {
      updateLegsStancePos();
      L_1.moveToStancePos();
      L_2.moveToStancePos();
      L_3.moveToStancePos();
      R_1.moveToStancePos();
      R_2.moveToStancePos();
      R_3.moveToStancePos();
    }
    void dynamicStance(
      float roll_angle = 0,
      float pitch_angle = 0,
      float yaw_angle = 0,
      float new_stance_height = 0)
    {
      // new_stance_height is an optional parameter
      if (new_stance_height != 0) {
        stance_height = new_stance_height;
      }

      updateLegsStancePos();
      L_1.moveFootByBodyCommand(roll_angle, pitch_angle, yaw_angle);
      L_2.moveFootByBodyCommand(roll_angle, pitch_angle, yaw_angle);
      L_3.moveFootByBodyCommand(roll_angle, pitch_angle, yaw_angle);
      R_1.moveFootByBodyCommand(roll_angle, pitch_angle, yaw_angle);
      R_2.moveFootByBodyCommand(roll_angle, pitch_angle, yaw_angle);
      R_3.moveFootByBodyCommand(roll_angle, pitch_angle, yaw_angle);
      
    }
};
