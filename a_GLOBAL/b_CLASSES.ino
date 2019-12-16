
class Leg {  
  protected:
  
    // INHERENT ATTRIBUTES //
    
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
    boolean is_left_leg = true;

  private:
    float coxa_length = 41.92; // Measured in mm
    float femur_length = 45.00;
    float tibia_length = 112.68;
    
  public:
    // STRUCTS //

    struct leg_angles_rad {
      float coxa_rad;
      float femur_rad;
      float tibia_rad;
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
      boolean is_left_leg = true;
    }
    
    Leg(
      Adafruit_PWMServoDriver drive_def = 0x40,
      uint8_t coxa_pin_def = 0,
      uint8_t femur_pin_def = 0,
      uint8_t tibia_pin_def = 0,
      uint16_t coxa_offset_def = 0,
      uint16_t femur_offset_def = 0,
      uint16_t tibia_offset_def = 0,
      float B_x_L_def = 0,
      float B_y_L_def = 0,
      boolean is_left_leg_def = true)
    {
      driver = drive_def;
      coxa_pin = coxa_pin_def;
      femur_pin = femur_pin_def;
      tibia_pin = tibia_pin_def;
      coxa_offset = coxa_offset_def;
      femur_offset = femur_offset_def;
      tibia_offset = tibia_offset_def;
      B_x_L = B_x_L_def;
      B_y_L = B_y_L_def;
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
    void moveToBodyXYZ(float B_x, float B_y, float B_z) {
      leg_angles_rad vals = bodyIK(B_x, B_y, B_z);
      moveCoxaToAngle(vals.coxa_rad);
      moveFemurToAngle(vals.femur_rad);
      moveTibiaToAngle(vals.tibia_rad);
    }

    // INVERSE KINEMATIC FUNCTIONS //

    leg_angles_rad bodyIK(float B_x_dest = 0, float B_y_dest = 0, float B_z_dest = 0) {
      /*
       *  ARGUMENTS:
       *  + B_x_dest: The x-coordinate, defined in frame B, of the destination foot position
       *  + B_y_dest: The y-coordinate, defined in frame B, of the destination foot position
       *  + B_z_dest: The z-coordinate, defined in frame B, of the destination foot position
       *  
       *  RETURNS:
       *  + leg_angles_rad: A struct containing the calculated coxa, femur and tibia angles.
       */

      float B_x_dest_from_L = B_x_dest - B_x_L;
      float B_y_dest_from_L = B_y_dest - B_y_L;
      
      float B_p_dest_from_L[2];
      B_p_dest_from_L[0] = B_x_dest_from_L;
      B_p_dest_from_L[1] = B_y_dest_from_L;

      float theta = atan2(B_p_dest_from_L[0], B_p_dest_from_L[1]);
      
      float L_p_dest[3];
      L_p_dest[0] = B_p_dest_from_L[0]*(cos(theta) + sin(theta));
      L_p_dest[1] = B_p_dest_from_L[1]*(-sin(theta) + cos(theta));
      L_p_dest[2] = B_z_dest;

      leg_angles_rad solved_leg_angles;
      solved_leg_angles = localIK(L_p_dest[0], L_p_dest[2]);
      solved_leg_angles.coxa_rad = theta;

      return solved_leg_angles;
    }

    leg_angles_rad localIK(float foot_local_x = 0, float foot_local_z = 0) {      
      /*  
       *  Calculates phi and rho (femur and tibia angles) from the x and z coordinates B_p_dest_from_L,
       *  where L is the frame attached to the coxa joint, with Z pointing up and X pointing
       *  along the coxa, and B_p_dest_from_L is the displacement of the commanded foot location from the
       *  origin of frame L, defined in frame B, the body frame.
       *  
       *  Returns a leg_angles_rad struct containing the calculated femur and tibia angles. Coxa
       *  angle is not calculated here, and is set to 0.
       */

      float eff_leg = float(sqrt(pow(foot_local_x, 2) + pow(foot_local_z, 2))); // effective leg length from femur joint to tibia joint
      float beta = acos((pow((eff_leg), 2) - pow(femur_length, 2) - pow(tibia_length, 2))/(-2*femur_length*tibia_length)); // angle between femur and tibia
      float phi = atan2(foot_local_x - coxa_length, foot_local_z);
      float rho = -(PI - beta);
      
      leg_angles_rad calc_values;
      
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
