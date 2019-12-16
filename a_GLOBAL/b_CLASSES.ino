
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
    float coxa_len = 0;
    float femur_len = 0;
    float tibia_len = 0;
    boolean is_left_leg = true;

    // LEG ANGLE LIMITS //

    float coxa_rad_max = radians(45); // Angles corresponding to servo pulse values in LEG SERVO PULSE LIMITS
    float coxa_rad_min = radians(-45);
    const float femur_rad_max = radians(161.17);
    const float femur_rad_min = radians(23.31);
    const float tibia_rad_max = radians(139.61);
    const float tibia_rad_min = radians(44.41);

    // LEG ANGLE OFFSETS //

    float coxa_rad_offset = radians(0); // To account for offset angle of front and back legs (with respect to middle legs)

    // LEG SERVO PULSE LIMITS //
    
    static const int16_t coxa_cw_limit = 200; // CW and CCW directions taking axis pointing up
    static const int16_t coxa_ccw_limit = 360;
    static const int16_t femur_up_limit = 380;
    static const int16_t femur_down_limit = 120;
    static const int16_t tibia_up_limit = 120;
    static const int16_t tibia_down_limit = 300;
    
  public:

    // VARIABLES USED OUTSIDE OF CLASS //
  
    static const int16_t coxa_middle = (coxa_cw_limit + coxa_ccw_limit)/2;
    static const int16_t femur_middle = (femur_up_limit + femur_down_limit)/2;
    static const int16_t tibia_middle = (tibia_up_limit + tibia_down_limit)/2;

    const float coxa_middle_rad = float_map(float(coxa_middle), coxa_cw_limit, coxa_ccw_limit, coxa_rad_min, coxa_rad_max);
    const float femur_middle_rad = float_map(float(femur_middle), femur_down_limit, femur_up_limit, femur_rad_min, femur_rad_max);
    const float tibia_middle_rad = float_map(float(tibia_middle), tibia_down_limit, tibia_up_limit, tibia_rad_min, tibia_rad_max);

    // STRUCTS //

    struct all_angles_rad {
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
      float coxa_len_def = 0;
      float femur_len_def = 0;
      float tibia_len_def = 0;
      float coxa_rad_offset_def = 0;
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
      float coxa_len_def = 0,
      float femur_len_def = 0,
      float tibia_len_def = 0,
      float coxa_rad_offset_def = 0,
      boolean is_left_leg_def = true)
    {
      driver = drive_def;
      coxa_pin = coxa_pin_def;
      femur_pin = femur_pin_def;
      tibia_pin = tibia_pin_def;
      coxa_offset = coxa_offset_def;
      femur_offset = femur_offset_def;
      tibia_offset = tibia_offset_def;
      coxa_len = coxa_len_def;
      femur_len = femur_len_def;
      tibia_len = tibia_len_def;
      coxa_rad_offset = coxa_rad_offset_def;
      is_left_leg = is_left_leg_def;
    }

    // SETTER FUNCTIONS //
    
    void set_driver(Adafruit_PWMServoDriver driver_object) {
      driver = driver_object;
    }
    void set_pins(uint8_t coxa_pin_num, uint8_t femur_pin_num, uint8_t tibia_pin_num) {
      coxa_pin = coxa_pin_num;
      femur_pin = femur_pin_num;
      tibia_pin = tibia_pin_num;
    }
    void set_offsets(uint16_t coxa, uint16_t femur, uint16_t tibia) {
      coxa_offset = coxa;
      femur_offset = femur;
      tibia_offset = tibia;
    }
    void set_lengths(float coxa, float femur, float tibia) {
      coxa_len = coxa;
      femur_len = femur;
      tibia_len = tibia;
    }
    void set_coxa_rad_offset(float coxa_rad_offset_val) {
      coxa_rad_offset = coxa_rad_offset_val;
    }

    // ACTION FUNCTIONS //
    
    void move_coxa(float coxa_rad_cmd) {
      driver.setPWM(coxa_pin, 0, constrain(float_map(coxa_rad_cmd, coxa_rad_min, coxa_rad_max, coxa_cw_limit, coxa_ccw_limit), coxa_cw_limit, coxa_ccw_limit) + coxa_offset);
    }
    void move_femur(float femur_rad_cmd) {
      driver.setPWM(femur_pin, 0, constrain(float_map(femur_rad_cmd, femur_rad_min, femur_rad_max, femur_down_limit, femur_up_limit), femur_down_limit, femur_up_limit) + femur_offset);
    }
    void move_tibia(float tibia_rad_cmd) {
      driver.setPWM(tibia_pin, 0, constrain(float_map(tibia_rad_cmd, tibia_rad_min, tibia_rad_max, tibia_down_limit, tibia_up_limit), tibia_up_limit, tibia_down_limit) + tibia_offset);
    }
    void move_leg(float end_z, float end_y, float end_x, float leg_flare = 50) {
      all_angles_rad vals = calc_leg(end_z, end_y, end_x, leg_flare);
      move_coxa(vals.coxa_rad);
      move_femur(vals.femur_rad);
      move_tibia(vals.tibia_rad);
    }

    // HOMING FUNCTIONS //

    void home_coxa() {
      move_coxa(coxa_middle_rad);
    }
    void home_femur() {
      move_femur(femur_middle_rad);
    }
    void home_tibia() {
      move_tibia(tibia_middle_rad);
    }

    // INVERSE KINEMATIC FUNCTIONS //

    all_angles_rad calc_stage_1(float z_height, float eff_leg_y) {      
      /*  Calculates femur and tibia joint angles from the desired z_height of the hexapod
       *  and the y-extension of the effective leg formed by the femur and tibia from the femur joint.
       *  +z down from femur joint axis, +y away from femur joint axis in coxa direction.  */  
            
      all_angles_rad calc_values;
      float alpha_angle = 0;
      float tibia_angle = 0;
      float eff_leg_rad = 0;
      float eff_leg = 0;
      float numerator = 0;
      float denominator = 0;

      eff_leg = float(sqrt(pow(eff_leg_y, 2) + pow(z_height, 2))); // effective leg length from femur joint to tibia joint
      tibia_angle = acos((pow((eff_leg), 2) - pow(femur_len, 2) - pow(tibia_len, 2))/(-2*femur_len*tibia_len)); // angle between femur and tibia
      eff_leg_rad = atan2(eff_leg_y, z_height);
      
      numerator = tibia_len*sin(tibia_angle);
      denominator = eff_leg;

      if ( eff_leg < 103 ) { // The point below which the femur angle is 90 degrees and some math sht happens
        alpha_angle = ((PI/2) - asin(numerator/denominator)) + PI/2;
      } else {
        alpha_angle = asin(numerator/denominator);
      }

      calc_values.coxa_rad = 0;
      calc_values.tibia_rad = tibia_angle;
      calc_values.femur_rad = alpha_angle + eff_leg_rad;
      
      return calc_values;
    }

    all_angles_rad calc_leg(float end_z, float end_y, float end_x, float leg_flare = 50) {
      /*  Takes in the x, y, z coordinates taking the coxa joint as origin and calculates
       *  the coxa angle (coxa_rad), and using that to calculate eff_leg_y, which in turn is
       *  fed into the calc_stage_1() function to calculate the femur and tibia angles.
       *  Outputs coxa_rad, femur_rad, tibia_rad in a struct
       *  +y away from coxa joint in coxa direction, +x in the y-cross-z direction from coxa in home position.  */

      all_angles_rad  output_angles_rad;
      
      float coxa_angle = 0;
      float eff_leg_y = 0;
      float angled_leg_x_offset = 0;
      float offset_end_x;
      float true_end_y = 0;

      angled_leg_x_offset = leg_flare*sin(coxa_rad_offset);

      if (is_left_leg) {
        true_end_y = leg_flare*cos(coxa_rad_offset) + end_y;
      } else {
        true_end_y = leg_flare*cos(coxa_rad_offset) - end_y;
      }

      if (is_left_leg) {
        offset_end_x = angled_leg_x_offset + end_x;
      } else {
        offset_end_x = angled_leg_x_offset - end_x;
      }
    
      coxa_angle = atan2(offset_end_x, true_end_y);
      eff_leg_y = (true_end_y/(cos(coxa_angle))) - coxa_len;
    
      output_angles_rad = calc_stage_1(end_z, eff_leg_y); // Now output_angles_rad has the femur and tibia angles, but coxa angle is still 0
      output_angles_rad.coxa_rad = coxa_rad_offset - coxa_angle; // Now output_angles_rad is complete with all angles in radians

      return output_angles_rad;      
    }
    
    // MISC FUNCTIONS //
    
    float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

};
