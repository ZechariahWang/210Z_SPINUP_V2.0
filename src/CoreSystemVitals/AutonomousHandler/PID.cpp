/**
 * @file PID.cpp
 * @author Zechariah Wang
 * @brief PID logic for translation, rotation, curve, arc, and sim
 * @version 0.1
 * @date 2023-02-13
 * 
 */

#include "main.h"
#include "vector"
#include "variant"
#include "array"


Slew             slew;
FinalizeAuton    data;
TranslationPID   mov_t;
RotationPID      rot_r;
CurvePID         cur_c;
ArcPID           arc_a;
SimultaneousPID  sim_s;

// i got chlamydia from this

/**
 * @brief Set drivetrain specs
 * 
 * @param n_wheelDiameter the dimater of the drivetrain wheel
 * @param n_gearRatio The gear ratio of the drivetrain
 * @param n_motorCartidge The motor cartridge of the drivetrain
 */

// Set the drivetrain constants (wheel size, motor cartridge, etc)
void TranslationPID::set_dt_constants(const double n_wheelDiameter, const double n_gearRatio, const double n_motorCartridge){
  mov_t.wheelDiameter = n_wheelDiameter;
  mov_t.ratio = n_gearRatio;
  mov_t.cartridge = n_motorCartridge;
}

/**
 * @brief Slew init local data variables
 * 
 */

double get_ticks_per_inch(){
  double c = mov_t.wheelDiameter * M_PI; double tpr = (50.0 * (3600.0 / mov_t.cartridge) * mov_t.ratio);
  return (tpr / c);
}

void Slew::set_slew_min_power(std::vector<double> min_power){
  slew.min_power = min_power;
}

void Slew::set_slew_distance(std::vector<double> distance){
  slew.max_distance = distance;
}

/**
 * @brief Initialize slew data logic
 * 
 * @param slew_enabled Is the slew controller enabled
 * @param max_speec Max speed of slew
 * @param target_pos target end of slew
 * @param current_pos current position of slew
 * @param start init location of slew
 * @param backwards_enabled are we going backwards?
 * @param tpi ticks per inch
 */

void Slew::initialize_slew(bool slew_enabled, const double max_speed, const double target_pos, const double current_pos, const double start, bool backwards_enabled, double tpi){
  slew.enabled = slew_enabled;
  slew.max_speed = max_speed;
  slew.slew_ticks_per_inch = tpi;
  slew.sign = utility::sgn(target_pos - current_pos);
  slew.x_intercept = start + (slew.sign * slew.max_distance[backwards_enabled]) * slew.slew_ticks_per_inch; // gt fix
  slew.y_intercept = max_speed * slew.sign;
  slew.slope = (slew.sign * slew.min_power[backwards_enabled] - slew.y_intercept) / (slew.x_intercept - start);
}

/**
 * @brief PID class constructors
 * 
 */

TranslationPID::TranslationPID(){ // Translation PID Constructor
  mov_t.t_tol = 10;
  mov_t.t_error_thresh = 100;
}

RotationPID::RotationPID(){ // Rotation PID Constructor
  rot_r.r_tol = 10;
  rot_r.r_error_thresh = 3;
}

CurvePID::CurvePID(){ // Curve PID Constructor
  cur_c.c_tol = 20;
  cur_c.c_error_thresh = 3;
}

ArcPID::ArcPID(){ // Arc PID Constructor
  arc_a.a_tol = 20;
  arc_a.a_error_thresh = 10;
}

SimultaneousPID::SimultaneousPID(){
  sim_s.t_s_tol = 10;
  sim_s.t_s_error_thresh = 100;
  sim_s.c_s_tol = 20;
  sim_s.c_s_error_thresh = 3;
}

// Reset core translation variables
void TranslationPID::reset_t_alterables(){
  mov_t.t_derivative = 0;
  mov_t.t_integral = 0;
  mov_t.t_error = 0;
  mov_t.t_prev_error = 0;
  mov_t.t_iterator = 0;
  mov_t.t_failsafe = 0;
}

// Reset core rotation variables
void RotationPID::reset_r_alterables(){
  rot_r.r_derivative = 0;
  rot_r.r_integral = 0;
  rot_r.r_error = 0;
  rot_r.r_prev_error = 0;
  rot_r.r_iterator = 0;
  rot_r.r_failsafe = 0;
}

// Reset core curve variables
void CurvePID::reset_c_alterables(){
  cur_c.c_derivative = 0;
  cur_c.c_integral = 0;
  cur_c.c_error = 0;
  cur_c.c_prev_error = 0;
  cur_c.c_iterator = 0;
  cur_c.c_failsafe = 0;
  cur_c.c_rightTurn = false;
}

// Reset core arc variables
void ArcPID::reset_a_alterables(){
  arc_a.a_derivative = 0;
  arc_a.a_integral = 0;
  arc_a.a_error = 0;
  arc_a.a_prev_error = 0;
  arc_a.a_iterator = 0;
  arc_a.a_failsafe = 0;
  arc_a.a_rightTurn = false;
}

void SimultaneousPID::reset_sim_alterables(){
  sim_s.t_s_derivative = 0;
  sim_s.t_s_integral = 0;
  sim_s.t_s_error = 0;
  sim_s.t_s_prev_error = 0;
  sim_s.t_s_iterator = 0;
  sim_s.t_s_failsafe = 0;

  sim_s.c_s_derivative = 0;
  sim_s.c_s_integral = 0;
  sim_s.c_s_error = 0;
  sim_s.c_s_prev_error = 0;
  sim_s.c_s_iterator = 0;
  sim_s.c_s_failsafe = 0;
  sim_s.c_s_rightTurn = false;
}

/**
 * @brief PID class constants
 * 
 */

// Set translation PID constants
void TranslationPID::set_t_constants(const double kp, const double ki, const double kd, const double r_kp){
  mov_t.t_kp = kp;
  mov_t.t_ki = ki;
  mov_t.t_kd = kd;
  mov_t.t_h_kp = r_kp;
}

// Set rotation PID constants
void RotationPID::set_r_constants(const double kp, const double ki, const double kd){
  rot_r.r_kp = kp;
  rot_r.r_ki = ki;
  rot_r.r_kd = kd;
}

// Set curve PID constants
void CurvePID::set_c_constants(const double kp, const double ki, const double kd){
  cur_c.c_kp = kp;
  cur_c.c_ki = ki;
  cur_c.c_kd = kd;
}

// Set arc PID constants
void ArcPID::set_a_constants(const double kp, const double ki, const double kd){
  arc_a.a_kp = kp;
  arc_a.a_ki = ki;
  arc_a.a_kd = kd;
}

void SimultaneousPID::set_sim_t_constants(const double kp, const double ki, const double kd, const double r_kp){
  sim_s.t_s_kp = kp;
  sim_s.t_s_ki = ki;
  sim_s.t_s_kd = kd;
  sim_s.t_s_r_kp = r_kp;
}

void SimultaneousPID::set_sim_c_constants(const double kp, const double ki, const double kd){
  sim_s.c_s_kp = kp;
  sim_s.c_s_ki = ki;
  sim_s.c_s_kd = kd;
}

/**
 * @brief calculate the min angle needed to reach a target theta within 360 degrees
 * 
 * @param targetHeading the target heading of the robot
 * @param currentrobotHeading the current angle held by the robot
 * @return the shortest turn angle needed to reach target theta
 */

double TranslationPID::find_min_angle(int16_t targetHeading, int16_t currentrobotHeading){
  double turnAngle = targetHeading - currentrobotHeading;
  if (turnAngle > 180 || turnAngle < -180) { turnAngle = turnAngle - (utility::sgn(turnAngle) * 360); }
  return turnAngle;
}

/**
 * @brief calculate slew max voltage request
 * 
 * @param current Current position of slew
 * @return Slew speed
 */

double Slew::calculate_slew(const double current){
  if (slew.enabled){
    slew.error = slew.x_intercept - current;
    if (utility::sgn(slew.error) != slew.sign){
      slew.enabled = false;
    }
    else if (utility::sgn(slew.error) == slew.sign){
      return ((slew.slope * slew.error) + slew.y_intercept) * slew.sign;
    }
  }
  return slew.max_speed;
}

/**
 * @brief compute translation PID movement logic
 * 
 * @param current the current position of the robot
 * @param target the desired target of the robot
 * @return PID calculated voltage at that specific position relative to target
 */

double TranslationPID::compute_t(double current, double target){
  mov_t.t_error = target - current;
  mov_t.t_derivative = mov_t.t_error - mov_t.t_prev_error;
  if (mov_t.t_ki != 0){
    mov_t.t_integral += mov_t.t_error;
  }
  if (utility::sgn(mov_t.t_error) !=  utility::sgn(mov_t.t_prev_error)){
    mov_t.t_integral = 0;
  }

  double output = (mov_t.t_kp * mov_t.t_error) + (mov_t.t_integral * mov_t.t_ki) + (mov_t.t_derivative * mov_t.t_kd);
  if (output * (12000.0 / 127) > mov_t.t_maxSpeed * (12000.0 / 127)) output = mov_t.t_maxSpeed;
  if (output * (12000.0 / 127) < -mov_t.t_maxSpeed * (12000.0 / 127)) output = -mov_t.t_maxSpeed;
  mov_t.t_prev_error = mov_t.t_error;
  return output;
}

/**
 * @brief compute rotation PID movement logic
 * 
 * @param current the current raw IMU value of the robot
 * @param target the desired target theta of the robot (IN RAW IMU VALUES)
 * @return PID calculated voltage at that specific angle relative to target angle
 */

double RotationPID::compute_r(double current, double target){
  rot_r.r_error = target - imu_sensor.get_rotation();
  rot_r.r_derivative = rot_r.r_error - rot_r.r_prev_error;
  if (rot_r.r_ki != 0){ rot_r.r_integral += rot_r.r_error; }
  if (rot_r.r_error == 0 || rot_r.r_error > target){ rot_r.r_integral = 0; }

  double output = (rot_r.r_kp * rot_r.r_error) + (rot_r.r_integral * rot_r.r_ki) + (rot_r.r_derivative * rot_r.r_kd);
  if (output * (12000.0 / 127) >= rot_r.r_maxSpeed * (12000.0 / 127)) { output = rot_r.r_maxSpeed; }
  if (output * (12000.0 / 127) <= -rot_r.r_maxSpeed * (12000.0 / 127)) { output = -rot_r.r_maxSpeed; }
  rot_r.r_prev_error = rot_r.r_error;
  return output;
}

/**
 * @brief compute curve PID movement logic
 * 
 * @param current the current raw IMU value of the robot
 * @param target the desired target theta the robot will curve to
 * @return PID calculated voltage at that specific angle relative to target curve
 */

double CurvePID::compute_c(double current, double target){
  cur_c.c_error = target - imu_sensor.get_rotation();
  cur_c.c_derivative = cur_c.c_error - cur_c.c_prev_error;
  if (cur_c.c_ki != 0){
    cur_c.c_integral += cur_c.c_error;
  }
  if (utility::sgn(cur_c.c_error) !=  utility::sgn(cur_c.c_prev_error)){
    cur_c.c_integral = 0;
  }
  double output = (cur_c.c_kp * cur_c.c_error) + (cur_c.c_integral * cur_c.c_ki) + (cur_c.c_derivative * cur_c.c_kd);

  if (output * (12000.0 / 127) >= cur_c.c_maxSpeed * (12000.0 / 127)) { output = cur_c.c_maxSpeed; }
  if (output * (12000.0 / 127) <= -cur_c.c_maxSpeed * (12000.0 / 127)) { output = -cur_c.c_maxSpeed; }
  cur_c.c_prev_error = cur_c.c_error;
  return output;
}

/**
 * @brief compute arc PID movement logic
 * 
 * @param tx the target global X value calculated from Odometry logic
 * @param ty the target global Y value calculated from Odometry logic
 * @return PID calculated voltage at that specific coordinate vector relative to target vector
 */

double ArcPID::compute_a(double tx, double ty){
  arc_a.a_error = sqrt(pow(tx - gx, 2) + pow(ty - gy, 2));
  arc_a.a_derivative = arc_a.a_error - arc_a.a_prev_error;
  if (arc_a.a_ki != 0){
    arc_a.a_integral += arc_a.a_error;
  }
  if (utility::sgn(arc_a.a_error) !=  utility::sgn(arc_a.a_prev_error)){
    arc_a.a_integral = 0;
  }
  double output = (arc_a.a_kp * arc_a.a_error) + (arc_a.a_integral * arc_a.a_ki) + (arc_a.a_derivative * arc_a.a_kd);

  if (output * (12000.0 / 127) >= arc_a.a_maxSpeed * (12000.0 / 127)) { output = arc_a.a_maxSpeed; }
  if (output * (12000.0 / 127) <= -arc_a.a_maxSpeed * (12000.0 / 127)) { output = -arc_a.a_maxSpeed; }
  arc_a.a_prev_error = arc_a.a_error;
  return output;
}

/**
 * @brief compute simultaneous translation PID movement logic
 * 
 * @param translationTarget the target translation target position
 * @param translationCurrent the current position of the robot relative to the target location
 * @return PID calculated voltage at that specific position relative to the target position
 */

double SimultaneousPID::compute_sim_mov_pid(double translationTarget, double translationCurrent){
  sim_s.t_s_error = translationTarget - translationCurrent;
  sim_s.t_s_derivative = sim_s.t_s_error - sim_s.t_s_prev_error;
  if (sim_s.t_s_ki != 0){
    sim_s.t_s_integral += sim_s.t_s_error;
  }
  if (utility::sgn(sim_s.t_s_error) !=  utility::sgn(sim_s.t_s_prev_error)){ sim_s.t_s_integral = 0; }

  double output = (sim_s.t_s_kp * sim_s.t_s_error) + (sim_s.t_s_integral * sim_s.t_s_ki) + (sim_s.t_s_derivative * sim_s.t_s_kd);
  if (output * (12000.0 / 127) > sim_s.t_s_maxSpeed * (12000.0 / 127)) output = sim_s.t_s_maxSpeed;
  if (output * (12000.0 / 127) < -sim_s.t_s_maxSpeed * (12000.0 / 127)) output = -sim_s.t_s_maxSpeed;
  sim_s.t_s_prev_error = sim_s.t_s_error;
  return output;
}

/**
 * @brief compute simultaneous curve PID movement logic
 * 
 * @param current the current raw IMU value of the robot
 * @param target the desired target theta the robot will curve to
 * @return PID calculated voltage at that specific angle relative to target curve
 */

double SimultaneousPID::compute_sim_cur_pid(double curvetargetTheta, double curveCurrent){
  sim_s.c_s_error = curvetargetTheta - imu_sensor.get_rotation();
  sim_s.c_s_derivative = sim_s.c_s_error - sim_s.c_s_prev_error;
  if (sim_s.c_s_ki != 0){
    sim_s.c_s_integral += sim_s.c_s_error;
  }
  if (utility::sgn(sim_s.c_s_error) !=  utility::sgn(sim_s.c_s_prev_error)){
    sim_s.c_s_integral = 0;
  }
  double output = (sim_s.c_s_kp * sim_s.c_s_error) + (sim_s.c_s_integral * sim_s.c_s_ki) + (sim_s.c_s_derivative * sim_s.c_s_kd);

  if (output * (12000.0 / 127) >= sim_s.c_s_maxSpeed * (12000.0 / 127)) { output = sim_s.c_s_maxSpeed; }
  if (output * (12000.0 / 127) <= -sim_s.c_s_maxSpeed * (12000.0 / 127)) { output = -sim_s.c_s_maxSpeed; }
  sim_s.c_s_prev_error = sim_s.c_s_error;
  return output;
}

/**
 * @brief Driver PID function. Main logic function, combining all translation PID components together
 * 
 * @param target the target translation target position
 * @param maxSpeed the maxspeed the robot may travel at
 */

void TranslationPID::set_translation_pid(double target, double maxSpeed){
  utility::fullreset(0, false); mov_t.reset_t_alterables();
  double TARGET_THETA = ImuMon(); double POSITION_TARGET = target; bool is_backwards = false; int8_t cd = 0;
  mov_t.t_maxSpeed = maxSpeed;
  mov_t.circumfrance = mov_t.wheelDiameter * M_PI;
  mov_t.ticks_per_rev = (50.0 * (3600.0 / mov_t.cartridge) * mov_t.ratio);
  mov_t.ticks_per_inches = (mov_t.ticks_per_rev / mov_t.circumfrance);
  target *= mov_t.ticks_per_inches;
  double init_left_pos = DriveFrontLeft.get_position(); double init_right_pos = DriveFrontRight.get_position();
  while (true){
    data.DisplayData();
    double avgPos = (DriveFrontLeft.get_position() + DriveFrontRight.get_position()) / 2;
    double avg_voltage_req = mov_t.compute_t(avgPos, target);
    double headingAssist = mov_t.find_min_angle(TARGET_THETA, ImuMon()) * mov_t.t_h_kp;
    cd++; if (cd <= 10){ utility::leftvoltagereq(0); utility::rightvoltagereq(0); continue;}
    if (target < 0) { is_backwards = true; } else { is_backwards = false; }
    double l_output = avg_voltage_req;
    double r_output = avg_voltage_req;

    utility::leftvoltagereq((l_output * (12000.0 / 127)) + headingAssist);
    utility::rightvoltagereq((r_output * (12000.0 / 127)) - headingAssist);
    if (fabs(mov_t.t_error) < mov_t.t_error_thresh){ mov_t.t_iterator++; } else { mov_t.t_iterator = 0;}
    if (fabs(mov_t.t_iterator) > mov_t.t_tol){
      utility::stop();
      break;
    }
    if (fabs(mov_t.t_error - mov_t.t_prev_error) < 0.3) mov_t.t_failsafe++;
    if (mov_t.t_failsafe > 10000){
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Driver PID function with distance sensor input enabled. Main logic function, combining all translation PID components together
 * 
 * @param target the target translation target position
 * @param maxSpeed the maxspeed the robot may travel at
 * @param distanceVal the distance from an object for the robot to stop at
 */

void TranslationPID::set_translation_pid_with_location_params(double target, double maxSpeed, double distanceVal, double slewEnabled){
  utility::fullreset(0, false); mov_t.reset_t_alterables();
  double TARGET_THETA = ImuMon(); double POSITION_TARGET = target; bool is_backwards = false; int8_t cd = 0;
  mov_t.t_maxSpeed = maxSpeed;
  mov_t.circumfrance = mov_t.wheelDiameter * M_PI;
  mov_t.ticks_per_rev = (50.0 * (3600.0 / mov_t.cartridge) * mov_t.ratio);
  mov_t.ticks_per_inches = (mov_t.ticks_per_rev / mov_t.circumfrance);
  target *= mov_t.ticks_per_inches;
  double init_left_pos = DriveFrontLeft.get_position(); double init_right_pos = DriveFrontRight.get_position();
  while (true){
    if (distance_sensor.get() < distanceVal) { utility::stop(); break; }
    data.DisplayData();
    double avgPos = (DriveFrontLeft.get_position() + DriveFrontRight.get_position()) / 2;
    double avg_voltage_req = mov_t.compute_t(avgPos, target);
    double headingAssist = mov_t.find_min_angle(TARGET_THETA, ImuMon()) * mov_t.t_h_kp;
    double l_output = 0; double r_output = 0;
    cd++; if (cd <= 10){ utility::leftvoltagereq(0); utility::rightvoltagereq(0); continue;}
    if (target < 0) { is_backwards = true; } else { is_backwards = false; }
    if (slewEnabled){
      slew.initialize_slew(true, maxSpeed, target, ((DriveFrontLeft.get_position() + DriveFrontRight.get_position()) / 2), ((init_left_pos + init_right_pos) / 2), is_backwards, get_ticks_per_inch());
      double slew_output = slew.calculate_slew(avgPos);
      double l_output = utility::clamp(avg_voltage_req, -slew_output, slew_output);
      double r_output = utility::clamp(avg_voltage_req, -slew_output, slew_output);
    }
    else{
      l_output = avg_voltage_req;
      r_output = avg_voltage_req;
    }

    utility::leftvoltagereq((l_output * (12000.0 / 127)) + headingAssist);
    utility::rightvoltagereq((r_output * (12000.0 / 127)) - headingAssist);
    if (fabs(mov_t.t_error) < mov_t.t_error_thresh){ mov_t.t_iterator++; } else { mov_t.t_iterator = 0;}
    if (fabs(mov_t.t_iterator) > mov_t.t_tol){
      utility::stop();
      break;
    }
    if (fabs(mov_t.t_error - mov_t.t_prev_error) < 0.3) mov_t.t_failsafe++;
    if (mov_t.t_failsafe > 10000){
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Driver PID function with simultaneous cata reset data. Main logic function, combining all translation PID components together
 * 
 * @param target the target translation target position
 * @param maxSpeed the maxspeed the robot may travel at
 */

void TranslationPID::set_translation_pid_with_sim_reset(double target, double maxSpeed, double slewEnabled){
  utility::fullreset(0, false); mov_t.reset_t_alterables();
  double TARGET_THETA = ImuMon(); double POSITION_TARGET = target; bool is_backwards = false; int8_t cd = 0;
  mov_t.t_maxSpeed = maxSpeed;
  mov_t.circumfrance = mov_t.wheelDiameter * M_PI;
  mov_t.ticks_per_rev = (50.0 * (3600.0 / mov_t.cartridge) * mov_t.ratio);
  mov_t.ticks_per_inches = (mov_t.ticks_per_rev / mov_t.circumfrance);
  target *= mov_t.ticks_per_inches;
  double init_left_pos = DriveFrontLeft.get_position(); double init_right_pos = DriveFrontRight.get_position();
  while (true){
		CataPrimer.move_voltage(12000);
		if (CataLimitMonitor.get_value() == 1){
			CataPrimer.move_voltage(0);
			break;
		}
    data.DisplayData();
    double avgPos = (DriveFrontLeft.get_position() + DriveFrontRight.get_position()) / 2;
    double avg_voltage_req = mov_t.compute_t(avgPos, target);
    double headingAssist = mov_t.find_min_angle(TARGET_THETA, ImuMon()) * mov_t.t_h_kp;
    double l_output = 0; double r_output = 0;
    cd++; if (cd <= 10){ utility::leftvoltagereq(0); utility::rightvoltagereq(0); continue;}
    if (target < 0) { is_backwards = true; } else { is_backwards = false; }
    if (slewEnabled){
      slew.initialize_slew(true, maxSpeed, target, ((DriveFrontLeft.get_position() + DriveFrontRight.get_position()) / 2), ((init_left_pos + init_right_pos) / 2), is_backwards, get_ticks_per_inch());
      double slew_output = slew.calculate_slew(avgPos);
      double l_output = utility::clamp(avg_voltage_req, -slew_output, slew_output);
      double r_output = utility::clamp(avg_voltage_req, -slew_output, slew_output);
    }
    else{
      l_output = avg_voltage_req;
      r_output = avg_voltage_req;
    }
    utility::leftvoltagereq((l_output * (12000.0 / 127)) + headingAssist);
    utility::rightvoltagereq((r_output * (12000.0 / 127)) - headingAssist);
    if (fabs(mov_t.t_error) < mov_t.t_error_thresh){ mov_t.t_iterator++; } else { mov_t.t_iterator = 0;}
    if (fabs(mov_t.t_iterator) > mov_t.t_tol){
      utility::stop();
      CataPrimer.move_voltage(0);
      break;
    }
    if (fabs(mov_t.t_error - mov_t.t_prev_error) < 0.3) mov_t.t_failsafe++;
    if (mov_t.t_failsafe > 10000){
      utility::stop();
      CataPrimer.move_voltage(0);
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Driver Rotation PID function. Main logic function, combining all rotation PID components together
 * 
 * @param t_theta the target theta angle
 * @param maxSpeed the maxspeed the robot may make the turn in 
 */

void RotationPID::set_rotation_pid(double t_theta, double maxSpeed){
  utility::fullreset(0, false);
  rot_r.reset_r_alterables();
  rot_r.r_maxSpeed = maxSpeed;
  while (true){
    data.DisplayData();
    double currentPos = imu_sensor.get_rotation();
    double vol = rot_r.compute_r(currentPos, t_theta);

    utility::leftvoltagereq(vol * (12000.0 / 127));
    utility::rightvoltagereq(-vol * (12000.0 / 127));
    if (fabs(rot_r.r_error) < 3) { rot_r.r_iterator++; } else { rot_r.r_iterator = 0;}
    if (fabs(rot_r.r_iterator) >= 10){
      utility::stop();
      break;
    }
    if (fabs(rot_r.r_error - rot_r.r_prev_error) < 0.3) {rot_r.r_failsafe++;}
    if (rot_r.r_failsafe > 100000){
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Driver Curve PID function. Main logic function, combining all curve PID components together
 * 
 * @param t_theta the target theta angle
 * @param maxSpeed the maxspeed the robot may make the turn in 
 * @param curveDamper The amount the swing will be dampered by
 * @param backwards whether or not the robot will make the curve backwards or forwards
 */

void CurvePID::set_curve_pid(double t_theta, double maxSpeed, double curveDamper, bool backwards){
  utility::fullreset(0, false);
  cur_c.reset_c_alterables();
  cur_c.c_maxSpeed = maxSpeed;
  cur_c.c_rightTurn = false;
  while (true){
    data.DisplayData();
    double currentPos = imu_sensor.get_rotation();
    double vol = cur_c.compute_c(currentPos, t_theta);

    if (cur_c.c_error > 0){ cur_c.c_rightTurn = true; } else { cur_c.c_rightTurn = false;}
    if (cur_c.c_rightTurn == true && backwards == false){
      utility::leftvoltagereq(vol * (12000.0 / 127));
      utility::rightvoltagereq(vol * (12000.0 / 127) * curveDamper);
    }
    else if (cur_c.c_rightTurn == false && backwards == false){
      utility::leftvoltagereq(fabs(vol) * (12000.0 / 127) * curveDamper);
      utility::rightvoltagereq(fabs(vol) * (12000.0 / 127));
    }
    if (cur_c.c_rightTurn == true && backwards == true){
      utility::leftvoltagereq(-vol * (12000.0 / 127) * curveDamper);
      utility::rightvoltagereq(-vol * (12000.0 / 127));
    }
    else if (cur_c.c_rightTurn == false && backwards == true){
      utility::leftvoltagereq(vol * (12000.0 / 127));
      utility::rightvoltagereq(vol * (12000.0 / 127) * curveDamper);
    }
    if (fabs(cur_c.c_error) < cur_c.c_error_thresh) { cur_c.c_iterator++; } else { cur_c.c_iterator = 0;}
    if (fabs(cur_c.c_iterator) >= cur_c.c_tol){
      utility::stop();
      break;
    }
    if (fabs(cur_c.c_error - cur_c.c_prev_error) < 0.3) {cur_c.c_failsafe++;}
    if (cur_c.c_failsafe > 100000){
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Driver Arc PID function. Main logic function, combining all arc PID components together
 * 
 * @param t_x the target X position coordinate
 * @param t_y the target Y position coordinate
 * @param maxSpeed the max speed the robot may make the turn in 
 * @param arcDamper the amount the arc movement will be dampered by
 */

void ArcPID::set_arc_pid(double t_x, double t_y, double maxSpeed, double arcDamper){
  odom odometry;
  FinalizeAuton data;
  utility::fullreset(0, false);
  arc_a.reset_a_alterables();
  arc_a.a_maxSpeed = maxSpeed;
  arc_a.a_rightTurn = false;
  while (true){
    odometry.Odometry();
    data.DisplayData();
    double currentPos = imu_sensor.get_rotation();
    double vol = arc_a.compute_a(t_x, t_y);

    if (arc_a.a_error >= 0){ arc_a.a_rightTurn = true; } else { arc_a.a_rightTurn = false;}
    if (arc_a.a_rightTurn){
      utility::leftvoltagereq(vol * (12000.0 / 127));
      utility::rightvoltagereq(vol * (12000.0 / 127) * arcDamper);
    }
    else if (arc_a.a_rightTurn == false){
      utility::leftvoltagereq(fabs(vol) * (12000.0 / 127) * arcDamper);
      utility::rightvoltagereq(fabs(vol) * (12000.0 / 127));
    }
    if (fabs(arc_a.a_error) < arc_a.a_error_thresh) { arc_a.a_iterator++; } else { arc_a.a_iterator = 0;}
    if (fabs(arc_a.a_iterator) >= arc_a.a_tol){
      utility::stop();
      break;
    }
    if (fabs(arc_a.a_error - arc_a.a_prev_error) < 0.3) {arc_a.a_failsafe++;}
    if (arc_a.a_failsafe > 100000){
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Driver simultaneous PID function. CURRENTLY WIP, not in use
 * 
 * @param curveStartEnabled     the target theta angle
 * @param curvetargetThetaStart the maxspeed the robot may make the turn in 
 * @param curveDamperStart      Damper of initial curve
 * @param curveMaxSpeedStart    Speed of initial curve
 * @param backwardsStart        Curve backwards or forwards?
 * @param translationEnabled    Movement enabled
 * @param translationTarget     Movement target
 * @param translationMaxSpeed   Movement maxspeed
 * @param curveEndEnabled       Curve end enabled?
 * @param curvetargetThetaEnd   Target angle for curve
 * @param curveDamperEnd        Damper of final curve
 * @param curveMaxSpeedEnd      Speed of final curve
 * @param backwardsEnd          Backwards curve?
 */

// ima actually shoot myself fr
void SimultaneousPID::set_sim_pid(bool curveStartEnabled, double curvetargetThetaStart, double curveDamperStart, double curveMaxSpeedStart, bool backwardsStart, bool translationEnabled, double translationTarget, double translationMaxSpeed, bool curveEndEnabled, double curvetargetThetaEnd, double curveDamperEnd, double curveMaxSpeedEnd, bool backwardsEnd){
  utility::fullreset(0, false);
  sim_s.reset_sim_alterables();
  if (curveStartEnabled == false && translationEnabled == false && curveEndEnabled == false) {} // ONLY ONE EVENT STATUS SHOULD BE SET TO TRUE AT ONCE

  // These are the two most likely parameters that will be used. gt make more cut edge conditions in the future in case i become monkey
  if (curveStartEnabled && translationEnabled && curveEndEnabled) {sim_s.curvePhaseStart = true; sim_s.translationPhase = false; sim_s.curvePhaseEnd = false;}
  if (curveStartEnabled == false && translationEnabled == true && curveEndEnabled == true){sim_s.curvePhaseStart = false; sim_s.translationPhase = true; sim_s.curvePhaseEnd = false;}
  if (curveEndEnabled && translationEnabled == false && curveEndEnabled == false) {sim_s.curvePhaseStart = true; sim_s.translationPhase = false; sim_s.curvePhaseEnd = false;}
  if (curveEndEnabled == false && translationEnabled == false && curveEndEnabled == true) {sim_s.curvePhaseStart = false; sim_s.translationPhase = false ; sim_s.curvePhaseEnd = true;}

  // Initial cuvre phase
  if (curveStartEnabled && sim_s.curvePhaseStart == true) {
    utility::fullreset(0, false);
    cur_c.reset_c_alterables();
    cur_c.c_maxSpeed = 30;
    while (sim_s.curvePhaseStart == true && sim_s.translationPhase == false && sim_s.curvePhaseEnd == false){
      data.DisplayData();
      double currentPos = imu_sensor.get_rotation();
      double vol = cur_c.compute_c(currentPos, curvetargetThetaStart);
      vol =90;

      if (cur_c.c_error > 0){ cur_c.c_rightTurn = true; } else { cur_c.c_rightTurn = false;}
      if (cur_c.c_rightTurn == true && backwardsStart == false){
        utility::leftvoltagereq(vol * (12000.0 / 127));
        utility::rightvoltagereq(vol * (12000.0 / 127) * curveDamperStart);
      }
      else if (cur_c.c_rightTurn == false && backwardsStart == false){
        utility::leftvoltagereq(fabs(vol) * (12000.0 / 127) * curveDamperStart);
        utility::rightvoltagereq(fabs(vol) * (12000.0 / 127));
      }
      if (cur_c.c_rightTurn == true && backwardsStart == true){
        utility::leftvoltagereq(-vol * (12000.0 / 127) * curveDamperStart);
        utility::rightvoltagereq(-vol * (12000.0 / 127));
      }
      else if (cur_c.c_rightTurn == false && backwardsStart == true){
        utility::leftvoltagereq(vol * (12000.0 / 127));
        utility::rightvoltagereq(vol * (12000.0 / 127) * curveDamperStart);
      }
      if (fabs(cur_c.c_error) < 3) { cur_c.c_iterator++; } else { cur_c.c_iterator = 0;}
      if (fabs(cur_c.c_iterator) >= 10){
        if (translationEnabled == true && curveEndEnabled == true){
          sim_s.curvePhaseEnd = false;
          sim_s.curvePhaseStart = false;
          sim_s.translationPhase = true;
          break;
        }
        else{
          sim_s.curvePhaseEnd = false;
          sim_s.curvePhaseStart = false;
          sim_s.translationPhase = false;
          break;
        }
      }
      pros::delay(10);
    }
  }

  // Translation movement phase
  if (translationEnabled && sim_s.translationPhase == true) {
    utility::fullreset(0, false);
    mov_t.reset_t_alterables();
    mov_t.t_maxSpeed = 70;
    double TARGET_THETA = ImuMon();
    double POSITION_TARGET = translationTarget;
    int8_t cd = 0;
    sim_s.t_s_maxSpeed = translationMaxSpeed;
    mov_t.circumfrance = mov_t.wheelDiameter * M_PI;
    mov_t.ticks_per_rev = (50.0 * (3600.0 / mov_t.cartridge) * mov_t.ratio);
    mov_t.ticks_per_inches = (mov_t.ticks_per_rev / mov_t.circumfrance);
    translationTarget *= mov_t.ticks_per_inches;
    while (sim_s.curvePhaseStart == false && sim_s.translationPhase == true && sim_s.curvePhaseEnd == false){
      data.DisplayData();
      double avgPos = (DriveFrontLeft.get_position() + DriveFrontRight.get_position()) / 2;
      double avg_voltage_req = mov_t.compute_t(avgPos, translationTarget);
      double headingAssist = mov_t.find_min_angle(TARGET_THETA, ImuMon()) * mov_t.t_h_kp;
      cd++; if (cd <= 10){ utility::leftvoltagereq(0); utility::rightvoltagereq(0); continue;}

      utility::leftvoltagereq(avg_voltage_req * (12000.0 / 127) + headingAssist);
      utility::rightvoltagereq(avg_voltage_req * (12000.0 / 127) - headingAssist);
      if (fabs(mov_t.t_error) < 10){ mov_t.t_iterator++; } else { mov_t.t_iterator = 0;}
      if (fabs(mov_t.t_iterator) > 20){
        if (curveEndEnabled == true){
          sim_s.curvePhaseEnd = true;
          sim_s.curvePhaseStart = false;
          sim_s.translationPhase = false;
          break;
        }
        else{
          sim_s.curvePhaseEnd = false;
          sim_s.curvePhaseStart = false;
          sim_s.translationPhase = false;
          break;
        }
      }
      pros::delay(10);
    }
  }

  // End curve phase
  if (curveEndEnabled && sim_s.curvePhaseEnd == true) {
    utility::fullreset(0, false);
    sim_s.reset_sim_alterables();
    while (sim_s.curvePhaseStart == false && sim_s.translationPhase == false && sim_s.curvePhaseEnd == true){
      data.DisplayData();
      double currentPos = imu_sensor.get_rotation();
      double vol = cur_c.compute_c(currentPos, curvetargetThetaEnd);
      vol = 60;
      std::cout << "boltage" << vol << std::endl;

      if (cur_c.c_error > 0){ cur_c.c_rightTurn = true; } else { cur_c.c_rightTurn = false;}
      if (cur_c.c_rightTurn == true && backwardsEnd == false){
        utility::leftvoltagereq(vol * (12000.0 / 127));
        utility::rightvoltagereq(vol * (12000.0 / 127) * curveDamperEnd);
      }
      else if (cur_c.c_rightTurn == false && backwardsEnd == false){
        utility::leftvoltagereq(fabs(vol) * (12000.0 / 127) * curveDamperEnd);
        utility::rightvoltagereq(fabs(vol) * (12000.0 / 127));
      }
      if (cur_c.c_rightTurn == true && backwardsEnd == true){
        utility::leftvoltagereq(-vol * (12000.0 / 127) * curveDamperEnd);
        utility::rightvoltagereq(-vol * (12000.0 / 127));
      }
      else if (cur_c.c_rightTurn == false && backwardsEnd == true){
        utility::leftvoltagereq(vol * (12000.0 / 127));
        utility::rightvoltagereq(vol * (12000.0 / 127) * curveDamperEnd);
      }
      if (fabs(cur_c.c_error) < cur_c.c_error_thresh) { cur_c.c_iterator++; } else { cur_c.c_iterator = 0;}
      if (fabs(cur_c.c_iterator) >= cur_c.c_tol){
        sim_s.curvePhaseEnd = false;
        sim_s.curvePhaseStart = false;
        sim_s.translationPhase = false;
        utility::stop();
        break;
      }
      pros::delay(10);
    }
  }
}


