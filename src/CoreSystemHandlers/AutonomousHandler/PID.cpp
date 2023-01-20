#include "main.h"
#include "vector"
#include "variant"
#include "array"

// Class init
FinalizeAuton data;
TranslationPID mov_t;
RotationPID rot_r;
CurvePID cur_c;
ArcPID arc_a;

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

// Set the drivetrain constants (wheel size, motor cartridge, etc)
void TranslationPID::set_dt_constants(const double n_wheelDiameter, const double n_gearRatio, const double n_motorCartridge){
  mov_t.wheelDiameter = n_wheelDiameter;
  mov_t.ratio = n_gearRatio;
  mov_t.cartridge = n_motorCartridge;
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

// Find min angle between target angle and currrent angle using ANGLE WRAPPED SYSTEM
double TranslationPID::find_min_angle(int16_t targetHeading, int16_t currentrobotHeading){
  double turnAngle = targetHeading - currentrobotHeading;
  if (turnAngle > 180 || turnAngle < -180){ turnAngle = turnAngle - (utility::sgn(turnAngle) * 360); }
  return turnAngle;
}

// Compute translation logic
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

// Compute rotation logic
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
//

// Compute curve logic
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

// Compute arc logic
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

// Translation PID Driver
void TranslationPID::set_translation_pid(double target, double maxSpeed){
  utility::fullreset(0, false);
  mov_t.reset_t_alterables();
  double TARGET_THETA = ImuMon();
  double POSITION_TARGET = target;
  int8_t cd = 0;
  mov_t.t_maxSpeed = maxSpeed;
  mov_t.circumfrance = mov_t.wheelDiameter * M_PI;
  mov_t.ticks_per_rev = (50.0 * (3600.0 / mov_t.cartridge) * mov_t.ratio);
  mov_t.ticks_per_inches = (mov_t.ticks_per_rev / mov_t.circumfrance);
  target *= mov_t.ticks_per_inches;
  while (true){
    data.DisplayData();
    double avgPos = (DriveFrontLeft.get_position() + DriveFrontRight.get_position()) / 2;
    double avg_voltage_req = mov_t.compute_t(avgPos, target);
    double headingAssist = mov_t.find_min_angle(TARGET_THETA, ImuMon()) * mov_t.t_h_kp;
    cd++; if (cd <= 10){ utility::leftvoltagereq(0); utility::rightvoltagereq(0); continue;}

    utility::leftvoltagereq(avg_voltage_req * (12000.0 / 127) + headingAssist);
    utility::rightvoltagereq(avg_voltage_req * (12000.0 / 127) - headingAssist);
    if (fabs(mov_t.t_error) < mov_t.t_error_thresh){ mov_t.t_iterator++; } else { mov_t.t_iterator = 0;}
    if (fabs(mov_t.t_iterator) > mov_t.t_tol){
      utility::stop();
      break;
    }
    if (fabs(mov_t.t_error - mov_t.t_prev_error) < 0.3) mov_t.t_failsafe++;
    if (mov_t.t_failsafe > 300){
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}

// Rotation PID Driver
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

// Curve PID Driver
void CurvePID::set_curve_pid(double t_theta, double maxSpeed, double curveDamper){
  utility::fullreset(0, false);
  cur_c.reset_c_alterables();
  cur_c.c_maxSpeed = maxSpeed;
  cur_c.c_rightTurn = false;
  while (true){
    data.DisplayData();
    double currentPos = imu_sensor.get_rotation();
    double vol = cur_c.compute_c(currentPos, t_theta);

    if (cur_c.c_error > 0){ cur_c.c_rightTurn = true; } else { cur_c.c_rightTurn = false;}
    std::cout << (bool)cur_c.c_rightTurn << std::endl;
    if (cur_c.c_rightTurn == true){
      utility::leftvoltagereq(vol * (12000.0 / 127));
      utility::rightvoltagereq(vol * (12000.0 / 127) * curveDamper);
    }
    else if (cur_c.c_rightTurn == false){
      utility::leftvoltagereq(fabs(vol) * (12000.0 / 127) * curveDamper);
      utility::rightvoltagereq(fabs(vol) * (12000.0 / 127));
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

// Arc PID Driver
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


