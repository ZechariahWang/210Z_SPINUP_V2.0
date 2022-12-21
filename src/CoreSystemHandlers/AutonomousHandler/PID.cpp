#include "main.h"
#include "vector"
#include "variant"
#include "array"

// Class init
TranslationPID mov_t;
RotationPID rot_r;

void TranslationPID::set_dt_constants(double n_wheelDiameter, double n_gearRatio, double n_motorCartridge){
  mov_t.wheelDiameter = n_wheelDiameter;
  mov_t.ratio = n_gearRatio;
  mov_t.cartridge = n_motorCartridge;
}

void TranslationPID::reset_t_alterables(){
  mov_t.t_derivative = 0;
  mov_t.t_integral = 0;
  mov_t.t_error = 0;
  mov_t.t_prev_error = 0;
  mov_t.t_iterator = 0;
  mov_t.t_error_thresh = 0;
  mov_t.t_failsafe = 0;
}

void RotationPID::reset_r_alterables(){
  rot_r.r_derivative = 0;
  rot_r.r_integral = 0;
  rot_r.r_error = 0;
  rot_r.r_prev_error = 0;
  rot_r.r_iterator = 0;
  rot_r.r_error_thresh = 0;
  rot_r.r_failsafe = 0;
}

void TranslationPID::set_t_constants(double kp, double ki, double kd, double r_kp){
  mov_t.t_kp = kp;
  mov_t.t_ki = ki;
  mov_t.t_kd = kd;
  mov_t.t_h_kp = r_kp;
}

void RotationPID::set_r_constants(double kp, double ki, double kd){
  rot_r.r_kp = kp;
  rot_r.r_ki = ki;
  rot_r.r_kd = kd;
}

double TranslationPID::find_min_angle(int targetHeading, int currentrobotHeading){
  double turnAngle = targetHeading - currentrobotHeading;
  if (turnAngle > 180 || turnAngle < -180){
    turnAngle = turnAngle - (utility::sgn(turnAngle) * 360);
  }
  return turnAngle;
}

double TranslationPID::compute_t(double current, double target){
  mov_t.t_error = target - current;
  mov_t.t_derivative = mov_t.t_error - mov_t.t_prev_error;
  if (mov_t.t_ki != 0){
    mov_t.t_integral += mov_t.t_error;
  }
  if (utility::sgn(mov_t.t_error) !=  utility::sgn(mov_t.t_prev_error)){
    mov_t.t_integral = 0;
  }

  double output = (mov_t.t_kp * mov_t.t_error) + (mov_t.t_ki * mov_t.t_ki) + (mov_t.t_kd * mov_t.t_kd);
  if (output * (12000.0 / 127) > mov_t.t_maxSpeed * (12000.0 / 127)) output = mov_t.t_maxSpeed;
  mov_t.t_prev_error = mov_t.t_error;
  return output;
}

double RotationPID::compute_r(double current, double target){
  rot_r.r_error = target - current;
  rot_r.r_derivative = rot_r.r_error - rot_r.r_prev_error;
  if (rot_r.r_ki != 0){
    rot_r.r_integral += rot_r.r_error;
  }
  if (utility::sgn(rot_r.r_error) !=  utility::sgn(rot_r.r_prev_error)){
    rot_r.r_integral = 0;
  }

  double output = (rot_r.r_kp * rot_r.r_error) + (rot_r.r_ki * rot_r.r_ki) + (rot_r.r_kd * rot_r.r_kd);
  if (output * (12000.0 / 127) > rot_r.r_maxSpeed * (12000.0 / 127)) output = rot_r.r_maxSpeed;
  rot_r.r_prev_error = rot_r.r_error;
  return output;
}

void TranslationPID::set_translation_pid(double target, double maxSpeed){
  utility::fullreset(0, false);
  mov_t.reset_t_alterables();
  double TARGET_THETA = ImuMon();
  double POSITION_TARGET = target;
  double cd = 0;
  mov_t.t_maxSpeed = maxSpeed;
  mov_t.circumfrance = mov_t.wheelDiameter * M_PI;
  mov_t.ticks_per_rev = (50.0 * (3600.0 / mov_t.cartridge) * mov_t.ratio);
  mov_t.ticks_per_inches = (mov_t.ticks_per_rev / mov_t.circumfrance);
  while (true){
    double currentPos = (DriveFrontLeft.get_position() + DriveFrontRight.get_position()) / 2;
    double vol = mov_t.compute_t(currentPos, target);
    double headingAssist = mov_t.find_min_angle(TARGET_THETA, ImuMon()) * mov_t.t_h_kp;
    cd++; if (cd <= 10){ utility::leftvreq(0); utility::rightvreq(0); continue;}

    utility::leftvreq(vol * (12000.0 / 127) + headingAssist);
    utility::rightvreq(vol * (12000.0 / 127) - headingAssist);
    if (fabs(mov_t.t_error) < mov_t.t_error_thresh) mov_t.t_iterator++;
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

void RotationPID::set_rotation_pid(double t_theta, double maxSpeed){
  utility::fullreset(0, false);
  rot_r.reset_r_alterables();
  while (true){
    double currentPos = imu_sensor.get_rotation();
    double vol = rot_r.compute_r(currentPos, t_theta);

    utility::leftvreq(vol);
    utility::rightvreq(-vol);
    if (fabs(rot_r.r_error) < rot_r.r_error_thresh) rot_r.r_iterator++;
    if (fabs(rot_r.r_iterator) > rot_r.r_tol){
      utility::stop();
      break;
    }
    if (fabs(rot_r.r_error - rot_r.r_prev_error) < 0.3) rot_r.r_failsafe++;
    if (rot_r.r_failsafe > 1000){
      utility::stop();
      break;
    }
  }
  pros::delay(10);
}


