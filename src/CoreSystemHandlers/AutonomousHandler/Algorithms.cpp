#include "main.h"
#include "vector"
#include "array"
#include "iostream"
#include "algorithm"

MotionAlgorithms mtp;

// Find min angle between target heading and current heading
double find_min_angle(int targetHeading, int currentrobotHeading){
  double turnAngle = targetHeading - currentrobotHeading;
  if (turnAngle > 180 || turnAngle < -180){
    turnAngle = turnAngle - (utility::sgn(turnAngle) * 360);
  }
  return turnAngle;
}

int radian_to_degrees(double angle) { return angle * 180 / M_PI; }
int degrees_to_radians(double angle){ return angle * M_PI / 180; }

void MotionAlgorithms::set_constants(double t_kp, double r_kp, double f_tt, double t){
  mtp.t_kp = t_kp;
  mtp.r_kp = r_kp;
  mtp.target_final_tol = f_tt;
  mtp.target_tol = t;
}

void MotionAlgorithms::reset_mtp_constants(){
  mtp.distance = 0;
  mtp.alpha = 0;
  mtp.t_error = 0;
  mtp.beta = 0;
}

void MotionAlgorithms::reset_swing_alterables(){
  mtp.a_error = 0;
  mtp.a_rightTurn = false;
}

// Move to reference pose algorithm
void MotionAlgorithms::move_to_reference_pose(double targetX, double targetY, double targetHeading, double radius){
  MotionAlgorithms Auton_Framework;
  FinalizeAuton data;
  odom odometry;
  mtp.reset_mtp_constants();
  while (true){
    odometry.Odometry();
    data.DisplayData();
    double abstargetAngle = atan2f(targetX - gx, targetY - gy) * 180 / M_PI;
    if (abstargetAngle < 0){ abstargetAngle += 360; }

    mtp.distance = sqrt(pow(targetX - gx, 2) + pow(targetY - gy, 2));
    mtp.alpha = find_min_angle(abstargetAngle, targetHeading);
    mtp.t_error = find_min_angle(abstargetAngle, ImuMon());
    mtp.beta = atan(radius / mtp.distance) * 180 / M_PI;

    if (alpha < 0){ beta = -beta;}
    if (fabs(alpha) < fabs(beta)){ mtp.r_error = mtp.t_error + alpha; }
    else{ mtp.r_error = mtp.t_error + beta; }

    if (mtp.r_error > 180 || mtp.r_error < -180){ mtp.r_error = mtp.r_error - (utility::sgn(mtp.r_error) * 360); }

    double linearVel = mtp.t_kp * mtp.distance;
    double turnVel = mtp.r_kp * mtp.r_error;
    double closetoTarget = false;

    if (mtp.distance < mtp.target_tol){ closetoTarget = true;}
    if (closetoTarget){
      linearVel = mtp.t_kp * mtp.distance * utility::sgn(cos(mtp.r_error * M_PI / 180));
      mtp.r_error = find_min_angle(targetHeading, ImuMon());
      turnVel = mtp.r_kp * atan(tan(mtp.r_error * M_PI / 180)) * 180 / M_PI;
    }
    // if (fabs(linearVel) > (90 * (12000.0 / 127))) { linearVel = 90 * (12000.0 / 127); }

    int left_volage = linearVel + turnVel;
    int right_voltage = linearVel - turnVel;
    int linError_f = sqrt(pow(targetX - gx, 2) + pow(targetY - gy, 2));

    utility::leftvoltagereq(left_volage * (12000.0) / 127);
    utility::rightvoltagereq(right_voltage * (12000.0 / 127));

    if (fabs(sqrt(pow(targetX - gx, 2) + pow(targetY - gy, 2))) < mtp.target_final_tol){
      utility::leftvoltagereq(0);
      utility::rightvoltagereq(0);
      break;
    }
    pros::delay(10);
  }
}

void MotionAlgorithms::swing_to_point(double tx, double ty, double swingDamper){
    //mtp.reset_swing_alterables();
    double currentPos = imu_sensor.get_rotation();
    double abstargetAngle = atan2f(tx - gx, ty - gy) * 180 / M_PI;
    double targetTheta = find_min_angle(abstargetAngle, ImuMon());
    if (abstargetAngle < 0){ abstargetAngle += 360; }

    if (targetTheta >= 0 && targetTheta <= 180){ mtp.a_rightTurn = true;}
    else{ mtp.a_rightTurn = false; }
    if (fabs(targetTheta) < 1.5) // Close enough to theta just drive lmao
    { 
      utility::leftvoltagereq(targetTheta);
      utility::rightvoltagereq(targetTheta);
    }
    if (mtp.a_rightTurn){
      utility::leftvoltagereq(targetTheta * (12000.0 / 127));
      utility::rightvoltagereq(targetTheta * (12000.0 / 127) * swingDamper);
    }
    else if (mtp.a_rightTurn == false){
      utility::leftvoltagereq(fabs(targetTheta) * (12000.0 / 127) * swingDamper);
      utility::rightvoltagereq(fabs(targetTheta) * (12000.0 / 127));
    }
    pros::delay(10);
}

// Turn to target coordinate position
void MotionAlgorithms::TurnToPoint(int targetX, int targetY){
  odom odometry;
  odometry.Odometry();
  double finalAngle;
  double distanceX = targetX - gx;
  double distanceY = targetY - gy;
  double hypot = pow(distanceX, 2) + pow(distanceY, 2);
  double targetDistance = sqrt(hypot);
  double robotHeading = ImuMon();
  double ACTUALROBOTHEADING = imu_sensor.get_rotation();
  double resetAmount = robotHeading;

  if (resetAmount < 180) finalAngle = resetAmount;
  double angle = atan2f(distanceX, distanceY) * 180 / M_PI;
}




