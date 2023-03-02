/**
 * @file Algorithms.cpp
 * @author Zechariah Wang
 * @brief MTRP algorithm, TTP, STP, CTP
 * @version 0.1
 * @date 2023-02-13
 * 
 */

#include "main.h"
#include "vector"
#include "array"
#include "iostream"
#include "algorithm"

double previousDriveError    = 0;
double previousTurnError     = 0;

const double t_kp            = 3;
const double t_ki            = 0.001;
const double t_kd            = 1.9;

double t_derivative          = 0;
double t_integral            = 0;
double t_tolerance           = 3;
double t_error               = 0;
double t_previouserror       = 0;
double t_multiplier          = 3000;
double t_averageposition     = 0;
double t_averageHeading      = 0;
double t_FailSafeCounter     = 0;
int t_threshholdcounter      = 0;

/**
 * @brief Find min angle to reach a target angle when wrapped to 360 degrees
 * 
 * @param targetHeading the target angle the robot reaches to turn to
 * @param currentrobotHeading the current robot angle
 * @return shortest path needed to reach desired angle
 */

double find_min_angle(const int16_t targetHeading, const int16_t currentrobotHeading){
  double turnAngle = targetHeading - currentrobotHeading;
  if (turnAngle > 180 || turnAngle < -180){ turnAngle = turnAngle - (utility::sgn(turnAngle) * 360); }
  return turnAngle;
}

/**
 * @brief Convert angle to desired unit
 * 
 * @param angle the current angle
 * @return angle in desired unit (degree or radians)
 */

int16_t radian_to_degrees_converter(const double angle) { return angle * 180 / M_PI; } // convert radian to degrees
int16_t degrees_to_radians_converter(const double angle){ return angle * M_PI / 180; } // Convert degrees to radian

MotionAlgorithms mtp; // move to point class material theme ocean

void MotionAlgorithms::set_constants(const double t_kp, const double r_kp, const double f_tt, const double t){ // Set constants
  mtp.t_kp = t_kp;
  mtp.r_kp = r_kp;
  mtp.target_final_tol = f_tt;
  mtp.target_tol = t;
}

void MotionAlgorithms::reset_mtp_constants(){ // Reset values
  mtp.distance = 0;
  mtp.alpha = 0;
  mtp.t_error = 0;
  mtp.beta = 0;
  mtp.iterator = 0;
}

void MotionAlgorithms::reset_swing_alterables(){ // Reset mtp values
  mtp.a_error = 0;
  mtp.a_rightTurn = false;
}

float Turn_PID(double t_theta){
  utility::fullreset(0, false);
  t_error = 0;
  t_previouserror = 0;
  t_integral = 0;
  t_derivative = 0;
  t_FailSafeCounter = 0;
  t_averageHeading = imu_sensor.get_rotation(); 
  t_error = t_theta - t_averageHeading; 
  t_integral += t_error; 
  if (t_error == 0 || t_error > t_theta) { t_integral = 0; }
  t_derivative = t_error - t_previouserror; 
  t_previouserror = t_error;
  double voltage = (t_error * t_kp * 0.01) * 94; 
  if(fabs(t_error) < t_tolerance){ t_threshholdcounter++; }
  else{ t_threshholdcounter = 0; }
  if (fabs(t_error - t_previouserror) < 0.3) { t_FailSafeCounter++; }
  else { t_FailSafeCounter = 0; }
  return voltage;
}

void MotionAlgorithms::simultaneous_mov_executor(double targetX, double targetY, double targetTheta, double translationSpeed, double rotationSpeed){
	odom Odom; FinalizeAuton data;
  while (true){
    Odom.Odometry(); data.DisplayData();
    
    double theta = ImuMon() * M_PI / 180;
    double driveError = sqrt(pow(targetX - gx, 2) + pow(targetY - gy, 2));
    double positionHypo = sqrt(pow(gx, 2) + pow(gy, 2));
    double driveOutput = (driveError * 13) + ((driveError - previousDriveError) * 1.3);

    double turnError = (-theta - targetTheta);
    turnError = atan2f(sinf(turnError), cosf(turnError));
    double turnOutput = Turn_PID(targetTheta);

    double angleDesired = atan2f(targetX - gx, targetY - gy);
    double angleDrive = (angleDesired - theta);
    angleDrive = atan2f(sinf(angleDrive), cosf(angleDrive));

    double velDrive = driveOutput * cos(angleDrive); 
    double velStrafe = driveOutput * sin(angleDrive);

    double speedFL; double speedBL;
    double speedFR; double speedBR;

    if(fabs(driveError) < 5 && fabs(turnError) < 3){ utility::leftvelreq(0); utility::rightvelreq(0); break; }
    else{
      speedFL = velDrive + velStrafe + turnOutput; speedBL = velDrive - velStrafe + turnOutput;
      speedFR = velDrive - velStrafe - turnOutput; speedBR = velDrive + velStrafe - turnOutput;
    }

    DriveFrontLeft.move_velocity((speedFL)); DriveBackLeft.move_velocity((speedBL));
    DriveFrontRight.move_velocity((speedFR)); DriveBackRight.move_velocity((speedBR));

    previousTurnError = turnError; 
    previousDriveError = driveError;
    pros::delay(10);
  }
}

/**
 * @brief move to a specific position at a specific angle
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 * @param targetHeading the target angle in degrees
 * @param radius the radius of the arc
 */

void MotionAlgorithms::move_to_reference_pose(const double targetX, const double targetY, const double targetHeading, const double radius){
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

    int16_t left_volage = linearVel + turnVel;
    int16_t right_voltage = linearVel - turnVel;
    int16_t linError_f = sqrt(pow(targetX - gx, 2) + pow(targetY - gy, 2));

    utility::leftvoltagereq(left_volage * (12000.0) / 127);
    utility::rightvoltagereq(right_voltage * (12000.0 / 127));

    if (fabs(sqrt(pow(targetX - gx, 2) + pow(targetY - gy, 2))) < mtp.target_final_tol)
    { 
      utility::stop();
      break;
    }
    else {mtp.iterator = 0;}
    if (mtp.iterator > 10) {
      utility::stop();
      break;
    }
    pros::delay(10);
  }
}

/**
 * @brief Swing to a specific coordinate position. Used for Pure Pursuit, and other motion algorithms
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 * @param swingDamper the amount the arc is dampered by
 */

// Swing to desired point
void MotionAlgorithms::swing_to_point(const double tx, const double ty, const double swingDamper){
  mtp.reset_swing_alterables();
  double defaultVoltage = 40;
  double abstargetAngle = atan2f(tx - gx, ty - gy) * 180 / M_PI;
  if (abstargetAngle < 0){ abstargetAngle += 360; }
  double targetTheta = find_min_angle(abstargetAngle, imu_sensor.get_rotation()) * 100;
  utility::leftvoltagereq((defaultVoltage * (12000.0 / 127)) + targetTheta);
  utility::rightvoltagereq((defaultVoltage * (12000.0 / 127)) - targetTheta);
}

// Curve to desired point
void curve_to_point(const double tx, const double ty, const double curveDamper){
  utility::leftvoltagereq(100 * (12000.0 / 127));
  utility::rightvoltagereq(100 * (12000.0 / 127));
}

/**
 * @brief Turn to a specific coordinate position. ONLY IN QUADRANT 1
 * 
 * @param targetX the target x coordinate
 * @param targetY the target y coordinate
 */

// Turn to target coordinate position
void MotionAlgorithms::TurnToPoint(const int targetX, const int targetY){
  odom odometry;
  RotationPID rot;
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
	rot.set_r_constants(5, 0.003, 35);
	rot.set_rotation_pid(angle, 90);
}




