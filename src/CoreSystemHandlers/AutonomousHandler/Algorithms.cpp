#include "main.h"
#include "vector"
#include "array"
#include "iostream"
#include "algorithm"

// Find min angle between target heading and current heading
int find_min_angle(int targetHeading, int currentrobotHeading){
  double turnAngle = targetHeading - currentrobotHeading;
  if (turnAngle > 180 || turnAngle < -180){
    turnAngle = turnAngle - (utility::sgn(turnAngle) * 360);
  }
  return turnAngle;
}

int radian_to_degrees(double angle) { return angle * 180 / M_PI; }
int degrees_to_radians(double angle){ return angle * M_PI / 180; }

double targetTolerance = 5;
double finalLocTolerance = 5;
double kp_lin = 13;
double kp_turn = 3.2;

// Move to reference pose algorithm
void MotionAlgorithms::MTRP(double tx, double ty, double targetHeading, double GlobalHeading){
  MotionAlgorithms Auton_Framework;
  FinalizeAuton data;
  odom odometry;
  while (true){
    odometry.Odometry();
    data.DisplayData();

    double currentX = gx;
    double currentY = gy;
    double targetX = tx;
    double targetY = ty;

    double abstargetAngle = atan2f(targetX - gx, targetY - gy) * 180 / M_PI;

    if (abstargetAngle < 0){
      abstargetAngle += 360;
    }

    double D = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
    double alpha = find_min_angle(abstargetAngle, targetHeading);
    double errorTerm1 = find_min_angle(abstargetAngle, ImuMon());

    double beta = atan(1/ D) * 180 / M_PI;
    double turn_Error;

    if (alpha < 0){
      beta = -beta;
    }

    if (fabs(alpha) < fabs(beta)){
      turn_Error = errorTerm1 + alpha;
    }
    else{
      turn_Error = errorTerm1 + beta;
    }

    if (turn_Error > 180 || turn_Error < -180){
      turn_Error = turn_Error - (utility::sgn(turn_Error) * 360);
    }

    int linearVel = kp_lin * D;
    int turnVel = kp_turn * turn_Error;

    double closetoTarget = false;

    if (D < targetTolerance){
      closetoTarget = true;
    }
    if (closetoTarget){
      linearVel = kp_lin * D * utility::sgn(cos(turn_Error * M_PI / 180));
      turn_Error = find_min_angle(targetHeading, ImuMon());
      turnVel = kp_turn * atan(tan(turn_Error * M_PI / 180)) * 180 / M_PI;
    }

    if (abs(linearVel) > (350 - abs(turnVel))){
      linearVel = 350 - abs(turnVel);
    }

    int leftVel_f = linearVel + turnVel;
    int rightVel_f = linearVel - turnVel;
    int linError_f = sqrt(pow(tx - gx, 2) + pow(ty - gy, 2));

    utility::leftvelreq(leftVel_f);
    utility::rightvelreq(rightVel_f);

    if ((fabs(targetX - gx) < finalLocTolerance) && (fabs(targetY - gy) < finalLocTolerance)){
      utility::leftvelreq(0);
      utility::rightvelreq(0);
      break;
    }
    pros::delay(10);
  }
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




