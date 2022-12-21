#include "main.h"
#include "vector"
#include "variant"
#include "array"

odom w2_odom;

double gx;
double gy;

odom::odom(){
  w2_odom.deltaArcLength                              = 0;
  w2_odom.previousArcLength                           = 0;
  w2_odom.currentarclength                            = 0;

  w2_odom.d_currentForward                            = 0;
  w2_odom.d_currentCenter                             = 0;
  w2_odom.d_currentOtheta                             = 0;
  w2_odom.d_rotationTheta                             = 0;

  w2_odom.d_deltaForward                              = 0;
  w2_odom.d_deltaCenter                               = 0;
  w2_odom.d_deltaTheta                                = 0;
  w2_odom.d_deltaOTheta                               = 0;

  w2_odom.d_deltaTheory                               = 0;
  w2_odom.d_deltaTheory2                              = 0;
  w2_odom.d_Theory                                    = 0;
  w2_odom.d_Theory2                                   = 0;
  w2_odom.d_totalRotationTheta                        = 0;

  w2_odom.d_deltaX                                    = 0;
  w2_odom.d_deltaY                                    = 0;

  w2_odom.d_previousForward                           = 0;
  w2_odom.d_previousCenter                            = 0;
  w2_odom.d_previousOTheta                            = 0;
  w2_odom.d_previoustheta                             = 0;
  w2_odom.counter                                     = 0;
}

// Global heading function. Essentially, consider this the robots current heading.
double globalTheta = 0;
double ImuMon() {
  globalTheta = fmod(imu_sensor.get_rotation(), 360);
  while (globalTheta < 0) {
    globalTheta += 360;
  }
  while (globalTheta > 360) {
    globalTheta -= 360;
  }
  return globalTheta;
}

// This function is for the primary odom framework used within the robot.
void odom::Odometry(){
	double currentTime = pros::millis();
  double theta = imu_sensor.get_rotation();
  double RX = (cos(ImuMon() * M_PI / 180 + M_PI)); // Local X value
  double RY = (sin(ImuMon() * M_PI / 180 + M_PI)); // local Y value

  if (fmod(w2_odom.counter, 3) < 1){
    theta = std::abs(atan2f(RY, RX) + M_PI); // theta is in radians
    double localtheta = theta * 58.5; // Translated value relative to IMU values
    if (localtheta > 361 && localtheta < 368) {};
    localtheta = theta; // Updating translated theta value
  }

  double r = 29 / (2 * M_PI);
  double angleRadian = imu_sensor.get_rotation() * (M_PI / 180);
  w2_odom.currentarclength = angleRadian * r;

  double val = imu_sensor.get_rotation();
  double offset = (2 * val * 6) / 2.75;
  double imuval = imu_sensor.get_rotation();

  w2_odom.d_currentForward = (double(-ForwardAux.get_value()) * M_PI / 180);
  w2_odom.d_currentCenter = ((double(-RotationSensor.get_position()) * 3 / 500) * M_PI / 180);
  w2_odom.d_currentOtheta = theta;
  w2_odom.d_rotationTheta = ((w2_odom.d_deltaForward) / 14.375); // In case of no inertial, we can use encoders instead

  w2_odom.d_deltaForward = w2_odom.d_currentForward - w2_odom.d_previousForward;
  w2_odom.d_deltaCenter = w2_odom.d_currentCenter - w2_odom.d_previousCenter;
  w2_odom.d_deltaTheta = theta - w2_odom.d_previoustheta;
  w2_odom.d_deltaOTheta = w2_odom.d_currentOtheta - w2_odom.d_previousOTheta;
  w2_odom.deltaArcLength = w2_odom.currentarclength - w2_odom.previousArcLength;

  w2_odom.d_deltaTheory = w2_odom.d_deltaOTheta;
  w2_odom.d_deltaTheory2 = w2_odom.d_deltaOTheta;
  w2_odom.d_Theory += w2_odom.d_deltaTheory;
  w2_odom.d_Theory2 += w2_odom.d_deltaTheory2;
  w2_odom.d_totalRotationTheta += w2_odom.d_rotationTheta;

  w2_odom.d_deltaX = ((w2_odom.d_deltaForward) * 1 * -sin(-theta));
  w2_odom.d_deltaY = ((w2_odom.d_deltaForward) * 1 * cos(-theta));

  gx = gx + w2_odom.d_deltaX;
  gy = gy + w2_odom.d_deltaY;

  w2_odom.d_previousForward = w2_odom.d_currentForward;
  w2_odom.d_previousCenter = w2_odom.d_currentCenter;
  w2_odom.d_previousOTheta = w2_odom.d_currentOtheta;
  w2_odom.d_previoustheta = theta;
  w2_odom.previousArcLength = w2_odom.currentarclength;
}



