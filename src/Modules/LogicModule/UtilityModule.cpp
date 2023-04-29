/**
 * @file UtilityModule.cpp
 * @author Zechariah Wang
 * @brief Helper functions and utlity namespace
 * @version 0.1
 * @date 2023-02-13
 */

#include "main.h"

MotionAlgorithms Auton_Framework;

/**
 * @brief idek what happened here it just makes common functions easier to access
 * 
 */

namespace utility // Global utility namespace for helper functions within PID autons
{
  int sgn(double num){
    return (num < 0) ? -1 : ((num > 0) ? 1 : 0); // Returns -1 if num is negative, and 1 if num is positive.
  }

  double clamp(double num, double min, double max){
    if (num > max){ return max; }
    if (num < min) { return min; }
    else { return num; }
  }
  void stop(){
    DriveFrontLeft.move_voltage(0);
    DriveBackLeft.move_voltage(0);
    DriveFrontRight.move_voltage(0);
    DriveBackRight.move_voltage(0);
    DriveMidLeft.move_voltage(0);
    DriveMidRight.move_voltage(0);
  }

  void stop_v(){
    DriveFrontLeft.move_velocity(0);
    DriveBackLeft.move_velocity(0);
    DriveFrontRight.move_velocity(0);
    DriveBackRight.move_velocity(0); 
  }

  void leftvreq(double voltage){
    DriveFrontLeft.move_voltage(voltage);
    DriveBackLeft.move_voltage(voltage);
  }

  void rightvreq(double voltage){
    DriveFrontRight.move_voltage(voltage);
    DriveBackRight.move_voltage(voltage);
  }

  void leftvelreq(double velocity){
    DriveFrontLeft.move_velocity(velocity);
    DriveBackLeft.move_velocity(velocity);
    DriveMidLeft.move_velocity(velocity);
  }
 
  void rightvelreq(double velocity){
    DriveFrontRight.move_velocity(velocity);
    DriveBackRight.move_velocity(velocity);
    DriveMidRight.move_velocity(velocity);
  }

  void leftvoltagereq(double voltage){
    DriveFrontLeft.move_voltage(voltage);
    DriveBackLeft.move_voltage(voltage);
    DriveMidLeft.move_voltage(voltage);
  }

  void rightvoltagereq(double voltage){
    DriveFrontRight.move_voltage(voltage);
    DriveBackRight.move_voltage(voltage);
    DriveMidRight.move_voltage(voltage);
  }

  void fullreset(double resetval, bool imu){
    DriveFrontLeft.set_zero_position(resetval);
    DriveBackLeft.set_zero_position(resetval);
    DriveFrontRight.set_zero_position(resetval);
    DriveBackRight.set_zero_position(resetval);

    if (imu == true){
      imu_sensor.tare_rotation();
    }
  }

  void eclipse_fullreset(double resetval, bool imu){
    DriveFrontLeft.set_zero_position(resetval);
    DriveFrontRight.set_zero_position(resetval);

    if (imu == true){
      imu_sensor.tare_rotation();
    }
  }
}
/**
 * @brief Override coordinate position to desired position
 * 
 */

void MotionAlgorithms::overRideCoordinatePos(double new_gx, double new_gy){
  gx = new_gx;
  gy = new_gy;
}

