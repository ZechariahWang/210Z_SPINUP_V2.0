/**
 * @file Odometry.cpp
 * @author Zechariah Wang
 * @brief Odometry logic for global position tracking within robot
 * @version 0.1
 * @date 2023-02-13
 * 
 */

#include "main.h"
#include "vector"
#include "variant"
#include "array"

odom      w2_odom;  // odom class init
new_odom  pt;       // odom position tracking init
double    gx;       // global X
double    gy;       // global Y

static bool XDRIVE_ENABLED = true;

/**
 * @brief The current theta of the robot wrapped to 360 degrees
 * @return angle wrapped to 360 degrees from raw IMU sensor data
 */

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

/**
 * @brief Set position tracking constants
 * 
 * @param vertical_wheel_distance distance from the vertical wheel to the center of gravity within the robot
 * @param horizontal_wheel_distance distance from the horizontal wheel to the center of gravity within the robot
 */

void new_odom::set_pt_constants(const double vertical_wheel_distance, const double horizontal_wheel_distance) {
  pt.vertical_distance = vertical_wheel_distance;
  pt.horizontal_distance = horizontal_distance;
}

/**
 * @brief Conversion functions
 * 
 * @param angle Eithercurrent radian or degrees of robot
 * @return angle converted to desired unit
 */

int16_t radian_to_degrees(const double angle) { return angle * 180 / M_PI; } // convert radian to degrees
int16_t degrees_to_radians(const double angle){ return angle * M_PI / 180; } // Convert degrees to radian

/**
 * @brief New Odometry version 2.0 currently WIP
 *
 */

void new_odom::compute_odometry(){
  double vertical_delta = (-ForwardAux.get_value() - pt.prev_vertical);
  double horizontal_delta = ((double(-RotationSensor.get_position()) * 3 / 500) - pt.prev_horizontal);
  
  pt.vertical_value = -ForwardAux.get_value();
  pt.horizontal_value = ((double(-RotationSensor.get_position())) * 3 / 500);
  pt.heading_value = ImuMon();

  double heading_rad = degrees_to_radians(ImuMon());
  double prev_heading_rad = degrees_to_radians(pt.prev_heading_value);
  double delta_heading_rad = heading_rad - prev_heading_rad;

  double local_x = 0;
  double local_y = 0;

  if (delta_heading_rad == 0) { local_x = horizontal_delta; local_y = vertical_delta; }
  else { 
    // Formula from team 5225A for position vector
    local_x = (2 * std::sin(delta_heading_rad / 2)) * ((horizontal_delta / delta_heading_rad + pt.horizontal_distance)); 
    local_y = (2 * std::sin(delta_heading_rad / 2)) * ((vertical_delta / delta_heading_rad + pt.vertical_distance));
  }
  double local_polar_angle = 0;
  double local_polar_length = 0;
  if (local_x == 0 && local_y == 0){ local_polar_angle = 0; local_polar_length = 0; }
  else {
    local_polar_angle = atan2f(local_y, local_x);
    local_polar_length = sqrt(pow(local_x, 2) + pow(local_y, 2));
  }
  double global_polar_angle = local_polar_angle - prev_heading_rad - (delta_heading_rad / 2);
  double delta_x = local_polar_length * cos(global_polar_angle);
  double delta_y = local_polar_length * sin(global_polar_angle);

  gx += delta_x;
  gy += delta_y;

  pt.prev_heading_value = pt.heading_value;
  pt.prev_vertical = pt.vertical_value;
  pt.prev_horizontal = pt.horizontal_value;
}

/**
 * @brief Currently operating Odometry Logic 
 * 
 */

void odom::Odometry(){
	double currentTime = pros::millis();
  double theta = ImuMon();
  double RX = (cos(ImuMon() * M_PI / 180 + M_PI)); // Local X value
  double RY = (sin(ImuMon() * M_PI / 180 + M_PI)); // local Y value
  gpsData = gps_sensor.get_status();

  if (fmod(w2_odom.counter, 3) < 1){
    theta = std::abs(atan2f(RY, RX) + M_PI); // theta is in radians
    double localtheta = theta * 58.5; // Translated value relative to IMU values
    if (localtheta > 361 && localtheta < 368) {};
    localtheta = theta; // Updating translated theta value
  }

  double r = 29 / (2 * M_PI);
  double angleRadian = imu_sensor.get_rotation() * (M_PI / 180);
  double val = imu_sensor.get_rotation();
  double offset = (2 * val * 6) / 2.75;
  double imuval = imu_sensor.get_rotation();

  w2_odom.currentarclength         = angleRadian * r;
  w2_odom.d_currentForward         = (double(-ForwardAux.get_value()) * M_PI / 180);
  w2_odom.d_currentCenter          = ((double(RotationSensor.get_position()) * 3 / 500) * M_PI / 180);
  w2_odom.d_currentOtheta          = theta;
  w2_odom.d_rotationTheta          = ((w2_odom.d_deltaForward) / 14.375); // In case of no inertial, we can use encoders instead

  w2_odom.d_deltaForward           = w2_odom.d_currentForward - w2_odom.d_previousForward;
  w2_odom.d_deltaCenter            = w2_odom.d_currentCenter - w2_odom.d_previousCenter;
  w2_odom.d_deltaTheta             = theta - w2_odom.d_previoustheta;
  w2_odom.d_deltaOTheta            = w2_odom.d_currentOtheta - w2_odom.d_previousOTheta;
  w2_odom.deltaArcLength           = w2_odom.currentarclength - w2_odom.previousArcLength;

  w2_odom.d_deltaTheory            = w2_odom.d_deltaOTheta;
  w2_odom.d_deltaTheory2           = w2_odom.d_deltaOTheta *1.35*0.103*0.9566*.9436*1.066*.867*2.26*0.0677*0.8366*1.069*1.16*0.5611*.927/1.33/1.307;
  w2_odom.d_Theory                 += w2_odom.d_deltaTheory;
  w2_odom.d_Theory2                += w2_odom.d_deltaTheory2;
  w2_odom.d_totalRotationTheta     += w2_odom.d_rotationTheta;

  if (XDRIVE_ENABLED)
  {
    w2_odom.d_deltaX = ((w2_odom.d_deltaForward) * 1 * -sin(-theta)) - ((w2_odom.d_deltaCenter - w2_odom.d_deltaTheory2) * 1 * -cos(-theta));
    w2_odom.d_deltaY = ((w2_odom.d_deltaForward) * 1 * cos(-theta)) - ((w2_odom.d_deltaCenter - w2_odom.d_deltaTheory2) * 1 * -sin(-theta));
  }
  else
  {
    w2_odom.d_deltaX = ((w2_odom.d_deltaForward) * 1 * -sin(-theta));
    w2_odom.d_deltaY = ((w2_odom.d_deltaForward) * 1 * cos(-theta));
  }

  if (GPS_ENABLED) {gx = gpsData.x; gy = gpsData.y; }
  else{ gx = gx + w2_odom.d_deltaX; gy = gy + w2_odom.d_deltaY;}

  w2_odom.d_previousForward = w2_odom.d_currentForward;
  w2_odom.d_previousCenter = w2_odom.d_currentCenter;
  w2_odom.d_previousOTheta = w2_odom.d_currentOtheta;
  w2_odom.d_previoustheta = theta;
  w2_odom.previousArcLength = w2_odom.currentarclength;
}



