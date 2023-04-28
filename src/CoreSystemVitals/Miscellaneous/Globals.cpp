#include "main.h"
#include "pros/adi.hpp"

/**
 * @brief Primary Robot globals
 * 
 */

 bool GPS_ENABLED;

pros::Motor InnerShooter(80, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DiskIntakeBot(69, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::ADIDigitalOut Launcher('z');
pros::ADIDigitalOut LeftBrake('v');
pros::ADIDigitalOut RightBrake('n');
pros::ADIDigitalOut Angler('j');
pros::ADIDigitalOut YaoMing('g');
pros::ADIEncoder FrontAux('l', 'x', true);
pros::ADIEncoder ForwardAux('v', 'h', false);
pros::Rotation RotationSensor(79);
pros::Imu imu_sensor_secondary(56);
pros::Vision vision_sensor(32);
pros::Gps gps_sensor(99);
pros::c::gps_status_s_t gpsData;
pros::ADIDigitalIn AutonSwitchForward('t');
pros::ADIDigitalIn AutonSwitchBackward('z');

// kartik i beg pls just these ones
pros::Motor CataPrimer(12, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DiskIntakeTop(11, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::ADIDigitalOut Expansion('d');
pros::ADIDigitalIn CataLimitMonitor('a');
pros::ADIDigitalOut pistonBooster('b');
pros::ADIDigitalOut intakeLift('c');
pros::ADIDigitalOut expansionBlocker('e');
pros::ADIDigitalOut elasticEjection('f');
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu_sensor(1);
pros::Distance distance_sensor(3);
pros::Motor DriveFrontLeft(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveFrontRight(16, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackLeft(6, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackRight(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveMidLeft(7, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveMidRight(9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor OuterShooter(13, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS); // REAL ONE

/**
 * @brief Programming robot globals
 * 
 */

// // Motors
// pros::Motor DriveFrontLeft(69, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveFrontRight(20, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveBackLeft(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveBackRight(78, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveMidLeft(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DriveMidRight(16, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// pros::Motor OuterShooter(1, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor InnerShooter(14, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DiskIntakeBot(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor DiskIntakeTop(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// // // Pistons
// pros::ADIDigitalOut Launcher('n');
// pros::ADIDigitalOut Expansion('x');
// pros::ADIDigitalOut LeftBrake('k');
// pros::ADIDigitalOut RightBrake('p');

// // Controller
// pros::Controller controller(pros::E_CONTROLLER_MASTER);

// // Sensors
// pros::ADIEncoder FrontAux('e', 'g', true);
// pros::ADIEncoder ForwardAux('c', 'd', false);
// pros::Rotation RotationSensor(19);
// pros::Imu imu_sensor(1);
// pros::Imu imu_sensor_secondary(10);
// pros::Vision vision_sensor(3);

// // Switches
// pros::ADIDigitalIn AutonSwitchForward('a');
// pros::ADIDigitalIn AutonSwitchBackward('b');