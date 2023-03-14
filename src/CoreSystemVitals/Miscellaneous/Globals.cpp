#include "main.h"

/**
 * @brief Primary Robot globals
 * 
 */

 bool GPS_ENABLED;

pros::Motor DriveFrontLeft(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveFrontRight(16, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackLeft(11, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveBackRight(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveMidLeft(89, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DriveMidRight(34, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor OuterShooter(13, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS); // REAL ONE
pros::Motor InnerShooter(80, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DiskIntakeBot(69, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DiskIntakeTop(27, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// Pistons
pros::ADIDigitalOut Launcher('z');
pros::ADIDigitalOut Expansion('a');
pros::ADIDigitalOut LeftBrake('v');
pros::ADIDigitalOut RightBrake('n');
pros::ADIDigitalOut Angler('c');
pros::ADIDigitalOut YaoMing('g');

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Sensors
pros::ADIEncoder FrontAux('l', 'x', true);
pros::ADIEncoder ForwardAux('v', 'h', false);
pros::Rotation RotationSensor(13);
pros::Imu imu_sensor(8);
pros::Imu imu_sensor_secondary(10);
pros::Vision vision_sensor(3);
pros::Gps gps_sensor(9);
pros::c::gps_status_s_t gpsData;

// Switches
pros::ADIDigitalIn AutonSwitchForward('t');
pros::ADIDigitalIn AutonSwitchBackward('z');

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