// Motors lolllll

#include "main.h"
extern bool GPS_ENABLED;
extern pros::Motor DriveFrontLeft;
extern pros::Motor DriveFrontRight;
extern pros::Motor DriveBackLeft;
extern pros::Motor DriveBackRight;
extern pros::Motor DriveMidLeft;
extern pros::Motor DriveMidRight;
extern pros::ADIEncoder FrontAux;
extern pros::ADIEncoder ForwardAux;
extern pros::Imu imu_sensor;
extern pros::Imu imu_sensor_secondary;
extern pros::Vision vision_sensor;
extern pros::Rotation RotationSensor;
extern pros::Gps gps_sensor;
extern pros::c::gps_status_s_t gpsData;

extern pros::Motor OuterShooter;
extern pros::Motor InnerShooter;
extern pros::Motor DiskIntakeBot;
extern pros::Motor DiskIntakeTop;

extern pros::ADIDigitalOut Launcher;
extern pros::ADIDigitalOut Expansion;
extern pros::ADIDigitalOut LeftBrake;
extern pros::ADIDigitalOut RightBrake;
extern pros::ADIDigitalOut Angler;
extern pros::ADIDigitalOut YaoMing;

extern pros::ADIDigitalIn AutonSwitchForward;
extern pros::ADIDigitalIn AutonSwitchBackward;
extern pros::Controller controller;

extern pros::Motor CataPrimer;
extern pros::ADIDigitalIn CataLimitMonitor;

class Global {
    private:
        int init;

    public:
        double ImuMonitor();

};
