#include "main.h"

void shoot_y(){
	Launcher.set_value(true);
	pros::delay(500);
	Launcher.set_value(false);
	pros::delay(500);
}

void a_rightSideDisk(){ // one during wp
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	odom odometry;
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	odometry.Odometry();
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(2, 1, 200); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);

	rot.set_r_constants(5, 0.003, 35);
	rot.set_rotation_pid(90, 90);
}

void a_leftSideDisk(){ // one after wp

}