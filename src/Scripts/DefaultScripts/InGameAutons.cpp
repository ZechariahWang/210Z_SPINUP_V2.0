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

void a_leftSideDisk() // one after wp
{}

void shoot_iterator(){
    Angler.set_value(false); 
	for (int i = 0; i < 5; i++){
		pros::delay(1000);
		DiskIntakeTop.move_voltage(-12000);
		pros::delay(90);
		DiskIntakeTop.move_voltage(0);
	}
	DiskIntakeTop.move_voltage(9000);
	Angler.set_value(true); 
}

void shoot_iterator_left(){
    Angler.set_value(false); 
	DiskIntakeTop.move_voltage(-12000);
	pros::delay(180);
	DiskIntakeTop.move_voltage(0);
	for (int i = 0; i < 2; i++){
		pros::delay(1000);
		DiskIntakeTop.move_voltage(-12000);
		pros::delay(70);
		DiskIntakeTop.move_voltage(0);
	}
	pros::delay(600);
	DiskIntakeTop.move_voltage(-12000);
	pros::delay(500);
	Angler.set_value(true); 
}

void shoot_iterator_auto_1(){
	pros::delay(500);
    Angler.set_value(false); 
	DiskIntakeTop.move_voltage(-6000);
	pros::delay(200);
	DiskIntakeTop.move_voltage(-4000);
	pros::delay(1000);
	DiskIntakeTop.move_voltage(-4300);
	pros::delay(300);
	DiskIntakeTop.move_voltage(9000);
	Angler.set_value(true); 
}

void shoot_iterator_auto_wp(){
	pros::delay(500);
    Angler.set_value(false); 
	DiskIntakeTop.move_voltage(-6000);
	pros::delay(400);
    OuterShooter.move_voltage(11000);
	DiskIntakeTop.move_voltage(-3000);
	pros::delay(1500);
	DiskIntakeTop.move_voltage(-3800);
	pros::delay(1000);
	DiskIntakeTop.move_voltage(9000);
	Angler.set_value(true); 
}

void rightside_scuffed(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	YaoMing.set_value(false);

    DiskIntakeTop.move_voltage(12000);
    OuterShooter.move_voltage(12000);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-90, 90);

	pros::delay(1000);

	pros::delay(500);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(0, 90);

    DiskIntakeTop.move_voltage(12000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(25, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-45, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(30, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-135, 90);

	pros::delay(500);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-45, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-57, 70);

    DiskIntakeTop.move_voltage(-10000);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(0, 60, 0.15, true);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(0, 90);

    DiskIntakeTop.move_voltage(-10000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-4, 70);

	DiskIntakeTop.move_voltage(-10000);

	pros::delay(300);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(8, 70);

	DiskIntakeTop.move_voltage(10000);

	// EVERYTHING ELSE IS EXTRA

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-45, 90);
	
	DiskIntakeTop.move_voltage(10000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(65, 70);

    DiskIntakeTop.move_voltage(10000);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(42, 90);
}

void winpoint(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	// WINPOINT

    DiskIntakeTop.move_voltage(-9500);
    OuterShooter.move_voltage(11700);
	YaoMing.set_value(false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-3, 90);

	pros::delay(200);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(45, 90, 0.2, false);

   DiskIntakeTop.move_voltage(10000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(20, 70);

   mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(20, 40);
 
	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-33, 90);

	pros::delay(500);
	shoot_iterator_left();
	pros::delay(500);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(45, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(74, 70);

	YaoMing.set_value(true);
    Angler.set_value(false); 
	OuterShooter.move_voltage(6000);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-180, 90);

    DiskIntakeTop.move_voltage(-9500);


	// OuterShooter.move_voltage(6000);
	// DiskIntakeTop.move_voltage(-12000);
	// pros::delay(1000);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(-90, 90, 0.05, true);


    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-12, 90);

	pros::delay(500);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(3, 90);

	YaoMing.set_value(true);

}

void leftside(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	// WINPOINT

    DiskIntakeTop.move_voltage(-9500);
    OuterShooter.move_voltage(11700);
	YaoMing.set_value(false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-3, 90);

	pros::delay(200);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(45, 90, 0.2, false);

   	DiskIntakeTop.move_voltage(10000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(20, 70);

   	mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(20, 40);
 
	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-33, 90);

	pros::delay(500);
	shoot_iterator_left();
	pros::delay(500);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(45, 90);

   	mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-20, 40);
}

void autonskills(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	// WINPOINT

    DiskIntakeTop.move_voltage(-9500);
    OuterShooter.move_voltage(12000 * 0.9);
	YaoMing.set_value(false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-3, 90);

	pros::delay(200);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(45, 90, 0.2, false);

   DiskIntakeTop.move_voltage(10000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(20, 70);

   mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(20, 40);
 
	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-33, 90);

	pros::delay(500);
	shoot_iterator_left();
	pros::delay(500);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(45, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(74, 70);

	YaoMing.set_value(true);
    Angler.set_value(false); 
	OuterShooter.move_voltage(6000);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-180, 90);

    DiskIntakeTop.move_voltage(-9500);


	// OuterShooter.move_voltage(6000);
	// DiskIntakeTop.move_voltage(-12000);
	// pros::delay(1000);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(-90, 90, 0.05, true);


    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-12, 90);

	pros::delay(500);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(30, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-135, 90);

	Expansion.set_value(false);
	YaoMing.set_value(true);

	Expansion.set_value(false);
	YaoMing.set_value(true);
}

void new_right_side(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	YaoMing.set_value(false);

    DiskIntakeTop.move_voltage(12000);
    OuterShooter.move_voltage(11800);

    mov.set_t_constants(0.45, 0, 5, 80);
	mov.set_translation_pid(25, 50);

	rot.set_r_constants(5, 0, 45);
	rot.set_rotation_pid(25.5, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(5, 50);
	Angler.set_value(false);

	pros::delay(3500);

	shoot_iterator_left();

	pros::delay(500);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-5, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-45, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-28, 70);

    DiskIntakeTop.move_voltage(-8000);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(0, 60, 0.13, true);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(0, 90);

	DiskIntakeTop.move_voltage(-8000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-3.2, 70);

	pros::delay(300);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(8, 70);

	DiskIntakeTop.move_voltage(10000);
}

void shoot_skills(int voltage){
    Angler.set_value(false); 
	DiskIntakeTop.move_voltage(-voltage);
	pros::delay(2000);
    Angler.set_value(true); 
}

void provincial_skills(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	SimultaneousPID sim;
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);

    DiskIntakeTop.move_voltage(9500);
    OuterShooter.move_voltage(12000);
	YaoMing.set_value(false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-2.5, 30);

	pros::delay(200);

    DiskIntakeTop.move_voltage(12000);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(-70, 70, 0.2, false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(15, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(90, 90); 

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-11, 70);
	Angler.set_value(false); 

	pros::delay(300);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(0, 70, 0.2, false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(38, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(0, 90);
	imu_sensor.set_rotation(0);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-2.5, 90);
	
	shoot_skills(9550);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-30, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(45, 90);

    DiskIntakeTop.move_voltage(12000);
    OuterShooter.move_voltage(12000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(56, 70);

	rot.set_r_constants(5, 0, 45);
	rot.set_rotation_pid(-45, 70);
	
	mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(2, 70);
	Angler.set_value(false);
	
	pros::delay(200);
	shoot_skills(8800);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-19, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(45, 70);

    DiskIntakeTop.move_voltage(12000);
    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(37, 50);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(0, 90);

    DiskIntakeTop.move_voltage(12000);
    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(26, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(90, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(26, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(180, 90);
    Angler.set_value(false); 

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-13, 70);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(15, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(270, 90);


    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-13, 70);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(15, 70);

    OuterShooter.move_voltage(12000);


	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(180, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(51, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(179, 90);

	shoot_skills(8800);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-23, 70);

    OuterShooter.move_voltage(12000);

    DiskIntakeTop.move_voltage(9500);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(235, 70);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(50, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(135.5, 70);
	
	mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(5, 70);

	pros::delay(500);
	shoot_skills(8800);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-15, 70);

    DiskIntakeTop.move_voltage(9500);
    OuterShooter.move_voltage(12000);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(235, 70);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(40, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(315, 70);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(36, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(358.5, 70);

	shoot_skills(8800);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(135, 70);

    OuterShooter.move_voltage(12000);

    DiskIntakeTop.move_voltage(9500);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(46, 70);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(27, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(94, 70);

	pros::delay(500);

	shoot_skills(8800);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(11.5, 70);

    DiskIntakeTop.move_voltage(9500);
	OuterShooter.move_voltage(12000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(80, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(20, 40);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-89.5, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-89.5, 70);

	pros::delay(500);
	shoot_skills(8800);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-37, 90);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-136, 70);

	// expand

	Expansion.set_value(false);
}

void shoot_cata(){
    CataPrimer.move_voltage(12000); 
    pros::delay(100);
	pistonBooster.set_value(true);
	pros::delay(300);
    CataPrimer.move_voltage(0);
	pistonBooster.set_value(false);
}

void reset_cata(){
	while (true){
		CataPrimer.move_voltage(12000);
		if (CataLimitMonitor.get_value() == 1){
			CataPrimer.move_voltage(0);
			break;
		}
	}
}

void worlds_rightside_roller_only(){
	MotionAlgorithms Auton_Framework; FinalizeAuton Init_Process; Slew slew;
	TranslationPID mov; RotationPID rot; CurvePID cur; ArcPID arc; SimultaneousPID sim;

	DiskIntakeTop.move_voltage(8000);

    mov.set_t_constants(0.45, 0, 5, 700);
	mov.set_translation_pid(-20, 127);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(90, 90);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(-8, 127);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(5, 127);
}

void worlds_leftside(){
	MotionAlgorithms Auton_Framework; FinalizeAuton Init_Process; Slew slew;
	TranslationPID mov; RotationPID rot; CurvePID cur; ArcPID arc; SimultaneousPID sim;

	reset_cata();
	DiskIntakeTop.move_voltage(12000);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(-2, 127);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(5, 127);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-10, 90);

	shoot_cata();
	intakeLift.set_value(false);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid_with_sim_reset(-190, 90);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(-12, 127);

	intakeLift.set_value(true);
	pros::delay(1000);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(5, 127);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-10, 90);

	intakeLift.set_value(false);
	shoot_cata();

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid_with_sim_reset(-140, 127, 0.03, true);

	intakeLift.set_value(false);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(-15, 127);

	intakeLift.set_value(true);
	pros::delay(1000);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-30, 90);

	shoot_cata();
}

void worlds_rightside(){
	MotionAlgorithms Auton_Framework; FinalizeAuton Init_Process; Slew slew;
	TranslationPID mov; RotationPID rot; CurvePID cur; ArcPID arc; SimultaneousPID sim;
	reset_cata();

	DiskIntakeTop.move_voltage(12000);

    mov.set_t_constants(0.45, 0, 5, 700);
	mov.set_translation_pid(-42, 90);

	pros::delay(500);
	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-135, 90);

	shoot_cata();

    mov.set_t_constants(0.45, 0, 5, 200);
	mov.set_translation_pid(-3, 90);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid_with_sim_reset(-228, 90);

	DiskIntakeTop.move_voltage(12000);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(-35, 60);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-320, 90);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(-12, 127);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(12, 127);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-158, 90);
	shoot_cata();

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-233, 90);

	reset_cata();

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(-31, 127);

	DiskIntakeTop.move_voltage(10000);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-180, 90);


    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(-7, 60);

    mov.set_t_constants(0.45, 0, 5, 300);
	mov.set_translation_pid(3, 60);
}