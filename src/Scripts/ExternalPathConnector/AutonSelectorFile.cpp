#include "main.h"

void shoot_iterator_2(){
    Angler.set_value(false); 
	for (int i = 0; i < 5; i++){
		pros::delay(1500);
		DiskIntakeTop.move_voltage(-12000);
		pros::delay(100);
		DiskIntakeTop.move_voltage(0);
	}
	DiskIntakeTop.move_voltage(9000);
	Angler.set_value(true); 
}

void shoot_iterator_auto_2(){
	pros::delay(500);
    Angler.set_value(false); 
	DiskIntakeTop.move_voltage(-6000);
	pros::delay(400);
	DiskIntakeTop.move_voltage(-3000);
	pros::delay(1500);
	DiskIntakeTop.move_voltage(-3300);
	pros::delay(1000);
	DiskIntakeTop.move_voltage(9000);
	Angler.set_value(true); 
}

void Run_MTRP_Debug(){ Debug_MTRP(); }
void Run_PID_Debug(){ PID_Debug(); }
void StandardAuton(){}

void __solo_wp__(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	odom odometry;
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	odometry.Odometry();
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	// Init_Process.ReceiveInput(time); // Enabled Auton Selector (STEP 1) ONLY FOR PROTOTYPE USE
	// Init_Process.SelectAuton(); // Enable Auton Selector (STEP 2) no

    DiskIntakeTop.move_voltage(-12000);
    OuterShooter.move_voltage(12000);
	YaoMing.set_value(false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-3, 90);

	pros::delay(200);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(45, 90, 0.4, false);

    DiskIntakeTop.move_voltage(12000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(40, 40);

    Angler.set_value(false); 
	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-38, 90);

  	mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(3, 40);

	pros::delay(2000);

    DiskIntakeTop.move_voltage(12000);

	shoot_iterator_2();

	pros::delay(500);


	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(45, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(74, 60);

    DiskIntakeTop.move_voltage(9000);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-180, 90);

	OuterShooter.move_voltage(6000);
    Angler.set_value(false); 
	DiskIntakeTop.move_voltage(-12000);
	pros::delay(1000);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(-90, 90, 0.05, true);

    DiskIntakeTop.move_voltage(-9000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-7, 90);

	pros::delay(500);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(3, 90);

	YaoMing.set_value(true);
}

void __left_side_priority_6_disks__(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	odom odometry;
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	odometry.Odometry();
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
    DiskIntakeTop.move_voltage(12000);
    OuterShooter.move_voltage(12000);
	YaoMing.set_value(true);

    DiskIntakeTop.move_voltage(10000);
    OuterShooter.move_voltage(12000);
	YaoMing.set_value(false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-3, 70);

	pros::delay(1000);

	cur.set_c_constants(6, 0, 45);
	cur.set_curve_pid(-75, 60, 0.15, false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(4, 70);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-2, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-10, 90);

	pros::delay(1000);

	shoot_iterator_2();

    DiskIntakeTop.move_voltage(0);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-7, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(45, 90);

    DiskIntakeTop.move_voltage(10000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(40, 50);

	pros::delay(1000);

    DiskIntakeTop.move_voltage(10000);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-28, 90);
	pros::delay(2500);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(4, 70);

	shoot_iterator_2();
}

void __right_side_priority_6_disks__(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	odom odometry;
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	odometry.Odometry();
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	YaoMing.set_value(false);

	YaoMing.set_value(false);

    DiskIntakeTop.move_voltage(12000);
    OuterShooter.move_voltage(12000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(25, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(25, 90);
	pros::delay(2000);


	shoot_iterator_auto_2();

	pros::delay(500);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(-45, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-29, 70);

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

void __left_side_priority_roller__(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	odom odometry;
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	odometry.Odometry();
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	// Init_Process.ReceiveInput(time); // Enabled Auton Selector (STEP 1) ONLY FOR PROTOTYPE USE
	// Init_Process.SelectAuton(); // Enable Auton Selector (STEP 2) no

    DiskIntakeTop.move_voltage(10000);
    OuterShooter.move_voltage(12000);
	YaoMing.set_value(false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-3, 70);

	pros::delay(1000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(2, 70);

}

void __right_side_priority_roller__(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	odom odometry;
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	odometry.Odometry();
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	// Init_Process.ReceiveInput(time); // Enabled Auton Selector (STEP 1) ONLY FOR PROTOTYPE USE
	// Init_Process.SelectAuton(); // Enable Auton Selector (STEP 2) no

    DiskIntakeTop.move_voltage(10000);
    OuterShooter.move_voltage(12000);
	YaoMing.set_value(false);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-19, 70);

	rot.set_r_constants(6, 0, 45);
	rot.set_rotation_pid(90, 90);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(-7, 70);

	pros::delay(1000);

    mov.set_t_constants(0.45, 0, 5, 50);
	mov.set_translation_pid(6, 70);
} 

void AutonSelectorPrimary(const u_int16_t autonType){
    switch (autonType){
    case 0: __solo_wp__();                      break;
    case 1: __solo_wp__();                      break;
    case 2: __left_side_priority_6_disks__();   break;
    case 3: __left_side_priority_roller__();    break;
    case 4: __right_side_priority_6_disks__();  break;
    case 5: __right_side_priority_roller__();   break;
    case 6:                                     break;
    case 7:                                     break;
    case 8:                                     break;
    case 9:                                     break;
    case 10:                                    break;
    default:                                    break;
    }
}