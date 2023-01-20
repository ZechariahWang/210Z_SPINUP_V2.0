#include "main.h"

void Run_MTRP_Debug(){ Debug_MTRP(); }
void Run_PID_Debug(){ PID_Debug(); }
void StandardAuton(){}
void __left_side_priority_5_disks__(){}
void __right_side_priority_5_disks__(){}
void __solo_wp__(){}
void __left_side_priority_roller__(){}
void __right_side_priority_roller__(){}

void __left_side_priority_8_disks__(){
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

    mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(-2, 90);

	cur.set_c_constants(5, 0.003, 35);
	cur.set_curve_pid(50, 90, 0);

    mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(24, 90);

	rot.set_r_constants(5, 0.003, 35);
	rot.set_rotation_pid(45, 90);

    mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(24, 90);
}

void __right_side_priority_8_disks__(){
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	odom odometry;
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
    
	rot.set_r_constants(5, 0.003, 35);
	rot.set_rotation_pid(-90, 90);

	mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(-22, 90);

	rot.set_r_constants(5, 0.003, 35);
	rot.set_rotation_pid(0, 90);

	mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(-5, 90);

	mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(1, 90);

	rot.set_r_constants(5, 0.003, 35);
	rot.set_rotation_pid(-48, 90);

	mov.set_t_constants(0.45, 0, 5, 5);
	mov.set_translation_pid(60, 90);

	rot.set_r_constants(5, 0.003, 35);
	rot.set_rotation_pid(45, 90);
}

void AutonSelectorPrimary(const u_int16_t autonType){
    switch (autonType){
    case 0:  __left_side_priority_8_disks__();  break;
    case 1: __solo_wp__();                      break;
    case 2: __left_side_priority_8_disks__();   break;
    case 3: __left_side_priority_5_disks__();   break;
    case 4: __left_side_priority_roller__();    break;
    case 5: __right_side_priority_8_disks__();  break;
    case 6: __right_side_priority_5_disks__();  break;
    case 7: __right_side_priority_roller__();   break;
    case 8:                                     break;
    case 9:                                     break;
    case 10:                                    break;
    default:                                    break;
    }
}