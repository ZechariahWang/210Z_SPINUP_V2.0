#include "main.h"
#include "display/lv_objx/lv_label.h"
#include "pros/motors.h"
#include "vector"
#include "variant"
#include "array"
#include "map"
#include "string"

const unsigned long int time = 100000; // Time until initialize phase ends. Effectively infinite.
const unsigned short int delayAmount = 10; // Dont overload the CPU during OP control

// wheres my dad ඞ
//-- LVGL object pointer initialization //--
lv_obj_t *displayDataL1;
lv_obj_t *displayDataL2;
lv_obj_t *displayDataL3;
lv_obj_t *displayDataL4;
lv_obj_t *displayDataL5;
lv_obj_t *debugLine1;
lv_obj_t *debugLine2;
lv_obj_t *finalizeAutonButton;
lv_obj_t *prevAutonButton;
lv_obj_t *nextAutonButton;

lv_obj_t *infoDisplay;
lv_obj_t *infoPage = lv_page_create(lv_scr_act(), NULL);

char buffer[100];
std::map<int, std::string> auton_Legend = {
    { 1, "Left Side Priority: Points" },
    { 2, "Right Side Priority: Points" },
    { 3, "Left Side Priority: WP" },
	{ 4, "Right Side Priority: WP " },
    { 5, "Solo Win Point (Left Side)" },
    { 6, "Skills" },
    { 7, "Empty Slot" },
    { 8, "Empty Slot" },
    { 9, "Empty Slot" },
    { 10, "Empty Slot" }
};

//-- LVGL on input functions //--
static lv_res_t btn_rel_action(lv_obj_t *btn){
	static bool pressed = true;
	if (pressed) {
		AutonFinalized = 1;
	} 
	return 0;
}

//-- LVGL prev auton selector functions
static lv_res_t onPrevPress(lv_obj_t *btn){
    SelectedAuton -= 1;
    if (SelectedAuton > 10){
        SelectedAuton = 1;
		sprintf(buffer, SYMBOL_LIST " Selected Path %d: %s", SelectedAuton, auton_Legend[SelectedAuton].c_str());
        lv_label_set_text(debugLine1, buffer);
    }
    else if (SelectedAuton < 1){
        SelectedAuton = 10;
		sprintf(buffer, SYMBOL_LIST " Selected Path %d: %s", SelectedAuton, auton_Legend[SelectedAuton].c_str());
        lv_label_set_text(debugLine1, buffer);
	}
	sprintf(buffer, SYMBOL_LIST " Selected Path %d: %s", SelectedAuton, auton_Legend[SelectedAuton].c_str());
    lv_label_set_text(debugLine1, buffer);
	return 1;
}

//-- LVGL next auton selector functions
static lv_res_t onNextPress(lv_obj_t *btn){
	SelectedAuton += 1;
    if (SelectedAuton > 10){
        SelectedAuton = 1;
		sprintf(buffer, SYMBOL_LIST " Selected Path %d: %s", SelectedAuton, auton_Legend[SelectedAuton].c_str());
        lv_label_set_text(debugLine1, buffer);
    }
    else if (SelectedAuton < 1){
        SelectedAuton = 10;
		sprintf(buffer, SYMBOL_LIST " Selected Path %d: %s", SelectedAuton, auton_Legend[SelectedAuton].c_str());
        lv_label_set_text(debugLine1, buffer);
    }
	sprintf(buffer, SYMBOL_LIST " Selected Path %d: %s", SelectedAuton, auton_Legend[SelectedAuton].c_str());
    lv_label_set_text(debugLine1, buffer);
	return 1;
}

void initialize() { // Init function control
	//-- New style initiation //--
    static lv_style_t style_new;                         
    lv_style_copy(&style_new, &lv_style_pretty);         
    style_new.body.radius = LV_RADIUS_CIRCLE;            
    style_new.body.main_color = LV_COLOR_BLACK;         
    style_new.body.grad_color = LV_COLOR_GRAY;         
    style_new.body.shadow.width = 8;                   
    style_new.body.border.width = 2;                    
    style_new.text.color = LV_COLOR_WHITE;                 

	//-- Info page style initiation //--
    static lv_style_t style_infoPage;                         
    lv_style_copy(&style_infoPage, &lv_style_pretty);              
    style_infoPage.body.main_color = lv_color_hsv_to_rgb(0, 0, 7);   
    style_infoPage.body.grad_color = lv_color_hsv_to_rgb(0, 0, 7);    
    style_infoPage.body.border.width = 1;   
	style_infoPage.text.color = LV_COLOR_WHITE; 

	lv_obj_set_size(infoPage, 500, 300);
	lv_obj_align(infoPage, NULL, LV_ALIGN_CENTER, 0, 30);  
	lv_obj_set_style(infoPage, &style_infoPage);

	//-- Debug Line //--
	debugLine1 = lv_label_create(infoPage, NULL);
    lv_label_set_text(debugLine1, " ");
    lv_obj_align(debugLine1, NULL, LV_ALIGN_CENTER, -200, -10);

	//-- Data line 1 //--
	displayDataL1 = lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL1, " ");
    lv_obj_align(displayDataL1, NULL, LV_ALIGN_CENTER, -200, -20);

	//-- Data line 2 //--
	displayDataL2= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL2, " ");
    lv_obj_align(displayDataL2, NULL, LV_ALIGN_CENTER, -200, -30);

	//-- Data line 3 //--
	displayDataL3= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL3, " ");
    lv_obj_align(displayDataL3, NULL, LV_ALIGN_CENTER, -200, -40);

	//-- Data line 4 //--
	displayDataL4= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL4, " ");
    lv_obj_align(displayDataL4, NULL, LV_ALIGN_CENTER, -200, -50);

	//-- Data line 5 //--
	displayDataL5= lv_label_create(infoPage, NULL);
    lv_label_set_text(displayDataL5, " ");
    lv_obj_align(displayDataL5, NULL, LV_ALIGN_CENTER, -200, -60);

	//-- Select Auton button //--
	finalizeAutonButton = lv_btn_create(lv_scr_act(), NULL);
	lv_btn_set_action(finalizeAutonButton, LV_BTN_ACTION_CLICK, btn_rel_action); 
    lv_obj_align(finalizeAutonButton, NULL, LV_ALIGN_CENTER, -18, 100);
	lv_obj_set_height(finalizeAutonButton, 40);
	lv_obj_set_width(finalizeAutonButton, 160);

	lv_obj_t *buttonText = lv_label_create(infoPage, NULL);
    lv_label_set_text(buttonText, "");  /*Set the text*/
    lv_obj_set_x(buttonText, 50); 

    buttonText = lv_label_create(finalizeAutonButton, NULL);
    lv_label_set_text(buttonText, SYMBOL_UPLOAD " SELECT");

	//-- Prev Auton button //--
	prevAutonButton = lv_btn_create(lv_scr_act(), NULL);
	lv_btn_set_action(prevAutonButton, LV_BTN_ACTION_CLICK, onPrevPress); 
    lv_obj_align(prevAutonButton, NULL, LV_ALIGN_CENTER, -155, 100);
	lv_obj_set_height(prevAutonButton, 40);
	lv_obj_set_width(prevAutonButton, 120);

	lv_obj_t *prevbuttonText = lv_label_create(infoPage, NULL);
    lv_label_set_text(prevbuttonText, "");  /*Set the text*/
    lv_obj_set_x(prevbuttonText, 30); 

    prevbuttonText = lv_label_create(prevAutonButton, NULL);
    lv_label_set_text(prevbuttonText, SYMBOL_PREV " PREV");

	//-- Next Auton button //--
	nextAutonButton = lv_btn_create(lv_scr_act(), NULL);
	lv_btn_set_action(nextAutonButton, LV_BTN_ACTION_CLICK, onNextPress); 
    lv_obj_align(nextAutonButton, NULL, LV_ALIGN_CENTER, 160, 100);
	lv_obj_set_height(nextAutonButton, 40);
	lv_obj_set_width(nextAutonButton, 120);

	lv_obj_t *nextbuttonText = lv_label_create(infoPage, NULL);
    lv_label_set_text(nextbuttonText, "");  /*Set the text*/
    lv_obj_set_x(nextbuttonText, 50); 

    nextbuttonText = lv_label_create(nextAutonButton, NULL);
    lv_label_set_text(nextbuttonText, SYMBOL_NEXT " NEXT");
 
	//-- Reset sensors and auton selector init //--
	pros::delay(3000);
	FinalizeAuton Init_Process;
	Init_Process.ResetAllPrimarySensors();
    Expansion.set_value(true);
	Launcher.set_value(true);
	OuterShooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	InnerShooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	DriveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	DriveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	DriveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	DriveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	DriveMidLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	DriveMidRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	sprintf(buffer, SYMBOL_LIST " Selected Path %d: %s", SelectedAuton, auton_Legend[SelectedAuton].c_str());
    lv_label_set_text(debugLine1, buffer);
	// Init_Process.ReceiveInput(time); // Enabled Auton Selector (STEP 1)
}

//--DONT TOUCH THESE FUNCTIONS--\*
void disabled() {}
void competition_initialize() {}
//------------------------------\*

// PID UNITS ARE IN INCHES
void autonomous(){  // Autonomous function control
	MotionAlgorithms Auton_Framework; // Auton framework class
	FinalizeAuton Init_Process; // Init framework class
	odom odometry;
	TranslationPID mov;
	RotationPID rot;
	CurvePID cur;
	ArcPID arc;
	odometry.Odometry();
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(2, 1, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	// Init_Process.SelectAuton(); // Enable Auton Selector (STEP 2)


    // mov.set_t_constants(0.45, 0, 5, 30);
	// mov.set_translation_pid(3, 110);

	// rot.set_r_constants(6, 0.003, 35);
	// rot.set_rotation_pid(90, 90);

	// mov.set_t_constants(0.45, 0, 5, 1.5);
	// mov.set_translation_pid(24, 90);

	// mov.set_t_constants(0.45, 0, 5, 1.5);
	// mov.set_translation_pid(-3, 90);


	// rot.set_r_constants(6, 0.003, 35);
	// rot.set_rotation_pid(-47, 90);

	// mov.set_t_constants(0.45, 0, 5, 30);
	// mov.set_translation_pid(60, 90);

	// pros::delay(500);

	// rot.set_r_constants(5, 0.003, 35);
	// rot.set_rotation_pid(45, 90);

	// rot.set_r_constants(5, 0.003, 35);
	// rot.set_rotation_pid(180, 40);

    // Auton_Framework.set_constants(7, 5, 5, 7);
    // Auton_Framework.move_to_reference_pose(40, 10, 45, 20);

	// PurePursuit2();

	// pros::delay(1000);

	// rot.set_r_constants(8, 0.003, 35);
	// rot.set_rotation_pid(-90, 110);


    mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(-2, 90);

	cur.set_c_constants(5, 0.003, 35);
	cur.set_curve_pid(-50, 90, 0);

	pros::delay(500);

    mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(2, 90);

	pros::delay(1000);

    mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(-2, 90);

	pros::delay(500);

	rot.set_r_constants(5, 0.003, 35);
	rot.set_rotation_pid(45, 90);

	pros::delay(500);

    mov.set_t_constants(0.45, 0, 5, 1.5);
	mov.set_translation_pid(28, 90);

	pros::delay(500);

	rot.set_r_constants(5, 0.003, 35);
	rot.set_rotation_pid(-28, 90);

	// PurePursuitTestPath();

	// rot.set_r_constants(5, 0.003, 35);
	// rot.set_rotation_pid(-90, 90);

	// pros::delay(1000);

	// mov.set_t_constants(0.45, 0, 5, 1.5);
	// mov.set_translation_pid(-5, 90);
}

void opcontrol(){ // Driver control function
	match_mov mov; // OP control framework class
	MotionAlgorithms Auton_Framework; // Auton framework class
	Init_AutonSwitchMain Init; // Init class framework
	FinalizeAuton data; // Data class
	odom odometry;
	char buffer[300];
	while (true){
		mov.dt_Control(); // Drivetrain control
		mov.power_intake(); // Intake control
		mov.launch_disk(); // Disk control
		mov.set_power_amount(); // Power control
		mov.power_shooter(); // Shooter control OVERRIDE 
		mov.set_motor_type();
		mov.init_expansion();

		odometry.Odometry();
		data.DisplayData(); // Display robot stats and info
		pros::delay(delayAmount); // Dont hog CPU ;)
	}
}

//lol ඞ
//background as black as kartik
// yes
