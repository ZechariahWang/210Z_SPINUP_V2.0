/**
 * @file main.cpp
 * @author Zechariah Wang
 * @brief Main logic file. Runs pre-init, init, auton, and op-control logic
 * @version 0.1
 * @date 2023-02-13
 */

#include "main.h"
#include "display/lv_objx/lv_label.h"
#include "pros/motors.h"
#include "vector"
#include "variant"
#include "array"
#include "map"
#include "string"

constexpr u_int64_t time           = 100000; // Time until initialize phase ends. Effectively infinite.
constexpr u_int16_t delayAmount    = 10; // Dont overload the CPU during OP control

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
    { 1, "Solo Win Point" },
    { 2, "LS Priority: Six Disks" },
    { 3, "LS Priority: Single Roller" },
	{ 4, "RS Priority: Six Disks " },
    { 5, "RS Priority: Single Roller" },
    { 6, "Empty Slot" },
    { 7, "Empty Slot" },
    { 8, "Empty Slot" },
    { 9, "Empty Slot" },
    { 10,"Empty Slot" }
};

//-- LVGL on input functions //--
static lv_res_t btn_rel_action(lv_obj_t *btn){
	static bool pressed = true;
	if (pressed) { AutonFinalized = 1; } 
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

	GPS_ENABLED = false; // DETERMINES WHETHER OR NOT USING GPS SENSOR
 
	//-- Reset sensors and auton selector init //--
	pros::delay(3000);
	FinalizeAuton Init_Process;
	Init_Process.ResetAllPrimarySensors();
    Expansion.set_value(true);
	Launcher.set_value(true);
    Angler.set_value(true); 
	OuterShooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	InnerShooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	DriveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	DriveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	DriveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	DriveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	DriveMidLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	DriveMidRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	sprintf(buffer, SYMBOL_LIST " Selected Path %d: %s", SelectedAuton, auton_Legend[SelectedAuton].c_str());
    lv_label_set_text(debugLine1, buffer);
	// Init_Process.ReceiveInput(time); // Enabled Auton Selector (STEP 1)
}

//--DONT TOUCH THESE FUNCTIONS--\*
void disabled() {}
void competition_initialize() {}
//------------------------------\*


/**
 * @brief Main autonomous function. Some key points to remember:
 * @brief 90 DEGREES CONSTANTS: 6, 0, 45
 * @brief 45 DEGREE CONSTANTS: 6, 0.003, 35
 * @brief PID units are in inches
 * 
 */

void autonomous(){  // Autonomous function control
	MotionAlgorithms Auton_Framework; FinalizeAuton Init_Process; 
	TranslationPID mov; RotationPID rot; CurvePID cur; ArcPID arc; SimultaneousPID sim;
	Auton_Framework.overRideCoordinatePos(0, 0);
	mov.set_dt_constants(3.125, 1.6, 600); // Parameters are : Wheel diameter, gear ratio, motor cartridge type
	imu_sensor.set_rotation(0);
	// Init_Process.ReceiveInput(time); // Enabled Auton Selector (STEP 1) ONLY FOR PROTOTYPE USE
	// Init_Process.SelectAuton(); // Enable Auton Selector (STEP 2) 
}

/**
 * @brief Main driver control function. All code used during game phase outside of auton
 * 
 */

void opcontrol(){ // Driver control function
	match_mov mov; MotionAlgorithms Auton_Framework; 
	Init_AutonSwitchMain Init;  FinalizeAuton data; 
	char buffer[300]; // Display Buffer
	YaoMing.set_value(true);
	while (true){
		mov.exponential_curve_accelerator();
		mov.power_intake(); // Intake control
		mov.launch_disk(); // Disk control
		mov.set_power_amount(); // Power control
		mov.misc_control();
		mov.set_motor_type(); // Set motor brake type
		mov.init_expansion(); // Initiate expansion
		mov.power_shooter(); // Shooter control OVERRIDE 

		data.DisplayData(); // Display robot stats and info
		pros::delay(delayAmount); // Dont hog CPU ;)
	}

}


// mov.on_off_v2();
//mov.on_off_controller(); // Bang bang controller
//lol ඞ
//background as black as kartik
