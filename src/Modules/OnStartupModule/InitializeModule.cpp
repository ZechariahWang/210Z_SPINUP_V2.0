/**
 * @file InitializeModule.cpp
 * @author Zechariah Wang
 * @brief Auton selector, and pre-match initialization
 * @version 0.1
 * @date 2023-02-13
 */

#include "main.h"
#include "map"
#include "pros/misc.h"
#include "string"

constexpr u_int16_t MaxLimit              = 11; // The max limit switches can go up to
constexpr u_int16_t MinLimit              = 0;  // The min limit switches can go up to
u_int16_t SelectedAuton                   = 1;  // Auton choice
u_int16_t AutonFinalized                  = 0;  // 0 = false, 1 = true
u_int16_t globalAuton                     = 1;  // Different auton function depending on selected auton
u_int16_t counterForward1                 = 0;  // Forward counter
u_int16_t counterBackward2                = 0;  // Reverse Counter
u_int16_t simultaneousInputLimit          = 20; // Limit before simultaneous switch is reverted back

static bool pressed1                      = true;  // Status of forward switch
static bool pressed2                      = true;  // Status of backward switch
static bool currentlyPressed1             = false; // Local switch status
static bool currentlyPressed2             = false; // Local switch status

char buffer2[100];
std::map<int, std::string> auton_Legend_secondary = {
    { 1, "Right Side" },
    { 2, "Left Side" },
    { 3, "Solo WP" }
};

/**
 * @brief iterate through the secondary legend
 * 
 */

void iterate_legend(){
    pros::Mutex mutex;
    u_int16_t iterator = 1;
    while (true){
        mutex.take(1000);
        sprintf(buffer2, "Auton %d: %s", iterator, auton_Legend_secondary[iterator].c_str());
        lv_label_set_text(debugLine1, buffer2);
        iterator++;
        if (iterator > 3){ iterator = 1; }
        if (iterator < 1) { iterator = 3; }
        if (AutonFinalized == 1){ break;}
        pros::delay(1000);
        mutex.give();
    }
}

/**
 * @brief Allows for user to select desired auton path in scripts
 * 
 * @param time time before selection phase expires
 */

// This funcion receieves input from lvgl auton selector. Calls the external path connector to run desired auton path.
void Init_AutonSwitchMain::ReceiveInput(u_int32_t time){
    FinalizeAuton data;
    int currentTime = 0;
    u_int16_t iterator = 1;
    u_int16_t legend_counter = 0;
    while (currentTime <= time){
        legend_counter++;
    	data.DisplayData();
        if (AutonFinalized == 1){
        	sprintf(buffer2, "Chosen Auton: %d", SelectedAuton);
	        lv_label_set_text(current_auton_display_selector, buffer2);
		    pros::delay(2000);
        	sprintf(buffer2, "Entering game phase...");
	        lv_label_set_text(current_auton_display_selector, buffer2);
            pros::delay(3000);
            break;
        }
        pros::delay(10);
    }
}

/**
 * @brief Recieve auton selector input but with no time limit (dont ask why theres a time parameter i was being autistic)
 * 
 */

// This function is for the auton selector, however with no time limit on choosing the desired auton. Will only break out once middle button is pressed.
void Init_AutonSwitchMain::ReceiveInput_noLimit(int32_t time){
    FinalizeAuton data;
    u_int16_t currentTime = 0;
    while (currentTime <= time){
    	data.DisplayData();
        if (AutonSwitchForward.get_new_press()){
            SelectedAuton += 1;
            if (SelectedAuton >= MaxLimit){ SelectedAuton = 0; }
            else if (SelectedAuton <= MinLimit){ SelectedAuton = 10; }
        }
        else if (AutonSwitchBackward.get_new_press()){
            SelectedAuton -= 1;
            if (SelectedAuton >= MaxLimit){ SelectedAuton = 0; }
            else if (SelectedAuton <= MinLimit){ SelectedAuton = 10; }
        }
        else if (AutonSwitchBackward.get_new_press() && AutonSwitchForward.get_new_press()){ break; }
        if (AutonFinalized == 1){ pros::delay(2000); break; }
        currentTime += 10;
        pros::delay(10);
    }
}

/**
 * @brief Select auton script
 * 
 */

void FinalizeAuton::SelectAuton(){
    int16_t chosenAuton = SelectedAuton;
    switch (chosenAuton) {
    case 0:  globalAuton = 0;  AutonSelectorPrimary(0);       break;
    case 1:  globalAuton = 1;  AutonSelectorPrimary(1);       break;
    case 2:  globalAuton = 2;  AutonSelectorPrimary(2);       break;
    case 3:  globalAuton = 3;  AutonSelectorPrimary(3);       break;
    case 4:  globalAuton = 4;  AutonSelectorPrimary(4);       break;
    case 5:  globalAuton = 5;  AutonSelectorPrimary(5);       break;
    case 6:  globalAuton = 6;  AutonSelectorPrimary(6);       break;
    case 7:  globalAuton = 7;  AutonSelectorPrimary(7);       break;
    case 8:  globalAuton = 8;  AutonSelectorPrimary(8);       break;
    case 9:  globalAuton = 9;  AutonSelectorPrimary(9);       break;
    case 10: globalAuton = 10; AutonSelectorPrimary(10);      break;
    default: globalAuton = 0;  AutonSelectorPrimary(0);       break;
    }
}

/**
 * @brief resets all vital sensors to default values
 * 
 */

void ResetSensors::ResetAllPrimarySensors(){
    imu_sensor.tare_rotation();
    RotationSensor.reset_position();
    DriveFrontLeft.set_zero_position(0);
    DriveFrontRight.set_zero_position(0);
    DriveBackLeft.set_zero_position(0);
    DriveBackRight.set_zero_position(0);
    gx = 0; gy = 0;
    Launcher.set_value(true);
}

/**
 * @brief Display robot metrics such as position, motor status, etc
 * 
 */

// Display chosen auton
void FinalizeAuton::DisplayCurrentAuton(){
	sprintf(buffer2, SYMBOL_LIST " Selected Path %d: %s", SelectedAuton, auton_Legend_secondary[SelectedAuton].c_str());
    lv_label_set_text(debugLine1, buffer2);
}

// Display metrics/robot data
void FinalizeAuton::DisplayData(){
	char buffer[300];
	sprintf(buffer, SYMBOL_GPS " X: %.2f Y: %.2f Theta: %f", gx, gy, ImuMon());
	lv_label_set_text(odom_readings_sensor, buffer);
    
	sprintf(buffer, SYMBOL_WARNING " FL: %.2f BL: %.2f", DriveFrontLeft.get_temperature(), DriveBackLeft.get_temperature());
	lv_label_set_text(dt_readings_sensor, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Ultrasonic Value: %d", SelectedAuton);
	lv_label_set_text(ultrasonic_readings_sensor, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Cata Limit Value: %d", SelectedAuton);
	lv_label_set_text(cata_readings_sensor, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Intake Readings: %d", SelectedAuton);
	lv_label_set_text(intake_readings_sensor, buffer);
}

void FinalizeAuton::output_sensor_data(){
    char buffer[300];
	sprintf(buffer, SYMBOL_GPS " X: %.2f Y: %.2f Theta: %f", gx, gy, ImuMon());
	lv_label_set_text(odom_readings_sensor, buffer);
    
	sprintf(buffer, SYMBOL_WARNING " FL: %.2f BL: %.2f", DriveFrontLeft.get_temperature(), DriveBackLeft.get_temperature());
	lv_label_set_text(dt_readings_sensor, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Ultrasonic Value: %d", SelectedAuton);
	lv_label_set_text(ultrasonic_readings_sensor, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Cata Limit Value: %d", SelectedAuton);
	lv_label_set_text(cata_readings_sensor, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Intake Readings: %d", SelectedAuton);
	lv_label_set_text(intake_readings_sensor, buffer);
}

void FinalizeAuton::output_auton_selector(){
    char buffer[300];
	sprintf(buffer, SYMBOL_DRIVE " Current Autonomous Route: %d", SelectedAuton);
	lv_label_set_text(current_auton_display_selector, buffer);
}

void FinalizeAuton::output_game_data(){
    char buffer[300];
	sprintf(buffer, SYMBOL_GPS " Controller Status %d", controller.is_connected());
	lv_label_set_text(controller_status_game, buffer);
    
	sprintf(buffer, SYMBOL_WARNING " Battery Capacity: %d", controller.get_battery_level());
	lv_label_set_text(battery_percent_game, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Battery Temperature: %d", controller.get_battery_capacity());
	lv_label_set_text(battery_temp_game, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Time since startup: %d", pros::millis());
	lv_label_set_text(time_since_startup_game, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Competition Status: %d", pros::c::competition_get_status());
	lv_label_set_text(competition_stat_game, buffer);
}

void FinalizeAuton::output_misc_data(){
    char buffer[300];
	sprintf(buffer, SYMBOL_GPS " Debug Line 1 %f", gx);
	lv_label_set_text(debug_line1_misc, buffer);
    
	sprintf(buffer, SYMBOL_WARNING " Debug Line 2 %f", gx);
	lv_label_set_text(debug_line2_misc, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Debug Line 3 %f", gx);
	lv_label_set_text(debug_line3_misc, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Debug Line 4: %f", gx);
	lv_label_set_text(debug_line4_misc, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Debug Line 5: %f", gx);
	lv_label_set_text(debug_line5_misc, buffer);

	sprintf(buffer, SYMBOL_DRIVE " Debug Line 6: %f", gx);
	lv_label_set_text(debug_line6_misc, buffer);
}