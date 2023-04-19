/**
 * @file GameAssets.cpp
 * @author Zechariah Wang
 * @brief OP Control logic (dt control, external mechanics, etc)
 * @version 0.1
 * @date 2023-02-13
 * 
 */

//ghg

#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "cmath"
#include "fstream"

match_mov         mov; 
MotionAlgorithms  t;
match_mov::match_mov(){ mov.p_set = 1; mov.it_ps = 1; } // Class Constructor

const u_int16_t forwardCurve       = 10;
const u_int16_t turnCurve          = 3;
const double euler                 = 2.71828;

static bool anglerStatus           = false; // False = set for priority intake
static bool expansionSet           = true;  // Expansion value
static bool PHASE_ONE              = false; // Expansion failsafe setback 1
static bool PHASE_TWO              = false; // Expansion failsafe setback 2
static bool maxPowerEnabled        = true;  // Max Power Setting
static bool maxIntakePowerEnabled  = true;  // Max Intake Setting
static bool arcLaunchToggle        = false; // toggle yao ming
static bool toggleRedCurve         = false; // toggle red curve
static bool turningRed             = false;
static bool forwardRed             = false;

uint16_t cataDelay = 700;
uint16_t c_delay_counter = 0;
uint16_t c_piston_counter = 0;
static bool shot_enabled = false;
uint16_t c_delay_counter_piston = 0;
uint16_t c_piston_counter_piston = 0;
static bool piston_shot_enabled = false;

u_int16_t expansionCounter         = 0; // Expansion power
u_int16_t speed_bang               = 127; // Flywheel acceleration
u_int16_t shot_iteration_counter   = 0;

/**
 * @brief Bang Bang Controller. For flywheel to recover to target speed faster
 * 
 */

void match_mov::on_off_controller(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        if (abs(OuterShooter.get_voltage()) > ((90 * (12000.0 / 127)))){
            OuterShooter.move_voltage((90 * (12000.0 / 127)));
        }
        else if (abs(OuterShooter.get_voltage()) < (90 * (12000.0 / 127))){
            OuterShooter.move_voltage((127 * (12000.0 / 127)));
        }
    }
    else{ OuterShooter.move_voltage(0); }
}

void match_mov::on_off_v2(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        if (abs(OuterShooter.get_voltage()) > (12000 * 0.9)){ OuterShooter.move_voltage(12000 * 0.9); }
        else if (abs(OuterShooter.get_voltage()) < (12000 * 0.9)){ OuterShooter.move_voltage(12000); }
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){ OuterShooter.move_voltage(-12000); }
    else{ OuterShooter.move_voltage(0); }

}

/**
 * @brief Raw flywheel control
 * 
 */

void match_mov::power_shooter(){ // Power shooter function 
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        OuterShooter.move_voltage(12000 * mov.p_set);
        InnerShooter.move_voltage(12000 * mov.p_set);
    }
    else{ OuterShooter.move_voltage(0); InnerShooter.move_voltage(0);}
}

static bool cata_initiated = 1;
static bool cata_need_to_reset_ima_kms = true;
static bool cata_movement_enabled = false;
void match_mov::prime_catapult(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
         cata_initiated = !cata_initiated;
    }
    if (CataLimitMonitor.get_value() == 0 && cata_initiated == 1 && cata_need_to_reset_ima_kms == true){  CataPrimer.move_voltage(12000); }
    else if (CataLimitMonitor.get_value() == 1 && cata_movement_enabled == false) { 
        CataPrimer.move_voltage(0); 
        cata_initiated = 0;
        cata_need_to_reset_ima_kms = false;
    }
}

/**
 * @brief Disk launcher control
 * 
 */

void match_mov::launch_disk(){ // Launch disk/piston control function
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && CataLimitMonitor.get_value() == 1){ 
        shot_enabled = true;
        cata_need_to_reset_ima_kms =    true;
        cata_initiated = 1;
        cata_movement_enabled = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) && CataLimitMonitor.get_value() == 1){ 
        piston_shot_enabled = true;
        cata_need_to_reset_ima_kms = true;
        cata_initiated = 1;
        cata_movement_enabled = true;
    }
    if (shot_enabled) {
        CataPrimer.move_voltage(12000); 
        c_delay_counter++;
        if (c_delay_counter > 70){
            cata_need_to_reset_ima_kms = true;
            cata_initiated = 1;
            c_delay_counter = 0;
            shot_enabled = false;
            cata_movement_enabled = false;
        }
    }
    if (piston_shot_enabled){
        CataPrimer.move_voltage(12000); 
        c_delay_counter_piston++;
        c_piston_counter_piston++;
        if (c_piston_counter_piston > mov.p_set) { pistonBooster.set_value(true); c_piston_counter_piston = 0; }
        if (c_delay_counter_piston > 30){
            cata_need_to_reset_ima_kms = true;
            cata_initiated = 1;
            pistonBooster.set_value(false);
            c_delay_counter_piston = 0;
            piston_shot_enabled = false;
            cata_movement_enabled = false;
        }
    }
}

/**
 * @brief Raw intake control
 * 
 */

void match_mov::power_intake(){ // Power intake function
    if (CataLimitMonitor.get_value() == 0) { DiskIntakeTop.move_voltage(0); return; }
    if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))){ DiskIntakeTop.move_voltage(12000); }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ DiskIntakeTop.move_voltage(-12000); }
    else{ DiskIntakeTop.move_voltage(0); }
}

/**
 * @brief Change power amount of flywheel
 * 
 */

void match_mov::set_power_amount(){ // Function for changing power of flywheel
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        mov.p_set += 1;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        mov.p_set -= 1;
    }
    controller.print(1, 0, "FW: %.2f SD: %f", mov.p_set, mov.it_ps);
}

/**
 * @brief All other misc controls
 * 
 */
bool intakeLifted = false;
void match_mov::misc_control(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        arcLaunchToggle = !arcLaunchToggle;
        YaoMing.set_value(arcLaunchToggle);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        if (anglerStatus) return;
        mov.l_stat = !mov.l_stat;
        Angler.set_value(mov.l_stat);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        intakeLifted = !intakeLifted;
        intakeLift.set_value(intakeLifted);

    }
}

/**
 * @brief Change drivetrain motor type
 * 
 */

void match_mov::set_motor_type(){
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){ mov.robotBrakeType = !mov.robotBrakeType; }
    if (mov.robotBrakeType == false){
        DriveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveMidLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveMidRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LeftBrake.set_value(true);
        RightBrake.set_value(true);
    }
    else {
        DriveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveMidLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveMidRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        LeftBrake.set_value(false);
        RightBrake.set_value(false);
    }
}

/**
 * @brief Expnansion initiation with failsafe
 * 
 */

void match_mov::init_expansion(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){ PHASE_ONE = true; }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){ PHASE_TWO = true; }
    if (PHASE_ONE && PHASE_TWO) Expansion.set_value(false);
}







