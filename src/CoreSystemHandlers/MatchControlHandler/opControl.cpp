/**
 * @file opControl.cpp
 * @author Zechariah Wang
 * @brief OP Control logic (dt control, external mechanics, etc)
 * @version 0.1
 * @date 2023-02-13
 * 
 */

#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "cmath"
#include "fstream"

match_mov         mov; 
MotionAlgorithms  t;
match_mov::match_mov(){ mov.p_set = 0.8; mov.it_ps = 1; } // Class Constructor

const u_int16_t forwardCurve       = 10;
const u_int16_t turnCurve          = 5;
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

u_int16_t expansionCounter         = 0; // Expansion power
u_int16_t speed_bang               = 127; // Flywheel acceleration
u_int16_t shot_iteration_counter   = 0;

/**
 * @brief Set exponential joystick accelerator curve type. Allows for more control of small movements, while maintaining max speed for large movements
 * 
 * @param red Red curve enabled. If not enabled, will use blue graph
 * @param t Joystick damper. Tune value as desired by driver
 * @return the value of the joystick accelerator
 */

int32_t joystick_accelerator(bool red, int8_t input, const double t){
    int16_t value = 0;
    if (red) { value = (std::exp(-t / 10) + std::exp((std::abs(input) - 100) / 10) * (1 - std::exp(-t / 10))) * input; }
    else { value = std::exp(((std::abs(input) - 100) * t) / 1000) * input; }
    return value;
}

/**
 * @brief Raw H-Drive DT Control
 * 
 */

void match_mov::dt_Control(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1
    int32_t rightYjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    int32_t leftXjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    if(abs(leftYjoystick) < 10) leftYjoystick = 0;
    if(abs(rightYjoystick) < 10) rightYjoystick = 0;

    int32_t left = (rightXjoystick + leftYjoystick) * (12000.0 / 127);
    int32_t right = (leftYjoystick - rightXjoystick) * (12000.0 / 127);
    utility::leftvoltagereq(left);
    utility::rightvoltagereq(right);
}

/**
 * @brief Exponential Joystick curve accelerator
 * 
 */

void match_mov::exponential_curve_accelerator(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1
    int32_t rightYjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    int32_t leftXjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    if(abs(leftYjoystick) < 10) leftYjoystick = 0;
    if(abs(rightYjoystick) < 10) rightYjoystick = 0;

    double turn_val = joystick_accelerator(turningRed, rightXjoystick, turnCurve);
    double forward_val = joystick_accelerator(forwardRed, leftYjoystick, forwardCurve);
    double turnVoltage = turn_val * (12000.0 / 127); 
    double forwardVoltage = forward_val * (12000.0 / 127);
    double left =  forwardVoltage + turnVoltage;
    double right = forwardVoltage - turnVoltage;
    utility::leftvoltagereq(left);
    utility::rightvoltagereq(right);
}

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
    //std::ofstream fw_writer("flywheelData.txt");
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        std::cout << OuterShooter.get_voltage() << std::endl;
        //fw_writer << "Flywheel speed: " << OuterShooter.get_voltage() << std::endl;
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

/**
 * @brief Raw intake control
 * 
 */

void match_mov::power_intake(){ // Power intake function
    if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))){ DiskIntakeTop.move_voltage(12000); }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ DiskIntakeTop.move_voltage(-12000); }
    else if (anglerStatus){ DiskIntakeTop.move_voltage(-12000); }
    else{ DiskIntakeTop.move_voltage(0); }
}

/**
 * @brief Disk launcher control
 * 
 */

void match_mov::launch_disk(){ // Launch disk/piston control function
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){ 
        if (mov.l_stat == false) { anglerStatus = true; }
     }
    if (anglerStatus) { mov.launch_iterator++; shot_iteration_counter++; }
    if (shot_iteration_counter > 5) {
        DiskIntakeTop.move_voltage(-12000); 
        mov.l_stat = true;
        if (mov.launch_iterator > 150){
            DiskIntakeTop.move_voltage(0); 
            Angler.set_value(mov.l_stat); 
            mov.launch_iterator = 0;
            shot_iteration_counter = 0;
            anglerStatus = false;
        }
    }
}

/**
 * @brief Change power amount of flywheel
 * 
 */

void match_mov::set_power_amount(){ // Function for changing power of flywheel
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        mov.p_set += 0.05;
        if (mov.p_set > 1) mov.p_set = 0;
        else if (mov.p_set < 0) mov.p_set = 1;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        mov.p_set -= 0.05;
        if (mov.p_set> 1) mov.p_set = 0;
        else if (mov.p_set < 0) mov.p_set = 1;
    }
    controller.print(1, 0, "FW: %.2f SD: %f", mov.p_set, mov.it_ps);
}

/**
 * @brief All other misc controls
 * 
 */

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
}

/**
 * @brief Change drivetrain motor type
 * 
 */

void match_mov::set_motor_type(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){ mov.robotBrakeType = !mov.robotBrakeType; }
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

/**
 * @brief Force rest coordinate position of robot
 * 
 */

void ForceReset(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){ gx = 0; gy = 0; }
}






