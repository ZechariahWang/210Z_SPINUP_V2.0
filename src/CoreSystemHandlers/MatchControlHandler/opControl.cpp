#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"

match_mov mov; // Op Control class init
MotionAlgorithms t;
match_mov::match_mov(){ mov.p_set = 1; mov.it_ps = 1; } // Class Constructor

static bool expansionSet           = true;  // Expansion value
static bool PHASE_ONE              = false; // Expansion failsafe setback 1
static bool PHASE_TWO              = false; // Expansion failsafe setback 2
static bool maxPowerEnabled        = true;  // Max Power Setting
static bool maxIntakePowerEnabled  = true;  // Max Intake Setting

u_int16_t expansionCounter         = 0; // Expansion power
u_int16_t speed_bang               = 127; // Flywheel acceleration

// The og code, standard h-drive control
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

void match_mov::on_off_controller(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        if (abs(OuterShooter.get_voltage()) > ((127 * (12000.0 / 127)))){
            OuterShooter.move_voltage((-127 * (12000.0 / 127)));
        }
        else if (abs(OuterShooter.get_voltage()) < (127 * (12000.0 / 127))){
            OuterShooter.move_voltage((-127 * (12000.0 / 127)));
        }
    }
    else{ OuterShooter.move_voltage(0); }
    std::cout << OuterShooter.get_voltage() << std::endl;
}

void match_mov::power_shooter(){ // Power shooter function
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        OuterShooter.move_voltage(-12000 * mov.p_set);
        InnerShooter.move_voltage(-12000 * mov.p_set);
    }
    else{ OuterShooter.move_voltage(0); InnerShooter.move_voltage(0);}
    std::cout << OuterShooter.get_voltage() << std::endl;
}

void match_mov::power_intake(){ // Power intake function
    if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))){
        DiskIntakeTop.move_voltage(12000 * mov.it_ps);
        DiskIntakeBot.move_voltage(12000);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        DiskIntakeTop.move_voltage(-12000 * mov.it_ps);
        DiskIntakeBot.move_voltage(-12000);
    }
    else{ DiskIntakeTop.move_voltage(0); DiskIntakeBot.move_voltage(0); }
}

void match_mov::launch_disk(){ // Launch disk/piston control function
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
        mov.l_stat = !mov.l_stat;
        Launcher.set_value(mov.l_stat);
    }
    if (mov.l_stat == false) mov.launch_iterator++;
    if (mov.launch_iterator > 10){
        mov.l_stat = !mov.l_stat;
        Launcher.set_value(mov.l_stat); 
        mov.launch_iterator = 0;
    }
}

void match_mov::set_power_amount(){ // Function for changing power of flywheel
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        // t.TurnToPoint(50, 0);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
        maxPowerEnabled = !maxPowerEnabled;
        if (maxPowerEnabled) mov.p_set = 0.6;
        else mov.p_set = 0.6;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        mov.p_set += 0.05;
        if (mov.p_set > 1) mov.p_set = 0;
        else if (mov.p_set < 0) mov.p_set = 1;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        mov.p_set -= 0.05;
        if (mov.p_set> 1) mov.p_set = 0;
        else if (mov.p_set < 0) mov.p_set = 1;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        maxIntakePowerEnabled = !maxIntakePowerEnabled;
        if (maxIntakePowerEnabled) mov.it_ps = 1;
        else mov.it_ps = 0.6;
    }
    controller.print(1, 0, "FW: %.2f SD: %f", mov.p_set, mov.it_ps);
}

void match_mov::set_motor_type(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        mov.robotBrakeType = !mov.robotBrakeType;
    }
    if (mov.robotBrakeType){
        DriveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveMidLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        DriveMidRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        LeftBrake.set_value(true);
        RightBrake.set_value(true);
    }
    else {
        DriveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveMidLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DriveMidRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        LeftBrake.set_value(false);
        RightBrake.set_value(false);
    }
}

void match_mov::init_expansion(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){ PHASE_ONE = true; }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){ PHASE_TWO = true; }
    if (PHASE_ONE && PHASE_TWO) Expansion.set_value(false);
}

void ForceReset(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){ gx = 0; gy = 0; }
}





