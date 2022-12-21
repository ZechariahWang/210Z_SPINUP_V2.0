#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"

match_mov mov;

match_mov::match_mov(){
    mov.p_set = 0.6;
}

// The og code, standard h-drive control
void match_mov::dt_Control(){
    double leftYjoystick  = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    double leftXjoystick  = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    double rightYjoystick = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    double rightXjoystick = (double)(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1
    if(fabs(leftYjoystick) < 10) leftYjoystick = 0;
    if(fabs(rightYjoystick) < 10) rightYjoystick = 0;

    double left = (rightXjoystick + leftYjoystick) * (12000.0 / 127);
    double right = (leftYjoystick - rightXjoystick) * (12000.0 / 127);
    utility::leftvoltagereq(left);
    utility::rightvoltagereq(right);
}

// Power shooter function
void match_mov::power_shooter(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        OuterShooter.move_voltage(12000 * mov.p_set);
        InnerShooter.move_voltage(12000 * mov.p_set);
    }
    else{
        OuterShooter.move_voltage(0);
        InnerShooter.move_voltage(0);
    }
}

// Power intake function
void match_mov::power_intake(){
    if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))){
        DiskIntakeTop.move_voltage(12000 * mov.it_ps);
        DiskIntakeBot.move_voltage(12000);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        DiskIntakeTop.move_voltage(-12000 * mov.it_ps);
        DiskIntakeBot.move_voltage(-12000);
    }
    else{
        DiskIntakeTop.move_voltage(0);
        DiskIntakeBot.move_voltage(0);
    }
}

// Launch disk/piston control function
void match_mov::launch_disk(){
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

// Function for changing power of flywheel
bool maxPowerEnabled = true;
bool maxIntakePowerEnabled = true;
void match_mov::set_power_amount(){
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

bool expansionSet = true;
bool rightstat = false;
bool ystat = false;
int expansionCounter = 0;
void match_mov::init_expansion(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        rightstat = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        ystat = true;
    }
    if (ystat && rightstat) Expansion.set_value(false);
}

void ForceReset(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        gx = 0;
        gy = 0;
    }
}





