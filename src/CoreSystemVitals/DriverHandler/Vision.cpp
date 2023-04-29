/**
 * @file Vision.cpp
 * @author Zechariah Wang
 * @brief NO IMPLEMENTATION
 * @version 0.1
 * @date 2023-02-13
 * 
 */
 
#include "main.h"

// vision control concept. No actual implementation yet
void vision_main_red(){
    constexpr int maxlimit      = 150;
    constexpr int minlimit      = -150;
    constexpr int lockrange     = 80;
    constexpr int defx          = 33; // || 37;
    constexpr int defy          = 31488; // || 18688;
    constexpr int redSIGint     = 1;

    int prevX                   = 0;
    int prevY                   = 0;
    
    pros::vision_signature_s_t RED_SIG =
    pros::Vision::signature_from_utility(redSIGint, 7141, 8465, 7803, -1221, -669, -945, 3.000, 0);

    pros::vision_object_s_t rtn = vision_sensor.get_by_sig(0, redSIGint);
    int xcoord = rtn.x_middle_coord;
    int ycoord = rtn.y_middle_coord; 

    xcoord = 0;
    ycoord = 0;

    if ((xcoord == defx) && (ycoord == defy)) {} // idle
    else if((xcoord < lockrange) && (xcoord > -lockrange)){} // Locked on
    else if ((xcoord > 0) && (xcoord > lockrange) && (xcoord != defx)) {} // Move turret right
    else if((xcoord < 0) && (xcoord < -lockrange) && (xcoord != defx)) {} // Move turret left
    else{}
}