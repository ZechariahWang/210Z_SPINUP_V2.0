#include "main.h"

// Test path for Pure Pursuit Algorithm
void PurePursuitTestPath(){
    MotionAlgorithms curveHandler;
    FinalizeAuton data;
    odom odometry;
    std::vector<CurvePoint> Path;

    const double finalX = -40;
    const double finalY = -40;
    CurvePoint StartPos(gx, gy, 0, 0, 10, 5, 1);
    CurvePoint newPoint1(-20, -20, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(-40, -40, 0, 0, 10, 5, 1);
    CurvePoint newPoint3(-60, -60, 0, 0, 10, 5, 1);
    CurvePoint newPoint4(-80, 120, 0, 0, 10, 5, 1);
    CurvePoint EndPos(finalX, finalY, 0, 0, 10, 5, 1);
    Path.push_back(StartPos);
    Path.push_back(newPoint1); 
    Path.push_back(newPoint2);
    // Path.push_back(newPoint3);
    // Path.push_back(newPoint4);

    while (true){
      data.DisplayData();
      odometry.Odometry();
      if (fabs(sqrt(pow(finalX - gx, 2) + pow(finalY - gy, 2))) <= 50){
        // curveHandler.set_constants(7, 5, 2.5, 3);
        // curveHandler.move_to_reference_pose(finalX, finalY, 0, 5);
        // break;
        utility::stop();
        break;
      }
      FollowCurve(Path, 0);
      pros::delay(10);
    }
}

void PurePursuit2(){
    MotionAlgorithms curveHandler;
    FinalizeAuton data;
    odom odometry;
    std::vector<CurvePoint> Path;

    const double finalX = 80;
    const double finalY = 120;
    CurvePoint StartPos(gx, gy, 0, 0, 10, 5, 1);
    CurvePoint newPoint1(50, 0, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(50, 25, 0, 0, 10, 5, 1);
    CurvePoint newPoint3(80, 25, 0, 0, 10, 5, 1);
    CurvePoint newPoint4(80, 120, 0, 0, 10, 5, 1);
    CurvePoint EndPos(finalX, finalY, 0, 0, 10, 5, 1);
    Path.push_back(StartPos);
    Path.push_back(newPoint1); 
    Path.push_back(newPoint2);
    Path.push_back(newPoint3);
    Path.push_back(newPoint4);

    while (true){
      data.DisplayData();
      odometry.Odometry();
      if (fabs(sqrt(pow(finalX - gx, 2) + pow(finalY - gy, 2))) <= 50){
        // curveHandler.set_constants(7, 5, 2.5, 3);
        // curveHandler.move_to_reference_pose(finalX, finalY, 0, 5);
        // break;
        utility::stop();
        break;
      }
      FollowCurve(Path, 0);
    }
}