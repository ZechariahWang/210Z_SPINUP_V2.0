#include "main.h"

// Test path for Pure Pursuit Algorithm
void PurePursuitTestPath(){
    MotionAlgorithms curveHandler;
    FinalizeAuton data;
    odom odometry;
    std::vector<CurvePoint> Path;

    const double finalX = 10;
    const double finalY = 50;
    CurvePoint StartPos(gx, gy, 0, 0, 10, 5, 1);
    CurvePoint newPoint1(10, 10, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(20, 20, 0, 0, 10, 5, 1);
    CurvePoint EndPos(finalX, finalY, 0, 0, 10, 5, 1);
    Path.push_back(StartPos);
    Path.push_back(newPoint1); 
    Path.push_back(newPoint2);
    Path.push_back(EndPos);

    while (true){
      data.DisplayData();
      odometry.Odometry();
      if (sqrt(pow(finalX - gx, 2) + pow(finalY - gy, 2)) <= 20){
        curveHandler.set_constants(7, 5, 2.5, 3);
        curveHandler.move_to_reference_pose(finalX, finalY, 0, 5);
        break;
      }
      FollowCurve(Path, 0);
    }
}