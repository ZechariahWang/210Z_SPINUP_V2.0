/**
 * @file AlgorithmModule.cpp
 * @author Zechariah Wang
 * @brief Helper functions for algorithms found within Algorithms.cpp and PurePursuit.cpp
 * @version 0.1
 * @date 2023-02-13
 */

#include "main.h"
const double SpeedCompensator = 0.3; // Adjusts speed 

/**
 * @brief Assign curve point values
 * 
 * @param x
 * @param y
 * @param moveSpeed
 * @param turnSpeed
 * @param followDistance
 * @param slowDownTurnRadians
 * @param slowDownTurnAmount
 */
 
CurvePoint::CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount){
    this->x = x;
    this->y = y;
    this->moveSpeed = moveSpeed;
    this->turnSpeed = turnSpeed;
    this->followDistance = followDistance;
    this->slowDownTurnRadians = slowDownTurnRadians;
    this->slowDownTurnAmount = slowDownTurnAmount;
}

/**
 * @brief Assign curve point values to class
 * @param thisPoint current point
 */

CurvePoint::CurvePoint(const CurvePoint &thisPoint){ // Assigns values to the class
    x = thisPoint.x;
    y = thisPoint.y;
    moveSpeed = thisPoint.moveSpeed;
    turnSpeed = thisPoint.turnSpeed;
    followDistance = thisPoint.followDistance;
    slowDownTurnRadians = thisPoint.slowDownTurnRadians;
    slowDownTurnAmount = thisPoint.slowDownTurnAmount;
}

/**
 * @brief Creates a new point, and sets values to the current x and y value
 * @return the new point
 */

Point CurvePoint::toPoint(){ // Sets a new point to the current x and y val
    Point newPoint;
    newPoint.setX(x);
    newPoint.setY(y);
    return newPoint;
}

/**
 * @brief Wraps angle to 2 PI, or 360 degrees
 * @param angle Angle to be wrapped
 * @return the angle to be returned
 */

int AngleWrap_C::AngleWrap(double angle){ // Wrap angle to 2 PI
    while (angle < -M_PI){ angle += 2 * M_PI; }
    while (angle > M_PI){ angle -= 2 * M_PI; }
    return angle;
}

/**
 * @brief Takes in two points, and finds the intersection status between two points. From this data, it will determine the most optimal path to follow
 * @param circleCenter the center of the circle around the robot
 * @param radius aka the LOOK AHEAD DISTANCE (L). This value can be tuned depending on the robot
 * @param linePoint1 the coordinate vector of the first point
 * @param linePoint2 the coordinate vector of the second point
 * @return all points points found in intersection
 */

// Function takes in 2 points, and checks the intersection status between both points
std::vector<Point> LineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){
    if (fabs(linePoint1.getY() - linePoint2.getY()) < 0.003){ linePoint1.setY(linePoint2.getY() + 0.003); }
    if (fabs(linePoint1.getX() - linePoint2.getX()) < 0.003){ linePoint1.setX(linePoint2.getX() + 0.003); }

    double m1 = (linePoint2.getY() - linePoint1.getY()) / (linePoint2.getX() - linePoint1.getX());
    double b = (linePoint1.getY()) - m1 * (linePoint1.getX());
    double x1 = linePoint1.getX() - circleCenter.getX();
    double y1 = linePoint1.getY() - circleCenter.getY();
    double quadraticA = 1.0 + pow(m1, 2);
    double quadraticB = (2 * m1 * y1) - (2 * pow(m1, 2) * x1);
    double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2 * y1 * m1 * x1) + pow(y1, 2) - pow(radius, 2);

    quadraticB = (-2 * gx) + (2.0 * m1 * b) - (2 * gy * m1);
    quadraticC = pow(gx, 2) + pow(b, 2) - (2 * b * gy) + pow(gy, 2) - pow(radius, 2);

    std::vector<Point> allPoints; double minX; double maxX;
    if (linePoint1.getX() < linePoint2.getX()){ minX = linePoint1.getX(); maxX = linePoint2.getX(); }
    else{ maxX = linePoint1.getX(); minX = linePoint2.getX(); }
    try
    {
        // Solution 1
        double xRoot1 = (-quadraticB + sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        double yRoot1 = m1 * (xRoot1) + b;
        if (xRoot1 > minX && xRoot1 < maxX){ 
            Point newPoint;
            newPoint.setX(xRoot1);
            newPoint.setY(yRoot1);
            allPoints.push_back(newPoint);
        }
        else{} // No Points
        // Solution 2
        double xRoot2 = (-quadraticB - sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
        double yRoot2 = m1 * (xRoot2) + b;
        if (xRoot2 > minX && xRoot2 < maxX){ 
            Point newPoint;
            newPoint.setX(xRoot2);
            newPoint.setY(yRoot2);
            allPoints.push_back(newPoint);
        }
        else{} // No points 
    }
    catch (std::exception e){} // No intersection, time to throw exception
    return allPoints;
}

/**
 * @brief Class getters and setters. 
 */

void CurvePoint::setPoint(Point point){ x = point.getX(); y = point.getY(); }
double CurvePoint::getFollowDistance(){ return followDistance; }
double CurvePoint::getX(){ return x; }
double CurvePoint::getY(){ return y; }

/**
 * @brief Gets the path, and decides which is the most optimal point to follow
 * @param pathPoints the vector of all points in the path
 * @param robotLocation the position of the robot
 * @param followRadius the look ahead distance to be used
 * @return the point to follow
 */

CurvePoint getFollowPointPath(std::vector<CurvePoint> pathPoints, Point robotLocation, double followRadius){
    CurvePoint followMe(pathPoints.at(0));
    std::vector<Point> intersections;
    std::vector<Point> intersections2;
    for (int i = 0; i < pathPoints.size() - 1; i++){ 
        CurvePoint startLine = pathPoints.at(i);
        CurvePoint endLine = pathPoints.at(i + 1);
        intersections = LineCircleIntersection(robotLocation, pathPoints.at(i).getFollowDistance(), startLine.toPoint(), endLine.toPoint());
        if (intersections.size() == 1){ followMe.setPoint(intersections.at(0)); }
        else if (intersections.size() == 2){
            Point one = intersections.at(0);
            Point two = intersections.at(1);
            double distanceOne = sqrtf(pow((endLine.getX() - one.getX()), 2) + pow((endLine.getY() - one.getY()), 2));
            double distanceTwo = sqrtf(pow((endLine.getX() - two.getX()), 2) + pow((endLine.getY() - two.getY()), 2));
            if (distanceOne < distanceTwo){ followMe.setPoint(one); }
            else{ followMe.setPoint(two); }
        }
    }
    return followMe;
}

/**
 * @brief Driver function. The logic that calls other components of Pure Pursuit
 * @param allPoints All key points in the path
 * @param followAngle The angle to follow the path in. Only for mecanum
 */

void FollowCurve(std::vector<CurvePoint> allPoints, double followAngle){
    Point robotPosition;
    MotionAlgorithms CurveHandler;
    robotPosition.setX(gx);
    robotPosition.setY(gy);
    CurvePoint followMe = getFollowPointPath(allPoints, robotPosition, allPoints.at(0).getFollowDistance());
    CurveHandler.swing_to_point(followMe.getX(), followMe.getY(), 0.6);
    //curver._CurveToPoint(followMe.getX(), followMe.getY()); // Go to point
    //ArcMovement(followMe.getX(), followMe.getY());
}


