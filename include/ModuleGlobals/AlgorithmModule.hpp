#include "main.h"


class MotionAlgorithms{
    private:
        bool init_MotionAlg;
    public:
        double target_tol = 10;
        double target_final_tol = 5;
        double t_kp = 10;
        double r_kp = 5;
        double iterator = 0;

        double distance;
        double alpha;
        double beta;
        double t_error;
        double r_error;

        double a_kp;
        double a_ki;
        double a_kd;
        double a_error;
        double a_prev_error;
        double a_integral;
        double a_derivative;
        double a_error_thresh = 3;
        double a_iterator;
        double a_tol = 10;
        double a_failsafe;
        double a_maxSpeed;
        bool a_rightTurn;

        void set_constants(double t_kp, double r_kp, double f_tt, double t);
        void reset_mtp_constants();
        void reset_swing_alterables();
        void TurnToPoint(int targetX, int targetY);
        void move_to_reference_pose(double tx, double ty, double targetHeading, double radius);
        void swing_to_point(double tx, double ty, double swingDamper);
        void overRideCoordinatePos(double new_gx, double new_gy);
        void simultaneous_mov_executor(double targetX, double targetY, double targetTheta, double translationSpeed, double rotationSpeed);
};

class AngleWrap_C{
    private:
        bool init;
    public:
        int AngleWrap(double angle);
};

class ArcMovement{
    private:
        bool init;
    public:
        void _CurveToPoint(double targetX, double targetY);
};

class Point{
    double xCoord;
    double yCoord;
    public:
        void setX(double x) {xCoord = x;}
        void setY(double y) {yCoord = y;}
        double getX(){
            return xCoord;
        }
        double getY(){
            return yCoord;
        }
};

class CurvePoint{
    private:
        bool init;
    public:

        double x;
        double y;
        double moveSpeed;
        double turnSpeed;
        double followDistance;
        double slowDownTurnRadians;
        double slowDownTurnAmount;
        
        CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount);
        CurvePoint(const CurvePoint &thisPoint);

        Point toPoint();
        void setPoint(Point point);
        double getFollowDistance();
        double getX();
        double getY();

};

void FollowCurve(std::vector<CurvePoint> allPoints, double followAngle);