#include "main.h"

extern double gx;
extern double gy;

double ImuMon();

class odom{
    private:
        bool init = true;
    public:
        double deltaArcLength                              = 0;
        double previousArcLength                           = 0;
        double currentarclength                            = 0;

        double d_currentForward                            = 0;
        double d_currentCenter                             = 0;
        double d_currentOtheta                             = 0;
        double d_rotationTheta                             = 0;

        double d_deltaForward                              = 0;
        double d_deltaCenter                               = 0;
        double d_deltaTheta                                = 0;
        double d_deltaOTheta                               = 0;

        double d_deltaTheory                               = 0;
        double d_deltaTheory2                              = 0;
        double d_Theory                                    = 0;
        double d_Theory2                                   = 0;
        double d_totalRotationTheta                        = 0;

        double d_deltaX                                    = 0;
        double d_deltaY                                    = 0;

        double d_previousForward                           = 0;
        double d_previousCenter                            = 0;
        double d_previousOTheta                            = 0;
        double d_previoustheta                             = 0;

        double counter                                     = 0;

        void Odometry();
};

class new_odom{
    private:
        bool init = true;
    public:
        double prev_vertical = 0;
        double prev_horizontal = 0;
        double prev_heading_value;

        double vertical_value;
        double horizontal_value;
        double heading_value;

        double vertical_distance = 0;
        double horizontal_distance = 0;

        void set_pt_constants(const double vertical_wheel_distance, const double horizontal_wheel_distance);
        void compute_odometry();
};