#include "main.h"

class TranslationPID{
    private:
        bool init;
    public:
        double t_kp;
        double t_ki;
        double t_kd;
        double t_h_kp;
        double t_error;
        double t_prev_error;
        double t_integral;
        double t_derivative;
        double t_error_thresh;
        double t_iterator;
        double t_tol;
        double t_failsafe;
        double t_maxSpeed;
    
        double wheelDiameter                           = 0;
        double ratio                                   = 0;
        double cartridge                               = 0;
        double circumfrance                            = 0;
        double ticks_per_rev                           = 0;
        double ticks_per_inches                        = 0;

        TranslationPID();
        void reset_t_alterables();
        void set_t_constants(double kp, double ki, double kd, double r_kp);
        void set_dt_constants(double n_wheelDiameter, double n_gearRatio, double n_motorCartridge);
        void set_translation_pid(double target, double maxSpeed);
        double find_min_angle(int targetHeading, int currentrobotHeading);
        double compute_t(double current, double target);
};

class RotationPID{
    private:
        bool init;
    public:
        double r_kp;
        double r_ki;
        double r_kd;
        double r_error;
        double r_prev_error;
        double r_integral;
        double r_derivative;
        double r_error_thresh = 3;
        double r_iterator;
        double r_tol = 10;
        double r_failsafe;
        double r_maxSpeed;

        RotationPID();
        void reset_r_alterables();
        void set_r_constants(double kp, double ki, double kd);
        double compute_r(double current, double target);
        void set_rotation_pid(double t_theta, double maxSpeed);
};

class CurvePID{
    private:
        bool init;
    public:
        double c_kp;
        double c_ki;
        double c_kd;
        double c_error;
        double c_prev_error;
        double c_integral;
        double c_derivative;
        double c_error_thresh = 3;
        double c_iterator;
        double c_tol = 10;
        double c_failsafe;
        double c_maxSpeed;
        bool c_rightTurn;

        CurvePID();
        void reset_c_alterables();
        void set_c_constants(double kp, double ki, double kd);
        double compute_c(double current, double target);
        void set_curve_pid(double t_theta, double maxSpeed, double curveDamper);
};

class ArcPID{
    private:
        bool init;
    public:
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

        ArcPID();
        void reset_a_alterables();
        void set_a_constants(double kp, double ki, double kd);
        double compute_a(double tx, double ty);
        void set_arc_pid(double t_x, double t_y, double maxSpeed, double arcDamper);
};