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

        //TranslationPID();
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
        double r_error_thresh;
        double r_iterator;
        double r_tol;
        double r_failsafe;
        double r_maxSpeed;
        void reset_r_alterables();
        void set_r_constants(double kp, double ki, double kd);
        double compute_r(double current, double target);
        void set_rotation_pid(double t_theta, double maxSpeed);
};

class CurvePID{

};

class ArcPID{

};