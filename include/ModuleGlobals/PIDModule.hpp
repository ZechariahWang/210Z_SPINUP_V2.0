#include "main.h"

class Slew{
    private:
        bool init;
    public:
        double enabled;
        double error;
        double x_intercept;
        double y_intercept;
        double sign;
        double slope;
        double max_speed;
        double slew_ticks_per_inch;

        std::vector<double> min_power;
        std::vector<double> max_distance;


        void initialize_slew(bool slew_enabled, const double max_speed, const double target_pos, const double current_pos, const double start, bool backwards_enabled, double tpi);
        void set_slew_min_power(std::vector<double> min_power);
        void set_slew_distance(std::vector<double> distance);
        double calculate_slew(const double current);
};

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
        void set_t_constants(const double kp, const double ki, const double kd, const double r_kp);
        void set_dt_constants(const double n_wheelDiameter, const double n_gearRatio, const double n_motorCartridge);
        void set_translation_pid(double target, double maxSpeed);
        void set_translation_pid_with_location_params(double target, double maxSpeed, double distanceVal, double slewEnabled);
        void set_translation_pid_with_sim_reset(double target, double maxSpeed, double slewEnabled);
        double find_min_angle(int16_t targetHeading, int16_t currentrobotHeading);
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
        void set_r_constants(const double kp, const double ki, const double kd);
        double compute_r(double current, double target);
        void set_rotation_pid(double t_theta, double maxSpeed);
        void set_rotation_pid_with_sim_reset(double t_theta, double maxSpeed);
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
        void set_c_constants(const double kp, const double ki, const double kd);
        double compute_c(double current, double target);
        void set_curve_pid(double t_theta, double maxSpeed, double curveDamper, bool backwards);
        void set_curve_pid_with_sim_reset(double t_theta, double maxSpeed, double curveDamper, bool backwards);
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
        void set_a_constants(const double kp, const double ki, const double kd);
        double compute_a(double tx, double ty);
        void set_arc_pid(double t_x, double t_y, double maxSpeed, double arcDamper);
};

class SimultaneousPID{
    private:
        bool init;
    public:
        double t_s_kp;
        double t_s_ki;
        double t_s_kd;
        double t_s_r_kp;
        double t_s_error;
        double t_s_prev_error;
        double t_s_integral;
        double t_s_derivative;
        double t_s_error_thresh = 3;
        double t_s_iterator;
        double t_s_tol = 10;
        double t_s_failsafe;
        double t_s_maxSpeed;

        double c_s_kp;
        double c_s_ki;
        double c_s_kd;
        double c_s_error;
        double c_s_prev_error;
        double c_s_integral;
        double c_s_derivative;
        double c_s_error_thresh = 3;
        double c_s_iterator;
        double c_s_tol = 10;
        double c_s_failsafe;
        double c_s_maxSpeed;
        bool c_s_rightTurn;

        bool curvePhaseStart = true;
        bool translationPhase = false;
        bool curvePhaseEnd = false;

        SimultaneousPID();
        void reset_sim_alterables();
        void set_sim_t_constants(const double kp, const double ki, const double kd, const double r_kp);
        void set_sim_c_constants(const double kp, const double ki, const double kd);
        double compute_sim_mov_pid(double translationTarget, double translationCurrent);
        double compute_sim_cur_pid(double curvetargetTheta, double curveCurrent);
        void set_sim_pid(bool curveStartEnabled, double curvetargetThetaStart, double curveDamperStart, double curveMaxSpeedStart, bool backwardsStart, bool translationEnabled, double translationTarget, double translationMaxSpeed, bool curveEndEnabled, double curvetargetThetaEnd, double curveDamperEnd, double curveMaxSpeedEnd, bool backwardsEnd);
};