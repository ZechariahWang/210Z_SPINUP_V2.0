#include "main.h"

extern unsigned short int globalAuton; // Global auton choice
extern unsigned short int AutonFinalized; // Final auton choice
extern unsigned short int SelectedAuton; // Auton choice
extern const unsigned short int MaxLimit; // The max limit switches can go up to
extern const unsigned short int MinLimit; // The min limit switches can go up to
void iterate_legend();

class Init_AutonSwitchMain{
    private:
        bool init;
    public:
        void ReceiveInput(u_int32_t time);
        void ReceiveInput_noLimit(int32_t time);
};

class ResetSensors : public Init_AutonSwitchMain{
    private:
        bool init;
    public:
        void ResetAllPrimarySensors();
};

class FinalizeAuton : public ResetSensors{
    private:
        bool init;
    public:
        void SelectAuton();
        void DisplayData();
        void DisplayCurrentAuton();
};