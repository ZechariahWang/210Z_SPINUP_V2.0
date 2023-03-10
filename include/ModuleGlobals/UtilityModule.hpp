#include "main.h"
// Classes for the main autonomous functions, as well as some algorithms. Designated algorithms are found in the algorithm framework

namespace utility
{
    int sgn(double num);
    void stop();
    void stop_v();
    void leftvreq(int voltage);
    void rightvreq(int voltage);
    void leftvelreq(double velocity);
    void rightvelreq(double velocity);
    void leftvoltagereq(double voltage);
    void rightvoltagereq(double voltage);
    void fullreset(double resetval, bool imu);
    void eclipse_fullreset(double resetval, bool imu);
}
