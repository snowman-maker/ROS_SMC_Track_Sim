#include "TD.h"
#include "cmath"
#include "iostream"

using namespace std;

// Optimal synthesis function
double fhan(double x1, double x2, double r0, double h0)
{
    double d  = r0 * h0;
    double d0 = h0 * d;
    double y  = x1 + h0 * x2;
    double a0 = sqrt(pow(d, 2) + 8 * r0 * abs(y));
    double a, f;

    if (abs(y) > d0)
        a = x2 + (a0 - d) / 2.0 * sign(y);
    else
        a = x2 + y / h0;
    
    if (abs(a) > d)
        f = -r0 * sign(a);
    else
        f = -r0 * a / d;
    
    return f;
}

// contrusctors
// params:
//      - hz: sampling period (s)
//      - rz0: max acceleration speed of tracking (m/s^2)
//      - hz0: filter factor (s). The larger the hz0, the stronger the ability to suppress noise, usually set to be greater than or equal to hz
TD::TD(double hz, double rz0, double hz0):_hz(hz), _rz0(rz0), _hz0(hz)
{
    reset();
}


void TD::reset()
{
    _x1 = 0;
    _x2 = 0;
}


void TD::track(double x)
{
    double x1 = _x1;
    double x2 = _x2;
    _x1 = x1 + _hz * x2;
    _x2 = x2 + _hz * fhan((x1-x), x2, _rz0, _hz0);
    // cout << "_x2: " << _x2 << ", _hz: " << _hz << ", fhan: " << fhan((_x1-x), _x2, _rz0, _hz0) << endl;
}