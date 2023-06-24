#include "FastTD.h"
#include "cmath"
#include "iostream"

using namespace std;

double fsg(double x, double d)
{
    auto temp = (sign(x + d) - sign(x - d)) / 2.0;
    return temp;
}

// Optimal synthesis function
double fast_fhan(double x1, double x2, double r, double h)
{
    double d  = r * pow(h, 2);
    double a0 = h * x2;
    double y = x1 + a0;
    double a1 = pow((d * (d + 8 * abs(y))), 0.5);
    double a2 = a0 + double(sign(y) * (a1 - d)) / 2.0;
    double a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
    double fh = -r * (a / d) * fsg(a, d) - r * sign(a) * (1 - fsg(a, d));
    return fh;
}

FAST_TD::FAST_TD(double r, double h):_r(r), _h(h)
{
    reset();
}

void FAST_TD::track(double x, double x_last)
{
    double fh = fast_fhan(x_last - x, _x2, _r, 1 * _h);
    double x1 = _x1;
    double x2 = _x2;
    _x1 = x1 + _h * x2;
    _x2 = x2 + _h * fh;
}

void FAST_TD::reset()
{
    _x1 = 0;
    _x2 = 0;
}
