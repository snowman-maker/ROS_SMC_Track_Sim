#ifndef TD_H_
#define TD_H_

template <typename T> int sign(T x){return (T(0)<x) - (x<T(0));}
double fhan(double x1, double x2, double r0, double h0);


// TD class for Tracking Differentiator
// Example 
//   TD diff_tracker = TD(0.1, 10, 0.1);
//   diff_tracker.track(data_to_track);
//   double x1 = diff_tracker.get_x1();
//   double x2 = diff_tracker.get_x2();
class TD
{
private:
    double _hz;      // sampling period
    double _rz0;     // max acceleration speed of tracking
    double _hz0;     // filter factor. The larger the hz0, the stronger the ability to suppress noise
    double _x1, _x2;  // state
public:
    TD(double hz, double rz0, double hz0);
    void track(double x);
    void reset();
    double get_x1(){return _x1;};
    double get_x2(){return _x2;};
};

#endif //TD_H_