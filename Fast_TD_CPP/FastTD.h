#ifndef FASTTD_H_
#define FASTTD_H_

template <typename T> int sign(T x){return (T(0)<x) - (x<T(0));}
double fast_fhan(double x1, double x2, double r, double h);
double fsg(double x, double d);

class FAST_TD
{
private:
    double _r;      
    double _h;     
    double _x1, _x2;  
public:
    FAST_TD(double _r, double _h);
    void track(double x, double x_last);
    void reset();
    double get_x1(){return _x1;};
    double get_x2(){return _x2;};
};

#endif /* FASTTD_H_ */