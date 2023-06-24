#include <iostream>
#include "ANTSMC.h"
#include "TD.h"
#include "FastTD.h"
#include <math.h>

void ANTSMC::initial(double T_, vehicleState car, waypoint waypoint, U U_r)
{
    T = T_;
    // car state
    x_car = car.x;
    y_car = car.y; 
    yaw_car = car.yaw;

    // desired state
    x_d = waypoint.x;
    y_d = waypoint.y; 
    yaw_d = waypoint.yaw;

    // system input
    v_d = U_r.v;    
    kesi_d = U_r.kesi;

    td = new FAST_TD(1, 0.05);

    distance_x_error = 0;
    distance_y_error = 0;
}//初始化

void ANTSMC::param_struct(){}//构造状态方程参数

U ANTSMC::cal_vel()
{   U output;
    param_struct();
    

    Eigen::Vector3d error;
    Eigen::Vector3d error_dot;
    Eigen::Vector3d s_err_dot;
    

    delta_x = x_d - x_car;
    delta_y = y_d - y_car;


    // error of x
    double dist_error_x = delta_x;
    td->track(dist_error_x, distance_x_error);
    distance_x_error = td->get_x1();
    dot_error_x = td->get_x2();
    

    // error of y
    double dist_error_y = delta_y;
    td->track(dist_error_y, distance_y_error);
    distance_y_error = td->get_x1();
    dot_error_y = td->get_x2();
    
    std::cout << "dist_error_x: " << dist_error_x << std::endl;
    std::cout << "dot_error_x: " << dot_error_x << std::endl;
    std::cout << "delta_x: " << delta_x << std::endl;
    std::cout << "dist_error_y: " << dist_error_y << std::endl;
    std::cout << "delta_y: " << delta_y << std::endl;
    std::cout << "dot_error_y: " << dot_error_y << std::endl;

    // error of yaw
    double yaw_error = yaw_d - yaw_car;
    double rad_yaw_error = yaw_error;
    std::cout << "rad_yaw_error: " << rad_yaw_error << std::endl;
    
    // load to Eigen Vector
    error << dist_error_x, dist_error_y, rad_yaw_error;
    error_dot << dot_error_x, dot_error_y, 0.0;
    s_err_dot << sign(dot_error_x), sign(dot_error_y), 0.0;

    // Sliding mode plane
    Eigen::Vector3d s_error;
    Eigen::Vector3d s_error_dot;
    for(auto i=0; i<error.size(); i++){
        s_error[i] = pow((1 + pow(error[i], 2)), 5 / 3.) * atan(error[i]);
        s_error_dot[i] = 1.0 / 2 * pow(abs(error_dot[i]), 5 / 3.) * s_err_dot[i];
    }

    Eigen::Vector3d s = s_error + s_err_dot;
    Eigen::Vector3d k_s = 0.25 * s;

    Eigen::Vector3d sign_s;
    for(auto i=0; i<s.size(); i++){
        sign_s[i] = sign(s[i]);
    }
    if (abs(s[0]) <= 3) sign_s[0] = 1 / 3. * s[0];
    if (abs(s[1]) <= 5) sign_s[1] = 1 / 5. * s[1];

    Eigen::Vector3d eta_s;
    for(auto i=0; i<s.size(); i++){
        eta_s[i] = (f_eta + pow(abs(s[i]), 0.5)) * sign_s[i];
    }

    // system input
    Eigen::Vector3d u;
    for(auto i=0; i<s.size(); i++){
        auto u1 = 1.2 * k_s[i] + 1.2 * eta_s[i];
        auto u2 = 1.2 * s_err_dot[i] * pow(abs(error_dot[i]), 1 / 3.) * (1 + 10 / 3. * error[i] * atan(error[i])) * pow((1 + pow(error[i], 2)), 2 / 3.);
        u[i] = u1 + u2;
    }
    std::cout << "u1, u2, u3 = " << u[0] << ", " << u[1] << ", " << u[2] << std::endl;


    U_e <<  1.0 * (u[0] + u[1]) + v_d, 1.0 * u[2] + kesi_d;

    if (abs(yaw_error) <= 1.0){
        U_e[1] = 0.0;
    }
    
    output.v = U_e[0];
    output.kesi = U_e[1];
    std::cout << "v: " << output.v << ", kesi: " << output.kesi << std::endl;

    return output;
}

void ANTSMC::test(){}
