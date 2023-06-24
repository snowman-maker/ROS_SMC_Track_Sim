#include <iostream>
#include <Eigen/Dense>
#include "Tool.h"
#include "TD.h"
#include "FastTD.h"
#ifndef ANTSMC_H_
#define ANTSMC_H_

class ANTSMC{
private:
	double T;//采样间隔
	double x_car, y_car, yaw_car, x_d, y_d, yaw_d;//车辆位姿和目标点位姿
	double v_d, kesi_d;
	int temp = 0;
	double delta_x, delta_y;
	double distance_x_error, dot_error_x, distance_y_error, dot_error_y;
	double f_eta = 0.5;
	FAST_TD* td;

	Eigen::Vector2d U_e; // control value
public:
	void initial(double T_, vehicleState car, waypoint waypoint, U U_r);//初始化
	void param_struct();//构造状态方程参数
	U cal_vel();//LQR控制器计算速度
	void test();
};

#endif /* ANTSMC_H_ */