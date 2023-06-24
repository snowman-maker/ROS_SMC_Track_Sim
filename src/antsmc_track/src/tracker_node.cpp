#include <ros/ros.h>
#include <iostream>
#include "ANTSMC.h"
#include <vector>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
using namespace std;

double freq, V_DESIRED;//采样频率，期望速度
bool limit_v_and_kesi;
double initial_x,initial_y,initial_yaw,initial_v,initial_kesi;//初始化车辆位姿
double slow_LEVE1_DISTANCE,slow_LEVE2_DISTANCE,slow_LEVE1_V,slow_LEVE2_V,goal_tolerance_DISTANCE;//定义二级减速距离和速度
#define T 1/freq //采样时间 
 
vehicleState update_state(U control, vehicleState car) {
	car.v = control.v;
	car.kesi = control.kesi;
	car.x += car.v * cos(car.yaw) * T;
	car.y += car.v * sin(car.yaw) * T;
	car.yaw += car.v * tan(car.kesi) * T;
	return car;
}

class Path {
private:
	vector<waypoint> path;
public:
	//添加新的路径点
	void Add_new_point(waypoint& p)
	{
		path.push_back(p);
	}
 
	void Add_new_point(vector<waypoint>& p) 
	{
		path = p;
	}
 
	//路径点个数
	unsigned int Size()
	{
		return path.size();
	}
 
	//获取路径点
	waypoint Get_waypoint(int index)
	{
		waypoint p;
		p.ID = path[index].ID;
		p.x = path[index].x;
		p.y = path[index].y;
		p.yaw = path[index].yaw;
		return p;
	}

	vector<waypoint> Get_waypoints(){
		return path;
	}
 
 
	// 搜索路径点, 将小车到起始点的距离与小车到每一个点的距离对比，找出最近的目标点索引值
	int Find_target_index(vehicleState state)
	{
		double min = abs(sqrt(pow(state.x - path[0].x, 2) + pow(state.y - path[0].y, 2)));
		int index = 0;
		for (int i = 0; i < path.size(); i++)
		{
			double d = abs(sqrt(pow(state.x - path[i].x, 2) + pow(state.y - path[i].y, 2)));
			if (d < min)
			{
				min = d;
				index = i;
			}
		}
 
		//索引到终点前，当（机器人与下一个目标点的距离Lf）小于（当前目标点到下一个目标点距离L)时，索引下一个目标点
		if ((index + 1) < path.size())
		{
			double current_x = path[index].x; double current_y = path[index].y;
			double next_x = path[index + 1].x; double next_y = path[index + 1].y;
			double L_ = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
			double L_1 = abs(sqrt(pow(state.x - next_x, 2) + pow(state.y - next_y, 2)));
			//ROS_INFO("L is %f,Lf is %f",L,Lf);
			if (L_1 < L_)
			{
				index += 1;
			}
		}
		return index;
	}
 
};

class ANTSMC_node {
private:
	//car
	vehicleState car;//小车状态
	U control;//小车控制量[v,kesi]
	int lastIndex;//最后一个点索引值
	waypoint lastPoint;//最后一个点信息
	string action;//小车目前动作：跟踪或跟踪完成(tracking or reach goal!)

	//ROS
	ros::Subscriber path_sub;//订阅路径，消息类型为nav_msgs::Path
	ros::Publisher vel_pub;//发布速度信息，消息类型为geometry_msgs::Twist
	ros::Publisher actual_state_pub;//发布小车实际位姿，消息类型为geometry_msgs::Pose2D
	ros::Publisher visual_state_pub;//向rviz发布小车虚拟轨迹，消息类型为visualization_msgs::Marker
	geometry_msgs::Point visual_state_pose;
	visualization_msgs::Marker visual_state_trajectory;
	geometry_msgs::Pose2D actual_pose;
	geometry_msgs::Twist vel_msg;
	int temp;//计数，达到终点时，用于判断控制器是否需要关闭

 
public:
    ANTSMC* controller;
    Path* path;

	ANTSMC_node(ros::NodeHandle& nh)//初始化中添加轨迹、小车初始位姿
	{
        controller = new ANTSMC();
        path = new Path();
        
		//ROS:
		path_sub = nh.subscribe("path",10,&ANTSMC_node::addpointcallback,this);
		vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
		visual_state_pub = nh.advertise<visualization_msgs::Marker>("visualization_pose",10);
		actual_state_pub = nh.advertise<geometry_msgs::Pose2D>("ANTSMC_pose",10);

		//robot state initialize:
		car.x = initial_x;
		car.y = initial_y;
		car.yaw = initial_yaw;
		car.v = initial_v;
		car.kesi = initial_kesi;
		action = "the car is tracking!!";
	}
 
	~ANTSMC_node() {
        delete(controller);
        delete(path);
	}
 
	void addpointcallback(const nav_msgs::Path::ConstPtr& msg){
		vector<waypoint> waypoints;
		for(int i=0;i<msg->poses.size();i++){
			waypoint waypoint;
			waypoint.ID = msg->poses[i].header.seq;
			waypoint.x = msg->poses[i].pose.position.x;
			waypoint.y = msg->poses[i].pose.position.y;
			//获取角度
			double roll,pitch,yaw;
	    		tf::Quaternion quat;
	    		tf::quaternionMsgToTF(msg->poses[i].pose.orientation, quat);
	    		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			waypoint.yaw = yaw;
			waypoints.push_back(waypoint);
		}
		path->Add_new_point(waypoints);//路径点vector数组传到path类中
		lastIndex = path->Size() - 1;
		lastPoint = path->Get_waypoint(lastIndex);
	}
 
	double slow_judge(double distance) {
		if (distance>=slow_LEVE2_DISTANCE&&distance <= slow_LEVE1_DISTANCE) {
			return slow_LEVE1_V;
		}
		else if (distance>=goal_tolerance_DISTANCE&&distance < slow_LEVE2_DISTANCE) {
			return slow_LEVE2_V;
		}
		else if (distance < goal_tolerance_DISTANCE) {
			action = "the car has reached the goal!";
			return 0.0;
		}
		else
		{
			return V_DESIRED;
		}
	}
 
	//控制器流程
	void ANTSMC_track() {
		U U_r;
		waypoint Point;
		double K; // curvature
		if(path->Size()!=0){
			//搜索路径点
			int target_index = path->Find_target_index(car);

			//获取路径点信息，构造期望控制量
			Point = path->Get_waypoint(target_index);//获取 x,y
			K = cal_K(path->Get_waypoints(),target_index);//计算曲率

			//减速判断
			double kesi = atan2(K, 1);
			double v_distance = abs(sqrt(pow(car.x - lastPoint.x, 2) + pow(car.y - lastPoint.y, 2)));
			//ROS_INFO("the distance is %f\n", v_distance);
			U_r.v = slow_judge(v_distance);
			U_r.kesi = kesi;
			std::cout << "U_r: v= " << U_r.v << ", kesi = " << U_r.kesi << std::endl;
			ROS_INFO("%s",action.c_str());//机器人动作


			//使用ANTSMC控制器
			controller->initial(T, car, Point, U_r);//初始化控制器
			control = controller->cal_vel();//计算输入[v, kesi]
			if(U_r.v==0) control.v = 0;//判断，期望速度为0，则机器人停下
			ROS_INFO("the speed is: %f,the kesi is: %f", control.v, control.kesi);
			ROS_INFO("the car position is x: %f, y: %f", car.x, car.y);

			//话题发布
			PUB();
			
			//小车位姿状态更新
			car = update_state(control, car);

			//控制器关闭判断
			shutdown_controller();

			ROS_INFO("--------------------------------------");
		}
	}
 
	//控制启停函数
	void node_control() {
        ros::Rate loop_rate(freq);
		Marker_set();//设置Marker属性
		//设置tf坐标转换
		tf::TransformBroadcaster br;
		tf::Transform transform;
		tf::Quaternion q;

		while (ros::ok()) {
			transform.setOrigin(tf::Vector3(car.x, car.y, 0));
			q.setRPY(0, 0, car.yaw);
			printf("car.x = %f, car.y = %f, car.yaw = %f \n", car.x, car.y, car.yaw);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "car"));

			ros::spinOnce();
			ANTSMC_track();
			loop_rate.sleep();
		}
	}

	void PUB(){
		visual_state_pose.x = car.x; 
		visual_state_pose.y = car.y;

		actual_pose.x = car.x; 
		actual_pose.y = car.y; 
		actual_pose.theta = car.yaw;

		vel_msg.linear.x = control.v;
		vel_msg.angular.z = control.v * tan(control.kesi);
		visual_state_trajectory.points.push_back(visual_state_pose);//visualization_msgs::Marker为一个容器，所以现需要向里面push_back结构体，再发布
		visual_state_pub.publish(visual_state_trajectory);//发布虚拟轨迹
		vel_pub.publish(vel_msg);//发布速度
		actual_state_pub.publish(actual_pose);//发布位姿
	}

	void shutdown_controller(){
		if(action == "the car has reached the goal!"){
			temp+=1;
			if(temp ==50){
				ROS_WARN("shutdown the controller!");
				temp = 0;
				ros::shutdown();
			}
		}
	}

	void Marker_set(){
		//设置消息类型参数
		visual_state_trajectory.header.frame_id = "map";
		visual_state_trajectory.header.stamp = ros::Time::now();
		visual_state_trajectory.action = visualization_msgs::Marker::ADD;
		visual_state_trajectory.ns = "LQR";
		//设置点的属性
		visual_state_trajectory.id = 0;
		visual_state_trajectory.type = visualization_msgs::Marker::POINTS;
		visual_state_trajectory.scale.x = 0.02;
		visual_state_trajectory.scale.y = 0.02;
		visual_state_trajectory.color.r = 1.0;
		visual_state_trajectory.color.a = 1.0;
	}
};
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "LQR_node");
	ros::NodeHandle n;
	ros::NodeHandle n_prv("~");

	n_prv.param<double>("freq",freq,20);
	n_prv.param<double>("initial_x",initial_x,0.0);
	n_prv.param<double>("initial_y",initial_y,2.0);
	n_prv.param<double>("initial_yaw",initial_yaw,0.6);
	n_prv.param<double>("initial_v",initial_v,0.5);
	n_prv.param<double>("initial_kesi",initial_kesi,0.1);
	n_prv.param<double>("slow_LEVE1_DISTANCE",slow_LEVE1_DISTANCE,5.0);
	n_prv.param<double>("slow_LEVE2_DISTANCE",slow_LEVE2_DISTANCE,2.0);
	n_prv.param<double>("goal_tolerance_DISTANCE",goal_tolerance_DISTANCE,0.1);
	n_prv.param<double>("slow_LEVE1_V",slow_LEVE1_V,0.35);
	n_prv.param<double>("slow_LEVE2_V",slow_LEVE2_V,0.15);

	ANTSMC_node* node = new ANTSMC_node(n);
	node->node_control();
	return (0);
}

 
