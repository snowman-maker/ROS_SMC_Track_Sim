一：仿真功能使用

	1.将该功能包放在工作空间XXX_ws的src下

	2.修改~XXX_ws/src/lqr_track/CMakeLists.txt中add_library()中的库函数的绝对路径。

	3.catkin_make
	
	4.配置路径点相关的参数(path_planning.launch中)，控制器相关参数(LQR_track.launch中)，控制器参数中
	  注意：请配置Q矩阵<R矩阵，否则会导致计算结果异常！！！

	4.roslaunch lqr_track LQR_track.launch

	5.roslaunch lqr_track path_planning.launch

二：实车运行使用

	1.将该功能包放在工作空间的src下

	2.修改lqr_track/CMakeLists.txt中add_library()中的库函数的绝对路径。

	3.catkin_make

	4.配置路径点相关的参数(path_planning.launch中)，控制器相关参数(LQR_track.launch中)，控制器参数中
	  注意：请配置Q矩阵<R矩阵，否则会导致计算结果异常！！！

	5.打开机器人底盘、雷达，保证能接收到雷达数据
	   可以配置功能包中的launch/turn_on_base_and_ladiar.launch文件来同时启动地盘雷达

	6.开启机器人定位功能，保证有map到base_link(或base_footprint)的tf转换，
	   如果有cartographer的话，可以运行如下命令：roslaunch lqr_track carto_robot_localization.launch

	7.进行机器人定位工作，并前往路径起始点附近，别放在超过路径起始点一米以外，跟踪效果会变差，与控制器参数有关，如Q、P矩阵选取

	8.roslaunch lqr_track LQR_track_world.launch

	9.roslaunch lqr_track path_planning.launch

	6.rviz需要自己进行修正，配置话题接口


