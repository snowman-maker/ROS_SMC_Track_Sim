一：仿真功能使用

	1.将该功能包放在工作空间XXX_ws的src下

	2.修改~XXX_ws/src/antsmc_track/CMakeLists.txt中add_library()中的库函数的绝对路径。

	3.catkin_make
	
	4.配置路径点相关的参数(path_planning.launch中)，控制器相关参数(LQR_track.launch中)，控制器参数中
	  注意：请配置Q矩阵<R矩阵，否则会导致计算结果异常！！！

	4.roslaunch antsmc_track my_track.launch

	5.roslaunch antsmc_track path_planning.launch
