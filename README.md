# pcl_livox_channel
livox-360雷达获取管道点云，在线简易拟合管道，ros节点<br />
管道拟合效果： <br />
<img src="https://github.com/HLkyss/pcl_livox_channel/assets/69629475/19c5aaf4-d89e-417e-8253-657ff3ccfabb" width="500"> <br />
<img src="https://github.com/HLkyss/pcl_livox_channel/assets/69629475/3b154340-99c2-4616-b447-dfbb8fe8ff89" width="500"> <br />
彩色为雷达获得的所有点云，白色为过滤后用于拟合的点云：<br />
<img src="https://github.com/HLkyss/pcl_livox_channel/assets/69629475/e0159fc6-35e0-473e-a806-795fb6d1ccb2" width="500"> <br />
动态效果：<br />
<img src="https://github.com/HLkyss/pcl_livox_channel/assets/69629475/018674fc-3f7c-4ccd-b5a6-65b9a35f72be" width="800"> <br />

***
工程描述：熟悉了ros和pcl的东西，用雷达获取管道点云数据，然后用pcl拟合，最后在rviz里可视化。</br>
参考：（更多细节参考，见代码）
[https://blog.csdn.net/qq_42367689/article/details/104358046](https://blog.csdn.net/qq_42367689/article/details/104358046) </br>
[http://wiki.ros.org/pcl/Tutorials/hydro](http://wiki.ros.org/pcl/Tutorials/hydro) </br>

1. 先运行运行ws_Livox下的livox_ros_driver2：
$ source /home/hl/project/Livox_ros_driver2_ws/devel/setup.sh
$ roslaunch livox_ros_driver2 rviz_MID360.launch
（更改网络设置：sudo ifconfig enp49s0 192.168.1.50，在/home/hl/Downloads/ws_Livox/src/livox_ros_driver2/config/MID360comnfig.json中有设置）
（	此时运行
	$ rostopic list #查看话题列表，找到雷达点云信息为/livox/lidar
	再运行
	$ rostopic info /livox/lidar #查看话题数据类型，为sensor_msgs/PointCloud2，据此修改PclProj代码）

2. 运行PclProj:
$ source /home/hl/project/PCL_livox_channel/pcl_ws/devel/setup.sh
$ rosrun PclProj PclProj

3. 在rviz中
在先前rviz_MID360.launch打开的rviz中，添加PointCloud2，话题为output
