# jaka_minicobo

## 安装依赖

`sudo apt install ros-noetic-moveit-ros-visualization`

`sudo apt install ros-noetic-joint-trajectory-controller`

`pip install pyrealsense2`

## build

`mkdir jaka_ws`

`cd ./jaka_ws/`

`git clone https://github.com/Mousikar/jaka_minicobo.git`

`mv jaka_minicobo/ src/`

`catkin_make`

`cd ./jaka_ws/src/jaka_sim_env/scripts`

`chmod +x *.py`

`cd ～`

`gedit .bashrc`

最后一行添加：

`source ~/jaka_ws/devel/setup.bash`

## 运行

**打开gezobo和rviz仿真环境：**

`roslaunch jaka_minicobo_moveit_config demo_mini_cam.launch`

**深度相机扫描点云并发布点云话题：**

`rosrun jaka_sim_env robot_scan.py`

**订阅点云规划路径：**

`rosrun jaka_sim_env pathplan_sim.py`

**rviz中：add --> By topic --> /workpiece_pointcloud的PointCloud2 --> OK**


**add --> By topic --> /pathplanning的Path --> OK**

**机器人仿真：**

`rosrun jaka_sim_env robmov.py`

## 总体代码
robot_scan.py: 实现从读取点云数据，发布坐标转换关系

pathplan_sim.py: 轨迹规划部分，并发布路径话题

robmov.py: 仿真环境中实现机器人沿订阅到的轨迹扫描
