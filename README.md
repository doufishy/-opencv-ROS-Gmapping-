# -opencv-ROS-Gmapping-
使用opencv实时查看ROS Gmapping建图效果

使用效果，见CSDN网页：https://blog.csdn.net/shuang_yu_/article/details/115343969

环境：
1. Ubuntu16
2. ROS Kinetic
3. Turtlebot3仿真环境，仿真环境的构建见创客 https://www.ncnynl.com/archives/201707/1790.html

4个ROS package介绍：
1. aeplanner 主要是功能实现部分，即接收来自Gmapping的地图和来自pubpose的位姿信息并通过imshow展示出来。
2. catkin_simple 与 kdtree 辅助包。
3. pubpose 监听tf消息，并将小车的位姿发布出来。

构建方法：
1. 上述四个文件都是ROSpackage，放到ROS工作环境的src文件中编译即可。

运行方法：
1. 构建Turtlebot3仿真环境。
2. 运行gazebo仿真环境:: roslaunch turtlebot3_gazebo turtlebot3_world.launch   
3. 运行Gmapping::      roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping  
4. 启动键盘控制::       roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch  
5. 构建该库的工作环境
6. 发布位姿信息::       rosrun pubpose pubpose_node 
7. 运行opencv展示部分:: rosrun aeplanner aeplanner_node 
