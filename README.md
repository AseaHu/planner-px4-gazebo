## planner-px4-gazebo
### 1. 将本仓库中PX4-Autopilot文件夹中的文件复制到计算机本地的对应文件夹
### 2. 编译planner_interface
### 3. 运行

- 运行px4：`$ roslaunch px4 plan_test.launch`

- 运行planner及接口节点：`$ source devel/setup.bash && roslaunch run1.launch （或 $ source devel/setup.bash && roslaunch run2.launch）` 

- 当第二个终端提示“Vehicle armed”之后，可以使用rviz的“2D NAV Gaol”工具点击地图界面设置目标点
