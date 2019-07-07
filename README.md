# guiding-robot-code
code for guiding robot, include tx2 and jetson nano parts

## How to run
When the robot power up, tx2 will set to ros master automatically. The things you need to do on tx2 is launching LiDAR sensor and 
launch motor controller and D435 on Jetson Nano.


## Gazebo
#### Requirements:
 -  $ sudo apt-get install ros-kinetic-joint-state-controller
 -  $ sudo apt-get install ros-kinetic-effort-controllers
 -  $ sudo apt-get install ros-kinetic-position-controllers
 -  $ sudo apt-get install ros-kinetic-velodyne
 -  $ sudo apt-get install ros-kinetic-velodyne-gazebo-plugins

#### Robot model
The robot model is available for 3D LiDAR sensing and differential drive motor control now.
```bash
$ roslaunch atbot_description gazebo.launch use_rviz:=true  
```
![image](figures/atbot_rviz.png)

#### move_base
```bash
$ roslaunch pointcloud_to_laserscan pcl2laser.launch 
$ roslaunch atbot move_base.launch
```
![image](figures/atbot_movebase.png)

