# Instructions

## Initial steps:

1. Download comprobfall2018-hw3.zip 

2. Unzip the folder inside ```$HOME/catkin_ws/src```

3. Run the following commands to compile and source the setup script:

```
$ cd $HOME/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

You might have to do rosdep update and rosdep install.

## Running the software:

1. Open a terminal and run the following commands:
```
$ roscore &
$ rosparam set scan_noise <std.deviation of scan noise>
$ rosparam set rotation_noise <std.deviation of angular noise>
$ rosparam set translation_noise <std.deviation of linear noise>
$ export ROBOT_INITIAL_POSE="-x <x of start position> -y <y of start position>" 
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<absolute path to world_x.world>
```
If you are running the experiment corresponding to ```map_x.txt```, you must provide the absolute path to the world file ```world_x.world```. The world files are located in folder ```$HOME/catkin_ws/src/comprobfall2018-hw3/turtlebot_simulator/turtlebot_gazebo/worlds```.

2. In a separate terminal, run the controller service server by running:
   
   ```
   $ rosrun turtlebot_ctrl turtlebot_control.py
   ```
   
## Subscribing to topics:

### Topic: turtlebot_scan
```/turtlebot_scan``` is a ROS topic that is published to by the node turtlebot_scan_handler, which is provided by us. The message description is defined by turtlebot_ctrl/TurtleBotScan.msg.


```$ rosmsg show turtlebot_ctrl/TurtleBotScan```

To see the messages getting published to the topic:

```$ rostopic echo /turtlebot_scan```

The message has one component: a float32 array called "ranges" of size 54. It contains range data within angle -30 degrees to +30 degrees.   


## Communicating with service:

### Service: turtlebot_control
turtlebot_control is a ROS service provided by us, which uses the service
description defined by turtlebot_ctrl/TurtleBotControl.srv. 

```$ rossrv show turtlebot_ctrl/TurtleBotControl```

The srv file is divided into two parts, separated by a − − −. The top part of
the srv file is the request, while the bottom part of the srv file is the response.

## Part B World:
![alt text](https://github.com/rajanya/comprobfall2018-hw3/blob/master/screenshots/partB_world.png)
