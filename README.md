# Adaptive Monte Carlo Localization
## Using Gazabo, Rviz, and ROS

![](images/AMCL.png)

## Setting up the enviroment:
For this project the following set up was used:
- Ubuntu 16.04 LTS OS
- Ros kinetic
- Gazebo 7.0.0
- Rviz 1.12.17

## Installation steps:
- Clone this repository to your home directory:
```
$ git clone https://github.com/AndresGarciaEscalante/Adaptive-Monte-Carlo-Localization.git
```
- Review the documentation and dependecies of the Packages mentioned below.

**Launch files for master branch**:
- Run the following commands in a terminal:
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
- Open a new terminal and execute the following commands:
```
$ source devel/setup.bash
$ roslaunch my_robot amcl.launch
```
- Load the the Rviz configuration located in the rviz_config folder.

- Use the **2D Nav Goal** option to drive around. If the robot lose its position use the **2D Pose Estimate**.

**Launch files for Teleop branch**
- Run the following commands in a terminal:
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
- Open a new terminal and execute the following commands:
```
$ source devel/setup.bash
$ roslaunch my_robot amcl.launch
```
- Finally one more terminal and execute the following commands:
```
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```
- Load the the Rviz configuration located in the rviz_config folder.

- Using the third terminal you will be allowed to control the robot position with the keyboards.

## Project Description
### My_robot Package
Provides the gazebo world environment and the robot model. 

### Pgm_map_creator Package
This package generates a **.pgm** file of a gazebo world. Using this powerful tool I was allowed to generate the folllowing 2D map:

![](images/map.png)

By default, AMCL package will treat 'darker' pixels as obstacle in the pgm map file, and 'lighter' pixels as free space. The threshold could be set as a parameter which we will cover when we are building the launch file.

For more detailed information please refere to the following link:
[pgm_map_creator](https://github.com/udacity/pgm_map_creator.git)

### Teleop_twist_keyboard Package (Teleop Branch)
Provides a control of the car by using keyboards.

For more detailed information please refere to the following link:
[teleop_twist_keyboard Package](https://github.com/ros-teleop/teleop_twist_keyboard)

### Map Server Node
The map_server node provides map data as a ROS service to other nodes such as the amcl node. Here, **map_server node** will **locate the map you created** in the Map Setup step and send it out as the map data.

### AMCL Node
It takes **odometry** and **laser scan** data to perform the AMCL localization.

### Move Base Node 
The move_base package is a very powerful tool. It utilizes a **costmap** - where each part of the map is divided into which area is occupied, like walls or obstacles, and which area is unoccupied. As the robot moves around, a **local costmap**, in relation to the **global costmap**, keeps getting updated allowing the package to define a continuous path for the robot to move along.

What makes this package more remarkable is that it has some built-in corrective behaviors or maneuvers. Based on specific conditions, like detecting a particular obstacle or if the robot is stuck, it will navigate the robot around the obstacle or rotate the robot till it finds a clear path ahead

## Project Outcome
The main objective of the project is to test the **AMCL** to determine the **position of the robot given a 2D map**.

**master branch**:
- The robot will move to a navigation goal by its own. It only needs a goal point to start the movement.

![](gif/AMCL_Autonomous.gif)

**Important:** Check full video in the following link:
[AMCL_Autonomous_Project](https://www.youtube.com/watch?v=vw_842utXio)

**Teleop branch**:
- The robot will move by using the keyboards.

![](gif/AMCL_Teleop.gif)

**Important:** Check full video in the following link:
[AMCL_Teleop_Project](https://www.youtube.com/watch?v=mieZXECb5GY)

## Future Improvements
- Create a single launch file to execute the project.