<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/ManchesterRoboticsLtd/TE3003B_Integration_of_Robotics_and_Intelligent_Systems/blob/main/Misc/Logos/Logotipo%20Vertical%20Bco_Transparente.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/ManchesterRoboticsLtd/TE3003B_Integration_of_Robotics_and_Intelligent_Systems/blob/main/Misc/Logos/Logotipo%20Vertical%20Azul%20transparente.png">
  <img alt="Shows ITESM logo in black or white." width="160" align="right">
</picture>

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/ManchesterRoboticsLtd/TE3003B_Integration_of_Robotics_and_Intelligent_Systems/blob/main/Misc/Logos/MCR2_Logo_White.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/ManchesterRoboticsLtd/TE3003B_Integration_of_Robotics_and_Intelligent_Systems/blob/main/Misc/Logos/MCR2_Logo_Black.png">
  <img alt="Shows MCR2 logo in black or white." width="150" align="right">
</picture>

---

# Gazebo Simulator

* This package contains the gazebo simulation files to run the robot model and the track for the Manchester Robotics Challenge.

## Installation

If you haven´t use the PuzzleBot simulation robot, it is important that you install the packages necessary for ROS Control. Please use the following command:

 `sudo apt-get install ros-$DISTRO$-ros-control ros-$DISTRO$-ros-controllers`

Download the folders 
  * puzzlebot_control
  * puzzlebot_gazebo
  * puzzlebot_world

into your folder `src`  in your usual catkin workspace (usually `~/catkin_ws/src`) , or create a new catkin workspace with the folders.  

<img src="https://user-images.githubusercontent.com/67285979/187089591-091a9058-dcc1-4abe-80fa-c4405f29bcea.png" alt="drawing" width="400"/>

* If you have already the folders in your catkin workspace Merge the folder, and replace the duplicated files. 

## Simulator documentation

 In order to execute this simulator run the following steps:

* Execute the following lines of code
  ```
  catkin_make
  source devel/setup.bash
  roslaunch puzzlebot_world puzzlebot_obstacle_world.launch
  ```
* Open a new terminal and run 
  ```
  source devel/setup.bash
  ```
Now you can use this terminal to interact with the simulator. Keep in mind that this instruction needs to be executed whenever you open a new terminal. Alternatively, you can add it to your .bashrc file. 

## Interface with Gazebo Simulator

* Install the teleop package using:
```
sudo apt get install ros $DISTRO$ teleop twist keyboard
```
* Run

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
* Input in the terminal the commands to move the robot (use the keyboard as explained in the terminal instructions)
* See how the robot traverses accordingly the environment.


## Add a new world to the MCR2 simulator

* Create your own Gazebo World ui.e., *test.world*
* Save the *.world* file inside the folder *Puzzlebot_Gazebo_Simulator/puzzlebot_world/worlds/*
* Open the launch file inside the folder *Puzzlebot_Gazebo_Simulator/puzzlebot_world/launch/puzzlebot_obstacle_world.launch*
* Change the following line with the name of your world

```
<arg name="world_name" value="$(find puzzlebot_world)/worlds/test.world"/>
```

* Save the launch file.
* Launch your nodes using the launch file *puzzlebot_obstacle_world.launch*


## Quick Troubleshooting
- ### Gazebo is not closing properly?
  If you closed the gazebo window, and terminal is still hanging. Use ` Ctl + C`  to kill the process. 
  
- ### Gazebo is not opening after quiting program?
  If the launcher does not display the interface, or the interface shuts down by itself during launching, the gazebo server and client process may be still stuck. Kill both with the following commands. 
  
  `killall gzserver` 
  `killall gzclient` 

- #### I have a different problem.

You can also raise a Git Issue if you have question, issues or doubts. 

## Extra references
- Gazebo GUI - https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b2
