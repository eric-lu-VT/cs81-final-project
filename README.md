# README.md - CS81 Final Project

This programming assignment

Authors: Eric Lu, Jordan Kirkbride, Julian Wu, Wendell Wu

## Setup

### Local Environment

These setup instructions assume that you have already [set up the ROS development environment](https://canvas.dartmouth.edu/courses/58298/pages/instructions-for-setting-up-ros-directly-ubuntu-or-docker) **through Docker**.

First and foremost, make sure to change the laser scan topic name in the code to `base_scan`, and base laser link topic to `base_laser_link`.

To download the program to your local PC environment:

1. Navigate to your ROS dev directory and run `docker-compose up --build`
2. On a *second* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
3. `cd ~/catkin_ws/src` and run `catkin_create_pkg cs81-final-project std_msgs rospy` to create a new catkin package with the necessary dependencies
4. `cd ~/catkin_ws` and run `catkin_make` to update the packages in the catkin workspace
5. Copy the contents of this repository into the `~/catkin_ws/src/cs81-final-project` directory
6. `chmod +x robot_dog.py` to grant the Python file executable permissions
7. `apt-get install ros-melodic-nav2d-tutorials` to install the 2D simulator
8. Run `sudo apt-get install ros-melodic-rviz`

To run the program locally on the PC:

1. Navigate to your ROS development directory and run `docker-compose up --build`
2. On a *second* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
3. Run `source /opt/ros/melodic/setup.bash`
4. Run `roscore` to start the VNC viewer
5. On a *third* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
6. `cd ~/catkin_ws/src` and run `rosrun stage_ros stageros /opt/ros/melodic/share/nav2d_tutorials/world/tutorial.world`
7. On a *fourth* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
8. `cd ~/catkin_ws/src` and run `rviz`
9. Open your web browser to `localhost:8080/vnc.html` and click connect.
10. On rviz: Go to "panels" in the menu bar, then hit add new panel, then hit tool properties. This should bring up a sidebar to the left.
11. In the sidebar, click add > by topic > /map > set topic to `/map` 
12. On a *fifth* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
13. `cd ~/catkin_ws/src` and choose one of the following:
    - To run pre-determined motion (mostly for simulation): `rosrun cs81_final_project robot_dog.py 1`
    - To let all motion be determined from the command line: `rosrun cs81_final_project  robot_dog.py 2`
14. The robot should now be moving in the VNC viewer on the browser, and there should also be a OccupancyGrid mapping in the rviz.
    - To see the OccupancyGrid mapping, you may need to zoom out focal point (on the right sidebar in rviz)

### Robot Environment

These setup instructions assume that you are using one of the Turtlebot 3 robots provided by Prof. Li. 
Let # be the id of the robot you are using; this will be used in the instructions below.

First and foremost, make sure to change the laser scan topic name in the code to `scan` and the base laser link topic to `base_scan`.

To download the program to the robot:

1. `ssh dartmouth@192.168.111.11`
   - password: `Robotics&7`
2. `sudo apt-get install tmux` if the robot does not already have tmux
3. `cd ~/catkin_ws/src` and run `catkin_create_pkg cs81-final-project std_msgs rospy` to create a new catkin package with the necessary dependencies
4. `cd ~/catkin_ws` and run `catkin_make` to update the packages in the catkin workspace
5. On a *second* terminal, navigate to the directory that contains the file `robot_dog.py`
6. Copy over files to robot with `scp -r robot_dog.py dartmouth@192.168.111.11:~/catkin_ws/src/cs81-final-project/src`
   - password: `Robotics&7`

To run the program on the robot:

1. `ssh dartmouth@192.168.111.11`
   - password: `Robotics&7`
2. Open *first* tmux terminal with `tmux`
3. Run `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
4. Ctrl b + d to exit the first tmux terminal
5. Open *second* tmux terminal with `tmux`
6. `cd ~/catkin_ws/src/cs81-final-project/src/`
7. Start running the robot:
    - To run pre-determined motion (mostly for simulation): `rosrun cs81_final_project robot_dog.py 1`
    - To let all motion be determined from the command line: `rosrun cs81_final_project  robot_dog.py 2`

Note that to open teleoperation, run `rosrun turtlebot3_teleop turtlebot3_teleop_key` from a new terminal connected to `ssh dartmouth@192.168.111.11`

To access `rviz` while running the program on the robot:

1. Navigate to your ROS development directory and run `docker-compose up --build`
2. On a *second* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
3. Run `export ROS_MASTER_URI=http://192.168.111.11:11311`
   - You can verify that the `ROS_MASTER_URI` was changed with `env | grep ROS`
   - You can verify that the real robot frames can be accessed with `rostopic list`
4. `cd ~/catkin_ws/src` and run `rviz`
5. Open your web browser to `localhost:8080/vnc.html` and click connect.
6. On rviz: Go to "panels" in the menu bar, then hit add new panel, then hit tool properties. This should bring up a sidebar to the left.
7. In the sidebar, find Global Options > Fixed Frame > set frame to `odom`
8. In the sidebar, click add > by topic > /map > set topic to `/map`
9. Follow the instructions 'run the program on the robot' as listed in the previous section.