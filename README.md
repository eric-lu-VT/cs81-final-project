# README.md - CS81 Final Project

This project provides an implementation of a robotic dog that is capable of searching a foreign environment for a target object (“ball”) and returning the object to a specified location using laser sensor and camera data. As a consequence of our implementation, the robot is also capable of mapping the explored environment as it proceeds in its search. We demonstrate through use of an OccupancyGrid paired with a laser sensor, a novel frontier-exploration algorithm, computer vision object detection and tracking using OpenCV and a camera, and a PD controller for object retrieval movement patterns that our robot dog implementation is capable of autonomously completing the specified task.

Authors: Eric Lu, Jordan Kirkbride, Julian Wu, Wendell Wu

## Setup

### Local Environment

These setup instructions assume that you have already [set up the ROS development environment](https://canvas.dartmouth.edu/courses/58298/pages/instructions-for-setting-up-ros-directly-ubuntu-or-docker) **through Docker**.

To download the program to your local PC environment:

1. Navigate to your ROS dev directory and run `docker-compose up --build`
2. On a *second* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
3. `cd ~/catkin_ws/src` and run `catkin_create_pkg cs81-final-project std_msgs rospy` to create a new catkin package with the necessary dependencies
4. `cd ~/catkin_ws` and run `catkin_make` to update the packages in the catkin workspace
5. Copy the contents of this repository into the `~/catkin_ws/src/cs81-final-project` directory
6. `chmod +x group8_final.py` to grant the Python file executable permissions
7. `apt-get install ros-melodic-nav2d-tutorials` to install the 2D simulator
8. Run `sudo apt-get install ros-melodic-rviz`

To run the program locally on your PC:

1. Navigate to your ROS development directory and run `docker-compose up --build`
2. On a *second* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
3. Run `source /opt/ros/melodic/setup.bash`
4. Run `roscore` to start the VNC viewer
5. On a *third* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
6. `cd ~/catkin_ws/src`, `export TURTLEBOT3_MODEL=waffle`, and then run `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
7. On a *fourth* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
8. `cd ~/catkin_ws/src` and run `rviz`
9. Open your web browser to `localhost:8080/vnc.html` and click connect.
10. In gazebo: add a sphere shape with radius 0.125m and rgba color (1, 0, 0, 1) for ambient, diffuse, specular, and emissive
11. In gazebo: insert/change the world model to something you want
   - We used the `turtlebot3_plaza` and `turtlebot3_square` from `opt/ros/melodic/share/turtlebot3_gazebo/models`
12. On rviz: Go to "panels" in the menu bar, then hit add new panel, then hit tool properties. This should bring up a sidebar to the left.
13. On rviz: In the left sidebar, click Global Options > Fixed Frame > change to "odom"
14. On rviz: In the left sidebar, click add > By display type > Map. Then change its topic name to "map"
15. On rviz: In the left sidebar, click add > By display type > PointCloud. Then change its topic name to "point_cloud", and size to 0.25 m (so that it can be more easily seen in rviz)
16. On a *fifth* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
17. `cd ~/catkin_ws/src` and run `rosrun cs81-final-project group8_final.py`
18. The robot should now be moving in the VNC viewer on the browser, there should be OccupancyGrid mapping + PointCloud visualizations in the rviz, and there should be a cv2 camera display in gazebo.