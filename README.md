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
   - Or `rosrun stage_ros stageros ~/catkin_ws/pa1/PA1.world`
   - Or `rosrun stage_ros stageros ~/catkin_ws/pa2/2017-02-11-00-31-57.world`
7. On a *fourth* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
8. `cd ~/catkin_ws/src` and run `rviz`
9. Open your web browser to `localhost:8080/vnc.html` and click connect.
10. On rviz: Go to "panels" in the menu bar, then hit add new panel, then hit tool properties. This should bring up a sidebar to the left.
11. In the left sidebar, click Global Options > Fixed Frame > change to "odom"
12. In the left sidebar, click add > By display type > Map. Then change its topic name to "map"
13. In the left sidebar, click add > By display type > PointCloud. Then change its topic name to "point_cloud", and size to 1m (so that it can be more easily seen in rviz)
14. On a *fifth* terminal, navigate to your ROS dev directory and enter rosbash with `docker-compose exec ros bash`
15. `cd ~/catkin_ws/src` and run `rosrun cs77-final-project robot_dog.py`
16. The robot should now be moving in the VNC viewer on the browser, and there should also be a OccupancyGrid mapping in the rviz.
   - To see the OccupancyGrid mapping, you may need to change the focal point (on the right sidebar in rviz)