# Notes for SLAM and Navigation in ROS
This note contains some basic understanding of SLAM, navigation and map, included demo using turtlebot3, and tuning of the parameters.

# Content
- [SLAM](#slam-simultaneous-localization-and-mapping)
- [Map-based Navigation](#map-based-navigation)
- [Demos](#demos)
    - [SLAM](#1-slam-demo-using-turtlebot3)
    - [Navigation](#2-map-based-navigation-demo-using-turtlebot3)
- [Map Structure](#map-structure)
- [Robot Setup to Support ROS Navigation Stack](#robot-setup-to-support-the-ros-navigation-stack)
- [Navigation Parameter Tuning](#navigation-parameter-tuning)
    - [Velocity & Acceleration](#1-velocity--acceleration)
    - [Global Planner](#2-global-planner-parameter-tuning)
    - [Local Planner](#3-local-planner-parameter-tuning)
- [Reactive Navigation](#reactive-navigation-bug-algorithm)

# SLAM (Simultaneous Localization and Mapping)
There are several SLAM approaches in ROS:
1. gmapping: contains a ROS wrapper for OpenSlam's Gmapping.
2. cartographer: developed by Google that provides *<b>real_time</b> simultaneos localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations
3. hector_slam: can be used without odometry  

# Map-based Navigation
Three main packages of the navigation stack:
1. map_server: used to publish and manipulate map files
2. amcl: responsible for localization using an existing map
3. move_base: makes the robot navigate in a map and move to the goal pose with respect to a given reference frame

# Demos
## 1. SLAM demo using Turtlebot3
1. Start turtlebot3 simulation
2. Open gmapping SLAM application
3. Control the robot to move around
4. Save the map

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_method:=gmapping
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun map_server map_saver -f [directory to save map files]
```

## 2. Map-based navigation demo using Turtlebot3
1. Start Turtlebot3 simulation
2. Launch navigation file

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=[path to map.yaml]
```

# Map Structure
To save map from the SLAM package, run this command to generate map.pgm and map.yaml file
```
rosrun map_server map_saver -f [directory to save map files]
```

## 1. pgm
This .pgm file is the image of the map itself.
```
P5          # First line is format used to encode the numbers into binary
Creator
Int Int     # 3rd line is width & length in pixels (rows & columns)
255         # 4th line is maximum gray scale
```

## 2. yaml
This .yaml file is the metadata of the pgm file
```
image       # Path to the image(.pgm) file
resolution  # Unit is: m/pixel
origin      # 2D pose of the lower left corner pixel of image
negate
occupied_threshold  # If occupancy probability > threshold, it is occupied
free_threshold      # If occupancy probability < threshold, it is completely free
```

# Robot Setup to Support The ROS Navigation Stack
![image](overview_tf.png)

List to setup:
1. Transformation (describe the relation between coordinate frames, through urdf or tf broadcaster)
2. Sensor information (sensor_msgs/LaserScan, sensor_msgs/PointCloud)
3. Odometry information (nav_msgs/Odometry)
4. Base controller (navigation stack assumes velocity commands can be sent using a geometry_msgs/Twist message, and the message is assumed to be in base coordinate frame)
5. Mapping (map_server)

# Navigation Parameter Tuning
## 1. Velocity & Acceleration
- Parameters of the velocity and acceleration are provided in the yaml configuration file of the local planner.
- For turtlebot3, the parameters are located at turtlebot3_navigation/param

### 1.1 How to obtain maximum velocity?
1. Translation: Move robot in a straight line until speed becomes constant, then echo the odom topic and record the maximum speed value
2. Rotation: Rotate the robot until it reaches a constant speed, then echo odom and record maximum speed value

### 1.2 How to obtain maximum acceleration
1. Try to get from motors manual
2. Use time stamp in the odom message to estimate the acceleration from the velocity
3. accel_translation = Vmax/t_Vmax
4. accel_rotation = Wmax/t_Wmax

### 1.3 How to set minimum values
Set the minimum translation and rotation to negative value so that it can move in two directions

## 2. Global Planner Parameter Tuning
Global planner is responsible for finding a global obstacle-free path from initial location to the goal location using environment map

It must adhere to nav core::BaseGlobalPlanner interface

Three types of built-in global planner in ROS:
1. carrot_planner: Simple global planner that will attempt to move the robot to goal as close as possible, even when goal point is an obstacle
2. navfn: Uses Dijkstra's algorithm to find the global path between any two locations
3. global_planner: a replacement of navfn that is more flexible and has more options

## 3. Local Planner Parameter Tuning
Local planner is responsible to execute the static path generated by global planner while avoiding dynamic obstacle

Must adhere to nav core::BaseLocalPlanner interface

### 3.1 Dynamic Window Approach (DWA)
1. A few translation & rotational velocity pairs will be generated
2. Each velocity pairs will be used to perform a simulation to predict the outcome of the velocity pair running for a short period of time.
3. Discard illegal trajectory (collision)
4. Evaluate (score) each trajectory from the simulation 
5. Pick the trajectory with highest score
6. Repeat

DWA depends on the local costmap which provides obstacle information

### 3.2 Tuning simulation time of DWA
simulation time: time allowed for the robot to move with the sampled velocities

Long simulation time (>=5): Heavy computation, but longer path

Short simulation time (<=2): limited performance, especially narrow doorway or gap between furnitures, because there is insufficient time to obtain the optimal trajectory that actually goes through

### 3.3 Trajectory Scoring
$cost =$
$path.distance.bias$ # Bias to stick trajectory to static path<br> 
$+ goal.distance.bias$ # Bias to choose trajectory closer to goal <br>
$+occdist.scale$    # How much to avoid obstacle

```
rosrun rqt_reconfigure rqt_reconfigure
# Can bse used to adjust parameters used in ROS dynamically
```

# Reactive Navigation (Bug Algorithm)
Reactive navigation runs without map, purely based on its sensors to avoid obstacle.

## Bug 0
1. Behavior 1: Head to goal straight
2. Behavior 2: Follow obstacle boundary

It will repeat behavior 1 until obstacle is detected, then behavior 2 until no obstacle in direction of the goal. Then loop again until goal reached.

## Bug 1
1. Behavior 1: Head to goal straight
2. Behavior 2: follow the obstacle, make a tour around it, and keep track of closest location to goal
3. Behavior 3: go to the point closts to goal

It will repeat behavior 1 until obstacle is detected, behavior 2 will be triggered and take one whole lap around obstacle, and trigger behavior 3 to go to the point closest to goal. Then loop again until goal reached.

## Bug 2
1. Behavior 1: Head to goal straight
2. Behavior 2: follow the obstacle, make a tour around it

It will repeat behavior 1 until obstacle is detected, then it will record its current location (point A) and trigger behavior 2. It will then make a tour around obstacle until its location lies on the straight line between point A and goal.

# Extra
gmapping:
http://wiki.ros.org/gmapping

urdf:
http://wiki.ros.org/urdf/Tutorials

odom:
http://wiki.ros.org/navigatoin/Tutorials/RobotSetup/Odom

move_base package link:
http://wiki.ros.org/move_base

recovery behavior link:
http://wiki.ros.org/rotate_recovery

