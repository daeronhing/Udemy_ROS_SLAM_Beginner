# Notes For Transformation
This notes contains quick guide about transformation and the basic usage of TF package in ROS.

Add these in package.xml
```xml
<build_depend>geometry_msgs</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>nav_msgs</build_depend>

<build_export_depend>geometry_msgs</build_export_depend>
<build_export_depend>tf</build_export_depend>

<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav_msgs</exec_depend>
<exec_depend>tf</exec_depend>
```

These changes need to be made in CMakeList
```
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs # Add
  nav_msgs      # Add
  tf            # Add
)
```

# 2D Transformation
From frame 1 rotated angle $\theta$ to frame 2. Transformation (translation + rotation) between frame 1 and 2 is as follow:

$\begin{bmatrix}    X_{1}   \\  Y_{1}   \\  1   \end{bmatrix}$  =
$\begin{bmatrix}    cos\theta & -sin\theta & x_{t}  \\  sin\theta & cos\theta & y_{t}   \\  0 & 0 & 1   \end{bmatrix}$
$\begin{bmatrix}    X_{2}   \\  Y_{2}   \\  1   \end{bmatrix}$

# 3D Rotation
R<sub>x</sub> =
$\begin{bmatrix}    1 & 0 & 0   \\  0 & cos\theta & -sin\theta  \\  0 & sin\theta & cos\theta    \end{bmatrix}$

R<sub>y</sub> =
$\begin{bmatrix}    cos\theta & 0 & sin\theta   \\  0 & 1 & 0   \\  -sin\theta & 0 & cos\theta    \end{bmatrix}$

R<sub>z</sub> =
$\begin{bmatrix}    cos\theta & -sin\theta & 0  \\  sin\theta & cos\theta & 0   \\  0 & 0 & 1   \end{bmatrix}$

If rotation as follow happens:
1.  Rotate $\alpha$ degree about Z-axis
2.  Rotate $\beta$  degree about Y-axis
3.  Rotate $\gamma$ degree about X-axis

Rotation matrix can be expressed as: <br/>
$R = R_{Z}(\alpha) R_{Y}(\beta) R_{X}(\gamma)$

# TF Package
In TF package, the frames can be either:
- Published by a broadcaster node
- Subscribed by a node that listen to the frames

## Useful command lines
### 1. view_frames
This command line will generate a pdf to store relation and information of all frames in a graphical style
```bash
cd [directory to store image pdf]
rosrun tf view_frames
```

However, in noetic distro, the above package is deprecated, hence need to install tf2-tools

```bash
sudo apt-get install ros-noetic-tf2-tools
cd [directory to store image pdf]
rosrun tf2-tools view_frames
```

### 2. tf_monitor
This command shows all transformations that are currently published in "/tf" topic. The frames and its broadcaster information will be shown.
```
rosrun tf tf_monitor
```

### 3. tf_echo
This command shows the current translation and rotation from frame_A to frame_B in a way that is easy to read.
```
rosrun tf tf_echo frame_A frame_B
```
### 4. roswtf
This tfwtf plugin helps to track down problems with tf.

## transform_broadcaster
This is used to send transformation updates to the "/tf" topic

Python:
```python
import tf
import rospy

transform_broadcaster = tf.TransformBroadcaster()   # Create a broadcaster

translation_vector = tuple(x,y,z)       # Set translation
rotation_quaternion = tuple(x,y,z,w)    # Set rotation
current_time = rospy.Time.now()

# Broadcast the transformation
transform_brodcaster.sendTransform(translation_vector, 
                                   rotation_quaternion, 
                                   current_time, 
                                   child_frame,  # frame 2
                                   parent_frame) # frame 1
```

C++:
```c++
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

tf::TransformBroadcaster broadcaster;   // Create a broadcaster
tf::Transform transformation;           // Create a transformation variable

transformation.setOrigin(tf::Vector3(x,y,z)); // Set translation
// Set rotation
tf::Quaternion q;
q.setRPY(0, 0, 0);
transformation.setRotation(q);

// Another way to set rotation without using RPY
transformation.setRotation(tf::Quaternion(x,y,z,w));

broadcaster.sendTransform(tf::StampedTransform(transformation, ros::Time::now(), 
                            parent_frame, child_frame)); // Frame 1, Frame 2
```

## transform_listener
This is used to subscribe to the "/tf" topic and get the transformation between two frames

Python:
```python
import tf

listener = tf.TransformListener()   # Create a listener
listener.waitForTransform(target_frame, source frame, rospy.Time(), rospy.Duration(sec))    # Optional
(translation, rotation) = listener.lookupTransform(target_frame,    # Frame 1
                                                    source frame,   # Frame 2
                                                    rospy.Time(0))
```

C++:
```c++
#include<tf/transform_listener.h>

tf::TransformListener listener;
tf::StampedTransform transformation

try{
    listener::lookupTransform(target_frame, source_frame, ros::Time(0), transformation);
}

catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    continue;
}

tf::Vector3 origin = transformation.getOrigin();
tf::Quaternion quat = transformation.getRotation();
double x = origin.x();
double q_w = quat.w();
```

> Target frame denotes the frame in the left hand side of the equation in section 2D/3D transformation. Source frame is the frame in right hand side.

## Roll,Pitch,Yaw & Quaternion Conversion
Python:
```python
import tf

Quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
# This line will return quaternion with numpy.ndarray data class
# Shape: (4,) with elements in order of (x,y,z,w)

RPY = tf.transformations.euler_from_quaternion(quaternion)
# This line will return a tuple in this order: roll, pitch and yaw
# It takes only tuple or numpy.ndarray data class as input
# quaternion = tuple(x, y, z, w) or np.array(x, y, z, w)
```

C++:
```c++
#include <tf/tf.h>

// Convert RPY into quaternion
tf::Quaternion q;
q.setRPY(roll, pitch, yaw);

// Convert quaternion into RPY
tf::Matrix3x3 m(q);
m.getRPY(roll, pitch, yaw);
```

# Extra
urdf
http://wiki.ros.org/urdf/Tutorials