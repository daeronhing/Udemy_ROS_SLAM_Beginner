#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    tf::StampedTransform transformation;

    ros::Rate rate(1);

    tf::Vector3 origin;
    tf::Quaternion quat;

    double roll;
    double pitch;
    double yaw;

    while(ros::ok()) {
        try {
            listener.lookupTransform("frame_a", "frame_b", ros::Time(0), transformation);
            origin = transformation.getOrigin();
            quat = transformation.getRotation();
            // printf("\nX: %f, Y: %f", origin.x(), origin.y());
            // printf("\nX: %f, W: %f", quat.x(), quat.w());
            tf::Matrix3x3 m(quat);
            m.getRPY(roll, pitch, yaw);
            printf("\nAngle: [%f, %f, %f]", roll, pitch, yaw);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            // ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }
    return 0;
}