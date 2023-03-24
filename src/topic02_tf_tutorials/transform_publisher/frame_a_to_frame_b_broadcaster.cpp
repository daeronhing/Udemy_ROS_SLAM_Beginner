#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "broadcaster");
    ros::NodeHandle nh;

    tf::TransformBroadcaster broadcaster;
    tf::Transform transformation;

    ros::Rate rate(1);

    while(ros::ok()) {
        transformation.setOrigin(tf::Vector3(1,1,0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transformation.setRotation(q);
        // transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        broadcaster.sendTransform(tf::StampedTransform(transformation, ros::Time::now(), "frame_a", "frame_b"));
        rate.sleep();
    }
    return 0;
}