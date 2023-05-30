#include "carrier_ros_web_interface/carrier_ros_web_map.h"

webMap::webMap()
{
    nh_ = ros::NodeHandle("");
    initialPosePublisher_ = nh_.advertise<geometry_msgs::Pose>("web_robot_pose", 10);


    timer_ = nh_.createTimer(ros::Duration(0.05), &webMap::publishInitialPose, this);
}

void webMap::run()
{
    ros::spin();
}

void webMap::publishInitialPose(const ros::TimerEvent& event)
{   
    tf::StampedTransform transform;
    try
    {
        tfListener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    geometry_msgs::Pose initialPoseMsg;
    initialPoseMsg.position.x = transform.getOrigin().getX() + 3.2;
    initialPoseMsg.position.y = (transform.getOrigin().getY() + 14.3);
    initialPoseMsg.position.z = transform.getOrigin().getZ();
    initialPoseMsg.orientation.x = transform.getRotation().getX();
    initialPoseMsg.orientation.y = transform.getRotation().getY();
    initialPoseMsg.orientation.z = transform.getRotation().getZ();
    initialPoseMsg.orientation.w = transform.getRotation().getW();
    ROS_INFO("%f", initialPoseMsg.position.x);

    initialPosePublisher_.publish(initialPoseMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "carrier_ros_web_map");
    ROS_INFO("Start carrier_ros_web_map_node");
    webMap Webmap;
    Webmap.run();

    return 0;
}