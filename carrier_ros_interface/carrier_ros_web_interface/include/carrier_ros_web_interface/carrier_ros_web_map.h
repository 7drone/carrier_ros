#ifndef WEB_MAP_H
#define WEB_MAP_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

class webMap
{
public:
    webMap();

    void run();

private:
    void publishInitialPose(const ros::TimerEvent& event);

    ros::NodeHandle nh_;
    ros::Publisher initialPosePublisher_;
    tf::TransformListener tfListener_;
    ros::Timer timer_;
};

#endif // WEB_MAP_H
