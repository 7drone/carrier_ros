#ifndef ROBOT_CONTROL_SERVER_H
#define ROBOT_CONTROL_SERVER_H

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_srvs/Trigger.h>
#include <carrier_ros_srv/RobotStart.h>

class RobotControlServer
{
public:
    RobotControlServer();

    void run();

private:
    ros::NodeHandle nh_;
    ros::ServiceServer startServiceServer_,
                       stopServiceServer_,
                       recallServiceServer_,
                       emergencyServiceServer_;



    
    ros::Subscriber systemSubscriber_;
    

    std_srvs::Trigger trigger;
    char data;
    carrier_ros_srv::RobotStart robotstart;
    std::string system;
    bool handleStartServiceRequest(carrier_ros_srv::RobotStart::Request& req, carrier_ros_srv::RobotStart::Response& res);
    bool handleStopServiceRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool handleRecallServiceRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool handleEmergencyServiceRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void handleSystemMessage(const std_msgs::Char::ConstPtr& msg);
};

#endif // ROBOT_CONTROL_SERVER_H
