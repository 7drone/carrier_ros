#include "carrier_ros_web_interface/carrier_ros_web_interface.h"

RobotControlServer::RobotControlServer()
{
    nh_ = ros::NodeHandle("~");
    system = "";
    data = 'i';
    system = "/robot/indoor";
    // Service Server 생성
    startServiceServer_ = nh_.advertiseService("/robot/start", &RobotControlServer::handleStartServiceRequest, this);
    stopServiceServer_ = nh_.advertiseService("/robot/stop", &RobotControlServer::handleStopServiceRequest, this);
    recallServiceServer_ = nh_.advertiseService("/robot/recall", &RobotControlServer::handleRecallServiceRequest, this);
    emergencyServiceServer_ = nh_.advertiseService("/robot/emergency", &RobotControlServer::handleEmergencyServiceRequest, this);

    // Topic Subscriber 생성
    systemSubscriber_ = nh_.subscribe("/robot/system", 1, &RobotControlServer::handleSystemMessage, this);
}

void RobotControlServer::run()
{
    ros::spin();
}

bool RobotControlServer::handleStartServiceRequest(carrier_ros_srv::RobotStart::Request& req, carrier_ros_srv::RobotStart::Response& res)
{
    if (data == 'o')
    {
        ros::ServiceClient client = nh_.serviceClient<carrier_ros_srv::RobotStart>(system+"/start");
        robotstart.request=req;
        if(client.call(robotstart)) ROS_INFO("Service call succeeded");
        else    ROS_ERROR("Failed to call service");
    }
    else
    {
        ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(system+"/start");
        if(client.call(trigger)) ROS_INFO("Service call succeeded");
        else    ROS_ERROR("Failed to call service");
    }
    return true; 
}

bool RobotControlServer::handleStopServiceRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(system+"/stop");

    if(client.call(trigger)) ROS_INFO("Service call succeeded");
    else    ROS_ERROR("Failed to call service");
    return true; 
}

bool RobotControlServer::handleRecallServiceRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(system+"/recall");

    if(client.call(trigger)) ROS_INFO("Service call succeeded");
    else    ROS_ERROR("Failed to call service");
    return true; 
}

bool RobotControlServer::handleEmergencyServiceRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(system+"/emergency");

    if(client.call(trigger)) ROS_INFO("Service call succeeded");
    else    ROS_ERROR("Failed to call service");
    return true; 
}

void RobotControlServer::handleSystemMessage(const std_msgs::Char::ConstPtr& msg)
{
    data=msg->data;
    if (msg->data == 'o')      system = "/robot/outdoor";
    else                        system = "/robot/indoor";
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_control_server");

    RobotControlServer controlServer;
    controlServer.run();

    return 0;
}