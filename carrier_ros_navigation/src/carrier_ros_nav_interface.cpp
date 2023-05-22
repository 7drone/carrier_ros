#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "carrier_ros_srv/RobotStart.h"
#include "carrier_ros_srv/RobotStatus.h"
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include <vector>

class NavInterface
{
public:
  NavInterface()
  {
    ros::NodeHandle nh;

    // ros service server
    robot_start_server_ = nh.advertiseService("/robot/start", &NavInterface::startCallback, this);
    robot_stop_server_ = nh.advertiseService("/robot/stop", &NavInterface::stopCallback, this);
    robot_recall_server_ = nh.advertiseService("/robot/recall", &NavInterface::recallCallback, this);
    robot_emergency_server_ = nh.advertiseService("/robot/emergency", &NavInterface::emergencyCallback, this);

    robot_status_client_ = nh.serviceClient<carrier_ros_srv::RobotStatus>("/robot/status");

    robot_srv.request.status=0;
    
    // ros navigation arction
    action_client_.reset(new MoveBaseClient("move_base", true));
    // MoveBase 서버가 준비될 때까지 대기
    ROS_INFO("Waiting for the move_base action server to come up...");
    // action_client_->waitForServer();
    ROS_INFO("The move_base action server is ready.");
    
  }

  // 목적지로 이동하기 위한 Navigation 액션 호출
  void navigateToGoal(double x, double y)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;

    action_client_->sendGoal(goal);
    action_client_->waitForResult();

    if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Navigation succeeded!");
    else
      ROS_WARN("Navigation failed.");
  }

private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  ros::ServiceServer robot_start_server_,
                     robot_stop_server_,
                     robot_recall_server_,
                     robot_emergency_server_;

  ros::ServiceClient robot_status_client_;

  carrier_ros_srv::RobotStatus robot_srv;

  boost::shared_ptr<MoveBaseClient> action_client_;

  double station_latitude, station_logitude; /////////////////////////////////////make param

  bool startCallback(carrier_ros_srv::RobotStart::Request& req, carrier_ros_srv::RobotStart::Response& res)
  {
    
    std::vector<double> dest_que_x, dest_que_y;
    for (size_t i = 0; i < req.latitude.size(); i++)
    {
      double latitude = req.latitude[i];
      double longitude = req.longitude[i];

      geodesy::UTMPoint utm_point;
      geographic_msgs::GeoPoint goepoint;
      goepoint.longitude = longitude;
      goepoint.latitude = latitude;
      geodesy::fromMsg(goepoint, utm_point, true, 's', 52);

      dest_que_x.push_back(utm_point.easting);
      dest_que_y.push_back(utm_point.northing);
    }

    //목적지로 이동전 로봇 상태 보내기
    robot_srv.request.status=0;
    robot_status_client_.call(robot_srv);
    // 목적지로 이동
    for(auto v:dest_que_x)
    {
      navigateToGoal(dest_que_x.back(), dest_que_y.back()); // 예시로 (1.0, 2.0)으로 설정
      /////////////////////////////////////////////////////////////////
      dest_que_x.pop_back();
      dest_que_y.pop_back();
    }
    //start
    
    res.success = true;
    return true;
  }

  bool stopCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    // 이동 중인 액션을 취소
    action_client_->cancelAllGoals();
    /////////////////////////////////////////////////////////
    //stop
    robot_srv.request.status=1;
    robot_status_client_.call(robot_srv);
    
    res.success = true;
    res.message = "Navigation stopped.";
    return true;
  }


  bool recallCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    // 목적지로 이동
    
    geodesy::UTMPoint utm_point;
    geographic_msgs::GeoPoint goepoint;
    goepoint.longitude = station_latitude;
    goepoint.latitude = station_logitude;
    geodesy::fromMsg(goepoint, utm_point, true, 's', 52);




    action_client_->cancelAllGoals();
    navigateToGoal(utm_point.easting, utm_point.northing);

    //come back
    robot_srv.request.status=2;
    robot_status_client_.call(robot_srv);

    res.success = true;
    res.message = "Navigation recalled to goal station.";
    return true;
  }

  bool emergencyCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    // 이동 중인 액션을 취소
    action_client_->cancelAllGoals();

    //stop
    robot_srv.request.status=1;
    robot_status_client_.call(robot_srv);

    res.success = true;
    res.message = "Emergency handled";
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_controller");
  NavInterface navinterface;

  ros::spin();
  return 0;
}
