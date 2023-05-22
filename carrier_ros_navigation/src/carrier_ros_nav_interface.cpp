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
    nh.param("station_latitude", station_latitude, 0.0);
    nh.param("station_longitude", station_longitude, 0.0);

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

  double station_latitude, station_longitude; /////////////////////////////////////make param

  bool startCallback(carrier_ros_srv::RobotStart::Request& req, carrier_ros_srv::RobotStart::Response& res)
  {
    
    std::vector<double> dest_que_x, dest_que_y;

    // req.lat,long 에 대한 위/경도 값 -> utm 좌표계로 변환
    // 변환한 값 -> dest_que 에 저장 
    // 위도 -> dest_que_x
    // 경도 -> dest_que_y 
    for (size_t i = 0; i < req.latitude.size(); i++)
    {
      double latitude = req.latitude[i];
      double longitude = req.longitude[i];

      geodesy::UTMPoint utm_point; // 객체 생성
      geographic_msgs::GeoPoint goepoint; // 포인트 위/경도 값 객체 생성
      goepoint.longitude = longitude; // 각 포인트에 대한 위 경도 값 변수 설정
      goepoint.latitude = latitude;
      geodesy::fromMsg(goepoint, utm_point, true, 's', 52); // utm_point 에 저장 

      dest_que_x.push_back(utm_point.easting);
      dest_que_y.push_back(utm_point.northing);
    }

    // robot statue cilent (e)
    robot_srv.request.status = 0;
    robot_status_client_.call(robot_srv);
    // Action to goal

    // for(auto v:dest_que_x)
    // {
      // navigateToGoal(dest_que_x.back(), dest_que_y.back()); // 예시로 (1.0, 2.0)으로 설정
      ///////////////////////////////////////////////////////////////
      // dest_que_x.pop_back();
      // dest_que_y.pop_back();
    // }
    //start
    // initial - 스테이션 위치로 위치

    double intial_state_x = 1.0;
    double intial_state_y = 2.0;
    navigateToGoal(intial_state_x, intial_state_y);

    for(size_t i = 1; i < dest_que_x.size(); i++)
    {
      navigateToGoal(dest_que_x[i], dest_que_y[i]);
    }
    if(!dest_que_x.empty() && !dest_que_y.empty())
    {
      navigateToGoal(dest_que_x[0], dest_que_y[0]);
    }
    res.success = true;
    // res.message = "Reach Destination Successed";
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
    goepoint.latitude = station_longitude;
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
