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

    robot_start_server_ = nh.advertiseService("/robot/start", &NavInterface::startCallback, this);
    robot_stop_server_ = nh.advertiseService("/robot/stop", &NavInterface::stopCallback, this);
    robot_recall_server_ = nh.advertiseService("/robot/recall", &NavInterface::recallCallback, this);
    robot_emergency_server_ = nh.advertiseService("/robot/emergency", &NavInterface::emergencyCallback, this);

    robot_status_client_ = nh.serviceClient<carrier_ros_srv::RobotStatus>("/robot/status");

    robot_srv.request.status=0;

    action_client_.reset(new MoveBaseClient("move_base", true));
    ROS_INFO("Waiting for the move_base action server to come up...");
    ROS_INFO("The move_base action server is ready.");
    
  }

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

  double station_latitude{1.0}, station_longitude{2.0}; 

  bool isNavigating = false;
  bool isFirstStart = true;
  int currentGoalIndex = 0;
  int pausedGoalIndex = -1;

  bool stopCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    pausedGoalIndex = currentGoalIndex;
    isNavigating = false;

    action_client_->cancelAllGoals();
    //stop
    robot_srv.request.status = 1;
    robot_status_client_.call(robot_srv);
    
    res.success = true;
    res.message = "Navigation stopped.";
    return true;
  }

  bool startCallback(carrier_ros_srv::RobotStart::Request& req, carrier_ros_srv::RobotStart::Response& res)
{
  std::vector<double> dest_que_x, dest_que_y;
  for (size_t i = 0; i < req.latitude.size(); i++)
  {
    double latitude = req.latitude[i];
    double longitude = req.longitude[i];

    geodesy::UTMPoint utm_point; 
    geographic_msgs::GeoPoint geo_point; 
    geo_point.longitude = longitude; 
    geo_point.latitude = latitude;
    geodesy::fromMsg(geo_point, utm_point, true, 's', 52);
    dest_que_x.push_back(utm_point.easting);
    dest_que_y.push_back(utm_point.northing);
  }

  if (!isFirstStart)
  {
    if (currentGoalIndex < dest_que_x.size() - 1)
    {
      dest_que_x.erase(dest_que_x.begin() + currentGoalIndex + 1, dest_que_x.end());
      dest_que_y.erase(dest_que_y.begin() + currentGoalIndex + 1, dest_que_y.end());
    }
  }
  else
  {
    for (size_t i = 0; i < req.latitude.size(); i++)
    {
      double latitude = req.latitude[i];
      double longitude = req.longitude[i];
      geodesy::UTMPoint utm_point;
      geographic_msgs::GeoPoint geo_point; 
      geo_point.longitude = longitude;
      geo_point.latitude = latitude;
      geodesy::fromMsg(geo_point, utm_point, true, 's', 52);
      dest_que_x.push_back(utm_point.easting);
      dest_que_y.push_back(utm_point.northing);
    }
  }

  currentGoalIndex = 0;
  isNavigating = true;
  isFirstStart = false;

  // Robot Goal -> Station -> Waypoints -> Destination
  // navigateToGoal(initial_state_x, initial_state_y); // utm 변환 값으로 넣어줘야 함
  if(pausedGoalIndex != -1)
  {
    for(size_t i = pausedGoalIndex; i < dest_que_x.size(); i++)
    {
      navigateToGoal(dest_que_x[i], dest_que_y[i]);
      if (i == dest_que_x.size() - 1)
      {
        ROS_INFO("Reached the last waypoint");
      }
    }
    pausedGoalIndex = -1;
  }
  else
  {
    navigateToGoal(station_latitude, station_longitude); // Initail Station Set -> UTM Transformation
    for(size_t i = currentGoalIndex + 1; i < dest_que_x.size(); i++)
    {
      navigateToGoal(dest_que_x[i], dest_que_y[i]);
      if (i == dest_que_x.size() - 1)
      {
        ROS_INFO("Reached the last waypoint");
      }
    }
  }
  navigateToGoal(dest_que_x[0], dest_que_y[0]);
  ROS_INFO("Reached the destination.");

  res.success = true;
  return true;
}

  bool recallCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    // recall 에 대해 고려
    
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
    action_client_->cancelAllGoals(); 
    navigateToGoal(station_latitude, station_longitude);
    //stop
    robot_srv.request.status=1;
    robot_status_client_.call(robot_srv);

    // @@@@ stop -> move initial station @@@@
    res.success = true;
    res.message = "Emergency handled, Return to station";
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
