#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "carrier_ros_srv/RobotStart.h"
#include "carrier_ros_srv/RobotStatus.h"
#include <geodesy/utm.h>
#include <ros/rate.h>
#include <geographic_msgs/GeoPoint.h>
#include <vector>
#include <sensor_msgs/NavSatFix.h>
#include "robot_localization/SetDatum.h"
#include <sensor_msgs/Imu.h>

class CarrierInterface{
    public:
        CarrierInterface();
        double station_latitude = 308308;
        double station_longitude = 4130121;
        double current_latitude = 37;
        double current_longitude = 126;
        double target_distance = 10.0;
        double target_latitude, target_longitude;
        double update_latitude, update_longitude;

        bool isNavigating = false;
        bool isFirstStart = true;
        int currentGoalIndex = 0;
        int pausedGoalIndex = -1;

        std::vector<double> dest_que_x, dest_que_y;

        #define pi 3.14159265358979323846
    
    private:
        ros::NodeHandle nh;

        ros::ServiceServer robot_start_server_,
                           robot_stop_server_,
                           robot_recall_server_,
                           robot_emergency_server_;

        ros::ServiceClient robot_status_client_, datum_client;
        ros::Subscriber carrier_gps_sub_;
        ros::Subscriber imu_sub_;
        carrier_ros_srv::RobotStatus robot_srv;
        robot_localization::SetDatum datum;
        
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        boost::shared_ptr<MoveBaseClient> action_client_;

        bool startCallback(carrier_ros_srv::RobotStart::Request& req, carrier_ros_srv::RobotStart::Response& res);
        bool stopCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
        bool recallCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
        bool emergencyCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data);
        void carrierGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& data);
        void navigateToGoal(double x, double y);
        void updatePath();
        double calc_distance(double lat1, double long1, double lat2, double long2);
        double calc_bearing(double lat1, double long1, double lat2, double long2);
        double deg2rad(double deg);
        double rad2deg(double deg);
        void Updatedatum();
};

CarrierInterface::CarrierInterface(){
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    robot_start_server_ = nh.advertiseService("/robot/outdoor/start", &CarrierInterface::startCallback, this);
    robot_stop_server_ = nh.advertiseService("/robot/outdoor/stop", &CarrierInterface::stopCallback, this);
    robot_recall_server_ = nh.advertiseService("/robot/outdoor/recall", &CarrierInterface::recallCallback, this);
    robot_emergency_server_ = nh.advertiseService("/robot/outdoor/emergency", &CarrierInterface::emergencyCallback, this);

    robot_status_client_ = nh.serviceClient<carrier_ros_srv::RobotStatus>("/robot/status");
    datum_client = nh.serviceClient<robot_localization::SetDatum>("/datum");
    carrier_gps_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("/carrier_ros_gps/fix", 10, &CarrierInterface::carrierGPSCallback, this);
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, &CarrierInterface::imuCallback, this);

    robot_srv.request.status = 0;
    action_client_.reset(new MoveBaseClient("move_base", true));

    ROS_INFO("Waiting for the move_base action server_ to come up...");
    ROS_INFO("The move_base action server_ is ready.");
}

void CarrierInterface::carrierGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& data) {
    current_latitude = data->latitude;
    current_longitude = data->longitude;
}

void CarrierInterface::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data){
    datum.request.geo_pose.orientation= imu_data->orientation;
}

void CarrierInterface::navigateToGoal(double x, double y){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;
    action_client_->sendGoal(goal);
    action_client_->waitForResult();

    if(action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Navigation succeeded!");
    else
      ROS_WARN("Navigation failed.");
}

void CarrierInterface::Updatedatum()
{
    datum.request.geo_pose.position.latitude = 37.298834;
    datum.request.geo_pose.position.longitude = 126.837294;
    datum.request.geo_pose.position.altitude = 0.0;
    datum_client.call(datum);
}

bool CarrierInterface::startCallback(carrier_ros_srv::RobotStart::Request& req, carrier_ros_srv::RobotStart::Response& res){
    Updatedatum();
    ROS_INFO("Update datum .. ");
    // action_client_->waitForServer();
    ROS_INFO("-> wait for server ");
    for(size_t i = 0; i < req.latitude.size(); i++){
        double latitude = req.latitude[i];
        double longitude = req.longitude[i];
        std::cout << "lati " << latitude << "lon" << longitude << std::endl;
        geodesy::UTMPoint utm_point;
        geographic_msgs::GeoPoint geo_point; 
        geo_point.latitude = latitude;
        geo_point.longitude = longitude;
        geodesy::fromMsg(geo_point, utm_point, true, 's', 52);
        std::cout << "geo_lati " << geo_point.latitude << "geo_lon" << geo_point.longitude << std::endl;
        dest_que_x.push_back(utm_point.easting);
        dest_que_y.push_back(utm_point.northing);
        std::cout << "dest_x " << utm_point.easting << "dest_y" << utm_point.northing << std::endl;
        std::cout << "dest_x q " << dest_que_x[0] << "dest_y q " << dest_que_y[0] << std::endl;
    }

    if(!isFirstStart){
        ROS_INFO("In ! is FIRST START = NOT FIRST");
        if(isNavigating && (currentGoalIndex < dest_que_x.size() - 1)){
            dest_que_x.erase(dest_que_x.begin() + currentGoalIndex + 1, dest_que_x.end());
            dest_que_y.erase(dest_que_y.begin() + currentGoalIndex + 1, dest_que_y.end());
        }
    }
    else{
        ROS_INFO("FIRST START , ELSE");
        for(size_t i = 0; i < req.latitude.size(); i++){
            std::cout << "------------------------------" << std::endl;
            double latitude = req.latitude[i];
            double longitude = req.longitude[i];
            geodesy::UTMPoint utm_point;
            geographic_msgs::GeoPoint geo_point; 
            geo_point.latitude = latitude;
            geo_point.longitude = longitude;
            geodesy::fromMsg(geo_point, utm_point, true, 's', 52);
            dest_que_x.push_back(utm_point.easting);
            dest_que_y.push_back(utm_point.northing);
        }
    }
    isNavigating = true;
    isFirstStart = false;

    if(pausedGoalIndex != -1){
        std::cout << "----------------11--------------" << std::endl;
        for(size_t i = pausedGoalIndex; i < dest_que_x.size(); i++){
            // navigateToGoal(dest_que_x[i], dest_que_y[i]);
            std::cout << "----------------22--------------" << std::endl;
            ros::Rate rate(0.2);
            while(ros::ok()){
                updatePath();
                ros::spinOnce();
                rate.sleep();
            }
            if(i == dest_que_x.size() - 1){
               ROS_INFO("Reached the last waypoint");
            }
        }
    }
    else{
        std::cout << "--------------------33----------" << std::endl;
        // navigateToGoal(station_latitude, station_longitude);
        for(size_t i = currentGoalIndex + 1; i < dest_que_x.size(); i++){
            // navigateToGoal(dest_que_x[i], dest_que_y[i]);
            ros::Rate rate(0.2);
            while(ros::ok()){
                std::cout << "-------------7676-----------------" << std::endl;
                updatePath();
                ros::spinOnce();
                rate.sleep();
            }
            if(i == dest_que_x.size() - 1){
                ROS_INFO("Reached the last waypoint");
            }
        }
    }
    // navigateToGoal(dest_que_x[0], dest_que_y[0]);
    ros::Rate rate(0.2);
    while(ros::ok()){
        std::cout << "---------------44---------------" << std::endl;
        updatePath();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Reached the destination.");
    res.success = true;
    return true;
}

bool CarrierInterface::stopCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
    pausedGoalIndex = currentGoalIndex;
    action_client_->waitForResult();
    isNavigating = false; 
    robot_srv.request.status = 1; 
    robot_status_client_.call(robot_srv); // carrier stop -> status update

    res.message = "Navigaion Stopped";
    return true;
}

bool CarrierInterface::recallCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){

    action_client_->cancelAllGoals();
    robot_srv.request.status = 2;
    robot_status_client_.call(robot_srv);

    navigateToGoal(station_latitude, station_longitude);

    res.success = true;
    res.message = "Recall handled, Repathing to station";
    return true;
}

bool CarrierInterface::emergencyCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
    action_client_->cancelAllGoals(); 
    robot_srv.request.status = 2;
    robot_status_client_.call(robot_srv);

    navigateToGoal(station_latitude, station_longitude);

    res.success = true;
    res.message = "Emergency handled, Return to station";
    return true;
}

double CarrierInterface::calc_distance(double lat1, double long1, double lat2, double long2){ 
  double theta, dist = 0;
  theta = long1 - long2;
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515 * 1.609344 * 1000;
  return dist;
  }

double CarrierInterface::calc_bearing(double lat1, double long1, double lat2, double long2){
    double x, y, theta;
    y = sin(deg2rad(long2 - long1)) * cos(deg2rad(lat2));
    x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) - sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(long2 - long1));
    theta = rad2deg(atan2(y, x));

    theta += 360;
    if (theta >= 360)   theta -= 360;

    return theta;
}

double CarrierInterface::deg2rad(double deg) {
    return (deg * pi / 180);
}

double CarrierInterface::rad2deg(double rad) {
    return (rad * 180 / pi);
}

void CarrierInterface::updatePath(){
    std::cout << "-----------------898-------------" << std::endl;
    std::cout << currentGoalIndex << std::endl;

    std::cout << dest_que_x[currentGoalIndex] << "ㅇㄹ" << dest_que_y[currentGoalIndex] << std::endl;

    double distance = calc_distance(current_latitude, current_longitude, dest_que_x[currentGoalIndex], dest_que_y[currentGoalIndex]);
    
    std::cout << "---------------121212---------------" << std::endl;
    double bearing = calc_bearing(current_latitude, current_longitude, dest_que_x[currentGoalIndex], dest_que_y[currentGoalIndex]);

    geographic_msgs::GeoPoint target_geo_point;
    geodesy::UTMPoint target_utm_point;
    geodesy::UTMPoint current_utm_point;
    geodesy::fromMsg(target_geo_point, target_utm_point);

    if(distance > target_distance){
        std::cout << "---------------03030---------------" << std::endl;
        double target_ratio = target_distance / distance;
        std::cout << "------------------111------------" << std::endl;
        target_utm_point.easting = current_utm_point.easting + (target_ratio * (dest_que_x[currentGoalIndex] - current_utm_point.easting));
        std::cout << "-------------2222-----------------" << std::endl;
        target_utm_point.northing = current_utm_point.northing + (target_ratio * (dest_que_y[currentGoalIndex] - current_utm_point.northing));
        update_latitude = target_utm_point.easting;
        update_longitude = target_utm_point.northing;
    } 
    else{
        // station error
        std::cout << "--------------44----------------" << std::endl;
        target_utm_point.easting = dest_que_x[currentGoalIndex];
        target_utm_point.northing = dest_que_y[currentGoalIndex];
        update_latitude = target_utm_point.easting;
        update_longitude = target_utm_point.northing;
    }
    std::cout << "---------------nannanananana---------------" << std::endl;
    navigateToGoal(update_latitude, update_longitude);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation_controller");
    ROS_INFO("Start navigation_controller");
    CarrierInterface carrierinterface;
    ros::spin();
    return 0;
}