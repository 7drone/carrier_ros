#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
class ImuHeading
{
public:
  ImuHeading() : nh_(""), private_nh_("~")
  {
    imu_sub_ = nh_.subscribe("imu/heading_data", 10, &ImuHeading::imuCallback, this);
    magnetic_sub_ = nh_.subscribe("imu/magnetic", 10, &ImuHeading::magneticCallback, this);

    timer_ = nh_.createTimer(ros::Duration(0.02), &ImuHeading::timerCallback, this);
  }

private:
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber magnetic_sub_;
  ros::Timer timer_;
  double imu_roll,imu_pitch,imu_yaw;

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(msg->orientation, quaternion);

    tf::Matrix3x3 matrix(quaternion);
    
    matrix.getRPY(imu_roll, imu_pitch, imu_yaw);
    ROS_INFO("%f", imu_yaw*180/M_PI);
  }

  void magneticCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
  {
    double north_heading;
    north_heading=atan2(msg->magnetic_field.y, msg->magnetic_field.x);
    ROS_INFO("%f", north_heading*180/M_PI);
    // ...
  }

  void timerCallback(const ros::TimerEvent& event)
  {
    // Publish north direction message
    // ...
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_node");
  ImuHeading node;
  ros::spin();

  return 0;
}
