#include "carrier_ros_perception/test.h"

Pcl_to_pcl2::Pcl_to_pcl2()
  :nh(""),
  priv_nh("~"),
  queue_size_ (100), 
  pcl_in_ ("/pcl_in_1"),
  pcl2_out_ ("/pcl2_out")
  timer = nh.createTimer(ros::Duration(0.02), &Pcl_to_pcl2::TimerPclIntegrate, this); //50hz
{

  // priv_nh_.getParam("/* param_name */", /* param_name */);
  // nh.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
  sensor_number = nh.param<int>("sensor_number", 1);
  // std::string yaml_file = nh.param<std::string>("dynamixel_info", "");
  ROS_INFO ("PointCloudConverter initialized to transform from PointCloud (%s) to PointCloud2 (%s).", nh.resolveName (pcl_in_).c_str (), nh.resolveName (pcl2_out_).c_str ());
}


inline std::string Pcl_to_pcl2::getFieldsList (const sensor_msgs::PointCloud &points)
{
  std::string result = "x y z";
  for (size_t i = 0; i < points.channels.size (); i++)
    result = result + " " + points.channels[i].name;
  return (result);
}


void Pcl_to_pcl2::Sensor1Callback (const sensor_msgs::PointCloudConstPtr &msg)
{
  if (pcl2_pub.getNumSubscribers () <= 0)
  {
    //ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points on %s, but no subscribers.", (int)msg->points.size (), nh.resolveName (pcl_in_).c_str ());
    return;
  }

  ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points (%s) on %s.", (int)msg->points.size (), getFieldsList (*msg).c_str (), nh.resolveName (pcl_in_).c_str ());

  // Convert to the old point cloud format
  if (!sensor_msgs::convertPointCloudToPointCloud2 (*msg, pcl2_data1))
  {
    ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
    return;
  }
  ROS_DEBUG ("[point_cloud_converter] Publishing a PointCloud2 with %d points on %s.", pcl2_data1.height * pcl2_data1.width, nh.resolveName (pcl2_out_).c_str ());
  // pcl2_pub.publish (pcl2_data1);
}

void Pcl_to_pcl2::initPublisher()
{
  pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl2_out_, 100);
}

void Pcl_to_pcl2::initSubscriber()
{
  if(sensor_number)
  sensor1_sub = priv_nh.subscribe("pointcloud1", 100, &Pcl_to_pcl2::Sensor1Callback, this);
  sensor2_sub = priv_nh.subscribe("pointcloud2", 100, &Pcl_to_pcl2::Sensor2Callback, this);
  sensor3_sub = priv_nh.subscribe("pointcloud3", 100, &Pcl_to_pcl2::Sensor3Callback, this);
}




void Pcl_to_pcl2::Sensor2Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg)
{
  //
}
void Pcl_to_pcl2::Sensor3Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg)
{
  //
}

void Pcl_to_pcl2::TimerPclIntegrate(const ros::TimerEvent& event)
{
  //pcl2_data1,pcl2_data2,pcl2_data3data를 가지고 합쳐서 내보내야함.
  pcl2_pub.publish(pcl2_integrate);
}

int main(int argc, char** argv) 
{

  return (0);
}
