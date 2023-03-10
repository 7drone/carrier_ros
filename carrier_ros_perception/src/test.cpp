#include "carrier_ros_perception/test.h"

void Pcl_to_pcl2::initPublisher()
{
  floor_pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("downsampled_point_cloud", 10);
}

void Pcl_to_pcl2::initSubscriber()
{
  if(lidar_numbeer)
  sensor1_sub = priv_nh.subscribe("pointcloud1", 100, &Pcl_to_pcl2::Sensor1Callback, this);
  sensor2_sub = priv_nh.subscribe("pointcloud2", 100, &Pcl_to_pcl2::Sensor2Callback, this)
  sensor3_sub = priv_nh.subscribe("pointcloud3", 100, &Pcl_to_pcl2::Sensor3Callback, this)
}

void Pcl_to_pcl2::Sensor1Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg)
{
  //
}

void Pcl_to_pcl2::Sensor2Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg)
{
  //
}
void Pcl_to_pcl2::Sensor3Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg)
{
  //
}



int main(int argc, char** argv) 
{

  return (0);
}
