#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

class PointCloudSubscriber {
public:
  PointCloudSubscriber() {
    // Initialize the ROS node
    ros::NodeHandle nh("~");
    
    // Create the subscriber to the PointCloud2 topic
    pc_sub_ = nh.subscribe("point_cloud_topic", 10, &PointCloudSubscriber::cloudmsg2cloud);
  }
  
  pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_dst;
    pcl::fromROSMsg(cloudmsg, cloud_dst);
    return cloud_dst;
  }

private:
  ros::Subscriber pc_sub_;
};

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "point_cloud_subscriber");
  PointCloudSubscriber pc_sub;


//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  
  // 1. 포인트 클라우드 데이터를 로드합니다.
//   pcl::io::loadPCDFile<pcl::PointXYZ> ("input_cloud.pcd", *cloud_input);

  // 2. Voxel Grid Filter를 생성합니다.
//   pcl::VoxelGrid<pcl::PointXYZ> sor;
//   sor.setInputCloud (cloud_input);
//   sor.setLeafSize (0.01f, 0.01f, 0.01f);

  // 3. 다운샘플링된 포인트 클라우드를 계산합니다.
//   sor.filter (*cloud_downsampled);
  
  // 4. 결과를 출력합니다.
//   pcl::io::savePCDFile<pcl::PointXYZ> ("downsampled_cloud.pcd", *cloud_downsampled);


  ros::spin();
  return (0);
}