#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

// Create a publisher for the combined pointcloud
// ros::Publisher publisher;

// // Create a PCL point cloud to hold the combined data
// pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// // Create a transform listener for the pointclouds
// tf2_ros::Buffer tf_buffer;
// tf2_ros::TransformListener tf_listener(tf_buffer);

// // Subscribe to incoming pointclouds from rider 1
// void subscribeToPointCloud1()
// {
//   ros::NodeHandle nh;
//   auto subscriber = nh.subscribe<sensor_msgs::PointCloud2>(
//     "/rider1/pointcloud", 1, [](const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
//     {

//       //pointcloud에 담긴내용을 cloud에 넣는다.
//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//       pcl::fromROSMsg(*pointcloud, *cloud);

//       // Transform the pointcloud into the desired frame of reference
//       geometry_msgs::TransformStamped transform;
//       try
//       {
//         transform = tf_buffer.lookupTransform("common_frame", pointcloud->header.frame_id, ros::Time(0));
//       }
//       catch (tf2::TransformException& ex)
//       {
//         ROS_WARN("%s", ex.what());
//         return;
//       }
//       //pointcloud, transform에 담긴 것을 변환해서 transformed_cloud에 넣는다.
//       sensor_msgs::PointCloud2 transformed_cloud;
//       tf2::doTransform(*pointcloud, transformed_cloud, transform);

//       // transformed_cloud에 담긴내용을 transformed에 넣는다.
//       pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
//       pcl::fromROSMsg(transformed_cloud, *transformed);

//       *combined_cloud += *transformed;

//       publishCombinedPointCloud();
//     }
//   );
// }

// // Subscribe to incoming pointclouds from rider 2
// void subscribeToPointCloud2()
// {
//   ros::NodeHandle nh;
//   auto subscriber = nh.subscribe<sensor_msgs::PointCloud2>(
//     "/rider2/pointcloud", 1, [](const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
//     {
//       // Convert the ROS pointcloud to a PCL pointcloud
//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//       pcl::fromROSMsg(*pointcloud, *cloud);

//       // Transform the pointcloud into the desired frame of reference
//       geometry_msgs::TransformStamped transform;
//       try
//       {
//         transform = tf_buffer.lookupTransform("common_frame", pointcloud->header.frame_id, ros::Time(0));
//       }
//       catch (tf2::TransformException& ex)
//       {
//         ROS_WARN("%s", ex.what());
//         return;
//       }
//       sensor_msgs::PointCloud2 transformed_cloud;
//       tf2::doTransform(*pointcloud, transformed_cloud, transform);

//       // Convert the transformed pointcloud to a PCL pointcloud
//       pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
//       pcl::fromROSMsg(transformed_cloud, *transformed);

//       // Add the new points to the combined pointcloud
//       *combined_cloud += *transformed;

//     // Publish the combined pointcloud
//       publishCombinedPointCloud();
//       }
//     );
// }

// void publishCombinedPointCloud()
// {
// // Convert the PCL pointcloud to a ROS pointcloud2
// sensor_msgs::PointCloud2 ros_cloud;
// pcl::toROSMsg(*combined_cloud, ros_cloud);

// // Set the header information for the ROS pointcloud2
// ros_cloud.header.frame_id = "common_frame";
// ros_cloud.header.stamp = ros::Time::now();

// // Publish the ROS pointcloud2
// publisher.publish(ros_cloud);
// }

int main(int argc, char** argv) 
{

  return (0);
}