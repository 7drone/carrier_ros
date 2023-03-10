#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h> // RANSAC 헤더파일
#include <pcl/sample_consensus/model_types.h> // RANSAC 헤더파일
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h> // RANSAC 헤더파일
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/point_cloud.h>

class Floor_Detection
{
  private:
    //ROS NodeHandle
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh;

    //ROS parameter

    // ROS Topic Publisher
    ros::Publisher floor_pcl2_pub;

    // ROS Topic Subscriber
    ros::Subscriber camera_pcl2_sub;

    // ROS Service Server

    // ROS Service Client
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, 
                                        finish_downsample, 
                                        finish_clustering, 
                                        finish_segment;
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

  public:
    Floor_Detection();
    ~Floor_Detection();

    pcl::PointCloud<pcl::PointXYZ>::Ptr Downsampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsample_input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Point_Cloud_Clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& clustering_input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Floor_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& segmentation_input);

    void initPublisher(void);
    void initSubscriber(void);

    void pcl2Callback(const sensor_msgs::PointCloud2ConstPtr &cloudmsg);

};



