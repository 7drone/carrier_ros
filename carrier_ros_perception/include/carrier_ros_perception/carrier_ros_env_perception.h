#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

class Camera_detection
{
  private:
    //ROS NodeHandle
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh;

    //ROS parameter
    int         kd_tree_search_point;
    std::string base_frame,
                input_depth_topic1, input_depth_topic2, input_depth_topic3;
    float       downthershold, upthershold, wheelthershold;
    
    // ROS Topic Publisher
    ros::Publisher projection_pub,
                   difference_pub,
                   diff_filter_pub;

    // ROS Topic Subscriber
    ros::Subscriber camera1_sub,
                    camera2_sub,
                    camera3_sub;
    // ROS Service Server

    // ROS Service Client

    // ROS Timer
    ros::Timer timer;

    // ROS TF2
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    int queue_size_;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    PointCloud::Ptr transformed_cloud_1, 
                    transformed_cloud_2, 
                    transformed_cloud_3, 
                    combined_cloud,
                    projection_cloud,
                    difference_cloud,
                    diff_filter_cloud;



  public:
    Camera_detection();
    ~Camera_detection();


    
    void Camera1callback(const sensor_msgs::PointCloud2ConstPtr &pointcloudmsg);
    void Camera2callback(const sensor_msgs::PointCloud2ConstPtr &pointcloudmsg);
    void Camera3callback(const sensor_msgs::PointCloud2ConstPtr &pointcloudmsg); 

    void Transformation_frame(const std::string frame_id, const PointCloud::Ptr input, PointCloud::Ptr &output);

    void TimerPclIntegrate(const ros::TimerEvent& event);
    
    void RestrictedEnv(const PointCloud::Ptr filter_input, 
                                                         const PointCloud::Ptr& projection_cloud,
                                                         const PointCloud::Ptr& difference_cloud,
                                                         const PointCloud::Ptr& diff_filter_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Projection(const PointCloud::Ptr projection_input); 
    void pclPublish(const PointCloud::Ptr pcl_publish, const std::string frame_id, std::string topic_name);

    void initPublisher(void);
    void initSubscriber(void);

};


