
#include "carrier_ros_perception/carrier_ros_floor_detection.h"


Floor_Detection::Floor_Detection()
  :nh(""),
  priv_nh("~"),
  input_cloud(""),
  cloud_input(new pcl::PointCloud<pcl::PointXYZ>),
  finish_downsample(new pcl::PointCloud<pcl::PointXYZ>),
  finish_clustering(new pcl::PointCloud<pcl::PointXYZ>),
  finish_segment(new pcl::PointCloud<pcl::PointXYZ>)
{
  input_cloud = nh.param<std::string>("input_cloud", "camera/depth/color/points");
  //param setting
  /* param_type */ /* param_name */
  // priv_nh_.getParam("/* param_name */", /* param_name */);
  // nh.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
  // use_moveit_ = priv_node_handle_.param<bool>("use_moveit", false);
  // std::string yaml_file = nh.param<std::string>("dynamixel_info", "");
}

Floor_Detection::~Floor_Detection(){}



pcl::PointCloud<pcl::PointXYZ>::Ptr Floor_Detection::Downsampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsample_input)
{
  vg.setInputCloud(downsample_input);   //data 넣기
  vg.setLeafSize(0.03f, 0.03f, 0.03f);  //point downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*cloud_downsampled);        //downsample data 넣기
  return cloud_downsampled;
}

//NormalEstimation
//Euclidean cluster extraction
//biggest cluster extraction
pcl::PointCloud<pcl::PointXYZ>::Ptr Floor_Detection::Point_Cloud_Clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& clustering_input)
{
  ne.setInputCloud(clustering_input);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);

  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  ne.compute (*normals);

  //Euclidean Cluster Extraction을 수행하서 cluster extration을 한다.
  //쓰이는 이유는 속도 증가시키기 위함.
  ece.setClusterTolerance (0.03);
  ece.setMinClusterSize (100);
  ece.setMaxClusterSize (25000);
  ece.setSearchMethod (tree);
  ece.setInputCloud (clustering_input);
  std::vector<pcl::PointIndices> cluster_indices;
  ece.extract(cluster_indices); // cluster_indices에는 

  //The biggest cluster_index detection
  float max_cluster_size = 0.0f;
  int max_cluster_index = -1;
  for (int i = 0; i < cluster_indices.size (); i++)
  {
    if (cluster_indices[i].indices.size () > max_cluster_size)
    {
      max_cluster_size = cluster_indices[i].indices.size ();
      max_cluster_index = i;
    }
  }
  //The biggest cluster_index extraction
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (clustering_input);
  extract.setIndices (boost::make_shared<const pcl::PointIndices> (cluster_indices[max_cluster_index]));

  extract.setNegative (false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor (new pcl::PointCloud<pcl::PointXYZ>);
  extract.filter (*cloud_floor);


  //floor index
  pcl::IndicesPtr floor_inliers(new std::vector<int>(cluster_indices[max_cluster_index].indices));
  return cloud_floor;
}

//모델 추정 후 노이즈 제거
pcl::PointCloud<pcl::PointXYZ>::Ptr Floor_Detection::Floor_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& segmentation_input)
{
  //바닥 평면 모델 추정 - RANSAC 알고리즘
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
  
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.1);
  seg.setInputCloud(segmentation_input);
  seg.segment(*inliers, *coefficients);

  //노이즈나 이상 포인트 제거
  sor.setInputCloud(segmentation_input);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*floor_filtered);

  pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients);
  floor_coefficients->values.resize(4);
  floor_coefficients->values[0] = coefficients->values[0];
  floor_coefficients->values[1] = coefficients->values[1];
  floor_coefficients->values[2] = coefficients->values[2];
  floor_coefficients->values[3] = coefficients->values[3];
  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_plane(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector4f floor_coefficients_eigen(floor_coefficients->values.data());
  for (const auto& point : *segmentation_input)
  {
    // 포인트를 평면 위에 투영
    pcl::PointXYZ projected_point;
    float distance = pcl::pointToPlaneDistance(point, floor_coefficients_eigen);
    pcl::projectPoint(point, floor_coefficients_eigen, projected_point);
    floor_plane->push_back(projected_point);
  }
  return floor_plane;
}

void Floor_Detection::initPublisher()
{
  floor_pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>("downsampled_point_cloud", 10);
}

void Floor_Detection::initSubscriber()
{
  camera_pcl2_sub = nh.subscribe(input_cloud, 100, &Floor_Detection::pcl2Callback, this);
}

void Floor_Detection::pcl2Callback(const sensor_msgs::PointCloud2ConstPtr &cloudmsg)
{
  pcl::fromROSMsg(*cloudmsg, *cloud_input);
  finish_downsample = Floor_Detection::Downsampling(cloud_input);
  finish_clustering = Floor_Detection::Point_Cloud_Clustering(finish_downsample);
  finish_segment = Floor_Detection::Floor_segmentation(finish_clustering);
  sensor_msgs::PointCloud2 bottom_cloudmsg;
  pcl::toROSMsg(*finish_segment, bottom_cloudmsg);
  bottom_cloudmsg.header = cloudmsg->header;  // Copy the header from the original message
  
  // Publish the downsampled point cloud message
  floor_pcl2_pub.publish(bottom_cloudmsg);


}



int main(int argc, char** argv) 
{
  ros::init(argc, argv, "point_cloud_subscriber");
  ROS_INFO("start init point_cloud_subscriber node");
  Floor_Detection floor_cluster;
  floor_cluster.initPublisher();
  floor_cluster.initSubscriber();
  
  ros::spin();
  return (0);
}
