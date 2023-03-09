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

class Floor_Detection{
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    public:
      Floor_Detection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_input);
      ~Floor_Detection();

      pcl::PointCloud<pcl::PointXYZ>::Ptr Downsampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsample_input);
      pcl::PointCloud<pcl::PointXYZ>::Ptr Point_Cloud_Clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& clustering_input);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& segmentation_input);
};


Floor_Detection::Floor_Detection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr finish_downsample, finish_clustering;
  finish_downsample = Floor_Detection::Downsampling(cloud_input);
  finish_clustering = Floor_Detection::Point_Cloud_Clustering(finish_downsample);

}

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


pcl::PointCloud<pcl::PointXYZ>::Ptr Floor_Detection::cluster_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& segmentation_input)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
  
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.1);
  seg.setInputCloud(segmentation_input);
  seg.segment(*inliers, *coefficients);


  sor.setInputCloud(segmentation_input);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr floor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*floor_filtered);
  return floor_filtered;
}