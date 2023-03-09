
#include "carrier_ros_perception/test.h"



class PointCloudSubscriber {
public:
  PointCloudSubscriber() {
    // Initialize the ROS node
    ros::NodeHandle nh("");
    
    // Create the subscriber to the PointCloud2 topic9
    pc_sub_ = nh.subscribe("camera/depth/color/points", 10, &PointCloudSubscriber::cloudmsg2cloud, this);
    
    // Create the publisher for the downsampled point cloud topic
    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("downsampled_point_cloud", 1);
  }
  
  void cloudmsg2cloud(const sensor_msgs::PointCloud2ConstPtr& cloudmsg)
  {
    // Convert the PointCloud2 message to a PCL point cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloudmsg, *cloud_input); //cloudmsg(ROS)를 cloud_input(pcl)으로 변환
    
    // Create the voxel grid downsampling filter
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_input);
    sor.setLeafSize(0.03f, 0.03f, 0.03f); //downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_downsampled); //voxelGrid에서 downsample 된 것을 pcl output형태에 넣기.
    // Apply the downsampling filter to the input point cloud
    
    ////////////////////////////////////////////////////////////////
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_downsampled);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    
    ne.setRadiusSearch (0.03);
    ne.compute (*normals);

    // 3. Euclidean Cluster Extraction을 수행하여 클러스터를 추출합니다.
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.03);
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_downsampled);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);

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

    // 5. 선택된 바닥 클러스터를 추출합니다.
    pcl::ExtractIndices<pcl::PointXYZ> extract;  
    extract.setInputCloud (cloud_downsampled);
    extract.setIndices (boost::make_shared<const pcl::PointIndices> (cluster_indices[max_cluster_index]));
    
 
    extract.setNegative (false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_floor (new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter (*cloud_floor);

    pcl::IndicesPtr floor_inliers (new std::vector<int>(cluster_indices[max_cluster_index].indices));

    // 바닥 평면 모델 추정
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(cloud_floor);
    seg.segment(*inliers, *coefficients);
    // std::cout << "Floor coefficients: " << *coefficients << std::endl;

    // if (inliers->indices.size () == 0)
    // {
    //   PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    //   return (-1);
    // }

    // 바닥이 아닌 포인트들 제거
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud(cloud_floor);
    sor2.setMeanK(50);
    sor2.setStddevMulThresh(1.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor2.filter(*floor_filtered);
    std::cout << "Filtered floor cloud has " << floor_filtered->size() << " points." << std::endl;

    // 추정된 평면 모델을 포인트 클라우드로 변환
    pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients);
    floor_coefficients->values.resize(4);
    floor_coefficients->values[0] = coefficients->values[0];
    floor_coefficients->values[1] = coefficients->values[1];
    floor_coefficients->values[2] = coefficients->values[2];
    floor_coefficients->values[3] = coefficients->values[3];
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_plane(new pcl::PointCloud<pcl::PointXYZ>);
    // for (const auto& point : *cloud_floor)
    // {
    //   // 포인트를 평면 위에 투영
    //   pcl::PointXYZ projected_point;
    //   float distance = pcl::pointToPlaneDistance(point, *floor_coefficients);
    //   pcl::projectPoint(point, -distance, *floor_coefficients, projected_point);
    //   floor_plane->push_back(projected_point);
    // }

    // // 3D 바닥 모델링
    // pcl::OrganizedFastMesh<pcl::PointXYZ> ofm;
    // ofm.setInputCloud(floor_plane);
    // ofm.setMaxEdgeLength(0.1);
    // pcl::PolygonMesh floor_mesh;
    // ofm.reconstruct(floor_mesh);








    // Convert the downsampled PCL point cloud object back to a PointCloud2 message
    sensor_msgs::PointCloud2 bottom_cloudmsg;
    pcl::toROSMsg(*cloud_floor, bottom_cloudmsg);
    bottom_cloudmsg.header = cloudmsg->header;  // Copy the header from the original message
    
    // Publish the downsampled point cloud message
    pc_pub_.publish(bottom_cloudmsg);
  }

private:
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;
};

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "point_cloud_subscriber");
  PointCloudSubscriber pc_sub;
  
  ros::spin();
  return (0);
}
