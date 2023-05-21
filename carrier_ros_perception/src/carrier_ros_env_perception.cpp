#include "carrier_ros_perception/carrier_ros_env_perception.h"




Camera_detection::Camera_detection()
  :nh(""),
  priv_nh("~"),
  queue_size_ (100),
  base_frame (""),
  input_depth_topic1(""), input_depth_topic2(""), input_depth_topic3(""),
  tf_listener(tf_buffer),
  transformed_cloud_1(new PointCloud),
  transformed_cloud_2(new PointCloud),
  transformed_cloud_3(new PointCloud),
  combined_cloud(new PointCloud),
  projection_cloud(new PointCloud),
  difference_cloud(new PointCloud),
  diff_filter_cloud(new PointCloud)
{
  base_frame = priv_nh.param<std::string>("base_frame", "base_link");
  input_depth_topic1 = priv_nh.param<std::string>("input_depth_topic1", "sensor1");
  input_depth_topic2 = priv_nh.param<std::string>("input_depth_topic2", "sensor2");
  input_depth_topic3 = priv_nh.param<std::string>("input_depth_topic3", "sensor3");
  downthershold = priv_nh.param<float>("downthershold", -0.3f);
  upthershold = priv_nh.param<float>("upthershold", 0.3f);
  wheelthershold = priv_nh.param<float>("wheelthershold", 0.05f);

  kd_tree_search_point = priv_nh.param<int>("kd_tree_search_point", 8);
  

  timer = nh.createTimer(ros::Duration(0.10), &Camera_detection::TimerPclIntegrate, this); //50hz

  // ROS_INFO ("floor_filter and obstacle detect (%s), (%s) and (%s) to PointCloud2 (%s).", nh.resolveName (input_depth_topic1).c_str (), 
  //                                                                                        nh.resolveName (input_depth_topic2).c_str (),
  //                                                                                        nh.resolveName (input_depth_topic3).c_str (),
  //                                                                                        nh.resolveName (output_topic).c_str ());
}

Camera_detection::~Camera_detection(){}

void Camera_detection::Camera1callback(const sensor_msgs::PointCloud2ConstPtr &pointcloud2msg)
{
    // Convert pointcloud2 to pcl pointcloud
    PointCloud::Ptr pcl_cloud(new PointCloud);
    pcl::fromROSMsg(*pointcloud2msg, *pcl_cloud);
    Transformation_frame(pointcloud2msg->header.frame_id, pcl_cloud, transformed_cloud_1);

}

void Camera_detection::Camera2callback(const sensor_msgs::PointCloud2ConstPtr &pointcloud2msg)
{
    // Convert pointcloud2 to pcl pointcloud
    PointCloud::Ptr pcl_cloud(new PointCloud);
    pcl::fromROSMsg(*pointcloud2msg, *pcl_cloud);
    Transformation_frame(pointcloud2msg->header.frame_id, pcl_cloud, transformed_cloud_2);

}

void Camera_detection::Camera3callback(const sensor_msgs::PointCloud2ConstPtr &pointcloud2msg)
{
    // Convert pointcloud2 to pcl pointcloud
    PointCloud::Ptr pcl_cloud(new PointCloud);
    pcl::fromROSMsg(*pointcloud2msg, *pcl_cloud);
    Transformation_frame(pointcloud2msg->header.frame_id, pcl_cloud, transformed_cloud_3);

}

void Camera_detection::Transformation_frame(const std::string frame_id, const PointCloud::Ptr input, PointCloud::Ptr &output)
{
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer.lookupTransform(base_frame, frame_id, ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    pcl_ros::transformPointCloud(*input, *output, transform.transform);
}



void Camera_detection::TimerPclIntegrate(const ros::TimerEvent& event)
{
  *combined_cloud = *transformed_cloud_1 + *transformed_cloud_2 + *transformed_cloud_3;
  //pcl2_data1,pcl2_data2,pcl2_data3data를 가지고 합쳐서 내보내야함.



  sensor_msgs::PointCloud2 pub1,pub2,pub3;
  if(!(combined_cloud->empty()))
  {
  Camera_detection::RestrictedEnv(combined_cloud,
                                 projection_cloud,
                                 difference_cloud,
                                 diff_filter_cloud);
  }
  // ROS_INFO("hi");
  // pclPublish(projection_cloud, base_frame, "projection_cloud");
  // pclPublish(difference_cloud, base_frame, "difference_cloud");
  // pclPublish(diff_filter_cloud, base_frame, "diff_filter_cloud");

  pcl::toROSMsg(*projection_cloud, pub1);
  pub1.header.frame_id = base_frame;
  pub1.header.stamp = ros::Time::now();
  // projection_pub.publish(pub1);
  
  pcl::toROSMsg(*difference_cloud, pub2);
  pub2.header.frame_id = base_frame;
  pub2.header.stamp = ros::Time::now();
  // difference_pub.publish(pub2);

  pcl::toROSMsg(*diff_filter_cloud, pub3);
  pub3.header.frame_id = base_frame;
  pub3.header.stamp = ros::Time::now();
  diff_filter_pub.publish(pub3);
}

/* filter_input is input , otherwise is output*/
void Camera_detection::RestrictedEnv(const PointCloud::Ptr filter_input, 
                                     const PointCloud::Ptr& projection_cloud,
                                     const PointCloud::Ptr& difference_cloud,
                                     const PointCloud::Ptr& diff_filter_cloud)
{
    projection_cloud->clear();
    difference_cloud->clear();
    diff_filter_cloud->clear();
    //downsampling 
    {
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud(filter_input);
      vg.setLeafSize(0.030f, 0.030f, 0.030f); //unit(m)
      vg.filter(*filter_input);
    }
    // Filter x bound
    {
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud(filter_input);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(0, 5);
      pass.setFilterLimitsNegative(false);
      
      pass.filter(*filter_input);
    }   

    // Filter z bound
    {
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud(filter_input);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(downthershold, upthershold);
      pass.setFilterLimitsNegative(false);
      
      pass.filter(*filter_input);
    }    

    //projection
    {
      std::map<std::pair<float, float>, float> max_z_points;
      std::pair<float, float> xy;
      for (const auto& point : filter_input->points) {
        xy = std::make_pair(round(point.x*100)/100, round(point.y*100)/100);
        auto search1 = max_z_points.find(xy);
        //max_z_pints안에 해당 (xy)가 없거나,  기존의  값이 작으면 새로운 값보다 작으면
        if (search1 == max_z_points.end() || search1->second < point.z) {
          max_z_points[xy] = point.z;
          // std::cout << "point.x : " << point.x << "point.y : " << point.y << std::endl;
        }
      }
      // std::cout << "finish" << std::endl << std::endl;

      for (const auto& point : filter_input->points) {
        xy = std::make_pair(round(point.x*100)/100, round(point.y*100)/100);
        //max_z_points안에 해당 (xy)가 있고 가장 큰 z 일경우
        if ((max_z_points[xy] - point.z)<0.00001) {
          

          projection_cloud->push_back(point);
        }
      }
    // ROS_INFO("MP is %ld, fi is %ld, pc is %ld", max_z_points.size(),
    //                                             filter_input->size(),
    //                                             projection_cloud->size());
    }

    pcl::copyPointCloud(*projection_cloud, *difference_cloud);
    

    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);

    std::vector<float> z_diff;
    // difference_cloud 데이터를 kdtree에 삽입합니다.

    PointCloud::Ptr z0cloud(new PointCloud);
    PointCloud::Ptr copy(new PointCloud); 
    pcl::copyPointCloud(*projection_cloud, *z0cloud);
    pcl::copyPointCloud(*projection_cloud, *copy);
    for (size_t i = 0; i < z0cloud->points.size(); ++i) 
    {
      z0cloud->points[i].z = 0.0f;
    }
    kdtree->setInputCloud(z0cloud);

    float max_z = 0.0f;
    //vector를 크기를 resize해준다.
    std::vector<int> search_Point_Idx(kd_tree_search_point);
    std::vector<float> search_Point_Distance(kd_tree_search_point);

    for (int i = 0; i < copy->size(); ++i) {
      pcl::PointXYZRGB target_Point = z0cloud->points[i];

      /*target_Point 근처의 k개의 point를 찾아 search_Point_Idx 안에 indexing한다. */
      if (kdtree->nearestKSearch(target_Point, kd_tree_search_point, search_Point_Idx, search_Point_Distance) > 0) 
      {
        for (int j = 0; j < search_Point_Idx.size(); ++j) 
        {
          pcl::PointXYZRGB pt = copy->points[search_Point_Idx[j]];          
          z_diff.push_back(abs(pt.z-copy->points[i].z));
        }
        max_z = *std::max_element(z_diff.begin(),z_diff.end());
        z_diff.clear();
      }
      difference_cloud->points[i].z = max_z;
      // ROS_INFO("z : %lf",max_z);
    }

    {
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud(difference_cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(wheelthershold, 10);
      pass.setFilterLimitsNegative(false);
      
      pass.filter(*diff_filter_cloud);
    }






    // ROS_INFO("MP is %ld, FI is %ld, FO is %ld", max_z_points.size(),
    //                                             filter_input->size(),
    //                                             projection_cloud->size());


}

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr Camera_detection::Projection(const PointCloud::Ptr projection_input)
// {


// } 

void Camera_detection::pclPublish(const PointCloud::Ptr pcl_publish, 
                                  const std::string frame_id, 
                                  std::string topic_name)
{
  //
}



void Camera_detection::initPublisher()
{
  projection_pub = nh.advertise<sensor_msgs::PointCloud2>("projection_cloud", 100);
  difference_pub = nh.advertise<sensor_msgs::PointCloud2>("difference_cloud", 100);
  diff_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("diff_filter_cloud", 100);
}

void Camera_detection::initSubscriber()
{
  camera1_sub = nh.subscribe(input_depth_topic1, 100, &Camera_detection::Camera1callback, this);
  camera2_sub = nh.subscribe(input_depth_topic2, 100, &Camera_detection::Camera2callback, this);
  camera3_sub = nh.subscribe(input_depth_topic3, 100, &Camera_detection::Camera3callback, this);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "carrier_ros_obstacle_detect");
  ROS_INFO("start carrier_ros_obstacle_detect node");
  Camera_detection Camera_detection;
  Camera_detection.initPublisher();
  Camera_detection.initSubscriber();
  ros::spin();
  return (0);
}
