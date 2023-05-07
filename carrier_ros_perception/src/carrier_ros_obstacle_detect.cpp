#include "carrier_ros_perception/carrier_ros_obstacle_detect.h"




Camera_detection::Camera_detection()
  :nh(""),
  priv_nh("~"),
  queue_size_ (100),
  base_frame (""),
  input_depth_topic1(""), input_depth_topic2(""), input_depth_topic3(""),
  output_topic(""),
  tf_listener(tf_buffer),
  transformed_cloud_1(new PointCloud),
  transformed_cloud_2(new PointCloud),
  transformed_cloud_3(new PointCloud),
  combined_cloud(new PointCloud)
{
  base_frame = priv_nh.param<std::string>("base_frame", "base_link");
  input_depth_topic1 = priv_nh.param<std::string>("input_depth_topic1", "sensor1");
  input_depth_topic2 = priv_nh.param<std::string>("input_depth_topic2", "sensor2");
  input_depth_topic3 = priv_nh.param<std::string>("input_depth_topic3", "sensor3");
  output_topic = priv_nh.param<std::string>("output_topic", "point_cloud2");
  downthershold = priv_nh.param<float>("downthershold", -0.3f);
  upthershold = priv_nh.param<float>("upthershold", 0.3f);

  timer = nh.createTimer(ros::Duration(0.05), &Camera_detection::TimerPclIntegrate, this); //50hz

  ROS_INFO ("floor_filter and obstacle detect (%s), (%s) and (%s) to PointCloud2 (%s).", nh.resolveName (input_depth_topic1).c_str (), 
                                                                                         nh.resolveName (input_depth_topic2).c_str (),
                                                                                         nh.resolveName (input_depth_topic3).c_str (),
                                                                                         nh.resolveName (output_topic).c_str ());
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



  sensor_msgs::PointCloud2 obstacle_detect;
  pcl::toROSMsg(*Camera_detection::Filter_floor(combined_cloud), obstacle_detect);

  obstacle_detect.header.frame_id = base_frame;
  obstacle_detect.header.stamp = ros::Time::now();

  obstacle_pub.publish(obstacle_detect);
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Camera_detection::Filter_floor(const PointCloud::Ptr filter_input)
{
    //downsampling 
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(filter_input);
    vg.setLeafSize(0.030f, 0.030f, 0.030f); //unit(m)
    vg.filter(*filter_input);

    // Filter bound
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(filter_input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(downthershold, upthershold);
    pass.setFilterLimitsNegative(false);
    
    pass.filter(*filter_input);
    
    
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

    // int grid[300][300];
    // for(auto iter=max_z_points.begin(); iter!=max_z_points.end(); iter++)
    // { 
    //   //0.03
    //   if(grid[int(ceil(iter->first.first*100/3))][int(ceil(iter->first.second*100/3))] < iter->second)
    //     grid[int(ceil(iter->first.first*100/3))][int(ceil(iter->first.second*100/3))] = iter->second;

    // }


    PointCloud::Ptr filtered_cloud(new PointCloud);
    for (const auto& point : filter_input->points) {
      xy = std::make_pair(round(point.x*100)/100, round(point.y*100)/100);
      //max_z_points안에 해당 (xy)가 있고 가장 큰 z 일경우
      if ((max_z_points[xy] - point.z)<0.00001) {
        

        filtered_cloud->push_back(point);
      }
    }





    std::map<std::pair<float, float>, float> max_z_changes;
    std::pair<float, float> xy;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

    // cloud 데이터를 kdtree에 삽입합니다.
    kdtree.setInputCloud(cloud);

    int k = 8;
    //vector를 크기를 resize해준다. //주변의 8point를 사용할 예정
    std::vector<int> pointIdxNKNSearch(k);
    std::vector<float> pointNKNSquaredDistance(k);

    for (int i = 0; i < cloud->size(); ++i) {
      pcl::PointXYZRGB searchPoint = cloud->points[i];
      xy = std::make_pair(round(searchPoint.x*100)/100, round(searchPoint.y*100)/100);

      if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        float max_z = searchPoint.z;
        float min_z = searchPoint.z;
        for (int j = 0; j < pointIdxNKNSearch.size(); ++j) {
          pcl::PointXYZRGB pt = cloud->points[pointIdxNKNSearch[j]];
          
          float z_diff = 0;
          if()
          // x,y의 좌표가 같다면 
          if (xy == std::make_pair(round(pt.x*100)/100, round(pt.y*100)/100)) {
            if (pt.z > max_z) {
              max_z = pt.z;
            }
            if (pt.z < min_z) {
              min_z = pt.z;
            }
          }
        }
        max_z_changes[xy] = max_z - min_z;
      }
    }








    // ROS_INFO("MP is %ld, FI is %ld, FO is %ld", max_z_points.size(),
    //                                             filter_input->size(),
    //                                             filtered_cloud->size());





    return filtered_cloud;
}

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr Camera_detection::Projection(const PointCloud::Ptr projection_input)
// {


// } 



void Camera_detection::initPublisher()
{
  obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 100);
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
