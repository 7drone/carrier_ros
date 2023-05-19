#include "carrier_ros_perception/carrier_ros_sensor_fusion.h"

Pcl_to_pcl2::Pcl_to_pcl2()
  :nh(""),
  priv_nh("~"),
  queue_size_ (100),
  base_frame (""),
  input_sensor1_topic (""), input_sensor2_topic (""), input_sensor3_topic (""),
  output_topic (""),
  tf_listener(tf_buffer),
  transformed_cloud_1(new pcl::PointCloud<pcl::PointXYZ>),
  transformed_cloud_2(new pcl::PointCloud<pcl::PointXYZ>),
  transformed_cloud_3(new pcl::PointCloud<pcl::PointXYZ>),
  combined_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
  // priv_nh_.getParam("/* param_name */", /* param_name */);
  // nh.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
  base_frame = priv_nh.param<std::string>("base_frame", "base_link");
  input_sensor1_topic = priv_nh.param<std::string>("input_sensor1_topic", "sensor1");
  input_sensor2_topic = priv_nh.param<std::string>("input_sensor2_topic", "sensor2");
  input_sensor3_topic = priv_nh.param<std::string>("input_sensor3_topic", "sensor3");
  output_topic = priv_nh.param<std::string>("output_topic", "point_cloud2");

  timer = nh.createTimer(ros::Duration(0.02), &Pcl_to_pcl2::TimerPclIntegrate, this); //50hz

  ROS_INFO ("Transform and Integrate from PointCloud (%s), (%s) and (%s) to PointCloud2 (%s).", nh.resolveName (input_sensor1_topic).c_str (), 
                                                                                                nh.resolveName (input_sensor2_topic).c_str (),
                                                                                                nh.resolveName (input_sensor3_topic).c_str (),
                                                                                                nh.resolveName (output_topic).c_str ());
}

Pcl_to_pcl2::~Pcl_to_pcl2(){}

void Pcl_to_pcl2::Sensor1Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg)
{
  sensor_msgs::PointCloud2  pointcloud2msg;
  // Convert to the old point cloud format
  if (!sensor_msgs::convertPointCloudToPointCloud2 (*pointcloudmsg, pointcloud2msg))
  {
    ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
    return;
  }


  //pointcloud에 담긴내용을 cloud에 넣는다.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pointcloud2msg, *cloud);

  // Transform the pointcloud into the desired frame of reference
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer.lookupTransform(base_frame, pointcloud2msg.header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  //pointcloud, transform에 담긴 것을 변환해서 transformed_cloud에 넣는다
  pcl_ros::transformPointCloud(*cloud, *transformed_cloud_1, transform.transform);
}

void Pcl_to_pcl2::Sensor2Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg)
{
  sensor_msgs::PointCloud2  pointcloud2msg;
  if (!sensor_msgs::convertPointCloudToPointCloud2 (*pointcloudmsg, pointcloud2msg))
  {
    ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
    return;
  }


  //pointcloud에 담긴내용을 cloud에 넣는다.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pointcloud2msg, *cloud);

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer.lookupTransform(base_frame, pointcloud2msg.header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  //pointcloud, transform에 담긴 것을 변환해서 transformed_cloud에 넣는다
  pcl_ros::transformPointCloud(*cloud, *transformed_cloud_2, transform.transform);
}

void Pcl_to_pcl2::Sensor3Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg)
{
  sensor_msgs::PointCloud2  pointcloud2msg;
  // Convert to the old point cloud format
  if (!sensor_msgs::convertPointCloudToPointCloud2 (*pointcloudmsg, pointcloud2msg))
  {
    ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
    return;
  }

  

  //pointcloud에 담긴내용을 cloud에 넣는다.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pointcloud2msg, *cloud);
  // Transform the pointcloud into the desired frame of reference
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer.lookupTransform(base_frame, pointcloud2msg.header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  //pointcloud, transform에 담긴 것을 변환해서 transformed_cloud에 넣는다
  pcl_ros::transformPointCloud(*cloud, *transformed_cloud_3, transform.transform);
}

void Pcl_to_pcl2::initPublisher()
{
  pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 100);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud>("lidar_pointcloud", 100);
}

void Pcl_to_pcl2::initSubscriber()
{
  sensor1_sub = nh.subscribe(input_sensor1_topic, 100, &Pcl_to_pcl2::Sensor1Callback, this);
  sensor2_sub = nh.subscribe(input_sensor2_topic, 100, &Pcl_to_pcl2::Sensor2Callback, this);
  sensor3_sub = nh.subscribe(input_sensor3_topic, 100, &Pcl_to_pcl2::Sensor3Callback, this);
}

void Pcl_to_pcl2::TimerPclIntegrate(const ros::TimerEvent& event)
{
  *combined_cloud = *transformed_cloud_1 + *transformed_cloud_2 + *transformed_cloud_3;
  //pcl2_data1,pcl2_data2,pcl2_data3data를 가지고 합쳐서 내보내야함.
  sensor_msgs::PointCloud2 pcl2_integrate;
  sensor_msgs::PointCloud pcl1_integrate;
  pcl::toROSMsg(*combined_cloud, pcl2_integrate);
  pcl2_integrate.header.frame_id = base_frame;
  pcl2_integrate.header.stamp = ros::Time::now();
  if (!sensor_msgs::convertPointCloud2ToPointCloud (pcl2_integrate, pcl1_integrate))
  {
    ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
    return;
  }

  pcl_pub.publish(pcl1_integrate);
  pcl2_pub.publish(pcl2_integrate);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "carrier_ros_sensor_fusion");
  ROS_INFO("start carrier_ros_sensor_fusion node");
  Pcl_to_pcl2 pcl_to_pcl2;
  pcl_to_pcl2.initPublisher();
  pcl_to_pcl2.initSubscriber();

  ros::spin();
  return (0);
}
