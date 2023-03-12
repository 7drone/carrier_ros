#include "carrier_ros_perception/test.h"

Pcl_to_pcl2::Pcl_to_pcl2()
  :nh(""),
  priv_nh("~"),
  queue_size_ (100),
  sensor_number (0),
  base_frame (""),
  input_sensor1_topic (""), input_sensor2_topic (""), input_sensor3_topic (""),
  output_topic (""),
  tf_listener(tf_buffer),
  combined_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
  // priv_nh_.getParam("/* param_name */", /* param_name */);
  // nh.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
  sensor_number = priv_nh.param<int>("sensor_number", 1);
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


inline std::string Pcl_to_pcl2::getFieldsList (const sensor_msgs::PointCloud &points)
{
  std::string result = "x y z";
  for (size_t i = 0; i < points.channels.size (); i++)
    result = result + " " + points.channels[i].name;
  return (result);
}


// void Pcl_to_pcl2::Sensor1Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg)
// {
//   sensor_msgs::PointCloud2ConstPtr  pointcloud2msg;
//   // Convert to the old point cloud format
//   if (!sensor_msgs::convertPointCloudToPointCloud2 (*pointcloudmsg, *pointcloud2msg))
//   {
//     ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
//     return;
//   }


//   //pointcloud에 담긴내용을 cloud에 넣는다.
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::fromROSMsg(*pointcloud2msg, *cloud);

//   // Transform the pointcloud into the desired frame of reference
//   geometry_msgs::TransformStamped transform;
//   try
//   {
//     transform = tf_buffer.lookupTransform(base_frame, pointcloud2msg->header.frame_id, ros::Time(0));
//   }
//   catch (tf2::TransformException& ex)
//   {
//     ROS_WARN("%s", ex.what());
//     return;
//   }
//   //pointcloud, transform에 담긴 것을 변환해서 transformed_cloud에 넣는다.
//   sensor_msgs::PointCloud2 transformed_cloud;
//   tf2::doTransform(*pointcloud2msg, transformed_cloud, transform);
  
//   // // transformed_cloud에 담긴내용을 transformed에 넣는다.
//   // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
//   // pcl::fromROSMsg(transformed_cloud, *transformed);

//   // *combined_cloud += *transformed;

//   // publishCombinedPointCloud();
// }

void Pcl_to_pcl2::initPublisher()
{
  pcl2_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 100);
}

void Pcl_to_pcl2::initSubscriber()
{
  if(sensor_number)
  sensor1_sub = priv_nh.subscribe("pointcloud1", 100, &Pcl_to_pcl2::Sensor1Callback, this);
  sensor2_sub = priv_nh.subscribe("pointcloud2", 100, &Pcl_to_pcl2::Sensor2Callback, this);
  sensor3_sub = priv_nh.subscribe("pointcloud3", 100, &Pcl_to_pcl2::Sensor3Callback, this);
}




void Pcl_to_pcl2::Sensor2Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg)
{
  //
}
void Pcl_to_pcl2::Sensor3Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg)
{
  //
}

void Pcl_to_pcl2::TimerPclIntegrate(const ros::TimerEvent& event)
{
  //pcl2_data1,pcl2_data2,pcl2_data3data를 가지고 합쳐서 내보내야함.
  sensor_msgs::PointCloud2 pcl2_integrate;
  pcl::toROSMsg(*combined_cloud, pcl2_integrate);

  pcl2_integrate.header.frame_id = "common_frame";
  pcl2_integrate.header.stamp = ros::Time::now();

  pcl2_pub.publish(pcl2_integrate);
}

int main(int argc, char** argv) 
{
  Pcl_to_pcl2 pcl_to_pcl2;
  pcl_to_pcl2.initPublisher();
  pcl_to_pcl2.initSubscriber();
  return (0);
}
