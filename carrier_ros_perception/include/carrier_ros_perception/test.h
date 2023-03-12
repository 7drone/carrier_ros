#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Pcl_to_pcl2
{
    private:
        //ROS NodeHandle
        ros::NodeHandle nh;
        ros::NodeHandle priv_nh;

        //ROS parameter
        int sensor_number;
        std::string base_frame, 
                    input_sensor1_topic, input_sensor2_topic, input_sensor3_topic,
                    output_topic;

        // ROS Topic Publisher
        ros::Publisher pcl2_pub;

        // ROS Topic Subscriber
        ros::Subscriber sensor1_sub,
                        sensor2_sub,
                        sensor3_sub;

        // ROS Service Server

        // ROS Service Client

        // ROS Timer
        ros::Timer timer;

        // ROS TF2
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;


        int queue_size_;
        sensor_msgs::PointCloud2 pcl2_data1, pcl2_data2, pcl2_data3;
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud;

    public:
        Pcl_to_pcl2();
        ~Pcl_to_pcl2();


        inline std::string getFieldsList (const sensor_msgs::PointCloud &points);

        void cloud_cb_points2 (const sensor_msgs::PointCloud2ConstPtr &msg);
        void cloud_cb_points (const sensor_msgs::PointCloudConstPtr &msg);

        void initPublisher(void);
        void initSubscriber(void);
      
        void Sensor1Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg);
        void Sensor2Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg);
        void Sensor3Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg); 

        void TimerPclIntegrate(const ros::TimerEvent& event);

};



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
  //pointcloud, transform에 담긴 것을 변환해서 transformed_cloud에 넣는다.
  sensor_msgs::PointCloud2 transformed_cloud;
  tf2::doTransform(pointcloud2msg, transformed_cloud, transform);
  
  // // transformed_cloud에 담긴내용을 transformed에 넣는다.
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_cloud, *transformed);

  *combined_cloud += *transformed;

  // publishCombinedPointCloud();
}












class PointCloudConverter 
{
  private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_points_, sub_points2_;
    ros::Publisher pub_points_, pub_points2_;

    int queue_size_;
    std::string points_in_, points2_in_, points_out_, points2_out_;

  public:
    PointCloudConverter () : nh_ ("~"), queue_size_ (100), 
                             points_in_ ("/points_in"), points2_in_ ("/points2_in"), 
                             points_out_ ("/points_out"), points2_out_ ("/points2_out")
    {
      // Subscribe to the cloud topic using both the old message format and the new
      sub_points_ = nh_.subscribe (points_in_, queue_size_, &PointCloudConverter::cloud_cb_points, this);
      sub_points2_ = nh_.subscribe (points2_in_, queue_size_, &PointCloudConverter::cloud_cb_points2, this);
      pub_points_ = nh_.advertise<sensor_msgs::PointCloud> (points_out_, queue_size_);
      pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2> (points2_out_, queue_size_);
      ROS_INFO ("PointCloudConverter initialized to transform from PointCloud (%s) to PointCloud2 (%s).", nh_.resolveName (points_in_).c_str (), nh_.resolveName (points2_out_).c_str ());
      ROS_INFO ("PointCloudConverter initialized to transform from PointCloud2 (%s) to PointCloud (%s).", nh_.resolveName (points2_in_).c_str (), nh_.resolveName (points_out_).c_str ());
    }
    

    inline std::string
      getFieldsList (const sensor_msgs::PointCloud2 &cloud)
    {
      std::string result;
      for (size_t i = 0; i < cloud.fields.size () - 1; ++i)
        result += cloud.fields[i].name + " ";
      result += cloud.fields[cloud.fields.size () - 1].name;
      return (result);
    }


    inline std::string
      getFieldsList (const sensor_msgs::PointCloud &points)
    {
      std::string result = "x y z";
      for (size_t i = 0; i < points.channels.size (); i++)
        result = result + " " + points.channels[i].name;
      return (result);
    }


    void
      cloud_cb_points2 (const sensor_msgs::PointCloud2ConstPtr &msg)
    {
      if (pub_points_.getNumSubscribers () <= 0)
      {
        //ROS_DEBUG ("[point_cloud_converter] Got a PointCloud2 with %d points on %s, but no subscribers.", msg->width * msg->height, nh_.resolveName (points2_in_).c_str ());
        return;
      }

      ROS_DEBUG ("[point_cloud_converter] Got a PointCloud2 with %d points (%s) on %s.", msg->width * msg->height, getFieldsList (*msg).c_str (), nh_.resolveName (points2_in_).c_str ());
      
      sensor_msgs::PointCloud output;
      // Convert to the new PointCloud format
      if (!sensor_msgs::convertPointCloud2ToPointCloud (*msg, output))
      {
        ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud2 to sensor_msgs::PointCloud failed!");
        return;
      }
      ROS_DEBUG ("[point_cloud_converter] Publishing a PointCloud with %d points on %s.", (int)output.points.size (), nh_.resolveName (points_out_).c_str ());
      pub_points_.publish (output);
    }


    void
      cloud_cb_points (const sensor_msgs::PointCloudConstPtr &msg)
    {
      if (pub_points2_.getNumSubscribers () <= 0)
      {
        //ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points on %s, but no subscribers.", (int)msg->points.size (), nh_.resolveName (points_in_).c_str ());
        return;
      }

      ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points (%s) on %s.", (int)msg->points.size (), getFieldsList (*msg).c_str (), nh_.resolveName (points_in_).c_str ());

      sensor_msgs::PointCloud2 output;
      // Convert to the old point cloud format
      if (!sensor_msgs::convertPointCloudToPointCloud2 (*msg, output))
      {
        ROS_ERROR ("[point_cloud_converter] Conversion from sensor_msgs::PointCloud to sensor_msgs::PointCloud2 failed!");
        return;
      }
      ROS_DEBUG ("[point_cloud_converter] Publishing a PointCloud2 with %d points on %s.", output.height * output.width, nh_.resolveName (points2_out_).c_str ());
      pub_points2_.publish (output);
    }

};
