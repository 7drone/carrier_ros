#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

class Pcl_to_pcl2
{
    private:
        //ROS NodeHandle
        ros::NodeHandle nh;
        ros::NodeHandle priv_nh;

        //ROS parameter

        // ROS Topic Publisher
        ros::Publisher pcl2_pub;

        // ROS Topic Subscriber
        ros::Subscriber sensor1_sub,
                        sensor2_sub,
                        sensor3_sub;

        // ROS Service Server

        // ROS Service Client


        int lidar_numbeer;

    public:
        Pcl_to_pcl2();
        ~Pcl_to_pcl2();

        void initPublisher(void);
        void initSubscriber(void);
      
        void Sensor1Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg);
        void Sensor2Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg);
        void Sensor3Callback(const sensor_msgs::PointCloudConstPtr &cloudmsg);  

};