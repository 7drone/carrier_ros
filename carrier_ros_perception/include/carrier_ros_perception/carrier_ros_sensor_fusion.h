#pragma once

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
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>


class Pcl_to_pcl2
{
    private:
        //ROS NodeHandle
        ros::NodeHandle nh;
        ros::NodeHandle priv_nh;

        //ROS parameter
        std::string base_frame, 
                    input_sensor1_topic, input_sensor2_topic, input_sensor3_topic,
                    output_topic;

        // ROS Topic Publisher
        ros::Publisher pcl2_pub,
                       pcl_pub;

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
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_1, 
                                            transformed_cloud_2, 
                                            transformed_cloud_3, 
                                            combined_cloud;

    public:
        Pcl_to_pcl2();
        ~Pcl_to_pcl2();

        void Sensor1Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg);
        void Sensor2Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg);
        void Sensor3Callback(const sensor_msgs::PointCloudConstPtr &pointcloudmsg); 

        void initPublisher(void);
        void initSubscriber(void);
      
        void TimerPclIntegrate(const ros::TimerEvent& event);
      
};