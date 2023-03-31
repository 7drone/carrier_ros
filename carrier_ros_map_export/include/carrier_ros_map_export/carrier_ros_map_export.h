#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "carrier_ros_map_export/image_loader.h"
#include <fstream>

#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/LoadMap.h"
#include "nav_msgs/GetMap.h"
#include "yaml-cpp/yaml.h"

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif


typedef struct 
{
  int threshold = 1000;
  int center_x = 2000;
  int center_y = 2000;
}parameter;

class MapServer
{
    private:
        //ROS NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh;

        //ROS parameter
        std::string map_frame_id_, robot_frame_id_;

        // ROS Topic Publisher
        ros::Publisher map_pub_,
                       metadata_pub_,
                       partition_map_pub_;

        // ROS Topic Subscriber
        ros::Subscriber sensor1_sub,
                        sensor2_sub,
                        sensor3_sub;

        // ROS Service Server
        ros::ServiceServer get_map_service_,
                           change_map_srv_;

        // ROS Service Client


        // ROS msg
        nav_msgs::MapMetaData meta_data_message_;
        nav_msgs::GetMap::Response map_resp_;
        nav_msgs::OccupancyGrid modified_map;
        geometry_msgs::TransformStamped transform;

        // ROS Timer
        ros::Timer timer;

        // ROS TF2
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tfListener;

        parameter cuttingparam;
        bool deprecated_;
        int queue_size_;
        double partition_x, partition_y, partition_threshold;
    
    public:
        MapServer();
        ~MapServer();
        
        void map_export(const std::string& fname, double res);

        bool mapCallback(nav_msgs::GetMap::Request  &req,
                         nav_msgs::GetMap::Response &res );

        bool changeMapCallback(nav_msgs::LoadMap::Request  &request,
                               nav_msgs::LoadMap::Response &response );

        bool loadMapFromValues(std::string map_file_name, double resolution,
                               int negate, double occ_th, double free_th,
                               double origin[3], MapMode mode);

        bool loadMapFromParams(std::string map_file_name, double resolution);

        bool loadMapFromYaml(std::string path_to_yaml);

        void Modified_map(parameter *input);

        void robotpose(const std::string source_frame, const std::string target_frame);

        void initPublisher(void);
        void initSubscriber(void){}
        void initService(void);

};