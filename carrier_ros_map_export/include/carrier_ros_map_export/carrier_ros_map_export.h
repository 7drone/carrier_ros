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
  int pixel_x = 2000;
  int pixel_y = 2000;
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


        // ROS Service Server
        ros::ServiceServer robot_map_update;

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
        tf2_ros::TransformListener tf_listener;
        tf::TransformListener listener;

        parameter cuttingparam;
        bool deprecated_;
        double partition_threshold,
               saved_res;
        std::string saved_fname;
        

    public:
        MapServer();
        ~MapServer();
        
        void map_export(const std::string& fname, double res);

        bool UpdateCallback(nav_msgs::GetMap::Request  &req,
                         nav_msgs::GetMap::Response &res );

        bool loadMapFromValues(std::string map_file_name, double resolution,
                               int negate, double occ_th, double free_th,
                               double origin[3], MapMode mode);
        

        bool loadMapFromParams(std::string map_file_name, double resolution);

        bool loadMapFromYaml(std::string path_to_yaml);

        void Modified_map(const parameter *input);

        void robotpose(const std::string source_frame, const std::string target_frame);

        void initPublisher(void);
        void initSubscriber(void){}
        void initService(void);

        void TimerTFListen(const ros::TimerEvent& event);
};