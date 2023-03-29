#include <ros/ros.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/LoadMap.h"

// Define the threshold for cutting the map
const int threshold = 50;

// Define the center point for cutting the map
const int center_x = 100;
const int center_y = 100;

class MapCutter {
public:
  MapCutter() {
    // Subscribe to the map topic
    map_sub_ = nh_.subscribe("/map", 1, &MapCutter::mapCallback, this);

    // Advertise the modified map topic
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("modified_map", 1);
    metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("modified_map_metadata", 1, true);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    

    // Copy over the header and info fields
    modified_map.header = msg->header;
    modified_map.info = msg->info;
    // Compute the start and end indices for the rows and columns to be copied
    int start_col = std::max(center_x - threshold, 0);
    int end_col = std::min(center_x + threshold, (int)msg->info.width - 1);
    int start_row = std::max(center_y - threshold, 0);
    int end_row = std::min(center_y + threshold, (int)msg->info.height - 1);

    // Compute the size of the modified map
    int width = end_col - start_col + 1;
    int height = end_row - start_row + 1;

    // Allocate memory for the modified map data
    modified_map.data.resize(width * height);

    // // Copy the map data from the original map to the modified map
    for (int j = start_row; j <= end_row; j++) {
      for (int i = start_col; i <= end_col; i++) {
        // Compute the index in the modified map
        int modified_index = (j - start_row) * width + (i - start_col);

        // Compute the index in the original map
        int original_index = j * msg->info.width + i;

        // Copy the value from the original map to the modified map
        modified_map.data[modified_index] = msg->data[original_index];
      }
    }

    // Set the width and height of the modified map
    modified_map.info.width = width;
    modified_map.info.height = height;

    // Set the origin of the modified map to the center of the cutout
    modified_map.info.origin.position.x = msg->info.origin.position.x + (start_col + end_col) * msg->info.resolution / 2;
    modified_map.info.origin.position.y = msg->info.origin.position.y + (start_row + end_row) * msg->info.resolution / 2;

    //set ros time
    modified_map.info.map_load_time = ros::Time::now();
    modified_map.header.stamp       = ros::Time::now();

    // Publish the modified map
    metadata_pub_.publish(modified_map.info);
    map_pub_.publish(modified_map);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Publisher map_pub_;
  ros::Publisher metadata_pub_;
  nav_msgs::OccupancyGrid modified_map;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_cutter");
  MapCutter cutter;
  ros::spin();
  return 0;
}