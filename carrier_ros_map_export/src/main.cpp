/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <vector>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/LoadMap.h"
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
  int threshold;
  int center_x;
  int center_y;
}parameter;

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res)
    {
      std::string mapfname = "";
      double origin[3];
      int negate;
      double occ_th, free_th;
      MapMode mode = TRINARY;
      ros::NodeHandle private_nh("~");

      cuttingparam.threshold = 1000;
      cuttingparam.center_x = 1000;
      cuttingparam.center_y = 1000;
      private_nh.param("frame_id", frame_id_, std::string("map"));
      // private_nh.param("partition_x", partition_x ,0);
      // private_nh.param("partition_y", partition_y ,0);
      private_nh.param("threshold", partition_threshold);

      ROS_INFO("partition_x : %f, partition_y : %f, threshold : %f",
                partition_x,      partition_y,       partition_threshold);
      //When called this service returns a copy of the current map
      get_map_service_ = nh_.advertiseService("static_map", &MapServer::mapCallback, this);

      //Change the currently published map
      change_map_srv_ = nh_.advertiseService("change_map", &MapServer::changeMapCallback, this);

      // Latched publisher for metadata
      metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

      // Latched publisher for data
      map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

      partition_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("partition_map", 1, true);

      deprecated_ = (res != 0);
      if (!deprecated_) {
        if (!loadMapFromYaml(fname))
        {
          exit(-1);
        }
      } else {
        if (!loadMapFromParams(fname, res))
        {
          exit(-1);
        }
      }
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Publisher metadata_pub_;
    ros::Publisher partition_map_pub_;
    ros::ServiceServer get_map_service_;
    ros::ServiceServer change_map_srv_;
    bool deprecated_;
    std::string frame_id_;
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;
    nav_msgs::OccupancyGrid modified_map;
    parameter cuttingparam;
    double partition_x, partition_y, partition_threshold;
    tf::StampedTransform transform;
    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }

    /** Callback invoked when someone requests to change the map */
    bool changeMapCallback(nav_msgs::LoadMap::Request  &request,
                           nav_msgs::LoadMap::Response &response )
    {
      if (loadMapFromYaml(request.map_url))
      {
        response.result = response.RESULT_SUCCESS;
        ROS_INFO("Changed map to %s", request.map_url.c_str());
      }
      else
      {
        response.result = response.RESULT_UNDEFINED_FAILURE;
      }
      return true;
    }

    /** Load a map given all the values needed to understand it
     */
    bool loadMapFromValues(std::string map_file_name, double resolution,
                           int negate, double occ_th, double free_th,
                           double origin[3], MapMode mode)
    {
      ROS_INFO("Loading map from image \"%s\"", map_file_name.c_str());
      try {
        map_server::loadMapFromFile(&map_resp_, map_file_name.c_str(),
                                    resolution, negate, occ_th, free_th,
                                    origin, mode);
      } catch (std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
        return false;
      }

      ros::Time::waitForValid();
      Modified_map(&cuttingparam);
      ros::Time::waitForValid();
      // To make sure get a consistent time in simulation
      modified_map.info.map_load_time =ros::Time::now();
      map_resp_.map.info.map_load_time =ros::Time::now();
      map_resp_.map.header.frame_id = frame_id_;
      modified_map.header.frame_id = frame_id_;
      modified_map.header.stamp = ros::Time::now();
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

      //Publish latched topics
      metadata_pub_.publish( meta_data_message_ );
      map_pub_.publish( map_resp_.map );
      partition_map_pub_.publish(modified_map);
      return true;
    }

    /** Load a map using the deprecated method
     */
    bool loadMapFromParams(std::string map_file_name, double resolution)
    {
      ros::NodeHandle private_nh("~");
      int negate;
      double occ_th;
      double free_th;
      double origin[3];
      private_nh.param("negate", negate, 0);
      private_nh.param("occupied_thresh", occ_th, 0.65);
      private_nh.param("free_thresh", free_th, 0.196);
      origin[0] = origin[1] = origin[2] = 0.0;
      return loadMapFromValues(map_file_name, resolution, negate, occ_th, free_th, origin, TRINARY);
    }

    /** Load a map given a path to a yaml file
     */
    bool loadMapFromYaml(std::string path_to_yaml)
    {
      std::string mapfname;
      MapMode mode;
      double res;
      int negate;
      double occ_th;
      double free_th;
      double origin[3];
      std::ifstream fin(path_to_yaml.c_str());
      if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s.", path_to_yaml.c_str());
        return false;
      }
#ifdef HAVE_YAMLCPP_GT_0_5_0
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
#else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
#endif
      try {
        doc["resolution"] >> res;
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        return false;
      }
      try {
        doc["negate"] >> negate;
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        return false;
      }
      try {
        doc["occupied_thresh"] >> occ_th;
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
        return false;
      }
      try {
        doc["free_thresh"] >> free_th;
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
        return false;
      }
      try {
        std::string modeS = "";
        doc["mode"] >> modeS;

        if(modeS=="trinary")
          mode = TRINARY;
        else if(modeS=="scale")
          mode = SCALE;
        else if(modeS=="raw")
          mode = RAW;
        else{
          ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
          return false;
        }
      } catch (YAML::Exception &) {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
        mode = TRINARY;
      }
      try {
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        return false;
      }
      try {
        doc["image"] >> mapfname;
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
        {
          ROS_ERROR("The image tag cannot be an empty string.");
          return false;
        }

        boost::filesystem::path mapfpath(mapfname);
        if (!mapfpath.is_absolute())
        {
          boost::filesystem::path dir(path_to_yaml);
          dir = dir.parent_path();
          mapfpath = dir / mapfpath;
          mapfname = mapfpath.string();
        }
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        return false;
      }
      return loadMapFromValues(mapfname, res, negate, occ_th, free_th, origin, mode);
    }

    /** Load a partition map 
     */


    


    void Modified_map(parameter *input)
    {
      modified_map.info = map_resp_.map.info;
      int start_col = std::max(input->center_x - input->threshold,0);
      int end_col = std::min(input->center_x + input->threshold, 
                             (int)map_resp_.map.info.width - 1);
      int start_row = std::max(input->center_y - input->threshold, 0);
      int end_row = std::min(input->center_y + input->threshold, 
                             (int)map_resp_.map.info.height - 1);
      ROS_INFO("x_1 : %d , x_2 : %d, x_3 : %d, x_4 : %d", 
              start_col, end_col, start_row, end_row);
      int width = end_col - start_col + 1;
      int height = end_row - start_row + 1;

      modified_map.data.resize(0);
      ROS_INFO("size : %ld",modified_map.data.size());
      std::vector<int8_t>::iterator start_iter, finish_iter;
      std::vector<int8_t> v;
      start_iter = map_resp_.map.data.begin() + start_col;
      finish_iter = map_resp_.map.data.begin() + end_col;
      int number = 0;
      for (int j = start_row; j <= end_row; j++) {
        modified_map.data.insert(modified_map.data.end(), 
                                 start_iter+j*map_resp_.map.info.width,
                                 finish_iter+j*map_resp_.map.info.width + 1);
      }
      
        // start_iter+=1;

        //if you want to check data size...
        // if(*start_iter != 0)
        // ROS_INFO("start iter : %d", *start_iter);
        // ROS_INFO("array : %d", map_resp_.map.data[number]);
        // number++;

      // for(int i=0; i<100000; i++)
      //   ROS_INFO("%d", map_resp_.map.data[i]);
      modified_map.info.width = width;
      modified_map.info.height = height;

      modified_map.info.origin.position.x = map_resp_.map.info.origin.position.x + (start_col + end_col) * map_resp_.map.info.resolution / 2;
      modified_map.info.origin.position.y = map_resp_.map.info.origin.position.y + (start_row + end_row) * map_resp_.map.info.resolution / 2;

    }


    // void robotpose(const std::string source_frame, const std::string target_frame)
    // {
    //   try
    //   {
    //     listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    //   }
    //   catch (tf::TransformException& ex)
    //   {
    //     ROS_ERROR("%s", ex.what());
    //     ros::Duration(1.0).sleep();
    //     continue;
    //   }
    //   partition_x = transform.getOrigin().x();
    //   partition_y = transform.getOrigin().y();
    // }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  if(argc != 3 && argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  if (argc != 2) {
    ROS_WARN("Using deprecated map server interface. Please switch to new interface.");
  }
  std::string fname(argv[1]);
  double res = (argc == 2) ? 0.0 : atof(argv[2]);

  try
  {
    ROS_INFO("main.cpp");
    MapServer ms(fname, res);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
