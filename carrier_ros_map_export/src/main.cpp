#include "carrier_ros_map_export/carrier_ros_map_export.h"


MapServer::MapServer()
  :nh_(""),
  private_nh("~"),
  map_frame_id_(""), robot_frame_id_(""),
  tf_listener(tf_buffer),
  saved_fname(""), saved_res(0.0)
{
  map_frame_id_ = nh_.param<std::string>("map_frame_id", "map");
  robot_frame_id_=nh_.param<std::string>("robot_frame_id", "base_footprint");
  partition_threshold = private_nh.param<double>("threshold", 160);
  initPublisher();
  initSubscriber();
  initService();
  cuttingparam.threshold=partition_threshold*8;
  timer = nh_.createTimer(ros::Duration(0.1), &MapServer::TimerTFListen, this); //10hz

}

MapServer::~MapServer(){}

void MapServer::map_export(const std::string& fname, double res)
{
  saved_fname = fname;
  saved_res = res;
  std::string mapfname = "";
  double origin[3];
  int negate;
  double occ_th, free_th;
  MapMode mode = TRINARY;


  // robot pose sub
  robotpose(map_frame_id_,robot_frame_id_);
  
  deprecated_ = (res != 0);
  if (!deprecated_)
  { 
    if (!loadMapFromYaml(fname)) 
      exit(-1);
  }
  else 
  {
    if (!loadMapFromParams(fname, res)) 
      exit(-1);
  }
}

bool MapServer::UpdateCallback(nav_msgs::GetMap::Request  &req,
                            nav_msgs::GetMap::Response &res )
{
  // request is empty; we ignore it

  // = operator is overloaded to make deep copy (tricky!)
  // map_export(saved_fname, saved_res);
  ROS_INFO("map update");
  ROS_INFO("pixel_x : %d, pixel_y : %d", cuttingparam.pixel_x
                                       , cuttingparam.pixel_y);
  return true;
}

void MapServer::initPublisher(void)
{
  /** entire map */
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

  /** partition map */
  partition_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("partition_map", 1, true);
}

void MapServer::initService(void)
{
  //When called this service returns a copy of the current map
  robot_map_update = nh_.advertiseService("map_update", &MapServer::UpdateCallback, this);

}

bool MapServer::loadMapFromValues(std::string map_file_name, double resolution,
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
  map_resp_.map.header.frame_id = "utm";
  modified_map.header.frame_id = map_frame_id_;
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

bool MapServer::loadMapFromParams(std::string map_file_name, double resolution)
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

bool MapServer::loadMapFromYaml(std::string path_to_yaml)
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

void MapServer::Modified_map(const parameter *input)
{
  modified_map.info = map_resp_.map.info;
  int start_col = std::max(input->pixel_x - input->threshold,0);
  int end_col = std::min(input->pixel_x + input->threshold, 
                          (int)map_resp_.map.info.width - 1);
  int start_row = std::max(input->pixel_y - input->threshold, 0);
  int end_row = std::min(input->pixel_y + input->threshold, 
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
  

  modified_map.info.width = width;
  modified_map.info.height = height;

  modified_map.info.origin.position.x = map_resp_.map.info.origin.position.x 
                                        + (start_col)/8;
  modified_map.info.origin.position.y = map_resp_.map.info.origin.position.y 
                                        + (start_row)/8;

}

void MapServer::robotpose(const std::string source_frame, const std::string target_frame)
{
  // tfListener.waitForTransform(source_frame, target_frame, ros::Time(0), ros::Duration(5.0));
  
  try
  {
    transform =tf_buffer.lookupTransform(source_frame, target_frame, ros::Time(0),ros::Duration(10));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  //
  cuttingparam.pixel_x = int(floor(transform.transform.translation.x+0.5))*8;
  cuttingparam.pixel_y = int(floor(transform.transform.translation.y+0.5))*8;
  
}

void MapServer::TimerTFListen(const ros::TimerEvent& event)
{
  robotpose(map_frame_id_,robot_frame_id_);
  // map_export(saved_fname, saved_res);
}

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
    MapServer ms;
    ms.map_export(fname, res);

    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
