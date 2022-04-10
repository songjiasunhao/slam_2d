#ifndef CARTOGRAPHER_ROS_MAP_BUILD_BRIDGE
#define CARTOGRAPHER_ROS_MAP_BUILD_BRIDGE

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "top_level_node_options.h"

namespace cartographer_ros{
    class MapBuilderBridge
    {
    public:
        MapBuilderBridge(const TopLevelNodeOptions& node_options, std::unique_ptr<mapping::MapBuildInterface> map_builder
        ,tf2_ros::Buffer * tf_buffer);
        ~MapBuilderBridge();
    void HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
    void HandleTrajectoryQuery(
      cartographer_ros_msgs::TrajectoryQuery::Request& request,
      cartographer_ros_msgs::TrajectoryQuery::Response& response);
    };
    
  
}

#endif