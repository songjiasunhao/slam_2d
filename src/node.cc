#include "../include/slam_2d/node.h"

#include <chrono>
#include <string>
#include <vector>
#include <cstdlib>

#include <Eigen/Core>
#include <absl/memory/memory.h>
#include <absl/strings/str_cat.h>



namespace cartographer_ros{
    //using声明与其他命名空间简化

//初始化列表，第一个初始的为成员变量top_node_options_,第二个为成员变量map_builder_bridge_
//列表的顺序与成员变量声明顺序相关，与列表中顺序无关，即初始化列表永远先初始化先声明的成员变量
   Node::Node(const TopLevelNodeOptions& top_node_options, 
        std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder, 
        tf2_ros::Buffer * const tf_buffer,
      const  bool collect_metrics):top_node_options_(top_node_options),
      map_builder_bridge_(top_node_options_,std::move(map_builder),tf_buffer)
     {

     }
}