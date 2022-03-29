#ifndef CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <set>


#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"//自己加的
#include "../slam_2d/top_level_node_options.h"

//需要更换的头文件
#include "cartographer/mapping/map_builder_interface.h"
#include "absl/synchronization/mutex.h"
//map_builder_bridge.h

namespace cartographer_ros{
    class Node
    {

    public:
        Node(const TopLevelNodeOptions& top_node_options, std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder, 
        tf2_ros::Buffer *tf_buffer,bool collect_metrics);
        ~Node();
    
    private:
        absl::Mutex mutex_;//锁

        const TopLevelNodeOptions top_node_options_;
         // c++11: GUARDED_BY 是在Clang Thread Safety Analysis（线程安全分析）中定义的属性
        // GUARDED_BY是数据成员的属性, 该属性声明数据成员受给定功能保护.
        // 对数据的读操作需要共享访问, 而写操作则需要互斥访问.
        // 官方介绍文档: https://clang.llvm.org/docs/ThreadSafetyAnalysis.html
        MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);//
    };
    
}
#endif