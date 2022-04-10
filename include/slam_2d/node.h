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
#include "cartographer_ros/map_builder_bridge.h"


namespace cartographer_ros{
    class Node
    {

    public:
        Node(const TopLevelNodeOptions& top_node_options, std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder, 
        tf2_ros::Buffer *tf_buffer,bool collect_metrics);
        ~Node();
         
                // The following functions handle adding sensor data to a trajectory.
        void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                                    const nav_msgs::Odometry::ConstPtr& msg);
        void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                                    const sensor_msgs::NavSatFix::ConstPtr& msg);
        void HandleLandmarkMessage(
            int trajectory_id, const std::string& sensor_id,
            const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);
        void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::Imu::ConstPtr& msg);
        void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                                    const sensor_msgs::LaserScan::ConstPtr& msg);
        void HandleMultiEchoLaserScanMessage(
            int trajectory_id, const std::string& sensor_id,
            const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
        void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                        const sensor_msgs::PointCloud2::ConstPtr& msg);
    private: 
        struct Subscriber {
        ::ros::Subscriber subscriber;

        // ::ros::Subscriber::getTopic() does not necessarily return the same
        // std::string
        // it was given in its constructor. Since we rely on the topic name as the
        // unique identifier of a subscriber, we remember it ourselves.
        std::string topic;
    };

     bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
    bool HandleTrajectoryQuery(
      ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
      ::cartographer_ros_msgs::TrajectoryQuery::Response& response);
    bool HandleStartTrajectory(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);
    bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);
    bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
                        cartographer_ros_msgs::WriteState::Response& response);
    bool HandleGetTrajectoryStates(
      ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
      ::cartographer_ros_msgs::GetTrajectoryStates::Response& response);
    bool HandleReadMetrics(
      cartographer_ros_msgs::ReadMetrics::Request& request,
      cartographer_ros_msgs::ReadMetrics::Response& response);
    
     void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
    bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
    bool ValidateTopicNames(const TrajectoryOptions& options); 
    
     std::unordered_map<int, std::vector<Subscriber>> subscribers_;

      absl::Mutex mutex_;//锁

        const TopLevelNodeOptions top_node_options_;
         // c++11: GUARDED_BY 是在Clang Thread Safety Analysis（线程安全分析）中定义的属性
        // GUARDED_BY是数据成员的属性, 该属性声明数据成员受给定功能保护.
        // 对数据的读操作需要共享访问, 而写操作则需要互斥访问.
        // 官方介绍文档: https://clang.llvm.org/docs/ThreadSafetyAnalysis.html
        MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);//

        ros::NodeHandle node_handle_;
        ros::Publisher submap_list_publisher_;
        ros::Publisher trajectory_node_list_publisher_;
        ros::Publisher landmark_poses_list_publisher_;
        ros::Publisher constraint_list_publisher_;
        ros::Publisher tracked_pos_publisher_;

        std::vector<ros::ServiceServer> service_servers_;
        ros::Publisher scan_matched_point_cloud_publisher_;
    };
    
}
#endif