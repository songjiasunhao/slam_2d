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

  namespace {
   
   
    /**
 * @brief 在node_handle中订阅topic,并与传入的回调函数进行注册
 * 
 * @tparam MessageType 模板参数,消息的数据类型
 * @param[in] handler 函数指针, 接受传入的函数的地址
 * @param[in] trajectory_id 轨迹id
 * @param[in] topic 订阅的topic名字
 * @param[in] node_handle ros的node_handle
 * @param[in] node node类的指针
 * @return ::ros::Subscriber 订阅者
 */
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstPtr&),//？？为什么要加typename
    const int trajectory_id, const std::string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node) {

        return node_handle->subscribe<MessageType>(
            topic, kInfiniteSubscriberQueueSize,  // kInfiniteSubscriberQueueSize = 0
            // 使用boost::function构造回调函数,被subscribe注册<void(const typename MessageType::ConstPtr&)表示的是要构造什么类型的函数指针
            boost::function<void(const typename MessageType::ConstPtr&)>(
                // c++11: lambda表达式，这里可以看作一个函数，且这个函数给function提供参数，用来构造函数指针
                [node, handler, trajectory_id, topic](const typename MessageType::ConstPtr& msg) {
                  (node->*handler)(trajectory_id, topic, msg);//此处传入Handle***Message函数，Handle***Message函数为真正的回调函数
                }));
      }
  }
//初始化列表，第一个初始的为成员变量top_node_options_,第二个为成员变量map_builder_bridge_
//列表的顺序与成员变量声明顺序相关，与列表中顺序无关，即初始化列表永远先初始化先声明的成员变量
   Node::Node(const TopLevelNodeOptions& top_node_options, 
        std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder, 
        tf2_ros::Buffer * const tf_buffer,
      const  bool collect_metrics):top_node_options_(top_node_options),
      map_builder_bridge_(top_node_options_,std::move(map_builder),tf_buffer)
     {
        absl::MutexLock lock(&mutex_);
        //判断collect_metrics;

        //1.声明要发布的topic,
        //Todo:: 消息类型可以换成ros的
          // 发布SubmapList
     sumap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  // 发布轨迹
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  // 发布landmark_pose
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  // 发布约束
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  // 发布tracked_pose, 默认不发布
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_ =
        node_handle_.advertise<::geometry_msgs::PoseStamped>(
            kTrackedPoseTopic, kLatestOnlyPublisherQueueSize);
   }
   // 李想 add，3d模式发布点云
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    point_cloud_map_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            kPointCloudMapTopic, kLatestOnlyPublisherQueueSize, true);

     }

     //Step: 2 声明发布对应名字的ROS服务, 并将服务的发布器放入到vector容器中
    service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
    service_servers_.push_back(node_handle_.advertiseService(
      kTrajectoryQueryServiceName, &Node::HandleTrajectoryQuery, this));
    service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
    service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
    service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));
    service_servers_.push_back(node_handle_.advertiseService(
      kGetTrajectoryStatesServiceName, &Node::HandleGetTrajectoryStates, this));
    service_servers_.push_back(node_handle_.advertiseService(
      kReadMetricsServiceName, &Node::HandleReadMetrics, this));

  // Step: 3 处理之后的点云的发布器
  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);
  }

    /**
   * @brief 获取对应id轨迹的 索引为submap_index 的submap
   *
   * @param[in] request 获取submap的请求
   * @param[out] response 服务的回应
   * @return true: ROS的service只能返回true, 返回false程序会中断
   */
  bool Node::HandleSubmapQuery(
      ::cartographer_ros_msgs::SubmapQuery::Request& request,
      ::cartographer_ros_msgs::SubmapQuery::Response& response) {
    absl::MutexLock lock(&mutex_);
    map_builder_bridge_.HandleSubmapQuery(request, response);
    return true;
  }

  /**
 * @brief 获取对应id的轨迹
 *
 * @param[in] request
 * @param[out] response
 * @return true: ROS的service只能返回true, 返回false程序会中断
 */
bool Node::HandleTrajectoryQuery(
    ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
    ::cartographer_ros_msgs::TrajectoryQuery::Response& response) {
  absl::MutexLock lock(&mutex_);

  // 检查对应id的轨迹是否存在, 如果存在判断一下该id轨迹的状态
  response.status = TrajectoryStateToStatus(
      request.trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response.status.message;
    return true;
  }
  // 获取轨迹
  map_builder_bridge_.HandleTrajectoryQuery(request, response);
  return true;
}

/**
 * @brief 订阅话题与注册回调函数
 * 
 * @param[in] options 配置参数
 * @param[in] trajectory_id 轨迹id  
 */
void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const int trajectory_id) {
  // laser_scan 的订阅与注册回调函数, 多个laser_scan 的topic 共用同一个回调函数
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }

  // multi_echo_laser_scans的订阅与注册回调函数
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }
  
  // point_clouds 的订阅与注册回调函数
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // imu 的订阅与注册回调函数,只有一个imu的topic
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, kImuTopic,
                                                &node_handle_, this),
         kImuTopic});
  }

  // odometry 的订阅与注册回调函数,只有一个odometry的topic
  if (options.use_odometry) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, kOdometryTopic,
                                                  &node_handle_, this),
         kOdometryTopic});
  }

  // gps 的订阅与注册回调函数,只有一个gps的topic
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
             &node_handle_, this),
         kNavSatFixTopic});
  }

  // landmarks 的订阅与注册回调函数,只有一个landmarks的topic
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
             &Node::HandleLandmarkMessage, 
             trajectory_id, 
             kLandmarkTopic,
             &node_handle_, 
             this),
         kLandmarkTopic});
  }
}


// 检查TrajectoryOptions是否存在2d或者3d轨迹的配置信息
bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

// 检查topic名字是否被其他轨迹使用
bool Node::ValidateTopicNames(const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    // 如果topic能够在subscribed_topics_中找到, 证明这个topic名字被之前的轨迹使用了
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

/**
 * @brief 处理里程计数据,里程计的数据走向有2个
 * 第1个是传入PoseExtrapolator,用于位姿预测
 * 第2个是传入SensorBridge,使用其传感器处理函数进行里程计数据处理
 * 
 * @param[in] trajectory_id 轨迹id
 * @param[in] sensor_id 里程计的topic名字
 * @param[in] msg 里程计的ros格式的数据
 */
void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  // extrapolators_使用里程计数据进行位姿预测
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

/**
 * @brief 处理imu数据,imu的数据走向有2个
 * 第1个是传入PoseExtrapolator,用于位姿预测与重力方向的确定
 * 第2个是传入SensorBridge,使用其传感器处理函数进行imu数据处理
 * 
 * @param[in] trajectory_id 轨迹id
 * @param[in] sensor_id imu的topic名字
 * @param[in] msg imu的ros格式的数据
 */
void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  // extrapolators_使用里程计数据进行位姿预测
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  // 根据配置,是否将传感器数据跳过
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

// 调用SensorBridge的传感器处理函数进行数据处理
void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

}