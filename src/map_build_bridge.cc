#include "../include/slam_2d/map_builder_bridge.h"

namespace cartographer_ros{
    namespace{}

    /**
 * @brief 根据传入的node_options, MapBuilder, 以及tf_buffer 完成三个本地变量的初始化
 * 
 * @param[in] node_options 参数配置
 * @param[in] map_builder 在node_main.cc中传入的MapBuilder
 * @param[in] tf_buffer tf_buffer
 */
MapBuilderBridge::MapBuilderBridge(
    const TopLevelNodeOptions& node_options,
    std::unique_ptr<mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_(std::move(map_builder)),
      tf_buffer_(tf_buffer) {}

      /**
 * @brief 获取对应id轨迹的 索引为 submap_index 的地图的栅格值及其他信息
 * 
 * @param[in] request 轨迹id与submap的index
 * @param[in] response 是否成功
 */
void MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {

  cartographer::mapping::proto::SubmapQuery::Response response_proto;

  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
    // 获取压缩后的地图数据
    const std::string error =
        map_builder_->SubmapToProto(submap_id, &response_proto);
        if (!error.empty()) {
            LOG(ERROR) << error;
            response.status.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
            response.status.message = error;
            return;
        }

    response.submap_version = response_proto.submap_version();

    // 将response_proto中的地图栅格值存入到response中
        for (const auto& texture_proto : response_proto.textures()) {
            response.textures.emplace_back();//作用与push_back相似，此处应该该是创建一个空对象
            // 获取response中存储地图变量的引用
            auto& texture = response.textures.back();
            // 对引用的变量进行赋值
            texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                                texture_proto.cells().end());
            texture.width = texture_proto.width();
            texture.height = texture_proto.height();
            texture.resolution = texture_proto.resolution();
            texture.slice_pose = ToGeometryMsgPose(
                cartographer::transform::ToRigid3(texture_proto.slice_pose()));
        }
    response.status.message = "Success.";
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    }

    // 获取对应id轨迹的所有位姿的集合
void MapBuilderBridge::HandleTrajectoryQuery(
    cartographer_ros_msgs::TrajectoryQuery::Request& request,
    cartographer_ros_msgs::TrajectoryQuery::Response& response) {
    // This query is safe if the trajectory doesn't exist (returns 0 poses).
    // However, we can filter unwanted states at the higher level in the node.
    const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
    for (const auto& node_id_data :
        node_poses.trajectory(request.trajectory_id)) {
        if (!node_id_data.data.constant_pose_data.has_value()) {
        continue;
        }
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = node_options_.map_frame;
        pose_stamped.header.stamp =
            ToRos(node_id_data.data.constant_pose_data.value().time);
        // 使用的是global坐标系下的坐标
        pose_stamped.pose = ToGeometryMsgPose(node_id_data.data.global_pose);
        response.trajectory.push_back(pose_stamped);
    }
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message = absl::StrCat(
        "Retrieved ", response.trajectory.size(),
        " trajectory nodes from trajectory ", request.trajectory_id, ".");
    }
}