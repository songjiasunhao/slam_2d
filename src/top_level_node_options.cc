#include "../include/slam_2d/top_level_node_options.h"

#include <vector>

namespace cartographer_ros{
    //
    TopLevelNodeOptions CreatNodeOptions(LuaParameterDictionary * const lua_parameter_dictionary)
    {
        TopLevelNodeOptions options;

        options.map_builder_options ;
        options.map_frame = lua_parameter_dictionary->GetString("map_frame");
        options.lookup_transform_timeout_sec =
            lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
        options.submap_publish_period_sec =
            lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
        options.pose_publish_period_sec =
            lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
        options.trajectory_publish_period_sec =
            lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
         if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
             options.publish_to_tf =
                lua_parameter_dictionary->GetBool("publish_to_tf");
            }
        if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
                options.publish_tracked_pose =
                    lua_parameter_dictionary->GetBool("publish_tracked_pose");
            }
        if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
                options.use_pose_extrapolator =
                    lua_parameter_dictionary->GetBool("use_pose_extrapolator");
            }
        return options;
    }

    std::tuple<TopLevelNodeOptions,TrajectoryOptions> LoadOptions(
        const std::string& configuration_directory,
        const std::string& configuration_basename){
            // // 获取配置文件所在的目录
            // 读取配置文件内容到code中
             // 根据给定的字符串, 生成一个lua字典

             // 创建元组tuple,元组定义了一个有固定数目元素的容器, 其中的每个元素类型都可以不相同
            // 将配置文件的内容填充进NodeOptions与TrajectoryOptions, 并返回
        }
}