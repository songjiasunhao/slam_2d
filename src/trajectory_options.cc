#include "../include/slam_2d/trajectory_options.h"

namespace cartographer_ros{
    namespace {
        void CheckTrajectoryOptions(const TrajectoryOptions& options){}
    }

    TrajectoryOptions CreatTrajectoryOptions(LuaParameterDictionary * const lua_parameter_dictionary)
    {
        TrajectoryOptions options;

    options.trajectory_builder_options =
      ::cartographer::mapping::CreateTrajectoryBuilderOptions(
          lua_parameter_dictionary->GetDictionary("trajectory_builder").get());
    options.tracking_frame =
        lua_parameter_dictionary->GetString("tracking_frame");
    options.published_frame =
        lua_parameter_dictionary->GetString("published_frame");
    options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
    options.provide_odom_frame =
        lua_parameter_dictionary->GetBool("provide_odom_frame");
    options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
    options.use_nav_sat = lua_parameter_dictionary->GetBool("use_nav_sat");
    options.use_landmarks = lua_parameter_dictionary->GetBool("use_landmarks");
    options.publish_frame_projected_to_2d =
        lua_parameter_dictionary->GetBool("publish_frame_projected_to_2d");
    options.num_laser_scans =
        lua_parameter_dictionary->GetNonNegativeInt("num_laser_scans");
    options.num_multi_echo_laser_scans =
        lua_parameter_dictionary->GetNonNegativeInt("num_multi_echo_laser_scans");
    options.num_subdivisions_per_laser_scan =
        lua_parameter_dictionary->GetNonNegativeInt(
            "num_subdivisions_per_laser_scan");
    options.num_point_clouds =
        lua_parameter_dictionary->GetNonNegativeInt("num_point_clouds");
    options.rangefinder_sampling_ratio =
        lua_parameter_dictionary->GetDouble("rangefinder_sampling_ratio");
    options.odometry_sampling_ratio =
        lua_parameter_dictionary->GetDouble("odometry_sampling_ratio");
    options.fixed_frame_pose_sampling_ratio =
        lua_parameter_dictionary->GetDouble("fixed_frame_pose_sampling_ratio");
    options.imu_sampling_ratio =
        lua_parameter_dictionary->GetDouble("imu_sampling_ratio");
    options.landmarks_sampling_ratio =
        lua_parameter_dictionary->GetDouble("landmarks_sampling_ratio");
    CheckTrajectoryOptions(options);
    return options;
    }
}