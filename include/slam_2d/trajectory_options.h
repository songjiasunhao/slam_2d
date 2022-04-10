#ifndef CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
#define CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H

#include <string>

namespace cartographer_ros{
    //一条轨迹的参数基础配置
    struct TrajectoryOptions
    {
        //定义 trajectoy_builder_options类的对象
        std::string tracking_frame;
        std::string publish_frame;
        std::string odom_frame;
        bool provide_odom_frame;
        bool use_odometry;
        bool use_nav_sat;
        bool use_landmarks;
        bool publish_frame_projected_to_2d;
        int num_laser_scans;
        int num_multi_echo_laser_scans;
        int num_subdivisions_per_laser_scan;
        int num_point_clouds;
        double rangefinder_sampling_ratio;
        double odometry_sampling_ratio;
        double fixed_frame_pose_sampling_ratio;
        double imu_sampling_ratio;
        double landmarks_sampling_ratio;
    };
    
    TrajectoryOptions CreatTrajectoryOptions(LuaParameterDictionary* lua_parameter_dicitonary);
}
#endif