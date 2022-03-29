#ifndef TOP_LEVEL_NODE_OPTIONS_H
#define TOP_LEVEL_NODE_OPTIONS_H

#include <string>
#include <tuple>

//tuple是C++11新标准里的类型。它是一个类似pair类型的模板。pair类型是每个成员变量各自可以是任意类型，但是只能有俩个成员，而tuple与pair不同的是它可以有任意数量的成员。但是每个确定的tuple类型的成员数目是固定的。
namespace cartographer_ros{
    struct TopLevelNodeOptions
    {
        //定义mapbuilderopthions类
        std::string map_frame;
        double lookup_transform_timeout_sec;//设定tf超时阈值
        double submap_publish_period_sec;//发布的间隔时间
        double pose_publish_period_sec;
        double trajectory_publish_period_sec;
        
        bool publish_to_tf = true;
        bool publish_tracked_pose =false;
        bool use_pose_extrapolator =true;
    };
    

    //TopLevelNodeOptions CreatNodeOptions(LuaParameterDictionary * lua_parameter_dictionary);
    //std::tuple<TopLevelNode,TrajectoryOptions> LoadOptions(const std::string& configuration_directory,const std::string& configruration_basename);

}
#endif
