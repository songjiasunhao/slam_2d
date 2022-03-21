#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

namespace cartographer_ros{//有两个namespace，这应该是为了代码规范，为了防止其他翻译单元的访问，和static数据成员类似，防止其他翻译单元访问
namespace{
    void Run(){
        constexpr double kTfBufferCacheTimeInSeconds =10;
        //tf2是TF1的新版本，tf2包分为tf2和tf2_ros，前者用来进行坐标变换等具体操作，后者负责与ROS消息打交道 注：以后可以换成tf
        tf2_ros::Buffer tf_buffer{ros::Duration(kTfBufferCacheTimeInSeconds)};
        tf2_ros::TransformListener tf(tf_buffer);
        
        //TopLevelOptions top_optins;//定义一些顶层配置
    }
}
}
int main(int argc, char** argv){

    /*调试工具设置，检查配置文件等操作*/
    ros::init(argc,argv,"carto_node");

    ros::start();

    //glog消息输出等

    //开始
    cartographer_ros::Run();
     
     ros::shutdown();
}