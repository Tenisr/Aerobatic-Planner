#include "intention_get_corridor/intention_get_corridor.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intention_get_corridor_node");
    ros::NodeHandle nh_;

    Config config;  //参数信息输入对象
    config.load(ros::NodeHandle("~"));  //私有句柄，由于进程的唯一性默认指向这里的intention_get_corridor_node，访问这个节点的私有参数（launch写入）

    GlobalPlanner intention_get_corridor(config, nh_);  //全局规划器申请，根据启动方式创建发布者和订阅者，重点是回调实现
    intention_get_corridor.initializeMap();   //从PCD加载地图，没有就跳过

    ros::Rate lr(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
