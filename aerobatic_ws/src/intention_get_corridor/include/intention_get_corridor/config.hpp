#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <vector>

#include <ros/ros.h>

struct Config
{
    std::string infmapTopic;            
    std::string odomTopic;              
    std::string targetTopic;           
    std::string pointCloudPath;       
    std::string corridorPath;           
    double dilateRadius;     //障碍物膨胀半径          
    double gridResolution;   //栅格地图分辨率     
    std::vector<double> r3Bound;    //规划空间边界      
    double localBoxHalfWidth;       //局部规划空间半宽度
    std::vector<double> expectedHeight;    //期望飞行高度范围
    int outlierThreshold;            
    bool useLoadPCDFile;   //加载地图方式，PCD调用或ROS订阅             
    double astar_weight;               
    int cnt_pos;    //预设位置点数量                    
    std::vector<double> set_att;  //预设航点姿态，四元数表示，因为PoseStamped的姿态用四元数表示
    std::vector<double> set_pos;  //预设航点位置，三维向量      

    inline void
    load(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("InfMapTopic", infmapTopic);
        nh_priv.getParam("OdomTopic", odomTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("PointCloudPath", pointCloudPath);
        nh_priv.getParam("CorridorPath", corridorPath);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("GridResolution", gridResolution);
        nh_priv.getParam("R3Bound", r3Bound);
        nh_priv.getParam("LocalBoxHalfWidth", localBoxHalfWidth);
        nh_priv.getParam("ExpectedHeight", expectedHeight);
        nh_priv.getParam("OutlierThreshold", outlierThreshold);
        nh_priv.getParam("PointCloudUsePCD", useLoadPCDFile);
        nh_priv.getParam("Astar_weight", astar_weight);
        nh_priv.getParam("SetPos", set_pos);
        nh_priv.getParam("SetAtt", set_att);
        cnt_pos = int(set_pos.size() / 3);
        if (int(set_pos.size()) != cnt_pos * 3 || int(set_att.size()) != cnt_pos * 4)   //航点信息存在问题
        {
            ROS_ERROR("set_pos count is wrong!");
            cnt_pos = 0;
        }
    }
};

#endif