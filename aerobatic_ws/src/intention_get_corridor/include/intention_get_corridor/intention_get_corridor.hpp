#ifndef _INTENTION_GET_CORR_DIR_HPP_
#define _INTENTION_GET_CORR_DIR_HPP_

#include "intention_get_corridor/config.hpp"
#include "intention_get_corridor/glbmap.hpp"
#include "intention_get_corridor/visualizer.hpp"
#include "intention_get_corridor/astar.hpp"
#include "intention_get_corridor/solver/firi.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <chrono>
#include <random>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <quadrotor_msgs/Corridor.h>
#include <quadrotor_msgs/CorridorList.h>
#include <quadrotor_msgs/PointSeq.h>
// #include <std_msgs/Float32MultiArray.h>

class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;			//发布者和接受者取决于启动模式，通过Rviz接受地图数据还是输入PCD点云文件
    ros::Subscriber targetSub;
    ros::Subscriber odomSub;
    ros::Subscriber mapSub;
    ros::Subscriber boundSub;
    ros::Subscriber intentionSub;
    ros::Publisher intentionPub;
    ros::Publisher keyPosPub;	//关键路径点？下来看看
    ros::Publisher targetPub;
    ros::Publisher mapPub, visMapPub;
    ros::Publisher corridorPub;

    bool mapInitialized, odomInitialized;
    std::shared_ptr<GlobalMap> glbMapPtr;
    Visualizer visualizer;		//可视化工具，用于Rviz发布，后面细看
    astar_ros astarfinder;		//考虑写一个D*，哈哈动态地图
    Eigen::Vector3d bound_min, bound_max;	//这里边界用两个点围成的立方体表示边界（什么的边界？规划的边界?）

    std::vector<Eigen::Matrix<double, 6, -1>> onetimeCorridor, partCorridor;	//安全走廊序列，要结合论文细看，由超平面定义
    geometry_msgs::PoseStamped last_target, lastmsg;	//ROS消息类型，包含时间戳Stamped、坐标信息Frame与位姿Pose，最后的目标点
    Eigen::Matrix<double, 6, -1> lastPolytope{6, 0}; 	//最后一个安全走廊
    bool getFirstTarget = false;
    int confirm = 0;

    nav_msgs::Odometry odom;    //里程计消息对象

public:
    GlobalPlanner(Config &conf, ros::NodeHandle &nh_)
        : config(conf), nh(nh_), mapInitialized(false), odomInitialized(false),
          glbMapPtr(std::make_shared<GlobalMap>(config)),
          visualizer(config, nh), astarfinder(nh, config)
    {
        mapPub = nh.advertise<sensor_msgs::PointCloud2>(config.infmapTopic, 1000);
        visMapPub = nh.advertise<sensor_msgs::PointCloud2>(config.infmapTopic + "_vis", 1000);
        corridorPub = nh.advertise<quadrotor_msgs::CorridorList>("corridor_list", 1000);
        keyPosPub = nh.advertise<geometry_msgs::PoseArray>("key_pos", 1000);

        mapSub = nh.subscribe("/globalmap", 1,
                              &GlobalPlanner::MapCallback, this,
                              ros::TransportHints().tcpNoDelay());
        boundSub = nh.subscribe("/boundmap", 1,
                                &GlobalPlanner::BoundCallback, this,
                                ros::TransportHints().tcpNoDelay());
        odomSub = nh.subscribe(config.odomTopic, 1,
                               &GlobalPlanner::OdomCallback, this,
                               ros::TransportHints().tcpNoDelay());

        if (config.useLoadPCDFile)
        {
            targetSub = nh.subscribe(config.targetTopic, 1,
                                        &GlobalPlanner::setposCallBack, this,
                                        ros::TransportHints().tcpNoDelay());
            intentionPub = nh.advertise<quadrotor_msgs::PointSeq>("/MyPointSeq", 1000);
        }
        else
        {
            intentionSub = nh.subscribe("/MyPointSeq", 1,
                                        &GlobalPlanner::IntentionCallback, this,
                                        ros::TransportHints().tcpNoDelay());
            targetPub = nh.advertise<geometry_msgs::PoseStamped>(config.targetTopic, 1000);
        }
    }

    inline void initializeMap(void)
    {
        pcl::PointCloud<pcl::PointXYZ> cloudDense;
        if (config.useLoadPCDFile)
        {
            if (!mapInitialized)
            {
                bound_min(0) = config.r3Bound[0];
                bound_min(1) = config.r3Bound[2];
                bound_min(2) = config.r3Bound[4];
                bound_max(0) = config.r3Bound[1];
                bound_max(1) = config.r3Bound[3];
                bound_max(2) = config.r3Bound[5];
                ROS_INFO("Initializing map from load file!");
                std::string path = ros::package::getPath("intention_get_corridor");
                pcl::io::loadPCDFile<pcl::PointXYZ>(path + config.pointCloudPath, cloudDense);

                glbMapPtr->initialize(cloudDense, config.outlierThreshold);
                pubMap(cloudDense);

                mapInitialized = true;
                glbMapPtr->getPointCloud(cloudDense, config.expectedHeight[1]);
                astarfinder.setCloudMap(cloudDense);
            }
        }
    }

    inline void BoundCallback(const nav_msgs::Odometry::ConstPtr &msg)  //要看看论文的
    {
        if (!config.useLoadPCDFile)
        {
            bound_min(0) = msg->pose.pose.position.x;
            bound_min(1) = msg->pose.pose.position.y;
            bound_min(2) = msg->pose.pose.position.z;

            bound_max(0) = msg->twist.twist.linear.x;
            bound_max(1) = msg->twist.twist.linear.y;
            bound_max(2) = msg->twist.twist.linear.z;
            std::cout << "min = " << bound_min.transpose() << std::endl
                      << "max = " << bound_max.transpose() << std::endl;
        }
    }

    inline void MapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)  //原始地图处理
    {
        if (config.useLoadPCDFile)
            return;

        pcl::PointCloud<pcl::PointXYZ> cloudDense;
        ROS_INFO("Initializing map from callback!");
        pcl::fromROSMsg(*msg, cloudDense);  //转换为PCL点云格式
        glbMapPtr->initialize(cloudDense, config.outlierThreshold, bound_min, bound_max);
        pubMap(cloudDense);
        mapInitialized = true;

        glbMapPtr->getPointCloud(cloudDense, config.expectedHeight[1]);
        astarfinder.change_map_bd(bound_max, bound_min);
        astarfinder.setCloudMap(cloudDense);
    }

    inline void pubMap(const pcl::PointCloud<pcl::PointXYZ> &cloudDense, double sleep_time = 2)
    {
        sensor_msgs::PointCloud2 cloudVisMsg, cloudMsg;
        pcl::PointCloud<pcl::PointXYZ> infcloud;

        pcl::toROSMsg(cloudDense, cloudVisMsg);
        glbMapPtr->getPointCloud(infcloud, config.expectedHeight[1]);   //地图过滤，处理函数是基于PCL格式的，所以这里用中间数据处理，后面再转回去

        pcl::toROSMsg(infcloud, cloudMsg);

        cloudVisMsg.header.frame_id = "world";
        cloudMsg.header.frame_id = "world";
        ros::Rate sleep_rate(1 / sleep_time);
        sleep_rate.sleep();
        std::cout << "count = " << cloudDense.size() << " | " << infcloud.size() << std::endl;
        mapPub.publish(cloudMsg);   //规划使用
        visMapPub.publish(cloudVisMsg);     //可视化使用

        ROS_WARN("Map has been published!");
    }

    inline void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        odom = *msg;
        odomInitialized = true;
    }

    inline bool incorridor(const Eigen::Matrix<double, 6, -1> &corridor, const Eigen::Vector3d &pt)
    {
        Eigen::Vector3d nor_vct, point;
        for (int i = 0; i < corridor.cols(); i++)
        {
            nor_vct = corridor.col(i).array().head(3);
            point = corridor.col(i).array().tail(3);

            double dot = nor_vct.dot(point - pt);
            if (dot < 0)
                return false;
        }
        return true;
    }

    inline bool checkInterCorridor(const Eigen::Matrix<double, 6, -1> cor1, const Eigen::Matrix<double, 6, -1> cor2)
    {
        Eigen::Matrix3Xd curIV;
        Eigen::Matrix<double, 6, -1> inter_corridor;
        inter_corridor.resize(6, cor1.cols() + cor2.cols());
        inter_corridor.leftCols(cor1.cols()) = cor1;
        inter_corridor.rightCols(cor2.cols()) = cor2;
        geoutils::enumerateVs(inter_corridor, curIV);
        return curIV.cols();
    }

    inline void CorridorShortCut(std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)
    {
        if (corridorSeq.size() > 2)
        {
            for (auto fiter = corridorSeq.begin(); fiter != corridorSeq.end() - 2;)
            {
                auto biter = fiter + 2;
                if (checkInterCorridor(*fiter, *biter))
                    corridorSeq.erase(biter - 1);
                else
                    fiter++;
            }
        }
    }

    inline bool corridorSeqGen(const std::vector<Eigen::Vector3d> &pathList,
                               const std::vector<Eigen::Matrix<double, 6, -1>> &nowcorridor,
                               std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)
    {
        corridorSeq.clear();
        Eigen::Matrix<double, 6, -1> cur_corridor;
        bool ret;
        if (nowcorridor.size())
        {
            cur_corridor = *(nowcorridor.end() - 1);
        }
        else
        {
            ret = corridor_generate(*pathList.begin(), corridorSeq);
            if (!ret)
            {
                ROS_ERROR("Failed to generate corridor from start point.");
                return false;
            }
            cur_corridor = *corridorSeq.begin();
        }

        for (auto iter = pathList.begin(); iter != pathList.end(); iter++)
        {
            if (!incorridor(cur_corridor, *iter))
            {
                if (iter != pathList.begin())
                {
                    ret = corridor_generate(*(iter - 1), corridorSeq);
                }
                else
                {
                    ret = corridor_generate(*iter, corridorSeq);
                }
                if (!ret)
                {
                    ROS_ERROR("Failed to generate corridor from start point.");
                    return false;
                }
                cur_corridor = *(corridorSeq.end() - 1);
            }
        }
        CorridorShortCut(corridorSeq);
        return true;
    }

    inline void IntentionCallback(const quadrotor_msgs::PointSeq::ConstPtr &msg) //aerobatic_ws/src/uav_simulator/Utils/quadrotor_msgs
    {
        if (!odomInitialized)
        {
            ROS_WARN("No Odom!");
            return;
        }
        if (!mapInitialized)
        {
            ROS_WARN("No Map!");
            return;
        }

        int cnt_pos = int(msg->length); //记录意图点数量 
        double isPosConstrain = 0.0;
        Eigen::Matrix3Xd key_pos;   //3*X矩阵，每一列表示一个关键路径点的xyz坐标
        geometry_msgs::Pose onepos; //中间变量用于整合压入keyPosArray，其中起始点姿态水平，注意orientation成员表示z轴方向向量
        geometry_msgs::PoseArray keyPosArray;   //意图点序列，其中第一个起始点为onepos，把Unity消息点转换为统一消息序列再发布
        Eigen::Vector3d zb;         //z_bodyFrame，z轴方向向量，指定姿态
        key_pos.resize(3, cnt_pos);
        key_pos.setZero();
        keyPosArray.poses.clear();

        key_pos(0, 0) = odom.pose.pose.position.x;
        key_pos(1, 0) = odom.pose.pose.position.y;
        key_pos(2, 0) = odom.pose.pose.position.z;
        onepos.position.x = key_pos(0, 0);
        onepos.position.y = key_pos(1, 0);
        onepos.position.z = key_pos(2, 0);
        onepos.orientation.x = 0.0;
        onepos.orientation.y = 0.0;
        onepos.orientation.z = 1.0;
        onepos.orientation.w = isPosConstrain;
        keyPosArray.poses.push_back(onepos);

        for (int i = 1; i < cnt_pos; i++)   //遍历意图点（不包含起始点），把Unity的坐标信息转换为ROS的坐标信息，其中Unity的位置消息为包含三维信息的一维数组
        {                                   //同时获取key_pos和keyPosArray，前者用于数学运算，后者用于消息传递
            isPosConstrain = 0.0;
            // unity x y z means ros x z y
            key_pos(0, i) = msg->pos[i * 3 + 0];
            key_pos(1, i) = msg->pos[i * 3 + 2];
            key_pos(2, i) = msg->pos[i * 3 + 1];
            if (msg->type[i] != 0)
            {
                zb(0) = msg->vel[i * 3 + 0];
                zb(1) = msg->vel[i * 3 + 2];
                zb(2) = msg->vel[i * 3 + 1];
                zb.normalize();             //归一化
                if (msg->type[i] == 2)
                    isPosConstrain = 1.0;
            }
            else
            {
                zb(0) = 0;
                zb(1) = 0;
                zb(2) = 1;
            }
            onepos.position.x = key_pos(0, i);
            onepos.position.y = key_pos(1, i);
            onepos.position.z = key_pos(2, i);
            onepos.orientation.x = zb(0);
            onepos.orientation.y = zb(1);
            onepos.orientation.z = zb(2);
            onepos.orientation.w = isPosConstrain;
            keyPosArray.poses.push_back(onepos);
        }

        std::cout << "key_pos = " << key_pos.transpose() << std::endl;

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "world";
        goal.header.stamp = ros::Time::now();   //用于TF处理
        goal.pose = onepos;     //msg序列的最后一个点
        targetPub.publish(goal);
        getCorridorFromSetPos(keyPosArray, key_pos);    //调用A*获取走廊
    }

    inline void loadPosAtt(Eigen::Matrix3Xd &keypos, geometry_msgs::PoseArray &keyPosArray, const Eigen::Vector3d target)
    {
        keyPosArray.poses.clear();
        geometry_msgs::Pose onepos;
        keypos.resize(3, config.cnt_pos + 1);
        keypos.setZero();

        keypos(0, 0) = odom.pose.pose.position.x;
        keypos(1, 0) = odom.pose.pose.position.y;
        keypos(2, 0) = odom.pose.pose.position.z;
        onepos.position.x = keypos(0, 0);
        onepos.position.y = keypos(1, 0);
        onepos.position.z = keypos(2, 0);
        onepos.orientation.x = 0.0;
        onepos.orientation.y = 0.0;
        onepos.orientation.z = 1.0;
        onepos.orientation.w = 0.0;
        keyPosArray.poses.push_back(onepos);

        auto it_pos = config.set_pos.begin();
        auto it_att = config.set_att.begin();
        Eigen::Vector3d zb;
        double isPosConstrain;
        for (int i = 1; i < config.cnt_pos + 1; i++)
        {
            keypos(0, i) = *it_pos++;
            keypos(1, i) = *it_pos++;
            keypos(2, i) = *it_pos++;
            zb(0) = *it_att++;
            zb(1) = *it_att++;
            zb(2) = *it_att++;
            zb.normalize();
            isPosConstrain = *it_att++;
            onepos.position.x = keypos(0, i);
            onepos.position.y = keypos(1, i);
            onepos.position.z = keypos(2, i);
            onepos.orientation.x = zb(0);
            onepos.orientation.y = zb(1);
            onepos.orientation.z = zb(2);
            onepos.orientation.w = isPosConstrain;
            keyPosArray.poses.push_back(onepos);
        }
    }

    inline void setposCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        static int seq = 0;
        if (!odomInitialized)
        {
            ROS_WARN("No Odom!");
            return;
        }
        geometry_msgs::PoseArray keyPosArray;
        Eigen::Matrix3Xd key_pos;
        Eigen::Vector3d target;
        target(0) = msg->pose.position.x;
        target(1) = msg->pose.position.y;
        target(2) = msg->pose.position.z;
        loadPosAtt(key_pos, keyPosArray, target);   //加载Config的预设路径点
        quadrotor_msgs::PointSeq pointseq;
        pointseq.header.frame_id = "world";
        pointseq.header.seq = ++seq;
        pointseq.header.stamp = ros::Time::now();
        pointseq.length = key_pos.cols();
        pointseq.pos.resize(key_pos.cols() * 3);
        pointseq.vel.resize(key_pos.cols() * 3);
        pointseq.acc.resize(key_pos.cols() * 3);
        pointseq.type.resize(key_pos.cols());
        for (int i = 0; i < key_pos.cols(); i++)
        {
            pointseq.pos[i * 3 + 0] = key_pos(0, i);
            pointseq.pos[i * 3 + 1] = key_pos(2, i);
            pointseq.pos[i * 3 + 2] = key_pos(1, i);
            pointseq.vel[i * 3 + 0] = keyPosArray.poses[i].orientation.x;
            pointseq.vel[i * 3 + 1] = keyPosArray.poses[i].orientation.y;
            pointseq.vel[i * 3 + 2] = keyPosArray.poses[i].orientation.z;
            pointseq.acc[i * 3 + 0] = 0.0;
            pointseq.acc[i * 3 + 1] = 0.0;
            pointseq.acc[i * 3 + 2] = 0.0;
            pointseq.type[i] = keyPosArray.poses[i].orientation.w;
        }
        intentionPub.publish(pointseq);     //发布意图，但是貌似只是用于监视而已，统一PointSeq消息类型
        getCorridorFromSetPos(keyPosArray, key_pos);    //调用A*获取走廊
    }

    void getCorridorFromSetPos(geometry_msgs::PoseArray &keyPosArray, const Eigen::Matrix3Xd &key_pos)   //Most Important
    {
        double length;
        onetimeCorridor.clear();    //生成前清空
        std::vector<Eigen::Vector3d> pathList;  //A*规划得到的路径点序列
        Eigen::Vector3d start, goal;
        auto iter = keyPosArray.poses.begin();
        iter++;
        int count = key_pos.cols() - 1;
        for (int i = 0; i < count; i++, iter++)     //每段关键点之间分别规划
        {
            start(0) = key_pos(0, i);
            start(1) = key_pos(1, i);
            start(2) = key_pos(2, i);
            goal(0) = key_pos(0, i + 1);
            goal(1) = key_pos(1, i + 1);
            goal(2) = key_pos(2, i + 1);
            astarfinder.setstart(start);
            astarfinder.setgoal(goal);
            pathList.clear();
            astarfinder.getpathlist(pathList);      //启动规划
            length = pathList.size();
            iter->orientation.x *= length;          //将路径的长度信息编码到z轴方向向量中
            iter->orientation.y *= length;
            iter->orientation.z *= length;
            if (pathList.empty())
            {
                ROS_ERROR("A* can't find a feasible path!");
                ROS_ERROR("key pos exit with idx = %d.", i);
                return;
            }
            partCorridor.clear();
            if (!corridorSeqGen(pathList, onetimeCorridor, partCorridor))       //安全走廊生成
            {
                ROS_ERROR("Failed to generate corridor from start point.");
                return;
            }
            visualizer.visualizePolytope(partCorridor);
            onetimeCorridor.insert(onetimeCorridor.end(), partCorridor.begin(), partCorridor.end());
            usleep(300000);
        }
        keyPosPub.publish(keyPosArray);
        visualizer.visualizePolytope(onetimeCorridor);
        savecorridor();

        pubCorridor(onetimeCorridor);
        onetimeCorridor.clear();
        usleep(2000000); // sleep 2s then delete corridor
        visualizer.visualizePolytope(onetimeCorridor, 0, true);
    }

    inline void pubCorridor(const std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)   //corridorSeq即onetimeCorridor，超平面
    {
        static int list_cnt = 0;
        quadrotor_msgs::CorridorList cor_list;  //aerobatic_ws/src/uav_simulator/Utils/quadrotor_msgs

        cor_list.corridor_type = quadrotor_msgs::CorridorList::CORRIDOR_TYPE_H;
        cor_list.corridor_cnt = corridorSeq.size();
        int cor_cnt = 0;
        for (auto iter = corridorSeq.begin(); iter != corridorSeq.end(); iter++)
        {
            quadrotor_msgs::Corridor cor;           
            cor.corridor_type = quadrotor_msgs::Corridor::CORRIDOR_TYPE_H;
            cor.cnt = cor_cnt++;
            cor.size = iter->cols();
            geometry_msgs::Vector3 vec;
            geometry_msgs::Point pt;
            for (int i = 0; i < iter->cols(); i++)
            {
                vec.x = iter->col(i).array().head(3)[0];
                vec.y = iter->col(i).array().head(3)[1];
                vec.z = iter->col(i).array().head(3)[2];

                pt.x = iter->col(i).array().tail(3)[0];
                pt.y = iter->col(i).array().tail(3)[1];
                pt.z = iter->col(i).array().tail(3)[2];

                cor.nom_vec_list.push_back(vec);
                cor.point_list.push_back(pt);
            }
            cor_list.corridor_list.push_back(cor);
        }
        cor_list.header.frame_id = "world";
        cor_list.header.seq = list_cnt++;
        cor_list.header.stamp = ros::Time::now();
        corridorPub.publish(cor_list);
    }

    inline bool corridor_generate(Eigen::Vector3d origin, std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)
    {
        if (origin[0] < bound_min(0) || origin[0] > bound_max(0) ||
            origin[1] < bound_min(1) || origin[1] > bound_max(1))
        {
            ROS_INFO("Out of bound!");
            return false;
        }

        double forceHeight = config.expectedHeight[0] - 0.1;
        if (origin(2) < forceHeight)
        {
            ROS_INFO("Your click is too low in height: z = %f, force it to be %f", origin(2), forceHeight);
            origin(2) = forceHeight;
        }
        // precompute the tiangle mesh of an unit cube
        const double unitCubeTris[108] = {0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1,
                                          0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0,
                                          0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0};

        Eigen::Map<const Eigen::Matrix<double, 36, 3, Eigen::ColMajor>> unitCubeMesh(unitCubeTris);

        Eigen::Matrix<double, 6, 1> bound;
        std::vector<double> localSurface;
        const int halfWi = std::max((int)(config.localBoxHalfWidth / config.gridResolution), 1);
        const double halfW = config.gridResolution * (halfWi + 0.5);
        localSurface.reserve((2 * halfWi + 1) * (2 * halfWi + 1) * (2 * halfWi + 1) * 3);

        bound(0) = std::max(bound_min(0), origin(0) - halfW);
        bound(1) = std::min(bound_max(0), origin(0) + halfW);
        bound(2) = std::max(bound_min(1), origin(1) - halfW);
        bound(3) = std::min(bound_max(1), origin(1) + halfW);
        bound(4) = std::max(bound_min(2), origin(2) - halfW);
        bound(5) = std::min(bound_max(2), origin(2) + halfW);

        bound(4) = std::max(bound(4), config.expectedHeight[0]);
        bound(5) = std::min(bound(5), config.expectedHeight[1]);

        Eigen::Matrix3Xd cubeMesh(3, unitCubeMesh.rows());
        cubeMesh.row(0) = unitCubeMesh.col(0).transpose().array() * (bound(1) - bound(0)) + bound(0);
        cubeMesh.row(1) = unitCubeMesh.col(1).transpose().array() * (bound(3) - bound(2)) + bound(2);
        cubeMesh.row(2) = unitCubeMesh.col(2).transpose().array() * (bound(5) - bound(4)) + bound(4);

        localSurface.clear();
        glbMapPtr->ogmPtr->getSurfacePointsInBox(glbMapPtr->ogmPtr->convertPosD2I(origin), halfWi, localSurface);

        std::vector<double> localSurface2;
        localSurface2.reserve(localSurface.size());
        for (int i = 0; i < (int)(localSurface.size()) / 3; i++)
        {
            if (localSurface[i * 3 + 2] > config.expectedHeight[1] + 0.51 * config.gridResolution ||
                localSurface[i * 3 + 2] < config.expectedHeight[0] - 0.51 * config.gridResolution)
            {
                continue;
            }
            localSurface2.push_back(localSurface[i * 3]);
            localSurface2.push_back(localSurface[i * 3 + 1]);
            localSurface2.push_back(localSurface[i * 3 + 2]);
        }
        localSurface = localSurface2;

        Eigen::Matrix<double, 6, -1> hPolytope;
        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> localSurfaceMat(&(localSurface[0]), 3, localSurface.size() / 3);

        int obstacleSize = localSurfaceMat.cols();
        Eigen::Matrix<double, 3, -1> obstacleMesh(3, obstacleSize * 36 + 36);
        for (int i = 0; i < obstacleSize; i++)
        {
            obstacleMesh.block<1, 36>(0, 36 * i) = (unitCubeMesh.col(0).transpose().array() - 0.5) * config.gridResolution + localSurfaceMat(0, i);
            obstacleMesh.block<1, 36>(1, 36 * i) = (unitCubeMesh.col(1).transpose().array() - 0.5) * config.gridResolution + localSurfaceMat(1, i);
            obstacleMesh.block<1, 36>(2, 36 * i) = (unitCubeMesh.col(2).transpose().array() - 0.5) * config.gridResolution + localSurfaceMat(2, i);
        }
        obstacleMesh.rightCols<36>() = cubeMesh;
        Eigen::Matrix<double, 3, -1> tempPoints(3, 0);
        firi::maximalVolInsPolytope(obstacleMesh, tempPoints, origin, hPolytope);

        std::vector<Eigen::Matrix<double, 6, -1>> hPolytopes;
        hPolytopes.push_back(hPolytope);

        corridorSeq.push_back(hPolytope);
        lastPolytope = hPolytope;

        return true;
    }

    inline void savecorridor()
    {
        std::string path = ros::package::getPath("intention_get_corridor");
        std::ofstream fout(path + config.corridorPath);
        Eigen::VectorXi numscol(onetimeCorridor.size());
        for (size_t i = 0; i < onetimeCorridor.size(); i++)
        {
            numscol(i) = onetimeCorridor[i].cols();
        }
        fout << onetimeCorridor.size() << std::endl;
        for (size_t i = 0; i < onetimeCorridor.size(); i++)
        {
            fout << "6 " << onetimeCorridor[i].cols() << std::endl;
            fout << onetimeCorridor[i] << std::endl;
        }
        fout.close();
        std::cout << "File Saved!!!" << std::endl;
    }
};
#endif