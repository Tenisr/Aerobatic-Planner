#!/bin/bash

cleanup() {
    echo -e "\n[INFO] 正在关闭所有仿真进程..."
    kill $(jobs -p) 2>/dev/null
    wait $(jobs -p) 2>/dev/null
    echo "[INFO] 所有进程已安全关闭。"
    exit 0
}

trap cleanup SIGINT SIGTERM

roslaunch livox_ros_driver2 msg_MID360.launch > /dev/null 2>&1 &
sleep 1

roslaunch fast_lio mapping_mid360.launch > /dev/null 2>&1 &
sleep 1

roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" &
sleep 1

/home/tenisr/tenisr/hardware/PX4/QGroundControl_4.3.0.AppImage > /dev/null 2>&1 &
sleep 2

roslaunch px4ctrl run_ctrl.launch odom:="/Odometry" &
sleep 1

roslaunch uav_gazebo algorithm.launch odom:="/Odometry" &
sleep 1

wait