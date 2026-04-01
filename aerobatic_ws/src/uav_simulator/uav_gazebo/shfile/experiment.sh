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
sleep 2

rosrun fast_lio imu_rotate.py &
sleep 1

roslaunch fast_lio mapping_mid360.launch > /dev/null 2>&1 &
sleep 2

roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600 gcs_url:=udp-b://@ &
sleep 1

roslaunch px4ctrl run_ctrl.launch odom:="/Odometry" &
sleep 1

# roslaunch uav_gazebo algorithm.launch odom:="/Odometry" &
# sleep 1

wait