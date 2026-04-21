#!/bin/bash

cleanup() {
    echo -e "\n[INFO] 正在关闭所有仿真进程..."
    kill $(jobs -p) 2>/dev/null
    wait $(jobs -p) 2>/dev/null
    echo "[INFO] 所有进程已安全关闭。"
    exit 0
}

trap cleanup SIGINT SIGTERM

# ROS master/slave network settings (host side)
# Example:
#   MASTER_IP=192.168.0.107 LOCAL_IP=192.168.0.107 ./experiment.sh
MASTER_IP=${MASTER_IP:-192.168.0.200}
ROS_PORT=${ROS_PORT:-11311}
LOCAL_IP=${LOCAL_IP:-$(hostname -I | awk '{print $1}')}

export ROS_MASTER_URI="http://${MASTER_IP}:${ROS_PORT}"
export ROS_IP="${LOCAL_IP}"

echo "[INFO] ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "[INFO] ROS_IP=${ROS_IP}"

roslaunch uav_gazebo visualize.launch &

wait