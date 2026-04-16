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
#   MASTER_IP=192.168.0.107 ./experiment.sh
MASTER_IP=${MASTER_IP:-192.168.0.107}
ROS_PORT=${ROS_PORT:-11311}
LOCAL_IP=${LOCAL_IP:-$(ip route get "${MASTER_IP}" 2>/dev/null | awk '/src/ { for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')}

export ROS_MASTER_URI="http://${MASTER_IP}:${ROS_PORT}"
export ROS_IP="${LOCAL_IP}"

echo "[INFO] ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "[INFO] ROS_IP=${ROS_IP}"

roslaunch livox_ros_driver2 msg_MID360.launch > /dev/null 2>&1 &
sleep 2

rosrun fast_lio imu_rotate.py &
sleep 1

roslaunch fast_lio mapping_mid360.launch rviz:=false > /dev/null 2>&1 &
sleep 2

roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600 gcs_url:=udp-b://@ &
sleep 1

roslaunch px4ctrl run_ctrl.launch odom:="/Odometry" config_file:="$(rospack find px4ctrl)/config/ctrl_param_fpv_real.yaml" &
sleep 1

roslaunch uav_gazebo algorithm.launch odom:="/Odometry" rviz:=false visual:=true &
sleep 1

wait
