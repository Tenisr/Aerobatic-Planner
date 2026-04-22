#!/bin/bash

# This script must be sourced so exported vars stay in the current shell.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
	echo "[ERROR] 请使用: source connect.sh"
	echo "[ERROR] 或者: . connect.sh"
	exit 1
fi

MASTER_IP=${MASTER_IP:-192.168.0.108}
ROS_PORT=${ROS_PORT:-11311}
LOCAL_IP=${LOCAL_IP:-$(ip route get "${MASTER_IP}" 2>/dev/null | awk '/src/ { for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }')}
LOCAL_IP=${LOCAL_IP:-$(hostname -I | awk '{print $1}')}

export ROS_MASTER_URI="http://${MASTER_IP}:${ROS_PORT}"
export ROS_IP="${LOCAL_IP}"

echo "[INFO] ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "[INFO] ROS_IP=${ROS_IP}"
echo "[INFO] ROS网络通信已配置"