#!/usr/bin/env python
# -*- coding: utf-8 -*-


# catkin_install_python(PROGRAMS 
#   src/imu_rotate.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_matrix, quaternion_from_matrix, quaternion_multiply, quaternion_matrix


def mat_from_rpy(roll, pitch, yaw):
    # 返回 3x3 矩阵
    M4 = euler_matrix(roll, pitch, yaw, axes='sxyz')
    return np.array(M4[0:3, 0:3])


class ImuRotater(object):
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/livox/imu')
        self.output_topic = rospy.get_param('~output_topic', '/livox/imu_rot')
        use_rpy = rospy.get_param('~use_rpy', True)

        if use_rpy:
            roll_deg = rospy.get_param('~roll_deg', 0.0)
            pitch_deg = rospy.get_param('~pitch_deg', 0.0)
            yaw_deg = rospy.get_param('~yaw_deg', 90.0)
            roll = np.deg2rad(roll_deg)
            pitch = np.deg2rad(pitch_deg)
            yaw = np.deg2rad(yaw_deg)
            self.R = mat_from_rpy(roll, pitch, yaw)
        else:
            R_list = rospy.get_param('~extrinsic_R', None)
            if R_list is None or len(R_list) != 9:
                rospy.logwarn("~extrinsic_R not set or invalid, defaulting to yaw=-90 deg")
                self.R = mat_from_rpy(0.0, 0.0, np.deg2rad(-90.0))
            else:
                self.R = np.array(R_list, dtype=float).reshape((3,3))

        # Precompute quaternion for rotation: from rotation matrix
        R4 = np.eye(4)
        R4[0:3, 0:3] = self.R
        self.q_rot = quaternion_from_matrix(R4)  # [x,y,z,w]

        self.pub = rospy.Publisher(self.output_topic, Imu, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Imu, self.cb_imu, queue_size=200)

        rospy.loginfo('IMU rotater ready. %s -> %s', self.input_topic, self.output_topic)
        rospy.loginfo('Rotation matrix:\n%s', np.array2string(self.R, formatter={'float_kind':lambda x: "%.6f"%x}))

    def cb_imu(self, msg):
        out = Imu()
        out.header = msg.header

        # rotate angular velocity and linear acceleration
        av = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        la = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        av_r = self.R.dot(av)
        la_r = self.R.dot(la)

        out.angular_velocity.x = float(av_r[0])
        out.angular_velocity.y = float(av_r[1])
        out.angular_velocity.z = float(av_r[2])

        out.linear_acceleration.x = float(la_r[0])
        out.linear_acceleration.y = float(la_r[1])
        out.linear_acceleration.z = float(la_r[2])

        # rotate orientation quaternion: q_out = q_rot * q_in
        q_in = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # if input quaternion is zero (not set), just compute from rotation
        if np.linalg.norm(q_in) < 1e-6:
            # produce a quaternion that represents only the extrinsic rotation
            q_out = self.q_rot
        else:
            q_out = quaternion_multiply(self.q_rot, q_in)

        out.orientation.x = float(q_out[0])
        out.orientation.y = float(q_out[1])
        out.orientation.z = float(q_out[2])
        out.orientation.w = float(q_out[3])

        # copy covariances (no rotation applied to covariance here)
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # copy header stamp and frame_id
        out.header.stamp = msg.header.stamp
        # optionally you may want to change frame_id
        out.header.frame_id = rospy.get_param('~output_frame_id', msg.header.frame_id)

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('imu_rotate_node', anonymous=False)
    node = ImuRotater()
    rospy.spin()

