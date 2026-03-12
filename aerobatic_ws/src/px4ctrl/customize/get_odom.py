#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class OdomPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher', anonymous=True)
        
        self.odom_pub = rospy.Publisher('/visual_slam/odom', Odometry, queue_size=10)

        self.model_name = rospy.get_param('~model_name', 'iris')
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)

        self.rate = rospy.Rate(50)  # 10 Hz

        self.last_stamp = None

        rospy.loginfo("OdomPublisher initialized for model: {}".format(self.model_name))

    def model_callback(self, data):
        try:
            index = data.name.index(self.model_name)
            pose = data.pose[index]
            twist = data.twist[index]

            stamp = rospy.Time.now()
            if hasattr(data, 'header') and hasattr(data.header, 'stamp'):
                stamp = data.header.stamp

            # 检查是否与上一次时间戳重复
            if self.last_stamp is not None and stamp == self.last_stamp:
                # rospy.logwarn("Skip repeated odometry at stamp: {}".format(stamp.to_sec()))
                return
            self.last_stamp = stamp
            
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            odom.pose.pose = pose
            odom.twist.twist = twist
            
            self.odom_pub.publish(odom)
            # rospy.loginfo("Simulation odometry info:{}".format(odom.pose))
        except ValueError:
            rospy.logwarn("Model name not found in /gazebo/model_states")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        odom_publisher = OdomPublisher()
        odom_publisher.run()
    except rospy.ROSInterruptException:
        pass