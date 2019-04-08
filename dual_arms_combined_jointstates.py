#!/usr/bin/env python
"""
By Khang Vu, 2019

This script will combine two JointState nodes into one
and publish it to /joint_states.

Current usage: xamyab_ur_common.launch
"""
import rospy
from sensor_msgs.msg import JointState


class CombinedJointStates:
    def __init__(self, node_name='CombinedJointStates', topic_name='/joint_states',
                 first_arm='/first_arm/joint_states', second_arm='/second_arm/joint_states'):
        self.first_msg = None
        self.second_msg = None
        rospy.init_node(node_name, anonymous=True)
        rospy.Subscriber(first_arm, JointState, self.first_arm_cb, queue_size=10)
        rospy.Subscriber(second_arm, JointState, self.second_arm_cb, queue_size=10)
        self.publisher = rospy.Publisher(topic_name, JointState, queue_size=10)

    def first_arm_cb(self, msg):
        self.first_msg = msg

    def second_arm_cb(self, msg):
        self.second_msg = msg

    def publish(self):
        while not rospy.is_shutdown():
            if self.second_msg is None or self.first_msg is None:
                continue

            msg = JointState()
            msg.header = self.first_msg.header
            msg.name.extend(self.first_msg.name)
            msg.name.extend(self.second_msg.name)
            msg.position.extend(self.first_msg.position)
            msg.position.extend(self.second_msg.position)
            msg.velocity.extend(self.first_msg.velocity)
            msg.velocity.extend(self.second_msg.velocity)
            msg.effort.extend(self.first_msg.effort)
            msg.effort.extend(self.second_msg.effort)
            self.publisher.publish(msg)


if __name__ == '__main__':
    CombinedJointStates().publish()
