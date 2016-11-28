#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState


def joint_states_callback(msg):
    rospy.loginfo("received JointState")
    param_dict = dict()
    for joint_id, joint_name in enumerate(msg.name):
        param_dict[joint_name] = msg.position[joint_id]

    rospy.set_param_raw('zeros', param_dict)

if __name__ == '__main__':
    """
    simple python node that subscribes to a joint state publisher and transforms every JointState message to a ros zeros parameter
    """
    rospy.myargv(argv=sys.argv)
    rospy.init_node('pyros_topic_to_param')

    rospy.loginfo('node is running..')

    rospy.Subscriber('joint_states', JointState, joint_states_callback)

    while not rospy.is_shutdown():
        rospy.spin()
        rospy.sleep(1.)