#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState

_latest_msg = None

def joint_states_callback(msg):
    rospy.loginfo("pyros_topic_to_param, received message")
    _latest_msg = msg

if __name__ == '__main__':
    """
    simple python node that subscribes to a joint state publisher and transforms first message to a ros parameter, then quits
    """
    rospy.myargv(argv=sys.argv)
    rospy.init_node('pyros_topic_to_param')

    rospy.Subscriber('joint_states', JointState, joint_states_callback)

    while _latest_msg is None:
        rospy.spin()

    param_dict = dict()
    for joint_id, joint_name in enumerate(_latest_msg.name):
        param_dict[joint_name] = _latest_msg.position[joint_id]

    rospy.set_param_raw('zeros', param_dict)

    rospy.spin()