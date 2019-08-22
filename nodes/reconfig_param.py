#!/usr/bin/env python

import rospy
import rosparam

from dynamic_reconfigure.server import Server
from mcdaq.cfg import mcdaqConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param} , {double_param} , {str_param} , {bool_param} , {size} """.format(
        **config))
    return config

if __name__ == "__main__":
    rospy.init_node("mcdaq_reconfig_param", anonymous = False)

    srv = Server(mcdaqConfig, callback)
    rospy.spin()