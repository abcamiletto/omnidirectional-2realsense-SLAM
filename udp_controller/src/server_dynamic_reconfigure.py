#!/usr/bin/env python
# coding: utf-8

import rospy
from dynamic_reconfigure.server import Server
from udp_controller.cfg import odom_covarianceConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {odom_gain}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("DRserver", anonymous = True)

    srv = Server(odom_covarianceConfig, callback)
    rospy.spin()
