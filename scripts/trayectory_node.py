#!/usr/bin/python

import rospy
from class_controlador import CONTROL_1

# Init of program
if __name__ == '__main__':

    rospy.init_node('cntrl_p4', anonymous=True)

    rospy.loginfo("RyCSV__2020-2")

    CONTROL_1()

    rospy.spin()