#!/usr/bin/python

import math

import numpy as np
import rospy
import numpy as np
from   rospy.numpy_msg import numpy_msg
import sys, select, termios, tty
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

import tf2_ros
import tf2_msgs.msg
import tf_conversions

import geometry_msgs.msg
from   geometry_msgs.msg    import Twist
from geometry_msgs.msg      import TransformStamped
from   nav_msgs.msg         import Odometry

class CONTROL_1:  

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub1 = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size=10)
        rospy.Subscriber("/odom",Odometry,self.callback)

        self.broadcts  = tf2_ros.TransformBroadcaster()
        self.transform = TransformStamped()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.MTH = [0]

        self.f = 10
        rate = rospy.Rate(self.f)
        self.vel_cruc = 0.30
        self.w_max = 1.047197

        self.a=0
        self.ori = 0
        Kp = (5,5,0.1)
        Ki = (0.01,0.01,0)
        e = [0,0,0]
        e_sum = [0,0,0]
        self.newmsg = Twist()

        rospy.loginfo("Inicializo correctamente")
        
        while (not rospy.is_shutdown()):
            for j in range(len(self.coordenadas)-1):
                while(len(self.MTH) == 1):
                    self.update_goal(j+1)
                    self.MTH = self.Calcular_MTH()
                    rate.sleep()

                e_ = 10
                while (e_ >= 0.1):
                    self.update_goal(j+1)
                    self.MTH = self.Calcular_MTH()
                    #rospy.loginfo(self.MTH)
                    
                    if(len(self.MTH) != 1):
                        e[0] = self.MTH[0,3]
                        e[1] = self.MTH[1,3]
                        e[2] = -self.ori+self.coordenadas[j+1][2]

                        e_sum[0] = e_sum[0] + e[0]/self.f
                        e_sum[1] = e_sum[1] + e[1]/self.f
                        e_sum[2] = e_sum[2] + e[2]/self.f
                        e_ = math.sqrt(e[0]**2+e[1]**2+e[2]**2)

                        self.newmsg.linear.x = Kp[0]*e[0]+Kp[1]*e[1]+Ki[0]*e_sum[0]+Ki[1]*e_sum[1]
                        self.newmsg.linear.y = 0
                        self.newmsg.linear.z = 0
                        self.newmsg.angular.x = 0
                        self.newmsg.angular.y = 0
                        self.newmsg.angular.z = Kp[2]*e[2]+Ki[2]*e_sum[2]
                        
                        if self.newmsg.linear.x > self.vel_cruc:
                            self.newmsg.linear.x = self.vel_cruc
                        elif self.newmsg.linear.x < -self.vel_cruc:
                            self.newmsg.linear.x = -self.vel_cruc

                        if self.newmsg.angular.z > self.w_max:
                            self.newmsg.angular.z = self.w_max
                        elif self.newmsg.angular.z < -self.w_max:
                            self.newmsg.angular.z = -self.w_max

                        self.pub1.publish(self.newmsg)
                        
                        rospy.loginfo(e_)
                    
                    
                    rate.sleep()

                rospy.loginfo("CAMBIO DE COORDENADA")
                rate.sleep()

    def update_goal(self, i):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "odom"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "Goal"
        t.transform.translation.x = self.coordenadas[i][0]
        t.transform.translation.y = self.coordenadas[i][1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcts.sendTransform(t)

    def Calcular_MTH(self):
        try:
            trans_base_marker = self.tfBuffer.lookup_transform("base_footprint", "Goal", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error trying to look for transform")
            return [0]
        quat_from_ROS = np.array([trans_base_marker.transform.rotation.x, \
                                    trans_base_marker.transform.rotation.y, \
                                    trans_base_marker.transform.rotation.z, \
                                    trans_base_marker.transform.rotation.w])
        rt_mat_from_ROS = tf_conversions.transformations.quaternion_matrix(quat_from_ROS)
        MTH_GOAL = rt_mat_from_ROS.copy()
        MTH_GOAL[0,3] = trans_base_marker.transform.translation.x
        MTH_GOAL[1,3] = trans_base_marker.transform.translation.y
        MTH_GOAL[2,3] = trans_base_marker.transform.translation.z

        #rospy.loginfo(MTH_GOAL)

        return MTH_GOAL

    def callback(self,data):
        aa = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        self.ori = aa[2]

    coordenadas = [ ( 0.0, 0.0,  0.0),
                    (-3.5, 0.0,  0.0),
                    (-3.5, 0.0,  1.5708),
                    (-3.5, 3.5,  1.5708),
                    (-3.5, 3.5,  0.0),
                    ( 1.5, 3.5,  0.0),
                    ( 1.5, 3.5, -1.5708),
                    ( 1.5,-1.5, -1.5708),
                    ( 1.5,-1.5,  0.0),
                    ( 3.5,-1.5,  0.0),
                    ( 3.5,-1.5, -1.5708),
                    ( 3.5,-8.0, -1.5708),
                    ( 3.5,-8.0, -3.1416),
                    (-2.5,-8.0, -3.1416),
                    (-2.5,-8.0, -4.7124),
                    (-2.5,-5.5, -4.7124),
                    (-2.5,-5.5, -6.2832),
                    ( 1.5,-5.5, -6.2832),
                    ( 1.5,-5.5, -4.7124),
                    ( 1.5,-3.5, -4.7124),
                    ( 1.5,-3.5, -3.1416),
                    (-1.0,-3.5, -3.1416)]