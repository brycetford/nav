#!/usr/bin/env python

from threading import Thread
import time
from math import *
import numpy as np

import rospy
from tf.transformations import quaternion_from_euler

import mavros
import mavros.setpoint as SP
from mavros.utils import *

class OffboardController:
    # Sends position targets to PX4 position controller

    def __init__(self):
        # Initial waypoint
        self.wp = WP(0, 0, 0, 0)
        self.offset = 0

        # publisher for mavros/setpoint_position/local
        self.pub = SP.get_pub_position_local(queue_size=10)

        # subscribe to mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.reached)

        # Create a daemon to send a stream of setpoints
        self.pos_thread = Thread(target=self.navigate, args=())
        self.pos_thread.daemon = True
        
        self.done = False
        self.done_evt = threading.Event()

        self.pos_thread.start()


    def navigate(self):
        # Publish a stream of the current setpoint
        rate = rospy.Rate(10)   # 10 Hz

        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",
                stamp=rospy.Time.now())
            )

        while not rospy.is_shutdown():
            pos = self.wp.get_offset(self.offset)
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = pos[2]

            yaw = radians(self.wp.get_yaw())
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = SP.Quaternion(*quaternion)

            self.pub.publish(msg)
            rate.sleep()


    def set(self, x, y, z, yaw, offset=0, delay=0, wait=True):
        # Call to update the current setpoint 

        self.done = False
        self.wp = WP(x, y, z, yaw)
        self.offset = offset

        if wait:
            rate = rospy.Rate(10)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()

        time.sleep(delay)


    def reached(self, topic):
        # Condition for reaching the target setpoint

        current = np.array([topic.pose.position.x, 
            topic.pose.position.y, 
            topic.pose.position.z])
        target = self.wp.get_pos()

        offset = np.linalg.norm(target - current)
        eps = 1     # Margin of error for reaching the setpoint
        rospy.loginfo(offset)
        if offset < eps:
            self.done = True
            self.done_evt.set()

class WP:

    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def get_pos(self):
        return np.array([self.x, self.y, self.z])

    def get_offset(self, delta):
        offset = delta * np.array([cos(self.yaw), sin(self.yaw), 0])
        return self.get_pos() + offset
    
    def get_yaw(self):
        return self.yaw

# Testing the position control
def position_ctl():
    rospy.init_node('setpoint_ctl')
    mavros.set_namespace()
    rate = rospy.Rate(10)

    setpoint = OffboardController()

    rospy.loginfo("Climbing")
    setpoint.set(0.0, 0.0, 3.0, 0, 0)
    setpoint.set(0.0, 0.0, 10.0, 0, 5)

if __name__ == "__main__":
    try:
        position_ctl()
    except rospy.ROSInterruptException:
        pass