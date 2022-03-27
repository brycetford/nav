#!/usr/bin/env python

from this import d
import rospy
import mavros
import math

import numpy as np

from tf.transformations import quaternion_from_euler
from nav.functions import Dubins_Primitive

from controller_msgs.msg import FlatTarget
from nav.msg import Dubins
import mavros.setpoint as SP

# Command loop actions
IDLE = 0
FOLLOW_TRAJECTORY = 1
GO_TO = 2


class Trajectory_Publisher:

    def __init__(self):
        
        self.name = "Trajectory_Publisher"
        self.command = IDLE
        self.primitive_queue = []

        rospy.init_node("trajectory_publisher", anonymous=True)
        mavros.set_namespace(ns=self.name)

        # Subscribers
        self.LocalPosSub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.local_pos_cb)
        self.DubinsSub = rospy.Subscriber('reference/dubins', Dubins, self.dubins_cb)
        
        # Publishers
        self.FlatTargetPub = rospy.Publisher('reference/flatsetpoint', FlatTarget, queue_size=10)
        self.RawTargetPub = SP.get_pub_position_local(queue_size=10)

        # Event Loop Timers
        self.CommandLoop = rospy.Timer(rospy.Duration(nsecs=100000), self.command_loop_cb)

    # Callback for current position
    def local_pos_cb(self, topic):

        self.current_pos = np.array([topic.pose.position.x, 
            topic.pose.position.y, 
            topic.pose.position.z])

    # Callback for dubins primitive
    def dubins_cb(self, topic):

        init_pose = np.array([topic.init_pos.x,
            topic.init_pos.y,
            topic.init_pos.z,
            topic.yaw])

        speed = topic.speed
        R = topic.R
        dt = topic.dt
        aci = topic.aci

        dubins = Dubins_Primitive(init_pose, speed, R, dt, aci)
        self.primitive_queue.append(dubins)

    # Decide what to do
    def command_loop_cb(self):

        if self.command == IDLE:
            return
            
        elif self.command == FOLLOW_TRAJECTORY:
            self.publish_flat_traj()

        elif self.command == GO_TO:
            self.publish_setpoint(self.target)

    # Publish a 'raw' xyz + yaw setpoint to position controller
    def publish_setpoint(self, setpoint):

        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",
                stamp=rospy.Time.now())
            )

        msg.pose.position.x = setpoint[0]
        msg.pose.position.y = setpoint[1]
        msg.pose.position.z = setpoint[2]

        quaternion = quaternion_from_euler(0, 0, setpoint[3])
        msg.pose.orientation = SP.Quaternion(*quaternion)

        self.RawTargetPub.publish(msg)

    # Publish a flat trajectory setpoint for geometric controller
    def publish_flat_traj(self):

        time = rospy.Time.now()

        primitive = self.primitive_queue[0]
        primitive.check_time(time)

        if primitive.expired == True:
            self.primitive_queue.pop(0)
            return

        msg = FlatTarget
        msg.header.stamp = time
        msg.type_mask = 2

        pos = primitive.get_pos(time)
        vel = primitive.get_vel(time)
        acc = primitive.get_acc(time)

        msg.position.x = pos[0]
        msg.position.y = pos[1]
        msg.position.z = pos[2]

        msg.velocity.x = vel[0]
        msg.velocity.y = vel[1]
        msg.velocity.z = vel[3]

        msg.acceleration.x = acc[0]
        msg.acceleration.y = acc[1]
        msg.acceleration.z = acc[2]

        self.FlatTargetPub.publish(msg)

    
if __name__ == "__main__":
    prajectory_publisher = Trajectory_Publisher()
