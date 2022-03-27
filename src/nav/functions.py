#!/usr/bin/env python

## Functions and classes used in nav package ##

import math
import numpy as np


# Types of primitives
NO_TURN = 0
TURN_UP = 1
TURN_DOWN = 2

class Dubins_Primitive:

    def __init__(self, init_pose, speed, R, dt, aci):
        
        self.pose = init_pose
        self.speed = speed
        self.R = R
        self.rad = speed*dt/R
        self.dt = dt
        self.active = False

        self.aci = aci

        # Turn centers for position calculation
        if aci == TURN_UP:
            self.center = init_pose[0:3] + np.array([-R*math.sin(init_pose[3]), R*math.cos(init_pose[3]), 0]) # For turning up

        elif aci == TURN_DOWN:
            self.center = init_pose[0:3] + np.array([R*math.sin(init_pose[3]), -R*math.cos(init_pose[3]), 0]) # For turning down

        self.expired = False

    # Check if the time range for the primitive has expired
    def check_time(self, time):

        if self.active == False:
            self.time = time

        if (time - self.time) >= self.dt:
            self.expired = True

    # Get current position target
    def get_pos(self, time):

        local_time = time - self.time

        if self.aci == NO_TURN:
            pos = self.pose[0:3] + self.speed*local_time*np.array([math.cos(self.pose[3]), math.sin(self.pose[3]), 0])
            return pos

        elif self.aci == TURN_UP:
            angle = (local_time*self.rad)/self.dt + self.pose[3]
            pos = self.center + self.R*np.array([math.sin(angle), -math.cos(angle), 0])
            return pos

        elif self.aci == TURN_DOWN:
            angle = (local_time*self.rad)/self.dt + self.pose[3]
            pos = self.center + self.R*np.array([math.sin(-angle), math.cos(-angle), 0])
            return pos

        else:
            return self.pose[0:3]

    # Get current velocity target
    def get_vel(self, time):

        local_time = time - self.time

        if self.aci == NO_TURN:
            vel = self.speed*np.array([math.cos(self.pose[3]), math.sin(self.pose[3]), 0])
            return vel

        elif self.aci == TURN_UP:
            angle = (local_time*self.rad)/self.dt + self.pose[3]
            vel = self.speed*np.array([math.cos(angle), math.sin(angle), 0]) # v * [pose direction]
            return vel

        elif self.aci == TURN_DOWN:
            angle = -(local_time*self.rad)/self.dt + self.pose[3]
            vel = self.speed*np.array([math.cos(angle), math.sin(angle), 0]) # v * [pose direction]
            return vel

        else:
            return np.array([0, 0, 0])

    # Get current acceleration target
    def get_acc(self, time):

        local_time = time - self.time

        if self.aci == NO_TURN:
            acc = np.array([0, 0, 0])
            return acc

        elif self.aci == TURN_UP:
            angle = (local_time*self.rad)/self.dt + self.pose[3] + math.pi/2
            vel = (self.speed**2)/(self.R)*np.array([math.cos(angle), math.sin(angle), 0]) # v^2/R * [pose_direction(ang + 90)]
            return vel

        elif self.aci == TURN_DOWN:
            angle = (local_time*self.rad)/self.dt + self.pose[3] - math.pi/2
            vel = (self.speed**2)/(self.R)*np.array([math.sin(-angle), math.cos(-angle), 0]) # v^2/R * [pose_direction(ang - 90)]
            return vel

        else:
            return np.array([0, 0, 0])


if __name__ == "__main__":
    init_pose = np.array([12.5, 17, 2, math.pi/4])
    init_time = 124987
    speed = 3
    R = 8
    dt = 1
    prim_NO_TURN = Dubins_Primitive(init_pose, speed, R, dt, aci=0)
    prim_TURN_UP = Dubins_Primitive(init_pose, speed, R, dt, aci=1)
    prim_TURN_DOWN = Dubins_Primitive(init_pose, speed, R, dt, aci=2)

    prim_NO_TURN.check_time(init_time)
    prim_TURN_UP.check_time(init_time)
    prim_TURN_DOWN.check_time(init_time)

    DT = np.linspace(0, 1, 15) + init_time
    print("No Turn")
    for t in DT:
        print(prim_NO_TURN.get_acc(t))

    print("Turn Up")
    for t in DT:
        print(prim_TURN_UP.get_acc(t))

    print("Turn Down")
    for t in DT:
        print(prim_TURN_DOWN.get_acc(t))