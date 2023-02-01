#!/usr/bin/env python3
import math
import os
import time
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float32


class DriverNode(DTROS):
    def __init__(self, node_name):
        super(DriverNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        
        # Static variables
        self._veh = os.environ["VEHICLE_NAME"]
        self._rate = rospy.Rate(1)
        self._robot_width_half = 0.05

        # Values to keep track on
        self.total_distance = 0.0   # Total distance progressed by robot
        self.average_distance = 0.0     # Average absolute distance progressed by each wheel
        self.left_distance = 0.0    # Total distance progressed by left wheel
        self.right_distance = 0.0   # Total distance progressed by right wheel
        self.is_rot = False     # Whether robot is going to perform rotation or not
        self.vel_changed = False    # Flag to indicate the changes in velocity

        self.prev_msg = None        
        
        # Publisher
        self.pub_vel = rospy.Publisher(
            f"{self._veh}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=10
        )
        
        # Subscribers
        self.delta_dist_left = rospy.Subscriber(
            f"{self._veh}/odometry_node/left_wheel_delta",
            Float32,
            self.param_update,
            callback_args="left"
        )
        self.delta_dist_right = rospy.Subscriber(
            f"{self._veh}/odometry_node/right_wheel_delta",
            Float32,
            self.param_update,
            callback_args="right"
        )
        self.sub_executed_cmd = rospy.Subscriber(
            f"/{self._veh}/wheels_driver_node/wheels_cmd_executed",
            WheelsCmdStamped,
            self.cb_executed_commands
        )
    
    def reset_variables(self):
        self.total_distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0

    def param_update(self, msg, wheel):
        if wheel == "right":
            self.right_distance += msg.data
            self.total_distance = (self.left_distance + self.right_distance) / 2.
            self.average_distance = (self.left_distance - self.right_distance) / 2.
        else:
            self.left_distance += msg.data
    
    def cb_executed_commands(self, msg):
        if self.prev_msg == None:
            self.vel_changed = True
        elif self.prev_msg.vel_left != msg.vel_left and\
            self.prev_msg.vel_right != msg.vel_right:
            self.vel_changed = True
    
    def send_msg(self, msg):
        while not self.vel_changed:
            # Send until it is received
            self.pub_vel.publish(msg)
            self._rate.sleep()
        
        self.prev_msg = msg
        # Switch back to original flag
        self.vel_changed = False

    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = 0.0
        msg.vel_right = 0.0

        self.send_msg(msg)

    def straight(self, distance, vel_left=0.4, vel_right=0.4):
        assert (vel_left > 0.0 and vel_right > 0.0) or (vel_left < 0.0 and vel_right < 0.0)
        
        # Reset the variables
        self.reset_variables()
        
        # Construct message
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        
        # Send a message
        self.send_msg(msg)
        
        # Move until distance reaches `distance`
        while abs(self.total_distance) < abs(distance):
            continue
        
        # Stop after reaching
        self.stop()

    def rotate(self, angle, vel_left=-0.4, vel_right=0.4):
        # Reset the variables
        self.reset_variables()
        
        # Construct message
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = vel_left
        msg.vel_right = vel_right

        # Send message created
        self.send_msg(msg)

        # Move until angle reaches to `angle`
        while True:
            ang = abs(self.average_distance) / self._robot_width_half
            if ang >= angle:
                break
        
        self.stop()


if __name__ == "__main__":
    # Initialize driver node
    driver = DriverNode("driver_node")

    ### Lab 2 Part 1
    # Straight line task
    driver.straight(1.25, 0.6, 0.6)
    time.sleep(5)
    driver.straight(1.25, 0.6, 0.6)
    time.sleep(5)

    # Roatation task
    # driver.rotate(pi/2, 0.4, -0.4)
    
    ### Lab 2 Part 2
    # TODO: ROS service to light the LED
    
    ### Uncomment below for running task for part 2 ###
    # # State 1.
    # time.sleep(5)

    # # State 2.
    # driver.rotate(math.pi/2, 0.4, -0.4)
    # for i in range(3):
    #     driver.straight(1.25)
    #     if i < 2:
    #         driver.rotate(math.pi/2, -0.4, 0.4)

    # # State 1.
    # time.sleep(5)

    # # State 3.
    # driver.rotate(math.pi/2, -0.4, 0.4)
    # driver.straight(1.25, 0.4, 0.4)
    # driver.rotate(math.pi, -0.4, 0.4)

    # # State 1.
    # time.sleep(5)
    
    # # State 4.
    # driver.rotate(math.pi/2, 0.4, -0.4)
    # for i in range(4):
    #     driver.straight(1.25, 0.4, 0.4)
    #     if i < 3:
    #         driver.rotate(math.pi/2, -0.4, 0.4)
    # driver.rotate(math.pi, -0.4, 0.4)